"""
explorer.py — หัวใจของ SLAM แบบกริด

วงจรที่ทุกช่อง:
  1) SENSE   : หมุน gimbal สแกน ToF 4 ทิศ
  2) MAP     : อัปเดต log-odds ของขอบช่อง (กำแพง/โล่ง)  -> Mapping
  3) LOCALIZE: ตรวจลายกำแพงเทียบแผนที่ + วัดค่าเบี่ยงจากกลางช่องด้วยระยะกำแพง
               แล้ว Recenter  -> Localization (ลด error สะสมจาก odometry)
  4) PLAN    : BFS หาช่องที่ยังไม่สำรวจที่ใกล้ที่สุด (frontier) + A* หาเส้นทาง
  5) ACT     : เดินทีละช่อง (ตัวถังไม่หมุน ใช้ล้อ mecanum) + แก้มุม yaw ด้วย IMU
  +) SHOOT   : (ถ้ามี hunter) ระหว่าง SENSE กล้องหาป้ายทุกทิศ -> หลังสแกนช่อง ยิงป้ายที่อยู่ในระยะ (shooter.py)
"""
import csv
import os
import time

import config as C
from grid_map import DIRS, DX, DY, WALL, OPEN, UNKNOWN


class RunLogger:
    FIELDS = ["step", "time_s", "event", "cell_x", "cell_y", "heading",
              "est_x_m", "est_y_m", "odom_x_m", "odom_y_m", "yaw_deg",
              "tof_N", "tof_E", "tof_S", "tof_W", "sharp_N", "sharp_E", "sharp_S", "sharp_W",
              "walls_NESW", "note"]

    def __init__(self, out_dir, echo=print):
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.path = os.path.join(out_dir, "exploration_log.csv")
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=self.FIELDS)
        self.w.writeheader()
        self.t0 = time.time()
        self.step = 0
        self.traj = []        # (t, est_x, est_y, odom_x, odom_y, cell_x, cell_y, event)
        self.echo = echo

    def log(self, robot, event, tof=None, walls=None, note="", sharp=None):
        ex, ey = robot.est_xy()
        od = robot.odom_xy()
        yw = robot.yaw()
        t = round(time.time() - self.t0, 2)
        row = {"step": self.step, "time_s": t, "event": event,
               "cell_x": robot.cell[0], "cell_y": robot.cell[1],
               "heading": DIRS[robot.heading],
               "est_x_m": round(ex, 3), "est_y_m": round(ey, 3),
               "odom_x_m": round(od[0], 3) if od else "",
               "odom_y_m": round(od[1], 3) if od else "",
               "yaw_deg": round(yw, 1) if yw is not None else "",
               "walls_NESW": walls or "", "note": note}
        if tof:
            for d, v in tof.items():
                row["tof_" + DIRS[d]] = round(v)
        for d, v in (sharp or {}).items():
            if v is not None:
                row["sharp_" + DIRS[d]] = round(v[0])
        self.w.writerow(row)
        self.f.flush()
        self.traj.append((t, ex, ey, od[0] if od else None, od[1] if od else None,
                          robot.cell[0], robot.cell[1], event))
        self.step += 1
        hb = getattr(self, "heartbeat", None)
        if hb:
            hb()
        if note:
            self.echo(f"[{event}] {note}")

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


class Explorer:
    def __init__(self, robot, gmap, logger, on_update=None, stop_event=None, echo=print, hunter=None):
        self.r = robot
        self.m = gmap
        self.L = logger
        self.on_update = on_update or (lambda: None)
        self.stop = stop_event
        self.echo = echo
        self.last_tof = {}
        self.plan = []            # เส้นทางที่วางแผนไว้ (list ของ cell) ไว้ให้ GUI วาด
        self.warnings = 0
        self.last_block = None     # (cell, d, conflict) ของการ blocked ล่าสุดใน step()
        self.move_fails = {}       # (cell, d) -> จำนวนครั้งที่เดินไปทางนี้ไม่สำเร็จ
        self.hunter = hunter
        if hunter is not None:
            hunter.attach(self)
        elif hasattr(robot, "on_look"):
            robot.on_look = None

    def stopped(self):
        if getattr(self.r, "halt", None):
            return True             # หุ่นสั่งหยุดเอง (ตัวถังบิดระหว่างเดิน) - ห้ามเดินต่อ
        return self.stop is not None and self.stop.is_set()

    # ------------------------------------------------------------ SENSE+MAP+LOCALIZE
    def sense(self, update_map=True, event="scan"):
        x, y = self.r.cell
        self.r.correct_yaw()        # สแกนตอนตัวถังเบี้ยว = ToF/Sharp ชี้เฉียง อ่านผิดทั้งชุด
        tof = self.r.scan_all()
        self.last_tof = tof
        obs = {}
        for d, mm in tof.items():
            obs[d] = WALL if mm < C.WALL_THRESHOLD_MM else OPEN
        # อ่านซ้ำเมื่อค่าก้ำกึ่ง (ใกล้ threshold) หรือขัดกับแผนที่เดิม -> ลด error จาก noise
        th = C.WALL_THRESHOLD_MM
        for d in range(4):
            reads = [tof[d]]
            prev = self.m.state(x, y, d)
            bnd = self.m.is_boundary(x, y, d)
            while len(reads) < 5:
                walls_n = sum(r < th for r in reads)
                maj = WALL if walls_n * 2 > len(reads) else OPEN
                tie = walls_n * 2 == len(reads)
                near_th = any(abs(r - th) < 80 for r in reads) and len(reads) < 3
                conflict = (prev != UNKNOWN and prev != maj and not bnd and len(reads) < 3)
                if not (tie or near_th or conflict):
                    break
                reads.append(self.r.read_dir(d))
            walls_n = sum(r < th for r in reads)
            obs[d] = WALL if walls_n * 2 > len(reads) else OPEN
            same = sorted(r for r in reads if (r < th) == (obs[d] == WALL))
            tof[d] = same[len(same) // 2]

        # Sharp ด้านข้างตัวถัง = "ความเห็นที่สอง" ของทิศนั้น (ไม่ใช่ตัวชี้ขาด)
        # ตรงกัน -> มั่นใจ, ขัดกัน -> อ่านทั้งคู่ซ้ำ ถ้ายังขัด = UNKNOWN (ไม่ใส่หลักฐานลงแผนที่)
        # เดิมให้ Sharp ชนะเสมอ: log 27/9 20:47 Sharp อ่าน "กำแพง" ทั้งที่ ToF เห็นโล่ง 1.3-1.9m
        # แล้วเขียนทับขอบที่เคยถูกไปแล้ว
        notes = []
        # อ่าน Sharp ซ้าย/ขวาทุกครั้งที่สแกน (บันทึกลง log คอลัมน์ sharp_* ไว้ตรวจเซนเซอร์ย้อนหลัง)
        sharp = {d: self.r.read_sharp(d) for d in range(4)}
        for d in range(4):
            if self.m.is_boundary(x, y, d):
                continue                    # ขอบสนาม: รู้อยู่แล้วว่าเป็นกำแพง (ดูด้านล่าง)
            sh = sharp.get(d)
            if sh is None:
                continue
            s_wall = sh[0] < C.SHARP_WALL_MM
            if s_wall == (obs[d] == WALL):
                continue
            t2 = self.r.read_dir(d)
            sh2 = self.r.read_sharp(d) or sh
            t_wall2, s_wall2 = t2 < th, sh2[0] < C.SHARP_WALL_MM
            open_mm = th + 0.25 * self.m.cell_m * 1000
            if obs[d] == OPEN and min(tof[d], t2) >= open_mm:
                # ToF โล่งชัดเจนทั้ง 2 ครั้ง (ไกลกว่ากำแพงขอบช่องมาก) -> เชื่อ ToF ไม่ต้อง UNKNOWN
                # recheck ก็ตัดสินด้วย ToF อยู่แล้ว (log ทุกรอบ: Sharp ~200-297 ตอน ToF 800-3000
                # แล้ว recheck = OPEN ทุกครั้ง) - เดิมทำให้หุ่นอ้อมไกล/ไปตรวจซ้ำเสียเวลา
                continue
            if t_wall2 == s_wall2:
                obs[d] = WALL if t_wall2 else OPEN
                tof[d] = t2
                continue
            notes.append(f"{DIRS[d]}: ToF {tof[d]:.0f}/{t2:.0f}mm vs Sharp {sh[0]:.0f}/{sh2[0]:.0f}mm "
                         f"ขัดกัน -> UNKNOWN")
            obs[d] = UNKNOWN
            tof[d] = th + 1                 # ไม่ใช้ทิศนี้ใน recenter

        # ขอบสนาม = กำแพงแน่นอน (prior) ถ้าเซนเซอร์บอกโล่ง = เซนเซอร์ชี้ผิดทิศ/ตัวถังเบี้ยว
        for d in range(4):
            if self.m.is_boundary(x, y, d):
                if obs[d] != WALL:
                    notes.append(f"!! {DIRS[d]} เป็นขอบสนามแต่อ่านโล่ง {tof[d]:.0f}mm "
                                 f"(ตัวถังเบี้ยว/gimbal ชี้ผิดทิศ?)")
                    tof[d] = th + 1         # ค่าที่อ่านได้เชื่อไม่ได้ ไม่ใช้ใน recenter
                obs[d] = WALL

        # ตรวจตำแหน่ง: ลายกำแพงที่เห็น ตรงกับแผนที่ที่มีอยู่หรือไม่
        mism = 0
        if self.m.visited[x][y]:
            for d in range(4):
                s = self.m.state(x, y, d)
                if (s != UNKNOWN and obs[d] != UNKNOWN and abs(self.m.get_lo(x, y, d)) >= 2
                        and s != obs[d]):
                    mism += 1
        sharp_notes = notes
        note = "; ".join(sharp_notes)
        if mism >= 2:
            self.warnings += 1
            cands = self.m.match_candidates(tuple(obs[d] for d in range(4)))
            best = [c for m_, c in cands if m_ == 0][:5]
            note = (f"LOCALIZATION WARNING: {mism} walls differ from map. matching cells={best}"
                    + (f" | {note}" if note else ""))
            # กู้ตำแหน่ง: ถ้ามีช่องข้างเคียงเพียงช่องเดียวที่ลายกำแพงตรง -> เชื่อว่าอยู่ช่องนั้น
            near = [(x + DX[d], y + DY[d]) for d in range(4)] + [(x, y)]
            zero = [c for m_, c in cands if m_ == 0 and self.m.visited[c[0]][c[1]]]
            near_ok = [c for c in zero if c in near]
            if len(near_ok) == 1 and near_ok[0] != (x, y):
                self.r.cell = near_ok[0]
                x, y = near_ok[0]
                note += f" -> RELOCALIZED to {near_ok[0]}"

        if update_map:
            for d in range(4):
                if obs[d] != UNKNOWN:
                    self.m.update(x, y, d, obs[d] == WALL)
            self.m.visited[x][y] = True

        walls = "".join({WALL: "W", OPEN: ".", UNKNOWN: "?"}[obs[d]] for d in range(4))
        self.L.log(self.r, event, tof, walls, note, sharp=sharp)

        if C.RECENTER:
            self.recenter(tof)
        self.on_update()
        return obs

    def _odo_k(self, axis):
        """scale odometry ของแกนโลก axis = median ของที่เรียนได้ล่าสุด (ยังไม่มี = 1.0)"""
        if not hasattr(self.r, "odo_k"):
            self.r.odo_k = [[], []]
        ks = self.r.odo_k[axis]
        # ยังไม่ได้เรียน -> ค่าตั้งต้นต่อแกน (strafe E-W odometry อ่านเกินมากกว่าเดินหน้า N-S)
        return sorted(ks)[len(ks) // 2] if ks else C.ODOM_K_DEFAULT[axis]

    def recenter(self, tof):
        """ใช้ระยะถึงกำแพงประมาณค่าเบี่ยงจากกลางช่อง แล้วขยับกลับ"""
        E = C.WALL_EXPECT_MM
        th = C.WALL_THRESHOLD_MM
        def wall(d):
            return tof.get(d, 1e9) < th
        tol = 0.25 * self.m.cell_m * 1000      # ค่าที่ห่างจาก E เกินนี้ ไม่ใช้ (กันค่าเพี้ยน)
        tof = dict(tof)

        # odometry ทำนายว่าเยื้องจากกลางช่องเท่าไร (เทียบจุดยึดล่าสุดที่ ToF วัดได้) ; None = ไม่รู้
        x, y = self.r.cell
        centre = ((x + 0.5) * self.m.cell_m, (y + 0.5) * self.m.cell_m)
        odo = self.r.odom_world() if C.ODOM_HOLD else None
        ref = getattr(self.r, "odo_ref", None)
        if ref is None:
            ref = self.r.odo_ref = [None, None]
        pred = [None, None]     # corr (mm) ที่ odometry บอกว่าควรขยับ
        if odo is not None:
            for i in range(2):
                if ref[i] is not None:
                    o0, t0 = ref[i][0], ref[i][1]
                    off = (t0 + (odo[i] - o0) / self._odo_k(i) - centre[i]) * 1000
                    if abs(off) > 0.4 * self.m.cell_m * 1000:
                        ref[i] = None           # เยื้องเกินครึ่งช่อง = จุดยึดผิด (เช่น relocalize) ทิ้ง
                    else:
                        pred[i] = -off
        cur_axis = [None]

        def clearly_open(d):
            v = tof.get(d, 1e9)
            return v == th + 1 or v >= th + 0.25 * self.m.cell_m * 1000   # th+1 = ToF/Sharp ขัดกัน (sense)

        cell_mm = self.m.cell_m * 1000

        def far_ref(d):
            """ช่องนี้โล่งทิศ d แต่ช่องถัดไปมีกำแพงไกล (แผนที่รู้แล้ว) -> ใช้กำแพงนั้นอ้างอิง (ห่าง EXPECT + 1 ช่อง)
            (log 30/9: (2,4)/(5,4) ไม่มีกำแพง N/S ในช่องตัวเอง เลยไม่เคยแก้แนวนั้น -> เยื้องใต้ 13-25 ซม. ชนมุมกำแพง)"""
            v = tof.get(d)
            nx, ny = x + DX[d], y + DY[d]
            if v is None or wall(d) or not self.m.in_bounds(nx, ny):
                return None
            if self.m.state(x, y, d) != OPEN or self.m.state(nx, ny, d) != WALL:
                return None
            ref = E + cell_mm
            return ref if abs(v - ref) < C.FAR_WALL_TOL_MM else None

        def corr_of(items, t, pos):
            """ค่าแก้ (mm, + = ไปทาง pos) จากรายการ (ทิศ, ระยะที่ควรอ่านได้ถ้าอยู่กลางช่อง)"""
            vals = [(t[d] - r) if d == pos else -(t[d] - r) for d, r in items]
            return sum(vals) / len(vals)

        def axis_corr(pos, neg):
            p = pred[cur_axis[0]]

            def plausible(d):
                if not wall(d):
                    return False
                if abs(tof[d] - E) < tol:
                    return True
                if tof[d] <= C.TOF_BLIND_ZONE_MM + 5:
                    # ชิดจน ToF อ่านได้แค่ค่าต่ำสุด = ใกล้กว่าที่ควรอย่างน้อย E-60 แน่ๆ -> ต้องถอยออก
                    # (log 28/9 01:47: (2,2) N=60 เกิน tol พอดีเลยไม่ถอย หุ่นจอดชิดกำแพงทั้งช่อง)
                    return True
                # เยื้องเยอะเกิน tol แต่ odometry บอกตรงกัน = เยื้องจริง ไม่ใช่ค่าเพี้ยน
                # (log 28/9: (4,0) S=429mm = เยื้อง 245mm ถูกทิ้งเพราะเกิน tol เลยไม่เคยแก้)
                c1 = tof[d] - E if d == pos else -(tof[d] - E)
                return (p is not None and abs(tof[d] - E) < 0.45 * self.m.cell_m * 1000
                        and abs(c1 - p) < C.ODOM_AGREE_MM)
            dp, dn = tof.get(pos), tof.get(neg)
            if plausible(pos) and plausible(neg) and abs(dp + dn - 2 * E) < 120:
                return (dp - dn) / 2.0, ((pos, E), (neg, E))
            if plausible(pos) and (not plausible(neg) or abs(dp - E) <= abs(dn - E)):
                return dp - E, ((pos, E),)
            if plausible(neg):
                return -(dn - E), ((neg, E),)
            # ไม่มีกำแพงในช่องตัวเองแนวนี้ -> ลองกำแพงไกลของช่องถัดไป
            items = tuple((d, r) for d, r in ((pos, far_ref(pos)), (neg, far_ref(neg))) if r is not None)
            if items:
                return corr_of(items, tof, pos), items
            return 0.0, ()

        corr = [0.0, 0.0]    # mm, world x/y  (ทิศที่ต้องขยับ)
        hold = []
        used_axes = [(), ()]  # ทิศ/ระยะอ้างอิงที่ใช้วัดแต่ละแกน (ใช้วัดซ้ำหลังขยับ)
        for pos, neg, axis in ((0, 2, 1), (1, 3, 0)):     # (N,S)->y  (E,W)->x
            cur_axis[0] = axis
            c, used = axis_corr(pos, neg)
            if abs(c) > 60 and used:
                # แก้เยอะ -> อ่านซ้ำทิศที่ใช้ แล้วใช้ค่ากลาง (กันค่า ToF หลุดทำให้หุ่นขยับผิด)
                hist = {d: [tof[d]] for d, _ in used}
                for d, _ in used:
                    hist[d] += [self.r.read_dir(d), self.r.read_dir(d)]
                    tof[d] = sorted(hist[d])[1]
                c, used = axis_corr(pos, neg)
            used_axes[axis] = used
            if used:
                if odo is not None:     # ToF วัดแกนนี้ได้ -> ตั้งจุดยึด odometry ใหม่
                    true = centre[axis] - c / 1000.0
                    # near = วัดจากกำแพงของช่องตัวเอง (~195mm แม่น) ; far_ref (ToF ~800mm) คลาดได้ ~100mm
                    near = all(r == E for _, r in used)
                    if (near and ref[axis] is not None and ref[axis][2]
                            and abs(true - ref[axis][1]) >= 0.5):
                        # เรียน scale odometry ของสนามนี้: ระยะ odometry / ระยะจริงตาม ToF - เฉพาะจุดยึดกำแพงใกล้ทั้งคู่
                        # (log 30/9 20:31: เรียนจาก far_ref ได้ k=0.83 ทั้งที่จริง ~1.10 -> เดินขาดช่องละ 140mm
                        #  3 ช่องหายไป 1 ช่อง ตำแหน่งผิดทั้งแผนที่)
                        k = (odo[axis] - ref[axis][0]) / (true - ref[axis][1])
                        ks = self.r.odo_k[axis]
                        ks.append(max(C.ODOM_K_MIN, min(C.ODOM_K_MAX, k)))
                        del ks[:-7]
                    ref[axis] = (odo[axis], true, near)
            elif (pred[axis] is not None and abs(pred[axis]) >= C.ODOM_HOLD_MIN_MM
                  and clearly_open(pos) and clearly_open(neg)):
                # ใช้ odometry เฉพาะแกนที่ "ไม่มีกำแพงจริง ๆ" ทั้งสองฝั่ง - ถ้ามีกำแพงแต่ค่าเยื้อง
                # ขัดกับ odometry ห้ามเดา (log 28/9 00:43: (2,1) N=388 บอกหุ่นอยู่ใต้ แต่ odometry
                # บอกเหนือ -> ดันถอยลงใต้อีก จนกำแพง N อ่านเกิน threshold แล้ว recheck ลบกำแพงทิ้ง)
                # ไม่มีกำแพงให้วัด -> ใช้ odometry แบบระวัง (gain + เพดาน)
                c = max(-C.ODOM_HOLD_MAX_MM, min(C.ODOM_HOLD_MAX_MM, C.ODOM_HOLD_GAIN * pred[axis]))
                hold.append(f"{'xy'[axis]}={-pred[axis]:+.0f}(แก้ {c:+.0f})")
            corr[axis] = c
        # ค่าเบี่ยงที่วัดได้ (ก่อนแก้) = -corr
        self.r.off = [-corr[0] / 1000.0, -corr[1] / 1000.0]
        note = "" if max(abs(corr[0]), abs(corr[1])) < 60 else (
            f"offset from centre dx={-corr[0]:.0f}mm dy={-corr[1]:.0f}mm")
        if hold:
            note = (note + " | " if note else "") + f"odom hold {' '.join(hold)}mm"
        self.L.log(self.r, "measured_pose", note=note)
        mv = [0.0, 0.0]
        for i in range(2):
            if abs(corr[i]) >= C.RECENTER_MIN_MM:
                mv[i] = max(-C.RECENTER_MAX_MM, min(C.RECENTER_MAX_MM, corr[i])) / 1000.0
        if mv != [0.0, 0.0]:
            self.r.shift(mv[0], mv[1])
            self.r.off = [self.r.off[0] + mv[0], self.r.off[1] + mv[1]]
            self.L.log(self.r, "recenter")
        # วัดซ้ำหลังขยับ (closed loop): ขยับสั้นๆ ของล้อ mecanum มักไปไม่ถึงที่สั่ง -> แก้ต่อจนเหลือ < RECENTER_MIN
        for it in range(C.RECENTER_ITERS):
            if mv == [0.0, 0.0] or self.stopped():
                break
            nxt, parts = [0.0, 0.0], []
            for axis, pos in ((1, 0), (0, 1)):
                if mv[axis] == 0.0 or not used_axes[axis]:
                    continue
                t2 = {d: self.r.read_dir(d) for d, _ in used_axes[axis]}
                r = corr_of(used_axes[axis], t2, pos)
                # เหลือเกิน MAX เล็กน้อย = ขยับรอบก่อนไม่ถึง (log 30/9 19:26: เหลือ 155mm ถูกข้าม -> เดินต่อทั้งที่เยื้อง)
                # -> แก้เต็ม MAX ; เกิน 1.6 เท่า = ค่าอ่านน่าสงสัย ข้าม
                if C.RECENTER_MIN_MM <= abs(r) <= 1.6 * C.RECENTER_MAX_MM:
                    nxt[axis] = max(-C.RECENTER_MAX_MM, min(C.RECENTER_MAX_MM, r)) / 1000.0
                    parts.append(f"{'xy'[axis]} เหลือ {-r:+.0f}mm")
            mv = nxt
            if mv == [0.0, 0.0]:
                break
            self.r.shift(mv[0], mv[1])
            self.r.off = [self.r.off[0] + mv[0], self.r.off[1] + mv[1]]
            self.L.log(self.r, "recenter", note=f"วัดซ้ำรอบ {it + 1}: {' '.join(parts)} -> ขยับต่อ")

    # ------------------------------------------------------------ ACT
    def step(self, d):
        """เดิน 1 ช่อง ไปทิศ d ; คืน False ถ้าไปไม่ได้ (เจอกำแพง)"""
        x, y = self.r.cell
        th = C.WALL_THRESHOLD_MM
        # 1) ยืนยันก่อนเดิน: อ่าน ToF ทิศที่จะไปใหม่ 2 ครั้ง (ขัดกันอ่านครั้งที่ 3) แล้วโหวตเสียงข้างมาก
        # ห้ามใช้ค่าจากการสแกน (last_tof): อ่านก่อน recenter ขยับหุ่น -> ค่าเก่า ระยะหน้าผิดเท่าที่ขยับ
        reads = [self.r.read_dir(d), self.r.read_dir(d)]
        if (reads[0] < th) != (reads[1] < th) or abs(reads[0] - reads[1]) > 60:
            reads.append(self.r.read_dir(d))
        walls_n = sum(r < th for r in reads)
        tof_wall = walls_n * 2 >= len(reads) and walls_n > 0
        sh = self.r.read_sharp(d)           # เดินข้าง: Sharp ฝั่งนั้นช่วยยืนยัน
        sharp_wall = sh is not None and sh[0] < C.SHARP_WALL_MM
        open_mm = th + 0.25 * self.m.cell_m * 1000
        if sharp_wall and not tof_wall and (self.m.get_lo(x, y, d) <= -2.0
                                            or min(reads) >= open_mm):
            # ขอบนี้ recheck/สแกนหลายรอบยืนยันแล้วว่าโล่ง -> Sharp ค้านตัวเดียวไม่พอให้หยุด
            # (ไม่งั้นวนซ้ำ: recheck บอกโล่ง -> step ถูก Sharp ห้าม -> UNKNOWN -> recheck ใหม่)
            # ถ้าผิดจริง เช็ค progress หลังเดินจะจับได้และถอยกลับ
            sharp_wall = False
        if (tof_wall or sharp_wall) and self.m.is_passed(x, y, d):
            if self.move_fails.get((self.r.cell, d), 0) >= 2:
                # เชื่อแผนที่แล้วยังเดินผ่านไม่ได้ 2 ครั้ง -> เลิกถือว่า "เคยผ่าน" ให้ logic ปกติตัดสิน (กันวนไม่จบ)
                self.m.passed.discard(self.m._edge_ref(x, y, d))
            else:
                self.L.log(self.r, "passed_edge", {d: min(reads)},
                           note=f"ขอบ {DIRS[d]} เคยเดินผ่านมาแล้ว แต่ ToF {min(reads):.0f}mm/Sharp "
                                f"{'-' if sh is None else f'{sh[0]:.0f}'}mm บอกกำแพง (หุ่นเยื้อง?) -> เชื่อแผนที่ "
                                f"เดินเต็มช่อง (มีตัวหยุดฉุกเฉิน ToF/Sharp ระหว่างเดิน)")
                tof_wall = sharp_wall = False
                # วางแผนระยะแบบทางโล่ง (ไม่งั้น _plan_move จำกัดระยะตามค่าที่อ่านผิด เดินไม่ถึงแล้ววนซ้ำ)
                reads = [max(v, th + self.m.cell_m * 1000) for v in reads]
        if tof_wall or sharp_wall:
            if tof_wall and (sh is None or sharp_wall):
                self.m.update(x, y, d, True)
                self.m.update(x, y, d, True)
                why = f"ToF {min(reads):.0f}mm"
            else:
                # เซนเซอร์ขัดกัน -> ไม่เสี่ยงเดิน แต่ก็ไม่ฟันธงว่ากำแพง (ให้ recheck ตัดสินทีหลัง)
                self.m.set_lo(x, y, d, 0.0)
                why = (f"ToF {min(reads):.0f}mm={'W' if tof_wall else '.'} vs "
                       f"Sharp {sh[0]:.0f}mm={'W' if sharp_wall else '.'} ขัดกัน -> UNKNOWN")
            self.last_block = (self.r.cell, d, not (tof_wall and (sh is None or sharp_wall)))
            self.L.log(self.r, "blocked", {d: min(reads)},
                       note=f"ไม่เดินไป {DIRS[d]}: {why} -> replan")
            self.on_update()
            return False
        prev = self.r.cell
        # ระยะไปทิศ d ก่อนเดิน (ค่ากลาง) -> ระยะที่จะสั่งเดิน (ห้ามเกินที่กำแพงข้างหน้าเหลือให้)
        before_mm = sorted(reads)[len(reads) // 2]
        dist_mm, before_mm, plan_note = self._plan_move(d, before_mm)
        # ช่วงแรกเร็ว หยุดก่อนเป้า MOVE_FAST_MARGIN_MM (ไถลต่อได้ไม่ชน) -> ช่วงสุดท้ายช้า (_finish_move)
        closed = C.MOVE_CLOSED_LOOP and C.MOVE_MODE != "rotate" and hasattr(self.r, "drive_cell")
        if closed:
            # เดินทั้งระยะด้วยการคุมตลอดทาง (ชะลอ/หยุดเองที่กลางช่อง) ; เดิน E/W เลือกกำแพงข้างให้ ToF ใช้คุมกลาง
            fast_mm = dist_mm
            walls, front_wall = self._drive_refs(d, before_mm)
            self.r.drive_cell(d, dist_mm / 1000.0, self._lat_wall(d), self._odo_k(0 if d in (1, 3) else 1),
                              side_walls=walls, front_wall=front_wall)
        else:
            fast_mm = max(0.25 * dist_mm, dist_mm - C.MOVE_FAST_MARGIN_MM)
            self.r.move_cell(d, fast_mm / 1000.0)
        after = self._front_after_move(d, before_mm)
        self._check_yaw(d)
        # ToF ก่อน-หลัง ใช้ตรวจ "เดินไปจริงไหม" ได้เฉพาะเมื่อค่าก่อนเดินไม่ไกลเกิน (ToF ระยะ ~1 ม. คลาดได้ 200mm)
        rel_ok = before_mm <= C.MOVE_TOPUP_RANGE_MM
        if rel_ok:
            # ล้อ mecanum ลื่น/สั่นอยู่กับที่ได้โดยที่ chassis.move() ยังรายงานว่าสำเร็จ
            progress = before_mm - after
            min_progress = min(self.m.cell_m * 1000 * 0.3, 0.5 * fast_mm)
            if progress < min_progress:
                after2 = self.r.read_dir(d)   # อ่านซ้ำ กัน outlier ครั้งเดียว
                if before_mm - after2 > progress:
                    after, progress = after2, before_mm - after2
            if progress < min_progress:
                note = (f"เดินไป {DIRS[d]} แล้วระยะแทบไม่เปลี่ยน (ก่อน {before_mm:.0f}mm "
                        f"หลัง {after:.0f}mm)")
                if -progress > min_progress:
                    note += (" !! ไกลขึ้นแทนที่จะใกล้ = หุ่นเดินสวนทิศที่สั่ง "
                             "(เช็ค STRAFE_SIGN / YAW_SIGN ใน config.py)")
                self._fail_move(prev, "no_progress", {d: after}, note)
                return False
        if C.MOVE_TOPUP:
            after = self._finish_move(d, after, dist_mm, before_mm)
        if not closed:
            self._fix_lateral(d)        # แบบใหม่คุมข้างด้วยเซนเซอร์แล้ว (odometry ข้างมองไม่เห็นล้อลื่น ห้ามแก้ทับ)
        # ยืนยันว่า "ถึงช่องใหม่จริง" ก่อนนับว่าผ่าน
        ok, after, why = self._arrived(d, before_mm, after, dist_mm, rel_ok)
        if not ok and C.MOVE_PUSH_RETRY:
            ok, after = self._push_rest(d, after, dist_mm, before_mm)
            if ok:
                self._fix_lateral(d)
        if not ok:
            key = (prev, d)
            self.move_fails[key] = self.move_fails.get(key, 0) + 1
            note = f"เดินไป {DIRS[d]} ไม่ถึงช่องใหม่: {why} ครั้งที่ {self.move_fails[key]}"
            if self.move_fails[key] >= C.MOVE_FAIL_BLOCK:
                # ไปทางนี้ไม่ผ่านซ้ำ -> ให้ planner เลี่ยงชั่วคราวด้วย UNKNOWN (ห้ามตีเป็นกำแพง: log 30/9 15:34 ทางโล่ง
                # ToF 1350mm ถูกตีเป็นกำแพงตั้งแต่ช่องแรก) ; recheck_suspect_walls จะวัดซ้ำตัดสินอีกครั้ง
                self.m.set_lo(prev[0], prev[1], d, 0.0)
                note += " -> ขอบนี้ UNKNOWN ไว้ก่อน (recheck ตัดสินใหม่)"
            self._fail_move(prev, "incomplete_move", {d: after}, note)
            return False
        self.m.mark_passed(prev[0], prev[1], d)     # เดินผ่านขอบนี้มาแล้วจริง -> โล่งแน่
        self.L.log(self.r, "move", {d: after}, note=plan_note)
        self.on_update()
        return True

    def _align_heading(self, tof=None, light=False):
        """กำแพงของช่องนี้ (ToF ใกล้ ~195) ทิศใดทิศหนึ่ง -> วัดมุมตัวถังเทียบกำแพง -> เอียงเกิน ALIGN_MIN_DEG หมุนแก้
        (ลำแสง ToF เป็นกรวย วัดได้ต่ำกว่าจริง ~ครึ่ง -> แก้ซ้ำทุกช่องจนลู่เข้าตรง)
        light=True (ช่องที่เดินผ่านซ้ำ/รอบ 2): ทำเมื่อห่างจากครั้งก่อน ALIGN_LIGHT_EVERY_S (IMU ลอย ~1°/นาที)"""
        tof = self.last_tof if tof is None else tof
        if light and time.time() - getattr(self, "_align_t", 0.0) < C.ALIGN_LIGHT_EVERY_S:
            return
        cands = [d for d in (0, 2, 1, 3)
                 if isinstance(tof.get(d), (int, float)) and C.ALIGN_WALL_MM[0] <= tof[d] <= C.ALIGN_WALL_MM[1]]
        if not cands:
            return
        d = cands[0]
        res = self.r.wall_skew(d, C.ALIGN_THETA_DEG)
        if res is None:
            return
        psi, rp, rm = res
        note = (f"กำแพง {DIRS[d]}: ToF +{C.ALIGN_THETA_DEG:.0f}°={rp:.0f} -{C.ALIGN_THETA_DEG:.0f}°={rm:.0f} "
                f"-> ตัวถังเอียง{'ขวา' if psi > 0 else 'ซ้าย'} {abs(psi):.1f}°")
        # (log 1/10 02:50: เดิมเทียบกับค่าช่องก่อน -1.9° -> ช่องถัดไป -6.3° (IMU ลอย/ล้อไถลระหว่างทาง ไม่ใช่แก้ผิดทิศ)
        #  -> ปิดตัวเองตั้งแต่ช่องที่ 2 แล้ว IMU ลอยสะสม ~15° จนจบรอบ -> ตอนนี้ตรวจทันทีที่เดิมหลังหมุนแก้แทน)
        self._align_t = time.time()
        if abs(psi) < C.ALIGN_MIN_DEG:
            return
        if abs(psi) > C.ALIGN_MAX_DEG:
            self.L.log(self.r, "align", note=note + f" เกิน {C.ALIGN_MAX_DEG}° (ค่าน่าสงสัย) ไม่แก้")
            return
        step = max(-C.ALIGN_STEP_MAX_DEG, min(C.ALIGN_STEP_MAX_DEG, psi))
        self.L.log(self.r, "align", note=note + f" -> หมุนแก้ {step:+.1f}°")
        yaw0_before = self.r._yaw0
        self.r.apply_skew(step)
        n_chk = getattr(self, "_align_checks", 0)
        if n_chk >= C.ALIGN_VERIFY_N or abs(step) < 1.5 or self.stopped():
            return
        # ตรวจทิศการหมุนแก้ (เฉพาะช่วงแรก): วัดกำแพงเดิมซ้ำทันที หมุนถูกทิศ -> เอียงลดลง ; ผิดทิศ -> ~เท่าตัว
        res2 = self.r.wall_skew(d, C.ALIGN_THETA_DEG)
        if res2 is None:
            return
        psi2 = res2[0]
        self._align_checks = n_chk + 1
        if psi2 * psi > 0 and abs(psi2) > abs(psi) + 0.5 * abs(step):
            self.r._yaw0 = yaw0_before
            self.r.correct_yaw()
            C.ALIGN_ENABLE = False
            self.L.log(self.r, "align", note=f"หมุนแก้แล้ววัดซ้ำเอียง {psi2:+.1f}° (มากขึ้นจาก {psi:+.1f}°) "
                                             f"= หมุนผิดทิศ -> หมุนคืน + ปิดการแก้เอียงเอง (เช็คเครื่องหมาย yaw)")
        else:
            self.L.log(self.r, "align", note=f"ตรวจหลังแก้: เหลือเอียง {psi2:+.1f}° (ทิศการหมุนแก้ถูก)")

    def _drive_refs(self, d, before_mm):
        """สำหรับเดินแบบคุมตลอดทาง: กำแพงข้างสองฝั่ง {ทิศ: (ช่องนี้, ช่องปลายทาง)} 2=มี 1=ยังไม่รู้ 0=โล่ง +
        มีกำแพงที่ปลายช่องปลายทางไหม (ToF ก่อนเดิน)"""
        x, y = self.r.cell
        nx, ny = x + DX[d], y + DY[d]
        walls = {}
        for s in ((d + 1) % 4, (d + 3) % 4):
            def code(cx, cy):
                if not self.m.in_bounds(cx, cy):
                    return 0
                st = self.m.state(cx, cy, s)
                return 2 if st == WALL else (0 if st == OPEN else 1)
            walls[s] = (code(x, y), code(nx, ny))
        front = before_mm < C.WALL_EXPECT_MM + self.m.cell_m * 1000 + 150
        return walls, front

    def _lat_wall(self, d):
        """เดิน E/W (สไลด์): ทิศ N/S ที่มีกำแพงให้ ToF ใช้คุมกลางระหว่างเดิน (ช่องนี้+ช่องปลายทาง มีกำแพงมากสุด,
        เท่ากันเอาช่องนี้ก่อน) ; เดิน N/S หรือไม่มีกำแพงเลย -> None (N/S ใช้ Sharp ซ้าย/ขวาเอง)"""
        if d in (0, 2):
            return None
        x, y = self.r.cell
        nx, ny = x + DX[d], y + DY[d]
        best, score = None, 0
        for s in (0, 2):
            here = self.m.state(x, y, s) == WALL
            there = self.m.in_bounds(nx, ny) and self.m.state(nx, ny, s) == WALL
            sc = 2 * here + there + (1 if here and there else 0)
            if sc > score:
                best, score = s, sc
        return best

    # ---- ตัววัดระยะที่เดินได้ (ใช้ตัวที่เชื่อได้ที่สุดตัวเดียว) ----
    def _abs_front(self, d, after):
        """ToF หน้าหลังเดินเห็นกำแพงขอบไกลของช่องใหม่ (อ่านได้ < EXPECT + ABS_REF_FRAC*ช่อง และแผนที่ไม่ได้บอกว่า
        ขอบนั้นโล่ง) -> ระยะที่ยังต้องเดิน (mm, + = ยังไม่ถึง / - = เลย) เทียบ EXPECT ตรงๆ ; ไม่ใช่ -> None"""
        x, y = self.r.cell
        if after >= C.WALL_EXPECT_MM + C.ABS_REF_FRAC * self.m.cell_m * 1000:
            return None
        if self.m.in_bounds(x, y) and self.m.state(x, y, d) == OPEN:
            return None
        return after - C.WALL_EXPECT_MM

    def _odo_moved(self, d):
        """ระยะ (mm) ที่เดินจริงตาม odometry (หาร scale ที่เรียนได้ของสนามนี้) ; ใช้ไม่ได้ -> None"""
        odo = self.r.moved_along(d)
        if odo is None:
            return None
        return odo * 1000 / self._odo_k(0 if d in (1, 3) else 1)

    def _check_yaw(self, d):
        ok = getattr(self.r, "yaw_ok", None)
        if ok is not None and not ok():
            self.L.log(self.r, "yaw_warn", note=f"หลังเดิน {DIRS[d]} มุมตัวถังเพี้ยน {self.r.yaw():.1f}° "
                                                f"(ชน/IMU กระโดด) -> ไม่ใช้ odometry ก้าวนี้ ; แก้มุม")
            self.r.correct_yaw()

    def _pred_front(self, d, before_mm):
        """ToF หน้าที่ควรอ่านได้ตอนนี้ตาม odometry = ค่าก่อนเดิน (อ่านตอนหุ่นนิ่ง แม่น) - ระยะที่เดินไป ; ไม่รู้ -> None"""
        if before_mm is None or before_mm > C.MOVE_TOPUP_RANGE_MM:
            return None
        moved = self._odo_moved(d)
        if moved is None:
            return None
        return max(C.TOF_BLIND_ZONE_MM, before_mm - moved)

    def _front_after_move(self, d, before_mm):
        """ToF หน้าหลังเดิน/ขยับ: ToF หน่วง - ทันทีหลังเดินเร็วยังอ่านยาวกว่าจริง ~110mm แล้วค่อยๆ ลดลง
        (log 30/9 20:57: (4,5)/(3,2)/(1,5) อ่าน 317-382 ทั้งที่จริง ~200-270 -> เติมตามนั้นแล้วชนกำแพงหน้า ToF=60)
        -> อ่านซ้ำจนค่าไม่ลดลงแล้ว + ถ้ายังยาวกว่าที่ odometry ทำนายเกิน TOF_LAG_MAX_MM ใช้ค่าทำนาย (ใกล้กว่า = ไม่ชน)"""
        a = self.r.read_dir(d)
        for _ in range(C.TOF_LAG_READS):
            time.sleep(C.TOF_LAG_WAIT_S)
            b = self.r.read_dir(d)
            done = abs(b - a) < C.TOF_LAG_TOL_MM
            a = b
            if done:
                break
        pred = self._pred_front(d, before_mm)
        x, y = self.r.cell
        if self.m.in_bounds(x, y) and self.m.state(x, y, d) == OPEN:
            pred = None                     # ข้างหน้าช่องนี้โล่ง (แผนที่รู้แล้ว) = ไม่มีกำแพงให้ชน ไม่ต้องกัน
        if pred is not None and a > pred + C.TOF_LAG_MAX_MM and a < C.TOF_MAX_VALID_MM:
            self.L.log(self.r, "tof_lag", {d: a},
                       note=f"ToF หน้า {DIRS[d]} อ่าน {a:.0f}mm แต่ odometry บอกควรเหลือ {pred:.0f}mm "
                            f"(ก่อนเดิน {before_mm:.0f}) -> ใช้ {pred:.0f} (กันชน)")
            a = pred
        return a

    def _finish_move(self, d, after, dist_mm, before_mm=None):
        """ช่วงสุดท้าย (ช้า, SHIFT_SPEED) ให้ถึงกลางช่องใหม่พอดี -> ToF หน้าล่าสุด
        มีกำแพงหน้า: เทียบ EXPECT ตรงๆ ; ไม่มี: odometry (scale ของสนาม) ; ไม่รู้ทั้งคู่: ไม่ขยับ (recenter ดูแลต่อ)"""
        E = C.WALL_EXPECT_MM
        for _ in range(C.MOVE_TOPUP_TRIES):
            rest = self._abs_front(d, after)
            how = "กำแพงหน้า"
            if rest is None:
                moved = self._odo_moved(d)
                if moved is None:
                    break
                rest, how = dist_mm - moved, "odometry"
                if after < C.TOF_MAX_VALID_MM:
                    rest = min(rest, after - E)          # ห้ามเข้าใกล้สิ่งที่ ToF เห็นข้างหน้าเกิน EXPECT
                rest = min(rest, C.ODOM_TOPUP_MAX_MM)
            if -C.MOVE_OVERSHOOT_MIN_MM < rest < C.MOVE_TOPUP_MIN_MM:
                break
            mv = max(-C.RECENTER_MAX_MM, min(C.MOVE_TOPUP_MAX_MM, rest)) / 1000.0
            self.r.shift(DX[d] * mv, DY[d] * mv)
            prev, after = after, self._front_after_move(d, before_mm)
            self.L.log(self.r, "overshoot" if mv < 0 else "topup", {d: after},
                       note=f"เดิน {DIRS[d]}: ขาด {rest:+.0f}mm ({how}) -> {'ถอย' if mv < 0 else 'เติม'} "
                            f"{abs(mv) * 1000:.0f}mm ToF {prev:.0f}->{after:.0f}mm")
            if abs(mv) >= 0.08 and abs(prev - after) < 0.25 * abs(mv) * 1000 and after < C.TOF_MAX_VALID_MM:
                self.r.recover_chassis()        # สั่งแล้วไม่ขยับ = action เก่าค้าง/ติด -> หยุดล้อ ไม่ดันต่อ
                break
        return after

    def _arrived(self, d, before_mm, after, dist_mm, rel_ok):
        """ถึงช่องใหม่จริงไหม -> (ถึง, ToF, เหตุผลถ้าไม่ถึง)
        กำแพงหน้าของช่องใหม่อยู่ที่ ~EXPECT = ถึงแน่ ; ToF ก่อน-หลัง (ค่าก่อนเดินไม่ไกล) ; ไม่งั้น odometry"""
        need = C.MOVE_ACCEPT_FRAC * dist_mm
        rest = self._abs_front(d, after)
        if rest is not None and rest < 0.4 * self.m.cell_m * 1000:
            return True, after, ""
        if rel_ok:
            if before_mm - after < need:
                again = self.r.read_dir(d)          # อ่านซ้ำ 1 ครั้งก่อนสรุปว่าไม่ถึง
                after = min(after, again)
            if before_mm - after >= need:
                return True, after, ""
            # ToF ระยะ ~1 ม. คลาดได้ 200mm -> ถ้า odometry บอกว่าเดินครบ และ ToF ยืนยันว่าขยับจริงอย่างน้อยครึ่งหนึ่ง
            # (ไม่ใช่ล้อหมุนฟรีอยู่กับที่) = ถึง ; ไม่งั้นถอยกลับทั้งที่ถึงแล้ว
            moved = self._odo_moved(d)
            if moved is not None and moved >= need and before_mm - after >= 0.5 * need:
                return True, after, ""
            return False, after, (f"ToF ขยับ {before_mm - after:.0f}mm"
                                  + (f" odometry {moved:.0f}mm" if moved is not None else "")
                                  + f" (ต้อง ≥{need:.0f})")
        moved = self._odo_moved(d)
        if moved is None or moved >= need:
            return True, after, ""
        return False, after, f"odometry {moved:.0f}mm (ต้อง ≥{need:.0f})"

    def _fix_lateral(self, d):
        """หลังเดิน: เยื้องด้านข้าง (ตั้งฉากทิศเดิน) ที่เกิดระหว่างเดิน/ไถล ตาม odometry -> ขยับคืน
        (log 30/9 15:47: ช่องโล่งไม่มีกำแพงข้างให้วัด เยื้องสะสม -> เข้าช่องแคบ (3,3) แล้วชนปลายกำแพง)"""
        lat = getattr(self.r, "moved_lateral", None)
        lat = lat(d) if lat else None
        if lat is None or abs(lat) * 1000 < C.LATERAL_FIX_MM:
            return
        mv = max(-C.LATERAL_FIX_MAX_MM, min(C.LATERAL_FIX_MAX_MM, -lat * 1000)) / 1000.0
        r = (d + 1) % 4
        self.r.shift(DX[r] * mv, DY[r] * mv)
        self.L.log(self.r, "lateral_fix", note=f"เดิน {DIRS[d]} แล้วเยื้องข้าง {lat * 1000:+.0f}mm (odometry) "
                                               f"-> ขยับคืน {abs(mv) * 1000:.0f}mm")

    def _push_rest(self, d, after, dist_mm, before_mm=None):
        """เดินค้างกลางทาง (ข้ามรอยต่อพื้นไม่ผ่าน/คำสั่ง timeout) แต่ข้างหน้ายังโล่งพอ -> หยุดล้อ แล้วเดินส่วนที่เหลือ
        ด้วยความเร็วเดิน (มีแรงส่งข้ามรอยต่อ) อีก 1 ครั้ง -> (ถึงไหม, ToF ล่าสุด)
        ต้องรู้ระยะที่เดินได้แน่ (odometry ใช้ได้) ; ไม่เกิน PUSH_REST_MAX_MM ; ห้ามเข้าใกล้ข้างหน้าเกิน EXPECT"""
        moved = self._odo_moved(d)
        if moved is None:
            return False, after
        rest = min(dist_mm - moved, C.PUSH_REST_MAX_MM)
        if after < C.TOF_MAX_VALID_MM:
            rest = min(rest, after - C.WALL_EXPECT_MM)
        if rest < 0.2 * dist_mm:
            return False, after
        self.r.recover_chassis()
        self.r.shift(DX[d] * rest / 1000.0, DY[d] * rest / 1000.0, speed=C.XY_SPEED)
        prev_after, after = after, self._front_after_move(d, before_mm)
        after = self._finish_move(d, after, dist_mm, before_mm) if C.MOVE_TOPUP else after
        m2 = self._odo_moved(d)
        ok = (m2 is not None and m2 >= C.MOVE_ACCEPT_FRAC * dist_mm) or (
            self._abs_front(d, after) is not None and self._abs_front(d, after) < 0.4 * self.m.cell_m * 1000)
        self.L.log(self.r, "push_rest", {d: after},
                   note=f"เดิน {DIRS[d]} ค้างที่ {moved:.0f}/{dist_mm:.0f}mm -> หยุดล้อแล้วเดินต่อ {rest:.0f}mm "
                        f"(ToF {prev_after:.0f}->{after:.0f}mm) {'ถึงแล้ว' if ok else 'ยังไม่ถึง'}")
        return ok, after

    def _plan_move(self, d, before_mm):
        """ระยะที่จะสั่งเดินไปทิศ d -> (mm, ระยะหน้าที่ใช้, note)
        ห้ามเดินไกลกว่าที่กำแพง/สิ่งกีดขวางข้างหน้าเหลือให้: เดิน min(1 ช่อง, ระยะหน้า - EXPECT)
        = ถ้าหุ่นล้ำไปข้างหน้าอยู่แล้ว จะเดินสั้นลงไปจอดกลางช่องปลายทางพอดี (log 30/9 14:11: (4,1) หน้า S 503
        แทน ~795 = ล้ำไป 29 ซม. แต่เดินเต็ม 630mm -> ชนกำแพง ; เดิมถอยก่อนเดินได้เฉพาะล้ำ 60-200mm)
        ลดระยะได้อย่างเดียว ไม่เดินเกิน 1 ช่อง: ToF ด้านข้างระยะ ~800-950 เคยอ่านยาวเกินจริง ~11 ซม.
        ((2,5) E=930 ทั้งที่กำแพง W บอกเยื้องแค่ 2 ซม.) ขาดเท่าไร _finish_move เติมหลังเดิน"""
        cell_mm = self.m.cell_m * 1000
        E = C.WALL_EXPECT_MM

        def dist_of(front):
            return max(0.2 * cell_mm, min(cell_mm, front - E))

        dist = dist_of(before_mm)
        if cell_mm - dist > 60:
            # จะเดินสั้นกว่า 1 ช่องมาก -> ยืนยันด้วยค่าอ่านสดอีก 2 ครั้ง (ค่าเดิมอาจเป็นค่าหลอก th+1 จาก sense)
            before_mm = sorted([before_mm, self.r.read_dir(d), self.r.read_dir(d)])[1]
            dist = dist_of(before_mm)
        note = ""
        if cell_mm - dist >= 40:
            note = (f"หน้า {DIRS[d]} {before_mm:.0f}mm -> เดินแค่ {dist:.0f}mm (ปกติ {cell_mm:.0f}) "
                    f"ให้จอดกลางช่องปลายทาง ไม่ชนกำแพง")
        return dist, before_mm, note

    def _fail_move(self, prev, event, tof, note):
        """เดินไม่สำเร็จ: ย้อนตำแหน่งจริงตาม odometry + ย้อนสถานะ แล้วสแกน **โดยไม่อัปเดตแผนที่**
        (เดิม: สแกนแล้วเขียนลงแผนที่ทันที ทั้งที่หุ่นอาจไม่ได้อยู่ช่องที่คิด -> เขียนทับกำแพง
        ที่เคยบันทึกถูกแล้ว ตามที่เห็นใน log 27/9 20:47)"""
        self.L.log(self.r, event, tof, note=note + " -> ถอยกลับ ไม่นับว่าผ่าน")
        self.r.undo_last_move()
        self.r.cell = prev
        self.last_tof = {}
        self._sense_no_look(update_map=False, event="scan_after_fail")

    def _sense_here(self, update_map=True, event="scan"):
        obs = self.sense(update_map, event)
        self.last_tof["_cell"] = self.r.cell
        if update_map and C.ALIGN_ENABLE and hasattr(self.r, "wall_skew") and not self.stopped():
            self._align_heading()           # หลังจัดกลางแล้ว: แก้ตัวถังเอียงเทียบกำแพงจริง (ก่อนยิง/เดินต่อ)
        if self.hunter is not None and not self.stopped():
            self.hunter.after_scan()        # ยิงป้ายที่เห็นระหว่างสแกนช่องนี้ (ถ้าอยู่ในระยะ)
            if update_map and not self.stopped() and hasattr(self.hunter, "engage_close"):
                self.hunter.engage_close(self.r.heading)   # หลังจัดกลางแล้ว: ก้มดูกำแพงชิดด้านที่สไลด์ผ่าน
        return obs

    def follow(self, path, rescan=True):
        """เดินตามลิสต์ทิศ; คืน True ถ้าถึง"""
        for d in path:
            if self.stopped():
                return False
            if not self.step(d):
                return False
            x, y = self.r.cell
            if rescan and C.LIGHT_RESCAN and self.m.visited[x][y]:
                self.light_sense()          # ช่องที่เคยสแกนครบแล้ว: อ่านแค่ทิศที่ใช้จัดกึ่งกลาง
            elif rescan:
                self._sense_here()
            else:
                self.r.correct_yaw()
        return True

    def light_sense(self):
        """ผ่านช่องที่เคยสแกนครบแล้ว (เดินย้อน): ไม่สแกน 4 ทิศ/ไม่หาป้าย/ไม่อ่านซ้ำ อ่าน ToF เฉพาะทิศที่มีกำแพง
        (หรือกำแพงไกลของช่องถัดไป) ไว้จัดกึ่งกลาง  (log 30/9: สแกน 26 ครั้งใน 20 ช่อง ครั้งละ ~6 วิ)
        ถ้ากำแพงที่แผนที่บอกว่ามี กลับอ่านได้ว่าโล่ง = ตำแหน่งอาจผิด -> สแกนเต็มแทน"""
        x, y = self.r.cell
        th = C.WALL_THRESHOLD_MM
        self.r.correct_yaw()
        dirs = []
        for pos, neg in ((0, 2), (1, 3)):
            near = [d for d in (pos, neg) if self.m.state(x, y, d) == WALL]
            if near:
                dirs += near
                continue
            for d in (pos, neg):
                nx, ny = x + DX[d], y + DY[d]
                if (self.m.in_bounds(nx, ny) and self.m.state(x, y, d) == OPEN
                        and self.m.state(nx, ny, d) == WALL):
                    dirs.append(d)
        tof = {d: self.r.read_dir(d) for d in dirs}
        bad = [d for d in dirs if self.m.state(x, y, d) == WALL and tof[d] >= th]
        if bad:
            self.L.log(self.r, "light_scan", tof,
                       note=f"กำแพง {''.join(DIRS[d] for d in bad)} ในแผนที่อ่านได้โล่ง -> สแกนเต็ม")
            return self._sense_here()
        self.last_tof = dict(tof)
        self.last_tof["_cell"] = (x, y)
        self.L.log(self.r, "light_scan", tof)
        if C.RECENTER:
            self.recenter(tof)
        if C.ALIGN_ENABLE and hasattr(self.r, "wall_skew") and not self.stopped():
            self._align_heading(tof, light=True)     # ขากลับ/รอบ 2 ก็แก้เอียง (log 1/10: ตอนท้ายเอียงสะสม)
        if self.hunter is not None and not self.stopped():
            self.hunter.look_walls()        # ช่องนี้ยังไม่เคยส่องกำแพงที่อยู่ระยะยิงพอดี -> ส่องหาป้าย
            self.hunter.after_scan()        # ยิงป้ายที่เพิ่งเห็น + ป้ายที่รู้ตำแหน่งแล้ว (รวมขยับเข้าถ้าไกลเกินนิด)
        self.on_update()

    # ------------------------------------------------------------ MISSIONS
    def explore(self):
        m = self.m
        self.r.reset_heading_ref()      # หุ่นวางตรงกับกริดตอนเริ่ม = มุมอ้างอิงที่ถูกต้อง
        self.r.odo_ref = [None, None]   # จุดยึด odometry ของ recenter เริ่มใหม่ทุกครั้ง
        self.r.odo_k = [[], []]         # scale odometry ที่เรียนได้ (สนามใหม่ = เรียนใหม่)
        self.m.start = [self.r.cell[0], self.r.cell[1], DIRS[self.r.heading]]
        self.L.log(self.r, "start", note=f"START cell={self.r.cell} heading={DIRS[self.r.heading]}")
        self._sense_here()
        guard = 0
        rechecked = set()
        while not self.stopped() and guard < 400:
            guard += 1
            tgt = m.nearest_frontier(self.r.cell)
            if tgt is None:
                # ไม่มี frontier แล้ว แต่ยังมีช่องที่ไม่ได้ไป -> ตรวจกำแพงที่กั้นช่องเหล่านั้นซ้ำ (กันกำแพงหลอก)
                if self.recheck_suspect_walls(rechecked):
                    continue
                break
            path = m.astar(self.r.cell, tgt, self.r.heading, turn_cost=0.2)
            if path is None:
                # frontier ไปได้แค่ผ่านขอบ UNKNOWN (ToF/Sharp ขัดกัน) -> ตรวจขอบซ้ำก่อน ไม่ใช่จบเลย
                # (log 28/9 01:28: (1,1)E,(2,1)E UNKNOWN ทั้งคู่ -> จบสำรวจโดยไม่ได้ recheck)
                if self.recheck_suspect_walls(rechecked):
                    continue
                break
            self.plan = self._cells_of(path)
            self.echo(f"frontier -> {tgt}  path={''.join(DIRS[d] for d in path)}")
            self.follow(path, rescan=True)
            if self.hunter is not None and not self.stopped() and getattr(self.hunter, "close_queue", None):
                self.hunter.resolve_close()     # ป้ายกำแพงชิดที่เพิ่งเจอ -> ถอยไปยิงจากช่องแนวตรง แล้วสำรวจต่อ
        if self.hunter is not None and not self.stopped():
            self.hunter.sweep()             # ยังมีเป้าที่ต้องยิงค้าง -> ไปช่องยิงก่อนกลับบ้าน
        if C.RETURN_HOME and not self.stopped():
            self.return_home()
        self.plan = []
        m.end = [self.r.cell[0], self.r.cell[1], DIRS[self.r.heading]]
        vis, tot = m.counts()
        self.L.log(self.r, "end", note=f"END cell={self.r.cell}  visited {vis}/{tot}")
        self.on_update()
        return vis, tot

    def recheck_suspect_walls(self, rechecked):
        m = self.m
        sus = []
        for x in range(m.w):
            for y in range(m.h):
                if not m.visited[x][y]:
                    continue
                for d in range(4):
                    nx, ny = x + DX[d], y + DY[d]
                    if not m.in_bounds(nx, ny) or (x, y, d) in rechecked:
                        continue
                    st = m.state(x, y, d)
                    back = (nx, ny, (d + 2) % 4)
                    if back in rechecked:
                        continue
                    # (ก) กำแพงที่กั้นช่องที่ยังไม่ได้ไป  (ข) ขอบที่ผลวัดขัดกันจนยังเป็น UNKNOWN
                    if (st == WALL and not m.visited[nx][ny]) or st == UNKNOWN:
                        sus.append((x, y, d))
        if not sus:
            return False
        reach = m.reachable_cells(self.r.cell)
        sus = [s for s in sus if (s[0], s[1]) in reach]
        if not sus:
            return False
        sus.sort(key=lambda s: len(m.astar(self.r.cell, (s[0], s[1])) or []))
        x, y, d = sus[0]
        rechecked.add((x, y, d))
        self.L.log(self.r, "recheck_plan",
                   note=f"ตรวจกำแพงซ้ำ {(x, y)}->{DIRS[d]} (เหลืออีก {len(sus) - 1} จุด แล้วจะกลับจุดเริ่ม)")
        if not self.go_to((x, y), rescan=True):
            return True
        self._recheck_edge(x, y, d)
        return True

    def _recheck_edge(self, x, y, d):
        """หุ่นอยู่ช่อง (x,y) แล้ว: อ่าน ToF ทิศ d 3 ครั้ง ตัดสินขอบนี้ (ผลชี้ขาด)"""
        m = self.m
        reads = [self.r.read_dir(d) for _ in range(3)]
        # นับว่าโล่งต้องไกลกว่า threshold ชัดเจน: ขอบโล่งจริงอ่านได้ ~CELL+EXPECT (≥ ~650 แม้หุ่น
        # เยื้อง 150) ส่วนค่าก้ำกึ่ง = กำแพงที่หุ่นอยู่ห่าง (log 28/9 00:43: (2,1) N อ่าน 491 ถูกนับ
        # ว่าโล่ง -> ลบกำแพงจริงทิ้งแล้วเดินชน)
        open_mm = C.WALL_THRESHOLD_MM + 0.25 * m.cell_m * 1000
        n_open = sum(r >= open_mm for r in reads)
        # recheck = ตัวชี้ขาด: ใช้ ToF 3 ครั้ง (ยืนยันแล้วว่าทิศถูกหลังแก้ GIMBAL_RIGHT_YAW)
        # Sharp แค่บันทึกไว้เทียบ ถ้าขัดกันจะเห็นใน note
        sh = self.r.read_sharp(d)
        # การวัดซ้ำแบบตั้งใจ (หุ่นอยู่กลางช่อง อ่าน 3 ครั้ง) ให้น้ำหนักเป็นผลตัดสิน
        m.set_lo(x, y, d, -2.0 if n_open >= 2 else max(2.0, m.get_lo(x, y, d)))
        self.L.log(self.r, "recheck", {d: sorted(reads)[1]},
                   note=f"recheck {(x, y)}->{DIRS[d]}: {n_open}/3 open -> "
                        f"{'OPEN' if m.state(x, y, d) == OPEN else 'WALL'}"
                        + (f" (Sharp {sh[0]:.0f}mm)" if sh is not None else ""))
        self.on_update()

    def go_to(self, target, rescan=C.RESCAN_ON_NAV):
        """เดินไปช่องเป้าหมายด้วย A* บนแผนที่ที่รู้ (replan อัตโนมัติถ้าเจอกำแพงใหม่)"""
        target = tuple(target)
        self.L.log(self.r, "goto", note=f"GOTO {target} from {self.r.cell}")
        decided, blocks = set(), {}
        for _ in range(60):
            if self.stopped():
                return False
            if self.r.cell == target:
                self.plan = []
                self.L.log(self.r, "arrived", note=f"ARRIVED at {target}")
                self.on_update()
                return True
            path = self.m.astar(self.r.cell, target, self.r.heading, turn_cost=0.2)
            if path is None:
                # ลองผ่านขอบที่ยังไม่รู้ (unknown) แล้วค่อยสแกนยืนยันระหว่างทาง
                path = self.m.astar(self.r.cell, target, self.r.heading, allow_unknown=True)
                if path is None:
                    self.L.log(self.r, "goto_fail", note=f"no path to {target} in current map")
                    self.plan = []
                    return False
                rescan = True
            self.plan = self._cells_of(path)
            self.on_update()
            self.last_block = None
            if self.follow(path, rescan=rescan) or self.last_block is None:
                continue
            cell, d, conflict = self.last_block
            key = (cell, d)
            blocks[key] = blocks.get(key, 0) + 1
            if conflict and key not in decided:
                # ToF/Sharp ขัดกัน -> ขอบกลายเป็น UNKNOWN แล้ว A* ก็พามาทางเดิมอีก วนไม่จบ
                # (log 28/9 01:28: ติด (1,1)->E 50+ รอบ) -> ตัดสินด้วย ToF 3 ครั้งแบบ recheck
                decided.add(key)
                self._recheck_edge(cell[0], cell[1], d)
            elif blocks[key] >= 3:
                self.L.log(self.r, "goto_fail", note=f"ติดที่ {cell}->{DIRS[d]} ซ้ำ {blocks[key]} ครั้ง - หยุด")
                self.plan = []
                return False
        return False

    def return_home(self):
        """กลับจุดเริ่ม แล้วจอดหันทิศเดียวกับตอนเริ่ม รอคำสั่งต่อไป"""
        if not self.m.start:
            return False
        self.L.log(self.r, "return_home", note=f"สำรวจ+ตรวจซ้ำครบแล้ว -> กลับจุดเริ่ม {tuple(self.m.start[:2])}")
        if not self.go_to(tuple(self.m.start[:2])):
            return False
        self.park()
        return True

    def park(self):
        """หันตัวถังกลับทิศเริ่มต้น (m.start[2]) + gimbal ชี้หน้า — ไม่อัปเดตแผนที่"""
        hd = self.m.start[2] if self.m.start else DIRS[self.r.heading]
        self.r.face(hd)
        self.L.log(self.r, "parked", note=f"PARKED at {self.r.cell} heading={hd} - รอคำสั่ง")
        self.on_update()

    def _sense_no_look(self, **kw):
        """สแกนตอนไม่แน่ใจตำแหน่ง (เดินพลาด/หาตำแหน่ง) -> ไม่หาป้าย (ตำแหน่งป้ายจะผิดตาม)"""
        look, self.r.on_look = getattr(self.r, "on_look", None), None
        try:
            return self.sense(**kw)
        finally:
            self.r.on_look = look

    def where_am_i(self):
        obs = self._sense_no_look(update_map=False, event="relocalize")
        cands = self.m.match_candidates(tuple(obs[d] for d in range(4)))
        return obs, cands

    def _cells_of(self, path):
        x, y = self.r.cell
        cells = [(x, y)]
        for d in path:
            x += DX[d]; y += DY[d]
            cells.append((x, y))
        return cells
