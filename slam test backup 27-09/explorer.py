"""
explorer.py — หัวใจของ SLAM แบบกริด

วงจรที่ทุกช่อง:
  1) SENSE   : หมุน gimbal สแกน ToF 4 ทิศ
  2) MAP     : อัปเดต log-odds ของขอบช่อง (กำแพง/โล่ง)  -> Mapping
  3) LOCALIZE: ตรวจลายกำแพงเทียบแผนที่ + วัดค่าเบี่ยงจากกลางช่องด้วยระยะกำแพง
               แล้ว Recenter  -> Localization (ลด error สะสมจาก odometry)
  4) PLAN    : BFS หาช่องที่ยังไม่สำรวจที่ใกล้ที่สุด (frontier) + A* หาเส้นทาง
  5) ACT     : เดินทีละช่อง (ตัวถังไม่หมุน ใช้ล้อ mecanum) + แก้มุม yaw ด้วย IMU
"""
import csv
import os
import time

import config as C
from grid_map import DIRS, DX, DY, WALL, OPEN, UNKNOWN


class RunLogger:
    FIELDS = ["step", "time_s", "event", "cell_x", "cell_y", "heading",
              "est_x_m", "est_y_m", "odom_x_m", "odom_y_m", "yaw_deg",
              "tof_N", "tof_E", "tof_S", "tof_W", "walls_NESW", "note"]

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

    def log(self, robot, event, tof=None, walls=None, note=""):
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
        self.w.writerow(row)
        self.f.flush()
        self.traj.append((t, ex, ey, od[0] if od else None, od[1] if od else None,
                          robot.cell[0], robot.cell[1], event))
        self.step += 1
        if note:
            self.echo(f"[{event}] {note}")

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


class Explorer:
    def __init__(self, robot, gmap, logger, on_update=None, stop_event=None, echo=print):
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

    def stopped(self):
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
        for d in range(4):
            if self.m.is_boundary(x, y, d):
                continue                    # ขอบสนาม: รู้อยู่แล้วว่าเป็นกำแพง (ดูด้านล่าง)
            sh = self.r.read_sharp(d)
            if sh is None:
                continue
            s_wall = sh[0] < C.SHARP_WALL_MM
            if s_wall == (obs[d] == WALL):
                continue
            t2 = self.r.read_dir(d)
            sh2 = self.r.read_sharp(d) or sh
            t_wall2, s_wall2 = t2 < th, sh2[0] < C.SHARP_WALL_MM
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
        self.L.log(self.r, event, tof, walls, note)

        if C.RECENTER:
            self.recenter(tof)
        self.on_update()
        return obs

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
                    off = (odo[i] - ref[i] - centre[i]) * 1000
                    if abs(off) > 0.4 * self.m.cell_m * 1000:
                        ref[i] = None           # เยื้องเกินครึ่งช่อง = จุดยึดผิด (เช่น relocalize) ทิ้ง
                    else:
                        pred[i] = -off
        cur_axis = [None]

        def clearly_open(d):
            v = tof.get(d, 1e9)
            return v == th + 1 or v >= th + 0.25 * self.m.cell_m * 1000   # th+1 = ToF/Sharp ขัดกัน (sense)

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
                return (dp - dn) / 2.0, (pos, neg)
            if plausible(pos) and (not plausible(neg) or abs(dp - E) <= abs(dn - E)):
                return dp - E, (pos,)
            if plausible(neg):
                return -(dn - E), (neg,)
            return 0.0, ()

        corr = [0.0, 0.0]    # mm, world x/y  (ทิศที่ต้องขยับ)
        hold = []
        for pos, neg, axis in ((0, 2, 1), (1, 3, 0)):     # (N,S)->y  (E,W)->x
            cur_axis[0] = axis
            c, used = axis_corr(pos, neg)
            if abs(c) > 60 and used:
                # แก้เยอะ -> อ่านซ้ำทิศที่ใช้ แล้วใช้ค่ากลาง (กันค่า ToF หลุดทำให้หุ่นขยับผิด)
                hist = {d: [tof[d]] for d in used}
                for d in used:
                    hist[d] += [self.r.read_dir(d), self.r.read_dir(d)]
                    tof[d] = sorted(hist[d])[1]
                c, used = axis_corr(pos, neg)
            if used:
                if odo is not None:     # ToF วัดแกนนี้ได้ -> ตั้งจุดยึด odometry ใหม่
                    ref[axis] = odo[axis] - (centre[axis] - c / 1000.0)
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

    # ------------------------------------------------------------ ACT
    def step(self, d):
        """เดิน 1 ช่อง ไปทิศ d ; คืน False ถ้าไปไม่ได้ (เจอกำแพง)"""
        x, y = self.r.cell
        th = C.WALL_THRESHOLD_MM
        # 1) ยืนยันก่อนเดิน: อ่าน ToF ทิศที่จะไปใหม่อีกครั้ง (รวมกับค่าจากการสแกน) แล้วโหวตเสียงข้างมาก
        reads = [self.r.read_dir(d)]
        if self.last_tof.get("_cell") == (x, y) and d in self.last_tof:
            reads.append(self.last_tof[d])
        if len(reads) == 1 or (reads[0] < th) != (reads[1] < th):
            reads.append(self.r.read_dir(d))
        walls_n = sum(r < th for r in reads)
        tof_wall = walls_n * 2 >= len(reads) and walls_n > 0
        sh = self.r.read_sharp(d)           # เดินข้าง: Sharp ฝั่งนั้นช่วยยืนยัน
        sharp_wall = sh is not None and sh[0] < C.SHARP_WALL_MM
        if sharp_wall and not tof_wall and self.m.get_lo(x, y, d) <= -2.0:
            # ขอบนี้ recheck/สแกนหลายรอบยืนยันแล้วว่าโล่ง -> Sharp ค้านตัวเดียวไม่พอให้หยุด
            # (ไม่งั้นวนซ้ำ: recheck บอกโล่ง -> step ถูก Sharp ห้าม -> UNKNOWN -> recheck ใหม่)
            # ถ้าผิดจริง เช็ค progress หลังเดินจะจับได้และถอยกลับ
            sharp_wall = False
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
        # ระยะไปทิศ d ก่อนเดิน (ใช้เช็คว่า "เดินไปจริง" ด้านล่าง) - ใช้ค่ากลาง (median)
        before_mm = sorted(reads)[len(reads) // 2]
        before_mm = self._premove_backoff(d, before_mm)
        self.r.move_cell(d)
        after = self.r.read_dir(d)
        # ตรวจว่า "เดินไปจริง": ล้อ mecanum ลื่น/สั่นอยู่กับที่ได้โดยที่ chassis.move() ยัง
        # รายงานว่าสำเร็จ -> เทียบระยะ ToF ทิศเดียวกันก่อน/หลัง
        # (ตัดเช็ค "collision" เดิมออก: lim = EXPECT - 0.25*CELL = 34mm ต่ำกว่า blind zone 60mm
        #  จึงไม่เคยทำงาน และถ้าทำงานก็ย้อนตำแหน่งผิด - หุ่นที่ชนกำแพงหน้าหลังเดินครบช่อง
        #  อยู่ในช่องใหม่แล้ว recenter จะถอยออกให้เอง)
        if before_mm < C.TOF_MAX_VALID_MM:
            progress = before_mm - after
            min_progress = self.m.cell_m * 1000 * 0.3
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
            # เติมระยะ: ขยับจริงไม่ถึง 1 ช่อง (ล้อลื่น/ไถลข้าง) -> เดินเพิ่มทีละนิดตาม ToF
            if C.MOVE_TOPUP and before_mm <= C.MOVE_TOPUP_RANGE_MM:
                after = self._top_up(d, before_mm, after)
        self.L.log(self.r, "move", {d: after})
        self.on_update()
        return True

    def _premove_backoff(self, d, before_mm):
        """ก่อนเดิน: ToF ข้างหน้าเห็นกำแพงไกลของช่องถัดไปใกล้กว่า CELL+EXPECT = หุ่นล้ำไปข้างหน้า
        เดินเต็มช่องจะชนกำแพงหน้า (log 28/9 00:03: (3,1) หน้า 652mm แทน ~784 -> เดินแล้วชน ToF 60)
        -> ถอยเท่าที่ล้ำก่อน แล้วค่อยเดิน (ไม่แก้กลางก้าว) ; คืนระยะหน้าใหม่"""
        cell_mm = self.m.cell_m * 1000
        lean = cell_mm + C.WALL_EXPECT_MM - before_mm
        # ล้ำได้แค่ 60-200mm เท่านั้นที่เชื่อว่าเป็นกำแพงไกลของช่องถัดไป - ใกล้กว่านั้นอาจเป็น
        # กำแพงขอบช่องนี้เองที่หุ่นอยู่ห่าง (log 28/9 00:43: หน้า 488 ถูกตีเป็นล้ำ 322 -> ถอย แล้วชน)
        if not C.PREMOVE_BACKOFF or not (60 <= lean <= 200):
            return before_mm
        # ยืนยันด้วยค่าอ่านสดอีกครั้ง: before_mm อาจมาจาก last_tof ที่ sense ใส่ค่าหลอก th+1
        # ไว้ (ToF/Sharp ขัดกัน) ซึ่งตกช่วงนี้พอดี (log 28/9: 425/454mm) -> ห้ามถอยตามค่านั้น
        fresh = self.r.read_dir(d)
        if abs(fresh - before_mm) > 60:
            return before_mm
        before_mm = (before_mm + fresh) / 2.0
        lean = cell_mm + C.WALL_EXPECT_MM - before_mm
        if not (60 <= lean <= 200):
            return before_mm
        back = min(lean, C.RECENTER_MAX_MM) / 1000.0
        self.r.shift(-DX[d] * back, -DY[d] * back)
        new = self.r.read_dir(d)
        if abs(new - (before_mm + back * 1000)) > 100:
            new = before_mm + back * 1000       # ค่าอ่านใหม่แปลก -> ใช้ค่าคำนวณ
        self.L.log(self.r, "premove_backoff", {d: new},
                   note=f"หน้า {DIRS[d]} {before_mm:.0f}mm (ควร ~{cell_mm + C.WALL_EXPECT_MM:.0f}) "
                        f"-> ถอย {back * 1000:.0f}mm ก่อนเดิน")
        return new

    def _fail_move(self, prev, event, tof, note):
        """เดินไม่สำเร็จ: ย้อนตำแหน่งจริงตาม odometry + ย้อนสถานะ แล้วสแกน **โดยไม่อัปเดตแผนที่**
        (เดิม: สแกนแล้วเขียนลงแผนที่ทันที ทั้งที่หุ่นอาจไม่ได้อยู่ช่องที่คิด -> เขียนทับกำแพง
        ที่เคยบันทึกถูกแล้ว ตามที่เห็นใน log 27/9 20:47)"""
        self.L.log(self.r, event, tof, note=note + " -> ถอยกลับ ไม่นับว่าผ่าน")
        self.r.undo_last_move()
        self.r.cell = prev
        self.last_tof = {}
        self.sense(update_map=False, event="scan_after_fail")

    def _top_up(self, d, before_mm, after):
        """เดินเพิ่มไปทิศ d จนระยะที่ขยับจริง (ToF ก่อน-หลัง) ใกล้ CELL_M ; คืนค่า ToF ล่าสุด"""
        cell_mm = self.m.cell_m * 1000
        for _ in range(C.MOVE_TOPUP_TRIES):
            # ห้ามเติมจนใกล้กำแพงข้างหน้าเกิน WALL_EXPECT_MM (log 2026-09-27: หน้า 466mm
            # เดินได้ 406mm แล้วยังสั่งเติมอีก 194mm x2 -> ดันชนกำแพง ToF อ่าน 60)
            short = min(cell_mm - (before_mm - after), after - C.WALL_EXPECT_MM)
            if short < C.MOVE_TOPUP_MIN_MM:
                break
            add = min(short, C.MOVE_TOPUP_MAX_MM) / 1000.0
            self.r.shift(DX[d] * add, DY[d] * add)
            prev_after, after = after, self.r.read_dir(d)
            if after > prev_after + C.MOVE_TOPUP_MIN_MM:
                # เติมแล้วไกลขึ้น = ToF ชี้หลุด/หุ่นเบี้ยว (log 27/9 20:46: ToF กระโดด 3000
                # แล้วคิดว่าขาด 2774mm) -> หยุดเติม
                self.L.log(self.r, "topup", {d: after},
                           note=f"เติมแล้วระยะไกลขึ้น ({prev_after:.0f}->{after:.0f}mm) - หยุดเติม")
                break
            self.L.log(self.r, "topup", {d: after},
                       note=f"เดิน {DIRS[d]} ขาด {short:.0f}mm -> เติม {add * 1000:.0f}mm")
        return after

    def _sense_here(self, update_map=True, event="scan"):
        obs = self.sense(update_map, event)
        self.last_tof["_cell"] = self.r.cell
        return obs

    def follow(self, path, rescan=True):
        """เดินตามลิสต์ทิศ; คืน True ถ้าถึง"""
        for d in path:
            if self.stopped():
                return False
            if not self.step(d):
                return False
            if rescan:
                self._sense_here()
            else:
                self.r.correct_yaw()
        return True

    # ------------------------------------------------------------ MISSIONS
    def explore(self):
        m = self.m
        self.r.reset_heading_ref()      # หุ่นวางตรงกับกริดตอนเริ่ม = มุมอ้างอิงที่ถูกต้อง
        self.r.odo_ref = [None, None]   # จุดยึด odometry ของ recenter เริ่มใหม่ทุกครั้ง
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

    def where_am_i(self):
        obs = self.sense(update_map=False, event="relocalize")
        cands = self.m.match_candidates(tuple(obs[d] for d in range(4)))
        return obs, cands

    def _cells_of(self, path):
        x, y = self.r.cell
        cells = [(x, y)]
        for d in path:
            x += DX[d]; y += DY[d]
            cells.append((x, y))
        return cells
