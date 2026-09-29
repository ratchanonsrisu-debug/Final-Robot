"""
aiming.py — โมเดลจุดกระทบ + การเล็งแบบวัดมุมแล้วสั่งหมุน (look-then-move)

ทำไมยิงไม่ตรงกับ crosshair?
  กล้องกับลำกล้องไม่ได้อยู่ตำแหน่งเดียวกัน (ห่างกัน b ซม.) และแกนอาจเอียงต่างกันเล็กน้อย (θ)
  จุดที่กระสุนไปโดนเมื่อมองผ่านกล้อง:  u = cx + f·tanθx + f·bx / Z   ->   u = cx + A + B/Z
  => offset ขึ้นกับ "ระยะถึงเป้า Z" ไม่ใช่ลำดับป้าย (ค่าคงที่ตัวเดียวจึงใช้ไม่ได้ทุกระยะ)
  กระสุนเจลตกตามแรงโน้มถ่วง: ระยะตก ∝ Z²  -> ในภาพ ∝ Z  จึงเพิ่มพจน์ Cy·Z ในแกนตั้ง

ค่า A, B, C มาจากการ calibrate ในโปรแกรม Colorshoot แล้ว (settings.json ส่วน hit_model) — โฟลเดอร์นี้ไม่มีโค้ด calibrate

การเล็ง: แปลง error พิกเซล -> มุม (atan(err/f)) แล้วสั่ง gimbal.move ด้วยมุมนั้นตรงๆ
  รอ gimbal นิ่ง + ภาพตามทัน แล้ววัดใหม่ ทำซ้ำจนคลาด < tolerance  (ไม่แกว่งจาก delay ของภาพแบบ PID ความเร็ว)
"""
import math
import os
import time

from vision import focal_px


class HitModel:
    def __init__(self, settings):
        self.S = settings

    def params(self, ammo=None):
        ammo = ammo or self.S["aim"]["ammo"]
        return self.S["hit_model"][ammo]

    def hit_px(self, w, h, z_cm, ammo=None):
        p = self.params(ammo)
        k = w / float(self.S["hit_model"]["base_w"])
        z = max(z_cm, 10.0)
        u = w / 2.0 + k * (p["Ax"] + p["Bx"] / z)
        v = h / 2.0 + k * (p["Ay"] + p["By"] / z + p["Cy"] * z)
        return u, v


def pixel_to_angle(du, dv, w, hfov):
    f = focal_px(w, hfov)
    return math.degrees(math.atan2(du, f)), -math.degrees(math.atan2(dv, f))   # yaw(+ขวา), pitch(+ขึ้น)


def target_point(track_or_det, aim_y_ratio):
    d = track_or_det.get("det", track_or_det)
    x, y, bw, bh = d["bbox"]
    cx = track_or_det.get("cx", d["cx"])
    cy = track_or_det.get("cy", d["cy"])
    # aim_y_ratio 0.5 = กึ่งกลาง (ใช้ centroid) ; อื่นๆ เลื่อนตามความสูงของส่วนสี
    ty = cy + (aim_y_ratio - 0.5) * bh
    return cx, ty


class Aimer:
    """ใช้ได้ทั้งหุ่นจริงและหุ่นจำลอง (robot ต้องมี read_frame, gimbal_move, gimbal_angles, fire)"""

    def __init__(self, robot, detector, tracker, settings, log=print, stop_flag=None):
        self.r, self.det, self.trk, self.S = robot, detector, tracker, settings
        self.model = HitModel(settings)
        self.log = log
        self.stop = stop_flag
        self.state = {}          # ให้ dashboard อ่านไปวาด
        self.shot_dir = None     # ถ้าตั้งไว้: เซฟภาพทุกนัด (ตอนล็อคเป้าก่อนยิง + หลังยิง) ไว้ย้อนดู
        self._lock = None

    def _save_shot_img(self, frame, d, ammo, tag, text):
        """เซฟภาพ 1 นัด: กรอบป้าย, จุดเล็ง (วงขาว), จุดที่คาดว่ากระสุนจะโดน (X แดง), กลางภาพ (+ เทา)"""
        if not self.shot_dir or frame is None:
            return ""
        import cv2
        os.makedirs(self.shot_dir, exist_ok=True)
        img = frame.copy()
        h, w = img.shape[:2]
        cv2.drawMarker(img, (w // 2, h // 2), (200, 200, 200), cv2.MARKER_CROSS, 24, 1)
        if d is not None:
            x, y, bw, bh = d["bbox"]
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
            tx, ty = target_point(d, self.S["aim"]["aim_y_ratio"])
            hx, hy = self.model.hit_px(w, h, d["dist_cm"], ammo)
            cv2.circle(img, (int(tx), int(ty)), 6, (255, 255, 255), 2)
            cv2.drawMarker(img, (int(hx), int(hy)), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
        from vision import put_text
        put_text(img, text, (10, 26), 0.6)
        name = f"{time.strftime('%H%M%S')}_{tag}.jpg"
        cv2.imwrite(os.path.join(self.shot_dir, name), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        return name

    def stopped(self):
        return self.stop is not None and self.stop.is_set()

    def fresh_frame(self):
        time.sleep(self.S["aim"]["settle_s"])
        f = None
        for _ in range(2):                 # ทิ้งเฟรมเก่าใน buffer เอาเฟรมล่าสุด
            nf = self.r.read_frame()
            if nf is not None:
                f = nf
        return f

    def _clamped_move(self, dyaw, dpitch):
        """dyaw/dpitch ในกรอบภาพ (+ขวา, +ขึ้น) -> แปลงเป็นคำสั่งจริงด้วย sign แล้วจำกัดตามมุมล็อค"""
        G = self.S["gimbal"]
        ty, tp = dyaw * G["yaw_sign"], dpitch * G["pitch_sign"]      # คำสั่งในกรอบของ gimbal
        ang = self.r.gimbal_angles()
        if ang is not None:
            p, y = ang
            ty = min(max(ty, G["yaw_min"] - y), G["yaw_max"] - y)
            tp = min(max(tp, G["pitch_min"] - p), G["pitch_max"] - p)
        self.r.gimbal_move(tp, ty)
        return ty * G["yaw_sign"], tp * G["pitch_sign"]

    def find_target(self, dets, spec):
        """เลือก detection ที่ตรงสเปค (สี, รูปทรง) ใกล้ตำแหน่งที่คาดไว้ที่สุด
        ระหว่างเล็ง (รู้ตำแหน่งที่คาดไว้แล้ว) ถ้ารูปทรงสลับไปมาใกล้เส้นเกณฑ์ ยังถือเป็นป้ายเดิม
        เมื่อสีตรงและอยู่ที่เดิม (29 ก.ย.: เขียวนอนสลับเป็นจัตุรัส -> หาไม่เจอ -> ไม่ยิง)"""
        color, shape, exp = spec
        cands = [d for d in dets if d["color"] == color and d["shape"] == shape]
        if not cands and exp is not None:
            near = [d for d in dets if d["color"] == color
                    and math.hypot(d["cx"] - exp[0], d["cy"] - exp[1]) <= 0.8 * max(d["bbox"][2], d["bbox"][3])]
            if len(near) == 1:
                return near[0]
        if not cands:
            return None
        if exp is None:
            return max(cands, key=lambda d: d["area"])
        return min(cands, key=lambda d: math.hypot(d["cx"] - exp[0], d["cy"] - exp[1]))

    def aim_and_fire(self, color, shape, start_xy=None, fire=True, ammo=None):
        """เล็ง -> ยิง -> (เจล) ตรวจว่าป้ายล้มหรือยัง ถ้ายังตั้งอยู่ = กระสุนไม่ออก/ไม่โดน -> เล็งซ้ำแล้วยิงใหม่
        (ป้อนกระสุนเจลของ EP ไม่สม่ำเสมอ: สั่งยิง ~3 ครั้งออก 1 นัด จึงต้องยืนยันด้วยภาพ)"""
        A = self.S["aim"]
        ammo = ammo or A["ammo"]
        confirm = fire and A.get("confirm_hit_" + ammo, ammo == "gel")
        max_trig = int(A.get("max_triggers_" + ammo, 8 if ammo == "gel" else 1)) if fire else 1
        # ยิงรัว: สั่งครั้งเดียวออกหลายนัด (เจลป้อนไม่สม่ำเสมอ ยิงทีละนัดโอกาสโดนน้อย)
        burst = max(1, min(8, int(A.get("burst_" + ammo, 3 if ammo == "gel" else 1))))
        t0 = time.time()
        res = {"color": color, "shape": shape, "fired": False, "iters": 0, "triggers": 0, "knocked": None,
               "burst": burst}
        exp = start_xy
        for trig in range(max_trig):
            d, exp, why = self._aim_lock(color, shape, exp, ammo, res)
            if d is None:
                # หลังยิงไปแล้วมองไม่เห็นป้ายติดกันหลายเฟรม = น่าจะล้มแล้ว
                # (ถ้าแค่เล็งไม่เข้า tolerance แต่ยังเห็นป้าย ไม่นับว่าล้ม)
                if trig > 0 and confirm and why == "lost":
                    res["knocked"] = True
                break
            if not fire:
                break
            tag = f"{color}_{shape}_t{trig + 1}"
            ey, ep = res.get("err_yaw", 0), res.get("err_pitch", 0)
            res.setdefault("images", []).append(self._save_shot_img(
                self._lock, d, ammo, tag + "_a_aim",
                f"{color}/{shape} Z={d['dist_cm']:.0f}cm trigger {trig + 1} x{burst} err=({ey:+.2f},{ep:+.2f})deg [{ammo}]"))
            self.r.fire(ammo, times=burst)
            res["fired"] = True
            res["triggers"] += 1
            # SDK สั่งแล้วไปต่อทันที -> ต้องรอให้ยิงรัวครบก่อนขยับ gimbal ไม่งั้นนัดหลังๆ ออกระหว่างหมุน
            time.sleep(float(A.get("fire_hold_s_per_shot", 0.25)) * burst)
            if not confirm:
                if self.shot_dir:
                    time.sleep(float(A.get("confirm_wait_s", 0.8)))
                    res["images"].append(self._save_shot_img(self.fresh_frame(), None, ammo, tag + "_b_after",
                                                             f"{color}/{shape} after trigger {trig + 1}"))
                break
            time.sleep(float(A.get("confirm_wait_s", 0.8)))
            gone, still = self._check_gone(color, shape, d)
            res["images"].append(self._save_shot_img(self._after, still, ammo, tag + "_b_after",
                                                     f"{color}/{shape} after trigger {trig + 1}: "
                                                     f"{'GONE (knocked)' if gone else 'still standing'}"))
            if gone:
                res["knocked"] = True
                self.log(f"  [hit] {color}/{shape} ล้มแล้ว ✓ (สั่งยิง {res['triggers']} ครั้ง)")
                break
            res["knocked"] = False
            self.log(f"  [hit] {color}/{shape} ยังตั้งอยู่ -> ยิงซ้ำ ({res['triggers']}/{max_trig})")
            if still is not None:
                exp = (still["cx"], still["cy"])
        res["time_s"] = round(time.time() - t0, 2)
        return res

    def _check_gone(self, color, shape, d):
        """หลังยิง: ป้ายล้มจริงไหม -> (gone, detection ที่ยังเห็นหรือ None)
        ต้องไม่เห็นป้ายติดกัน confirm_knock_frames เฟรม (กันตรวจจับหลุดเฟรมเดียวแล้วเข้าใจผิดว่าล้ม)"""
        n = max(1, int(self.S["aim"].get("confirm_knock_frames", 3)))
        x, y, bw, bh = d["bbox"]
        self._after = None
        for i in range(n):
            frame = self.fresh_frame() if i == 0 else self.r.read_frame()
            if frame is None:
                return False, None
            if i == 0:
                self._after = frame
            still = self.find_target(self.det.detect(frame), (color, shape, (d["cx"], d["cy"])))
            if still is not None and math.hypot(still["cx"] - d["cx"], still["cy"] - d["cy"]) <= 1.5 * max(bw, bh):
                return False, still
        return True, None

    def _aim_lock(self, color, shape, exp, ammo, res):
        """เล็งจนคลาด < tolerance ติดกัน confirm_frames ครั้ง
        -> คืน (detection, exp, "ok") หรือ (None, exp, why)  why = "lost" (ไม่เห็นป้ายติดกัน 3 เฟรม) / "nolock" / "stopped" """
        A = self.S["aim"]
        hfov = self.S["camera"]["hfov_deg"]
        ok_count = 0
        miss = 0
        prev_err = None
        for it in range(A["max_iters"] + A["confirm_frames"] + 2):
            if self.stopped():
                return None, exp, "stopped"
            frame = self.fresh_frame()
            if frame is None:
                continue
            h, w = frame.shape[:2]
            if isinstance(exp, str):          # "center" = ป้ายควรอยู่ใกล้กลางภาพหลังหมุนหยาบ
                exp = (w / 2.0, h / 2.0)
            dets = self.det.detect(frame)
            d = self.find_target(dets, (color, shape, exp))
            if d is None:
                miss += 1
                self.log(f"  [aim] ไม่เห็น {color}/{shape} (รอบ {it}, หายติดกัน {miss})")
                self.state = {"frame_dets": dets}
                if miss >= 3:
                    return None, exp, "lost"
                continue
            miss = 0
            tx, ty = target_point(d, A["aim_y_ratio"])
            hx, hy = self.model.hit_px(w, h, d["dist_cm"], ammo)
            eyaw, epitch = pixel_to_angle(tx - hx, ty - hy, w, hfov)
            err = math.hypot(eyaw, epitch)
            self.state = {"target": d, "hit": (hx, hy), "aim": (tx, ty), "err": (eyaw, epitch), "frame_dets": dets}
            res.update({"iters": res.get("iters", 0) + 1, "dist_cm": d["dist_cm"], "err_yaw": eyaw, "err_pitch": epitch})
            if abs(eyaw) < A["tolerance_deg"] and abs(epitch) < A["tolerance_deg"]:
                ok_count += 1
                if ok_count >= A["confirm_frames"]:
                    self._lock = frame
                    self.log(f"  [aim] {color}/{shape} Z={d['dist_cm']:.0f}cm err=({eyaw:+.2f},{epitch:+.2f})° "
                             f"iters={it + 1} -> locked")
                    return d, (d["cx"], d["cy"]), "ok"
                exp = (d["cx"], d["cy"])
                continue
            ok_count = 0
            # ตรวจทิศการหมุนอัตโนมัติ: ถ้าหมุนแล้ว error แกนนั้นโตขึ้นและเครื่องหมายเดิม -> กลับทิศ
            if prev_err is not None:
                G = self.S["gimbal"]
                for ax, key, e0, e1 in (("yaw", "yaw_sign", prev_err[0], eyaw), ("pitch", "pitch_sign", prev_err[1], epitch)):
                    if abs(e0) > 1.0 and abs(e1) > abs(e0) * 1.3 and e0 * e1 > 0:
                        G[key] *= -1
                        self.log(f"  [aim] ทิศ {ax} กลับด้าน -> {key}={G[key]} (จะบันทึกลง settings)")
            dyaw, dpitch = self._clamped_move(eyaw * A["gain"], epitch * A["gain"])
            prev_err = (eyaw, epitch)
            # คาดตำแหน่งป้ายในภาพหลังหมุน (ช่วยเลือกป้ายเดิมถ้ามีสี/รูปซ้ำ)
            f = focal_px(w, hfov)
            exp = (d["cx"] - f * math.tan(math.radians(dyaw)), d["cy"] + f * math.tan(math.radians(dpitch)))
        self.log(f"  [aim] {color}/{shape} ล็อคไม่สำเร็จ ({res.get('iters')} รอบ)")
        return None, exp, "nolock"
