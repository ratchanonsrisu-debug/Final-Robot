"""
robot_io.py — แหล่งภาพ + gimbal + blaster  (หน้าตาเดียวกันทุกแบบ)
    RealEP     : DJI RoboMaster EP ผ่าน robomaster SDK
    SimEP      : ฉากจำลอง 3 มิติ (ป้ายสี กำแพงสีใกล้เคียง แสงสะท้อน ลำกล้องเยื้องกล้อง) ใช้ทดสอบทั้งระบบ
    FileSource : รูป/โฟลเดอร์รูป/วิดีโอ/เว็บแคม  ใช้ทดสอบและ calibrate สีจากภาพที่แคปไว้
"""
import glob
import math
import os
import threading
import time

import cv2
import numpy as np


class RealEP:
    name = "RoboMaster EP"

    def __init__(self, resolution="720p", log=print):
        from robomaster import robot, blaster, camera
        self._blaster = blaster
        self.log = log
        self.ep = robot.Robot()
        self.ep.initialize(conn_type="ap")
        try:
            self.ep.set_robot_mode(mode=robot.FREE)
        except Exception as e:
            log(f"set_robot_mode: {e}")
        res = {"360p": camera.STREAM_360P, "540p": camera.STREAM_540P}.get(resolution, camera.STREAM_720P)
        self.ep.camera.start_video_stream(display=False, resolution=res)
        self._ang = None
        self.ep.gimbal.sub_angle(freq=20, callback=self._on_angle)
        self.lock = threading.Lock()
        log("เชื่อมต่อ RoboMaster EP แล้ว")

    def _on_angle(self, a):
        self._ang = (a[0], a[1])          # (pitch, yaw) เทียบตัวถัง

    def read_frame(self):
        try:
            return self.ep.camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Exception:
            return None

    def read_frame_seq(self):
        """อ่านทีละเฟรมตามลำดับ ไม่ข้ามเฟรม (ใช้ใน thread grabber เพื่อไม่พลาดแสงแฟลช IR ที่ติดแค่ 1-3 เฟรม)"""
        try:
            return self.ep.camera.read_cv2_image(strategy="pipeline", timeout=0.5)
        except Exception:
            return None

    def gimbal_angles(self):
        return self._ang

    def gimbal_move(self, dpitch, dyaw, speed=120):
        if abs(dpitch) < 0.05 and abs(dyaw) < 0.05:
            return
        with self.lock:
            self.ep.gimbal.move(pitch=float(dpitch), yaw=float(dyaw),
                                pitch_speed=speed, yaw_speed=speed).wait_for_completed()

    def gimbal_moveto(self, pitch, yaw, speed=120):
        with self.lock:
            self.ep.gimbal.moveto(pitch=float(pitch), yaw=float(yaw),
                                  pitch_speed=speed, yaw_speed=speed).wait_for_completed()

    def gimbal_speed(self, pitch_speed, yaw_speed):
        self.ep.gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)

    def recenter(self):
        with self.lock:
            self.ep.gimbal.recenter().wait_for_completed()

    def fire(self, ammo="ir", times=1):
        """times = ยิงรัวกี่นัดในคำสั่งเดียว (SDK รับ 1-8)"""
        ft = self._blaster.INFRARED_FIRE if ammo == "ir" else self._blaster.WATER_FIRE
        self.ep.blaster.fire(fire_type=ft, times=max(1, min(8, int(times))))

    def close(self):
        for f in (lambda: self.ep.gimbal.drive_speed(0, 0), lambda: self.ep.gimbal.unsub_angle(),
                  lambda: self.ep.camera.stop_video_stream(), lambda: self.ep.close()):
            try:
                f()
            except Exception:
                pass


# ======================================================================
def _rot(pitch_deg, yaw_deg):
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    return Ry @ Rx


class SimEP:
    """ฉากจำลอง: X ขวา, Y ลง, Z หน้า (ซม.) จุดหมุน gimbal = จุดกำเนิด"""
    name = "Simulator"
    GEL_SPEED = 2600.0    # cm/s

    def __init__(self, w=960, h=540, hfov=96.0, seed=1, scene=None,
                 barrel_offset=(1.5, 5.5), barrel_angle=(0.7, -0.5), yaw_sign=1,
                 latency_s=0.25, flash_s=0.3, gel_feed_p=1.0):
        self.w, self.h, self.hfov = w, h, hfov
        self.f = (w / 2) / math.tan(math.radians(hfov) / 2)
        self.rnd = np.random.default_rng(seed)
        self.pitch, self.yaw = 0.0, 0.0
        self.yaw_sign = yaw_sign
        self.b_off = np.array([barrel_offset[0], barrel_offset[1], 0.0])     # ลำกล้องอยู่ขวา 1.5 ล่าง 5.5 ซม.
        tx, ty = math.radians(barrel_angle[0]), math.radians(barrel_angle[1])
        d = np.array([math.tan(tx), -math.tan(ty), 1.0]); self.b_dir = d / np.linalg.norm(d)
        self.scene = scene or self.default_scene()
        self.wall_z = 180.0
        self.hits = []
        self._ir_from = self._ir_until = 0.0
        self._ir_pt = None
        self.latency_s, self.flash_s = latency_s, flash_s   # delay ของ video stream + ระยะเวลาที่แสงติด
        self.gel_feed_p = gel_feed_p      # โอกาสที่สั่งยิงเจลแล้วกระสุนออกจริง (หุ่นจริง ~1/3)
        self.triggers = 0
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        self._rays_cam = np.stack([(u - w / 2) / self.f, (v - h / 2) / self.f, np.ones_like(u, float)], -1)

    @staticmethod
    def default_scene():
        # (สี, รูปทรง, x, y, z) ป้ายหันหน้าเข้าหากล้อง ขนาดตามจริง
        return [
            ("BLUE", "CIRCLE", -38, -2, 110),
            ("RED", "HORIZONTAL", -12, 3, 120),
            ("YELLOW", "VERTICAL", 14, 0, 100),
            ("GREEN", "SQUARE", 40, 4, 130),
            ("RED", "VERTICAL", 70, -3, 150),
        ]

    COLORS = {"RED": (30, 25, 215), "YELLOW": (20, 225, 240), "GREEN": (60, 170, 20), "BLUE": (165, 95, 35)}
    SIZES = {"VERTICAL": (6, 9), "HORIZONTAL": (9, 6), "SQUARE": (7, 7), "CIRCLE": (7, 7)}

    def _inside(self, shape, lx, ly):
        sw, sh = self.SIZES[shape]
        if shape == "CIRCLE":
            return lx ** 2 + ly ** 2 <= (sw / 2) ** 2
        return (np.abs(lx) <= sw / 2) & (np.abs(ly) <= sh / 2)

    def read_frame(self):
        time.sleep(0.01)
        R = _rot(self.pitch, self.yaw)
        rays = self._rays_cam @ R.T
        dz = np.maximum(rays[..., 2], 1e-6)
        # กำแพง: สีส้มอ่อน/ครีม มีลายกระเบื้อง (ให้ hue ใกล้แดง/เหลือง เพื่อทดสอบการแยกป้ายจากกำแพง)
        t = self.wall_z / dz
        X = rays[..., 0] * t; Y = rays[..., 1] * t
        img = np.zeros((self.h, self.w, 3), np.float32)
        base = np.array([120, 170, 215], np.float32)                  # BGR ครีมอมส้ม
        tile = (((np.floor(X / 30) + np.floor(Y / 30)) % 2) * 12)[..., None]
        img[:] = base
        img += tile
        img[Y > 25] = (95, 105, 110)                                   # พื้น
        zbuf = np.full((self.h, self.w), 1e9)
        for color, shape, sx, sy, sz in self.scene:
            t = sz / dz
            lx = rays[..., 0] * t - sx; ly = rays[..., 1] * t - sy
            m = self._inside(shape, lx, ly) & (t < zbuf)
            # เสาไม้ (สีขาวใส) ใต้ป้าย
            stick = (np.abs(lx) < 1.0) & (ly > self.SIZES[shape][1] / 2) & (ly < self.SIZES[shape][1] / 2 + 15) & (t < zbuf)
            img[stick] = (225, 225, 220)
            col = np.array(self.COLORS[color], np.float32)
            # แสงสะท้อนบนอะคริลิก: แถบเฉียงสว่าง
            glare = m & (np.abs(lx * 0.8 + ly - self.SIZES[shape][1] * 0.1) < 0.9)
            shade = 0.85 + 0.15 * (ly / 10.0)
            img[m] = col * np.clip(shade[m], 0.7, 1.1)[:, None]
            img[glare] = (245, 248, 250)
            zbuf[m] = t[m]
        # จุด IR
        if self._ir_pt is not None and self._ir_from <= time.time() < self._ir_until:
            pc = R.T @ self._ir_pt
            if pc[2] > 1:
                u = self.f * pc[0] / pc[2] + self.w / 2; v = self.f * pc[1] / pc[2] + self.h / 2
                cv2.circle(img, (int(u), int(v)), 4, (255, 120, 255), -1)
                cv2.circle(img, (int(u), int(v)), 2, (255, 255, 255), -1)
        img += self.rnd.normal(0, 4, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def gimbal_angles(self):
        return (self.pitch, self.yaw)

    def gimbal_move(self, dpitch, dyaw, speed=120):
        time.sleep(0.02)
        self.pitch += dpitch * (1 + self.rnd.normal(0, 0.03)) + self.rnd.normal(0, 0.04)
        self.yaw += self.yaw_sign * dyaw * (1 + self.rnd.normal(0, 0.03)) + self.rnd.normal(0, 0.04)

    def gimbal_moveto(self, pitch, yaw, speed=120):
        self.pitch, self.yaw = pitch, yaw * self.yaw_sign

    def gimbal_speed(self, pitch_speed, yaw_speed):
        self.pitch += pitch_speed * 0.05
        self.yaw += yaw_speed * 0.05 * self.yaw_sign

    def recenter(self):
        self.pitch = self.yaw = 0.0

    def _shot_point(self, ammo):
        R = _rot(self.pitch, self.yaw)
        o = R @ self.b_off; d = R @ self.b_dir
        best = None
        planes = [(sz, (c, s, sx, sy)) for c, s, sx, sy, sz in self.scene] + [(self.wall_z, None)]
        for z, info in sorted(planes, key=lambda p: p[0]):
            t = (z - o[2]) / d[2]
            p = o + d * t
            if ammo == "gel":
                tf = np.linalg.norm(p - o) / self.GEL_SPEED
                p = p + np.array([0, 0.5 * 981 * tf * tf, 0])
            if info is None:
                return p, None
            c, s, sx, sy = info
            if self._inside(s, p[0] - sx, p[1] - sy):
                return p, (c, s)
        return best, None

    def fire(self, ammo="ir", times=1):
        """times = ยิงรัว: แต่ละนัดมีโอกาสกระสุนออก gel_feed_p แยกกัน"""
        self.triggers += 1
        hit = None
        for _ in range(max(1, int(times))):
            h = self._fire_one(ammo)
            hit = hit or h
        return hit

    def _fire_one(self, ammo):
        if ammo == "gel" and self.rnd.random() > self.gel_feed_p:
            return None                    # กระสุนไม่ออก
        p, hit = self._shot_point(ammo)
        self.hits.append(hit)
        if ammo == "gel" and hit is not None:     # ป้ายโดนเจลแล้วล้ม
            for i, (c, sh, sx, sy, sz) in enumerate(self.scene):
                if (c, sh) == hit and abs(p[0] - sx) < 6 and abs(p[1] - sy) < 6:
                    self.scene.pop(i)
                    break
        self._ir_pt = p
        self._ir_from = time.time() + self.latency_s
        self._ir_until = self._ir_from + self.flash_s
        return hit

    def close(self):
        pass


# ======================================================================
class FileSource:
    """ภาพนิ่ง / โฟลเดอร์ / วิดีโอ / เว็บแคม (ไม่มี gimbal)"""
    name = "File"

    def __init__(self, src):
        self.images, self.idx, self.cap = [], 0, None
        if isinstance(src, int) or (isinstance(src, str) and src.isdigit()):
            self.cap = cv2.VideoCapture(int(src))
        elif os.path.isdir(src):
            self.images = sorted(sum([glob.glob(os.path.join(src, e)) for e in ("*.jpg", "*.png", "*.jpeg", "*.bmp")], []))
        elif src.lower().endswith((".mp4", ".avi", ".mov", ".mkv")):
            self.cap = cv2.VideoCapture(src)
        else:
            self.images = [src]
        self._cur = None
        self.name = f"File: {os.path.basename(str(src))}"

    def next_image(self, step=1):
        if self.images:
            self.idx = (self.idx + step) % len(self.images)
            self._cur = None

    def read_frame(self):
        time.sleep(0.03)
        if self.cap is not None:
            ok, f = self.cap.read()
            if not ok:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, f = self.cap.read()
            return f if ok else None
        if not self.images:
            return None
        if self._cur is None:
            self._cur = cv2.imread(self.images[self.idx])
        return None if self._cur is None else self._cur.copy()

    def gimbal_angles(self):
        return None

    def gimbal_move(self, *a, **k):
        pass

    gimbal_moveto = gimbal_speed = gimbal_move

    def recenter(self):
        pass

    def fire(self, ammo="ir", times=1):
        pass

    def close(self):
        if self.cap is not None:
            self.cap.release()


# ======================================================================
class Grabbed:
    """ห่อแหล่งภาพ: thread เดียวอ่านทุกเฟรมต่อเนื่อง ทุกส่วนอ่านจากที่นี่ (กันแย่งเฟรม)
    + โหมดบันทึก (record) เก็บทุกเฟรมพร้อมเวลา ใช้จับแสง IR ที่กระพริบแวบเดียว"""

    def __init__(self, src):
        self.src = src
        self.latest, self.t = None, 0.0
        self.freeze = False
        self._run = True
        self._cv = threading.Condition()
        self._rec = None
        self._hist = []          # เฟรมล่าสุดไม่กี่เฟรม (ใช้เป็นภาพพื้นหลังก่อนยิง)
        self._reader = getattr(src, "read_frame_seq", None) or src.read_frame
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._run:
            if self.freeze and self.latest is not None and self._rec is None:
                time.sleep(0.05)
                with self._cv:
                    self.t = time.time(); self._cv.notify_all()
                continue
            f = self._reader()
            if f is None:
                time.sleep(0.005)
                continue
            now = time.time()
            with self._cv:
                self.latest, self.t = f, now
                self._hist.append((now, f))
                if len(self._hist) > 8:
                    self._hist.pop(0)
                if self._rec is not None:
                    self._rec.append((now, f))
                self._cv.notify_all()

    def peek(self):
        return self.latest

    def read_frame(self):
        """รอเฟรมที่ใหม่กว่าเวลาที่เรียก (ภาพหลัง gimbal หยุดจริง)"""
        t0 = time.time()
        with self._cv:
            self._cv.wait_for(lambda: self.t > t0, timeout=1.5)
            return None if self.latest is None else self.latest.copy()

    def recent(self, n=6):
        with self._cv:
            return [f for _, f in self._hist[-n:]]

    def start_record(self):
        with self._cv:
            self._rec = []

    def stop_record(self):
        with self._cv:
            r, self._rec = self._rec or [], None
        return r

    def __getattr__(self, k):
        return getattr(self.src, k)

    def close(self):
        self._run = False
        self.src.close()
