"""
shooter.py — เชื่อม "ตรวจป้าย + เล็งยิง" (vision.py / aiming.py จาก Colorshoot) เข้ากับ SLAM explorer

  RealShooterIO : กล้อง + gimbal + blaster ของหุ่นจริง ใช้การเชื่อมต่อเดียวกับ RealRobot
                  (ห้ามเปิด robot.Robot() ซ้ำ - หุ่นรับได้ทีละ connection)
  SimShooterIO  : กล้องจำลอง เรนเดอร์ป้ายในเขาวงกตจำลองตามตำแหน่งหุ่น/มุม gimbal (ทดสอบไม่ต้องต่อหุ่น)
  TargetHunter  : ทุกครั้งที่ scan_all() หัน gimbal ไปอ่าน ToF ทิศไหน -> ถ่ายภาพหาป้ายทิศนั้นด้วย (look)
                  สแกนช่องเสร็จ -> ป้ายที่ตรงสเปค + อยู่ในระยะยิง -> หันไปดูซ้ำ ต้องเห็นหลายเฟรม
                  (stability gate) -> Aimer เล็ง -> ยิง ; ตำแหน่งป้ายเก็บเป็นพิกัดโลก (m) ลงแผนที่
  run_round2    : รอบสอง - ใช้แผนที่ + ตำแหน่งเป้าจากรอบแรก เดินไปยิงทีละเป้า

พิกัด: มุมโลกวัดตามเข็มนาฬิกาจากทิศเหนือ (N=0, E=90) ; ภาพ: bearing + = ขวาของกล้อง
"""
import copy
import csv
import math
import os
import threading
import time

import numpy as np

import config as C
import settings as ST
from aiming import Aimer
from grid_map import DIRS, DX, DY, WALL
from vision import ASPECT_HORZ, ASPECT_VERT, SignDetector, Tracker

SHAPES = ("VERTICAL", "HORIZONTAL", "SQUARE", "CIRCLE")
COLORS = ("RED", "YELLOW", "GREEN", "BLUE")


def parse_spec(text):
    """"RED:CIRCLE, BLUE:SQUARE, GREEN" -> [(สี, รูปทรง|None), ...]  (ว่าง = ใช้ settings.json targets)"""
    out = []
    for tok in (text or "").replace(";", ",").split(","):
        tok = tok.strip().upper()
        if not tok:
            continue
        c, _, s = tok.partition(":")
        c, s = c.strip(), s.strip()
        if c not in COLORS:
            raise ValueError(f"ไม่รู้จักสี '{c}' (ใช้ {', '.join(COLORS)})")
        if s in ("", "*", "ANY"):
            s = None
        elif s not in SHAPES:
            raise ValueError(f"ไม่รู้จักรูปทรง '{s}' (ใช้ {', '.join(SHAPES)})")
        out.append((c, s))
    return out


# ======================================================================
# IO ของหุ่นจริง
# ======================================================================
class RealShooterIO:
    """หน้าตาเดียวกับ RealEP ของ Colorshoot (read_frame/gimbal_move/gimbal_angles/fire) แต่ใช้ ep ของ RealRobot"""

    def __init__(self, robot, resolution=None, log=print):
        from robomaster import blaster, camera
        self.rb = robot
        self.ep = robot._ep
        self._blaster = blaster
        self.log = log
        resolution = resolution or C.CAM_RESOLUTION
        before = robot.rates(1.0)
        res = {"360p": camera.STREAM_360P, "540p": camera.STREAM_540P}.get(resolution, camera.STREAM_720P)
        self._res = res
        self.iso = None
        if C.CAM_ISOLATE:
            try:
                self.iso = _IsoCam(self.ep, res, log)
            except Exception as e:                          # noqa: BLE001
                log(f"[camera] เปิดตัวถอดรหัสแยก process ไม่สำเร็จ ({e!r}) -> ใช้แบบเดิมของ SDK")
                self.iso = None
        if self.iso is None:
            self.ep.camera.start_video_stream(display=False, resolution=res)
        self.latest, self.t = None, 0.0
        self._miss = 0               # read_frame ติดกันกี่ครั้งที่ไม่มีภาพใหม่
        self._run = True
        self._cv = threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()
        time.sleep(1.0)
        after = robot.rates(1.0)
        log(f"[shooter] เปิดกล้อง {resolution} แล้ว")
        # ตรวจว่าวิดีโอทำให้ข้อมูลเซนเซอร์ช้าลงไหม (ToF/IMU/odometry/มุม gimbal: ครั้ง/วินาที)
        log("[diag] Hz ก่อนเปิดกล้อง " + " ".join(f"{k}={v:.0f}" for k, v in before.items())
            + " | หลังเปิด " + " ".join(f"{k}={v:.0f}" for k, v in after.items()))
        if after["tof"] < 0.6 * max(before["tof"], 1):
            log("[diag] !! ToF มาช้าลงมากหลังเปิดกล้อง - ลองตั้ง CAM_RESOLUTION = \"540p\" ใน config.py")

    def _loop(self):
        # thread เดียวอ่านทุกเฟรมต่อเนื่อง (เหมือน Grabbed ใน Colorshoot) ทุกส่วนอ่านจากที่นี่
        if self.iso is not None:
            self._loop_iso()
            return
        while self._run:
            try:
                f = self.ep.camera.read_cv2_image(strategy="pipeline", timeout=0.5)
            except Exception:
                f = None
            if f is None:
                time.sleep(0.005)
                continue
            with self._cv:
                self.latest, self.t = f, time.time()
                self._cv.notify_all()

    def _loop_iso(self):
        last = 0.0
        while self._run:
            try:
                r = self.iso.grab(last)
            except Exception:                               # noqa: BLE001
                r = None
            if r is None:
                if not self.iso.alive():
                    self.iso.restart()                      # ตัวถอดรหัสพัง -> เปิดใหม่ (โปรแกรมหลักไม่ตาย)
                time.sleep(0.005)
                continue
            last, f = r
            with self._cv:
                self.latest, self.t = f, time.time()
                self._cv.notify_all()

    def peek(self):
        return self.latest

    def read_frame(self):
        """รอเฟรมที่มาถึงหลังเวลาที่เรียก (ภาพหลัง gimbal หยุดจริง) ; ไม่มีภาพใหม่ -> None (ห้ามคืนภาพเก่า:
        ภาพค้างจากที่อื่น = หาป้ายไม่เจอ/ได้ตำแหน่งป้ายผิดช่อง) ; ไม่มีภาพใหม่ 2 ครั้งติด -> เปิดสตรีมใหม่"""
        t0 = time.time()
        with self._cv:
            fresh = self._cv.wait_for(lambda: self.t > t0, timeout=1.5)
            f = None if (self.latest is None or not fresh) else self.latest.copy()
        if f is not None:
            self._miss = 0
            return f
        self._miss += 1
        self.log(f"[camera] !! ไม่มีภาพใหม่จากกล้อง 1.5 วิ (ครั้งที่ {self._miss})")
        if self._miss >= 2:
            self._restart_stream()
        return None

    def _restart_stream(self):
        self.log("[camera] ภาพค้าง -> ปิดแล้วเปิดสตรีมกล้องใหม่")
        if self.iso is not None:
            self.iso.restart(force=True)
            self._miss = 0
            time.sleep(1.0)
            return
        try:
            self.ep.camera.stop_video_stream()
        except Exception:
            pass
        time.sleep(0.5)
        try:
            self.ep.camera.start_video_stream(display=False, resolution=self._res)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[camera] เปิดสตรีมใหม่ไม่สำเร็จ: {e}")
        self._miss = 0
        time.sleep(1.0)

    # ---- gimbal (ให้ Aimer ใช้) ----
    def gimbal_angles(self):
        return (self.rb._gimbal_pitch, self.rb._gimbal_yaw)

    def gimbal_move(self, dpitch, dyaw, speed=120):
        if abs(dpitch) < 0.05 and abs(dyaw) < 0.05:
            return
        try:
            self.ep.gimbal.move(pitch=float(dpitch), yaw=float(dyaw),
                                pitch_speed=speed, yaw_speed=speed).wait_for_completed(timeout=3.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[shooter] gimbal.move ล้มเหลว: {e}")

    def fire(self, ammo="ir", times=1):
        ft = self._blaster.INFRARED_FIRE if ammo == "ir" else self._blaster.WATER_FIRE
        self.ep.blaster.fire(fire_type=ft, times=max(1, min(8, int(times))))

    # ---- ใช้โดย TargetHunter ----
    def looking(self, d):
        """scan_all() หัน gimbal ไปทิศ d ให้แล้ว - หุ่นจริงไม่ต้องทำอะไร"""

    def point_dir(self, d):
        rel = (d - self.rb.body_heading) % 4
        pitch = C.GIMBAL_PITCH_SIDE if rel in (1, 3) else C.GIMBAL_PITCH
        self.rb._goto_gimbal(self.rb._gimbal_yaw_for(d), pitch)
        time.sleep(C.TOF_SETTLE_S)

    def point_angle(self, world_deg, pitch):
        """หัน gimbal ไปมุมโลก world_deg (ตามเข็มจากเหนือ ; ไม่ต้องเป็นทิศหลัก) ก้ม/เงย pitch"""
        rel = (world_deg - self.rb.body_heading * 90.0 + 180.0) % 360.0 - 180.0
        yaw = self.rb.gimbal_yaw_sign * rel * (1 if C.GIMBAL_RIGHT_YAW > 0 else -1)
        # สั่งครั้งเดียว ไม่ดันซ้ำ: ก้มแล้วหันไปด้านหลังเฉียง gimbal ไปติด adaptor (log 1/10: สั่ง -135 ได้ -120)
        ok = self.rb._goto_gimbal(yaw, pitch, attempts=1, extra_wait_s=0.8)
        time.sleep(C.TOF_SETTLE_S)
        return ok

    def cam_world_deg(self, d=None):
        """มุมโลกที่กล้องชี้ (ตามเข็ม จากเหนือ) ; d = ทิศที่สั่ง gimbal ไว้ (ยังไม่เล็งขยับ) -> ใช้มุมตามคำสั่ง
        (มุมที่อ่านจาก sub_angle อาจค้างเมื่อข้อมูลมาช้า -> เคยทำให้ตำแหน่งป้ายผิดช่อง)
        d = None (หลังเล็งขยับ gimbal เอง) -> รอค่ามุมใหม่ แล้วใช้มุมที่วัดจริง"""
        if d is not None:
            return d * 90.0
        self.rb._wait_fresh("_ang_n", 2, 0.5)
        # มุมที่วัดจริง (sub_angle) ฝั่งขวา = GIMBAL_RIGHT_YAW เสมอ (ดู _gimbal_yaw_for / gimbal_yaw_sign)
        cw = self.rb._gimbal_yaw * (1 if C.GIMBAL_RIGHT_YAW > 0 else -1)
        return self.rb.body_heading * 90.0 + cw

    def cam_xy(self):
        return self.rb.est_xy()

    def close(self):
        self._run = False
        if self.iso is not None:
            self.iso.close()
            return
        try:
            self.ep.camera.stop_video_stream()
        except Exception:
            pass


class _IsoCam:
    """เปิดสตรีมกล้องด้วยคำสั่ง SDK แต่ถอดรหัสภาพใน process แยก (camproc.py) ส่งภาพผ่าน shared memory
    ตัวถอดรหัส (ffmpeg) พัง = ตายแค่ process ลูก -> เปิดใหม่ ; โปรแกรมหลักยังคุมล้อ/หยุดหุ่นได้"""
    HDR = 32

    def __init__(self, ep, res, log):
        from multiprocessing import shared_memory
        import subprocess
        import sys
        self._sp, self._py = subprocess, sys.executable
        self.ep, self.res, self.log = ep, res, log
        self.shm = shared_memory.SharedMemory(create=True, size=self.HDR + 1920 * 1080 * 3)
        self.hdr = np.ndarray((4,), dtype=np.float64, buffer=self.shm.buf[:self.HDR])
        self.hdr[:] = 0.0
        self.addr = ep.camera.video_stream_addr
        self._script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camproc.py")
        self._errf = open(os.path.join(os.path.dirname(self._script), "output", "camproc_stderr.log"), "a",
                          encoding="utf-8")
        self.proc, self.n_restart, self._t_spawn = None, 0, 0.0
        self._stream(True)
        self._spawn()

    def _stream(self, on):
        cam = self.ep.camera
        if on:
            if not cam._stream_sdk(1, self.res) or not cam._video_stream(1, self.res):
                raise RuntimeError("สั่งเปิดสตรีมกล้องไม่สำเร็จ")
        else:
            try:
                cam._video_stream(0)
                cam._stream_sdk(0)
            except Exception:                               # noqa: BLE001
                pass

    def _spawn(self):
        self._errf.write(f"--- {time.strftime('%H:%M:%S')} start camproc ---\n")
        self._errf.flush()
        self.proc = self._sp.Popen([self._py, self._script, str(self.addr[0]), str(self.addr[1]), self.shm.name,
                                    str(os.getpid())], stdout=self._sp.DEVNULL, stderr=self._errf,
                                   creationflags=getattr(self._sp, "CREATE_NO_WINDOW", 0))
        self._t_spawn = time.time()
        self._seq_spawn = float(self.hdr[0])

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def restart(self, force=False):
        """ตัวลูกตาย -> เปิดใหม่ ; เปิดแล้วไม่ได้ภาพสักเฟรมติดกันหลายครั้ง (หุ่นปิด/ต่อวิดีโอไม่ได้) -> ลองห่างขึ้น
        และ log นานๆ ครั้ง (log 1/10: หลังปิดหุ่น log กล้องรัวเป็นร้อยบรรทัด)"""
        now = time.time()
        fails = getattr(self, "_fails", 0)
        if not force and now - self._t_spawn < (2.0 if fails < 6 else 15.0):
            return                                          # เว้นระยะ ไม่เปิดถี่
        got = float(self.hdr[0]) != getattr(self, "_seq_spawn", -1.0)
        self._fails = 0 if got else fails + 1
        code = self.proc.poll() if self.proc is not None else None
        if self.alive():
            self.proc.kill()
            self.proc.wait(timeout=2)
        self.n_restart += 1
        loud = self._fails < 6 or self._fails % 20 == 0
        if loud:
            self.log(f"[camera] ตัวถอดรหัสภาพ (process แยก) {'หยุด code ' + str(code) if code is not None else 'ถูกสั่งรีสตาร์ท'}"
                     f" -> เปิดใหม่ (ครั้งที่ {self.n_restart}"
                     f"{f', ไม่ได้ภาพติดกัน {self._fails} ครั้ง: หุ่นปิด/หลุด? ลองทุก 15 วิ' if self._fails >= 6 else ''})"
                     f" โปรแกรมหลักทำงานต่อ")
        if self._fails in (3, 6) or (self._fails > 6 and self._fails % 20 == 0):
            # เปิดใหม่หลายครั้งยังไม่ได้ภาพ -> สั่งหุ่นปิด/เปิดสตรีมด้วย
            self._stream(False)
            time.sleep(0.3)
            try:
                self._stream(True)
            except Exception as e:                          # noqa: BLE001
                if loud:
                    self.log(f"[camera] เปิดสตรีมใหม่ไม่สำเร็จ: {e}")
        self._spawn()

    def grab(self, last):
        """ภาพใหม่ (seq != last) -> (seq, ภาพ BGR copy) ; ไม่มี/กำลังเขียน -> None"""
        s1 = float(self.hdr[0])
        if s1 == last or s1 <= 0 or int(s1) % 2:
            return None
        w, h = int(self.hdr[1]), int(self.hdr[2])
        f = np.frombuffer(self.shm.buf, dtype=np.uint8, count=w * h * 3, offset=self.HDR).reshape(h, w, 3).copy()
        if float(self.hdr[0]) != s1:
            return None                                     # ถูกเขียนทับระหว่าง copy -> รอเฟรมถัดไป
        return s1, f

    def close(self):
        if self.alive():
            self.proc.kill()
        self._stream(False)
        try:
            del self.hdr
            self.shm.close()
            self.shm.unlink()
        except Exception:                                   # noqa: BLE001
            pass


# ======================================================================
# IO จำลอง
# ======================================================================
def _rot(pitch_deg, yaw_deg):
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    return Ry @ Rx


class _SimCam:
    """ย่อจาก SimEP ของ Colorshoot: X ขวา, Y ลง, Z หน้า (ซม.) จุดหมุน gimbal = จุดกำเนิด
    ป้ายทุกใบหันหน้าเข้าหากล้อง, ลำกล้องเยื้องตาม hit_model เจลที่ calibrate ไว้ (ซ้าย 1.0 ต่ำ 2.75 ซม.)"""
    GEL_SPEED = 2600.0
    COLORS = {"RED": (30, 25, 215), "YELLOW": (20, 225, 240), "GREEN": (60, 170, 20), "BLUE": (165, 95, 35)}
    SIZES = {"VERTICAL": (6, 9), "HORIZONTAL": (9, 6), "SQUARE": (7, 7), "CIRCLE": (7, 7)}

    def __init__(self, w=960, h=540, hfov=96.0, seed=1):
        self.w, self.h = w, h
        self.f = (w / 2) / math.tan(math.radians(hfov) / 2)
        self.rnd = np.random.default_rng(seed)
        self.pitch = self.yaw = 0.0
        self.b_off = np.array([-1.0, 2.75, 0.0])
        self.b_dir = np.array([0.0, 0.0, 1.0])
        self.scene = []
        self.wall_z = 400.0
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        self._rays = np.stack([(u - w / 2) / self.f, (v - h / 2) / self.f, np.ones_like(u, float)], -1)

    def _inside(self, shape, lx, ly, kx=1.0):
        """kx = ป้ายหดแนวนอนเท่านี้ (มองเฉียง: cos มุมระหว่างแนวมองกับหน้าป้าย)"""
        lx = lx / max(kx, 0.05)
        sw, sh = self.SIZES[shape]
        if shape == "CIRCLE":
            return lx ** 2 + ly ** 2 <= (sw / 2) ** 2
        return (np.abs(lx) <= sw / 2) & (np.abs(ly) <= sh / 2)

    def render(self):
        R = _rot(self.pitch, self.yaw)
        rays = self._rays @ R.T
        dz = np.maximum(rays[..., 2], 1e-6)
        t = self.wall_z / dz
        X = rays[..., 0] * t; Y = rays[..., 1] * t
        img = np.zeros((self.h, self.w, 3), np.float32)
        img[:] = (200, 200, 195)                                    # โฟมขาว
        img += (((np.floor(X / 60) + np.floor(Y / 60)) % 2) * 10)[..., None]
        img[Y > 20] = (95, 105, 110)                                # พื้น
        zbuf = np.full((self.h, self.w), 1e9)
        for color, shape, sx, sy, sz, kx in self.scene:
            t = sz / dz
            lx = rays[..., 0] * t - sx; ly = rays[..., 1] * t - sy
            m = self._inside(shape, lx, ly, kx) & (t < zbuf) & (rays[..., 2] > 0.05)
            img[m] = np.array(self.COLORS[color], np.float32)
            zbuf[m] = t[m]
        img += self.rnd.normal(0, 4, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def shoot(self, ammo):
        """-> index ใน scene ที่โดน หรือ None"""
        R = _rot(self.pitch, self.yaw)
        o = R @ self.b_off; d = R @ self.b_dir
        for i, (c, s, sx, sy, sz, kx) in sorted(enumerate(self.scene), key=lambda e: e[1][4]):
            tt = (sz - o[2]) / d[2]
            if tt <= 0:
                continue
            p = o + d * tt
            if ammo == "gel":
                tf = np.linalg.norm(p - o) / self.GEL_SPEED
                p = p + np.array([0, 0.5 * 981 * tf * tf, 0])
            if self._inside(s, p[0] - sx, p[1] - sy, kx):
                return i
        return None


class SimShooterIO:
    """กล้องจำลองติดกับ SimRobot: ป้ายในโลก = [{"color","shape","x","y"}] (เมตร) ; มองทะลุกำแพงไม่ได้"""

    def __init__(self, robot, world, targets, log=print, seed=1):
        self.rb, self.world, self.targets = robot, world, targets
        self.log = log
        self.cam = _SimCam(seed=seed)
        self.gyaw = 0.0            # มุม gimbal เทียบตัวถัง (+ = ขวา)
        self.gpitch = 0.0
        self.shots = []

    def _prepare(self):
        """ตั้ง scene ของกล้องตามตำแหน่งหุ่น (ป้ายทุกใบหันหน้าเข้าหากล้อง ใช้กรอบที่หมุนตามทิศ gimbal หยาบ)"""
        rx, ry = self.rb.est_xy()
        hb = self.rb.body_heading
        base = round(self.gyaw / 90.0) * 90.0
        a = math.radians(base)
        scene, ids = [], []
        for i, t in enumerate(self.targets):
            if t.get("knocked"):
                continue
            vx, vy = t["x"] - rx, t["y"] - ry
            f = vx * DX[hb] + vy * DY[hb]
            r = vx * DX[(hb + 1) % 4] + vy * DY[(hb + 1) % 4]
            Z = f * math.cos(a) + r * math.sin(a)
            X = -f * math.sin(a) + r * math.cos(a)
            if Z < 0.15 or not los_clear(self.world, rx, ry, t["x"], t["y"]):
                continue
            inc = wall_incidence(self.world, t["x"], t["y"], rx, ry)     # ป้ายติดกำแพง: มองเฉียง = แคบลง
            kx = math.cos(math.radians(inc)) if inc is not None else 1.0
            scene.append((t["color"], t["shape"], X * 100, 4.0, Z * 100, kx))
            ids.append(i)
        self.cam.scene = scene
        self.cam.yaw = self.gyaw - base
        self.cam.pitch = self.gpitch
        return ids

    def read_frame(self):
        self._prepare()
        return self.cam.render()

    def peek(self):
        return None                 # sim: หน้าต่างกล้องใช้ภาพที่ตรวจล่าสุดแทน

    def gimbal_angles(self):
        return (self.gpitch, self.gyaw)

    def gimbal_move(self, dpitch, dyaw, speed=120):
        self.gpitch += dpitch
        self.gyaw += dyaw

    def fire(self, ammo="ir", times=1):
        ids = self._prepare()
        k = self.cam.shoot(ammo)
        hit = None if k is None else self.targets[ids[k]]
        c = self.world.cell_m
        tc = None if hit is None else (int(hit["x"] // c), int(hit["y"] // c))
        self.shots.append({"ammo": ammo, "hit": None if hit is None else (hit["color"], hit["shape"]),
                           "from": tuple(self.rb.cell), "target_cell": tc})
        if hit is not None and ammo == "gel":
            hit["knocked"] = True
        return hit

    def looking(self, d):
        self.point_dir(d)

    def point_dir(self, d):
        self.gyaw = {0: 0.0, 1: 90.0, 2: 180.0, 3: -90.0}[(d - self.rb.body_heading) % 4]
        self.gpitch = 0.0

    def point_angle(self, world_deg, pitch):
        self.gyaw = (world_deg - self.rb.body_heading * 90.0 + 180.0) % 360.0 - 180.0
        self.gpitch = pitch
        return True

    def cam_world_deg(self, d=None):
        return d * 90.0 if d is not None else self.rb.body_heading * 90.0 + self.gyaw

    def cam_xy(self):
        return self.rb.est_xy()

    def close(self):
        pass


def random_sim_targets(world, n=6, avoid=(), seed=None):
    """วางป้ายสุ่มในเขาวงกตจำลองให้คล้ายสนามจริง: แถวป้าย 3 ใบชิดกำแพงในช่องเดียว (คนละแบบ)
    + ที่เหลือวางกลางช่องหรือชิดกำแพง ช่องละ 1 ใบ (ไม่วางในช่อง avoid)"""
    import random
    rnd = random.Random(seed)
    c = world.cell_m
    cells = [(x, y) for x in range(world.w) for y in range(world.h) if (x, y) not in avoid]
    rnd.shuffle(cells)
    out = []

    def sign():
        return {"color": rnd.choice(COLORS), "shape": rnd.choice(SHAPES)}

    def along_wall(x, y, d, u):
        """จุดชิดกำแพงทิศ d ของช่อง (x,y) ห่างกำแพง 6 ซม. ; u = ตำแหน่งตามแนวกำแพง 0..1"""
        cx, cy = (x + 0.5) * c, (y + 0.5) * c
        k = 0.5 * c - 0.06
        if d in (0, 2):
            return (x + u) * c, cy + (k if d == 0 else -k)
        return cx + (k if d == 1 else -k), (y + u) * c

    row_done = False
    for x, y in cells:
        if len(out) >= n:
            break
        walls = [d for d in range(4) if world.state(x, y, d) == WALL]
        if not row_done and walls and n - len(out) >= 3:
            d = rnd.choice(walls)
            kinds = set()
            for u in (0.22, 0.5, 0.78):             # 3 ใบห่างกัน ~17 ซม. คนละแบบ
                while True:
                    k = sign()
                    if (k["color"], k["shape"]) not in kinds:
                        break
                kinds.add((k["color"], k["shape"]))
                k["x"], k["y"] = along_wall(x, y, d, u)
                out.append(k)
            row_done = True
            continue
        k = sign()
        if walls and rnd.random() < 0.5:
            k["x"], k["y"] = along_wall(x, y, rnd.choice(walls), rnd.uniform(0.3, 0.7))
        else:
            k["x"], k["y"] = (x + rnd.uniform(0.4, 0.6)) * c, (y + rnd.uniform(0.4, 0.6)) * c
        out.append(k)
    return out


# ======================================================================
# กติกาการยิงบนกริด
# ======================================================================
def los_clear(m, x0, y0, x1, y1):
    """เส้นตรงจาก (x0,y0) ถึง (x1,y1) (เมตร) ไม่ผ่านขอบที่แผนที่ m บอกว่าเป็นกำแพง"""
    c = m.cell_m
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.03))
    px, py = int(x0 // c), int(y0 // c)
    for i in range(1, n + 1):
        x = x0 + (x1 - x0) * i / n; y = y0 + (y1 - y0) * i / n
        cx, cy = int(x // c), int(y // c)
        if (cx, cy) == (px, py):
            continue
        if not m.in_bounds(cx, cy) or not m.in_bounds(px, py):
            return False
        if abs(cx - px) + abs(cy - py) == 1:
            dd = [k for k in range(4) if px + DX[k] == cx and py + DY[k] == cy][0]
            if m.state(px, py, dd) == WALL:
                return False
        else:                                   # ข้ามมุมพอดี: ตรวจทั้งสองแกน
            ddx = 1 if cx > px else 3
            ddy = 0 if cy > py else 2
            if m.state(px, py, ddx) == WALL or m.state(cx, py, ddy) == WALL:
                return False
        px, py = cx, cy
    return True


def _wall_at(m, cx, cy, d):
    """ขอบทิศ d ของช่อง (cx,cy) เป็นกำแพง (รวมขอบสนาม)"""
    if not m.in_bounds(cx, cy):
        return False
    return m.state(cx, cy, d) == WALL


def wall_incidence(m, x, y, px, py):
    """มุม (องศา) ระหว่างแนวมองจากจุด (px,py) ไปป้ายที่ (x,y) กับเส้นตั้งฉากของกำแพงที่ป้ายติดอยู่
    (หากำแพงในแผนที่ที่อยู่ห่างป้าย ≤ WALL_SIGN_TOL_M) ; 0 = มองหน้าตรง ; None = ไม่รู้ว่าติดกำแพงไหน"""
    c = m.cell_m
    tol = C.WALL_SIGN_TOL_M
    cands = []
    k = int(round(y / c))                                   # กำแพงแนวนอน y = k*c
    if abs(y - k * c) <= tol:
        cx = int(x // c)
        if _wall_at(m, cx, k - 1, 0) or _wall_at(m, cx, k, 2):
            cands.append((abs(y - k * c), math.degrees(math.atan2(abs(px - x), abs(py - y) + 1e-9))))
    k = int(round(x / c))                                   # กำแพงแนวตั้ง x = k*c
    if abs(x - k * c) <= tol:
        cy = int(y // c)
        if _wall_at(m, k - 1, cy, 1) or _wall_at(m, k, cy, 3):
            cands.append((abs(x - k * c), math.degrees(math.atan2(abs(py - y), abs(px - x) + 1e-9))))
    if not cands:
        return None
    cands.sort()
    if len(cands) == 2 and abs(cands[0][0] - cands[1][0]) < 0.03:     # มุมห้อง: เลือกกำแพงที่มองตรงกว่า
        return min(cands[0][1], cands[1][1])
    return cands[0][1]


def deskew_shape(aspect, inc_deg):
    """รูปทรงสี่เหลี่ยมหลังแก้มุมเฉียง: มองเฉียง inc องศา ความกว้างในภาพหดเหลือ cos(inc) -> สัดส่วนจริง = aspect/cos"""
    if aspect is None:
        return None
    a = aspect / max(math.cos(math.radians(inc_deg)), 0.2)
    if ASPECT_VERT <= a <= ASPECT_HORZ:
        return "SQUARE"
    if 0.45 <= a < ASPECT_VERT:
        return "VERTICAL"
    if ASPECT_HORZ < a <= 2.2:
        return "HORIZONTAL"
    return None


def inline_dir(rc, tc):
    """ป้ายช่อง tc อยู่แนวตรง (แถว/คอลัมน์เดียวกัน) กับหุ่นช่อง rc -> (ทิศ, ห่างกี่ช่อง)
    ช่องเดียวกัน / ทแยง -> None (กติกา: ห้ามยิงจากช่องทแยง)"""
    dx, dy = tc[0] - rc[0], tc[1] - rc[1]
    if (dx and dy) or (dx == 0 and dy == 0):
        return None
    if dx == 0:
        return (0 if dy > 0 else 2), abs(dy)
    return (1 if dx > 0 else 3), abs(dx)


def line_open(m, rc, d, n):
    """ทางตรงจากช่อง rc ไป n ช่องตามทิศ d ต้องรู้แน่ว่าโล่งทุกขอบ"""
    from grid_map import OPEN
    x, y = rc
    for _ in range(n):
        if m.state(x, y, d) != OPEN:
            return False
        x += DX[d]; y += DY[d]
    return True


def target_cell(m, x, y):
    """ช่องที่ป้ายอยู่ (ใช้แสดงผล/บันทึก)"""
    c = m.cell_m
    return min(max(int(x // c), 0), m.w - 1), min(max(int(y // c), 0), m.h - 1)


def snap_cell_for_view(m, x, y, rc):
    """ช่องของป้ายเมื่อมองจากช่อง rc: ถ้าป้ายอยู่แนวตรงกับหุ่น แต่ตำแหน่งประมาณเลยกำแพง "ปลายแนว" ไปไม่เกิน
    SNAP_M (ป้ายชิดกำแพงฝั่งหุ่น + ระยะจากขนาดป้ายคลาด ~10%) -> ดึงกลับมาช่องก่อนกำแพง
    แก้เฉพาะตามแนวยิงเท่านั้น (ห้ามดึงด้านข้าง ไม่งั้นป้ายช่องทแยงจะกลายเป็นแนวตรง)"""
    c = m.cell_m
    tc = target_cell(m, x, y)
    il = inline_dir(rc, tc)
    if il is None:
        return tc
    d, n = il
    px, py = tc[0] - DX[d], tc[1] - DY[d]              # ช่องก่อนหน้าในแนวยิง
    if m.state(px, py, d) != WALL:
        return tc
    # ระยะที่ตำแหน่งประมาณเลยเส้นกำแพงนั้นไป
    over = (x - tc[0] * c) if d == 1 else (tc[0] * c + c - x) if d == 3 else            (y - tc[1] * c) if d == 0 else (tc[1] * c + c - y)
    return (px, py) if over <= C.SNAP_M else tc


# ======================================================================
# TargetHunter
# ======================================================================
class TargetHunter:
    SHOT_FIELDS = ["time_s", "round", "target", "color", "shape", "from_cell", "target_cell", "dir",
                   "cells_away", "dist_cm", "err_yaw", "err_pitch", "triggers", "burst", "fired", "knocked",
                   "note", "images"]

    def __init__(self, io, settings, log=print, sim=False):
        self.io = io
        self.S = settings
        # ตอนสแกน gimbal หันไปถึง 180° (ด้านหลัง) -> ขอบเขต yaw ของ Colorshoot (±120) ใช้ไม่ได้
        self.S["gimbal"]["yaw_min"], self.S["gimbal"]["yaw_max"] = -250.0, 250.0
        self.log = log
        self.sim = sim
        self.det = SignDetector(self.S)
        self.view = {"t": 0.0, "frame": None, "dets": []}   # ภาพ+ป้ายที่ตรวจล่าสุด (หน้าต่างกล้อง GUI)
        self.status = ""                                   # หุ่นกำลังทำอะไร (แสดงใต้ภาพกล้อง)
        _detect = self.det.detect

        def _detect_rec(frame, *a, **k):
            ds = _detect(frame, *a, **k)
            self.view = {"t": time.time(), "frame": frame,
                         "dets": [(d["color"], d["shape"], d["bbox"], d["dist_cm"]) for d in ds]}
            return ds
        self.det.detect = _detect_rec
        self.aimer = Aimer(io, self.det, Tracker(), self.S, log=log)
        self.enabled = C.SHOOT_ENABLE
        self.fire = C.SHOOT_FIRE
        self.ammo = C.SHOOT_AMMO
        self.max_tiles = C.SHOOT_MAX_TILES
        self.spec = parse_spec(C.SHOOT_TARGETS)
        self.round = 1
        self.targets = []            # dict ต่อป้าย (ดู _observe)
        self.pending = {}            # ทิศ -> ช่องที่หุ่นอยู่ตอนเห็นป้ายที่ยิงได้
        self.ex = None
        self.m = None
        self._shots_path = None
        self.plan = []               # แผนยิง [(target, ช่อง, ทิศ)] (GUI วาด)
        self.looked = set()          # (x, y, ทิศ) ที่ส่องกล้องหาป้ายไปแล้ว
        self.close_done = set()      # ช่องที่ก้มดูกำแพงชิดแล้ว (engage_close)
        self.close_queue = []        # CLOSE_MODE="retreat": ป้ายกำแพงชิดที่ต้องถอยไปยิงจากช่องแนวตรง (resolve_close)
        self.trust = None            # (ช่อง, ทิศ) ช่องยิงตามแผนที่ยืนอยู่ (run_shoot_plan) -> can_fire เชื่อแผน
        self.t0 = time.time()

    # ---------------------------------------------------------------- setup
    def attach(self, explorer):
        self.ex = explorer
        self.m = explorer.m
        self.aimer.stop = explorer.stop
        explorer.r.on_look = self.look

    def set_out_dir(self, out_dir):
        self.aimer.shot_dir = os.path.join(out_dir, "shots")
        self._shots_path = os.path.join(out_dir, "shots.csv")

    def reset(self, round_no=1):
        self.round = round_no
        self.looked = set()
        self.close_done = set()
        self.close_queue = []
        self.targets = []
        self.pending = {}
        self.t0 = time.time()

    def spec_text(self):
        if self.spec:
            return ", ".join(f"{c}:{s or '*'}" for c, s in self.spec)
        T = self.S["targets"]
        return f"สี {'/'.join(T['colors'])} × รูป {'/'.join(T['shapes'])} (settings.json)"

    def color_spec(self, color):
        """สีนี้อยู่ในเป้าที่ต้องยิง (รูปทรงไหนก็ได้) ; ใช้ตอนเห็นป้ายเฉียง/ใกล้ รูปทรงยังไม่แน่"""
        if self.spec:
            return any(color == c for c, _s in self.spec)
        return color in self.S["targets"]["colors"]

    def spec_ok(self, color, shape):
        if self.spec:
            return any(color == c and (s is None or shape == s) for c, s in self.spec)
        T = self.S["targets"]
        return color in T["colors"] and shape in T["shapes"]

    def cell_m(self):
        return self.m.cell_m if self.m is not None else C.CELL_M

    def elapsed(self):
        return round(time.time() - self.t0, 1)

    def done(self, t):
        return t["fired"] or t.get("skip", False)

    # ---------------------------------------------------------------- geometry
    def _world_pos(self, det, cam_deg, cam_xy):
        th = math.radians(cam_deg + det["bearing_deg"])
        dist = det["dist_cm"] / 100.0
        return cam_xy[0] + dist * math.sin(th), cam_xy[1] + dist * math.cos(th)

    def _in_field(self, x, y):
        if self.m is None:
            return True
        mg = 0.10
        return -mg <= x <= self.m.w * self.m.cell_m + mg and -mg <= y <= self.m.h * self.m.cell_m + mg

    def _behind_wall(self, det, x, y, cam_xy, tof_mm):
        """ป้ายที่ "อยู่หลังกำแพง" เป็นไปไม่ได้ = ของหลอก (เช่นก้อนสีบนกำแพงสี) -> True
        (1) ป้ายอยู่กลางลำแสง ToF แต่ไกลกว่ากำแพงที่ ToF วัดได้  (2) เส้นมองผ่านกำแพงที่รู้แล้วในแผนที่"""
        if tof_mm is not None and abs(det["bearing_deg"]) < C.TOF_CHECK_BEARING_DEG \
                and tof_mm < C.TOF_MAX_VALID_MM - 100 and det["dist_cm"] * 10.0 > tof_mm + C.TOF_BEHIND_MM:
            return "ไกลกว่ากำแพงที่ ToF วัดได้"
        if self.m is not None:
            vx, vy = x - cam_xy[0], y - cam_xy[1]
            L = math.hypot(vx, vy)
            if L > 0.15:
                k = (L - 0.10) / L                  # ถอยปลายเข้ามา 10 ซม. (ป้ายชิดกำแพงยังผ่าน)
                if not los_clear(self.m, cam_xy[0], cam_xy[1], cam_xy[0] + vx * k, cam_xy[1] + vy * k):
                    return "เส้นมองผ่านกำแพงในแผนที่"
        return None

    def _match(self, color, shape, x, y):
        """ป้ายเดิม = สีเดียวกัน + (รูปเดียวกันในระยะ MERGE หรือรูปต่างแต่แทบจุดเดียวกัน)
        ป้ายวางเรียงเป็นแถว -> ป้ายสีเดียวกันคนละรูปที่อยู่ติดกันต้องไม่ถูกรวมเป็นใบเดียว"""
        best, bd = None, 1e9
        for t in self.targets:
            if t["color"] != color:
                continue
            dd = math.hypot(t["x"] - x, t["y"] - y)
            lim = C.TARGET_MERGE_M if shape in t["shapes"] else C.TARGET_FLIP_M
            if dd <= lim and dd < bd:
                best, bd = t, dd
        return best

    def _true_shape(self, det, x, y, cam_xy):
        """-> (รูปทรงที่ใช้, นับโหวตได้ไหม) : มองเฉียง 20-60° แก้สัดส่วนสี่เหลี่ยมกลับ ; เกิน 60° ไม่นับโหวต"""
        inc = wall_incidence(self.m, x, y, *cam_xy) if (self.m is not None and cam_xy) else None
        if inc is None or inc <= C.OBLIQUE_DESKEW_MIN_DEG:
            return det["shape"], True
        if inc > C.OBLIQUE_VOTE_MAX_DEG:
            return det["shape"], False
        if det["shape"] == "CIRCLE":
            return "CIRCLE", True             # วงกลมที่ยังตรวจเป็นวงกลมได้ตอนเฉียง = วงกลมจริง
        s = deskew_shape(det.get("aspect"), inc)
        return (s, True) if s else (det["shape"], False)

    def _observe(self, det, x, y, cell, cam_xy=None, shape_vote=None):
        shape, vote = shape_vote if shape_vote is not None else self._true_shape(det, x, y, cam_xy)
        det = dict(det, shape=shape)
        oblique = not vote
        t = self._match(det["color"], det["shape"], x, y)
        w = 1.0 / max(det["dist_cm"], 30.0) ** 2      # ใกล้ = แม่นกว่ามาก
        if t is None:
            t = {"id": max((o["id"] for o in self.targets), default=0) + 1,
                 "color": det["color"], "shape": det["shape"],
                 "x": x, "y": y, "w": w, "n_obs": 0, "shapes": {}, "views": [],
                 "fired": False, "knocked": None, "triggers": 0, "attempts": 0, "skip": False,
                 "shot_from": None, "shot_dir": None, "locked": False, "first_seen_s": self.elapsed()}
            self.targets.append(t)
        elif not t["locked"]:        # ตำแหน่งตอนล็อคเล็งแม่นกว่า ไม่เอาค่าตอนมองผ่านมาเฉลี่ยทับ
            W = t["w"] + w
            t["x"] = (t["x"] * t["w"] + x * w) / W
            t["y"] = (t["y"] * t["w"] + y * w) / W
            t["w"] = W
        t["n_obs"] += 1
        if not oblique:          # มองเฉียงมาก รูปทรงเพี้ยน ไม่นับโหวต (ยังไม่มีโหวตเลย -> can_fire ไม่ยอมยิง)
            t["shapes"][det["shape"]] = t["shapes"].get(det["shape"], 0) + 1
        if t["shapes"]:
            t["shape"] = max(t["shapes"], key=t["shapes"].get)
        if cell is not None and list(cell) not in t["views"]:
            t["views"].append(list(cell))
        self._refresh(t)
        return t

    def _refresh(self, t):
        t["spec"] = self.spec_ok(t["color"], t["shape"])
        t["cell"] = list(target_cell(self.m, t["x"], t["y"])) if self.m is not None else \
            [int(t["x"] // C.CELL_M), int(t["y"] // C.CELL_M)]

    def shape_share(self, t):
        n = sum(t["shapes"].values())
        return t["shapes"].get(t["shape"], 0) / float(n) if n else 0.0

    # ---------------------------------------------------------------- กติกา
    def can_fire(self, t, rc, dist_cm, relaxed=False, min_views=None):
        """(ทิศ, None) ถ้ายิงได้ตามกติกา ไม่งั้น (None, เหตุผล)
        - ต้องเป็นเป้าที่กำหนด, รูปทรงมั่นใจ, เห็นจากหลายช่อง (กันของหลอก)
        - ช่องป้ายอยู่แนวตรงกับหุ่น ห่าง 1..2 ช่อง (ห้ามทแยง / ห้ามช่องเดียวกัน) ทางตรงโล่ง
        - ระยะอยู่ในช่วงที่ทดสอบแล้ว (ปกติ 60-80 ซม. ; relaxed ใช้ตอนเก็บตก/รอบสอง) และไม่เกิน 2 ช่อง"""
        if not t["spec"]:
            return None, "ไม่ใช่เป้าที่กำหนด"
        if self.done(t):
            return None, "ยิงแล้ว" if t["fired"] else "ข้ามแล้ว"
        if self.shape_share(t) < C.SHAPE_MIN_SHARE:
            return None, f"รูปทรงไม่แน่ใจ {t['shapes']}"
        tr = self._trusted_dir(t, rc)
        if tr is not None:
            # ยืนช่องยิงตามแผน (ตรวจกติกาแถว/ทางโล่งตอนวางแผนแล้ว) -> เหลือตรวจระยะอย่างเดียว
            lo, hi = C.SHOOT_RELAXED_CM if relaxed else C.SHOOT_WINDOW_CM
            if not (lo <= dist_cm <= hi):
                return None, f"ระยะ {dist_cm:.0f} ซม. นอกช่วง {lo:.0f}-{hi:.0f}"
            return tr, None
        need = C.FIRE_MIN_VIEWS if min_views is None else min_views
        if len(t["views"]) < need:
            return None, f"เห็นจาก {len(t['views'])} ช่อง (ต้อง ≥{need})"
        if self.m is None:
            return None, "ไม่มีแผนที่"
        tc = snap_cell_for_view(self.m, t["x"], t["y"], rc)
        il = inline_dir(rc, tc)
        sf = t.get("shot_from")
        if (sf is not None and tuple(sf) == tuple(rc) and t.get("shot_dir") is not None
                and t.get("close_ang") is None):
            # เคยยิงสำเร็จจากช่องนี้ทิศนี้ (รอบ 1) -> ใช้ทิศเดิม (log 1/10 รอบ 2: ตำแหน่งป้ายที่คำนวณใหม่คลาด ~10 ซม.
            # ข้ามเส้นช่อง -> "ป้ายช่อง (1,3) ทแยงกับหุ่น (0,2)" ไม่ยิง ทั้งที่รอบ 1 ยิงจากช่องนี้ได้)
            il = (t["shot_dir"], il[1] if (il is not None and il[0] == t["shot_dir"]) else 1)
        if il is None:
            return None, f"ป้ายช่อง {tc} {'ช่องเดียวกัน' if tuple(tc) == tuple(rc) else 'ทแยง'}กับหุ่น {tuple(rc)}"
        d, n = il
        if n > self.max_tiles:
            return None, f"ห่าง {n} ช่อง > {self.max_tiles:g}"
        if not line_open(self.m, rc, d, n):
            return None, "ทางตรงไปป้ายมีกำแพง/ยังไม่รู้"
        c = self.m.cell_m
        inc = wall_incidence(self.m, t["x"], t["y"], (rc[0] + 0.5) * c, (rc[1] + 0.5) * c)
        if inc is not None and inc > C.OBLIQUE_FIRE_MAX_DEG:
            return None, f"มองป้ายเฉียง {inc:.0f}° (ต้องยิงจากช่องที่เห็นหน้าตรง)"
        lo, hi = C.SHOOT_RELAXED_CM if relaxed else C.SHOOT_WINDOW_CM
        hi = min(hi, self.max_tiles * self.cell_m() * 100.0)
        if not (lo <= dist_cm <= hi):
            return None, f"ระยะ {dist_cm:.0f} ซม. นอกช่วง {lo:.0f}-{hi:.0f}"
        return d, None

    def _trusted_dir(self, t, rc):
        """ยืนอยู่ช่องยิงตามแผน (self.trust) และป้าย t อยู่หน้าทิศนั้น ห่างแนวกลางช่องไม่เกิน PLAN_TRUST_LATERAL_M
        (= อยู่แถว/คอลัมน์เดียวกัน ไม่ใช่ทแยง) ห่าง 1..max_tiles ช่อง -> ทิศนั้น ; ไม่งั้น None"""
        if not (C.PLAN_TRUST and self.trust) or tuple(self.trust[0]) != tuple(rc):
            return None
        d = self.trust[1]
        c = self.cell_m()
        vx, vy = t["x"] - (rc[0] + 0.5) * c, t["y"] - (rc[1] + 0.5) * c
        along = vx * DX[d] + vy * DY[d]
        lateral = abs(vx * DY[d] - vy * DX[d])
        if lateral > C.PLAN_TRUST_LATERAL_M or not (0.5 * c <= along <= (self.max_tiles + 0.5) * c):
            return None
        return d

    # ---------------------------------------------------------------- look (ระหว่าง scan_all)
    def look(self, d, tof_mm=None):
        """เรียกจาก robot.scan_all() หลังอ่าน ToF ทิศ d (gimbal/กล้องชี้ทิศ d อยู่) -> หาป้ายในภาพ
        ป้ายไกลก็บันทึกลงแผนที่ไว้ก่อน (รู้ล่วงหน้าว่าต้องไปยิงจากช่องไหน) ยิงเฉพาะตัวที่เข้ากติกาแล้ว"""
        if not self.enabled:
            return
        self.io.looking(d)
        self.status = f"ส่องหาป้าย ทิศ {DIRS[d]}"
        frame = self.io.read_frame()
        if frame is None:
            return
        cell = tuple(self.ex.r.cell) if self.ex else None
        if cell is not None:
            self.looked.add((cell[0], cell[1], d))
        cam_deg, cam_xy = self.io.cam_world_deg(d), self.io.cam_xy()
        for det in self.det.detect(frame):
            if det["dist_cm"] > C.TARGET_MAP_MAX_CM:
                continue
            x, y = self._world_pos(det, cam_deg, cam_xy)
            if not self._in_field(x, y):
                continue
            why = self._behind_wall(det, x, y, cam_xy, tof_mm)
            if why:
                self.log(f"  [look] ตัด {det['color']}/{det['shape']} @{det['dist_cm']:.0f}ซม.: {why}")
                continue
            n_before = len(self.targets)
            t = self._observe(det, x, y, cell, cam_xy)
            if len(self.targets) > n_before or t["n_obs"] == 2:
                self.log(f"  [look] {'เจอป้ายใหม่' if t['n_obs'] == 1 else 'ยืนยันป้าย'} #{t['id']} "
                         f"{t['color']}/{t['shape']} ~({x:.2f},{y:.2f}) ช่อง {tuple(t['cell'])} "
                         f"จากช่อง {cell} ทิศ {DIRS[d]} ระยะ {det['dist_cm']:.0f} ซม."
                         f"{' [เป้า]' if t['spec'] else ''}")
            # ตรงนี้ยังอยู่กลาง scan_all: กำแพงของช่องนี้ยังไม่ถูกเขียนลงแผนที่ -> ห้ามตัดสินกติกาที่ต้องใช้แผนที่
            # (เคยทำให้ "ทางตรงไปป้ายยังไม่รู้" แล้วไม่ยิงเลย) แค่จดทิศไว้ ; after_scan ตรวจกติกาครบหลังอัปเดตแผนที่
            lo, hi = C.SHOOT_WINDOW_CM
            if cell is not None and t["spec"] and not self.done(t) and lo <= det["dist_cm"] <= hi:
                self.pending[d] = cell

    # ---------------------------------------------------------------- engage
    def after_scan(self):
        """หลังสแกนช่องเสร็จ (explorer._sense_here): ยิงป้ายที่เข้ากติกา แล้วกลับไปสำรวจต่อ (ไม่จัดตำแหน่งเพิ่ม)"""
        if not self.enabled:
            self.pending = {}
            return
        cell = tuple(self.ex.r.cell)
        dirs = [d for d, c in self.pending.items() if c == cell]
        self.pending = {}
        if not self.fire:
            return
        # + ป้ายที่รู้ตำแหน่งแล้ว (เห็นจากช่องก่อนๆ) ที่ยิงจากช่องนี้ได้ แม้ภาพตอนสแกนช่องนี้จะไม่ติด/ระยะคลาดขอบ
        dirs += [d for d in self._known_dirs() if d not in dirs]
        # ระหว่างสำรวจ ห้ามขยับตัวหุ่นเพื่อยิง (หมุนแค่ gimbal) - การเดิน/จัดกึ่งกลางต้องไม่ถูกรบกวน
        # (log 30/9 16:56: (4,5) ชิดกำแพง W อยู่แล้ว ยังสั่ง "ขยับเข้า 150mm" ไปยิง) ; ป้ายที่ไกลเกินนิดหน่อย
        # เก็บตอน sweep ท้ายรอบ 1 / รอบ 2 (run_shoot_plan ขยับเข้าได้ที่ช่องยิงที่วางแผนไว้)
        if not dirs:
            return
        for d in dirs:
            if self.aimer.stopped():
                return
            self.engage_dir(d)       # _stable_candidate ตรวจกติกาครบ (แผนที่อัปเดตแล้ว) และ log เหตุผลถ้าไม่ยิง
        self.io.point_dir(self.ex.r.body_heading)      # gimbal กลับชี้หน้าตัวถัง

    def look_walls(self):
        """ช่องที่ผ่านแบบไม่ได้ส่องกล้อง (light scan / เดินพลาด): ทิศที่มีกำแพงไกลของช่องถัดไป (ป้ายบนกำแพงนั้น
        อยู่ห่าง ~80 ซม. หน้าตรง = ระยะยิงพอดี) และยังไม่เคยส่องจากช่องนี้ -> หันไปส่อง แล้วยิงถ้าเข้ากติกา"""
        if not (self.enabled and C.LOOK_WALLS) or self.ex is None or self.m is None:
            return
        m = self.m
        x, y = self.ex.r.cell
        dirs = [d for d in range(4)
                if (x, y, d) not in self.looked and m.in_bounds(x + DX[d], y + DY[d])
                and m.state(x, y, d) != WALL and m.state(x + DX[d], y + DY[d], d) == WALL]
        for d in dirs:
            if self.aimer.stopped():
                break
            self.io.point_dir(d)
            self.look(d, None)
        if dirs:
            self.io.point_dir(self.ex.r.body_heading)

    def engage_known(self):
        """ผ่านช่องแบบ light scan (ไม่ได้ถ่ายภาพหาป้าย): ป้ายที่อยู่ในแผนที่แล้ว ถ้าช่องนี้ยิงได้ตามกติกา
        (แนวตรง ระยะจากกลางช่องอยู่ในช่วง) -> หันไปตรวจ+ยิงเฉพาะทิศนั้น (ตรวจเห็นนิ่ง/ล็อคก่อนยิงเหมือนเดิม)"""
        if not (self.enabled and self.fire) or self.ex is None:
            return
        dirs = self._known_dirs()
        for d in dirs:
            if self.aimer.stopped():
                return
            self.engage_dir(d)
        if dirs:
            self.io.point_dir(self.ex.r.body_heading)

    def approach(self, t, d):
        """ยืนที่ช่องยิงแล้ว (เก็บตก/รอบ 2): ระยะถึงป้ายนอกช่วงยิง -> ขยับในช่องตามแนวยิงให้ได้ ~APPROACH_TARGET_CM
        (ToF ทิศที่ขยับไปต้องเหลือ ≥ APPROACH_MIN_FRONT_MM) -> ระยะที่ขยับ (m, + = เข้าหาป้าย) ให้ retreat() ถอยคืน"""
        r = self.ex.r
        cx, cy = r.est_xy()
        dist = math.hypot(t["x"] - cx, t["y"] - cy) * 100.0
        lo, hi = C.SHOOT_WINDOW_CM
        if lo + 3 <= dist <= hi - 3:
            return 0.0
        mv = max(-C.APPROACH_MAX_MM, min(C.APPROACH_MAX_MM, (dist - C.APPROACH_TARGET_CM) * 10.0))
        md = d if mv > 0 else (d + 2) % 4
        room = r.read_dir(md) - C.APPROACH_MIN_FRONT_MM
        if room < 20:
            self.log(f"  [hunt] ป้าย #{t['id']} ห่าง ~{dist:.0f} ซม. แต่ขยับทิศ {DIRS[md]} ไม่ได้ (ชิดกำแพง)")
            return 0.0
        mv = math.copysign(min(abs(mv), room), mv) / 1000.0
        r.shift(DX[d] * mv, DY[d] * mv)
        r.off = [r.off[0] + DX[d] * mv, r.off[1] + DY[d] * mv]
        self.log(f"  [hunt] ป้าย #{t['id']} ห่าง ~{dist:.0f} ซม. -> ขยับ{'เข้า' if mv > 0 else 'ออก'} "
                 f"{abs(mv) * 1000:.0f}mm ก่อนยิง")
        return mv

    def retreat(self, d, mv):
        """ถอยคืนระยะที่ approach() ขยับไป (กลับกลางช่องก่อนเดินต่อ ไม่งั้นเดินข้างแล้วเฉียดมุมกำแพง)"""
        if not mv:
            return
        r = self.ex.r
        r.shift(-DX[d] * mv, -DY[d] * mv)
        r.off = [r.off[0] - DX[d] * mv, r.off[1] - DY[d] * mv]

    def _known_dirs(self):
        """ทิศที่มีป้ายเป้า (รู้ตำแหน่งแล้ว ยังไม่ได้ยิง) ยิงได้ตามกติกาจากช่องนี้ (ระยะประมาณจากตำแหน่งหุ่น)"""
        rc = tuple(self.ex.r.cell)
        cx, cy = self.ex.r.est_xy()
        dirs = []
        for t in self.targets:
            if not t["spec"] or self.done(t):
                continue
            d, _ = self.can_fire(t, rc, math.hypot(t["x"] - cx, t["y"] - cy) * 100.0)
            if d is not None and d not in dirs:
                dirs.append(d)
        return dirs

    def _more_in_dir(self, d, exclude):
        """ยังมีป้ายเป้าอื่นที่รู้ตำแหน่ง (ยังไม่ยิง/ยังไม่ลองรอบนี้) อยู่แนวตรงทิศ d จากช่องนี้ไหม (ป้ายเรียงแถว)"""
        rc = tuple(self.ex.r.cell)
        for t in self.targets:
            if not t["spec"] or self.done(t) or t["id"] in exclude:
                continue
            il = inline_dir(rc, t["cell"])
            if il is not None and il[0] == d and il[1] <= self.max_tiles:
                return True
        return False

    def _point(self, d, yaw_off=0.0):
        """หัน gimbal ไปทิศ d (+ เยื้อง yaw_off องศา ไว้ค้นหาป้ายเมื่อไม่เห็นที่มุมตรง)"""
        if not yaw_off:
            self.io.point_dir(d)
            return
        rel = (d - self.ex.r.body_heading) % 4
        self.io.point_angle(d * 90.0 + yaw_off, C.GIMBAL_PITCH_SIDE if rel in (1, 3) else C.GIMBAL_PITCH)

    def _stable_candidate(self, d, relaxed, min_views, exclude=(), yaw_off=0.0):
        """อ่าน STABLE_FRAMES เฟรม ต้องเห็นป้ายเดิม (เข้ากติกาทุกข้อ) ≥ STABLE_MIN เฟรม (relaxed: ทุกเฟรม)"""
        rc = tuple(self.ex.r.cell)
        counts, last, why = {}, {}, {}
        seen_any, n_frames = [], 0           # ไว้บอกใน log ว่าเห็นอะไรแต่ไม่นับ (ไม่ใช่เป้า/รูปไม่ตรง)
        for i in range(C.STABLE_FRAMES):
            frame = self.aimer.fresh_frame() if i == 0 else self.io.read_frame()
            if frame is None:
                continue
            n_frames += 1
            cam_deg, cam_xy = self.io.cam_world_deg(None if yaw_off else d), self.io.cam_xy()
            seen = set()
            for det in self.det.detect(frame):
                if det["dist_cm"] > C.TARGET_MAP_MAX_CM:
                    continue
                x, y = self._world_pos(det, cam_deg, cam_xy)
                if not self._in_field(x, y) or self._behind_wall(det, x, y, cam_xy, None):
                    continue
                t = self._observe(det, x, y, rc, cam_xy)
                ts = self._true_shape(det, x, y, cam_xy)[0]     # det["shape"] = รูปในภาพดิบ (aimer ใช้หาเป้า)
                seen_any.append(f"{det['color']}/{ts}@{det['dist_cm']:.0f}")
                if t["id"] in seen or not t["spec"] or t["id"] in exclude:
                    continue
                seen.add(t["id"])
                fd, w = self.can_fire(t, rc, det["dist_cm"], relaxed, min_views)
                if fd != d:
                    why[t["id"]] = w or f"อยู่ทิศ {DIRS[fd]}"
                    continue
                if ts != t["shape"]:
                    continue                          # เฟรมนี้เห็นรูปทรงอื่น ไม่นับ
                counts[t["id"]] = counts.get(t["id"], 0) + 1
                last[t["id"]] = (det, t)
        need = C.STABLE_FRAMES if relaxed else C.STABLE_MIN
        ok = [last[k] for k, n in counts.items() if n >= need]
        if not ok:
            if counts or why:
                self.log(f"  [hunt] ไม่ยิง: เห็น {counts} (ต้อง ≥{need}/{C.STABLE_FRAMES}) "
                         + " ".join(f"#{k}:{v}" for k, v in why.items()))
            elif relaxed:
                self.log(f"  [hunt] ทิศ {DIRS[d]} ไม่เห็นป้ายเป้าที่ยิงได้เลย (ได้ภาพ {n_frames}/{C.STABLE_FRAMES} เฟรม"
                         + (f" เห็น {' '.join(sorted(set(seen_any)))}" if seen_any else " ไม่เห็นป้ายเลย") + ")")
            return None
        return min(ok, key=lambda e: e[0]["dist_cm"])

    def engage_dir(self, d, relaxed=False, min_views=None, yaw_off=0.0):
        """หัน gimbal ไปทิศ d แล้วยิงป้ายที่เข้ากติกาทีละใบ: เห็นนิ่ง -> ล็อค (ยังไม่ยิง) -> ตรวจว่าเป็นป้ายที่ตั้งใจ
        + ยังเข้ากติกา -> ยิง  (ป้ายเรียงแถว: กันล็อคไปโดนป้ายข้างๆ ที่ไม่ใช่เป้า)"""
        rc = tuple(self.ex.r.cell)
        tried = set()                 # ป้ายที่ลองแล้วในรอบนี้ (ไม่วนลองป้ายเดิมซ้ำในทิศเดียวกัน)
        for _ in range(4):
            if self.aimer.stopped():
                return
            self._point(d, yaw_off)
            cand = self._stable_candidate(d, relaxed, min_views, exclude=tried, yaw_off=yaw_off)
            if cand is None:
                return
            det, t = cand
            tried.add(t["id"])
            self.log(f"[hunt] เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} ทิศ {DIRS[d]} "
                     f"ระยะ {det['dist_cm']:.0f} ซม. -> ล็อค")
            self.status = f"เล็งเป้า #{t['id']} {t['color']}/{t['shape']} ระยะ {det['dist_cm']:.0f} ซม."
            # 1) ล็อคอย่างเดียว
            self.aimer.state = {}
            # aimer หาเป้าด้วยรูปทรง "ในภาพดิบ" (มองเฉียง รูปในภาพ != รูปจริงที่แก้มุมแล้ว)
            self.aimer.aim_and_fire(det["color"], det["shape"], start_xy=(det["cx"], det["cy"]),
                                    fire=False, ammo=self.ammo)
            st = self.aimer.state
            lk = st.get("target")
            tol = self.S["aim"]["tolerance_deg"]
            if lk is None or "err" not in st or max(abs(st["err"][0]), abs(st["err"][1])) >= tol:
                # นับครั้งเฉพาะ "ล็อคไม่ได้" (ไม่นับกรณีตรวจกติกาไม่ผ่าน เช่นระยะ 59 ซม. ตอนผ่าน -> ไม่งั้นรอบ 2
                # ไปถึงช่องยิงที่วางแผนไว้แล้วกลับถูกข้าม เพราะรอบตรวจตอนผ่านนับไปก่อนแล้ว)
                t["attempts"] += 1
                if t["attempts"] >= C.SHOOT_MAX_ATTEMPTS:
                    t["skip"] = True
                self._log_shot(t, d, {}, "ล็อคไม่ได้")
                continue
            # 2) ตรวจป้ายที่ล็อค = ป้ายที่ตั้งใจ และยังเข้ากติกา (ใช้ตำแหน่งตอนล็อค ซึ่งแม่นสุด)
            x, y = self._world_pos(lk, self.io.cam_world_deg(), self.io.cam_xy())
            bad = None
            if lk["color"] != t["color"]:
                bad = f"ล็อคได้สี {lk['color']}"
            elif self._true_shape(lk, x, y, self.io.cam_xy())[0] != t["shape"]:
                bad = f"ล็อคได้รูป {self._true_shape(lk, x, y, self.io.cam_xy())[0]} ไม่ใช่ {t['shape']}"
            elif (lk["shape"] != "CIRCLE" and lk["shape"] == t["shape"]      # เฉียง: กว้างหดเอง ไม่ใช่ป้ายซ้อน
                  and abs(lk["zw"] - lk["zh"]) / max(lk["dist_cm"], 1.0) > C.FIRE_MAX_SIZE_INCONS):
                bad = (f"ขนาดกว้าง/สูงไม่สอดคล้อง (ระยะจากกว้าง {lk['zw']:.0f} / สูง {lk['zh']:.0f} ซม.) "
                       f"= ป้ายซ้อนกันหลายใบ?")
            elif math.hypot(x - t["x"], y - t["y"]) > C.VERIFY_M:
                bad = f"ป้ายที่ล็อคห่างจากเป้า {math.hypot(x - t['x'], y - t['y']) * 100:.0f} ซม. (ป้ายข้างๆ?)"
            else:
                chk = dict(t, x=x, y=y)
                fd, w = self.can_fire(chk, rc, lk["dist_cm"], relaxed, min_views)
                if fd != d:
                    bad = w or f"ตอนล็อคอยู่ทิศ {DIRS[fd]}"
            if bad:
                self.log(f"  [hunt] ไม่ยิงเป้า #{t['id']}: {bad}")
                self._log_shot(t, d, {"dist_cm": lk["dist_cm"]}, "ไม่ยิง: " + bad)
                continue
            t["x"], t["y"], t["locked"] = x, y, True
            self._refresh(t)
            # 3) ยิง (เล็งซ้ำสั้นๆ จากจุดที่ล็อคไว้)
            res = self.aimer.aim_and_fire(lk["color"], lk["shape"], start_xy=(lk["cx"], lk["cy"]),
                                          fire=self.fire, ammo=self.ammo)
            t["fired"] = t["fired"] or bool(res.get("fired"))
            if res.get("knocked") is not None:
                t["knocked"] = res["knocked"]
            t["triggers"] += res.get("triggers", 0)
            if res.get("fired"):
                t["shot_from"], t["shot_dir"], t["shot_s"] = list(rc), d, self.elapsed()
                for o in self.targets:                # ป้ายซ้ำ (ตำแหน่งคลาดตอนเห็นไกล) = ใบเดียวกัน ไม่ต้องไปยิงอีก
                    if (o is not t and not self.done(o) and o["color"] == t["color"] and o["shape"] == t["shape"]
                            and math.hypot(o["x"] - t["x"], o["y"] - t["y"]) <= C.TARGET_DUP_M):
                        o["skip"], o["dup_of"] = True, t["id"]
                        self.log(f"  [hunt] ป้าย #{o['id']} = ป้ายเดียวกับ #{t['id']} ที่ยิงแล้ว (ห่าง "
                                 f"{math.hypot(o['x'] - t['x'], o['y'] - t['y']) * 100:.0f} ซม.)")
            note = "fired" if res.get("fired") else "ล็อคซ้ำไม่ได้/ไม่ยิง"
            self.status = f"ยิงเป้า #{t['id']} {t['color']}/{t['shape']} แล้ว!" if res.get("fired") else \
                f"ไม่ยิงเป้า #{t['id']} (ล็อคซ้ำไม่ได้)"
            self._log_shot(t, d, res, note)
            if self.ex is not None:
                self.ex.L.log(self.ex.r, "shoot",
                              note=f"เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} "
                                   f"จากช่อง {rc} ทิศ {DIRS[d]} ระยะ {res.get('dist_cm', 0) or 0:.0f} ซม.: {note}")
                self.ex.on_update()
            if t["fired"] and not self._more_in_dir(d, tried):
                return                        # ไม่มีป้ายเป้าอื่นในแนวนี้ ไม่ต้องอ่านภาพซ้ำอีกรอบ (~3-4 วิ/เป้า)

    # ---------------------------------------------------------------- ป้ายบนกำแพงชิด (ยิงจากช่องตัวเอง)
    def engage_close(self, last_d):
        """(ผู้ใช้ 30/9 23:45) หลังจัดกลางช่อง (สแกนเต็มครั้งแรกของช่อง): กำแพงของช่องนี้ด้านที่ขนานกับทางที่เพิ่ง
        สไลด์มา -> ก้มกล้อง หันเฉียง CLOSE_ANGLES จากด้านนั้น หาป้ายเป้าบนกำแพงนั้นแล้วยิงเลยจากช่องนี้
        (ระยะ ~34 ซม. ; ยกเว้นกติกา "ห้ามยิงจากช่องของป้าย/ห้ามทแยง" เฉพาะกรณีนี้ ผู้ใช้ตัดสินใจว่าแม่นพอ)
        ด้านอื่นไม่ต้อง: ป้ายด้านนั้นเห็นจากช่องก่อนหน้าแล้ว"""
        if not (self.enabled and C.CLOSE_LOOK_ENABLE) or self.ex is None or self.m is None:
            return
        x, y = self.ex.r.cell
        if (x, y) in self.close_done:
            return
        self.close_done.add((x, y))
        cand = range(4) if C.CLOSE_ALL_WALLS else ((last_d + 1) % 4, (last_d + 3) % 4)
        sides = [s for s in cand if self.m.state(x, y, s) == WALL]
        # มุมเฉียงเดียวกันเห็นได้ 2 กำแพง (เช่น 45° = ปลายกำแพง N + ปลายกำแพง E) -> ไม่หันซ้ำ
        angs = {}
        for s in sides:
            for off in C.CLOSE_ANGLES:
                angs.setdefault(round(s * 90.0 + off) % 360, []).append(s)
        b = self.ex.r.body_heading * 90.0
        order = sorted(angs, key=lambda a: (a - b + 180.0) % 360.0)    # กวาดทางเดียว ไม่หมุน gimbal ไปมา
        if order:
            self.log(f"  [close] ช่อง {(x, y)} ก้มดูกำแพง {''.join(DIRS[s] for s in sides)} มุม "
                     + " ".join(f"{a}°" for a in order))
        for a in order:
            if self.aimer.stopped():
                return
            self._close_at(float(a), angs[a])
        if order:
            self.io.point_dir(self.ex.r.body_heading)

    def resolve_close(self):
        """(explorer.explore หลังเดินจบ path) ป้ายกำแพงชิดที่จดไว้ -> ไปช่องยิงแนวตรงที่ใกล้สุด (แผนเดียวกับเก็บตก/รอบ 2:
        ระยะ ~70 ซม. หน้าตรง ตรวจรูปทรงก่อนยิง) แล้วกลับไปสำรวจต่อ ; ไม่มีช่องยิงตามกติกา -> เก็บไว้ให้เก็บตก/รอบ 2"""
        todo = [t for t in self.close_queue if not self.done(t)]
        self.close_queue = []
        if not todo or not (self.enabled and self.fire) or self.ex is None or self.aimer.stopped():
            return
        self.ex.L.log(self.ex.r, "close_retreat",
                      note="ถอยไปยิงป้ายกำแพงชิด: " + ", ".join(f"#{t['id']} {t['color']}/{t['shape']}" for t in todo))
        run_shoot_plan(self.ex, self, todo, relaxed=True, min_views=1, deadline_s=C.ROUND1_SWEEP_DEADLINE_S)
        for t in todo:
            if not t["fired"]:
                t["skip"], t["attempts"] = False, 0        # ยังไม่ได้ยิง -> ให้เก็บตกท้ายรอบ 1 ลองอีก

    def _on_wall(self, x, y, rc, s):
        """ตำแหน่ง (x,y) อยู่บนกำแพงทิศ s ของช่อง rc (ห่างเส้นกำแพง <= CLOSE_WALL_TOL_M และอยู่ในช่วงช่องนั้น)"""
        c = self.m.cell_m
        rx, ry = rc
        if s in (0, 2):
            line = (ry + 1) * c if s == 0 else ry * c
            return abs(y - line) <= C.CLOSE_WALL_TOL_M and rx * c - 0.05 <= x <= (rx + 1) * c + 0.05
        line = (rx + 1) * c if s == 1 else rx * c
        return abs(x - line) <= C.CLOSE_WALL_TOL_M and ry * c - 0.05 <= y <= (ry + 1) * c + 0.05

    def _close_shape(self, det, x, y, cam_xy, s):
        """รูปทรงจริงของป้ายบนกำแพงทิศ s (รู้กำแพงแน่ ไม่ต้องหาจากแผนที่): มุมเฉียง = มุมระหว่างแนวมองกับทิศ s
        -> (รูปทรง, มุม) ; แก้กลับไม่ได้ -> (None, มุม) (ไม่นับ/ไม่ยิง: รูปผิด = ยิงผิดป้าย -1)"""
        ray = math.degrees(math.atan2(x - cam_xy[0], y - cam_xy[1]))
        inc = abs((ray - s * 90.0 + 180.0) % 360.0 - 180.0)
        if det["shape"] == "CIRCLE":
            return "CIRCLE", inc
        if inc > C.OBLIQUE_VOTE_MAX_DEG:
            return None, inc
        return deskew_shape(det.get("aspect"), inc), inc

    def _close_at(self, ang, walls, force_fire=False):
        """force_fire: รอบ 2/เก็บตก ยิงซ้ำแบบที่รอบ 1 ยิงได้ (ไม่เข้าคิวถอย) ; ก้มดูกำแพงชิด: ขยายขอบบนของพื้นที่ตรวจ (roi.y0) เป็น CLOSE_ROI_Y0 ระหว่างนี้เท่านั้น
        (ภาพกล้องจริง 1/10: ก้ม -15..-22 ป้ายระยะ 25-35 ซม. ขยับขึ้นไปทับเส้น roi.y0=0.4 แล้วถูกตัดทิ้ง)"""
        roi = self.S.setdefault("roi", {})
        y0 = roi.get("y0", 0.0)
        roi["y0"] = min(y0, C.CLOSE_ROI_Y0)
        try:
            return self._close_at_inner(ang, walls, force_fire)
        finally:
            roi["y0"] = y0

    def _close_at_inner(self, ang, walls, force_fire=False):
        """หันมุมโลก ang ก้ม CLOSE_PITCH_DEG -> ป้ายเป้าบนกำแพง (ทิศใน walls) ของช่องนี้ ระยะ CLOSE_SHOOT_CM ที่เห็นนิ่ง
        -> ล็อค -> ตรวจ -> ยิง ; คืน True ถ้ายิง ; เฟรมแรกไม่เห็นป้ายบนกำแพงเลย -> ไปมุมถัดไปทันที (ประหยัดเวลา)"""
        walls = [walls] if isinstance(walls, int) else list(walls)
        rc = tuple(self.ex.r.cell)
        rel = round((ang - self.ex.r.body_heading * 90.0 + 180.0) % 360.0 - 180.0)
        if not hasattr(self, "_close_blocked"):
            self._close_blocked = set()
        if rel in self._close_blocked:
            return False            # มุมนี้ (เทียบตัวถัง) เคยหันไม่ถึง = ติด adaptor -> ไม่ดัน gimbal ซ้ำ
        self.status = f"ก้มดูป้ายบนกำแพงชิด มุม {ang % 360:.0f}°"
        if not self.io.point_angle(ang, C.CLOSE_PITCH_DEG):
            self._close_blocked.add(rel)
            self.log(f"  [close] gimbal หันไปมุม {ang % 360:.0f}° (เทียบตัวถัง {rel:+d}°) ไม่ถึง (ติด adaptor?) "
                     f"-> ข้ามมุมนี้ และไม่หันมุมนี้อีก")
            self.io.point_dir(self.ex.r.body_heading)        # ถอยออกจากจุดที่ติดก่อน
            return False
        lo, hi = C.CLOSE_SHOOT_CM
        counts, last, seen_any, wall_of = {}, {}, [], {}
        for i in range(C.STABLE_FRAMES):
            frame = self.aimer.fresh_frame() if i == 0 else self.io.read_frame()
            if frame is None:
                continue
            cam_deg, cam_xy = self.io.cam_world_deg(), self.io.cam_xy()
            seen = set()
            on_wall = 0
            for det in self.det.detect(frame):
                x, y = self._world_pos(det, cam_deg, cam_xy)
                if not (lo <= det["dist_cm"] <= hi):
                    continue
                s = next((w for w in walls if self._on_wall(x, y, rc, w)), None)
                if s is None:
                    continue
                on_wall += 1
                shp, inc = self._close_shape(det, x, y, cam_xy, s)
                retreat = C.CLOSE_MODE == "retreat"
                if shp is None:
                    seen_any.append(f"{det['color']}/?(เฉียง {inc:.0f}° แก้รูปไม่ได้)@{det['dist_cm']:.0f}")
                    if not (retreat and self.color_spec(det["color"])):
                        continue
                    # ถอยไปยิง: รู้แค่ว่ามีป้ายสีเป้าตรงนี้ก็พอ (ไม่นับโหวตรูปทรง ไปดูหน้าตรงที่ช่องยิง)
                    t = self._observe(det, x, y, rc, cam_xy, shape_vote=(det["shape"], False))
                else:
                    t = self._observe(det, x, y, rc, cam_xy, shape_vote=(shp, True))
                    seen_any.append(f"{det['color']}/{shp}@{det['dist_cm']:.0f}")
                want = t["spec"] or (retreat and self.color_spec(t["color"]))
                if t["id"] in seen or not want or self.done(t):
                    continue
                seen.add(t["id"])
                counts[t["id"]] = counts.get(t["id"], 0) + 1
                last[t["id"]] = (det, t)
                wall_of[t["id"]] = s
            if i == 0 and not on_wall:
                break
        ok = [last[k] for k, n in counts.items() if n >= C.STABLE_MIN]
        if not ok:
            if seen_any:
                self.log(f"  [close] มุม {ang % 360:.0f}°: เห็น {' '.join(sorted(set(seen_any)))} "
                         f"แต่ไม่ใช่เป้าที่ยังไม่ยิง/ไม่นิ่ง")
            return False
        det, t = min(ok, key=lambda e: e[0]["dist_cm"])
        s = wall_of[t["id"]]
        if C.CLOSE_MODE == "retreat" and not force_fire:
            # มีช่องแนวตรงที่ยิงได้ตามกติกา (เดินไปถึงได้) -> ไม่ยิงจากช่องตัวเอง จดไว้ ถอยไปยิงหน้าตรง ~70 ซม. หลังเดินจบ path
            # ไม่มี (เช่นช่องแคบมีกำแพงหน้า-หลัง ป้ายเห็นได้จากช่องนี้ช่องเดียว; sim seed 11 (4,0)) -> ยิงจากช่องนี้แบบเดิม
            later, here = [], []
            for d_, t_ in ok:
                (later if shoot_spots(self, t_) else here).append((d_, t_))
            for d_, t_ in later:
                if t_ not in self.close_queue:
                    self.close_queue.append(t_)
                    self.log(f"[close] เจอป้าย #{t_['id']} {t_['color']}/{t_['shape']} บนกำแพง {DIRS[wall_of[t_['id']]]} "
                             f"ช่องนี้ {rc} ระยะ {d_['dist_cm']:.0f} ซม. -> ถอยไปยิงจากช่องแนวตรงทีหลัง (ไม่ยิงตรงนี้)")
            here = [(d_, t_) for d_, t_ in here if t_["spec"]]      # ยิงจากตรงนี้ได้เฉพาะรูปทรงที่แน่ใจว่าเป็นเป้า
            if not here:
                if later:
                    self.status = f"เจอป้ายชิดกำแพง #{later[0][1]['id']} -> จะถอยไปยิงจากช่องแนวตรง"
                return False
            det, t = min(here, key=lambda e: e[0]["dist_cm"])
            s = wall_of[t["id"]]
            self.log(f"[close] ป้าย #{t['id']} ไม่มีช่องแนวตรงที่ยิงได้ตามกติกา -> ยิงจากช่องนี้แทน")
        self.log(f"[close] เป้า #{t['id']} {t['color']}/{t['shape']} บนกำแพง {DIRS[s]} ช่องนี้ {rc} "
                 f"มุม {ang % 360:.0f}° ระยะ {det['dist_cm']:.0f} ซม. -> ล็อค")
        self.aimer.state = {}
        self.aimer.aim_and_fire(det["color"], det["shape"], start_xy=(det["cx"], det["cy"]),
                                fire=False, ammo=self.ammo)
        st = self.aimer.state
        lk = st.get("target")
        tol = self.S["aim"]["tolerance_deg"]
        if lk is None or "err" not in st or max(abs(st["err"][0]), abs(st["err"][1])) >= tol:
            self.log(f"  [close] ล็อคเป้า #{t['id']} ไม่ได้")
            self._log_shot(t, s, {}, "close: ล็อคไม่ได้")
            return False
        cam_xy = self.io.cam_xy()
        x, y = self._world_pos(lk, self.io.cam_world_deg(), cam_xy)
        bad = None
        if lk["color"] != t["color"]:
            bad = f"ล็อคได้สี {lk['color']}"
        elif self._close_shape(lk, x, y, cam_xy, s)[0] != t["shape"]:
            bad = f"ล็อคได้รูป {self._close_shape(lk, x, y, cam_xy, s)[0]} ไม่ใช่ {t['shape']}"
        elif math.hypot(x - t["x"], y - t["y"]) > C.VERIFY_M:
            bad = f"ป้ายที่ล็อคห่างจากเป้า {math.hypot(x - t['x'], y - t['y']) * 100:.0f} ซม. (ป้ายข้างๆ?)"
        elif not (lo <= lk["dist_cm"] <= hi):
            bad = f"ระยะตอนล็อค {lk['dist_cm']:.0f} ซม. นอกช่วง {lo}-{hi}"
        if bad:
            self.log(f"  [close] ไม่ยิงเป้า #{t['id']}: {bad}")
            self._log_shot(t, s, {"dist_cm": lk["dist_cm"]}, "close ไม่ยิง: " + bad)
            return False
        res = self.aimer.aim_and_fire(lk["color"], lk["shape"], start_xy=(lk["cx"], lk["cy"]),
                                      fire=self.fire, ammo=self.ammo)
        t["fired"] = t["fired"] or bool(res.get("fired"))
        t["triggers"] += res.get("triggers", 0)
        if res.get("fired"):
            t["shot_from"], t["shot_dir"], t["shot_s"] = list(rc), s, self.elapsed()
            t["close_ang"] = ang
            t["locked"] = True
        note = "fired (close)" if res.get("fired") else "close: ล็อคซ้ำไม่ได้/ไม่ยิง"
        self.status = f"ยิงเป้า #{t['id']} (กำแพงชิด) แล้ว!" if res.get("fired") else f"ไม่ยิงเป้า #{t['id']}"
        self._log_shot(t, s, res, note)
        self.ex.L.log(self.ex.r, "shoot",
                      note=f"เป้า #{t['id']} {t['color']}/{t['shape']} บนกำแพง {DIRS[s]} ของช่อง {rc} "
                           f"(ยิงจากช่องตัวเอง มุม {ang % 360:.0f}°) ระยะ {res.get('dist_cm', 0) or 0:.0f} ซม.: {note}")
        self.ex.on_update()
        return bool(res.get("fired"))

    def _log_shot(self, t, d, res, note):
        if not self._shots_path:
            return
        new = not os.path.exists(self._shots_path)
        os.makedirs(os.path.dirname(self._shots_path), exist_ok=True)
        rc = tuple(self.ex.r.cell) if self.ex else None
        il = inline_dir(rc, t["cell"]) if rc else None
        with open(self._shots_path, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=self.SHOT_FIELDS)
            if new:
                w.writeheader()
            w.writerow({"time_s": self.elapsed(), "round": self.round, "target": t["id"],
                        "color": t["color"], "shape": t["shape"],
                        "from_cell": f"{rc[0]},{rc[1]}" if rc else "",
                        "target_cell": f"{t['cell'][0]},{t['cell'][1]}", "dir": DIRS[d],
                        "cells_away": il[1] if il else "",
                        "dist_cm": round(res.get("dist_cm", 0) or 0, 1),
                        "err_yaw": round(res.get("err_yaw", 0) or 0, 2),
                        "err_pitch": round(res.get("err_pitch", 0) or 0, 2),
                        "triggers": res.get("triggers", 0), "burst": res.get("burst", ""),
                        "fired": int(bool(res.get("fired"))), "knocked": res.get("knocked"),
                        "note": note, "images": " ".join(i for i in res.get("images", []) if i)})

    # ---------------------------------------------------------------- เก็บตก (ท้ายรอบ 1)
    def sweep(self):
        """สำรวจครบแล้วยังมีเป้าที่ต้องยิงค้าง -> เดินไปช่องยิงที่ใกล้สุด (เส้นทางสั้นสุด) แล้วยิง ก่อนกลับจุดเริ่ม
        ใช้ระยะ relaxed + ยอมเห็นจากช่องเดียวถ้าเห็นนิ่งครบทุกเฟรม ; ข้ามถ้าใกล้หมดเวลา"""
        left = [t for t in self.confirmed() if t["spec"] and not t["fired"]]
        if not left or not self.fire:
            return
        if not C.SWEEP_ENABLE:
            self.ex.L.log(self.ex.r, "sweep", note=f"ยังไม่ได้ยิง {len(left)} เป้า (SWEEP_ENABLE=False ไม่อ้อมไปยิง): "
                          + ", ".join(f"#{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])}" for t in left))
            return
        for t in left:                        # ให้โอกาสใหม่จากช่องที่เหมาะกว่า
            t["skip"], t["attempts"] = False, 0
        self.ex.L.log(self.ex.r, "sweep", note=f"สำรวจครบ ยังไม่ได้ยิง {len(left)} เป้า: "
                      + ", ".join(f"#{t['id']} {t['color']}/{t['shape']}" for t in left))
        run_shoot_plan(self.ex, self, left, relaxed=True, min_views=1, deadline_s=C.ROUND1_SWEEP_DEADLINE_S)

    # ---------------------------------------------------------------- export
    def confirmed(self):
        """ป้ายที่เชื่อได้: เห็น ≥2 ครั้ง หรือเคยล็อคเล็งได้ (กันของหลอกเฟรมเดียว)"""
        return [t for t in self.targets if t["n_obs"] >= 2 or t["locked"]]

    def export(self, include_unconfirmed=False):
        out = []
        for t in (self.targets if include_unconfirmed else self.confirmed()):
            if t.get("dup_of"):
                continue                  # ใบเดียวกับป้ายที่ยิงแล้ว (ตำแหน่งคลาด) ไม่ส่งเป็นเป้าแยก/ไม่ให้รอบ 2 ไปยิงซ้ำ
            e = {k: t[k] for k in ("id", "color", "shape", "cell", "n_obs", "spec", "fired",
                                   "knocked", "triggers", "shot_from", "shot_dir", "locked")}
            e["close_ang"] = t.get("close_ang")      # ยิงจากช่องตัวเองแบบก้มเฉียง (รอบ 2 ทำแบบเดิม)
            e["views"] = len(t["views"])
            e["confirmed"] = t["n_obs"] >= 2 or t["locked"]
            e["x_m"], e["y_m"] = round(t["x"], 3), round(t["y"], 3)
            out.append(e)
        return out

    def load(self, targets):
        """โหลดเป้าจากแผนที่ที่บันทึกไว้ (map.json -> meta.targets) สำหรับรอบสอง"""
        self.targets = []
        for i, e in enumerate(targets):
            t = {"id": e.get("id", i + 1), "color": e["color"], "shape": e["shape"],
                 "x": e["x_m"], "y": e["y_m"], "w": 1.0, "n_obs": max(2, e.get("n_obs", 2)),
                 "shapes": {e["shape"]: 1}, "views": [["r1", k] for k in range(e.get("views", 2))],
                 "fired": False, "knocked": None, "triggers": 0, "attempts": 0, "skip": False,
                 "shot_from": e.get("shot_from"), "shot_dir": e.get("shot_dir"), "close_ang": e.get("close_ang"),
                 "locked": e.get("locked", False)}
            self._refresh(t)
            self.targets.append(t)


# ======================================================================
# วางแผนเส้นทางยิง (เก็บตกรอบ 1 + รอบ 2)
# ======================================================================
def shoot_spots(hunter, t, relaxed=True):
    """ช่องที่ยืนยิงเป้า t ได้ตามกติกา -> [(ช่อง, ทิศ, ค่าปรับ)]  ค่าปรับ 0 = ระยะในช่วงที่ทดสอบ (60-80 ซม.)"""
    m = hunter.m
    c = m.cell_m
    out = []
    for x in range(m.w):
        for y in range(m.h):
            if not m.visited[x][y]:
                continue
            # ไม่ใช้ snap_cell_for_view ตอนวางแผน: การดึงกลับข้ามกำแพงเชื่อได้เฉพาะตอน "เห็นป้ายอยู่จริง"
            # (ป้ายชิดกำแพงอีกฝั่งจะถูกดึงมาฝั่งที่มองไม่เห็น)
            il = inline_dir((x, y), target_cell(m, t["x"], t["y"]))
            if il is None or il[1] > hunter.max_tiles or not line_open(m, (x, y), il[0], il[1]):
                continue
            inc = wall_incidence(m, t["x"], t["y"], (x + 0.5) * c, (y + 0.5) * c)
            if inc is not None and inc > C.OBLIQUE_FIRE_MAX_DEG:
                continue                            # มองเฉียงเกิน ไม่ใช่ช่องยิง
            dist = math.hypot((x + 0.5) * c - t["x"], (y + 0.5) * c - t["y"]) * 100.0
            lo, hi = C.SHOOT_WINDOW_CM
            m_ = C.PLAN_DIST_MARGIN_CM            # วัดจากกลางช่อง: เผื่อหุ่นเยื้อง/ระยะคลาด
            if lo + m_ <= dist <= hi - m_:
                pen = 0.0
            else:
                # ชิดขอบช่วง: ยังใช้ได้แต่ให้ความสำคัญรองลงมา (ตอนยิงจริงตรวจระยะที่วัดได้อีกครั้ง ห้ามเกินช่วง)
                lo, hi = C.SHOOT_RELAXED_CM if relaxed else C.SHOOT_WINDOW_CM
                hi = min(hi, hunter.max_tiles * c * 100.0)
                if lo <= dist <= hi:
                    pen = C.PLAN_OUTSIDE_WINDOW_PENALTY
                elif relaxed and lo - C.PLAN_APPROACH_CM <= dist <= hi + C.PLAN_APPROACH_CM:
                    pen = C.PLAN_OUTSIDE_WINDOW_PENALTY + 0.5      # ต้องขยับเข้า/ออกในช่องก่อนยิง (approach)
                else:
                    continue
            if [x, y] in t.get("bad_spots", []):
                continue                            # เคยไปยืนยิงช่องนี้แล้วไม่สำเร็จ ลองช่องอื่น
            if t.get("shot_from") and tuple(t["shot_from"]) == (x, y):
                pen -= 0.5                          # เคยยิงสำเร็จจากช่องนี้ในรอบ 1
            out.append(((x, y), il[0], pen))
    # เคยยิงสำเร็จ (รอบ 1) -> ใช้ช่อง+ทิศนั้นอย่างเดียว (เห็นหน้าป้ายตรงแน่) จนกว่าจะไปแล้วยิงไม่ได้
    # (log 30/9 20:57: #6 ยิงสำเร็จจาก (5,1) ทิศ W แต่รอบ 2 ไปยืน (4,0) หัน N ที่ใกล้กว่า = มองป้ายด้านข้าง ไม่เห็น)
    sf = t.get("shot_from")
    if sf and list(sf) not in t.get("bad_spots", []) and m.visited[sf[0]][sf[1]] and t.get("shot_dir") is not None:
        return [((sf[0], sf[1]), t["shot_dir"], -0.5)]
    return out


def _bfs_all(m, sources):
    from collections import deque
    dist = {}
    for s in sources:
        dd = {s: 0}
        q = deque([s])
        while q:
            u = q.popleft()
            # ข้ามขอบ UNKNOWN ได้ (เหมือน go_to ที่ลองทางที่ยังไม่รู้แล้วสแกนยืนยันระหว่างทาง)
            for _, nx, ny in m.neighbors(*u, allow_unknown=True):
                v = (nx, ny)
                if v not in dd:
                    dd[v] = dd[u] + 1
                    q.append(v)
        dist[s] = dd
    return dist


def plan_route(hunter, start, targets, relaxed=True):
    """ลำดับเป้า + ช่องยืนยิง ที่ทำให้เดินรวมสั้นสุด (ยิงหลายเป้าจากช่องเดียวได้ = ระยะ 0)
    เป้า ≤ 7 ใบ: ลองทุกลำดับ (เลือกช่องแต่ละเป้าด้วย DP) ; มากกว่านั้นใช้ใกล้สุดก่อน
    -> [(target, ช่อง, ทิศ)] (เป้าที่ไม่มีช่องยิงได้ถูกตัดออก)"""
    import itertools
    m = hunter.m
    spots = {t["id"]: shoot_spots(hunter, t, relaxed) for t in targets}
    ts = [t for t in targets if spots[t["id"]]]
    if not ts:
        return []
    cells = {tuple(start)} | {s[0] for t in ts for s in spots[t["id"]]}
    D = _bfs_all(m, cells)
    # ตัดช่องยิงที่เดินไปไม่ถึงออกก่อน: เดิมถ้าเป้าใบเดียวไม่มีช่องที่ไปถึง ทุกลำดับได้ INF -> แผนว่าง
    # -> ไม่ยิงสักใบ (sim seed 7: ขอบ (0,4)-(0,5) เป็น UNKNOWN รอบ 2 ยิง 0/4)
    reach = D[tuple(start)]
    for t in ts:
        spots[t["id"]] = [sp for sp in spots[t["id"]] if sp[0] in reach]
    ts = [t for t in ts if spots[t["id"]]]
    if not ts:
        return []
    INF = 1e9

    def best_for(order):
        # DP ชั้นละเป้า: layer[i] = {spot: (cost, prev)}
        prev_layer = {tuple(start): (0.0, None)}
        hist = []
        for t in order:
            layer = {}
            for cell, d, pen in spots[t["id"]]:
                bc, bp = INF, None
                for pc, (cost, _) in prev_layer.items():
                    step = D.get(pc, {}).get(cell)
                    if step is None:
                        continue
                    v = cost + step + pen
                    if v < bc:
                        bc, bp = v, pc
                if bp is not None and (cell not in layer or bc < layer[cell][0]):
                    layer[cell] = (bc, bp, d)
            if not layer:
                return INF, None
            hist.append(layer)
            prev_layer = {k: (v[0], v[1]) for k, v in layer.items()}
        end = min(prev_layer, key=lambda k: prev_layer[k][0])
        total = prev_layer[end][0]
        seq, cur = [], end
        for t, layer in zip(reversed(order), reversed(hist)):
            cost, pc, d = layer[cur]
            seq.append((t, cur, d))
            cur = pc
        return total, list(reversed(seq))

    if len(ts) <= 7:
        best = (INF, None)
        for order in itertools.permutations(ts):
            r = best_for(order)
            if r[0] < best[0]:
                best = r
        return best[1] or []
    seq, cur, left = [], tuple(start), list(ts)
    while left:
        opts = [(D[cur].get(c, INF) + pen, t, c, d) for t in left for c, d, pen in spots[t["id"]]
                if cur in D]
        if not opts:
            break
        _, t, c, d = min(opts, key=lambda o: o[0])
        seq.append((t, c, d)); left.remove(t); cur = c
        if cur not in D:
            D.update(_bfs_all(m, [cur]))
    return seq


def run_shoot_plan(explorer, hunter, targets, relaxed=True, min_views=None, deadline_s=None):
    """เดินตามแผน plan_route ทีละเป้า (วางแผนใหม่ทุกครั้ง เผื่อยิงระหว่างทางไปแล้ว/ไปไม่ถึง)"""
    r = explorer.r
    while not explorer.stopped():
        todo = [t for t in targets if not hunter.done(t)]
        if not todo:
            break
        if deadline_s is not None and hunter.elapsed() > deadline_s:
            explorer.L.log(r, "shoot_plan", note=f"เวลา {hunter.elapsed():.0f} วิ เกิน {deadline_s} - เลิกเก็บตก")
            break
        plan = plan_route(hunter, r.cell, todo, relaxed)
        if not plan:
            explorer.L.log(r, "shoot_plan", note="ไม่มีช่องที่ยิงเป้าที่เหลือได้ตามกติกา: " +
                           ", ".join(f"#{t['id']}" for t in todo))
            break
        hunter.plan = plan
        t, cell, d = plan[0]
        hunter.status = f"กำลังไปยิงเป้า #{t['id']} {t['color']}/{t['shape']} (ยืนยิงที่ช่อง {cell})"
        explorer.echo(f"[plan] เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} "
                      f"<- ยืนยิงที่ {cell} หัน {DIRS[d]}  (แผน: " +
                      " -> ".join(f"#{p[0]['id']}@{p[1]}" for p in plan) + ")")
        if not explorer.go_to(cell):
            t["skip"] = True
            continue
        hunter.trust = (tuple(cell), d)
        if not hunter.done(t):
            if t.get("close_ang") is not None and list(cell) == list(t.get("shot_from") or []):
                hunter._close_at(t["close_ang"], d, force_fire=True)      # รอบ 1 ยิงได้จากช่องตัวเองแบบก้มเฉียง -> ทำซ้ำ
                for off in C.CLOSE_ANGLES:
                    # มุมเดิมไม่เห็น (ยืนต่างจากรอบ 1 นิดหน่อย) -> ลองมุมเฉียงอีกข้างของกำแพงเดียวกันก่อนยอมแพ้
                    a = (d * 90.0 + off) % 360.0
                    if hunter.done(t) or hunter.aimer.stopped() or abs((a - t["close_ang"] + 180) % 360 - 180) < 1:
                        continue
                    hunter._close_at(a, d, force_fire=True)
                hunter.io.point_dir(r.body_heading)
            else:
                mv = hunter.approach(t, d)
                hunter.engage_dir(d, relaxed=relaxed, min_views=min_views)
                for off in C.PLAN_SEARCH_YAW:
                    # ไม่เห็น/ไม่ยิงที่มุมตรง (หุ่นเยื้อง/เอียงเล็กน้อย) -> หันเยื้องหาอีกนิดก่อนยอมแพ้ช่องนี้
                    # (log 1/10 รอบ 2: ไปยืนช่องที่รอบ 1 ยิงได้ แต่ "ไม่เห็นป้ายเลย" แล้วข้ามทุกเป้า)
                    if hunter.done(t) or hunter.aimer.stopped():
                        break
                    hunter.log(f"  [hunt] ไม่ได้ยิงเป้า #{t['id']} ที่มุมตรง -> หันเยื้อง {off:+.0f}° หา")
                    hunter.engage_dir(d, relaxed=relaxed, min_views=min_views, yaw_off=off)
                hunter.retreat(d, mv)
            if not hunter.done(t):
                # ยิงไม่ได้จากช่องนี้ (มุมเฉียง/ป้ายข้างๆ บัง) -> ลองช่องยิงอื่นได้อีก รวมไม่เกิน MAX_SPOTS ช่อง
                t.setdefault("bad_spots", []).append(list(cell))
                t["skip"] = len(t["bad_spots"]) >= C.SHOOT_MAX_SPOTS
                t["attempts"] = 0
        hunter.trust = None
        hunter.io.point_dir(r.body_heading)
    hunter.trust = None
    hunter.plan = []


def run_round2(explorer, hunter):
    """รอบสอง: ใช้แผนที่รอบแรก วางเส้นทางสั้นสุดที่ผ่านช่องยิงของทุกเป้าที่กำหนด แล้วยิงให้ครบ"""
    r = explorer.r
    hunter.round = 2
    hunter.t0 = time.time()
    for t in hunter.targets:
        t["fired"], t["knocked"], t["attempts"], t["triggers"], t["skip"] = False, None, 0, 0, False
    todo = [t for t in hunter.targets if t["spec"]]
    explorer.L.log(r, "round2", note=f"รอบ 2: เป้าที่ต้องยิง {len(todo)} ใบ: " +
                   ", ".join(f"#{t['id']} {t['color']}/{t['shape']}" for t in todo))
    r.reset_heading_ref()
    run_shoot_plan(explorer, hunter, todo, relaxed=True, min_views=1)
    shot = sum(1 for t in todo if t["fired"])
    explorer.L.log(r, "round2_end", note=f"รอบ 2 จบ: ยิง {shot}/{len(todo)} เป้า เวลา {hunter.elapsed():.0f} วิ")
    explorer.on_update()
    return shot, len(todo)


def make_settings(sim=False):
    """sim: ใช้ค่าเริ่มต้น (สีใน settings.json เรียนจากกล้องจริง ไม่ตรงกับภาพจำลอง) ; จริง: settings.json"""
    if sim:
        S = copy.deepcopy(ST.DEFAULTS)
        S["aim"]["settle_s"] = 0.0
        S["aim"]["confirm_wait_s"] = 0.0
        S["aim"]["fire_hold_s_per_shot"] = 0.0
        S["aim"]["confirm_hit_gel"] = False
        return S
    return ST.load()
