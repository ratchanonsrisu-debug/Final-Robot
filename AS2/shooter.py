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
from vision import SignDetector, Tracker

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

    def __init__(self, robot, resolution="720p", log=print):
        from robomaster import blaster, camera
        self.rb = robot
        self.ep = robot._ep
        self._blaster = blaster
        self.log = log
        res = {"360p": camera.STREAM_360P, "540p": camera.STREAM_540P}.get(resolution, camera.STREAM_720P)
        self.ep.camera.start_video_stream(display=False, resolution=res)
        self.latest, self.t = None, 0.0
        self._run = True
        self._cv = threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()
        log("[shooter] เปิดกล้องแล้ว")

    def _loop(self):
        # thread เดียวอ่านทุกเฟรมต่อเนื่อง (เหมือน Grabbed ใน Colorshoot) ทุกส่วนอ่านจากที่นี่
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

    def peek(self):
        return self.latest

    def read_frame(self):
        """รอเฟรมที่มาถึงหลังเวลาที่เรียก (ภาพหลัง gimbal หยุดจริง)"""
        t0 = time.time()
        with self._cv:
            self._cv.wait_for(lambda: self.t > t0, timeout=1.5)
            return None if self.latest is None else self.latest.copy()

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

    def cam_world_deg(self):
        # มุมที่วัดจริง (sub_angle) ฝั่งขวา = GIMBAL_RIGHT_YAW เสมอ (ดู _gimbal_yaw_for / gimbal_yaw_sign)
        cw = self.rb._gimbal_yaw * (1 if C.GIMBAL_RIGHT_YAW > 0 else -1)
        return self.rb.body_heading * 90.0 + cw

    def cam_xy(self):
        return self.rb.est_xy()

    def close(self):
        self._run = False
        try:
            self.ep.camera.stop_video_stream()
        except Exception:
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

    def _inside(self, shape, lx, ly):
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
        for color, shape, sx, sy, sz in self.scene:
            t = sz / dz
            lx = rays[..., 0] * t - sx; ly = rays[..., 1] * t - sy
            m = self._inside(shape, lx, ly) & (t < zbuf) & (rays[..., 2] > 0.05)
            img[m] = np.array(self.COLORS[color], np.float32)
            zbuf[m] = t[m]
        img += self.rnd.normal(0, 4, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def shoot(self, ammo):
        """-> index ใน scene ที่โดน หรือ None"""
        R = _rot(self.pitch, self.yaw)
        o = R @ self.b_off; d = R @ self.b_dir
        for i, (c, s, sx, sy, sz) in sorted(enumerate(self.scene), key=lambda e: e[1][4]):
            tt = (sz - o[2]) / d[2]
            if tt <= 0:
                continue
            p = o + d * tt
            if ammo == "gel":
                tf = np.linalg.norm(p - o) / self.GEL_SPEED
                p = p + np.array([0, 0.5 * 981 * tf * tf, 0])
            if self._inside(s, p[0] - sx, p[1] - sy):
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
            scene.append((t["color"], t["shape"], X * 100, 4.0, Z * 100))
            ids.append(i)
        self.cam.scene = scene
        self.cam.yaw = self.gyaw - base
        self.cam.pitch = self.gpitch
        return ids

    def read_frame(self):
        self._prepare()
        return self.cam.render()

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

    def cam_world_deg(self):
        return self.rb.body_heading * 90.0 + self.gyaw

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
        self.targets = []
        self.pending = {}
        self.t0 = time.time()

    def spec_text(self):
        if self.spec:
            return ", ".join(f"{c}:{s or '*'}" for c, s in self.spec)
        T = self.S["targets"]
        return f"สี {'/'.join(T['colors'])} × รูป {'/'.join(T['shapes'])} (settings.json)"

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

    def _observe(self, det, x, y, cell):
        t = self._match(det["color"], det["shape"], x, y)
        w = 1.0 / max(det["dist_cm"], 30.0) ** 2      # ใกล้ = แม่นกว่ามาก
        if t is None:
            t = {"id": len(self.targets) + 1, "color": det["color"], "shape": det["shape"],
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
        t["shapes"][det["shape"]] = t["shapes"].get(det["shape"], 0) + 1
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
        need = C.FIRE_MIN_VIEWS if min_views is None else min_views
        if len(t["views"]) < need:
            return None, f"เห็นจาก {len(t['views'])} ช่อง (ต้อง ≥{need})"
        if self.m is None:
            return None, "ไม่มีแผนที่"
        tc = snap_cell_for_view(self.m, t["x"], t["y"], rc)
        il = inline_dir(rc, tc)
        if il is None:
            return None, f"ป้ายช่อง {tc} {'ช่องเดียวกัน' if tuple(tc) == tuple(rc) else 'ทแยง'}กับหุ่น {tuple(rc)}"
        d, n = il
        if n > self.max_tiles:
            return None, f"ห่าง {n} ช่อง > {self.max_tiles:g}"
        if not line_open(self.m, rc, d, n):
            return None, "ทางตรงไปป้ายมีกำแพง/ยังไม่รู้"
        lo, hi = C.SHOOT_RELAXED_CM if relaxed else C.SHOOT_WINDOW_CM
        hi = min(hi, self.max_tiles * self.cell_m() * 100.0)
        if not (lo <= dist_cm <= hi):
            return None, f"ระยะ {dist_cm:.0f} ซม. นอกช่วง {lo:.0f}-{hi:.0f}"
        return d, None

    # ---------------------------------------------------------------- look (ระหว่าง scan_all)
    def look(self, d, tof_mm=None):
        """เรียกจาก robot.scan_all() หลังอ่าน ToF ทิศ d (gimbal/กล้องชี้ทิศ d อยู่) -> หาป้ายในภาพ
        ป้ายไกลก็บันทึกลงแผนที่ไว้ก่อน (รู้ล่วงหน้าว่าต้องไปยิงจากช่องไหน) ยิงเฉพาะตัวที่เข้ากติกาแล้ว"""
        if not self.enabled:
            return
        self.io.looking(d)
        frame = self.io.read_frame()
        if frame is None:
            return
        cell = tuple(self.ex.r.cell) if self.ex else None
        cam_deg, cam_xy = self.io.cam_world_deg(), self.io.cam_xy()
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
            t = self._observe(det, x, y, cell)
            if cell is not None:
                fd, _ = self.can_fire(t, cell, det["dist_cm"])
                if fd == d:
                    self.pending[d] = cell

    # ---------------------------------------------------------------- engage
    def after_scan(self):
        """หลังสแกนช่องเสร็จ (explorer._sense_here): ยิงป้ายที่เข้ากติกา แล้วกลับไปสำรวจต่อ (ไม่จัดตำแหน่งเพิ่ม)"""
        if not self.enabled or not self.pending:
            self.pending = {}
            return
        cell = tuple(self.ex.r.cell)
        dirs = [d for d, c in self.pending.items() if c == cell]
        self.pending = {}
        if not self.fire or not dirs:
            return
        for d in dirs:
            if self.aimer.stopped():
                return
            self.engage_dir(d)
        self.io.point_dir(self.ex.r.body_heading)      # gimbal กลับชี้หน้าตัวถัง

    def _stable_candidate(self, d, relaxed, min_views):
        """อ่าน STABLE_FRAMES เฟรม ต้องเห็นป้ายเดิม (เข้ากติกาทุกข้อ) ≥ STABLE_MIN เฟรม (relaxed: ทุกเฟรม)"""
        rc = tuple(self.ex.r.cell)
        counts, last, why = {}, {}, {}
        for i in range(C.STABLE_FRAMES):
            frame = self.aimer.fresh_frame() if i == 0 else self.io.read_frame()
            if frame is None:
                continue
            cam_deg, cam_xy = self.io.cam_world_deg(), self.io.cam_xy()
            seen = set()
            for det in self.det.detect(frame):
                if det["dist_cm"] > C.TARGET_MAP_MAX_CM:
                    continue
                x, y = self._world_pos(det, cam_deg, cam_xy)
                if not self._in_field(x, y) or self._behind_wall(det, x, y, cam_xy, None):
                    continue
                t = self._observe(det, x, y, rc)
                if t["id"] in seen or not t["spec"]:
                    continue
                seen.add(t["id"])
                fd, w = self.can_fire(t, rc, det["dist_cm"], relaxed, min_views)
                if fd != d:
                    why[t["id"]] = w or f"อยู่ทิศ {DIRS[fd]}"
                    continue
                if det["shape"] != t["shape"]:
                    continue                          # เฟรมนี้เห็นรูปทรงอื่น ไม่นับ
                counts[t["id"]] = counts.get(t["id"], 0) + 1
                last[t["id"]] = (det, t)
        need = C.STABLE_FRAMES if relaxed else C.STABLE_MIN
        ok = [last[k] for k, n in counts.items() if n >= need]
        if not ok:
            if counts or why:
                self.log(f"  [hunt] ไม่ยิง: เห็น {counts} (ต้อง ≥{need}/{C.STABLE_FRAMES}) "
                         + " ".join(f"#{k}:{v}" for k, v in why.items()))
            return None
        return min(ok, key=lambda e: e[0]["dist_cm"])

    def engage_dir(self, d, relaxed=False, min_views=None):
        """หัน gimbal ไปทิศ d แล้วยิงป้ายที่เข้ากติกาทีละใบ: เห็นนิ่ง -> ล็อค (ยังไม่ยิง) -> ตรวจว่าเป็นป้ายที่ตั้งใจ
        + ยังเข้ากติกา -> ยิง  (ป้ายเรียงแถว: กันล็อคไปโดนป้ายข้างๆ ที่ไม่ใช่เป้า)"""
        rc = tuple(self.ex.r.cell)
        for _ in range(4):
            if self.aimer.stopped():
                return
            self.io.point_dir(d)
            cand = self._stable_candidate(d, relaxed, min_views)
            if cand is None:
                return
            det, t = cand
            t["attempts"] += 1
            if t["attempts"] >= C.SHOOT_MAX_ATTEMPTS:
                t["skip"] = True                      # ครั้งนี้เป็นครั้งสุดท้าย (ยิงไม่ได้ก็ข้าม)
            self.log(f"[hunt] เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} ทิศ {DIRS[d]} "
                     f"ระยะ {det['dist_cm']:.0f} ซม. -> ล็อค")
            # 1) ล็อคอย่างเดียว
            self.aimer.state = {}
            self.aimer.aim_and_fire(det["color"], t["shape"], start_xy=(det["cx"], det["cy"]),
                                    fire=False, ammo=self.ammo)
            st = self.aimer.state
            lk = st.get("target")
            tol = self.S["aim"]["tolerance_deg"]
            if lk is None or "err" not in st or max(abs(st["err"][0]), abs(st["err"][1])) >= tol:
                self._log_shot(t, d, {}, "ล็อคไม่ได้")
                continue
            # 2) ตรวจป้ายที่ล็อค = ป้ายที่ตั้งใจ และยังเข้ากติกา (ใช้ตำแหน่งตอนล็อค ซึ่งแม่นสุด)
            x, y = self._world_pos(lk, self.io.cam_world_deg(), self.io.cam_xy())
            bad = None
            if lk["color"] != t["color"]:
                bad = f"ล็อคได้สี {lk['color']}"
            elif lk["shape"] != t["shape"]:
                bad = f"ล็อคได้รูป {lk['shape']} ไม่ใช่ {t['shape']}"
            elif lk["shape"] != "CIRCLE" and abs(lk["zw"] - lk["zh"]) / max(lk["dist_cm"], 1.0) > C.FIRE_MAX_SIZE_INCONS:
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
            res = self.aimer.aim_and_fire(lk["color"], t["shape"], start_xy=(lk["cx"], lk["cy"]),
                                          fire=self.fire, ammo=self.ammo)
            t["fired"] = t["fired"] or bool(res.get("fired"))
            if res.get("knocked") is not None:
                t["knocked"] = res["knocked"]
            t["triggers"] += res.get("triggers", 0)
            if res.get("fired"):
                t["shot_from"], t["shot_dir"], t["shot_s"] = list(rc), d, self.elapsed()
            note = "fired" if res.get("fired") else "ล็อคซ้ำไม่ได้/ไม่ยิง"
            self._log_shot(t, d, res, note)
            if self.ex is not None:
                self.ex.L.log(self.ex.r, "shoot",
                              note=f"เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} "
                                   f"จากช่อง {rc} ทิศ {DIRS[d]} ระยะ {res.get('dist_cm', 0) or 0:.0f} ซม.: {note}")
                self.ex.on_update()

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
        for t in left:                        # ให้โอกาสใหม่จากช่องที่เหมาะกว่า
            t["skip"], t["attempts"] = False, 0
        self.ex.L.log(self.ex.r, "sweep", note=f"สำรวจครบ ยังไม่ได้ยิง {len(left)} เป้า: "
                      + ", ".join(f"#{t['id']} {t['color']}/{t['shape']}" for t in left))
        run_shoot_plan(self.ex, self, left, relaxed=True, min_views=1, deadline_s=C.ROUND1_SWEEP_DEADLINE_S)

    # ---------------------------------------------------------------- export
    def confirmed(self):
        """ป้ายที่เชื่อได้: เห็น ≥2 ครั้ง หรือเคยล็อคเล็งได้ (กันของหลอกเฟรมเดียว)"""
        return [t for t in self.targets if t["n_obs"] >= 2 or t["locked"]]

    def export(self):
        out = []
        for t in self.confirmed():
            e = {k: t[k] for k in ("id", "color", "shape", "cell", "n_obs", "spec", "fired",
                                   "knocked", "triggers", "shot_from", "shot_dir", "locked")}
            e["views"] = len(t["views"])
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
                 "shot_from": e.get("shot_from"), "shot_dir": e.get("shot_dir"),
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
            dist = math.hypot((x + 0.5) * c - t["x"], (y + 0.5) * c - t["y"]) * 100.0
            lo, hi = C.SHOOT_WINDOW_CM
            m_ = C.PLAN_DIST_MARGIN_CM            # วัดจากกลางช่อง: เผื่อหุ่นเยื้อง/ระยะคลาด
            if lo + m_ <= dist <= hi - m_:
                pen = 0.0
            else:
                lo, hi = C.SHOOT_RELAXED_CM
                if not (relaxed and lo + m_ <= dist <= min(hi, hunter.max_tiles * c * 100.0) - m_):
                    continue
                pen = C.PLAN_OUTSIDE_WINDOW_PENALTY
            if t.get("shot_from") and tuple(t["shot_from"]) == (x, y):
                pen -= 0.5                          # เคยยิงสำเร็จจากช่องนี้ในรอบ 1
            out.append(((x, y), il[0], pen))
    return out


def _bfs_all(m, sources):
    from collections import deque
    dist = {}
    for s in sources:
        dd = {s: 0}
        q = deque([s])
        while q:
            u = q.popleft()
            for _, nx, ny in m.neighbors(*u):
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
        explorer.echo(f"[plan] เป้า #{t['id']} {t['color']}/{t['shape']} ช่อง {tuple(t['cell'])} "
                      f"<- ยืนยิงที่ {cell} หัน {DIRS[d]}  (แผน: " +
                      " -> ".join(f"#{p[0]['id']}@{p[1]}" for p in plan) + ")")
        if not explorer.go_to(cell):
            t["skip"] = True
            continue
        if not hunter.done(t):
            hunter.engage_dir(d, relaxed=relaxed, min_views=min_views)
            if not hunter.done(t):
                t["skip"] = True                     # ยิงไม่ได้จากช่องนี้ ข้าม
        hunter.io.point_dir(r.body_heading)
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
