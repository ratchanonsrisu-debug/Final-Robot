"""
robots.py — RealRobot (ต่อหุ่นจริงผ่าน robomaster SDK) และ SimRobot (จำลอง)

Interface ที่ explorer.py / gui.py ใช้ร่วมกัน (ทั้งสองคลาสต้องมีครบ):
    .cell            (x, y) ช่องปัจจุบัน (grid)
    .heading         ทิศที่ถือว่าหุ่น "หันอยู่" ตอนนี้ (0=N,1=E,2=S,3=W)
    .off             [dx, dy] เมตร - ค่าชดเชยตำแหน่งจริงเทียบกับกึ่งกลางช่อง (ใช้ log/แสดงผล)
    .est_xy()        ตำแหน่งประมาณการ (เมตร, พิกัดโลก) = กึ่งกลางช่องปัจจุบัน + off
    .odom_xy()       ตำแหน่งดิบจาก odometry (เมตร) หรือ None
    .yaw()           มุม yaw ปัจจุบัน (องศา) หรือ None
    .scan_all()      หมุน gimbal อ่าน ToF ครบ 4 ทิศโลก -> {0: mm, 1: mm, 2: mm, 3: mm}
    .read_dir(d)     อ่าน ToF ทิศโลก d ทิศเดียว (ใช้ยืนยัน/โหวตซ้ำ)
    .move_cell(d)    เดินหนึ่งช่องไปทิศโลก d
    .correct_yaw()   แก้มุมตัวถังกลับสู่ทิศอ้างอิงด้วย IMU (เรียกตอนไม่ได้สแกนซ้ำ)
    .shift(dx, dy)   ขยับเล็กน้อย (เมตร, พิกัดโลก) ใช้ตอน recenter เท่านั้น
    .set_pose(x,y,heading)   บอกโปรแกรมว่าหุ่นอยู่ช่องไหนหันทางไหนจริง ๆ (manual)
    .close()         ปิดการเชื่อมต่อ/เก็บกวาด
"""
import math
import statistics
import time

import config as C
from grid_map import DIRS, DX, DY, WALL, dir_index

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError:
    robot = None


def _ang_diff(target, cur):
    """บีบผลต่างมุม (องศา) ให้อยู่ในช่วง [-180, 180)"""
    return (target - cur + 180.0) % 360.0 - 180.0


# ======================================================================
# SimRobot — จำลองการเคลื่อนที่/เซนเซอร์ ไม่ต้องต่อหุ่นจริง
# ======================================================================
class SimRobot:
    def __init__(self, world, sx, sy, heading, cell_m, seed=None):
        import random
        self.world = world                 # GridMap = เขาวงกตความจริง (ground truth)
        self.cell = (sx, sy)
        self.heading = dir_index(heading)
        self.cell_m = cell_m
        self.off = [0.0, 0.0]
        self._rnd = random.Random(seed)
        self._yaw = 0.0
        self.body_heading = self.heading   # strafe: ตัวถังหันทิศเดิมตลอด (กล้องจำลองใช้)
        self.on_look = None                # callback(d) หลังสแกน ToF ทิศ d (TargetHunter.look)

    # ---------- pose ----------
    def est_xy(self):
        x, y = self.cell
        return ((x + 0.5) * self.cell_m + self.off[0], (y + 0.5) * self.cell_m + self.off[1])

    def odom_xy(self):
        return self.est_xy()

    def odom_world(self):
        return self.est_xy()

    def yaw(self):
        return self._yaw

    def set_pose(self, cx, cy, heading):
        self.cell = (cx, cy)
        self.heading = dir_index(heading)
        self.body_heading = self.heading
        self.off = [0.0, 0.0]
        self.odo_ref = [None, None]

    # ---------- sensors ----------
    def _true_open_distance_mm(self, x, y, d):
        """float: ระยะจำลองจริงจากกึ่งกลางช่องปัจจุบันถึงกำแพงถัดไปตามทิศ d โดยเดินไล่
        ทีละช่องในเขาวงกตความจริง (ไม่ใช่แค่ดูช่องติดกันช่องเดียว) - ให้ค่าลดลงจริงเมื่อ
        เดินเข้าใกล้ ไม่งั้นเช็ค "เดินไปจริงไหม" ฝั่ง explorer.step() จะเข้าใจผิดว่าไม่ขยับ
        เพราะเปิดโล่งแบบเดิมคืนค่าคงที่ไม่ว่าจะยืนตรงไหนก็ตาม"""
        cells, cx, cy = 0, x, y
        while cells < 20:
            if self.world.state(cx, cy, d) == WALL:
                break
            nx, ny = cx + DX[d], cy + DY[d]
            if not self.world.in_bounds(nx, ny):
                break
            cx, cy = nx, ny
            cells += 1
        return (cells + 0.5) * self.cell_m * 1000.0

    def read_dir(self, d):
        x, y = self.cell
        is_wall = self.world.state(x, y, d) == WALL
        # เยื้องไปทาง d มาก -> ระยะเหลือน้อยลง (จำลองผลของการไม่อยู่กึ่งกลางช่อง)
        axis_off = self.off[1] if d in (0, 2) else self.off[0]
        sign = 1.0 if d in (0, 1) else -1.0
        lean_mm = -sign * axis_off * 1000.0
        if is_wall:
            base = C.WALL_EXPECT_MM + lean_mm
        else:
            base = min(self._true_open_distance_mm(x, y, d), C.TOF_MAX_VALID_MM - 50) + lean_mm
        mm = base + self._rnd.gauss(0, C.SIM_TOF_NOISE_MM)
        if self._rnd.random() < C.SIM_OUTLIER_P:
            mm = self._rnd.uniform(C.TOF_BLIND_ZONE_MM, C.TOF_MAX_VALID_MM)
        return max(C.TOF_BLIND_ZONE_MM, min(C.TOF_MAX_VALID_MM, mm))

    def scan_all(self):
        out = {}
        for d in range(4):
            out[d] = self.read_dir(d)
            if self.on_look:
                self.on_look(d, out[d])
        return out

    def read_sharp(self, d):
        return None             # จำลองไม่มี Sharp -> explorer ใช้ ToF อย่างเดียว

    def scan_all_detailed(self):
        """dict: {d: {"median": mm, "samples": [mm, ...]}} - เหมือน RealRobot เพื่อให้
        gui.py เรียกได้เหมือนกันไม่ต้องแยกเคส sim/real"""
        out = {}
        for d in range(4):
            samples = [self.read_dir(d) for _ in range(C.TOF_SAMPLES)]
            out[d] = {"median": statistics.median(samples), "samples": samples}
        return out

    # ---------- movement ----------
    def move_cell(self, d, dist_m=None):
        # odometry จำลอง = ระยะที่สั่ง (ช่วงเร็ว) + ที่ shift ตามแนวเดิน ; ตำแหน่งจริงจำลองยังลงกลางช่อง + noise
        self._move_d, self._along = d, (dist_m if dist_m is not None else self.cell_m)
        time.sleep(C.SIM_STEP_DELAY_S)
        nx, ny = self.cell[0] + DX[d], self.cell[1] + DY[d]
        self.cell = (nx, ny)
        self.heading = d
        self.off = [self._rnd.gauss(0, C.SIM_MOVE_NOISE_M),
                    self._rnd.gauss(0, C.SIM_MOVE_NOISE_M)]

    def correct_yaw(self):
        self._yaw = 0.0

    def drive_cell(self, d, dist_m, lat_dir=None, odo_k=1.0, side_walls=None, front_wall=False):
        """จำลอง: เหมือน move_cell (ไม่มีล้อลื่นให้คุม)"""
        self.move_cell(d, dist_m)

    def shift(self, dx, dy, speed=None):
        d = getattr(self, "_move_d", None)
        if d is not None:
            self._along += dx * DX[d] + dy * DY[d]
        time.sleep(0.02)

    def reset_heading_ref(self):
        self._yaw = 0.0

    def undo_last_move(self):
        pass                    # จำลอง: move_cell สำเร็จเสมอ ไม่มีอะไรต้องย้อน

    def moved_along(self, d):
        md = getattr(self, "_move_d", None)
        if md is None:
            return None
        if d == md:
            return self._along
        return -self._along if d == (md + 2) % 4 else 0.0

    def moved_lateral(self, d):
        return 0.0              # จำลอง: ไม่เยื้องด้านข้าง

    def recover_chassis(self):
        pass

    def face(self, heading):
        self._yaw = 0.0
        self.heading = dir_index(heading)

    def close(self):
        pass


# ======================================================================
# RealRobot — ต่อหุ่นจริงผ่าน robomaster SDK
# ======================================================================
class RealRobot:
    #: dict: มุม gimbal (สัมพัทธ์ตัวถัง) ที่ต้องหมุนไป เมื่อทิศโลกที่ต้องการมองห่าง
    #: จาก "หน้าตัวถัง" ไป rel ก้าว (0=หน้า,1=ขวา,2=หลัง,3=ซ้าย) - เครื่องหมายขวา/ซ้าย
    #: มาจาก C.GIMBAL_RIGHT_YAW (เดิมเขียนตายตัว ขวา=-90 ซึ่งสลับด้านกับหุ่นจริง)
    _GIMBAL_REL_YAW = {0: 0, 1: C.GIMBAL_RIGHT_YAW, 2: 180, 3: -C.GIMBAL_RIGHT_YAW}

    def __init__(self, sx, sy, heading, cell_m, log=print):
        if robot is None:
            raise RuntimeError("ไม่พบไลบรารี robomaster (pip install robomaster)")
        self.log = log
        self.cell = (sx, sy)
        self.heading = dir_index(heading)
        #: int: ทิศที่ "ตัวถังจริง ๆ" หันอยู่ในโลก - คงที่ตลอดถ้า MOVE_MODE="strafe"
        #: (เดินข้างด้วยล้อ mecanum โดยไม่หมุนตัว) เปลี่ยนเฉพาะตอน MOVE_MODE="rotate"
        self.body_heading = self.heading
        self.cell_m = cell_m
        self.off = [0.0, 0.0]

        self._yaw = 0.0
        self._pos = (0.0, 0.0)
        self._tof = [0, 0, 0, 0]
        #: float: มุม yaw จริงของ gimbal ตอนนี้ (จาก sub_angle - สมัครไว้ตลอดทั้ง session)
        #: ใช้ยืนยันว่า gimbal หยุดนิ่งที่มุมเป้าหมายแล้วจริง ก่อนเริ่มอ่าน ToF - ไม่เชื่อแค่
        #: wait_for_completed() อย่างเดียว (เคยสงสัยว่าค่า ToF ผิดปกติเกิดจากเริ่มอ่านตอน
        #: gimbal ยังหมุนไม่หยุด)
        self._gimbal_yaw = 0.0
        self._gimbal_pitch = 0.0
        #: callback(d) หลังอ่าน ToF ทิศ d ใน scan_all() ขณะ gimbal/กล้องยังชี้ทิศนั้น (TargetHunter.look)
        self.on_look = None
        self._att_n = 0         # จำนวน callback attitude/position ที่ได้รับแล้ว
        self._pos_n = 0
        self._tof_n = 0
        self._ang_n = 0
        self._move_start = None  # odometry ตอนเริ่ม move_cell ล่าสุด (ใช้ undo_last_move)
        self._move_start_pos = None  # เหมือนกันแต่ไม่ถูกล้างตอน undo (ใช้ moved_along)
        #: bool: True = อ่าน Sharp ไม่ได้ (สาย/พอร์ตผิด) -> ใช้ ToF อย่างเดียว
        self._sharp_off = False

        self.log("[robot] กำลังเชื่อมต่อ RoboMaster (AP mode) ...")
        self._ep = robot.Robot()
        self._ep.initialize(conn_type="ap")
        self.ensure_free_mode()
        self._ep.gimbal.sub_angle(freq=20, callback=self._on_gimbal_angle)
        time.sleep(0.2)
        self._ep.gimbal.recenter().wait_for_completed()
        time.sleep(0.2)
        #: int: +1 ถ้า gimbal.moveto(yaw=+X) ทำให้มุมที่วัดจริงเพิ่มขึ้น, -1 ถ้ากลับด้าน
        #: (coordinate mode ของ moveto() ไม่รับประกันเครื่องหมายเดียวกับที่คาดไว้เสมอ
        #: ต้องวัดจริงจากหุ่นแต่ละตัว ไม่งั้นทิศ E/W ที่สแกนได้จะสลับซ้าย-ขวากัน)
        self.gimbal_yaw_sign = self._calibrate_gimbal_yaw_sign()

        self._ep.sensor.sub_distance(freq=C.TOF_FREQ, callback=self._on_tof)
        self._ep.chassis.sub_attitude(freq=C.CHASSIS_FREQ, callback=self._on_attitude)
        self._ep.chassis.sub_position(freq=C.CHASSIS_FREQ, callback=self._on_position)
        # รอให้ IMU/odometry ส่งค่าแรกมาจริงก่อนเก็บจุดอ้างอิง (log 27/9 20:46: รอแค่ 0.3s
        # แล้ว callback ยังไม่มา -> _yaw0 = 0 ทั้งที่หุ่นอยู่ -92.7 -> correct_yaw หมุนแก้ 30 องศา
        # ทุกก้าวจนเดินเฉียงชนกำแพง)
        deadline = time.time() + 3.0
        while (self._att_n < 2 or self._pos_n < 2) and time.time() < deadline:
            time.sleep(0.05)
        if self._att_n < 2 or self._pos_n < 2:
            self.log("[robot] !! ยังไม่ได้ค่า IMU/odometry จากหุ่น - มุมอ้างอิงอาจผิด ให้ต่อหุ่นใหม่")
        #: จุดอ้างอิง yaw/odometry ตอนเริ่ม (หรือหลัง set_pose ล่าสุด) - correct_yaw()
        #: จะดึงมุมกลับมาที่ self._yaw0 เสมอ, odom_xy() รายงานเทียบจาก self._pos0
        self._yaw0 = self._yaw
        self._pos0 = self._pos
        self._odom_h0 = self.body_heading
        self._odom_yaw_ref = self._yaw
        self.log("[robot] เชื่อมต่อสำเร็จ พร้อมใช้งาน")

    # ---------- callbacks (เธรดของ DDS) ----------
    # ทุก callback นับจำนวนครั้ง (*_n) ไว้ -> โค้ดที่อ่านค่าหลังสั่งเคลื่อนที่ "รอค่าใหม่" จริง ไม่ใช่ sleep ตายตัว
    # (AS2 เปิดวิดีโอกล้องด้วย: log 30/9 ค่า ToF/IMU มาช้า -> ToF ทิศ S อ่านได้ค่าของทิศ E ที่อ่านก่อนหน้า
    #  E=3000 9/14 ช่อง ทั้งที่ SLAM เดิม (ไม่เปิดกล้อง) เป็น 0)
    def _on_tof(self, info):
        self._tof = list(info)
        self._tof_n += 1

    def _on_gimbal_angle(self, info):
        self._gimbal_pitch = info[0]
        self._gimbal_yaw = info[1]
        self._ang_n += 1

    def _wait_fresh(self, attr, n=2, timeout=1.0):
        """รอ callback ใหม่ของ attr ("_tof_n"/"_att_n"/"_pos_n"/"_ang_n") อีก n ครั้ง -> True ถ้าได้ทันเวลา"""
        target = getattr(self, attr) + n
        deadline = time.time() + timeout
        while getattr(self, attr) < target:
            if time.time() > deadline:
                return False
            time.sleep(0.005)
        return True

    def _wait_still(self, timeout=None, tol_m=0.003, n=3):
        """รอจน odometry นิ่ง (ค่าใหม่ n ครั้งติดกันขยับ < tol_m) = หุ่นหยุดจริง -> ระยะ (เมตร) ที่ไหลไประหว่างรอ
        (log 30/9 14:11: chassis.move รายงานเสร็จแล้วหุ่นยังไหลต่อ 110-140mm ก่อนอ่าน ToF -> ชนกำแพง/ตำแหน่งผิด)"""
        start = self._pos
        last_n, last = self._pos_n, self._pos
        still = 0
        deadline = time.time() + (C.STILL_TIMEOUT_S if timeout is None else timeout)
        while time.time() < deadline:
            if self._pos_n != last_n:
                last_n, p = self._pos_n, self._pos
                still = still + 1 if math.hypot(p[0] - last[0], p[1] - last[1]) < tol_m else 0
                last = p
                if still >= n:
                    break
            time.sleep(0.005)
        return math.hypot(self._pos[0] - start[0], self._pos[1] - start[1])

    def rates(self, sec=1.0):
        """Hz ของข้อมูลแต่ละชนิดในช่วง sec วินาที (ใช้ตรวจว่าเปิดกล้องแล้วข้อมูลเซนเซอร์ช้าลงไหม)"""
        keys = ("_tof_n", "_att_n", "_pos_n", "_ang_n")
        a = [getattr(self, k) for k in keys]
        time.sleep(sec)
        return {k[1:-2]: (getattr(self, k) - v) / sec for k, v in zip(keys, a)}

    def _on_attitude(self, info):
        self._yaw = info[0]
        self._att_n += 1

    def _on_position(self, info):
        self._pos = (info[0], info[1])
        self._pos_n += 1

    # ---------- pose ----------
    def est_xy(self):
        x, y = self.cell
        return ((x + 0.5) * self.cell_m + self.off[0], (y + 0.5) * self.cell_m + self.off[1])

    def odom_xy(self):
        return (self._pos[0] - self._pos0[0], self._pos[1] - self._pos0[1])

    def odom_world(self):
        """(ตะวันออก, เหนือ) เมตร จาก odometry — กรอบ odometry คือกรอบตอน "เปิดหุ่น" ไม่ใช่
        ตอนเริ่มสำรวจ ต้องหมุนด้วยมุม IMU ตอนตั้ง _pos0 ก่อน (log 28/9 09:51: เปิดหุ่นหันไป 138°
        เดินหน้า -> odom (-0.50, +0.44) ; เดิมไม่หมุน odom hold เลยใช้ไม่ได้ทั้งรอบ)"""
        dx, dy = self.odom_xy()
        a = math.radians(self._odom_yaw_ref)
        f = dx * math.cos(a) + dy * math.sin(a)          # หน้าตัวถัง (ตอนตั้ง _pos0)
        r = -dx * math.sin(a) + dy * math.cos(a)         # ขวาตัวถัง
        f, r = f / C.ODOM_SCALE_FWD, r / C.ODOM_SCALE_STRAFE
        h = self._odom_h0
        r_h = (h + 1) % 4
        return (f * DX[h] + r * DX[r_h], f * DY[h] + r * DY[r_h])

    def yaw(self):
        return self._yaw

    def set_pose(self, cx, cy, heading):
        """ผู้ใช้ยืนยันเองว่าหุ่นอยู่ช่องไหนหันทางไหนจริง ๆ (ตั้งจุดอ้างอิงใหม่)"""
        self.cell = (cx, cy)
        self.heading = dir_index(heading)
        self.body_heading = self.heading
        self.off = [0.0, 0.0]
        self._yaw0 = self._yaw
        self._pos0 = self._pos
        self._odom_h0 = self.body_heading
        self._odom_yaw_ref = self._yaw
        self.odo_ref = [None, None]

    # ---------- ToF / gimbal ----------
    def _calibrate_gimbal_yaw_sign(self):
        """หมุน gimbal ทดสอบสั้น ๆ วัดจาก sub_angle() จริงว่า +yaw ทำให้มุมเพิ่มไปทางไหน
        (พอร์ตมาจาก Classwork8/SLAM.py ที่ยืนยันแล้วว่า moveto() ใช้ coordinate mode
        คนละแบบกับ move()/drive_speed() เครื่องหมาย +/- ไม่รับประกันตรงกันเสมอ)

        ลองซ้ำได้สูงสุด 3 ครั้ง และ**เชื่อผลก็ต่อเมื่อขนาดมุมที่เปลี่ยนจริงใกล้เคียงกับที่สั่งไป
        มากพอ** (ไม่ใช่แค่ดูเครื่องหมาย) - เจอจริงจากการทดสอบว่าบางครั้งอ่านค่าตอน gimbal
        ยังเคลื่อนที่ไม่นิ่ง (เช่นสั่ง +30 แต่วัดได้แค่ -18) ถ้าใช้ค่านั้นตรงๆ จะได้ sign ผิด
        ใช้ self._gimbal_yaw ที่สมัคร sub_angle() ไว้แล้วตั้งแต่ __init__ (ไม่ต้องสมัครเอง)
        """
        test_yaw = 30.0
        try:
            for attempt in range(3):
                self._ep.gimbal.moveto(pitch=0, yaw=0, yaw_speed=60).wait_for_completed()
                time.sleep(0.3)
                start = self._gimbal_yaw
                self._ep.gimbal.moveto(pitch=0, yaw=test_yaw, yaw_speed=60).wait_for_completed()
                time.sleep(0.3)
                end = self._gimbal_yaw
                self._ep.gimbal.moveto(pitch=0, yaw=0, yaw_speed=60).wait_for_completed()
                delta = end - start
                if abs(abs(delta) - test_yaw) <= test_yaw * 0.3:
                    sign = 1 if delta > 0 else -1
                    self.log(f"[gimbal] yaw sign = {sign:+d} (รอบ {attempt + 1}/3: "
                             f"สั่ง moveto(yaw=+{test_yaw:.0f}) -> มุมเปลี่ยนจริง {delta:+.1f}°)")
                    return sign
                self.log(f"[gimbal] รอบ {attempt + 1}/3 ได้ค่าไม่น่าเชื่อถือ (เปลี่ยนจริง "
                         f"{delta:+.1f}° ต่างจากที่สั่ง {test_yaw:.0f}° มากเกินไป) ลองใหม่...")
                time.sleep(0.3)
        except Exception as e:                                  # noqa: BLE001
            self.log(f"[gimbal] คาลิเบรตทิศ yaw ไม่สำเร็จ ({e}) ใช้ค่าเริ่มต้น +1")
            return 1
        self.log("[gimbal] วัดทิศ yaw ไม่ชัดเจนหลังลอง 3 รอบ (gimbal อาจติดขัด/สั่นมาก) "
                 "ใช้ค่าเริ่มต้น +1 - ควรตรวจ gimbal ก่อนวิ่งจริง")
        return 1

    def _gimbal_yaw_for(self, d):
        """มุม gimbal สัมพัทธ์ตัวถัง (คูณ gimbal_yaw_sign แล้ว) ที่ทำให้ ToF ชี้ไปทิศโลก d"""
        rel = (d - self.body_heading) % 4
        return self.gimbal_yaw_sign * self._GIMBAL_REL_YAW[rel]

    def _goto_gimbal(self, yaw, pitch, tol_deg=3.0, extra_wait_s=1.5, attempts=2):
        """สั่ง gimbal ไปมุมเป้าหมาย แล้ว**ยืนยันจริงจาก self._gimbal_yaw** (sub_angle)
        ว่าไปถึงแล้วก่อนคืนค่า - ไม่เชื่อ wait_for_completed() เพียงอย่างเดียว เพราะเคยสงสัย
        ว่าค่า ToF ที่อ่านได้ "เร็ว/ผิดปกติ" อาจเกิดจากเริ่มอ่านตั้งแต่ gimbal ยังหมุนไม่หยุดจริง
        """
        self._safe_sweep(yaw, pitch)
        # ลองสั่งได้ 2 ครั้ง: log 30/9 16:56 สั่ง yaw=90 แต่ gimbal ค้างที่ 1° -> ToF "E" อ่านระยะทิศ N ไปจัดกึ่งกลางผิดแกน
        for attempt in range(attempts):
            try:
                self._ep.gimbal.moveto(pitch=pitch, yaw=yaw,
                                       yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
            except Exception as e:                          # noqa: BLE001
                self.log(f"[gimbal] moveto(yaw={yaw}) ล้มเหลว: {e}")
                continue
            deadline = time.time() + extra_wait_s
            while time.time() < deadline:
                if abs(self._gimbal_yaw - yaw) <= tol_deg:
                    return True
                time.sleep(0.03)
            self.log(f"[gimbal] สั่งไป yaw={yaw:.0f} แต่มุมจริงยังไม่ถึง (อ่านได้ {self._gimbal_yaw:.1f}°) "
                     f"{'-> สั่งซ้ำ' if attempt < attempts - 1 else '- ไปต่อ ค่าที่ได้อาจไม่แม่น'}")
        return False

    def _safe_sweep(self, yaw, pitch):
        """ก้มอยู่/จะก้ม แล้วต้องหันไกล -> ยก gimbal ขึ้น GIMBAL_PITCH ที่มุมเดิมก่อน แล้วหันที่ระดับนั้น
        (ท่าก้มต่อจาก _goto_gimbal ที่ปลายทาง) ; log 1/10 08:17: ก้ม -15 หัน 45->135 ลำกล้องกวาดผ่าน Sharp ขวา
        เกี่ยวค้างที่ 47° แล้วตัวถังบิด 46° ระหว่างเดินต่อ"""
        safe = C.GIMBAL_PITCH - 3.0
        cur_y, cur_p = self._gimbal_yaw, self._gimbal_pitch
        if abs(yaw - cur_y) <= C.GIMBAL_SAFE_TURN_DEG or min(pitch, cur_p) >= safe:
            return
        try:
            if cur_p < safe:
                self._ep.gimbal.moveto(pitch=C.GIMBAL_PITCH, yaw=round(cur_y, 1),
                                       yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=2.0)
            if pitch < safe:
                self._ep.gimbal.moveto(pitch=C.GIMBAL_PITCH, yaw=yaw,
                                       yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[gimbal] ยกก่อนหันไม่สำเร็จ: {e}")

    def _twisted(self, yaw_start):
        """มุมตัวถังเปลี่ยนจากตอนเริ่มเดิน/shift เกิน YAW_TWIST_ABORT_DEG -> คืนค่ามุม (ไม่งั้น None)"""
        dy = _ang_diff(yaw_start, self._yaw)
        return dy if abs(dy) > C.YAW_TWIST_ABORT_DEG else None

    def _halt_twist(self, where, dy):
        self.halt = (f"ตัวถังหมุนเอง {abs(dy):.0f}° ระหว่าง {where} (gimbal เกี่ยว Sharp/โดนชน?) -> หยุดล้อ หยุดสำรวจ "
                     f"กันเดินเฉียงชน ; จัดหุ่นให้ตรงแล้วเริ่มใหม่")
        self.log("[chassis] !! " + self.halt)

    @staticmethod
    def _interpret_tof(raw):
        """int: แปลงค่า ToF ดิบให้ตีความถูกต้อง (พอร์ตมาจาก Classwork8/SLAM.py ที่เจอบั๊กนี้
        มาก่อนแล้วจริง - ดู docstring หัวไฟล์นั้นข้อ 7)

        ``raw <= 0`` หรือต่ำกว่า ``TOF_BLIND_ZONE_MM`` = ชิดวัตถุมากจนสะท้อนไม่ทัน (blind
        zone) = **กำแพงชัวร์เสมอ** ห้ามทิ้งค่านี้ไปเฉยๆ (เวอร์ชันก่อนหน้าเคยทิ้งค่านอกช่วง
        [40,3000] แล้ว fallback เป็น 3000 (=โล่งสุด) ทำให้ตอนเอากำแพงเข้าใกล้จนสะท้อนไม่ทัน
        กลับอ่านได้ว่าโล่งที่สุดแทน - ตรงข้ามความจริงเป๊ะ) ``raw`` สูงเกิน ``TOF_MAX_VALID_MM``
        ก็เป็นค่าที่ใช้ได้ปกติ (แปลว่าโล่งจริง) แค่ตัดเพดานไว้ไม่ให้ตัวเลขประหลาด
        """
        if raw <= 0 or raw < C.TOF_BLIND_ZONE_MM:
            return C.TOF_BLIND_ZONE_MM
        # ค่าดิบ -> mm จริง (สูตรจาก sensor_calib.py - ดู TOF_CAL_* ใน config.py)
        return min(C.TOF_CAL_K * raw + C.TOF_CAL_C, C.TOF_MAX_VALID_MM)

    def _sample_dir(self, d):
        """(float, list): (median, ตัวอย่างดิบที่ตีความแล้วทุกตัว) ของทิศโลก d - แยกจาก
        read_dir() เพื่อให้ scan_all_detailed() เอาตัวอย่างดิบไปโชว์ได้โดยไม่ต้องสแกนซ้ำ"""
        yaw = self._gimbal_yaw_for(d)
        rel = (d - self.body_heading) % 4
        pitch = C.GIMBAL_PITCH_SIDE if rel in (1, 3) else C.GIMBAL_PITCH
        self._goto_gimbal(yaw, pitch)
        time.sleep(C.TOF_SETTLE_S)
        samples, stable = self._fresh_tof_samples()
        if not stable:
            self.log(f"[tof] ทิศ {DIRS[d]} ค่าไม่นิ่ง/มาไม่ทัน {[round(v) for v in samples]} - ใช้ค่ากลาง")
        return float(statistics.median(samples)), samples

    def _fresh_tof_samples(self):
        """อ่าน ToF เฉพาะค่าที่ "มาใหม่" หลัง gimbal หยุด (นับ callback) จนได้ TOF_SAMPLES ค่าที่นิ่ง
        (ตัดค่าสูงสุด/ต่ำสุดแล้วห่างกัน ≤ max(TOF_STABLE_MM, 6%)) ; ค่าที่ค้างมาจากตอน gimbal ยังหมุน
        จะไม่นิ่ง -> ถูกเลื่อนทิ้งไปเอง  -> (ตัวอย่าง, นิ่งไหม)"""
        n = C.TOF_SAMPLES
        got, last = [], self._tof_n
        deadline = time.time() + C.TOF_FRESH_TIMEOUT_S
        while time.time() < deadline:
            if self._tof_n != last:
                last = self._tof_n
                got.append(self._interpret_tof(self._tof[C.TOF_INDEX]))
                if len(got) >= n:
                    tail = sorted(got[-n:])
                    core = tail[1:-1] if n >= 4 else tail
                    if core[-1] - core[0] <= max(C.TOF_STABLE_MM, 0.06 * statistics.median(tail)):
                        return got[-n:], True
            time.sleep(0.005)
        if not got:                     # ไม่มีค่าใหม่มาเลย -> ใช้ค่าล่าสุดที่มี
            got = [self._interpret_tof(self._tof[C.TOF_INDEX])]
        return got[-n:], False

    def read_dir(self, d):
        median, _ = self._sample_dir(d)
        return median

    @staticmethod
    def _sharp_adc_to_mm(adc, table):
        """ADC -> mm จริงด้วยตาราง calibrate (piecewise linear), นอกช่วง: ใกล้กว่า 10cm
        คืนค่า mm ที่จุดใกล้สุด (ยังเป็นกำแพง), ไกลกว่า 30cm คืน SHARP_FAR_MM"""
        pts = sorted(table)                     # ADC น้อย (ไกล) -> มาก (ใกล้)
        if adc < pts[0][0]:
            return C.SHARP_FAR_MM
        if adc >= pts[-1][0]:
            return pts[-1][1]
        for (a0, m0), (a1, m1) in zip(pts, pts[1:]):
            if a0 <= adc <= a1:
                return m0 + (adc - a0) * (m1 - m0) / (a1 - a0)
        return C.SHARP_FAR_MM

    def read_sharp(self, d):
        """(mm, expect_mm) ของ Sharp ที่ชี้ไปทิศโลก d หรือ None ถ้าไม่มี Sharp ชี้ทิศนั้น
        (Sharp ติดตัวถัง: ซ้าย = rel 3, ขวา = rel 1 เทียบ body_heading)"""
        if not C.USE_SHARP or self._sharp_off:
            return None
        rel = (d - self.body_heading) % 4
        if rel == 1:
            port, table, expect = C.SHARP_RIGHT, C.SHARP_R_TABLE, C.SHARP_R_EXPECT_MM
        elif rel == 3:
            port, table, expect = C.SHARP_LEFT, C.SHARP_L_TABLE, C.SHARP_L_EXPECT_MM
        else:
            return None
        vals = []
        for _ in range(C.SHARP_SAMPLES):
            try:
                v = self._ep.sensor_adaptor.get_adc(id=port[0], port=port[1])
            except Exception as e:                      # noqa: BLE001
                self.log(f"[sharp] อ่าน hub {port} ไม่ได้: {e} - ปิดใช้ Sharp ใช้ ToF อย่างเดียว")
                self._sharp_off = True
                return None
            if v is not None:
                vals.append(v)
        if not vals:
            return None
        return self._sharp_adc_to_mm(statistics.median(vals), table), expect

    def scan_all(self):
        out = {}
        for d in range(4):
            out[d] = self.read_dir(d)
            if self.on_look:
                try:
                    self.on_look(d, out[d])  # กล้องอยู่บน gimbal เดียวกับ ToF -> หาป้ายทิศนี้ไปด้วยเลย
                except Exception as e:                      # noqa: BLE001
                    self.log(f"[look] หาป้ายทิศ {DIRS[d]} ผิดพลาด: {e!r}")
        try:
            self._ep.gimbal.moveto(pitch=C.GIMBAL_PITCH, yaw=0,
                                   yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[gimbal] recenter ล้มเหลว: {e}")
        return out

    def scan_all_detailed(self):
        """dict: {d: {"median": mm, "samples": [mm, ...]}} - เหมือน scan_all() แต่ให้
        ตัวอย่างดิบทุกตัวติดมาด้วย ใช้ตอน Calibrate/debug เพื่อดูว่าค่าที่วัดได้แต่ละทิศ
        นิ่งแค่ไหน (ไม่ใช้ระหว่างสำรวจปกติ กัน log รกเกินไป)"""
        out = {}
        for d in range(4):
            median, samples = self._sample_dir(d)
            out[d] = {"median": median, "samples": samples}
        try:
            self._ep.gimbal.moveto(pitch=C.GIMBAL_PITCH, yaw=0,
                                   yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[gimbal] recenter ล้มเหลว: {e}")
        return out

    # ---------- yaw correction ----------
    def correct_yaw(self):
        """แก้มุมตัวถังกลับสู่ทิศอ้างอิงทีละนิด มีเพดานต่อครั้งเสมอ (กันไม่ให้ค่า yaw

        ที่อ่านเพี้ยนชั่วขณะ (เช่น IMU กระตุกตอนหุ่นสั่น) สั่งหมุนมุมใหญ่ๆ แบบควบคุมไม่ได้
        - เวอร์ชันก่อนหน้าเคยลอง "สลับ YAW_SIGN แล้วหมุนแก้ซ้ำทันทีในคอลเดียว" ซึ่งถ้าอ่านค่า
        ผิดจังหวะจะยิ่งหมุนเพิ่มจนหมุนเลยไปเกือบ 180 องศาโดยไม่ตั้งใจ (เกิดขึ้นจริงระหว่างทดสอบ)
        เวอร์ชันนี้จึงแก้ได้แค่ครั้งละไม่เกิน YAW_CORRECT_MAX_DEG และถ้าแก้แล้วเบี้ยวขึ้น จะแค่
        หยุดรอบนี้ไว้ก่อน (ไม่หมุนซ้ำทันที) ให้รอบถัดไปอ่านค่าสดใหม่มาตัดสินใจแทน
        """
        self._wait_fresh("_att_n", 2, 1.0)          # มุมหลังเดิน/หมุนล่าสุดต้องเป็นค่าใหม่
        err = _ang_diff(self._yaw0, self._yaw)
        if abs(err) < C.YAW_TOLERANCE_DEG:
            return
        if abs(err) > C.YAW_SANE_MAX_DEG:
            # มุมเพี้ยนขนาดนี้ระหว่างก้าวแทบเป็นไปไม่ได้ = มุมอ้างอิงผิด ไม่ใช่หุ่นเบี้ยว
            # ห้ามหมุนแก้ (หมุนแก้ตามค่าอ้างอิงผิดคือสาเหตุหุ่นเดินเฉียงชนกำแพง)
            self.log(f"[yaw] !! มุมต่างจากอ้างอิง {err:+.1f}° เกิน {C.YAW_SANE_MAX_DEG}° - ไม่หมุนแก้ "
                     f"(มุมอ้างอิงน่าจะผิด) วางหุ่นตรงแล้วเริ่มสำรวจใหม่")
            return
        step = max(-C.YAW_CORRECT_MAX_DEG, min(C.YAW_CORRECT_MAX_DEG, err))
        self._rotate_raw(C.YAW_SIGN * step)
        time.sleep(0.1)
        self._wait_fresh("_att_n", 3, 1.0)          # ห้ามตัดสินจากค่า yaw ก่อนหมุน (ค้างมา)
        new_err = _ang_diff(self._yaw0, self._yaw)
        if abs(new_err) > abs(err) + 1.0:
            self.log(f"[yaw] แก้มุมแล้วเบี้ยวขึ้นแทนที่จะลดลง ({err:+.1f}° -> {new_err:+.1f}°) "
                     f"- หยุดแก้รอบนี้ไว้ก่อน ถ้าเกิดซ้ำบ่อยให้ลองตั้ง YAW_SIGN = {-C.YAW_SIGN} "
                     f"ใน config.py เอง (ไม่ใช้วิธีสลับอัตโนมัติแล้ว เพราะเคยทำให้หมุนเลยเถิด)")

    def ensure_free_mode(self):
        """โหมด FREE: gimbal หมุนอิสระจากตัวถัง (โหมด gimbal_lead ตัวถังจะหมุนตาม gimbal ; log 1/10 รอบ 2:
        สั่ง gimbal 180° แล้วมุมจริงค่อยๆ ตาม ตัวถังหมุนไปเอง -86° ระหว่างเดิน -> สั่งโหมดให้ชัดทุกครั้งที่เริ่มรอบ)"""
        try:
            self._ep.set_robot_mode(mode=robot.FREE)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[robot] ตั้งโหมด FREE ไม่สำเร็จ: {e}")

    def reset_heading_ref(self):
        """ใช้มุมตอนนี้เป็นทิศอ้างอิง (เรียกตอนเริ่มสำรวจ - ผู้ใช้วางหุ่นตรงกับกริดแล้ว)"""
        self.ensure_free_mode()
        self.halt = None
        self._yaw0 = self._yaw
        self._yaw_start = self._yaw
        self._body_start = self.body_heading
        self.log(f"[yaw] ตั้งมุมอ้างอิง = {self._yaw0:.1f}°")

    def face(self, heading):
        """จอด: หันตัวถังกลับทิศเดียวกับตอนเริ่มสำรวจ + gimbal ชี้หน้า (ใช้หลังกลับถึงจุดเริ่ม)
        strafe: ตัวถังไม่เคยหมุน แค่แก้ yaw ให้ตรงมุมอ้างอิงเดิม
        rotate: หมุนตัวถังกลับทิศเริ่ม แล้วใช้มุม IMU ตอนเริ่มเป็นอ้างอิง (ไม่สะสม error จากการเลี้ยว)"""
        h = dir_index(heading)
        if C.MOVE_MODE == "rotate" and h != self.body_heading:
            deg = {1: -90.0, 2: 180.0, 3: 90.0}[(h - self.body_heading) % 4]
            self._rotate_raw(deg)
            self.body_heading = h
            self._yaw0 = self._yaw
        if getattr(self, "_body_start", None) == self.body_heading:
            self._yaw0 = self._yaw_start
        for _ in range(3):
            if abs(_ang_diff(self._yaw0, self._yaw)) < C.YAW_TOLERANCE_DEG:
                break
            self.correct_yaw()
            time.sleep(0.2)
        try:
            self._ep.gimbal.moveto(pitch=C.GIMBAL_PITCH, yaw=0,
                                   yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[gimbal] recenter ล้มเหลว: {e}")
        self.heading = h

    def undo_last_move(self):
        """ถอยกลับตามระยะที่ odometry วัดได้จริงตั้งแต่เริ่ม move_cell ล่าสุด
        ใช้เมื่อเดินไม่สำเร็จ (ไม่ใช่ถอยตามคำสั่ง เพราะหุ่นอาจขยับไปคนละทางกับที่สั่ง)"""
        if self._move_start is None:
            return
        dx = self._pos[0] - self._move_start[0]
        dy = self._pos[1] - self._move_start[1]
        self._move_start = None
        if math.hypot(dx, dy) < 0.03:
            return
        # odometry (กรอบตอนเปิดเครื่อง) -> กรอบตัวถังตอนนี้ (x หน้า, y ขวา, yaw ตามเข็ม = บวก)
        # ตรวจกับ log 27/9 20:47: yaw -89, เดินหน้า -> odom -y ถูกต้อง
        a = math.radians(self._yaw)
        bx = math.cos(a) * dx + math.sin(a) * dy
        by = -math.sin(a) * dx + math.cos(a) * dy
        scale = min(1.0, 0.7 / math.hypot(bx, by))
        self.log(f"[undo] เดินไม่สำเร็จ -> ถอยกลับตาม odometry ({-bx * scale:+.2f}, {-by * scale:+.2f}) m")
        try:
            self._cancel_speed_timer()
            self._ep.chassis.move(x=-bx * scale, y=-by * scale, z=0,
                                  xy_speed=C.SHIFT_SPEED).wait_for_completed(timeout=6.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[undo] ถอยกลับล้มเหลว: {e}")
        self._wait_still()

    def _cancel_speed_timer(self):
        """drive_speed(timeout=) ของ SDK ตั้ง timer ฝั่งคอมไว้สั่ง "หยุด" อีกทีหลัง 0.5 วิ -> ถ้าไปโดนตอน chassis.move
        กำลังทำงาน action จะค้าง/ถูกตัด (log 1/10 06:34: shift 31mm ไม่เสร็จใน 2 วิ 2 ครั้งติด) -> ยกเลิกก่อนเสมอ"""
        t = getattr(self._ep.chassis, "_auto_timer", None)
        if t is not None:
            try:
                t.cancel()
            except Exception:                               # noqa: BLE001
                pass

    def _stop_chassis(self):
        """หยุดล้อทันที: หลัง chassis.move() timeout action ยังค้างดันต่อได้ (log 30/9: ชนแล้วสไลด์ไถลข้างกำแพง)"""
        self._cancel_speed_timer()
        try:
            self._ep.chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(0.15)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] สั่งหยุดล้อไม่สำเร็จ: {e}")

    @staticmethod
    def _move_timeout(dist_m, speed):
        """เวลารอ chassis.move ให้เสร็จ: ระยะ/ความเร็ว + เผื่อเร่ง-เบรก"""
        return max(C.MOVE_TIMEOUT_MIN_S, dist_m / speed * 1.5 + C.MOVE_TIMEOUT_EXTRA_S)

    def recover_chassis(self):
        """หยุดล้อแล้วรอ ให้ action ที่ค้าง (timeout) หลุดก่อนสั่งใหม่"""
        self._stop_chassis()
        time.sleep(C.CHASSIS_RECOVER_S)

    def yaw_ok(self):
        """มุมตัวถังยังใกล้มุมอ้างอิง (odometry แปลงทิศได้ถูก)"""
        return abs(_ang_diff(self._yaw0, self._yaw)) <= C.ODOM_YAW_MAX_DEG

    def moved_along(self, d):
        """ระยะ (เมตร) ที่ odometry บอกว่าเดินไปตามทิศโลก d ตั้งแต่เริ่ม move_cell ล่าสุด (None = ไม่รู้)
        มุมตัวถังเพี้ยนเกิน ODOM_YAW_MAX_DEG -> None (แปลงแกนผิด ห้ามใช้ตัดสิน)"""
        if self._move_start_pos is None or not self.yaw_ok():
            return None
        dx = self._pos[0] - self._move_start_pos[0]
        dy = self._pos[1] - self._move_start_pos[1]
        a = math.radians(self._yaw)                  # กรอบ odometry -> กรอบตัวถัง (เหมือน undo_last_move)
        bx = math.cos(a) * dx + math.sin(a) * dy     # หน้า
        by = -math.sin(a) * dx + math.cos(a) * dy    # ขวา
        h, r = self.body_heading, (self.body_heading + 1) % 4
        wx = bx * DX[h] + by * DX[r]
        wy = bx * DY[h] + by * DY[r]
        return wx * DX[d] + wy * DY[d]

    def moved_lateral(self, d):
        """ระยะ (เมตร) ที่เยื้องไปทาง "ขวาของทิศ d" (ทิศ d+1) ตั้งแต่เริ่ม move_cell ล่าสุด ตาม odometry (None = ไม่รู้)"""
        return self.moved_along((d + 1) % 4)

    def _rotate_raw(self, deg):
        if abs(deg) < 0.3:
            return
        try:
            self._cancel_speed_timer()
            self._ep.chassis.move(x=0, y=0, z=deg, xy_speed=C.XY_SPEED,
                                  z_speed=C.Z_SPEED).wait_for_completed(timeout=6.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] หมุน {deg:.1f} องศา ล้มเหลว: {e}")

    # ---------- movement ----------
    def _world_to_body(self, wx, wy):
        """หมุนเวกเตอร์โลก (wx=ทิศตะวันออก, wy=ทิศเหนือ) เป็นพิกัดตัวถัง
        (bx=หน้า, by=ขวา) ตาม body_heading ปัจจุบัน"""
        h = self.body_heading
        bx = wx * DX[h] + wy * DY[h]
        by = wx * DY[h] - wy * DX[h]
        return bx, by

    def shift(self, dx, dy, speed=None):
        """ขยับ (เมตร, พิกัดโลก) - จัดกึ่งกลาง/เติม/ถอย ; speed=None ใช้ SHIFT_SPEED
        ตรวจด้วย odometry ว่าขยับจริง (log 30/9 19:26: shift timeout แล้วหุ่นไม่ขยับเลย 2 ครั้งติด
        -> จอดขาดกลางช่อง 155mm แล้วเดินต่อ) ; ได้ไม่ถึงครึ่ง -> หยุดล้อ สั่งส่วนที่เหลือซ้ำ ;
        ยังไม่ขยับอีก -> ขับด้วย drive_speed วัด odometry เอง (ไม่พึ่งระบบ action ของ chassis.move)"""
        speed = speed or C.SHIFT_SPEED
        bx, by = self._world_to_body(dx, dy)
        want = math.hypot(bx, by)
        start = self._pos
        self._shift_action(bx, by, speed, f"shift({dx:+.3f},{dy:+.3f})")
        if want < C.SHIFT_VERIFY_MIN_M or not self.yaw_ok():
            return
        ux, uy = bx / want, by / want
        got = self._body_progress(start, ux, uy)
        if got >= C.SHIFT_OK_FRAC * want:
            return
        if got < -0.03:
            # odometry บอกว่าไปสวนทาง = ไหลค้างจากก้าวก่อน/ค่าไม่เชื่อถือ -> ห้ามสั่งเพิ่ม ให้ผู้เรียกวัดใหม่เอง
            # (log 30/9 20:31: ได้ -171mm แล้วสั่งซ้ำ 321mm + drive 288mm = ขยับเกินที่สั่ง 2 เท่า)
            self.log(f"[chassis] shift({dx:+.3f},{dy:+.3f}) odometry บอกขยับสวนทาง {got * 1000:.0f}mm "
                     f"-> ไม่สั่งซ้ำ (วัด ToF ใหม่แทน)")
            return
        rest = want - max(0.0, got)          # ไม่สั่งเกินระยะเดิม
        self.log(f"[chassis] shift({dx:+.3f},{dy:+.3f}) ขยับจริงแค่ {got * 1000:.0f}/{want * 1000:.0f}mm "
                 f"(odometry) -> หยุดล้อ แล้วสั่งส่วนที่เหลือ {rest * 1000:.0f}mm ซ้ำ")
        self.recover_chassis()
        start2 = self._pos
        self._shift_action(ux * rest, uy * rest, speed, "shift ซ้ำ")
        got2 = self._body_progress(start2, ux, uy)
        if got2 >= C.SHIFT_OK_FRAC * rest or got2 < -0.03 or not self.yaw_ok():
            return
        rest2 = rest - max(0.0, got2)
        self.recover_chassis()
        got3 = self._drive_body(ux, uy, rest2)
        self.log(f"[chassis] shift ซ้ำยังไม่ขยับ ({got2 * 1000:.0f}mm) -> ขับ drive_speed "
                 f"{rest2 * 1000:.0f}mm ได้ {got3 * 1000:.0f}mm")

    def _shift_action(self, bx, by, speed, name):
        """chassis.move ครั้งเดียว (พิกัดตัวถัง) + รอหุ่นนิ่ง"""
        tmo = self._move_timeout(math.hypot(bx, by), speed)
        self._cancel_speed_timer()
        try:
            t0 = time.time()
            start = self._pos
            yaw_start = self._yaw
            twist = None
            lim = math.hypot(bx, by) + C.SHIFT_RUNAWAY_MM / 1000.0
            act = self._ep.chassis.move(x=bx, y=C.STRAFE_SIGN * by, z=0, xy_speed=speed, z_speed=C.Z_SPEED)
            ok, runaway = False, 0.0
            # เฝ้า odometry ระหว่างรอ: ขยับรวมเกินที่สั่ง + SHIFT_RUNAWAY_MM (ทิศไหนก็ได้) = หุ่นไหล/วิ่งเอง -> หยุดล้อทันที
            # (log 1/10 06:34 (5,5): สั่ง W 31mm แต่ odometry ไป N 270+180mm ชนกำแพงหน้า)
            while time.time() - t0 < tmo:
                if act._event.wait(0.03) and act.is_completed:
                    ok = True
                    break
                runaway = math.hypot(self._pos[0] - start[0], self._pos[1] - start[1])
                twist = self._twisted(yaw_start)
                if runaway > lim or twist is not None:
                    break
            if not ok:
                act.wait_for_completed(timeout=0.01)        # ให้ SDK ปิด action นี้ (ไม่ค้างขวางคำสั่งถัดไป)
            if twist is not None:
                self._stop_chassis()
                self._halt_twist(name, twist)
            elif runaway > lim:
                self.log(f"[chassis] {name} !! หุ่นขยับไป {runaway * 1000:.0f}mm เกินที่สั่ง "
                         f"{math.hypot(bx, by) * 1000:.0f}mm -> หยุดล้อทันที")
                self._stop_chassis()
            elif not ok:
                self.log(f"[chassis] {name} ไม่ยืนยันว่าสำเร็จใน {tmo:.1f}s (timeout/ถูกปฏิเสธ) -> หยุดล้อ")
                self._stop_chassis()
            elif time.time() - t0 > 0.7 * tmo:
                self.log(f"[time] {name} ใช้ {time.time() - t0:.1f}s (ช้ากว่าปกติ)")
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] {name} ล้มเหลว: {e}")
            self._stop_chassis()
        self._wait_still()

    def _body_progress(self, start, ux, uy):
        """ระยะ (เมตร) ที่ odometry บอกว่าขยับไปตามทิศ (ux=หน้า, uy=ขวา ของตัวถัง) ตั้งแต่ตำแหน่ง start
        (แปลงกรอบเหมือน undo_last_move)"""
        dx = self._pos[0] - start[0]
        dy = self._pos[1] - start[1]
        a = math.radians(self._yaw)
        bx = math.cos(a) * dx + math.sin(a) * dy
        by = -math.sin(a) * dx + math.cos(a) * dy
        return bx * ux + by * uy

    def _drive_body(self, ux, uy, dist):
        """ขับด้วย drive_speed ไปตามทิศตัวถัง (ux, uy) จน odometry ได้ระยะ dist (เมตร) หรือหมดเวลา -> ระยะที่ได้"""
        v = C.SHIFT_DRIVE_SPEED
        start = self._pos
        deadline = time.time() + dist / v * 2.0 + 1.0
        got = 0.0
        try:
            while time.time() < deadline:
                got = self._body_progress(start, ux, uy)
                if got >= dist - 0.01:
                    break
                self._ep.chassis.drive_speed(x=v * ux, y=C.STRAFE_SIGN * v * uy, z=0, timeout=0.5)
                time.sleep(0.05)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] drive_speed ล้มเหลว: {e}")
        self._stop_chassis()
        self._wait_still()
        return self._body_progress(start, ux, uy)

    def move_cell(self, d, dist_m=None):
        """เดินหนึ่งช่องไปทิศโลก d ; dist_m = ระยะที่ต้องเดินจริง (explorer คำนวณจากกำแพงข้างหน้า) None = 1 ช่อง"""
        if C.MOVE_MODE == "rotate" and d != self.body_heading:
            turn_steps = (d - self.body_heading) % 4
            deg = {1: -90.0, 2: 180.0, 3: 90.0}[turn_steps]   # +90=ซ้าย (chassis.move ก้อนเดียว)
            self._rotate_raw(deg)
            self.body_heading = d
            self._yaw0 = self._yaw          # ทิศอ้างอิงใหม่หลังเลี้ยว

        self._move_start = self._pos
        self._move_start_pos = self._pos
        L = self.cell_m if dist_m is None else dist_m
        bx, by = self._world_to_body(DX[d] * L, DY[d] * L)
        bx *= C.MOVE_FACTOR_FWD
        by *= C.MOVE_FACTOR_STRAFE
        ok = False
        # เดิมรอ 8s: log 30/9 timeout 7 ครั้ง/รอบ = เสีย ~56s ทั้งที่เดิน 0.6m ใช้แค่ ~1.3s
        tmo = self._move_timeout(math.hypot(bx, by), C.XY_SPEED)
        t0 = time.time()
        try:
            self._cancel_speed_timer()
            action = self._ep.chassis.move(x=bx, y=C.STRAFE_SIGN * by, z=0,
                                           xy_speed=C.XY_SPEED, z_speed=C.Z_SPEED)
            ok = action.wait_for_completed(timeout=tmo)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] เดินไปทิศ {DIRS[d]} ล้มเหลว: {e}")
        if not ok:
            self._stop_chassis()
            self.log(f"[chassis] เดินไปทิศ {DIRS[d]} ไม่ยืนยันว่าสำเร็จใน {tmo:.1f}s (timeout/ถูกบล็อก) -> สั่งหยุดล้อแล้ว "
                     f"(explorer จะตรวจว่าไปถึงจริงไหมด้วย ToF + odometry)")
        elif time.time() - t0 > 0.7 * tmo:
            self.log(f"[time] เดินไปทิศ {DIRS[d]} ใช้ {time.time() - t0:.1f}s (ช้ากว่าปกติ)")

        drift = self._wait_still()                  # ห้ามอ่าน ToF/ตัดสินอะไรตอนหุ่นยังไหลอยู่
        if drift > 0.03:
            self.log(f"[drift] เดินไปทิศ {DIRS[d]} คำสั่งเสร็จแล้วหุ่นยังไหลต่อ {drift * 1000:.0f}mm (รอจนหยุดก่อนอ่านค่า)")
        self.cell = (self.cell[0] + DX[d], self.cell[1] + DY[d])
        self.heading = d
        self.off = [0.0, 0.0]
        if C.MOVE_MODE != "rotate":
            self.correct_yaw()

    # ---------- เดินแบบคุมตลอดทาง (MOVE_CLOSED_LOOP=True) ----------
    def _sharp_once(self, rel):
        """Sharp ด้าน rel (1=ขวา, 3=ซ้าย เทียบตัวถัง) อ่านครั้งเดียว -> (mm, expect) ; ใช้ไม่ได้ -> None"""
        if not C.USE_SHARP or self._sharp_off:
            return None
        port, table, expect = ((C.SHARP_RIGHT, C.SHARP_R_TABLE, C.SHARP_R_EXPECT_MM) if rel == 1 else
                               (C.SHARP_LEFT, C.SHARP_L_TABLE, C.SHARP_L_EXPECT_MM))
        try:
            v = self._ep.sensor_adaptor.get_adc(id=port[0], port=port[1])
        except Exception:                                   # noqa: BLE001
            return None
        if v is None:
            return None
        return self._sharp_adc_to_mm(v, table), expect

    def drive_cell(self, d, dist_m, lat_dir=None, odo_k=1.0, side_walls=None, front_wall=False):
        """side_walls = {ทิศโลกข้าง: (ช่องนี้, ช่องปลายทาง)} 2=มีกำแพง 1=ยังไม่รู้ 0=โล่ง จากแผนที่ (ToF) ;
        front_wall = ToF ก่อนเดินยืนยันว่ามีกำแพงที่ปลายช่องปลายทาง
        Sharp เชื่อเฉพาะด้านที่แผนที่บอกว่ามีกำแพง + ค่า < CL_SHARP_TRUST_MM (log 30/9 23:26: ไม่มีกำแพง Sharp
        อ่านหลอก 185-250 แทน 999 -> ดันหุ่นไป E ทุกก้าว 100-150mm / Sharp หน้าหลอกทำให้เดินช้าจนเกิน/หมดเวลา)
        เดินไปทิศโลก d ระยะ dist_m ด้วย drive_speed คุมทุกรอบ (แบบ nathon-aie/SLAM ปรับให้เดินสไลด์):
        - เดิน N/S: ToF บน gimbal ชี้ทิศเดิน (หยุดหน้ากำแพง) + Sharp ซ้าย/ขวาคุมกลางแนว E-W ตลอดทาง
        - เดิน E/W: gimbal ชี้ lat_dir (กำแพงด้าน N/S ที่แผนที่รู้) คุมกลางแนว N-S ; Sharp ฝั่งที่เดินไป = ตัวหยุดหน้ากำแพง
        - ระยะ: กำแพงหน้าเห็นแล้ว -> เทียบระยะอ้างอิงตรงๆ ; ไม่งั้น odometry (หาร scale สนาม)
        ช้าลงช่วง CL_DECEL_MM สุดท้าย ; ไม่แก้ yaw ระหว่างทาง (แก้ด้วย correct_yaw หลังหยุด)"""
        rel = (d - self.body_heading) % 4
        ns_move = rel in (0, 2)
        side = d if (ns_move or lat_dir is None) else lat_dir
        srel = (side - self.body_heading) % 4
        self._goto_gimbal(self._gimbal_yaw_for(side),
                          C.GIMBAL_PITCH_SIDE if srel in (1, 3) else C.GIMBAL_PITCH)
        self._wait_fresh("_tof_n", 3, 0.6)            # ToF ค่าแรกต้องมาหลัง gimbal หันเสร็จ (ไม่ใช่ทิศเก่า)
        use_tof_front = side == d
        use_tof_lat = not ns_move and side != d
        self._move_start = self._pos
        self._move_start_pos = self._pos
        start = self._pos
        ux, uy = DX[d], DY[d]                         # ทิศเดิน (โลก)
        px, py = DX[(d + 1) % 4], DY[(d + 1) % 4]     # ทิศ "ขวาของทิศเดิน" (โลก)
        L = dist_m * 1000.0
        E = C.WALL_EXPECT_MM
        V = C.CL_SPEED
        deadline = time.time() + dist_m / V * 2.0 + 2.0
        last_prog, last_t = 0.0, time.time()
        why = "ครบระยะ"
        lat_used = set()
        # I ของตัวคุมข้าง (ล้อไถลข้างสม่ำเสมอ P อย่างเดียวค้างเยื้อง) ; เริ่มจากค่าไถลที่เรียนได้จากก้าวก่อนทิศเดียวกัน
        # (log 1/10: ช่องโล่งไม่มีกำแพงข้าง เยื้อง 80-144mm ต่อก้าว -> ชดเชยล่วงหน้าได้)
        if not hasattr(self, "_slip_i"):
            self._slip_i = {}
        integ, last_loop = C.CL_SLIP_MEMORY * self._slip_i.get(d, 0.0), time.time()
        self._calib_sharp(side_walls)
        hist = []                                     # ToF หน้าล่าสุด (ใช้ค่าน้อยสุด กันค่าหน่วง)
        yaw_start = self._yaw
        try:
            while True:
                now = time.time()
                if now > deadline:
                    why = "หมดเวลา"
                    break
                twist = self._twisted(yaw_start)
                if twist is not None:
                    why = f"ตัวถังหมุนเอง {abs(twist):.0f}°"
                    self._stop_chassis()
                    self._halt_twist(f"เดิน {DIRS[d]}", twist)
                    break
                # --- ระยะที่เดินได้ (odometry) ---
                moved = self._body_progress_world(start, ux, uy) * 1000.0 / odo_k
                rest = L - moved
                how = "odo"
                # --- กำแพงหน้า (ระยะจริง) ---
                front = None
                if use_tof_front:
                    hist.append(self._interpret_tof(self._tof[C.TOF_INDEX]))
                    del hist[:-3]
                    front = min(hist)
                    if front < E + C.ABS_REF_FRAC * self.cell_m * 1000:
                        rest, how = front - E, "tof"
                    if front <= C.CL_EMERGENCY_MM:
                        why = f"ฉุกเฉิน ToF หน้า {front:.0f}mm"
                        break
                elif front_wall and rest < C.CL_SHARP_FRONT_MM:
                    sh = self._sharp_once(rel)                  # ใกล้ถึงกำแพงที่ยืนยันแล้ว = ค่าจริง
                    if sh is not None and sh[0] < C.SHARP_WALL_MM:
                        rest, how = sh[0] - sh[1], "sharp"
                        if sh[0] <= C.CL_SHARP_EMERGENCY_MM:
                            why = f"ฉุกเฉิน Sharp หน้า {sh[0]:.0f}mm"
                            break
                rest = min(rest, L + C.CL_OVER_MAX_MM - moved)   # ห้ามเกินระยะที่วางแผนมากเกิน
                if rest <= C.CL_STOP_TOL_MM:
                    why = f"ถึง ({how})"
                    break
                v = V if rest >= C.CL_DECEL_MM else max(C.CL_MIN_SPEED, V * rest / C.CL_DECEL_MM)
                # --- ค่าเยื้องข้าง (mm, + = เยื้องไปทาง "ขวาของทิศเดิน") ---
                off = None
                if ns_move:
                    vals = []
                    for srel_ in (1, 3):                # 1 = Sharp ขวาตัวถัง (ชี้ E) , 3 = ซ้าย (ชี้ W)
                        wd = (self.body_heading + srel_) % 4           # ทิศโลกที่ Sharp ชี้
                        # 2 = แผนที่ยืนยันมีกำแพง (ค่าจริงแน่ ใช้ได้ถึง CL_SIDE_WALL_MM) ; 1 = ยังไม่รู้ (เชื่อแค่ค่าต่ำ
                        # CL_SHARP_TRUST_MM) ; 0 = โล่ง (ค่า Sharp หลอก ห้ามใช้) ; ครึ่งแรกดูช่องนี้ ครึ่งหลังดูช่องปลายทาง
                        known = 1 if side_walls is None else side_walls.get(wd, (1, 1))[moved >= L / 2]
                        if not known:
                            continue
                        s = self._sharp_once(srel_)
                        lim = C.CL_SIDE_WALL_MM if known == 2 else C.CL_SHARP_TRUST_MM
                        if s is None or s[0] >= lim:
                            continue
                        # ใกล้กำแพงทิศ wd เกิน expect = เยื้องไปทาง wd (expect = ค่าที่เรียนจาก ToF ตอนอยู่กลางช่อง)
                        toward = self._sharp_expect(srel_, s[1]) - s[0]
                        sign = 1 if wd == (d + 1) % 4 else -1
                        vals.append(sign * toward)
                    if vals:
                        off = sum(vals) / len(vals)
                        lat_used.add("sharp")
                elif use_tof_lat:
                    t = self._interpret_tof(self._tof[C.TOF_INDEX])
                    if t < E + C.CL_LAT_TOF_RANGE_MM:
                        sign = 1 if side == (d + 1) % 4 else -1
                        off = sign * (E - t)
                        lat_used.add("tof")
                vl = 0.0
                dt, last_loop = now - last_loop, now
                if off is not None:
                    integ = max(-C.CL_LAT_I_MAX, min(C.CL_LAT_I_MAX, integ + off * dt))
                    if abs(off) > C.CL_LAT_DEADBAND_MM or abs(integ) > 5:
                        vl = max(-C.CL_LAT_MAX, min(C.CL_LAT_MAX, -C.CL_LAT_KP * off - C.CL_LAT_KI * integ))
                elif integ:
                    # กำแพงข้างหมด: ใช้ค่าไถลที่ I เรียนได้ชดเชยต่อ (ไม่มีตัววัด แต่ล้อยังไถลเท่าเดิม)
                    vl = max(-C.CL_LAT_MAX, min(C.CL_LAT_MAX, -C.CL_LAT_KI * integ))
                wx, wy = ux * v + px * vl, uy * v + py * vl
                bx, by = self._world_to_body(wx, wy)
                self._ep.chassis.drive_speed(x=bx, y=C.STRAFE_SIGN * by, z=0, timeout=0.5)
                # --- ค้าง (ล้อหมุนแต่ไม่ไป) ---
                if moved - last_prog > 15:
                    last_prog, last_t = moved, now
                elif now - last_t > C.CL_STALL_S:
                    why = f"ค้างที่ {moved:.0f}mm"
                    break
                time.sleep(0.03)
        except Exception as e:                              # noqa: BLE001
            why = f"ผิดพลาด {e}"
        self._stop_chassis()
        if lat_used:
            self._slip_i[d] = integ                  # ค่าไถลข้างที่เรียนได้ ใช้ตั้งต้นก้าวต่อไปทิศนี้
        drift = self._wait_still()
        moved = self._body_progress_world(start, ux, uy) * 1000.0 / odo_k
        self.log(f"[drive] ทิศ {DIRS[d]} {moved:.0f}/{L:.0f}mm {why} ; คุมข้างด้วย "
                 f"{'+'.join(sorted(lat_used)) or 'ไม่มี (odometry)'}"
                 + (f" ; ไหลหลังหยุด {drift * 1000:.0f}mm" if drift > 0.03 else ""))
        self.cell = (self.cell[0] + DX[d], self.cell[1] + DY[d])
        self.heading = d
        self.off = [0.0, 0.0]
        self.correct_yaw()

    def _calib_sharp(self, side_walls):
        """ตอนเริ่มเดิน (หุ่นเพิ่งจัดกลางช่องด้วย ToF): ด้านที่แผนที่ยืนยันว่าช่องนี้มีกำแพง -> ค่า Sharp ตอนนี้ = ค่าตอน
        "อยู่กลางช่อง" ของ Sharp ตัวนั้น เก็บค่ากลางล่าสุดไว้ใช้แทนค่าคงที่ (log 1/10: มี Sharp คุมยังเยื้อง 80-104mm
        ค่าคงที่ 149/141 จาก 27/9 กับ ToF อาจนิยามกลางช่องไม่ตรงกัน)"""
        if side_walls is None or not C.CL_SHARP_AUTOCAL:
            return
        if not hasattr(self, "_sharp_cal"):
            self._sharp_cal = {1: [], 3: []}
        for srel in (1, 3):
            wd = (self.body_heading + srel) % 4
            if side_walls.get(wd, (0, 0))[0] != 2:
                continue
            vals = [v[0] for v in (self._sharp_once(srel) for _ in range(3)) if v is not None]
            if not vals:
                continue
            v = sorted(vals)[len(vals) // 2]
            base = C.SHARP_R_EXPECT_MM if srel == 1 else C.SHARP_L_EXPECT_MM
            if v < C.CL_SIDE_WALL_MM and abs(v - base) < C.CL_SHARP_CAL_MAX_MM:
                self._sharp_cal[srel].append(v)
                del self._sharp_cal[srel][:-9]

    def _sharp_expect(self, srel, default):
        cal = getattr(self, "_sharp_cal", {}).get(srel, [])
        if len(cal) < 3:
            return default
        return sorted(cal)[len(cal) // 2]

    # ---------- วัดมุมตัวถังเทียบกำแพง (แก้ "เอียงขวาค้าง") ----------
    def wall_skew(self, d, theta):
        """ตัวถังหมุนเอียงจากกริดกี่องศา (+ = เอียงตามเข็ม/ขวา) วัดจากกำแพงทิศ d ของช่องนี้: ToF ที่มุม ±theta จาก
        แนวตั้งฉาก (มุมเทียบตัวถัง) ; ตัวถังตรง = สองค่าเท่ากัน ; เอียงขวา = ด้าน +theta ยาวกว่า
        tan(psi) = (r+ - r-) cos(theta) / ((r+ + r-) sin(theta)) (r = ระยะจากจุดหมุน gimbal = ToF + ระยะเผื่อ)
        (ผู้ใช้ 1/10: หุ่นเอียงขวาแล้วค้างแบบนั้นตลอด = มุมอ้างอิง IMU ผิด/IMU ลอย IMU อย่างเดียวไม่รู้)
        -> (องศา, r+, r-) หรือ None (gimbal ไปไม่ถึง/ค่าไม่นิ่ง)"""
        e = self.cell_m * 500.0 - C.WALL_EXPECT_MM           # ToF อ่าน 195 ตอนกำแพงห่างจุดกลาง 300
        rrel = (d - self.body_heading) % 4
        pitch = C.GIMBAL_PITCH_SIDE if rrel in (1, 3) else C.GIMBAL_PITCH
        rs = []
        for sgn in (1, -1):
            world = d * 90.0 + sgn * theta
            rel = (world - self.body_heading * 90.0 + 180.0) % 360.0 - 180.0
            yaw = self.gimbal_yaw_sign * rel * (1 if C.GIMBAL_RIGHT_YAW > 0 else -1)
            if not self._goto_gimbal(yaw, pitch, attempts=1, extra_wait_s=0.8):
                return None
            time.sleep(C.TOF_SETTLE_S)
            samples, stable = self._fresh_tof_samples()
            if not stable:
                return None
            rs.append(statistics.median(samples) + e)
        rp, rm = rs
        t = math.radians(theta)
        psi = math.degrees(math.atan((rp - rm) * math.cos(t) / ((rp + rm) * math.sin(t))))
        return psi, rp - e, rm - e

    def apply_skew(self, psi):
        """ตัวถังเอียงตามเข็ม psi องศาจากกริด -> ตั้งมุมอ้างอิง IMU ใหม่ (ตรงจริง = มุมตอนนี้ - psi) แล้วหมุนแก้
        (yaw ของ IMU ตามเข็ม = บวก ; correct_yaw จำกัดการหมุนต่อครั้งอยู่แล้ว)"""
        self._yaw0 = self._yaw - psi
        self.correct_yaw()

    def _body_progress_world(self, start, wx, wy):
        """ระยะ (m) ที่ odometry บอกว่าไปตามทิศโลก (wx, wy) ตั้งแต่ start"""
        bx, by = self._world_to_body(wx, wy)
        return self._body_progress(start, bx, by)

    # ---------- lifecycle ----------
    def close(self):
        for fn in (self._ep.sensor.unsub_distance, self._ep.chassis.unsub_attitude,
                  self._ep.chassis.unsub_position, self._ep.gimbal.unsub_angle):
            try:
                fn()
            except Exception:                               # noqa: BLE001
                pass
        try:
            self._ep.close()
        except Exception:                                   # noqa: BLE001
            pass
