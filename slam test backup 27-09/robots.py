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
        return {d: self.read_dir(d) for d in range(4)}

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
    def move_cell(self, d):
        time.sleep(C.SIM_STEP_DELAY_S)
        nx, ny = self.cell[0] + DX[d], self.cell[1] + DY[d]
        self.cell = (nx, ny)
        self.heading = d
        self.off = [self._rnd.gauss(0, C.SIM_MOVE_NOISE_M),
                    self._rnd.gauss(0, C.SIM_MOVE_NOISE_M)]

    def correct_yaw(self):
        self._yaw = 0.0

    def shift(self, dx, dy):
        time.sleep(0.02)

    def reset_heading_ref(self):
        self._yaw = 0.0

    def undo_last_move(self):
        pass                    # จำลอง: move_cell สำเร็จเสมอ ไม่มีอะไรต้องย้อน

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
        self._att_n = 0         # จำนวน callback attitude/position ที่ได้รับแล้ว
        self._pos_n = 0
        self._move_start = None  # odometry ตอนเริ่ม move_cell ล่าสุด (ใช้ undo_last_move)
        #: bool: True = อ่าน Sharp ไม่ได้ (สาย/พอร์ตผิด) -> ใช้ ToF อย่างเดียว
        self._sharp_off = False

        self.log("[robot] กำลังเชื่อมต่อ RoboMaster (AP mode) ...")
        self._ep = robot.Robot()
        self._ep.initialize(conn_type="ap")
        self._ep.gimbal.sub_angle(freq=20, callback=self._on_gimbal_angle)
        time.sleep(0.2)
        self._ep.gimbal.recenter().wait_for_completed()
        time.sleep(0.2)
        #: int: +1 ถ้า gimbal.moveto(yaw=+X) ทำให้มุมที่วัดจริงเพิ่มขึ้น, -1 ถ้ากลับด้าน
        #: (coordinate mode ของ moveto() ไม่รับประกันเครื่องหมายเดียวกับที่คาดไว้เสมอ
        #: ต้องวัดจริงจากหุ่นแต่ละตัว ไม่งั้นทิศ E/W ที่สแกนได้จะสลับซ้าย-ขวากัน)
        self.gimbal_yaw_sign = self._calibrate_gimbal_yaw_sign()

        self._ep.sensor.sub_distance(freq=10, callback=self._on_tof)
        self._ep.chassis.sub_attitude(freq=10, callback=self._on_attitude)
        self._ep.chassis.sub_position(freq=10, callback=self._on_position)
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
        self.log("[robot] เชื่อมต่อสำเร็จ พร้อมใช้งาน")

    # ---------- callbacks (เธรดของ DDS) ----------
    def _on_tof(self, info):
        self._tof = list(info)

    def _on_gimbal_angle(self, info):
        self._gimbal_yaw = info[1]

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
        """(ตะวันออก, เหนือ) เมตร จาก odometry — กรอบ odometry คือ (หน้า, ขวา) ของตัวถัง
        ตอนตั้ง _pos0 (log 28/9: หัน N แล้วเดิน E -> odom y เพิ่ม = ขวา)"""
        f, r = self.odom_xy()
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

    def _goto_gimbal(self, yaw, pitch, tol_deg=3.0, extra_wait_s=1.5):
        """สั่ง gimbal ไปมุมเป้าหมาย แล้ว**ยืนยันจริงจาก self._gimbal_yaw** (sub_angle)
        ว่าไปถึงแล้วก่อนคืนค่า - ไม่เชื่อ wait_for_completed() เพียงอย่างเดียว เพราะเคยสงสัย
        ว่าค่า ToF ที่อ่านได้ "เร็ว/ผิดปกติ" อาจเกิดจากเริ่มอ่านตั้งแต่ gimbal ยังหมุนไม่หยุดจริง
        """
        try:
            self._ep.gimbal.moveto(pitch=pitch, yaw=yaw,
                                   yaw_speed=C.GIMBAL_YAW_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[gimbal] moveto(yaw={yaw}) ล้มเหลว: {e}")
            return
        deadline = time.time() + extra_wait_s
        while time.time() < deadline:
            if abs(self._gimbal_yaw - yaw) <= tol_deg:
                return
            time.sleep(0.03)
        self.log(f"[gimbal] สั่งไป yaw={yaw:.0f} แต่มุมจริงยังไม่นิ่ง (อ่านได้ {self._gimbal_yaw:.1f}°) "
                 f"- จะอ่าน ToF ต่อไปเลย ค่าที่ได้อาจไม่แม่น")

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
        samples = []
        for _ in range(C.TOF_SAMPLES):
            samples.append(self._interpret_tof(self._tof[C.TOF_INDEX]))
            time.sleep(0.03)
        return float(statistics.median(samples)), samples

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
        out = {d: self.read_dir(d) for d in range(4)}
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
        new_err = _ang_diff(self._yaw0, self._yaw)
        if abs(new_err) > abs(err) + 1.0:
            self.log(f"[yaw] แก้มุมแล้วเบี้ยวขึ้นแทนที่จะลดลง ({err:+.1f}° -> {new_err:+.1f}°) "
                     f"- หยุดแก้รอบนี้ไว้ก่อน ถ้าเกิดซ้ำบ่อยให้ลองตั้ง YAW_SIGN = {-C.YAW_SIGN} "
                     f"ใน config.py เอง (ไม่ใช้วิธีสลับอัตโนมัติแล้ว เพราะเคยทำให้หมุนเลยเถิด)")

    def reset_heading_ref(self):
        """ใช้มุมตอนนี้เป็นทิศอ้างอิง (เรียกตอนเริ่มสำรวจ - ผู้ใช้วางหุ่นตรงกับกริดแล้ว)"""
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
            self._ep.chassis.move(x=-bx * scale, y=-by * scale, z=0,
                                  xy_speed=C.XY_SPEED).wait_for_completed(timeout=6.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[undo] ถอยกลับล้มเหลว: {e}")

    def _rotate_raw(self, deg):
        if abs(deg) < 0.3:
            return
        try:
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

    def shift(self, dx, dy):
        """ขยับเล็กน้อย (เมตร, พิกัดโลก) ใช้ตอน recenter เท่านั้น"""
        bx, by = self._world_to_body(dx, dy)
        try:
            self._ep.chassis.move(x=bx, y=C.STRAFE_SIGN * by, z=0,
                                  xy_speed=C.XY_SPEED, z_speed=C.Z_SPEED).wait_for_completed(timeout=4.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] shift({dx:.3f},{dy:.3f}) ล้มเหลว: {e}")

    def move_cell(self, d):
        """เดินหนึ่งช่องไปทิศโลก d"""
        if C.MOVE_MODE == "rotate" and d != self.body_heading:
            turn_steps = (d - self.body_heading) % 4
            deg = {1: -90.0, 2: 180.0, 3: 90.0}[turn_steps]   # +90=ซ้าย (chassis.move ก้อนเดียว)
            self._rotate_raw(deg)
            self.body_heading = d
            self._yaw0 = self._yaw          # ทิศอ้างอิงใหม่หลังเลี้ยว

        self._move_start = self._pos
        bx, by = self._world_to_body(DX[d] * self.cell_m, DY[d] * self.cell_m)
        bx *= C.MOVE_FACTOR_FWD
        by *= C.MOVE_FACTOR_STRAFE
        ok = False
        try:
            action = self._ep.chassis.move(x=bx, y=C.STRAFE_SIGN * by, z=0,
                                           xy_speed=C.XY_SPEED, z_speed=C.Z_SPEED)
            ok = action.wait_for_completed(timeout=8.0)
        except Exception as e:                              # noqa: BLE001
            self.log(f"[chassis] เดินไปทิศ {DIRS[d]} ล้มเหลว: {e}")
        if not ok:
            self.log(f"[chassis] เดินไปทิศ {DIRS[d]} ไม่ยืนยันว่าสำเร็จ (timeout/ถูกบล็อก) "
                     f"- ตำแหน่งที่บันทึกไว้อาจไม่ตรงกับจริง (explorer จะตรวจซ้ำด้วย ToF)")

        self.cell = (self.cell[0] + DX[d], self.cell[1] + DY[d])
        self.heading = d
        self.off = [0.0, 0.0]
        if C.MOVE_MODE != "rotate":
            self.correct_yaw()

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
