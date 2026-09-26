"""Class Work 8 - SLAM (ToF + gimbal อย่างเดียว) สำหรับ RoboMaster EP

ทำไมมีไฟล์นี้แยกจาก ``SLAM.py``
--------------------------------
``SLAM.py`` ใช้ Sharp IR ซ้าย/ขวาคุมกึ่งกลางช่องระหว่างเดิน แต่ไฟล์นี้ตัดตัวแปร
นั้นออกทั้งหมด เหลือแค่ **ToF (ติดอยู่บน gimbal) + odometry + IMU** เท่านั้น
เป้าหมายคือไล่จูนความแม่นยำของการเดิน/จอดกึ่งกลางช่องให้นิ่งที่สุดก่อน (ช้าได้
ไม่เป็นไร) เก็บ log ละเอียดพอจะเห็นว่าจุดไหนยังไม่แม่น แล้วค่อยตัดสินใจว่าจะ
เติม Sharp ตรงไหนทีหลัง

หลักการที่ต่างจาก SLAM.py
-------------------------
- **gimbal หมุนอิสระจากแชสซี** (``ep_gimbal.moveto(yaw=X, pitch=0)`` หมุนไปมุม X
  องศาที่สัมพัทธ์กับแชสซีได้ในคำสั่งเดียว ช่วงมุมกว้างถึง ±250 องศา - ดูตัวอย่าง
  ทางการ ``examples/03_gimbal/01_move.py``) ทุกช่องจะหมุน "แค่ gimbal" (ไม่หมุน
  ตัวหุ่น) ไปทั้ง 4 ทิศ หน้า/ขวา/หลัง/ซ้าย เพื่ออ่าน ToF ครบทุกด้าน แล้วกลับมา
  ชี้หน้าไว้ก่อนออกเดิน - เร็วกว่าและไม่กวนตำแหน่ง odometry/yaw ของแชสซีเลย
- **สถาปัตยกรรมการเดิน (หลังศึกษา SDK เต็มรูปแบบ): "เซนเซอร์ก่อน -> อัปเดต
  แผนที่ -> ตัดสินใจ -> เดินทีละก้าวที่ยืนยันแล้ว"** ทุกการเคลื่อนที่ใช้
  ``chassis.move(x=,y=,z=)`` (คำสั่งเดิน/หมุนแบบบล็อกจนกว่าเฟิร์มแวร์ยืนยันจริง
  ว่าไปถึง ปิดลูปด้วย odometry ของตัวมันเอง) แทน ``drive_speed()`` แบบสตรีม
  ความเร็วต่อเนื่องในลูป Python ที่ไม่มีการยืนยันจากฮาร์ดแวร์เลยแบบเดิม - เครื่อง
  หมายยืนยันจากตัวอย่างทางการ ``examples/02_chassis/01_move.py``: x บวก=หน้า,
  y บวก=ขวา/ลบ=ซ้าย, z บวก=ซ้าย/ลบ=ขวา (ตรงข้ามกับ ``drive_speed()``)
- **``chassis.move()`` ยกเลิกกลางทางไม่ได้เลย** (ไม่มี abort ใน SDK) ความ
  ปลอดภัยทั้งหมดจึงมาจาก **คำนวณระยะปลอดภัยสูงสุดจากค่า ToF ที่อ่านสด ๆ ก่อน
  สั่งทุกครั้ง** (``Driver.safe_step_mm``) ไม่ใช่คอยเบรกระหว่างทางแบบเดิม - ทุก
  ก้าว (จัดกึ่งกลาง/เดินเข้าช่องใหม่/ถอย) เป็นคำสั่งเดี่ยวที่ยืนยันจากเฟิร์มแวร์
  แล้วอ่านเซนเซอร์ใหม่ก่อนตัดสินใจก้าวถัดไปเสมอ (``wait_for_completed()`` คืน
  False ไม่ได้แปลว่าไม่ได้เดินจริง - ข้อความยืนยันอาจหายระหว่างทาง จึงห้ามเชื่อ
  ผลลัพธ์คำสั่งเฉย ๆ ต้องเซนเซอร์ใหม่เสมอ)
- **``chassis.sub_status()`` เป็นเซฟตี้เน็ตอิสระชั้นสุดท้าย** ไม่เกี่ยวกับ ToF
  เลย - บิต ``impact_x/y/z`` (ชนจริงจากฮาร์ดแวร์) และ ``slip_flag`` (ล้อลื่น/
  ติดขัด) มอนิเตอร์ตลอดเวลา ถ้าติดขึ้นมาเมื่อไหร่ หยุดภารกิจทั้งหมดทันที ไม่
  สำรวจต่อ
- **ตรวจกำแพงครบทุกด้านจริง ๆ ทุกช่อง** ไม่เดาว่าด้านหลังเปิดเพราะเพิ่งเดินผ่าน
  มา (ต่างจาก SLAM.py ที่ประหยัดเวลาด้วยการไม่ตรวจซ้ำ) ถ้าตรวจซ้ำแล้วขัดแย้งกับ
  ความรู้เดิม (เช่น edge ที่เคยยืนยันว่าโล่งแต่รอบนี้เจอ wall) จะ **ไม่ทับข้อมูล
  เดิม** (แผนที่เพิ่มกำแพงได้อย่างเดียว) แต่ log เป็นสัญญาณเตือนไว้แทน
- **เพดานระยะสะสมระหว่างจัดกึ่งกลาง** (``MAX_CREEP_TOTAL_MM``/
  ``MAX_TOTAL_LATERAL_MM``) วัดจริงจาก odometry ตลอดการจัดหนึ่งรอบ ไม่ใช่แค่
  ต่อก้าว - กันกรณีอ้างอิงกำแพงข้างเดียวคลาดเคลื่อน (เช่น ``SIDE_STOP_MM`` ไม่
  ตรงสนามจริง) แล้วไล่ขยับไปทางเดิมซ้ำ ๆ ไม่มีที่สิ้นสุดจนหลุดเส้นทาง
- **กรอง outlier ของ ToF ก่อนตัดสินใจทุกครั้ง** (``SensorHub._filter_tof_samples``)
  หุ่นหยุดนิ่งสนิทตอนอ่านทุกครั้งอยู่แล้ว จึงเก็บตัวอย่างได้เยอะ (``TOF_SAMPLES``)
  แล้วตัดค่าที่เพี้ยนออกก่อนหา median ถ้าข้อมูลไม่นิ่งพอจะเชื่อได้ คืน None
  แทนที่จะฝืนใช้ค่าที่อาจผิด

ใช้ซ้ำจาก ``SLAM.py`` เฉพาะส่วนที่ไม่ผูกกับ config เซนเซอร์ (``Maze``,
``ExplorePlanner``, ค่าคงที่ทิศทาง, ค่าคอนฟิกสนาม) ส่วน ``SensorHub``/``Driver``
เขียนใหม่เป็นของตัวเอง เพราะของ ``SLAM.py`` ผูกกับ Sharp/IR config ของไฟล์นั้น
โดยตรง

ฮาร์ดแวร์ที่ต้องต่อ
------------------
- ToF (Distance Sensor) ติดบน gimbal -> sensor.sub_distance()
- ไม่มี Sharp / ไม่มี IR digital ในไฟล์นี้

วิธีใช้
------
    python SLAM_tof_gimbal.py --calib    วัดค่า ToF จริง (ต้องทำก่อนใช้งานครั้งแรก)
    python SLAM_tof_gimbal.py            วิ่งจริงในสนาม
"""

import argparse
import csv
import math
import statistics
import sys
import threading
import time

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError as exc:
    robot = None
    ROBOT_IMPORT_ERROR = exc
else:
    ROBOT_IMPORT_ERROR = None

from SLAM import (
    Maze, ExplorePlanner, NORTH, EAST, SOUTH, WEST, DIR_NAMES,
    MAZE_W, MAZE_H, CELL_SIZE_M, START_CELL, START_HEADING,
    BORDER_OPENINGS, _timestamp, _edge_direction, _stats,
)


# =====================================================================
# CONFIG - แก้ค่าทั้งหมดที่นี่ที่เดียว
# =====================================================================
TOF_INDEX = 0
# ---------- ค่าที่ได้จาก `--calib` เท่านั้น ห้ามเดา ----------
FRONT_STOP_MM = 191            # ToF ตอนหุ่นอยู่กลางช่องแล้วหันหน้าชนกำแพง
SIDE_STOP_MM = 140             # ToF (ผ่าน gimbal) ตอนหุ่นอยู่กลางช่องมีกำแพงข้าง
TOF_MAX_VALID_MM = 4000
TOF_SAMPLES = 7                 # จำนวนตัวอย่างต่อการอ่านหนึ่งทิศ (หุ่นหยุดนิ่ง
                                 # สนิทตอนอ่านทุกครั้งอยู่แล้ว เก็บเยอะหน่อยได้
                                 # ไม่เสียเวลามาก แลกกับกรอง glitch ได้ดีขึ้น)
TOF_SAMPLE_INTERVAL = 0.05
TOF_OUTLIER_MM = 80             # ตัวอย่างที่เบี่ยงจาก median เกินนี้ถือว่าเป็น
                                 # ค่าเพี้ยน (glitch) ตัดทิ้งก่อนสรุปผล
TOF_MIN_VALID_SAMPLES_FRAC = 0.5  # ถ้าเหลือตัวอย่างดี ๆ น้อยกว่านี้ของทั้งหมด
                                   # ถือว่าอ่านไม่ได้ (None) ดีกว่าฝืนใช้ค่าที่
                                   # อาจผิด

# ---------- gimbal ----------
GIMBAL_PITCH = 0
GIMBAL_YAW_SPEED = 200.0        # deg/s ไม่กระทบความแม่นยำ แค่ความเร็ว settle
GIMBAL_SETTLE_S = 0.2           # รอเพิ่มหลัง wait_for_completed ก่อนเริ่มอ่าน
GIMBAL_YAW_FRONT = 0
GIMBAL_YAW_LEFT = 90            # +yaw = ซ้าย (อ้างอิงจาก examples/03_gimbal/01_move.py)
GIMBAL_YAW_RIGHT = -90
GIMBAL_YAW_BACK = 180

# ---------- การเดิน: chassis.move() แบบก้าวเดียวที่ยืนยันจากเฟิร์มแวร์ ----------
# แทนที่ drive_speed() แบบสตรีมความเร็วต่อเนื่องทั้งหมด - ดู docstring หัวไฟล์
MOVE_XY_SPEED = 0.5             # m/s - ค่าต่ำสุดที่ chassis.move() ยอมรับจริง
                                 # (checker บังคับ [0.5, 2.0] ช้ากว่านี้สั่งไม่ได้)
MOVE_Z_SPEED = 45.0             # deg/s ตอนหมุน (ตัวอย่างทางการใช้ 45)
MOVE_TIMEOUT_MARGIN_S = 3.0     # บวกเข้ากับเวลาที่คำนวณจากระยะ/ความเร็ว กัน
                                 # wait_for_completed() ค้างตลอดกาล (SDK ไม่มี
                                 # abort และไม่รับประกันว่า push ยืนยันจะมาถึง)

# ---------- การจอดกึ่งกลางช่อง ----------
ALIGN_TOLERANCE_MM = 15
LATERAL_TOLERANCE_MM = 15
LATERAL_MAX_ITERS = 6           # วนวัด-ขยับ-วัดซ้ำซ้าย-ขวาได้กี่รอบก่อนยอมแพ้
MAX_CREEP_STEP_MM = 40          # ก้าวเดียวตอนจัดกึ่งกลางห้ามเกินนี้ (กันก้าวใหญ่
                                 # เกินไปแม้ error ที่วัดได้จะผิดพลาด)
MIN_SAFE_MM = 60                # ห้ามสั่งก้าวใดที่ทำให้ระยะเหลือน้อยกว่านี้เด็ดขาด
                                 # (คำนวณจากค่าที่อ่านสด ๆ ก่อนสั่งทุกครั้ง ไม่ใช่
                                 # คอยเบรกระหว่างทางแบบเดิม เพราะ chassis.move()
                                 # ยกเลิกกลางทางไม่ได้)
MAX_CREEP_TOTAL_MM = 200        # เพดานระยะสะสม (วัดจริงจาก odometry ไม่ใช่แค่
                                 # นับคำสั่ง) ที่ยอมให้ไล่จัดหน้า-หลังได้ต่อการ
                                 # เรียก center_in_cell หนึ่งครั้ง
MAX_TOTAL_LATERAL_MM = 200      # เพดานระยะสะสมซ้าย-ขวาต่อการจัดกึ่งกลางหนึ่ง
                                 # รอบเช่นกัน - กันกรณีอ้างอิงกำแพงข้างเดียว
                                 # (เช่น SIDE_STOP_MM คลาดเคลื่อน) แล้วไล่ขยับ
                                 # ไปเรื่อย ๆ ไม่มีที่สิ้นสุดจนหลุดเส้นทาง

# ---------- การนับช่อง ----------
CELL_COMPLETE_RATIO = 0.85

# ---------- ระบบ ----------
SETTLE_S = 0.3
MAX_STEPS = 350
BACK_RETRY = 4                  # backtrack/reroute ไม่ผ่านกี่ครั้งก่อนปิดทางไว้ก่อน
DDS_FREQ = 20
SUB_WAIT_S = 2.0
SENSOR_STALE_S = 0.5
DRIVE_WATCHDOG_S = 0.5          # ใช้กับ drive_speed(0,0,0) ฉุกเฉินเท่านั้น
                                 # (การเดินปกติทั้งหมดใช้ chassis.move() แล้ว)
CONN_TYPE = "ap"
LOG_PREFIX = "tofgimbal"

#: tuple: ลำดับสแกน (ทิศสัมพัทธ์ 0=หน้า 1=ขวา 2=หลัง 3=ซ้าย, มุม gimbal, ชื่อ)
SCAN_PLAN = (
    (0, GIMBAL_YAW_FRONT, "front"),
    (1, GIMBAL_YAW_RIGHT, "right"),
    (2, GIMBAL_YAW_BACK, "back"),
    (3, GIMBAL_YAW_LEFT, "left"),
)


def wall_cutoff_mm(base_mm):
    """int: เกณฑ์ ToF ที่ใช้ตัดสินว่ามีกำแพงอยู่ในทิศนั้น หน่วย mm

    เรขาคณิตแบบเดียวกับ ``SLAM.py`` ``front_wall_threshold_mm()``: เมื่อหุ่นอยู่
    กลางช่อง กำแพงขอบช่องนี้อ่านได้ ``base_mm`` ส่วนกำแพงขอบช่องถัดไปอ่านได้
    ``base_mm + CELL_SIZE`` เส้นแบ่งจึงวางไว้กึ่งกลางระหว่างสองค่านั้น
    """
    return base_mm + int(CELL_SIZE_M * 1000.0 / 2.0)


def require_calibration():
    """ตรวจว่าค่าจาก --calib ครบแล้ว ถ้าไม่ครบให้หยุดพร้อมบอกว่าขาดตัวไหน"""
    missing = [name for name in ("FRONT_STOP_MM", "SIDE_STOP_MM")
               if globals()[name] is None]
    if not missing:
        return
    print("\n[STOP] ยังไม่ได้คาลิเบรตเซนเซอร์ ค่าที่ยังขาด:")
    for name in missing:
        print("         - {0}".format(name))
    print("\n  รัน `python SLAM_tof_gimbal.py --calib` ก่อน (ใช้เวลาไม่ถึง 1 นาที)")
    print("  แล้วคัดลอกบล็อกค่าที่มันพิมพ์ออกมา ไปวางทับใน CONFIG ด้านบนของไฟล์นี้")
    sys.exit(1)


# =====================================================================
# ชั้นอ่านเซนเซอร์ + คุม gimbal
# =====================================================================
class SensorSnapshot(object):
    """ค่าเซนเซอร์ทั้งชุด ณ เวลาเดียวกัน แช่แข็งไว้แล้ว (ไม่มี Sharp/IR)"""

    __slots__ = ("t", "tof_mm", "yaw", "pos_x", "pos_y", "fresh", "stale_reason")

    def __init__(self, t, tof_mm, yaw, pos_x, pos_y, fresh, stale_reason):
        self.t = t
        #: int or None: ระยะ ToF ปัจจุบัน (ทิศที่ gimbal ชี้อยู่ ณ ตอนนี้) mm
        self.tof_mm = tof_mm
        self.yaw = yaw
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.fresh = fresh
        self.stale_reason = stale_reason


class SensorHub(object):
    """เจ้าของ subscription (ToF, attitude, position) + ตัวคุม gimbal

    ไม่มี sensor_adaptor เลยเพราะไม่มี Sharp/IR ต่อในไฟล์นี้ - เบากว่า
    ``SLAM.py`` มาก
    """

    def __init__(self, ep_robot):
        self._chassis = ep_robot.chassis
        self._sensor = ep_robot.sensor
        self._gimbal = ep_robot.gimbal

        self._lock = threading.Lock()
        self._tof = [0, 0, 0, 0]
        self._tof_t = 0.0
        self._yaw = 0.0
        self._att_t = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_t = 0.0
        #: เซฟตี้เน็ตอิสระจาก ToF - chassis.sub_status() ให้บิตชน/ลื่นจริงจาก
        #: ฮาร์ดแวร์ (accelerometer-derived) ไม่เกี่ยวกับตรรกะ ToF ของเราเลย
        self._impact = False
        self._slip = False
        self._roll_over = False
        self._status_t = 0.0
        self._started = False
        #: int: +1 ถ้า moveto(yaw=+X) ทำให้มุมที่วัดจริงเพิ่มขึ้น (ซ้าย ตาม
        #: ธรรมเนียมที่ examples/03_gimbal/01_move.py ยืนยันไว้สำหรับ move()),
        #: -1 ถ้ากลับด้าน หาได้จาก calibrate_gimbal_yaw_sign() เท่านั้น เพราะ
        #: moveto() ใช้ coordinate mode คนละโหมดกับ move() (YCPN vs CUR) ในซอร์ส
        #: SDK เครื่องหมาย +/- จึงไม่รับประกันว่าจะตรงกัน ต้องวัดจากฮาร์ดแวร์จริง
        self.gimbal_yaw_sign = 1

    def _on_tof(self, info):
        now = time.time()
        with self._lock:
            self._tof = list(info)
            self._tof_t = now

    def _on_attitude(self, info):
        now = time.time()
        with self._lock:
            self._yaw = info[0]
            self._att_t = now

    def _on_position(self, info):
        now = time.time()
        with self._lock:
            self._pos_x, self._pos_y = info[0], info[1]
            self._pos_t = now

    def _on_status(self, info):
        # (static_flag, up_hill, down_hill, on_slope, is_pick_up, slip_flag,
        #  impact_x, impact_y, impact_z, roll_over, hill_static)
        now = time.time()
        with self._lock:
            self._slip = bool(info[5])
            self._impact = bool(info[6]) or bool(info[7]) or bool(info[8])
            self._roll_over = bool(info[9])
            self._status_t = now

    def start(self):
        """เปิด subscription ทั้งหมด แล้วรอจนมีข้อมูลชุดแรกเข้ามา"""
        self._started = True
        self._sensor.sub_distance(freq=DDS_FREQ, callback=self._on_tof)
        self._chassis.sub_attitude(freq=DDS_FREQ, callback=self._on_attitude)
        self._chassis.sub_position(freq=DDS_FREQ, callback=self._on_position)
        self._chassis.sub_status(freq=DDS_FREQ, callback=self._on_status)

        deadline = time.time() + SUB_WAIT_S
        while time.time() < deadline:
            with self._lock:
                got_all = (self._tof_t > 0 and self._att_t > 0
                           and self._pos_t > 0 and self._status_t > 0)
            if got_all:
                print("[SENSOR] subscription พร้อมใช้งาน (ToF, attitude, "
                      "position, status)")
                return
            time.sleep(0.05)

        with self._lock:
            missing = []
            if self._tof_t == 0:
                missing.append("ToF")
            if self._att_t == 0:
                missing.append("attitude")
            if self._pos_t == 0:
                missing.append("position")
            if self._status_t == 0:
                missing.append("status")
        if missing:
            raise RuntimeError(
                "ไม่ได้รับข้อมูลจาก subscription: {0} - ตรวจการเชื่อมต่อหุ่น"
                .format(", ".join(missing)))

    def emergency_flags(self):
        """tuple: (impact, slip, roll_over) จาก ``chassis.sub_status()`` ล่าสุด

        เซฟตี้เน็ตอิสระจาก ToF โดยสิ้นเชิง (ฮาร์ดแวร์ตรวจจับการชน/ลื่นเอง)
        ``run_explore`` เช็คตัวนี้ก่อนตัดสินใจทำอะไรทุกรอบ
        """
        with self._lock:
            return self._impact, self._slip, self._roll_over

    def stop(self):
        """ปิด subscription ทั้งหมด (เรียกซ้ำได้ ไม่ throw)"""
        if not self._started:
            return
        for name, fn in (("distance", self._sensor.unsub_distance),
                         ("attitude", self._chassis.unsub_attitude),
                         ("position", self._chassis.unsub_position),
                         ("status", self._chassis.unsub_status)):
            try:
                fn()
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] unsub {0} ล้มเหลว: {1}".format(name, exc))
        self._started = False

    def snapshot(self):
        """SensorSnapshot: อ่านเซนเซอร์ปัจจุบันเป็นชุดเดียว"""
        now = time.time()
        with self._lock:
            tof_raw = self._tof[TOF_INDEX] if TOF_INDEX < len(self._tof) else 0
            tof_age = now - self._tof_t if self._tof_t else 1e9
            att_age = now - self._att_t if self._att_t else 1e9
            pos_age = now - self._pos_t if self._pos_t else 1e9
            yaw = self._yaw
            pos_x, pos_y = self._pos_x, self._pos_y

        tof_mm = tof_raw if 0 < tof_raw <= TOF_MAX_VALID_MM else None

        stale = ""
        if tof_age > SENSOR_STALE_S:
            stale = "tof"
        elif att_age > SENSOR_STALE_S:
            stale = "attitude"
        elif pos_age > SENSOR_STALE_S:
            stale = "position"

        return SensorSnapshot(t=now, tof_mm=tof_mm, yaw=yaw, pos_x=pos_x,
                               pos_y=pos_y, fresh=(stale == ""), stale_reason=stale)

    def point_gimbal(self, yaw_deg):
        """หมุน gimbal ไปมุม yaw_deg (สัมพัทธ์กับแชสซี ตามความหมายของ
        ``GIMBAL_YAW_FRONT/LEFT/RIGHT/BACK``) แล้วรอให้นิ่ง ไม่อ่านค่า

        คูณด้วย ``gimbal_yaw_sign`` ก่อนส่งจริงเสมอ เพื่อแก้เครื่องหมายที่วัดได้
        จาก ``calibrate_gimbal_yaw_sign()`` - ที่อื่นในไฟล์นี้จึงเรียกด้วยค่า
        "ตามความหมาย" ได้เลย (0=หน้า, +90=ซ้าย, -90=ขวา, 180=หลัง) ไม่ต้องมานั่ง
        คูณเครื่องหมายเองทุกจุดที่เรียก

        ครอบ try/except ไว้เพราะ SDK โยน ``Exception`` ตรง ๆ (ไม่ใช่คืนค่า False)
        ถ้ามี action ค้างอยู่ที่ gimbal เดิมยังไม่จบ (เช่น action ก่อนหน้า
        timeout แต่ยังค้างฝั่งเฟิร์มแวร์) - ไม่ให้ทั้งโปรแกรมตายเพราะ exception
        ที่ไม่ได้ดัก

        Returns:
            bool: True ถ้าคำสั่งไปถึงและยืนยันจบแล้ว (ไม่ได้แปลว่า gimbal ชี้
            ถูกทิศแน่นอน 100% - แค่ไม่ได้ error/timeout ระหว่างทาง)
        """
        try:
            action = self._gimbal.moveto(pitch=GIMBAL_PITCH,
                                          yaw=self.gimbal_yaw_sign * yaw_deg,
                                          yaw_speed=GIMBAL_YAW_SPEED)
            ok = action.wait_for_completed(timeout=6.0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] point_gimbal({0}) ล้มเหลว: {1}".format(yaw_deg, exc))
            ok = False
        time.sleep(GIMBAL_SETTLE_S)
        return ok

    def calibrate_gimbal_yaw_sign(self):
        """หมุน gimbal ทดสอบด้วย ``moveto()`` ตัวจริง (ไม่ใช่ ``move()``) แล้ววัด
        จาก ``sub_angle()`` ว่า +yaw ที่สั่งไปทำให้มุมเพิ่มไปทางไหนจริง ๆ

        ต้องวัดเอาจริงเพราะ ``moveto()`` ใช้ coordinate mode คนละโหมดกับ
        ``move()`` (``COORDINATE_YCPN`` vs ``COORDINATE_CUR`` - ดูซอร์ส
        ``gimbal.py``) ตัวอย่างทางการที่ยืนยันว่า "+yaw = ซ้าย" นั้นยืนยันไว้ให้
        ``move()`` เท่านั้น ไม่รับประกันว่า ``moveto()`` จะใช้เครื่องหมายเดียวกัน
        (เหมือนที่ ``chassis.move(z=)``/``drive_speed(z=)`` ก็เคยสลับเครื่องหมาย
        กันมาแล้วในหุ่นตัวเดียวกันนี้)
        """
        readings = {}

        def _on_angle(info):
            # (pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle)
            readings["yaw"] = info[1]

        self._gimbal.sub_angle(freq=10, callback=_on_angle)
        time.sleep(0.3)
        self._gimbal.moveto(pitch=0, yaw=0, yaw_speed=60).wait_for_completed()
        time.sleep(0.3)
        start_yaw = readings.get("yaw")

        test_yaw = 30
        self._gimbal.moveto(pitch=0, yaw=test_yaw, yaw_speed=60).wait_for_completed()
        time.sleep(0.3)
        end_yaw = readings.get("yaw")

        self._gimbal.moveto(pitch=0, yaw=0, yaw_speed=60).wait_for_completed()
        self._gimbal.unsub_angle()

        if start_yaw is None or end_yaw is None:
            raise RuntimeError(
                "ไม่ได้รับข้อมูลจาก gimbal.sub_angle() - ตรวจการเชื่อมต่อหุ่น")

        delta = end_yaw - start_yaw
        if abs(delta) < 3.0:
            raise RuntimeError(
                "หมุน gimbal ทดสอบแล้วมุมแทบไม่ขยับ ({0:.2f} องศา) - gimbal "
                "อาจติดขัด หรือ sub_angle ไม่ทำงาน".format(delta))

        self.gimbal_yaw_sign = 1 if delta > 0 else -1
        print("[GIMBAL] สั่ง moveto(yaw=+{0}) จริง -> มุมที่วัดได้เปลี่ยน {1:+.2f} "
              "-> GIMBAL_YAW_SIGN = {2:+d} ({3})"
              .format(test_yaw, delta, self.gimbal_yaw_sign,
                      "+yaw=ซ้าย เหมือน move()" if self.gimbal_yaw_sign > 0
                      else "+yaw=ขวา (กลับด้านจาก move()!)"))

    def read_tof(self, yaw_deg):
        """หมุน gimbal ไปทิศที่ต้องการ แล้ววัด ToF จาก ``TOF_SAMPLES`` ตัวอย่าง
        พร้อมกรอง outlier ก่อนสรุปผล (ดู ``_filter_tof_samples``)

        Returns:
            int or None: ระยะ mm ที่กรองแล้ว หรือ None ถ้าอ่านไม่ได้/เชื่อถือไม่ได้
        """
        self.point_gimbal(yaw_deg)
        samples = []
        for _ in range(TOF_SAMPLES):
            snap = self.snapshot()
            if snap.tof_mm is not None:
                samples.append(snap.tof_mm)
            time.sleep(TOF_SAMPLE_INTERVAL)
        return self._filter_tof_samples(samples)

    @staticmethod
    def _filter_tof_samples(samples):
        """int or None: กรอง outlier ออกก่อนสรุปเป็นค่าเดียว (median)

        หุ่นหยุดนิ่งสนิทตอนอ่านทุกครั้งอยู่แล้ว (ตัดสินใจเฉพาะตอนหยุดนิ่ง) การ
        เก็บหลายตัวอย่างจึงช่วยกันค่าที่เพี้ยนแวบเดียว (glitch จากสัญญาณสะท้อน/
        สัญญาณรบกวน) ได้จริง ไม่ใช่แค่เฉลี่ยเฉย ๆ: ตัดตัวอย่างที่เบี่ยงจาก
        median ดิบเกิน ``TOF_OUTLIER_MM`` ออกก่อน แล้วค่อยหา median ใหม่จากที่
        เหลือ ถ้าเหลือน้อยเกินไป (ข้อมูลไม่นิ่งพอจะเชื่อได้) คืน None แทนที่จะ
        ฝืนใช้ค่าที่อาจผิด - ให้ผู้เรียกตัดสินใจว่าจะลองอ่านใหม่หรือถือว่า
        "ไม่รู้" ดีกว่าเดาผิด
        """
        if not samples:
            return None
        raw_median = statistics.median(samples)
        filtered = [s for s in samples if abs(s - raw_median) <= TOF_OUTLIER_MM]
        if len(filtered) < max(1, int(len(samples) * TOF_MIN_VALID_SAMPLES_FRAC)):
            print("[TOF-FILTER] ตัวอย่าง {0} -> เหลือ {1} หลังกรอง outlier "
                  "(median ดิบ={2}mm) - น้อยเกินไป ถือว่าอ่านไม่ได้"
                  .format(samples, len(filtered), raw_median))
            return None
        return int(statistics.median(filtered))


# =====================================================================
# การเคลื่อนที่
# =====================================================================
class Driver(object):
    """ชั้นควบคุมการเคลื่อนที่ - ใช้ ``chassis.move()`` แบบก้าวเดียวที่ยืนยัน
    จากเฟิร์มแวร์ทุกจุด (ปิดลูปด้วย odometry ของตัวมันเอง) แทน
    ``drive_speed()`` แบบสตรีมความเร็วต่อเนื่องในลูป Python ที่ไม่มีการยืนยัน
    ใด ๆ เลยแบบเดิม (ดูเหตุผล/หลักฐานจาก SDK เต็ม ๆ ใน docstring หัวไฟล์)

    ``chassis.move()`` ยกเลิกกลางทางไม่ได้เลย - ความปลอดภัยทั้งหมดจึงมาจาก
    ``safe_step_mm()`` ที่คำนวณระยะปลอดภัยสูงสุดจากค่า ToF ที่อ่านสด ๆ ก่อน
    สั่งทุกครั้ง ไม่ใช่คอยเบรกระหว่างทาง
    """

    def __init__(self, chassis, hub):
        self.chassis = chassis
        self.hub = hub

    # ---------- พื้นฐาน / ฉุกเฉินเท่านั้น ----------
    def stop(self):
        """สั่งหยุดล้อทันทีด้วย drive_speed(0,0,0) - ใช้ตอนเริ่ม/จบ step และ
        เป็นเครื่องมือฉุกเฉินพยายามแทรกหยุด action ที่ค้างอยู่ (SDK ไม่รับ
        ประกันว่าจะขัดจริง แต่ลองดีกว่าไม่ลอง - ดู docstring หัวไฟล์)
        """
        try:
            self.chassis.drive_speed(x=0, y=0, z=0, timeout=DRIVE_WATCHDOG_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))

    @staticmethod
    def _move_timeout(dx_m=0.0, dy_m=0.0, dz_deg=0.0):
        """float: เวลาที่ให้รอ ``wait_for_completed()`` ก่อนถือว่าค้าง

        ต้องใส่เสมอ เพราะ SDK ไม่รับประกันว่า push ยืนยันจบจะมาถึง (อาจค้าง
        ตลอดกาลถ้าไม่ใส่ timeout - ดู docstring หัวไฟล์)
        """
        t_xy = (abs(dx_m) + abs(dy_m)) / MOVE_XY_SPEED if (dx_m or dy_m) else 0.0
        t_z = abs(dz_deg) / MOVE_Z_SPEED if dz_deg else 0.0
        return max(t_xy, t_z) * 2.0 + MOVE_TIMEOUT_MARGIN_S

    def bounded_move(self, dx_m=0.0, dy_m=0.0, dz_deg=0.0):
        """สั่ง ``chassis.move()`` ก้าวเดียว แล้วรอให้เฟิร์มแวร์ยืนยัน

        คืนค่า ``ok`` เป็นแค่สัญญาณอย่างเดียว - ``False``/timeout ไม่ได้แปลว่า
        ไม่ได้เดินจริง (ข้อความยืนยันอาจหายระหว่างทางแม้หุ่นเดินไปแล้วจริง)
        ผู้เรียกต้องอ่านเซนเซอร์ใหม่เองเสมอ ห้ามเชื่อค่านี้เพียงอย่างเดียว
        ครอบ try/except กันกรณี action ซ้อนกัน (SDK โยน Exception ตรง ๆ)
        """
        try:
            action = self.chassis.move(x=dx_m, y=dy_m, z=dz_deg,
                                        xy_speed=MOVE_XY_SPEED,
                                        z_speed=MOVE_Z_SPEED)
            ok = action.wait_for_completed(
                timeout=self._move_timeout(dx_m, dy_m, dz_deg))
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] chassis.move(x={0:.3f},y={1:.3f},z={2:.1f}) "
                  "ล้มเหลว: {3}".format(dx_m, dy_m, dz_deg, exc))
            ok = False
        return ok

    @staticmethod
    def safe_step_mm(current_mm, desired_delta_mm, min_safe_mm=MIN_SAFE_MM):
        """float: ตัดขนาดก้าวให้ปลอดภัยเสมอ จากค่า ToF ที่อ่านสด ๆ ณ ตอนนี้

        หัวใจของ "ตัดสินใจก่อนเดินทุกครั้ง": ``desired_delta_mm`` (บวกเสมอ)
        คือระยะที่อยากขยับเข้าใกล้กำแพงที่ ``current_mm`` วัดอยู่ ถ้าจะทำให้
        ระยะเหลือน้อยกว่า ``min_safe_mm`` จะตัดเหลือแค่พอดี ไม่มีทางพาก้าวใด
        เข้าใกล้กำแพงกว่า ``min_safe_mm`` ได้เลย ไม่ว่าตรรกะที่คำนวณ
        ``desired_delta_mm`` มาจะผิดพลาดแค่ไหน - ``chassis.move()`` ยกเลิก
        กลางทางไม่ได้ จึงต้องกันไว้ก่อนสั่งเท่านั้น
        """
        if current_mm is None:
            return 0.0
        room = current_mm - min_safe_mm
        if room <= 0:
            return 0.0
        return max(0.0, min(desired_delta_mm, room))

    # ---------- การหมุน ----------
    def turn_relative(self, delta_deg):
        """หมุนแชสซีสัมพัทธ์ ``delta_deg`` องศา (+=ซ้าย -=ขวา ยืนยันเครื่อง
        หมายจากตัวอย่างทางการ ``examples/02_chassis/01_move.py``) ด้วย
        ``chassis.move(z=)`` ก้าวเดียว ปิดลูปด้วยเฟิร์มแวร์เอง ไม่ต้องมี
        P-controller/วัด yaw sign เองอีกต่อไป
        """
        if abs(delta_deg) < 1.0:
            return True
        print("[TURN] หมุน {0:+.0f} องศา ({1})"
              .format(delta_deg, "ซ้าย" if delta_deg > 0 else "ขวา"))
        return self.bounded_move(dz_deg=delta_deg)

    def turn_to(self, current_heading, target_heading):
        """หมุนจาก grid heading หนึ่งไปอีกอัน เลือกทิศที่หมุนน้อยสุด แล้วสั่ง
        ครั้งเดียว (180 องศาก็สั่งทีเดียวได้ เพราะ ``chassis.move()`` ปิดลูป
        เอง ไม่ต้องกลัวลังเลแบบตัวคุม P เดิม)
        """
        delta_cells = (target_heading - current_heading) % 4
        if delta_cells == 0:
            self.hub.point_gimbal(GIMBAL_YAW_FRONT)
            return current_heading

        # +90=ซ้าย(delta_cells=3), -90=ขวา(delta_cells=1), 180=ทางไหนก็ได้
        if delta_cells == 1:
            delta_deg = -90.0
        elif delta_cells == 3:
            delta_deg = 90.0
        else:
            delta_deg = 180.0

        print("[TURN] {0} -> {1}".format(DIR_NAMES[current_heading],
                                         DIR_NAMES[target_heading]))
        self.turn_relative(delta_deg)

        # gimbal ไม่ตามแชสซีอัตโนมัติใน FREE mode (ยืนยันจาก set_robot_mode()
        # docstring ของ SDK) ต้องสั่งชี้หน้าซ้ำเสมอ เพราะขั้นตอนถัดไปต้องพึ่ง
        # ค่า ToF หน้าตรงหลังหมุนเสร็จ
        self.hub.point_gimbal(GIMBAL_YAW_FRONT)
        return target_heading

    # ---------- การจัดกึ่งกลางช่อง ----------
    def _creep_axis(self, gimbal_yaw, target_mm, tol_mm, drive_axis, sign,
                     max_iters, known_mm=None, max_total_mm=None):
        """ไล่ปรับระยะตามแกนที่กำหนดด้วยก้าว ``chassis.move()`` ที่ยืนยันแล้ว
        ทีละก้าว (แทนลูป ``drive_speed`` ต่อเนื่องแบบเดิม)

        ทุกก้าว: อ่านค่าสด ๆ -> คำนวณ error -> ตัดขนาดก้าวด้วย
        ``safe_step_mm`` -> ``bounded_move`` ก้าวเดียว -> อ่านใหม่ วนจนกว่า
        จะนิ่งหรือครบ ``max_iters`` หรือครบเพดานระยะสะสม ``max_total_mm``
        (วัดจริงจาก odometry ไม่ใช่แค่บวกเลขคำสั่ง - กันกรณีอ้างอิงกำแพงข้าง
        เดียวคลาดเคลื่อนแล้วไล่ขยับไม่มีที่สิ้นสุดจนหลุดเส้นทาง)

        Args:
            gimbal_yaw (int or None): มุม gimbal ที่ต้องชี้ก่อนอ่าน ToF แต่ละ
                ครั้ง (None = ใช้ค่าที่ gimbal ชี้อยู่แล้ว เช่นแกนหน้า-หลังที่
                ชี้หน้าไว้แล้วตั้งแต่ต้น ไม่ต้องเสียเวลาหมุนซ้ำทุกก้าว)
            sign (int): +1 หรือ -1 บอกทิศของแกน (x หรือ y) ที่ทำให้ระยะ "ลดลง"
                (เข้าใกล้เป้าหมาย) เมื่อ error เป็นบวก
            known_mm (int or None): ค่าที่อ่านมาแล้วสด ๆ จากผู้เรียก ใช้แทนการ
                อ่านซ้ำรอบแรก (ลดจำนวนครั้งที่ต้องหมุน gimbal โดยไม่จำเป็น)
            max_total_mm (float or None): เพดานระยะสะสมที่ยอมให้ขยับในคำสั่ง
                นี้ทั้งหมด (None = ไม่จำกัด - ใช้เมื่อ ``max_iters`` เล็กพอที่
                จะมีเพดานโดยปริยายจาก ``MAX_CREEP_STEP_MM`` อยู่แล้ว)
        """
        def _read():
            if gimbal_yaw is None:
                return self.hub.snapshot().tof_mm
            return self.hub.read_tof(gimbal_yaw)

        final_mm = known_mm if known_mm is not None else _read()
        start_pos = self.hub.snapshot() if max_total_mm is not None else None
        for _ in range(max_iters):
            if final_mm is None:
                break
            error = final_mm - target_mm
            if abs(error) <= tol_mm:
                break

            room_mm = None
            if start_pos is not None:
                now_pos = self.hub.snapshot()
                moved_mm = 1000.0 * math.hypot(now_pos.pos_x - start_pos.pos_x,
                                                now_pos.pos_y - start_pos.pos_y)
                room_mm = max_total_mm - moved_mm
                if room_mm <= 0:
                    print("[CREEP-LIMIT] แกน {0} ขยับสะสมแล้ว {1:.0f}mm ครบ"
                          "เพดาน {2}mm - หยุดไว้ก่อน (กันดริฟท์ไม่มีที่สิ้นสุด)"
                          .format(drive_axis, moved_mm, max_total_mm))
                    break

            desired = min(abs(error), MAX_CREEP_STEP_MM)
            if room_mm is not None:
                desired = min(desired, room_mm)
            if error > 0:
                # ไกลกว่าเป้าหมาย (ยังห่างจากกำแพงที่วัดอยู่) -> ขยับเข้าใกล้
                # ได้ แต่ต้องผ่าน safe_step_mm กันเข้าใกล้เกิน MIN_SAFE_MM
                step_mm = self.safe_step_mm(final_mm, desired)
                if step_mm <= 0:
                    break
                delta_m = sign * (step_mm / 1000.0)
            else:
                # ใกล้กำแพงนี้เกินไปแล้ว -> ถอยออก ไม่มีความเสี่ยงชนกำแพงด้าน
                # นี้เพิ่ม จึงไม่ต้องผ่าน safe_step_mm
                delta_m = -sign * (desired / 1000.0)

            if drive_axis == "x":
                self.bounded_move(dx_m=delta_m)
            else:
                self.bounded_move(dy_m=delta_m)
            final_mm = _read()
        return final_mm

    def center_in_cell(self):
        """จัดกึ่งกลางช่องให้แม่นที่สุดก่อนสแกนกำแพง/ออกเดิน

        หน้า-หลัง: ไล่ ToF หน้าให้เท่ากับ ``FRONT_STOP_MM``
        ซ้าย-ขวา: วนวัด-ตัดสินใจ-ขยับทีละก้าว (ช้าได้เลย) จนกว่าจะนิ่งหรือครบ
        ``LATERAL_MAX_ITERS`` - **แก้เฉพาะตอนเห็นกำแพงสองข้างเท่านั้น** (เทียบ
        ซ้าย=ขวากันเอง ไม่พึ่งค่าคาลิเบรตสัมบูรณ์) เห็นข้างเดียวหรือไม่เห็นเลย
        จะไม่แก้อะไร เพราะเคยพบว่าการแก้โดยอ้างอิง ``SIDE_STOP_MM`` (ค่าคาลิเบรต
        สัมบูรณ์) ข้างเดียว พาหุ่นออกจากกึ่งกลางจริงไปชนกำแพงได้ ถ้า
        ``SIDE_STOP_MM`` ไม่ตรงกับ "กึ่งกลางจริง" ของสนามนั้น ๆ

        Returns:
            dict: ค่า mm/จำนวนรอบ/error สุดท้าย ไว้ log
        """
        result = {"front_mm": None, "left_mm": None, "right_mm": None,
                  "iters": 0, "final_err_x": None, "final_err_y": None}

        self.hub.point_gimbal(GIMBAL_YAW_FRONT)
        front_mm = self._creep_axis(None, FRONT_STOP_MM, ALIGN_TOLERANCE_MM,
                                     "x", +1, max_iters=LATERAL_MAX_ITERS,
                                     max_total_mm=MAX_CREEP_TOTAL_MM)
        result["front_mm"] = front_mm
        if front_mm is not None:
            result["final_err_x"] = front_mm - FRONT_STOP_MM
        print("[CENTER-X] หน้า={0}mm target={1}mm err={2}"
              .format(front_mm, FRONT_STOP_MM, result["final_err_x"]))

        # เพดานระยะสะสมซ้าย-ขวา วัดจริงจาก odometry ตลอดทั้งการจัดกึ่งกลางรอบ
        # นี้ (ไม่ใช่แค่ต่อก้าว) กันกรณีอ้างอิงกำแพงข้างเดียวคลาดเคลื่อน (เช่น
        # SIDE_STOP_MM ไม่ตรง) แล้วไล่ขยับไปทางเดิมซ้ำ ๆ ไม่มีที่สิ้นสุดจนหลุด
        # เส้นทาง - ดูจุดที่พี่รายงานว่า "ขวาไม่มีกำแพง มันขวาจนหลุดเส้นทาง"
        lateral_start = self.hub.snapshot()

        for i in range(LATERAL_MAX_ITERS):
            result["iters"] = i + 1

            now_lateral = self.hub.snapshot()
            moved_mm = 1000.0 * math.hypot(now_lateral.pos_x - lateral_start.pos_x,
                                            now_lateral.pos_y - lateral_start.pos_y)
            if moved_mm >= MAX_TOTAL_LATERAL_MM:
                print("[CENTER-Y] ขยับซ้าย-ขวาสะสมแล้ว {0:.0f}mm ครบเพดาน "
                      "{1}mm - หยุดจัดกึ่งกลางไว้ก่อน (กันหลุดเส้นทาง)"
                      .format(moved_mm, MAX_TOTAL_LATERAL_MM))
                break

            left_mm = self.hub.read_tof(GIMBAL_YAW_LEFT)
            right_mm = self.hub.read_tof(GIMBAL_YAW_RIGHT)
            result["left_mm"], result["right_mm"] = left_mm, right_mm

            left_wall = left_mm is not None and left_mm < wall_cutoff_mm(SIDE_STOP_MM)
            right_wall = right_mm is not None and right_mm < wall_cutoff_mm(SIDE_STOP_MM)
            print("[CENTER-Y] รอบ {0}/{1} ซ้าย={2}mm(wall={3}) ขวา={4}mm(wall={5}) "
                  "สะสม={6:.0f}mm"
                  .format(i + 1, LATERAL_MAX_ITERS, left_mm, left_wall,
                          right_mm, right_wall, moved_mm))

            if left_wall and right_wall:
                # มีกำแพงสองข้าง - เทียบซ้าย=ขวากันเอง (self-referencing) ไม่
                # ต้องพึ่งค่า SIDE_STOP_MM ที่คาลิเบรตมาเลย ปลอดภัยกว่าเพราะ
                # ถ้าคาลิเบรตคลาดเคลื่อน จะแค่ทำให้ไม่ปรับ (ซ้าย≈ขวาอยู่แล้ว)
                # ไม่มีทางถูกลากไปทางใดทางหนึ่งแบบผิด ๆ
                err = left_mm - right_mm
                if abs(err) <= LATERAL_TOLERANCE_MM:
                    result["final_err_y"] = err
                    print("[CENTER-Y] นิ่งแล้ว err={0}mm (เทียบซ้าย=ขวา)".format(err))
                    break
                print("[CENTER-Y] -> ก้าวเข้าซ้าย ไล่ให้ซ้าย={0}mm (เท่าขวาตอนนี้)"
                      .format(right_mm))
                self._creep_axis(GIMBAL_YAW_LEFT, right_mm, LATERAL_TOLERANCE_MM,
                                  "y", -1, max_iters=1, known_mm=left_mm)
            else:
                # เห็นกำแพงข้างเดียวหรือไม่เห็นเลย - "ไม่แก้" โดยตั้งใจ เพราะ
                # การแก้โดยอ้างอิง SIDE_STOP_MM (ค่าคาลิเบรตสัมบูรณ์) ข้างเดียว
                # เคยพาหุ่นวิ่งเข้าหากำแพงทั้งที่วางไว้กึ่งกลางแล้วจริง ๆ (ถ้า
                # SIDE_STOP_MM ไม่ตรงกับ "กึ่งกลางจริง" ของสนามนี้ การแก้แบบ
                # นี้จะยิ่งพาออกจากศูนย์กลาง ไม่ใช่เข้าใกล้) เชื่อตำแหน่งที่
                # วางไว้ตั้งแต่ต้น/จากการเดินตรงเข้ามาแทน
                result["final_err_y"] = None
                print("[CENTER-Y] เห็นกำแพงข้างเดียวหรือไม่เห็นเลย - ไม่แก้ "
                      "ซ้าย-ขวา (เชื่อตำแหน่งที่วาง/เดินเข้ามาแทน)")
                break

        self.hub.point_gimbal(GIMBAL_YAW_FRONT)
        return result

    def scan_four_walls(self, heading):
        """หมุน gimbal ส่อง ToF ทั้ง 4 ทิศสัมบูรณ์ (N/E/S/W) รอบช่องปัจจุบัน

        Returns:
            dict: {ทิศสัมบูรณ์: {"wall": bool, "mm": int|None, "label": str}}
        """
        readings = {}
        for rel, gyaw, label in SCAN_PLAN:
            mm = self.hub.read_tof(gyaw)
            base_mm = FRONT_STOP_MM if rel in (0, 2) else SIDE_STOP_MM
            wall = mm is not None and mm < wall_cutoff_mm(base_mm)
            absolute_dir = (heading + rel) % 4
            readings[absolute_dir] = {"wall": wall, "mm": mm, "label": label}
        self.hub.point_gimbal(GIMBAL_YAW_FRONT)
        return readings

    # ---------- การเดินหน้า/ถอย: chassis.move() ก้าวเดียวเต็มระยะ ----------
    def advance_one_cell(self, front_mm):
        """เดินเข้าช่องใหม่หนึ่งช่อง (``chassis.move()`` ก้าวเดียวเต็มระยะ
        ``CELL_SIZE_M``)

        รับ ``front_mm`` ที่อ่านสด ๆ มาก่อนแล้ว (จากขั้นตอน sense ก่อน
        ตัดสินใจ) เช็คว่ามีที่ว่างพอสำหรับเดินเต็มช่องจริงก่อนสั่งเสมอ -
        ``chassis.move()`` ยกเลิกกลางทางไม่ได้ จึงต้องมั่นใจก่อนสั่งเท่านั้น
        ไม่ใช่คอยเบรกระหว่างทางแบบเดิม หลังเดินเสร็จวัดระยะที่เดินได้จริงจาก
        odometry ของเราเอง (ไม่เชื่อแค่ผลลัพธ์คำสั่งเฉย ๆ)

        Returns:
            tuple: (ok, traveled_m, reason)
        """
        cell_mm = CELL_SIZE_M * 1000.0
        if front_mm is None or front_mm < cell_mm + MIN_SAFE_MM:
            print("[ADVANCE] หน้า={0}mm ไม่พอสำหรับเดินเต็มช่อง (ต้องการ "
                  "{1:.0f}mm+เผื่อ {2}mm) - ปฏิเสธไม่เดิน"
                  .format(front_mm, cell_mm, MIN_SAFE_MM))
            return False, 0.0, "not_enough_clearance"

        start = self.hub.snapshot()
        confirmed = self.bounded_move(dx_m=CELL_SIZE_M)
        end = self.hub.snapshot()
        traveled = math.hypot(end.pos_x - start.pos_x, end.pos_y - start.pos_y)
        ok = traveled >= CELL_SIZE_M * CELL_COMPLETE_RATIO
        print("[MOVE] สั่งเดินหน้า {0:.2f} m (ยืนยันจากคำสั่ง={1}) วัดจาก "
              "odometry ได้จริง {2:.3f} m -> {3}"
              .format(CELL_SIZE_M, confirmed, traveled,
                      "สำเร็จ" if ok else "ไม่สำเร็จ"))
        return ok, traveled, ("odometry_confirmed" if ok else "short")

    def backup(self, distance_m):
        """ถอยกลับตามระยะที่กำหนด ด้วย ``chassis.move()`` ก้าวเดียว (x ติดลบ
        เป็นพิกัดสัมพัทธ์กับตัวรถเองอยู่แล้ว ไม่ต้องรับ heading/คำนวณ yaw
        เป้าหมายเหมือนเดิม)
        """
        if distance_m < 0.03:
            return
        print("[BACK] ถอยกลับ {0:.3f} m เข้าช่องเดิม".format(distance_m))
        self.bounded_move(dx_m=-distance_m)


# =====================================================================
# ช่วย export ผลลัพธ์ (Map / Log / Trajectory)
# =====================================================================
def export_run(maze, ts, start_cell, start_heading, end_cell, end_heading,
                steps_rows, trajectory_rows, conflicts):
    """เซฟผลการสำรวจเป็นไฟล์: แผนที่ / log การตัดสินใจ / trajectory / สรุป"""
    map_txt_path = "{0}_map_{1}.txt".format(LOG_PREFIX, ts)
    with open(map_txt_path, "w", encoding="utf-8") as f:
        f.write(maze.render(robot=(end_cell[0], end_cell[1], end_heading), legend=True))
        f.write("\n")

    map_csv_path = "{0}_map_{1}.csv".format(LOG_PREFIX, ts)
    with open(map_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for y in range(maze.height - 1, -1, -1):
            row = []
            for x in range(maze.width):
                if maze.is_visited(x, y):
                    row.append(0)
                elif all(maze.has_wall(x, y, d) for d in range(4)):
                    row.append(1)
                else:
                    row.append(0.5)
            writer.writerow(row)

    steps_csv_path = "{0}_steps_{1}.csv".format(LOG_PREFIX, ts)
    with open(steps_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "x", "y", "heading",
                          "tof_front_mm", "tof_right_mm", "tof_back_mm", "tof_left_mm",
                          "wall_N", "wall_E", "wall_S", "wall_W",
                          "center_iters", "center_err_x_mm", "center_err_y_mm",
                          "kind", "target", "ok", "hop_time_s", "impact", "slip"])
        writer.writerows(steps_rows)

    traj_csv_path = "{0}_trajectory_{1}.csv".format(LOG_PREFIX, ts)
    with open(traj_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "step", "phase", "x", "y", "heading",
                          "pos_x", "pos_y", "yaw"])
        writer.writerows(trajectory_rows)

    seen, total_edges = maze.edge_stats()
    summary_lines = [
        "เริ่มที่ {0} หัน {1}".format(start_cell, DIR_NAMES[start_heading]),
        "สิ้นสุดที่ {0} หัน {1}".format(end_cell, DIR_NAMES[end_heading]),
        "สำรวจ {0}/{1} ช่อง ({2:.1f}%)".format(
            len(maze.visited), maze.width * maze.height, maze.coverage_pct()),
        "ด้านที่ตรวจแล้ว {0}/{1}".format(seen, total_edges),
        "จุดที่สแกนซ้ำแล้วขัดแย้งกับความรู้เดิม (ไม่ทับให้): {0}".format(len(conflicts)),
    ]
    for cx, cy, cd in conflicts:
        summary_lines.append("  - ช่อง {0} ทิศ {1}".format((cx, cy), DIR_NAMES[cd]))
    summary_path = "{0}_summary_{1}.txt".format(LOG_PREFIX, ts)
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines) + "\n")

    print("\n[EXPORT] บันทึกผลลัพธ์แล้ว:")
    for path in (map_txt_path, map_csv_path, steps_csv_path, traj_csv_path, summary_path):
        print("  - {0}".format(path))


# =====================================================================
# State machine หลัก - สำรวจให้ครบทุกช่องที่ไปถึงได้ (full coverage)
# =====================================================================
def run_explore(hub, driver):
    """สำรวจแผนที่ที่ไม่รู้จักมาก่อนให้ครบทุกช่องที่ไปถึงได้ แล้ว export ผลลัพธ์

    Returns:
        bool: True เมื่อสำรวจจบแบบ "done"
    """
    ts = _timestamp()
    t0 = time.time()
    maze = Maze(MAZE_W, MAZE_H, BORDER_OPENINGS)
    planner = ExplorePlanner(START_CELL)
    x, y = START_CELL
    heading = START_HEADING

    print("=" * 62)
    print("  SLAM EXPLORE (ToF + gimbal เท่านั้น) - สำรวจให้ครบทุกช่อง")
    print("  สนาม {0}x{1} ({2} ช่อง) ช่องละ {3:.2f} m | เริ่มที่ {4} หัน {5}"
          .format(MAZE_W, MAZE_H, MAZE_W * MAZE_H, CELL_SIZE_M, START_CELL,
                  DIR_NAMES[START_HEADING]))
    print("=" * 62)

    steps_rows = []
    trajectory_rows = []
    conflicts = []
    fail_key = None
    fail_count = 0
    done = False

    def _log_traj(step, phase):
        snap = hub.snapshot()
        trajectory_rows.append([
            round(time.time() - t0, 3), step, phase, x, y, DIR_NAMES[heading],
            round(snap.pos_x, 4), round(snap.pos_y, 4), round(snap.yaw, 2),
        ])

    def _emg():
        impact, slip, _ = hub.emergency_flags()
        return impact, slip

    # ครอบทั้งลูปด้วย try/finally เพื่อไม่ให้ log หายไปเปล่า ๆ เวลาชนกำแพง/
    # กด Ctrl+C/error กลางคัน - เดิมมี export อยู่หลังลูปเฉย ๆ ไม่มีอะไรครอบ พอ
    # เกิด exception กลางทาง (เช่นหุ่นชนแล้วต้องกดหยุดฉุกเฉิน) ข้อมูลที่เก็บไว้
    # ในตัวแปรจะหายหมดไม่ถูกเขียนลงไฟล์เลย
    try:
        for step in range(MAX_STEPS):
            print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} ---"
                  .format(step, x, y, DIR_NAMES[heading]))
            hop_t0 = time.time()

            driver.stop()
            time.sleep(SETTLE_S)

            # เช็คเซฟตี้เน็ตอิสระจากฮาร์ดแวร์ (ไม่เกี่ยวกับ ToF เลย) ก่อนทำ
            # อะไรทั้งนั้นทุกรอบ - ถ้าเคยชน/ล้อลื่นจริง หยุดภารกิจทั้งหมดทันที
            # ไม่สำรวจต่อ
            impact, slip, roll_over = hub.emergency_flags()
            if impact or roll_over:
                print("[EMERGENCY] ตรวจพบการชน/พลิกคว่ำจากฮาร์ดแวร์จริง "
                      "(impact={0}, roll_over={1}) - หยุดภารกิจทั้งหมดทันที"
                      .format(impact, roll_over))
                driver.stop()
                break

            snap = hub.snapshot()
            if not snap.fresh:
                print("[ERROR] เซนเซอร์ {0} ขาดการอัปเดต หยุดเพื่อความปลอดภัย"
                      .format(snap.stale_reason))
                break

            _log_traj(step, "arrive")
            center = driver.center_in_cell()
            _log_traj(step, "centered")
            print("[CENTER] รอบ={0} หน้า={1}mm(err={2}) ซ้าย={3}mm ขวา={4}mm err_y={5}"
                  .format(center["iters"], center["front_mm"], center["final_err_x"],
                          center["left_mm"], center["right_mm"], center["final_err_y"]))

            readings = driver.scan_four_walls(heading)
            print("สแกน 4 ทิศ -> " + ", ".join(
                "{0}:{1}({2}mm)".format(DIR_NAMES[d], "WALL" if r["wall"] else "open", r["mm"])
                for d, r in sorted(readings.items())))

            for direction, r in readings.items():
                if r["wall"] and maze.is_known(x, y, direction) and not maze.has_wall(x, y, direction):
                    conflicts.append((x, y, direction))
                    print("[CONFLICT] ช่อง({0},{1}) ทิศ {2} เคยยืนยันว่าโล่ง แต่สแกนรอบนี้"
                          "เจอ wall - ไม่ทับความรู้เดิม (เก็บไว้เป็นสัญญาณเตือน)"
                          .format(x, y, DIR_NAMES[direction]))
                else:
                    maze.set_wall(x, y, direction, r["wall"])

            maze.mark_visited(x, y)
            print("\n[MAP] แผนที่ที่หุ่นรับรู้ ณ ตอนนี้ (v/^/</> = ตำแหน่ง+ทิศหุ่น):")
            print(maze.render(robot=(x, y, heading), legend=True))

            wall_row = [int(maze.has_wall(x, y, d)) for d in (NORTH, EAST, SOUTH, WEST)]
            tof_row = [readings[(heading + rel) % 4]["mm"] for rel in (0, 1, 2, 3)]

            kind, target = planner.next_move(maze, x, y, heading)
            if kind == "done":
                steps_rows.append([step, x, y, DIR_NAMES[heading], *tof_row, *wall_row,
                                    center["iters"], center["final_err_x"],
                                    center["final_err_y"], "done", "", True,
                                    round(time.time() - hop_t0, 2), *_emg()])
                print("\n[DONE] สำรวจครบทุกช่องที่ไปถึงได้แล้ว ใช้ไป {0} ก้าว".format(step))
                done = True
                break

            next_heading = _edge_direction((x, y), target)
            print("ตัดสินใจ -> {0} ไปช่อง {1} (ทาง {2})"
                  .format("เดินต่อ" if kind == "go" else "ย้อนกลับ/reroute",
                          target, DIR_NAMES[next_heading]))

            heading = driver.turn_to(heading, next_heading)
            _log_traj(step, "turned")

            snap2 = hub.snapshot()
            front_blocked = (snap2.tof_mm is not None
                             and snap2.tof_mm < wall_cutoff_mm(FRONT_STOP_MM))
            if front_blocked:
                if kind == "go":
                    print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm - ยกเลิกการเดิน "
                          "แล้วมาร์กลงแผนที่".format(snap2.tof_mm))
                    maze.set_wall(x, y, heading, True)
                    if planner.stack and planner.stack[-1] == target:
                        planner.stack.pop()
                else:
                    print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm บนทางที่เคยยืนยันว่าโล่ง "
                          "- ไม่แก้แผนที่ น่าจะหลงตำแหน่ง".format(snap2.tof_mm))
                steps_rows.append([step, x, y, DIR_NAMES[heading], *tof_row, *wall_row,
                                    center["iters"], center["final_err_x"],
                                    center["final_err_y"], kind, target, False,
                                    round(time.time() - hop_t0, 2), *_emg()])
                continue

            ok, traveled, _ = driver.advance_one_cell(snap2.tof_mm)
            _log_traj(step, "advanced")
            steps_rows.append([step, x, y, DIR_NAMES[heading], *tof_row, *wall_row,
                                center["iters"], center["final_err_x"],
                                center["final_err_y"], kind, target, ok,
                                round(time.time() - hop_t0, 2), *_emg()])

            if ok:
                x, y = target
                fail_key = None
                fail_count = 0
            else:
                key = (x, y, heading)
                fail_count = fail_count + 1 if key == fail_key else 1
                fail_key = key

                if kind == "go":
                    snap3 = hub.snapshot()
                    front_wall_now = (snap3.tof_mm is not None
                                      and snap3.tof_mm < wall_cutoff_mm(FRONT_STOP_MM))
                    if front_wall_now:
                        print("[RECOVER] ยืนยันด้วย ToF ว่ามีกำแพงจริง มาร์กลงแผนที่")
                        maze.set_wall(x, y, heading, True)
                    elif fail_count >= 2:
                        print("[RECOVER] เดินไม่ผ่านทางเดิมเป็นครั้งที่ {0} ทั้งที่ ToF "
                              "ว่าโล่ง - ปิดทางนี้ไว้ก่อนเพื่อไม่ให้ติดวนอยู่ที่เดิม"
                              .format(fail_count))
                        maze.set_wall(x, y, heading, True)
                    else:
                        print("[RECOVER] ToF บอกว่าข้างหน้าโล่ง แต่เดินไม่ไป "
                              "น่าจะล้อลื่นหรือติดขัด - ลองใหม่อีกครั้งก่อนตัดสิน")
                    if planner.stack and planner.stack[-1] == target:
                        planner.stack.pop()
                else:
                    print("[RECOVER] ย้อนกลับ/reroute ไปช่อง {0} ไม่สำเร็จ (ครั้งที่ {1}) "
                          "- ไม่แก้แผนที่ เพราะทางนี้เคยยืนยันว่าโล่งแล้ว"
                          .format(target, fail_count))
                    if fail_count >= BACK_RETRY:
                        if planner.has_other_exit(maze, (x, y), target):
                            print("[RECOVER] ปิดทางนี้ไว้ก่อน มีทางออกอื่นให้ไปแทน")
                            planner.block_edge((x, y), target)
                        else:
                            print("[RECOVER] ทางนี้เป็นทางออกทางเดียวของช่องนี้ "
                                  "- ไม่ปิด ลองใหม่ต่อไป")
                        fail_key = None
                        fail_count = 0
                driver.backup(traveled)
        else:
            print("\n[FAIL] ครบ {0} ก้าวแล้วยังสำรวจไม่ครบ".format(MAX_STEPS))
    finally:
        # ส่วนนี้ทำงานเสมอ ไม่ว่าจะจบแบบ done/FAIL/break หรือ exception กลางคัน
        # (ชนกำแพงจนต้อง Ctrl+C, sensor error ฯลฯ) เพื่อให้มี log ไว้ดูย้อนหลัง
        # ทุกครั้ง ไม่ใช่แค่ตอนที่จบแบบสวย ๆ เท่านั้น
        driver.stop()
        seen, total_edges = maze.edge_stats()
        print("\n" + "=" * 62)
        print("  สรุปผล")
        print("  เริ่มที่ {0} หัน {1}".format(START_CELL, DIR_NAMES[START_HEADING]))
        print("  สิ้นสุดที่ {0} หัน {1}".format((x, y), DIR_NAMES[heading]))
        print("  สำรวจ {0}/{1} ช่อง ({2:.1f}%) | ด้านที่ตรวจแล้ว {3}/{4} | ขัดแย้ง {5} จุด"
              .format(len(maze.visited), MAZE_W * MAZE_H, maze.coverage_pct(),
                      seen, total_edges, len(conflicts)))
        print("=" * 62)

        export_run(maze, ts, START_CELL, START_HEADING, (x, y), heading,
                   steps_rows, trajectory_rows, conflicts)

    return done


# =====================================================================
# โหมด --calib : วัดค่า ToF จริงจากหุ่นตัวนี้ในสนามนี้
# =====================================================================
def run_calibration(hub):
    """วัด FRONT_STOP_MM/SIDE_STOP_MM สั้นกว่า SLAM.py มาก (ToF ไม่ต้องทำ
    hysteresis แบบ Sharp) หุ่นจะไม่ขยับตลอด (แค่ gimbal หมุนเอง)
    """
    print("\n" + "=" * 62)
    print("  โหมดคาลิเบรต ToF - หุ่นจะไม่ขยับ (gimbal จะหมุนไปมองเอง)")
    print("=" * 62)

    print("\n[1/2] วางหุ่น 'กลางช่อง' หันหน้าเข้าชนกำแพง")
    input("      พร้อมแล้วกด Enter...")
    front_samples = []
    for _ in range(30):
        mm = hub.read_tof(GIMBAL_YAW_FRONT)
        if mm is not None:
            front_samples.append(mm)
        time.sleep(0.05)
    front_mean, front_sd = _stats(front_samples)

    print("\n[2/2] วางหุ่น 'กลางช่อง' ที่มีกำแพงอยู่ทาง 'ซ้าย' ของหุ่น "
          "(ไม่ต้องหันเข้าหา ใช้ gimbal หันไปมองเอง)")
    input("      พร้อมแล้วกด Enter...")
    side_samples = []
    for _ in range(30):
        mm = hub.read_tof(GIMBAL_YAW_LEFT)
        if mm is not None:
            side_samples.append(mm)
        time.sleep(0.05)
    hub.point_gimbal(GIMBAL_YAW_FRONT)
    side_mean, side_sd = _stats(side_samples)

    print("\n" + "-" * 62)
    print("  ผลการคาลิเบรต")
    print("-" * 62)
    problems = []
    if front_mean is None:
        problems.append("FRONT: ไม่ได้ค่า ToF เลย ตรวจว่าเสียบโมดูลและ gimbal หันหน้าจริง")
    else:
        print("  หน้า:  mean={0:.1f}mm sd={1:.1f}".format(front_mean, front_sd))
    if side_mean is None:
        problems.append("SIDE: ไม่ได้ค่า ToF เลย ตรวจว่ามีกำแพงอยู่ทางซ้ายจริงในระยะที่วัดได้")
    else:
        print("  ข้าง:  mean={0:.1f}mm sd={1:.1f}".format(side_mean, side_sd))

    if problems:
        print("\n  พบปัญหาที่ควรแก้ก่อนลงสนามจริง:")
        for item in problems:
            print("    * {0}".format(item))

    print("\nคัดลอกบล็อกนี้ไปวางทับใน CONFIG ด้านบนของไฟล์:\n")
    print("FRONT_STOP_MM         = {0}"
          .format(int(round(front_mean)) if front_mean is not None else "None   # วัดไม่ได้"))
    print("SIDE_STOP_MM          = {0}"
          .format(int(round(side_mean)) if side_mean is not None else "None   # วัดไม่ได้"))
    print()


# =====================================================================
# MAIN
# =====================================================================
def main():
    parser = argparse.ArgumentParser(
        description="SLAM สำรวจด้วย ToF+gimbal อย่างเดียว (ไม่มี Sharp/IR) "
                    "สำหรับ RoboMaster EP (Class Work 8)")
    parser.add_argument("--calib", action="store_true",
                        help="วัดค่า ToF จริง หุ่นจะไม่ขยับ (gimbal หมุนเอง)")
    parser.add_argument("--conn", default=CONN_TYPE,
                        choices=["ap", "sta", "rndis"],
                        help="วิธีเชื่อมต่อหุ่น (ค่าเริ่มต้น {0})".format(CONN_TYPE))
    args = parser.parse_args()

    if robot is None:
        print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
        print("        ติดตั้ง SDK ก่อน")
        return 1

    if not args.calib:
        require_calibration()

    ep_robot = robot.Robot()
    print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
    ep_robot.initialize(conn_type=args.conn)

    print("กำลัง recenter gimbal ...")
    ep_robot.gimbal.recenter().wait_for_completed()

    hub = SensorHub(ep_robot)
    success = False
    try:
        hub.start()
        hub.calibrate_gimbal_yaw_sign()
        if args.calib:
            run_calibration(hub)
            success = True
        else:
            ep_robot.set_robot_mode(robot.FREE)
            time.sleep(0.5)

            driver = Driver(ep_robot.chassis, hub)
            success = run_explore(hub, driver)
    except KeyboardInterrupt:
        print("\n[STOP] ผู้ใช้สั่งหยุด")
    finally:
        try:
            ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))
        hub.stop()
        try:
            ep_robot.close()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ปิดการเชื่อมต่อไม่สำเร็จ: {0}".format(exc))

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
