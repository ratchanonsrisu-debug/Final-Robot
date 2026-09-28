"""Class Work 8 - SLAM: สำรวจแผนที่ไม่รู้จักให้ครบ สำหรับ RoboMaster EP

โหมดการทำงาน
------------
สำรวจเขาวงกตที่ไม่รู้จักแผนที่มาก่อนให้ครบทุกช่องที่ไปถึงได้ (full-coverage
exploration ด้วย DFS + backtrack) สร้าง occupancy map จากเซนเซอร์ระหว่างทาง
จบภารกิจแล้วรายงานตำแหน่งเริ่ม/จบ พร้อม export แผนที่ / log การสำรวจ /
trajectory ออกเป็นไฟล์

สถาปัตยกรรมที่ตกลงกันไว้ (เวอร์ชันนี้ ต่างจากฉบับแรก)
-----------------------------------------------------
- **ToF บน gimbal คือความจริงหนึ่งเดียวเรื่องกำแพง** หมุน gimbal สแกนครบ 4 ทิศ
  (หน้า/ขวา/หลัง/ซ้าย) ทุกครั้งที่จอดในช่องใหม่ ก่อนหน้านี้ gimbal ถูกล็อกหน้า
  ตลอดภารกิจ - เวอร์ชันนี้เลิกทำแบบนั้นแล้ว เพราะไม่มีทางเห็นกำแพงข้าง/หลังได้
  ถ้าไม่หมุน
- **Sharp ซ้าย/ขวา (ข้างละ 1 ตัว) ใช้เพื่อ "จัดกึ่งกลางซ้าย-ขวาตอนจอด" เท่านั้น**
  ไม่ใช่ตัวตัดสินกำแพง (ToF ตัดสินกำแพงล้วน ๆ) และไม่ทำงานระหว่างเดินหน้าเลย
- **IR 45 องศา 2 ตัว (ทแยงซ้าย-หน้า/ขวา) เป็นตัวกระตุ้น emergency-stop ระหว่าง
  เดินหน้าเข้าช่องใหม่เท่านั้น** ไม่ใช้ตัดสินกำแพง ไม่ทำงานตอนจอด
- **ห้ามมีการปรับทิศ/ตำแหน่งด้วยเซนเซอร์ใด ๆ ระหว่างเดินหน้าเข้าช่องเด็ดขาด**
  (ตกลงกับผู้ใช้ชัดเจนหลังพบว่าเวอร์ชันก่อนหน้า - ดู
  ``Occupancy110.py``/``occupancy110-lateral-hold`` - เคยลองใส่ Sharp-skew→yaw
  ระหว่างเดินหน้าแล้วไปหักล้างกับ IMU yaw-hold จนดริฟท์ไม่จบสิ้น) เดินหน้าเข้า
  ช่องใหม่ทั้งหมดเป็นความเร็วคงที่ค่าเดียว ปรับได้แค่ตอนจอดนิ่งเท่านั้น
- **การหมุนเลี้ยวใช้ ``chassis.move(z=)`` ก้อนเดียวยืนยันจากเฟิร์มแวร์** (แม่น,
  เสี่ยงชนต่ำ เพราะเกิดขึ้นตอนอยู่กลางช่องที่จัดตำแหน่งแล้ว)
- **เดินหน้าเข้าช่องใหม่ใช้ ``chassis.drive_speed()`` ความเร็วคงที่ค่าเดียว**
  (ไม่ใช่ ``chassis.move()``) เพราะ ``chassis.move()`` ของ SDK ตัวนี้ไม่มีทาง
  ยกเลิกกลางทางได้เลย ส่วน ``drive_speed`` สั่ง 0 แทรกแล้วหยุดจริงทันที จึงเป็น
  ทางเดียวที่ IR/impact จะ "หยุดหุ่นได้จริง" ไม่ใช่ best-effort
- **การจัดกึ่งกลางช่องใช้ ``chassis.move()`` ทีละก้อน** อ่าน-ตัดสินใจ-ขยับ-อ่านใหม่
  เหมือนเดิม (front-back ด้วย ToF ที่สแกนไปแล้ว, ซ้าย-ขวาด้วย Sharp)

ฮาร์ดแวร์ที่ต้องต่อ (อัปเดตตามสายจริงที่ต่อในเครื่อง)
------------------------------------------------------
- ToF (Distance Sensor)                 -> sensor.sub_distance() ติดบน gimbal
  หมุนได้เต็มที่ (gimbal.moveto ก่อนอ่านทุกทิศ แล้วชี้หน้ากลับก่อนออกเดินเสมอ)
- Sharp IR analog ซ้าย                  -> sensor adaptor hub 3 port 1 (ADC)
- Sharp IR analog ขวา                   -> sensor adaptor hub 2 port 2 (ADC)
- IR digital 45 องศา ซ้าย-หน้า          -> sensor adaptor hub 1 port 1 (IO)
- IR digital 45 องศา ขวา                -> sensor adaptor hub 2 port 1 (IO)
- Camera ต่อไว้เฉยๆ ไม่ใช้ในตรรกะการสำรวจ/สร้างแผนที่เลย (ตกลงกับผู้ใช้แล้ว)

วิธีใช้
------
    python SLAM.py --calib    วัดค่าเซนเซอร์จริง (ต้องทำก่อนใช้งานครั้งแรก)
    python SLAM.py --sim      ทดสอบตรรกะสำรวจ โดยไม่ต้องต่อหุ่น
    python SLAM.py            วิ่งจริงในสนาม

ข้อควรรู้เรื่อง SDK (ตรวจสอบจาก source ใน src/robomaster/ แล้ว)
--------------------------------------------------------------
1. ``chassis.move(z=)`` กับ ``chassis.drive_speed(z=)`` ใช้เครื่องหมายตรงข้ามกัน
   ตัวอย่างทางการ examples/02_chassis/01_move.py ระบุ ``move(z=+90)`` = เลี้ยวซ้าย
   ส่วน examples/02_chassis/03_speed.py ระบุ ``drive_speed(z=+30)`` = เลี้ยวขวา
   ไฟล์นี้ใช้ ``move(z=)`` สำหรับเลี้ยว และ ``drive_speed(x=)`` (ไม่มี z) สำหรับ
   เดินหน้าเข้าช่องใหม่ - ไม่มีจุดไหนใช้ ``drive_speed(z=)`` เลยจึงไม่ต้องกังวล
   เรื่องเครื่องหมายสลับกันของสองคำสั่งนี้อีก
2. ``chassis.move()`` **ไม่มี abort ใน SDK** ยกเลิกกลางทางไม่ได้เลย - เหตุผลที่
   การเดินหน้าเข้าช่องใหม่ (จุดเสี่ยงชนที่สุด) ใช้ ``drive_speed()`` แทน เพราะ
   สั่ง 0 แทรกได้จริงและหยุดจริงทันที ไม่ใช่ best-effort
3. ``sensor_adaptor.get_adc()`` / ``get_io()`` เป็น blocking round-trip ที่มี
   timeout 3 วินาที (client.py ``send_sync_msg``) เรียกใน control loop ไม่ได้
   จึงใช้ ``sensor_adaptor.sub_adapter()`` แบบ push แทน และเหลือ get_adc ไว้เป็น
   fallback กรณี sub_adapter ไม่ส่งข้อมูลมาเท่านั้น
4. ค่าที่ ``sub_adapter`` คืนมาเป็น ADC ดิบสเกลเดียวกับ ``get_adc`` (uint16 ทั้งคู่
   ดู protocol.py:1954 เทียบกับ sensor.py:75) สลับไปมาได้โดยค่าไม่เพี้ยน
5. ``sensor_adaptor.start()`` เป็น no-op (module.py ``Module.start`` = pass)
6. ``robot.initialize()`` เรียก ``set_robot_mode(FREE)`` ให้อยู่แล้ว แต่ gimbal ใน
   FREE mode **ไม่ตามแชสซีหมุนอัตโนมัติ** ต้องสั่ง ``point_gimbal`` ชี้หน้าซ้ำ
   ทุกครั้งหลังหมุนแชสซี ไม่งั้น ToF จะชี้ผิดทิศแบบเงียบ ๆ
7. ToF อ่านได้ ``0`` หรือ ``< TOF_BLIND_ZONE_MM`` (~60mm) แปลว่า "ชิดวัตถุมาก
   จนสะท้อนไม่ทัน (blind zone)" ต้องตีความว่า **มีกำแพงชัวร์** ห้ามตีความว่า
   "ไม่มีข้อมูล/ไม่มีอะไรในระยะ" (บั๊กที่เจอจาก ``IR_6/sensors.py`` ของแล็บก่อน
   หน้า ถ้าตีความผิดจะทำให้หุ่นคิดว่าโล่งทั้งที่ชนกำแพงอยู่)
"""

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError as exc:  # โหมด --sim ทดสอบตรรกะได้โดยไม่ต้องมี SDK ติดตั้ง
    robot = None
    ROBOT_IMPORT_ERROR = exc
else:
    ROBOT_IMPORT_ERROR = None

try:
    import cv2                  # noqa: ใช้แค่คำนวณ % พิกเซลขาวจากกล้อง (log-only)
except ImportError:
    cv2 = None


# =====================================================================
# CONFIG - แก้ค่าทั้งหมดที่นี่ที่เดียว
# =====================================================================

# ---------- ขนาดสนาม (Class Work 8: สำรวจให้ครบ 5x4 ช่อง = 20 ช่อง) ----------
MAZE_W = 4                      # จำนวนช่องแกน X (ทิศตะวันออกเป็นบวก) 0..3
MAZE_H = 5                      # จำนวนช่องแกน Y (ทิศเหนือเป็นบวก) 0..4
CELL_SIZE_M = 0.60              # ความกว้าง 1 ช่อง หน่วยเมตร
START_CELL = (0, 0)
START_HEADING = 0               # 0=North 1=East 2=South 3=West
# ช่องเปิดจริงตรงขอบสนาม (ไม่ใช่กำแพงตัน) แก้ตามสนามจริงของแต่ละรอบ/แต่ละกลุ่ม
# ทิศ: 0=N 1=E 2=S 3=W (ใช้เลขดิบเพราะค่านี้ต้องกำหนดก่อนที่ค่าคงที่ NORTH/EAST
# ฯลฯ จะถูกประกาศด้านล่างของไฟล์) ตอนนี้ยังไม่รู้ว่าเป็น N หรือ E ที่ (3,4) จึง
# ใส่ไว้ทั้งคู่ ปล่อยให้เซนเซอร์ตอนไปถึงช่องนั้นเป็นคนตัดสินว่าด้านไหนโล่งจริง
# (ด้านที่จริง ๆ ยังตันก็จะถูกมาร์กเป็นกำแพงตามปกติจากการสแกน ไม่ต่างจากขอบอื่น)
BORDER_OPENINGS = [(3, 4, 0), (3, 4, 1)]

# ---------- การต่อสายเซนเซอร์ (อัปเดตตามสายจริงที่ต่อในเครื่อง) ----------
TOF_INDEX = 0                   # sub_distance คืน list 4 ตัว ใช้ตัวไหน (ToF อยู่บน gimbal)
SHARP_LEFT = (3, 1)             # (hub_id, port) อ่านด้วย ADC - ใช้จัดกึ่งกลางตอนจอดเท่านั้น
SHARP_RIGHT = (2, 2)            # (hub_id, port) อ่านด้วย ADC - ใช้จัดกึ่งกลางตอนจอดเท่านั้น
IR_LEFT_45 = (1, 1)             # (hub_id, port) อ่านด้วย IO - ทแยงซ้าย-หน้า, e-stop ระหว่างเดินหน้าเท่านั้น
IR_RIGHT_45 = (2, 1)            # (hub_id, port) อ่านด้วย IO - ทแยงขวา, e-stop ระหว่างเดินหน้าเท่านั้น

# ---------- ค่าที่ได้จาก `--calib` เท่านั้น ห้ามเดา ----------
# ตราบใดที่ยังเป็น None โปรแกรมจะปฏิเสธที่จะวิ่งในสนามจริง
# เหตุผล: threshold ที่ผิดทำให้หุ่น "วิ่งดูปกติทุกอย่างแต่สร้างแผนที่ผิด"
# ซึ่งแยกไม่ออกจากบั๊กของ odometry หรือของ flood fill ตอนอยู่หน้าสนาม
FRONT_STOP_MM                 = 140
SIDE_STOP_MM                  = 142
SHARP_LEFT_REF                = 495
SHARP_RIGHT_REF               = 504
SHARP_LEFT_CLOSER_IS_HIGHER   = True
SHARP_RIGHT_CLOSER_IS_HIGHER  = True
IR_TRIGGERED_VALUE            = 1  # ค่า IO ตอนมีสิ่งกีดขวาง (0 หรือ 1) ใช้ร่วมกันทั้ง 2 ตัว

FRONT_WALL_MM_OVERRIDE = None   # ปกติปล่อย None ให้คำนวณจากเรขาคณิตของช่อง

# ---------- เกณฑ์ตรวจกำแพง (ToF คือความจริงหนึ่งเดียว) ----------
TOF_MAX_VALID_MM = 4000         # เกินนี้ถือว่าอ่านไม่ได้ / ไม่มีอะไรอยู่ในระยะ (โล่ง)
TOF_BLIND_ZONE_MM = 60          # ต่ำกว่านี้ = ชิดวัตถุมากจนสะท้อนไม่ทัน -> ถือเป็นกำแพงชัวร์เสมอ
TOF_SAMPLES = 7                 # จำนวนตัวอย่างต่อการอ่าน 1 ทิศ (หุ่นหยุดนิ่งตอนอ่านทุกครั้ง)
TOF_SAMPLE_INTERVAL = 0.05
TOF_OUTLIER_MM = 80             # ตัวอย่างที่เบี่ยงจาก median เกินนี้ถือว่าเป็น glitch ตัดทิ้ง
TOF_MIN_VALID_SAMPLES_FRAC = 0.5
WALL_OPEN_CONFIRM_STREAK = 3    # ต้องเจอ "โล่ง" ติดกันกี่ครั้งจึงจะลบกำแพงที่เคยมาร์กไว้ออก
                                 # (เพิ่มกำแพงได้ทันทีเมื่อเจอ 1 ครั้ง แต่ลบต้องยืนยันหนักแน่นกว่า
                                 # - กันกำแพงผีค้างถาวรจาก glitch ตอนต้น โดยไม่ทำให้แผนที่แกว่ง)
EMERGENCY_STOP_MM = 80          # ตอนเดินหน้า: ToF หน้าต่ำกว่านี้ = หยุดฉุกเฉินทันที (คนละเกณฑ์กับ
                                 # การจัดกึ่งกลาง เป็นแค่ตัวกันชนสุดท้ายระหว่างทาง)

# ---------- gimbal (สแกน ToF 4 ทิศ) ----------
GIMBAL_PITCH = 0
GIMBAL_YAW_SPEED = 80.0         # deg/s - ตั้งใจให้ช้ากว่าเดิม ไม่ต้องรีบ เน้นอ่านนิ่ง/แม่น
GIMBAL_SETTLE_S = 0.25          # รอเพิ่มหลัง wait_for_completed ก่อนเริ่มอ่าน
GIMBAL_YAW_FRONT = 0
GIMBAL_YAW_LEFT = 90            # +yaw = ซ้าย (อ้างอิงจาก examples/03_gimbal/01_move.py)
GIMBAL_YAW_RIGHT = -90
GIMBAL_YAW_BACK = 180
#: tuple: ลำดับสแกน (ทิศสัมพัทธ์ 0=หน้า 1=ขวา 2=หลัง 3=ซ้าย, มุม gimbal, ชื่อ)
SCAN_PLAN = (
    (0, GIMBAL_YAW_FRONT, "front"),
    (1, GIMBAL_YAW_RIGHT, "right"),
    (2, GIMBAL_YAW_BACK, "back"),
    (3, GIMBAL_YAW_LEFT, "left"),
)

# ---------- IR 45 องศา (e-stop ระหว่างเดินหน้าเท่านั้น ไม่เกี่ยวกับแผนที่/ตอนจอดเลย) ----------
IR_ESTOP_ENABLED = True

# ---------- Camera (log-only - "กำแพงสนามแข่งเป็นสีขาว") ----------
# ตกลงกับผู้ใช้แล้ว: กล้อง "ไม่มีผลต่อการตัดสินใจ/หยุดใด ๆ เลย" ToF ยังคุม
# กำแพง/e-stop 100% เหมือนเดิมทุกประการ กล้องแค่ถ่ายรูป + คำนวณ % พิกเซลขาวไว้
# ให้คนดูย้อนหลัง เผื่อวันหลังอยากเปิดเป็น veto เสริมเมื่อเห็นว่าแยกขาว/ไม่ขาว
# ได้ชัดเจนจริงในแสงสนามแข่งจริง (ไม่ใช่แค่แสงตอนซ้อม)
CAMERA_LOG_ENABLED = True       # ปิดง่าย ๆ ตรงนี้ถ้ากล้องมีปัญหาแล้วอยากตัดทิ้งไปก่อน
CAMERA_SAVE_PHOTOS = True       # เซฟไฟล์ .jpg ทุกครั้งที่ log ไหม (ปิดได้ถ้าดิสก์เต็ม)
CAMERA_WHITE_SAT_MAX = 60       # HSV Saturation ต่ำกว่านี้ + Value สูงกว่า VAL_MIN = "ขาว"
CAMERA_WHITE_VAL_MIN = 180
CAMERA_ROI_FRAC = 0.5           # ครอปเฉพาะสี่เหลี่ยมกึ่งกลางเฟรม (เศษส่วนของทั้งภาพ)
                                 # กันขอบเฟรม/แชสซีตัวเองก่อกวนค่า % ขาว

# ---------- ความเร็ว ----------
# เดินหน้าเข้าช่องใหม่: drive_speed คงที่ค่าเดียว ไม่มี P-control ปรับทิศ/ตำแหน่งใด ๆ
# ระหว่างทางทั้งสิ้น (ตกลงกับผู้ใช้แล้ว - ห้ามมีข้อยกเว้น) ปรับได้แค่ตอนจอดนิ่งเท่านั้น
HOP_SPEED = 0.18                 # m/s เดินหน้าเข้าช่องใหม่
BACKUP_SPEED = 0.12              # m/s ถอยกลับเข้าช่องเดิมหลังเดินไม่ผ่าน (คงที่เช่นกัน)

# ---------- chassis.move() : ใช้กับการเลี้ยวและการจัดกึ่งกลางตอนจอดเท่านั้น ----------
MOVE_XY_SPEED = 0.5              # m/s - ค่าต่ำสุดที่ chassis.move() ยอมรับจริง (checker บังคับ [0.5, 2.0])
MOVE_Z_SPEED = 45.0              # deg/s ตอนหมุน
MOVE_TIMEOUT_MARGIN_S = 3.0      # กัน wait_for_completed() ค้างตลอดกาล

# ---------- การจัดกึ่งกลางช่อง (chassis.move() ทีละก้อน อ่าน-ตัดสินใจ-ขยับ-อ่านใหม่) ----------
ALIGN_TOLERANCE_MM = 15          # front-back (ToF)
LATERAL_TOLERANCE_ADC = 15       # ซ้าย-ขวา (Sharp ADC ดิบ ไม่แปลงเป็น mm)
LATERAL_MAX_ITERS = 6
MAX_CREEP_STEP_MM = 40           # ก้าวเดียวตอนจัดกึ่งกลางห้ามเกินนี้
MIN_SAFE_MM = 60                 # ห้ามสั่งก้าวใดที่ทำให้ระยะเหลือน้อยกว่านี้เด็ดขาด
MAX_CREEP_TOTAL_MM = 200         # เพดานระยะสะสม front-back ต่อการจัดกึ่งกลาง 1 ครั้ง
MAX_TOTAL_LATERAL_MM = 200       # เพดานระยะสะสมซ้าย-ขวา (กันอ้างอิง Sharp ข้างเดียวคลาดเคลื่อน
                                 # แล้วไล่ขยับไม่มีที่สิ้นสุดจนหลุดเส้นทาง)

# ---------- การนับช่อง ----------
CELL_COMPLETE_RATIO = 0.85      # เดินได้ >= 85% ของช่อง = ถือว่าเข้าช่องใหม่แล้ว
MOVE_TIMEOUT_RATIO = 2.0        # timeout = (CELL_SIZE_M / HOP_SPEED) * ค่านี้

# ---------- ระบบ ----------
CONTROL_DT = 0.04               # คาบของลูปตรวจระหว่างเดินหน้า (โพลว่าจะ e-stop ไหม)
SETTLE_S = 0.25                 # เวลารอให้หุ่นนิ่งก่อนอ่านค่าลงแผนที่
MAX_STEPS = 350                 # สำรวจครบ 20 ช่องพร้อม backtrack ใช้ก้าวมากกว่าเดินไปเป้าหมายเดียว
BACK_RETRY = 4                  # เดิน backtrack/reroute ไม่ผ่านกี่ครั้งติดกัน ก่อนปิดทางนั้นไว้ก่อน
DDS_FREQ = 20                   # Hz รองรับเฉพาะ 1, 5, 10, 20, 50
ADAPTER_WAIT_S = 2.0            # รอ sub_adapter นานแค่ไหนก่อนสลับไป fallback
SENSOR_STALE_S = 0.5            # ไม่มีข้อมูลใหม่เกินนี้ = ถือว่าค่าใช้ไม่ได้
DRIVE_WATCHDOG_S = 0.5          # หุ่นหยุดเองถ้าเราไม่ส่งคำสั่งใหม่ภายในเวลานี้ (drive_speed เท่านั้น)
CONN_TYPE = "ap"                # ap / sta / rndis
LOG_PREFIX = "slam8"            # ชื่อไฟล์ผลลัพธ์ (map/steps/trajectory/summary) ขึ้นต้นด้วยคำนี้
#: str: โฟลเดอร์ที่ไฟล์นี้อยู่ - ไฟล์ผลลัพธ์ทุกชนิด (log/CSV/map/state/รูปกล้อง)
#: บันทึกไว้ที่นี่เสมอ ไม่ว่าจะสั่งรัน ``python SLAM.py`` จาก cwd ไหนก็ตาม
#: (ก่อนหน้านี้ path เป็นแบบสัมพัทธ์ล้วน ๆ ถ้ารันจากโฟลเดอร์อื่นไฟล์ผลลัพธ์จะ
#: ไปกระจัดกระจายอยู่ที่อื่น หาไม่เจอ)
LOG_DIR = os.path.dirname(os.path.abspath(__file__))


def _log_path(name):
    """str: ต่อชื่อไฟล์เข้ากับ ``LOG_DIR`` เสมอ"""
    return os.path.join(LOG_DIR, name)


# =====================================================================
# ค่าคงที่และฟังก์ชันช่วยทั่วไป
# =====================================================================
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_NAMES = ("N", "E", "S", "W")
DIR_ARROWS = ("^", ">", "v", "<")
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
INF = 9999


def wrap_deg(angle):
    """บีบมุมให้อยู่ในช่วง [-180, 180) องศา"""
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    """จำกัดค่าให้อยู่ระหว่าง low ถึง high"""
    return max(low, min(high, value))


def wall_cutoff_mm(base_mm):
    """int: เกณฑ์ ToF ที่ใช้ตัดสินว่ามีกำแพงอยู่ในทิศนั้น หน่วย mm

    เรขาคณิต: เมื่อหุ่นจอดกลางช่อง กำแพงที่ขอบช่องนี้อ่านได้ ``base_mm``
    ส่วนกำแพงที่ขอบช่องถัดไปอ่านได้ ``base_mm + CELL_SIZE`` เส้นแบ่งจึงวางไว้
    ตรงกลางระหว่างสองค่านั้นพอดี เพราะเป็นจุดที่ทนต่อการจอดคลาดเคลื่อนได้มากที่สุด
    ใช้ร่วมกันทั้งหน้า-หลัง (``base_mm=FRONT_STOP_MM``) และซ้าย-ขวา
    (``base_mm=SIDE_STOP_MM``) เพราะเป็นเรขาคณิตเดียวกัน
    """
    if FRONT_WALL_MM_OVERRIDE is not None:
        return FRONT_WALL_MM_OVERRIDE
    return base_mm + int(CELL_SIZE_M * 1000.0 / 2.0)


def tof_is_wall(mm, base_mm):
    """bool: ตีความค่า ToF (ที่ผ่าน ``SensorHub`` แปลง blind-zone มาแล้ว) ว่ามีกำแพงไหม

    ``mm`` เป็น None แปลว่าไม่มีอะไรอยู่ในระยะที่วัดได้ = โล่ง ไม่ใช่กำแพง
    """
    if mm is None:
        return False
    return mm < wall_cutoff_mm(base_mm)


def sharp_closeness(adc, closer_is_higher):
    """int: แปลงค่า ADC ดิบให้เป็นสเกล "ยิ่งมากยิ่งใกล้กำแพง" เสมอ ไม่ว่าเซนเซอร์

    ตัวนั้นจะตอบสนองไปทางไหนจริง ๆ (อนุมานจาก ``closer_is_higher`` ที่วัดมาจาก
    ``--calib``) ทำให้สูตรคำนวณ error ทางฝั่งซ้ายและขวาเขียนเป็นสูตรเดียวกันได้
    โดยไม่ต้องแยกเคสตามขั้วของเซนเซอร์อีก
    """
    return adc if closer_is_higher else -adc


def ir_triggered(io_value):
    """True ถ้า IR 45 องศาตัวนั้นกำลังเจอสิ่งกีดขวาง"""
    if io_value is None or IR_TRIGGERED_VALUE is None:
        return False
    return int(io_value) == int(IR_TRIGGERED_VALUE)


def require_calibration():
    """ตรวจว่าค่าจาก --calib ครบแล้ว ถ้าไม่ครบให้หยุดพร้อมบอกว่าขาดตัวไหน"""
    missing = [name for name in (
        "FRONT_STOP_MM", "SIDE_STOP_MM",
        "SHARP_LEFT_REF", "SHARP_RIGHT_REF",
        "SHARP_LEFT_CLOSER_IS_HIGHER", "SHARP_RIGHT_CLOSER_IS_HIGHER",
        "IR_TRIGGERED_VALUE",
    ) if globals()[name] is None]
    if not missing:
        return
    print("\n[STOP] ยังไม่ได้คาลิเบรตเซนเซอร์ ค่าที่ยังขาด:")
    for name in missing:
        print("         - {0}".format(name))
    print("\n  รัน `python SLAM.py --calib` ก่อน (ใช้เวลาไม่ถึง 2 นาที)")
    print("  แล้วคัดลอกบล็อกค่าที่มันพิมพ์ออกมา ไปวางทับใน CONFIG ด้านบนของไฟล์นี้")
    print("\n  เหตุผลที่ไม่ให้รันด้วยค่าเดา: threshold ที่ผิดจะทำให้หุ่นวิ่งดูปกติ")
    print("  ทุกอย่างแต่สร้างแผนที่ผิด ซึ่งหน้าสนามจะแยกไม่ออกเลยว่าปัญหาอยู่ที่")
    print("  threshold, odometry หรือ flood fill\n")
    sys.exit(1)


# =====================================================================
# ชั้นอ่านเซนเซอร์
# =====================================================================
class SensorSnapshot(object):
    """ค่าเซนเซอร์ทั้งชุด ณ เวลาเดียวกัน แช่แข็งไว้แล้ว

    ทั้งรอบการตัดสินใจจะอ้างอิงจากออบเจกต์ตัวเดียวนี้ ไม่ใช่ไปอ่านตัวแปร global
    ทีละตัวซึ่งอาจถูก callback เขียนทับกลางคันจนได้ค่าคนละช่วงเวลามาปนกัน
    """

    __slots__ = ("t", "tof_mm", "adc_left", "adc_right", "ir_left", "ir_right",
                 "yaw", "pos_x", "pos_y", "impact", "slip", "roll_over",
                 "fresh", "stale_reason")

    def __init__(self, t, tof_mm, adc_left, adc_right, ir_left, ir_right,
                 yaw, pos_x, pos_y, impact, slip, roll_over,
                 fresh, stale_reason):
        #: float: เวลาที่ถ่ายภาพนิ่งชุดนี้
        self.t = t
        #: int or None: ระยะ ToF ปัจจุบัน (ทิศที่ gimbal ชี้อยู่ตอนนี้) mm
        #: ผ่านการแปลง blind-zone แล้ว (ดู ``SensorHub._interpret_tof``)
        #: None = ไม่มีอะไรอยู่ในระยะที่วัดได้ (โล่ง) ไม่ใช่ "สตรีมพัง"
        self.tof_mm = tof_mm
        #: int or None: ค่า ADC ดิบของ Sharp ซ้าย (ใช้จัดกึ่งกลางตอนจอดเท่านั้น)
        self.adc_left = adc_left
        #: int or None: ค่า ADC ดิบของ Sharp ขวา (ใช้จัดกึ่งกลางตอนจอดเท่านั้น)
        self.adc_right = adc_right
        #: int or None: ค่า IO ดิบของ IR 45 องศาซ้าย (e-stop ตอนเดินหน้าเท่านั้น)
        self.ir_left = ir_left
        #: int or None: ค่า IO ดิบของ IR 45 องศาขวา (e-stop ตอนเดินหน้าเท่านั้น)
        self.ir_right = ir_right
        #: float: yaw ดิบจาก IMU หน่วยองศา ช่วง [-180, 180] (ใช้ log เท่านั้น)
        self.yaw = yaw
        #: float: ตำแหน่งล้อแกน x หน่วยเมตร
        self.pos_x = pos_x
        #: float: ตำแหน่งล้อแกน y หน่วยเมตร
        self.pos_y = pos_y
        #: bool: เซฟตี้เน็ตอิสระจาก ToF - ชนจริงจากฮาร์ดแวร์ (accelerometer)
        self.impact = impact
        #: bool: ล้อลื่น/ติดขัด จาก chassis.sub_status()
        self.slip = slip
        #: bool: หุ่นพลิกคว่ำ/เอียงมาก
        self.roll_over = roll_over
        #: bool: False เมื่อมีสตรีมสำคัญตัวใดตัวหนึ่งขาดการอัปเดต
        self.fresh = fresh
        #: str: ชื่อสตรีมที่ขาดการอัปเดต ว่างเปล่าถ้าปกติ
        self.stale_reason = stale_reason

    def is_wall(self, base_mm):
        """bool: ตีความ ``tof_mm`` ปัจจุบัน (ไม่ว่า gimbal ชี้ทิศไหนอยู่) ว่ามีกำแพงไหม"""
        return tof_is_wall(self.tof_mm, base_mm)

    def emergency_trip(self):
        """bool: ควร e-stop ทันทีไหม (ใช้เฉพาะระหว่างเดินหน้าเข้าช่องใหม่)

        รวม 3 แหล่ง: IR ทแยง 2 ตัว, impact/roll_over จากฮาร์ดแวร์ (อิสระจาก ToF
        โดยสิ้นเชิง), และ ToF หน้าที่ใกล้ผิดปกติ (กันชนสุดท้ายระหว่างทาง ไม่ใช่
        การปรับทิศ/ตำแหน่ง จึงไม่ขัดกับกฎ "ห้ามปรับระหว่างเดินหน้า")
        """
        if not self.fresh:
            return True
        if self.impact or self.roll_over:
            return True
        if IR_ESTOP_ENABLED and (ir_triggered(self.ir_left) or ir_triggered(self.ir_right)):
            return True
        if self.tof_mm is not None and self.tof_mm < EMERGENCY_STOP_MM:
            return True
        return False


class SensorHub(object):
    """เจ้าของ subscription ทั้งหมด + ตัวคุม gimbal และเป็นทางเดียวที่โค้ดส่วนอื่น
    อ่านเซนเซอร์

    callback ของ DDS ทำงานคนละเธรดกับ control loop ทุกฟิลด์จึงถูกเขียนและอ่าน
    ภายใต้ล็อกเดียวกัน

    Args:
        ep_robot: ออบเจกต์ robot.Robot ที่ initialize แล้ว
    """

    def __init__(self, ep_robot):
        self._chassis = ep_robot.chassis
        self._sensor = ep_robot.sensor
        self._adaptor = ep_robot.sensor_adaptor
        self._gimbal = ep_robot.gimbal
        self._camera = ep_robot.camera
        #: bool: กล้องเปิดสตรีมสำเร็จไหม (log-only - ถ้าเปิดไม่ได้ก็แค่ข้าม
        #: ไม่ต้อง fail ทั้งภารกิจ เพราะกล้องไม่มีผลต่อการตัดสินใจใด ๆ)
        self._camera_ready = False

        self._lock = threading.Lock()
        self._tof = [0, 0, 0, 0]
        self._tof_t = 0.0
        self._io = [0] * 12
        self._ad = [0] * 12
        self._adapter_t = 0.0
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

        #: bool: True = ใช้ sub_adapter (push), False = fallback ไป get_adc (blocking)
        self.use_adapter = True
        self._poll_cache = (None, None)
        self._poll_t = 0.0
        self._started = False
        #: int: +1 ถ้า moveto(yaw=+X) ทำให้มุมที่วัดจริงเพิ่มขึ้น (ซ้าย), -1 ถ้า
        #: กลับด้าน หาได้จาก calibrate_gimbal_yaw_sign() เท่านั้น (moveto ใช้
        #: coordinate mode คนละโหมดกับ move() เครื่องหมาย +/- ไม่รับประกันตรงกัน)
        self.gimbal_yaw_sign = 1

        # ---------- sensor log ดิบต่อเนื่อง (ทุก snapshot() ที่ถูกเรียกจริง) ----------
        # เขียนที่จุดเดียวคือ ``snapshot()`` เพราะโค้ดทั้งไฟล์อ่านเซนเซอร์ผ่าน
        # เมธอดนี้ทางเดียวเท่านั้น (ดู docstring ``SensorSnapshot``) - เพิ่ม log
        # ตรงนี้จุดเดียวจึงครอบคลุมทุกจังหวะ (สแกน/จัดกึ่งกลาง/เดินหน้า/ถอย/
        # คาลิเบรต) โดยไม่ต้องแก้โค้ดจุดอ่านเซนเซอร์ทุกจุดทั่วไฟล์
        self._slog_file = None
        self._slog_writer = None
        #: int: หมายเลขก้าว (step) ปัจจุบัน ผู้เรียกระดับบนตั้งค่าไว้เฉย ๆ ให้
        #: แถว log อ้างอิงได้ว่าอยู่ก้าวไหน (อ่าน/เขียนจากเธรดควบคุมเธรดเดียว
        #: ไม่ต้องล็อก)
        self.step = -1
        #: str: ชื่อจังหวะปัจจุบัน (เช่น "scan_front", "hop", "center_lateral")
        #: ใช้กำกับแถว log ให้รู้ว่าค่าที่อ่านมาเกิดขึ้นตอนไหนของก้าวนั้น
        self.phase = "idle"

    # ---------- callback (ทำงานบนเธรดของ DDS) ----------
    def _on_tof(self, info):
        now = time.time()
        with self._lock:
            self._tof = list(info)
            self._tof_t = now

    def _on_adapter(self, info):
        io_values, ad_values = info
        now = time.time()
        with self._lock:
            self._io = list(io_values)
            self._ad = list(ad_values)
            self._adapter_t = now

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

    # ---------- วงจรชีวิต ----------
    def _check_wiring(self):
        """ตรวจว่า hub id ที่คอนฟิกไว้อยู่ในช่วงที่ sub_adapter รองรับ

        AdapterSubject.decode (sensor.py:71) ถอดรหัสมาแค่ 6 บอร์ด (index 0-11)
        ส่วน get_adc รับ id ได้ถึง 8 ถ้าคอนฟิกเกิน 6 จึงต้องบังคับใช้ fallback
        ไม่งั้นจะได้ IndexError กลางทางตอนวิ่ง
        """
        ports = (("SHARP_LEFT", SHARP_LEFT), ("SHARP_RIGHT", SHARP_RIGHT),
                 ("IR_LEFT_45", IR_LEFT_45), ("IR_RIGHT_45", IR_RIGHT_45))
        for name, (hub_id, port) in ports:
            if not 1 <= hub_id <= 8 or port not in (1, 2):
                raise ValueError(
                    "{0} = (hub {1}, port {2}) ไม่ถูกต้อง hub ต้องอยู่ 1-8 "
                    "และ port ต้องเป็น 1 หรือ 2".format(name, hub_id, port))
        too_high = [name for name, (hub_id, _) in ports if hub_id > 6]
        if too_high:
            self.use_adapter = False
            print("[WARN] {0} ใช้ hub เกิน 6 ซึ่ง sub_adapter ถอดรหัสไม่ถึง "
                  "จึงต้องใช้ get_adc แบบ blocking ทั้งหมด (ช้ากว่ามาก)"
                  .format(", ".join(too_high)))

    def start(self):
        """เปิด subscription ทั้งหมด แล้วรอจนมีข้อมูลชุดแรกเข้ามา"""
        # ตั้งธงก่อนเริ่ม subscribe เพื่อให้ stop() ยังเก็บกวาดตัวที่สมัครไปแล้วได้
        # ถ้าเกิด exception กลางคัน
        self._started = True
        self._check_wiring()
        self._sensor.sub_distance(freq=DDS_FREQ, callback=self._on_tof)
        self._adaptor.sub_adapter(freq=DDS_FREQ, callback=self._on_adapter)
        self._chassis.sub_attitude(freq=DDS_FREQ, callback=self._on_attitude)
        self._chassis.sub_position(freq=DDS_FREQ, callback=self._on_position)
        self._chassis.sub_status(freq=DDS_FREQ, callback=self._on_status)

        if CAMERA_LOG_ENABLED and cv2 is not None:
            try:
                self._camera.start_video_stream(display=False)
                self._camera_ready = True
                print("[CAMERA] เปิดสตรีมกล้องแล้ว (log-only ไม่มีผลต่อการตัดสินใจ)")
            except Exception as exc:                        # noqa: BLE001
                print("[WARN] เปิดกล้องไม่สำเร็จ ({0}) - จะข้าม camera log ไปเลย"
                      .format(exc))
        elif CAMERA_LOG_ENABLED and cv2 is None:
            print("[WARN] ไม่มี cv2 ติดตั้ง - ข้าม camera log ไปเลย (pip install opencv-python)")

        deadline = time.time() + ADAPTER_WAIT_S
        while time.time() < deadline:
            with self._lock:
                got_all = (self._tof_t > 0 and self._att_t > 0
                           and self._pos_t > 0 and self._status_t > 0)
                got_adapter = self._adapter_t > 0
            if got_all and (got_adapter or not self.use_adapter):
                print("[SENSOR] subscription พร้อมใช้งาน (ToF, attitude, position, "
                      "status{0})".format(", adapter" if self.use_adapter else ""))
                return
            time.sleep(0.05)

        with self._lock:
            got_adapter = self._adapter_t > 0 or not self.use_adapter
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
        if not got_adapter:
            self.use_adapter = False
            print("[WARN] sub_adapter ไม่ส่งข้อมูลใน {0} วินาที "
                  "สลับไปใช้ get_adc/get_io แบบ blocking แทน"
                  .format(ADAPTER_WAIT_S))
            print("[WARN] โหมดนี้ control loop จะช้าลงมาก ควรตรวจว่าเสียบ hub "
                  "ถูกพอร์ตหรือไม่")

    def stop(self):
        """ปิด subscription ทั้งหมด (เรียกซ้ำได้ ไม่ throw)"""
        if not self._started:
            return
        for name, fn in (("distance", self._sensor.unsub_distance),
                         ("adapter", self._adaptor.unsub_adapter),
                         ("attitude", self._chassis.unsub_attitude),
                         ("position", self._chassis.unsub_position),
                         ("status", self._chassis.unsub_status)):
            try:
                fn()
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] unsub {0} ล้มเหลว: {1}".format(name, exc))
        if self._camera_ready:
            try:
                self._camera.stop_video_stream()
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] ปิดสตรีมกล้องล้มเหลว: {0}".format(exc))
            self._camera_ready = False
        self._started = False
        self.close_sensor_log()

    # ---------- sensor log ดิบต่อเนื่อง ----------
    def attach_sensor_log(self, path):
        """เปิดไฟล์ CSV แล้วเริ่มบันทึกทุก ``snapshot()`` ที่ถูกเรียกจากนี้ไป
        (real-time, flush ทุกแถว) - เรียกครั้งเดียวหลัง ``start()``

        แถวหนึ่งคือค่าเซนเซอร์ดิบทั้งชุด ณ เวลาหนึ่ง พร้อม ``step``/``phase``
        ที่ผู้เรียกระดับบนตั้งไว้ (ดู ``set_phase`` / ``self.phase``) ทำให้
        ย้อนดูได้ว่าตอนเกิดปัญหา (e-stop, ล้อลื่น, ชน) ค่าดิบของ ToF/Sharp/IR/
        impact ตอนนั้นเป็นเท่าไหร่ ไม่ต้องเดาจาก log สรุปต่อช่องอย่างเดียว
        """
        self._slog_file = open(path, "w", newline="", encoding="utf-8")
        self._slog_writer = csv.writer(self._slog_file)
        self._slog_writer.writerow([
            "t", "step", "phase", "tof_mm", "adc_left", "adc_right",
            "ir_left", "ir_right", "impact", "slip", "roll_over",
            "pos_x", "pos_y", "yaw", "fresh", "stale_reason"])
        self._slog_file.flush()
        print("[SENSORLOG] บันทึกค่าเซนเซอร์ดิบทุกจังหวะไปที่ {0}".format(path))

    def close_sensor_log(self):
        """ปิดไฟล์ sensor log (เรียกซ้ำได้ ไม่ throw)"""
        if self._slog_file is not None:
            try:
                self._slog_file.close()
            except Exception:                           # noqa: BLE001
                pass
            self._slog_file = None
            self._slog_writer = None

    def set_phase(self, step, phase):
        """ตั้งบริบท (step, phase) ที่จะแปะกำกับ ``snapshot()`` ครั้งถัดไปในไฟล์
        log จนกว่าจะถูกเปลี่ยนอีกที (ผู้เรียกระดับบนอัปเดตก่อนเข้าแต่ละช่วง)
        """
        self.step = step
        self.phase = phase

    def _write_sensor_log(self, snap):
        if self._slog_writer is None:
            return
        self._slog_writer.writerow([
            round(snap.t, 4), self.step, self.phase, snap.tof_mm,
            snap.adc_left, snap.adc_right, snap.ir_left, snap.ir_right,
            int(snap.impact), int(snap.slip), int(snap.roll_over),
            round(snap.pos_x, 4), round(snap.pos_y, 4), round(snap.yaw, 2),
            int(snap.fresh), snap.stale_reason])
        self._slog_file.flush()

    # ---------- กล้อง (log-only) ----------
    def log_camera_frame(self, tag):
        """dict: ถ่ายภาพปัจจุบัน 1 เฟรม คำนวณ % พิกเซล "ขาว" ในกรอบกลางเฟรม
        แล้วเซฟไฟล์ (ถ้า ``CAMERA_SAVE_PHOTOS``) - **ไม่มีผลต่อการตัดสินใจใด ๆ
        เลย** เก็บไว้ให้คนดูย้อนหลังเท่านั้น (ตกลงกับผู้ใช้แล้ว - ดู
        ``CAMERA_LOG_ENABLED`` docstring ใน CONFIG)

        Args:
            tag (str): ชื่อสั้น ๆ บอกบริบท ใช้ตั้งชื่อไฟล์ (เช่น "calib_front_wall",
                "step012")

        Returns:
            dict: {"white_pct": float|None, "photo_path": str|None}
        """
        result = {"white_pct": None, "photo_path": None}
        if not self._camera_ready:
            return result
        try:
            img = self._camera.read_cv2_image(timeout=2, strategy="newest")
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] อ่านภาพจากกล้องไม่สำเร็จ ({0})".format(tag))
            print("       {0}".format(exc))
            return result
        if img is None:
            return result

        h, w = img.shape[:2]
        cx0, cx1 = int(w * (0.5 - CAMERA_ROI_FRAC / 2)), int(w * (0.5 + CAMERA_ROI_FRAC / 2))
        cy0, cy1 = int(h * (0.5 - CAMERA_ROI_FRAC / 2)), int(h * (0.5 + CAMERA_ROI_FRAC / 2))
        roi = img[cy0:cy1, cx0:cx1]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = (hsv[:, :, 1] <= CAMERA_WHITE_SAT_MAX) & (hsv[:, :, 2] >= CAMERA_WHITE_VAL_MIN)
        white_pct = 100.0 * float(mask.mean())
        result["white_pct"] = white_pct

        if CAMERA_SAVE_PHOTOS:
            photo_path = _log_path("{0}_cam_{1}_{2}.jpg".format(LOG_PREFIX, tag, _timestamp()))
            try:
                cv2.imwrite(photo_path, img)
                result["photo_path"] = photo_path
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] เซฟรูป {0} ไม่สำเร็จ: {1}".format(photo_path, exc))

        print("[CAMERA] {0}: ขาว {1:.1f}%{2}".format(
            tag, white_pct,
            " (เซฟ {0})".format(result["photo_path"]) if result["photo_path"] else ""))
        return result

    # ---------- การอ่าน ----------
    @staticmethod
    def _adapter_index(hub_id, port):
        """แปลง (hub_id, port) เป็น index ของ list ที่ sub_adapter คืนมา

        อ้างอิง AdapterSubject.decode ใน sensor.py:71 ที่ไล่ ``for i in range(0, 6)``
        แล้วเก็บ port 1 ที่ ``i*2`` และ port 2 ที่ ``i*2+1``
        """
        return (hub_id - 1) * 2 + (port - 1)

    def _poll_adaptor(self, now):
        """fallback: อ่านผ่าน get_adc/get_io ซึ่ง block ได้นานถึง 3 วินาทีต่อครั้ง"""
        if self._poll_cache[0] is not None and now - self._poll_t < 0.05:
            return self._poll_cache
        adc = {}
        io = {}
        for key, (hub, port) in (("l", SHARP_LEFT), ("r", SHARP_RIGHT)):
            try:
                adc[key] = self._adaptor.get_adc(id=hub, port=port)
            except Exception:                           # noqa: BLE001
                adc[key] = None
        for key, (hub, port) in (("l", IR_LEFT_45), ("r", IR_RIGHT_45)):
            try:
                io[key] = self._adaptor.get_io(id=hub, port=port)
            except Exception:                           # noqa: BLE001
                io[key] = None
        self._poll_cache = (adc, io)
        self._poll_t = now
        return self._poll_cache

    @staticmethod
    def _interpret_tof(tof_raw):
        """int or None: แปลงค่า ToF ดิบเป็นค่าที่ใช้ตัดสินใจได้ พร้อมกันบั๊ก blind-zone

        ``tof_raw >= TOF_MAX_VALID_MM`` = ไม่มีอะไรอยู่ในระยะ -> None (โล่ง)
        ``0 < tof_raw < TOF_BLIND_ZONE_MM`` = ชิดวัตถุมากจนสะท้อนไม่ทัน -> ยังคง
        เป็นระยะจริงที่วัดได้ (ไม่ใช่ None) เพราะมันคือ "กำแพงชัวร์ อยู่ใกล้มาก"
        ต่างจากเวอร์ชันแรกที่โยน ``tof_raw == 0`` ทิ้งเป็น None (=โล่ง) ซึ่งผิด
        (ดู docstring หัวไฟล์ ข้อ 7 - บั๊กที่เจอจาก IR_6/sensors.py)
        ``tof_raw == 0`` เป๊ะมักเป็นค่าตอนสตรีมยังไม่อัปเดตจริง ๆ (ไม่ใช่ระยะจริง)
        จึงปัดขึ้นเป็น ``TOF_BLIND_ZONE_MM`` แทนการส่งค่า 0 มม. ตรง ๆ
        """
        if tof_raw <= 0:
            return None if tof_raw == 0 else TOF_BLIND_ZONE_MM
        if tof_raw >= TOF_MAX_VALID_MM:
            return None
        if tof_raw < TOF_BLIND_ZONE_MM:
            return TOF_BLIND_ZONE_MM
        return tof_raw

    def snapshot(self):
        """SensorSnapshot: อ่านเซนเซอร์ทุกตัวเป็นชุดเดียว"""
        now = time.time()
        with self._lock:
            tof_raw = self._tof[TOF_INDEX] if TOF_INDEX < len(self._tof) else 0
            tof_age = now - self._tof_t if self._tof_t else 1e9
            att_age = now - self._att_t if self._att_t else 1e9
            pos_age = now - self._pos_t if self._pos_t else 1e9
            status_age = now - self._status_t if self._status_t else 1e9
            yaw = self._yaw
            pos_x, pos_y = self._pos_x, self._pos_y
            impact, slip, roll_over = self._impact, self._slip, self._roll_over
            if self.use_adapter:
                ad = list(self._ad)
                io = list(self._io)
                adapter_age = now - self._adapter_t if self._adapter_t else 1e9
            else:
                ad = io = None
                adapter_age = 0.0

        if ad is not None:
            adc_left = ad[self._adapter_index(*SHARP_LEFT)]
            adc_right = ad[self._adapter_index(*SHARP_RIGHT)]
            ir_left = io[self._adapter_index(*IR_LEFT_45)]
            ir_right = io[self._adapter_index(*IR_RIGHT_45)]
        else:
            adc, io_map = self._poll_adaptor(now)
            adc_left, adc_right = adc["l"], adc["r"]
            ir_left, ir_right = io_map["l"], io_map["r"]

        tof_mm = self._interpret_tof(tof_raw)

        stale = ""
        if tof_age > SENSOR_STALE_S:
            stale = "tof"
        elif att_age > SENSOR_STALE_S:
            stale = "attitude"
        elif pos_age > SENSOR_STALE_S:
            stale = "position"
        elif status_age > SENSOR_STALE_S:
            stale = "status"
        elif adapter_age > SENSOR_STALE_S:
            stale = "adapter"

        snap = SensorSnapshot(
            t=now, tof_mm=tof_mm, adc_left=adc_left, adc_right=adc_right,
            ir_left=ir_left, ir_right=ir_right, yaw=yaw,
            pos_x=pos_x, pos_y=pos_y, impact=impact, slip=slip,
            roll_over=roll_over, fresh=(stale == ""), stale_reason=stale)
        self._write_sensor_log(snap)
        return snap

    def emergency_flags(self):
        """tuple: (impact, slip, roll_over) จาก ``chassis.sub_status()`` ล่าสุด"""
        with self._lock:
            return self._impact, self._slip, self._roll_over

    # ---------- gimbal ----------
    def point_gimbal(self, yaw_deg):
        """หมุน gimbal ไปมุม yaw_deg (สัมพัทธ์กับแชสซี) แล้วรอให้นิ่ง ไม่อ่านค่า

        คูณด้วย ``gimbal_yaw_sign`` ก่อนส่งจริงเสมอ ที่อื่นในไฟล์นี้จึงเรียกด้วย
        ค่า "ตามความหมาย" ได้เลย (0=หน้า, +90=ซ้าย, -90=ขวา, 180=หลัง)
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
        """หมุน gimbal ทดสอบด้วย ``moveto()`` จริง แล้ววัดจาก ``sub_angle()`` ว่า
        +yaw ที่สั่งไปทำให้มุมเพิ่มไปทางไหนจริง ๆ (ต้องวัดเอง เพราะ moveto()
        กับ move() ของแชสซีใช้ coordinate mode คนละโหมดกัน ไม่รับประกันว่า
        เครื่องหมาย +/- จะตรงกัน)
        """
        readings = {}

        def _on_angle(info):
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
        print("[GIMBAL] สั่ง moveto(yaw=+{0}) จริง -> มุมเปลี่ยน {1:+.2f} "
              "-> GIMBAL_YAW_SIGN = {2:+d}".format(test_yaw, delta, self.gimbal_yaw_sign))

    def read_tof(self, yaw_deg, phase="tof_read"):
        """หมุน gimbal ไปทิศที่ต้องการ แล้ววัด ToF จาก ``TOF_SAMPLES`` ตัวอย่าง
        พร้อมกรอง outlier ก่อนสรุปผล

        Args:
            phase (str): ชื่อจังหวะไว้กำกับ sensor log ของตัวอย่างทั้งหมดที่
                อ่านรอบนี้ (เช่น "scan_front") ไม่กระทบตรรกะการอ่าน/กรองเลย

        Returns:
            int or None: ระยะ mm ที่กรองแล้ว (ผ่าน blind-zone fix แล้ว) หรือ
            None ถ้าอ่านไม่ได้/เชื่อถือไม่ได้
        """
        self.phase = phase
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
        """int or None: กรอง outlier ออกก่อนสรุปเป็นค่าเดียว (median)"""
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

    def read_walls_scan(self, heading):
        """หมุน gimbal ส่อง ToF ทั้ง 4 ทิศสัมบูรณ์ (N/E/S/W) รอบช่องปัจจุบัน แล้ว
        ชี้หน้ากลับก่อนคืนค่าเสมอ (ต้องชี้หน้าไว้ก่อนเดินหน้าเข้าช่องใหม่เสมอ)

        Returns:
            dict: {ทิศสัมบูรณ์ 0-3: {"wall": bool, "mm": int|None, "label": str}}
        """
        readings = {}
        for rel, gyaw, label in SCAN_PLAN:
            mm = self.read_tof(gyaw, phase="scan_" + label)
            base_mm = FRONT_STOP_MM if rel in (0, 2) else SIDE_STOP_MM
            wall = tof_is_wall(mm, base_mm)
            absolute_dir = (heading + rel) % 4
            readings[absolute_dir] = {"wall": wall, "mm": mm, "label": label}
        self.point_gimbal(GIMBAL_YAW_FRONT)
        return readings


# =====================================================================
# แผนที่และ Flood Fill
# =====================================================================
class Maze(object):
    """แผนที่กำแพงของสนาม สร้างจากการสำรวจ (ไม่รู้แผนที่มาก่อน)

    กำแพงเก็บเป็น bitmask ต่อช่อง โดยบิตที่ i หมายถึงกำแพงทางทิศ i
    (0=N 1=E 2=S 3=W) และเก็บ ``known`` แยกไว้อีกชุดเพื่อบอกว่าด้านนั้น
    "เคยเห็นมาแล้วจริง" หรือ "ยังไม่เคยไปดู" ส่วน ``visited`` เก็บว่าช่องไหน
    หุ่นเคยไปยืนจริงแล้วบ้าง (คนละเรื่องกับ known ที่เป็นระดับ "ด้าน")

    Args:
        width (int): จำนวนช่องแกน x
        height (int): จำนวนช่องแกน y
        border_openings (iterable): เซ็ตของ ``(x, y, direction)`` ที่เป็นขอบสนาม
            แต่ไม่ให้ล็อกเป็นกำแพงล่วงหน้า (สนามนี้มีช่องเปิดจริงตรงขอบ) ปล่อย
            ให้เซนเซอร์ตอนไปถึงช่องนั้นเป็นคนตัดสินแทน เหมือนด้านอื่น ๆ ทุกประการ
    """

    def __init__(self, width, height, border_openings=None):
        self.width = width
        self.height = height
        self.walls = [[0] * height for _ in range(width)]
        self.known = [[0] * height for _ in range(width)]
        self.visited = set()
        self.border_openings = set(border_openings or ())
        #: dict: {frozenset({cell_a, cell_b}): จำนวนครั้งติดที่เจอ "โล่ง" ล่าสุด}
        #: ใช้กับ edge ที่เคยมาร์กเป็นกำแพงแล้วเท่านั้น (ดู ``set_wall``)
        self._open_streak = {}
        #: list: log ของ edge ที่เคยมาร์กว่าเป็นกำแพงแล้วถูกลบออกภายหลัง
        #: (x, y, direction, streak_ที่ทำให้ลบ) - ไว้ export เป็น "conflicts"
        self.evicted_walls = []
        self._add_borders()

    def _add_borders(self):
        """ใส่กำแพงขอบสนามรอบนอก ซึ่งปกติรู้แน่นอนอยู่แล้วโดยไม่ต้องไปวัด

        ยกเว้นด้านที่อยู่ใน ``border_openings`` (สนามนี้มีช่องเปิดจริงตรงขอบ)
        จะข้ามไปเฉย ๆ ไม่ล็อกเป็นกำแพง ปล่อยให้เซนเซอร์ตอนไปถึงช่องนั้นวัดเองสด ๆ
        """
        for x in range(self.width):
            if (x, 0, SOUTH) not in self.border_openings:
                self.set_wall(x, 0, SOUTH, True)
            if (x, self.height - 1, NORTH) not in self.border_openings:
                self.set_wall(x, self.height - 1, NORTH, True)
        for y in range(self.height):
            if (0, y, WEST) not in self.border_openings:
                self.set_wall(0, y, WEST, True)
            if (self.width - 1, y, EAST) not in self.border_openings:
                self.set_wall(self.width - 1, y, EAST, True)

    def in_bounds(self, x, y):
        """bool: พิกัดนี้อยู่ในสนามหรือไม่"""
        return 0 <= x < self.width and 0 <= y < self.height

    def has_wall(self, x, y, direction):
        """bool: ช่อง (x, y) มีกำแพงทางทิศ direction หรือไม่"""
        return bool(self.walls[x][y] & (1 << direction))

    def is_known(self, x, y, direction):
        """bool: เคยตรวจด้านนี้ของช่องนี้แล้วหรือยัง"""
        return bool(self.known[x][y] & (1 << direction))

    def set_wall(self, x, y, direction, present):
        """บันทึกผลการตรวจกำแพงหนึ่งด้าน พร้อมอัปเดตช่องข้างเคียงให้สอดคล้องกัน

        นโยบายไม่สมมาตร (asymmetric wall evidence - อ้างอิงแนวทางจาก
        ``nattannsra18/robomaster-autonomous-maze-navigation``): เจอกำแพง 1
        ครั้งมาร์กทันที (ปลอดภัยไว้ก่อน) แต่ edge ที่เคยมาร์กว่าเป็นกำแพงแล้ว
        จะถูก "ลบ" ออกได้ก็ต่อเมื่อเจอ "โล่ง" ยืนยันติดกัน
        ``WALL_OPEN_CONFIRM_STREAK`` ครั้งเท่านั้น (ไม่ใช่ครั้งเดียว) - กัน
        กำแพงผีค้างถาวรจาก glitch ตอนต้น (กระทบ Map Accuracy โดยตรง) โดยไม่ทำให้
        แผนที่แกว่งจาก noise ครั้งเดียว เพราะ planner สแกนซ้ำทุก edge ที่ผ่าน
        ทุกครั้งอยู่แล้ว โอกาสได้ 3 ครั้งติดจากการมาเยือนซ้ำจึงไม่ใช่เรื่องยาก
        ขอบสนาม (ไม่มี ``nx, ny`` ในสนาม) ไม่มีสิทธิ์ถูกลบ เพราะเป็น ground
        truth ที่ seed ไว้ตั้งแต่ ``_add_borders`` อยู่แล้ว (ยกเว้นช่องที่อยู่ใน
        ``border_openings`` ซึ่งไม่ได้ถูก seed เป็นกำแพงตั้งแต่ต้น จึงไม่มีอะไร
        ให้ลบอยู่แล้วเช่นกัน)
        """
        if not self.in_bounds(x, y):
            return
        nx, ny = x + DX[direction], y + DY[direction]
        opposite = (direction + 2) % 4
        edge_key = frozenset(((x, y), (nx, ny))) if self.in_bounds(nx, ny) else None
        currently_wall = bool(self.walls[x][y] & (1 << direction))

        if present:
            self.walls[x][y] |= (1 << direction)
            self.known[x][y] |= (1 << direction)
            if self.in_bounds(nx, ny):
                self.walls[nx][ny] |= (1 << opposite)
                self.known[nx][ny] |= (1 << opposite)
            if edge_key is not None:
                self._open_streak.pop(edge_key, None)
            return

        # present == False: เจอ "โล่ง" ในการอ่านครั้งนี้
        self.known[x][y] |= (1 << direction)
        if self.in_bounds(nx, ny):
            self.known[nx][ny] |= (1 << opposite)

        if not currently_wall:
            return  # ยังไม่เคยมาร์กเป็นกำแพง ไม่มีอะไรให้ลบ
        if edge_key is None:
            return  # ขอบสนามจริง ไม่มีสิทธิ์ถูกลบ

        streak = self._open_streak.get(edge_key, 0) + 1
        if streak >= WALL_OPEN_CONFIRM_STREAK:
            self.walls[x][y] &= ~(1 << direction)
            self.walls[nx][ny] &= ~(1 << opposite)
            self._open_streak.pop(edge_key, None)
            self.evicted_walls.append((x, y, direction, streak))
            print("[MAP-EVICT] ({0},{1}) ทิศ {2}: เคยมาร์กเป็นกำแพง แต่เจอโล่ง "
                  "ยืนยันติดกัน {3} ครั้งแล้ว - ลบออกจากแผนที่"
                  .format(x, y, DIR_NAMES[direction], streak))
        else:
            self._open_streak[edge_key] = streak

    def observe(self, x, y, heading, front, left, right):
        """บันทึกผลการตรวจกำแพงจากท่ายืนปัจจุบันลงแผนที่

        เซนเซอร์ให้ผลเป็น "หน้า/ซ้าย/ขวา" เทียบกับตัวหุ่น ต้องแปลงเป็นทิศสัมบูรณ์
        ของสนามก่อนตามทิศที่หุ่นหันอยู่
        """
        self.set_wall(x, y, heading, front)
        self.set_wall(x, y, (heading + 1) % 4, right)
        self.set_wall(x, y, (heading + 3) % 4, left)

    def mark_visited(self, x, y):
        """บันทึกว่าหุ่นเคยยืนที่ช่องนี้จริงแล้ว"""
        self.visited.add((x, y))

    def is_visited(self, x, y):
        """bool: หุ่นเคยยืนที่ช่องนี้แล้วหรือยัง"""
        return (x, y) in self.visited

    def coverage_pct(self):
        """float: % ของช่องทั้งหมดที่สำรวจ (ยืนไป) แล้ว"""
        total = self.width * self.height
        return 100.0 * len(self.visited) / total if total else 0.0

    def flood_known(self, goals):
        """BFS ระยะจากทุกช่องไปยัง ``goals`` โดยเดินผ่านได้เฉพาะด้านที่ "ยืนยัน
        แล้วจริง ๆ ว่าโล่ง" (``is_known`` และไม่มีกำแพง) เท่านั้น

        ต่างจาก Flood Fill แบบมองโลกในแง่ดี (ด้านที่ยังไม่เคยตรวจ = เดินผ่านได้)
        ตรงนี้ตั้งใจเข้มงวดกว่า เพราะใช้หาเส้นทาง backtrack/reroute ที่ต้องมั่นใจ
        ว่าเดินได้จริงทุกก้าว ไม่ใช่แค่เดาว่าน่าจะได้

        Returns:
            list: ตาราง distance[x][y] ค่า INF แปลว่าไปไม่ถึงด้วยความรู้ปัจจุบัน
        """
        dist = [[INF] * self.height for _ in range(self.width)]
        queue = deque()
        for gx, gy in goals:
            if self.in_bounds(gx, gy):
                dist[gx][gy] = 0
                queue.append((gx, gy))

        while queue:
            cx, cy = queue.popleft()
            next_dist = dist[cx][cy] + 1
            for direction in range(4):
                if self.has_wall(cx, cy, direction) or not self.is_known(cx, cy, direction):
                    continue
                nx, ny = cx + DX[direction], cy + DY[direction]
                if self.in_bounds(nx, ny) and dist[nx][ny] > next_dist:
                    dist[nx][ny] = next_dist
                    queue.append((nx, ny))
        return dist

    def shortest_known_path(self, start, goal):
        """หาเส้นทางจาก ``start`` ไป ``goal`` โดยเดินผ่านได้เฉพาะด้านที่ยืนยัน
        แล้วว่าโล่ง (เหมือน :meth:`flood_known`) ใช้ตอน reroute ไปช่องที่ยังไม่
        เคยสำรวจซึ่งอยู่ไกลจาก stack การเดินปัจจุบัน

        Returns:
            list or None: รายการช่อง [start, ..., goal] หรือ None ถ้าไปไม่ถึง
        """
        if start == goal:
            return [start]
        came_from = {start: None}
        queue = deque([start])
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) == goal:
                path = []
                node = goal
                while node is not None:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path
            for direction in range(4):
                if self.has_wall(cx, cy, direction) or not self.is_known(cx, cy, direction):
                    continue
                nxt = (cx + DX[direction], cy + DY[direction])
                if self.in_bounds(*nxt) and nxt not in came_from:
                    came_from[nxt] = (cx, cy)
                    queue.append(nxt)
        return None

    def edge_stats(self):
        """tuple: (จำนวนด้านที่ตรวจแล้ว, จำนวนด้านทั้งหมด)

        นับด้านละหนึ่งครั้ง โดยไล่เฉพาะทิศเหนือกับตะวันออกของทุกช่อง แล้วเติม
        ขอบใต้ของแถวล่างสุดและขอบตะวันตกของคอลัมน์ซ้ายสุด
        """
        total = 0
        seen = 0
        for x in range(self.width):
            for y in range(self.height):
                for direction in (NORTH, EAST):
                    total += 1
                    seen += 1 if self.is_known(x, y, direction) else 0
        for x in range(self.width):
            total += 1
            seen += 1 if self.is_known(x, 0, SOUTH) else 0
        for y in range(self.height):
            total += 1
            seen += 1 if self.is_known(0, y, WEST) else 0
        return seen, total

    # ---------- บันทึก/โหลดสถานะ (ให้เอาแผนที่ที่สำรวจแล้วมาใช้ต่อได้ในโหมด
    # goto/return-start โดยไม่ต้องสำรวจใหม่) ----------
    def to_state(self):
        """dict: แปลงแผนที่ทั้งหมดเป็นข้อมูลที่ json.dump ได้ตรง ๆ

        เก็บเฉพาะสิ่งที่จำเป็นต่อการวางแผนเส้นทางใหม่ (``walls``/``known``/
        ``visited``) ไม่เก็บ ``_open_streak`` (แค่ตัวนับระหว่างสำรวจ หมดอายุ
        ประโยชน์ทันทีที่สำรวจเสร็จ) แต่เก็บ ``evicted_walls`` ไว้เป็นบันทึกประวัติ
        """
        return {
            "width": self.width,
            "height": self.height,
            "walls": self.walls,
            "known": self.known,
            "visited": sorted(list(self.visited)),
            "border_openings": sorted(list(self.border_openings)),
            "evicted_walls": self.evicted_walls,
        }

    @classmethod
    def from_state(cls, state):
        """Maze: สร้างกลับจาก ``to_state()`` (ใช้ตอนโหลดแผนที่เก่ามาวางแผนต่อ)

        สร้างโดยไม่ผ่าน ``_add_borders`` ปกติ (ที่ ``__init__`` เรียกอัตโนมัติ)
        เพราะ state ที่บันทึกไว้มีข้อมูลกำแพงขอบสนามอยู่แล้วครบถ้วน ถ้าเรียกซ้ำ
        จะไม่ผิดอะไร (ผลลัพธ์เหมือนเดิม) แต่เขียนให้ตรงไปตรงมากว่าคือ set ค่าตรง ๆ
        """
        maze = cls.__new__(cls)
        maze.width = state["width"]
        maze.height = state["height"]
        maze.walls = state["walls"]
        maze.known = state["known"]
        maze.visited = set(tuple(c) for c in state["visited"])
        maze.border_openings = set(tuple(c) for c in state["border_openings"])
        maze._open_streak = {}
        maze.evicted_walls = [tuple(row) for row in state.get("evicted_walls", [])]
        return maze

    def _edge_glyph(self, x, y, direction, horizontal):
        """str: สัญลักษณ์ของขอบหนึ่งด้าน แยกกำแพง / โล่งที่ยืนยันแล้ว / ยังไม่เคยดู"""
        if self.has_wall(x, y, direction):
            return "---" if horizontal else "|"
        if self.is_known(x, y, direction):
            return "   " if horizontal else " "
        return " . " if horizontal else ":"

    def render(self, robot=None, legend=False):
        """วาดแผนที่เป็น ASCII พร้อมเลขกำกับแกน x/y เทียบพิกัดจริงได้ตรง ๆ

        ขอบที่ยังไม่เคยตรวจจะแสดงเป็นจุด เพื่อให้แยกออกจากขอบที่ยืนยันแล้วว่าโล่ง
        ช่องที่หุ่นเคยไปยืนแล้วจะมีจุดกลางช่อง ( . ) ส่วนช่องที่ยังไม่เคยไปเว้นว่างไว้

        Args:
            robot (tuple): (x, y, heading) ตำแหน่งหุ่น ใส่ None ได้
            legend (bool): ต่อท้ายด้วยคำอธิบายสัญลักษณ์และความคืบหน้าการสำรวจ

        Returns:
            str: แผนที่หลายบรรทัด พร้อม print ได้เลย
        """
        gutter = "    "  # เผื่อที่ให้ป้ายแถว "y=N " ด้านซ้าย (กว้าง 4 ตัวอักษร)
        lines = [gutter + " " + "".join(
            "{0:^3} ".format(x) for x in range(self.width))]
        for y in range(self.height - 1, -1, -1):
            top = "+"
            for x in range(self.width):
                top += self._edge_glyph(x, y, NORTH, True) + "+"
            lines.append(gutter + top)

            mid = self._edge_glyph(0, y, WEST, False)
            for x in range(self.width):
                if robot is not None and (robot[0], robot[1]) == (x, y):
                    cell = " {0} ".format(DIR_ARROWS[robot[2]])
                elif self.is_visited(x, y):
                    cell = " . "
                else:
                    cell = "   "
                mid += cell + self._edge_glyph(x, y, EAST, False)
            lines.append("y={0:<2}".format(y) + mid)

        bottom = "+"
        for x in range(self.width):
            bottom += self._edge_glyph(x, 0, SOUTH, True) + "+"
        lines.append(gutter + bottom)

        if legend:
            seen, total = self.edge_stats()
            lines.append("--- = กำแพง | ว่าง = ตรวจแล้วโล่ง | . = เคยไปแล้ว | ช่องว่าง = ยังไม่เคยไป"
                         "   (ด้านที่สำรวจแล้ว {0}/{1}, ช่องที่สำรวจแล้ว {2:.1f}%)"
                         .format(seen, total, self.coverage_pct()))
        return "\n".join(lines)


# =====================================================================
# ตัววางแผนสำรวจ - DFS + backtrack จนครบทุกช่องที่ไปถึงได้
# =====================================================================
class ExplorePlanner(object):
    """ตัดสินใจว่าก้าวต่อไปควรไปช่องไหน เพื่อสำรวจให้ครบทุกช่องที่ไปถึงได้

    ตรรกะพอร์ตมาจาก DFS + backtrack ของงาน Occupancy Grid Mapping เดิม (ดู
    ``Occupancy112.py`` ``ExplorePlanner``) แต่ปรับให้ใช้ API ของ ``Maze`` ในไฟล์
    นี้ ที่ต่างจากต้นฉบับคือ ``Driver`` ที่นี่ไม่มีเซนเซอร์หลังและไม่ถอยหลังแบบ
    มองไม่เห็นเลย backtrack ทุกครั้งจึงเป็น "หมุนหันไปทางที่จะเดินแล้วเดินหน้า"
    เหมือนการเดินไปช่องใหม่ทุกประการ ทำให้ ``"go"`` กับ ``"back"`` ใช้โค้ดขับ
    เดียวกันได้ ไม่ต้องแยกเคสระดับการเคลื่อนที่เลย
    """

    #: ลำดับทิศที่พิจารณาก่อน (เทียบกับทิศที่หุ่นหันอยู่): หน้า, ขวา, ซ้าย, หลัง
    PREFERENCE = (0, 1, 3, 2)

    def __init__(self, start_cell):
        self.stack = [tuple(start_cell)]
        #: set: เก็บคู่ช่อง (a, b) ที่ map บอกว่าเดินได้แต่หุ่นเดินจริงไม่ผ่านซ้ำ ๆ
        self.blocked = set()

    @staticmethod
    def _edge(cell_a, cell_b):
        """frozenset: กุญแจไม่มีทิศทางของ edge ระหว่างสองช่อง ใช้กับ ``blocked``"""
        return frozenset((cell_a, cell_b))

    def block_edge(self, cell_a, cell_b):
        """ปิดทาง (a, b) ไว้ เพราะเดินจริงไม่ผ่านซ้ำ ๆ ทั้งที่แผนที่บอกว่าโล่ง"""
        self.blocked.add(self._edge(cell_a, cell_b))

    def _edge_blocked(self, cell_a, cell_b):
        return self._edge(cell_a, cell_b) in self.blocked

    def has_other_exit(self, maze, cell, avoid_cell):
        """bool: ช่อง ``cell`` มีทางออกอื่นนอกจากไปทาง ``avoid_cell`` หรือไม่

        ใช้กันไม่ให้ ``block_edge`` ปิดทางออกทางเดียวของช่องจนหุ่นขังตัวเอง
        (พอร์ตกฎ ``BACK-ONLYWAY`` จาก Occupancy112 มา)
        """
        x, y = cell
        for direction in range(4):
            if maze.has_wall(x, y, direction):
                continue
            nb = (x + DX[direction], y + DY[direction])
            if not maze.in_bounds(*nb) or nb == avoid_cell:
                continue
            if self._edge_blocked(cell, nb):
                continue
            return True
        return False

    def next_move(self, maze, x, y, heading):
        """ตัดสินใจก้าวถัดไป

        Returns:
            tuple: ``("go", cell)`` ไปช่องข้างเคียงที่ยังไม่เคยไป,
            ``("back", cell)`` ถอยกลับ/เดินไปตาม stack (ก้าวเดียว),
            หรือ ``("done", None)`` เมื่อไม่เหลือช่องที่ไปถึงได้แล้ว
        """
        cur = (x, y)

        # 1) มีช่องข้างเคียงที่ยังไม่เคยไปไหม (เรียงหน้า->ขวา->ซ้าย->หลัง)
        for rel in self.PREFERENCE:
            direction = (heading + rel) % 4
            nb = (x + DX[direction], y + DY[direction])
            if not maze.in_bounds(*nb):
                continue
            if maze.is_visited(*nb) or self._edge_blocked(cur, nb):
                continue
            if maze.has_wall(x, y, direction):
                continue
            self.stack.append(nb)
            return "go", nb

        # 2) ไม่มีช่องใหม่ติดกัน -> ถอยตาม stack ทีละก้าว
        if self.stack and self.stack[-1] == cur:
            self.stack.pop()
        if self.stack and not self._edge_blocked(cur, self.stack[-1]):
            return "back", self.stack[-1]

        # 3) stack ว่างด้วย -> หาช่องที่ยังไม่เคยไปแต่ไปถึงได้จริงที่ใกล้สุด
        #    แล้วสร้างเส้นทางทับเข้า stack (เดินด้วยกลไก "back" ตัวเดียวกัน)
        frontier = [
            (cx, cy)
            for cx in range(maze.width) for cy in range(maze.height)
            if not maze.is_visited(cx, cy)
        ]
        if not frontier:
            return "done", None

        reach = maze.flood_known([cur])
        candidates = [c for c in frontier if reach[c[0]][c[1]] < INF]
        if not candidates:
            return "done", None
        candidates.sort(key=lambda c: abs(c[0] - x) + abs(c[1] - y))

        for target in candidates:
            path = maze.shortest_known_path(cur, target)
            if path and len(path) >= 2:
                self.stack = list(reversed(path))
                if self.stack[-1] == cur:
                    self.stack.pop()
                return "back", self.stack[-1]

        return "done", None


# =====================================================================
# การเคลื่อนที่
# =====================================================================
class Driver(object):
    """ชั้นควบคุมการเคลื่อนที่ระดับ "หนึ่งช่อง" และ "หนึ่งการเลี้ยว"

    การหมุนทุกครั้งใช้ ``chassis.drive_speed`` แบบปิดลูปกับ IMU เท่านั้น
    ไม่ใช้ ``chassis.move(z=)`` เลย เพราะสองตัวนี้ใช้เครื่องหมายตรงข้ามกัน
    (ดูหัวข้อ "ข้อควรรู้เรื่อง SDK" ด้านบนของไฟล์)

    Args:
        chassis: ออบเจกต์ chassis ของหุ่น
        hub (SensorHub): ตัวอ่านเซนเซอร์
    """

    def __init__(self, chassis, hub):
        self.chassis = chassis
        self.hub = hub

    # ---------- พื้นฐาน ----------
    def stop(self):
        """สั่งหยุดล้อทันทีด้วย drive_speed(0,0,0) - ใช้ตอนเริ่ม/จบ step"""
        try:
            self.chassis.drive_speed(x=0, y=0, z=0, timeout=DRIVE_WATCHDOG_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))

    def _drive(self, x=0.0, y=0.0, z=0.0):
        """ส่งคำสั่งขับพร้อม watchdog กันหุ่นวิ่งต่อถ้า control loop ตาย"""
        self.chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_WATCHDOG_S)

    @staticmethod
    def _move_timeout(dx_m=0.0, dy_m=0.0, dz_deg=0.0):
        """float: เวลาที่ให้รอ wait_for_completed() ก่อนถือว่าค้าง"""
        t_xy = (abs(dx_m) + abs(dy_m)) / MOVE_XY_SPEED if (dx_m or dy_m) else 0.0
        t_z = abs(dz_deg) / MOVE_Z_SPEED if dz_deg else 0.0
        return max(t_xy, t_z) * 2.0 + MOVE_TIMEOUT_MARGIN_S

    def bounded_move(self, dx_m=0.0, dy_m=0.0, dz_deg=0.0):
        """สั่ง ``chassis.move()`` ก้อนเดียว แล้วรอให้เฟิร์มแวร์ยืนยัน

        ใช้เฉพาะตอนเลี้ยวและตอนจัดกึ่งกลาง (ไม่เคยใช้ตอนเดินหน้าเข้าช่องใหม่
        เพราะ ``chassis.move()`` ยกเลิกกลางทางไม่ได้ - ดู docstring หัวไฟล์)
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
        """float: ตัดขนาดก้าวให้ปลอดภัยเสมอ จากค่า ToF ที่อ่านสด ๆ ณ ตอนนี้"""
        if current_mm is None:
            return 0.0
        room = current_mm - min_safe_mm
        if room <= 0:
            return 0.0
        return max(0.0, min(desired_delta_mm, room))

    # ---------- การหมุน (chassis.move(z=) ก้อนเดียว) ----------
    def turn_to(self, current_heading, target_heading):
        """หมุนจาก grid heading หนึ่งไปอีกอัน เลือกทิศที่หมุนน้อยสุด สั่งครั้งเดียว
        (180 องศาก็สั่งทีเดียวได้ เพราะ ``chassis.move()`` ปิดลูปเอง ไม่ต้องกลัว
        ความลังเลแบบตัวคุม P เดิม) แล้วชี้ gimbal กลับหน้าเสมอ (ไม่ตามแชสซี
        อัตโนมัติใน FREE mode)

        Returns:
            int: ทิศที่หุ่นหันอยู่จริงหลังหมุนเสร็จ
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
        self.bounded_move(dz_deg=delta_deg)
        self.hub.point_gimbal(GIMBAL_YAW_FRONT)
        return target_heading

    # ---------- การจัดกึ่งกลางช่อง (chassis.move() ทีละก้อน) ----------
    def _creep_front_back(self, target_mm, tol_mm, max_iters, max_total_mm):
        """ไล่ปรับระยะหน้า-หลังด้วย ToF (gimbal ต้องชี้หน้าอยู่แล้วก่อนเรียก)"""
        self.hub.phase = "center_front_back"

        def _read():
            return self.hub.snapshot().tof_mm

        final_mm = _read()
        start_pos = self.hub.snapshot()
        for _ in range(max_iters):
            if final_mm is None:
                break
            error = final_mm - target_mm
            if abs(error) <= tol_mm:
                break
            now_pos = self.hub.snapshot()
            moved_mm = 1000.0 * math.hypot(now_pos.pos_x - start_pos.pos_x,
                                            now_pos.pos_y - start_pos.pos_y)
            room_mm = max_total_mm - moved_mm
            if room_mm <= 0:
                print("[CENTER-X] ขยับสะสมแล้ว {0:.0f}mm ครบเพดาน {1}mm - หยุดไว้ก่อน"
                      .format(moved_mm, max_total_mm))
                break
            desired = min(abs(error), MAX_CREEP_STEP_MM, room_mm)
            if error > 0:
                step_mm = self.safe_step_mm(final_mm, desired)
                if step_mm <= 0:
                    break
                delta_m = step_mm / 1000.0
            else:
                delta_m = -desired / 1000.0
            self.bounded_move(dx_m=delta_m)
            final_mm = _read()
        return final_mm

    def _sharp_lateral_hold(self, left_wall, right_wall, max_total_mm):
        """ไล่ปรับระยะซ้าย-ขวาด้วย Sharp (3 กรณี ตาม filter ที่ตกลงกัน)

        1) เห็นกำแพงทั้ง 2 ฝั่ง -> เทียบ closeness ซ้าย=ขวากันเอง (self-reference)
        2) เห็นข้างเดียว -> ยึด closeness ของฝั่งนั้นเทียบ REF ที่คาลิเบรตไว้
        3) ไม่เห็นเลย -> ไม่แก้อะไร (พื้นที่โล่ง ไม่มีอะไรให้อ้างอิง)

        คุมบนค่า ADC ดิบ (ผ่าน ``sharp_closeness``) ไม่แปลงเป็น mm ตามเหตุผล
        เดียวกับเวอร์ชันแรก: จุดที่ต้องการคือ "จุดสมดุล" ซึ่งไม่ขึ้นกับเส้นโค้ง
        ของเซนเซอร์ การแปลงเป็นระยะทางมีแต่จะเพิ่มโอกาสผิดจากสมการ calibration

        Returns:
            dict: ค่า ADC สุดท้าย/error/จำนวนรอบ ไว้ log
        """
        self.hub.phase = "center_lateral"
        result = {"left_adc": None, "right_adc": None, "iters": 0, "final_err": None,
                  "mode": "none"}
        if not left_wall and not right_wall:
            snap = self.hub.snapshot()
            result["left_adc"], result["right_adc"] = snap.adc_left, snap.adc_right
            print("[CENTER-Y] ไม่เห็นกำแพงทั้ง 2 ฝั่ง (พื้นที่โล่ง) - ไม่แก้ซ้าย-ขวา")
            return result

        start_pos = self.hub.snapshot()
        for i in range(LATERAL_MAX_ITERS):
            result["iters"] = i + 1
            snap = self.hub.snapshot()
            result["left_adc"], result["right_adc"] = snap.adc_left, snap.adc_right

            now_pos = self.hub.snapshot()
            moved_mm = 1000.0 * math.hypot(now_pos.pos_x - start_pos.pos_x,
                                            now_pos.pos_y - start_pos.pos_y)
            if moved_mm >= max_total_mm:
                print("[CENTER-Y] ขยับสะสมแล้ว {0:.0f}mm ครบเพดาน {1}mm - หยุดไว้ก่อน"
                      .format(moved_mm, max_total_mm))
                break

            if left_wall and right_wall:
                result["mode"] = "both"
                cl = sharp_closeness(snap.adc_left, SHARP_LEFT_CLOSER_IS_HIGHER)
                cr = sharp_closeness(snap.adc_right, SHARP_RIGHT_CLOSER_IS_HIGHER)
                error = cl - cr  # บวก = ใกล้ซ้ายเกิน -> ต้องขยับขวา (+y)
            elif left_wall:
                result["mode"] = "left_only"
                cl = sharp_closeness(snap.adc_left, SHARP_LEFT_CLOSER_IS_HIGHER)
                cl_ref = sharp_closeness(SHARP_LEFT_REF, SHARP_LEFT_CLOSER_IS_HIGHER)
                error = cl - cl_ref
            else:
                result["mode"] = "right_only"
                cr = sharp_closeness(snap.adc_right, SHARP_RIGHT_CLOSER_IS_HIGHER)
                cr_ref = sharp_closeness(SHARP_RIGHT_REF, SHARP_RIGHT_CLOSER_IS_HIGHER)
                error = cr_ref - cr  # บวก = ใกล้ขวาเกิน -> ต้องขยับขวา (+y) เช่นกัน

            result["final_err"] = error
            if abs(error) <= LATERAL_TOLERANCE_ADC:
                print("[CENTER-Y] นิ่งแล้ว ({0}) err={1}".format(result["mode"], error))
                break

            print("[CENTER-Y] รอบ {0}/{1} ({2}) L={3} R={4} err={5}"
                  .format(i + 1, LATERAL_MAX_ITERS, result["mode"],
                          snap.adc_left, snap.adc_right, error))
            step_mm = min(MAX_CREEP_STEP_MM, max_total_mm - moved_mm)
            if step_mm <= 0:
                break
            delta_m = (step_mm / 1000.0) * (1.0 if error > 0 else -1.0)
            self.bounded_move(dy_m=delta_m)

        return result

    def recenter_lateral(self, walls, heading):
        """จัดกึ่งกลางซ้าย-ขวาด้วย Sharp อีกครั้ง เฉพาะหลังเลี้ยวเสร็จ (ก่อนเดินหน้า)

        ``center_in_cell`` จัดกึ่งกลางไปแล้วครั้งหนึ่งตอนยังหัน heading เดิม
        (ก่อนตัดสินใจ/ก่อนเลี้ยว) แต่ ``move_accuracy_test.py`` พบว่าการหมุนของ
        ``chassis.move(z=)`` มักเหลือดริฟท์ด้านข้างเล็กน้อยหลังหมุนเสร็จ (ดู
        docstring ``_wait_until_still`` ในไฟล์นั้น) ซึ่งเกิด "หลัง" จุดที่เคย
        จัดกึ่งกลางไปแล้ว จึงไม่ถูกแก้ ต้องเรียกซ้ำอีกรอบโดยคำนวณ left/right
        ใหม่จาก ``heading`` ปัจจุบัน (หลังเลี้ยวแล้ว ไม่ใช่ heading ตอนสแกน)
        ยังคงเป็นการจัดกึ่งกลางตอนจอดนิ่งเหมือนเดิมทุกประการ (หุ่นหมุนเสร็จ
        แล้ว ยังไม่เริ่มเดินหน้า) จึงไม่ขัดกฎ "ห้ามปรับระหว่างเดินหน้า"

        Args:
            walls (dict): ผลจาก ``hub.read_walls_scan()`` ของช่องนี้ (index
                ด้วยทิศสัมบูรณ์ ไม่เปลี่ยนตามการหมุน จึงใช้ค่าเดิมได้เลย)
            heading (int): ทิศที่หุ่นหันอยู่ตอนนี้ (หลังเลี้ยวแล้ว)

        Returns:
            dict: ผลลัพธ์จาก ``_sharp_lateral_hold`` ไว้ log
        """
        left_wall = walls[(heading + 3) % 4]["wall"]
        right_wall = walls[(heading + 1) % 4]["wall"]
        return self._sharp_lateral_hold(left_wall, right_wall, MAX_TOTAL_LATERAL_MM)

    def center_in_cell(self, walls, heading):
        """จัดกึ่งกลางช่องให้แม่นที่สุดก่อนออกเดิน (ปรับได้แค่ตอนจอดนิ่งเท่านั้น)

        front-back ใช้ ToF (สแกนไปแล้วตอน ``read_walls_scan`` gimbal ชี้หน้าอยู่
        พอดี), ซ้าย-ขวาใช้ Sharp เท่านั้น (ToF ไม่ต้องสแกนซ้ำ)

        Args:
            walls (dict): ผลจาก ``hub.read_walls_scan(heading)`` ของช่องนี้
            heading (int): ทิศที่หุ่นหันอยู่ตอนนี้

        Returns:
            dict: รวมผลลัพธ์จาก front-back + ซ้าย-ขวา ไว้ log
        """
        front_mm = self._creep_front_back(FRONT_STOP_MM, ALIGN_TOLERANCE_MM,
                                           LATERAL_MAX_ITERS, MAX_CREEP_TOTAL_MM)
        print("[CENTER-X] หน้า={0}mm target={1}mm".format(front_mm, FRONT_STOP_MM))

        left_wall = walls[(heading + 3) % 4]["wall"]
        right_wall = walls[(heading + 1) % 4]["wall"]
        lateral = self._sharp_lateral_hold(left_wall, right_wall, MAX_TOTAL_LATERAL_MM)

        return {"front_mm": front_mm, "lateral": lateral}

    # ---------- การเดินหน้า/ถอย: drive_speed คงที่ ไม่มีการปรับใด ๆ ระหว่างทาง ----------
    def advance_one_cell(self, heading):
        """เดินหน้าหนึ่งช่องตาราง ด้วยความเร็วคงที่ค่าเดียว (``HOP_SPEED``) ไม่มี
        y หรือ z เลยตลอดการเดิน - ตกลงกับผู้ใช้ชัดเจนว่าห้ามมีการปรับทิศ/ตำแหน่ง
        ด้วยเซนเซอร์ใด ๆ ระหว่างเดินหน้าเข้าช่องใหม่ทั้งสิ้น เพราะสนามเป็นช่อง
        สี่เหลี่ยมฉาก ไม่มีความเอียง จอดกึ่งกลาง+หันหัวแม่นตั้งแต่ต้นก็เพียงพอให้
        เข้าเป้าหมายกึ่งกลางช่องถัดไปได้เองโดยไม่ต้องมีฟีดแบ็กระหว่างทาง
        (เวอร์ชันก่อนหน้าเคยลองใส่ฟีดแบ็กระหว่างทางแล้วดริฟท์ไม่จบสิ้น - ดู
        docstring หัวไฟล์)

        ใช้ ``drive_speed()`` ไม่ใช่ ``chassis.move()`` เพราะสั่ง 0 แทรกได้จริง
        และหยุดจริงทันที - เป็นทางเดียวที่ IR/impact/ToF-ฉุกเฉิน จะ "หยุดหุ่น
        ได้จริง" ระหว่างทาง (จุดเสี่ยงชนที่สุด) ไม่ใช่ best-effort

        วัดระยะที่เดินได้จาก **ระยะที่โปรเจกต์ลงบนแกนหน้า (ทิศ yaw ตอนเริ่มฮ็อป)**
        ไม่ใช่ระยะตรงเส้นตรง (``hypot``) เหมือนเวอร์ชันก่อนหน้า เพราะพบจากสนาม
        จริงว่าถ้าหุ่นสไลด์ด้านข้างระหว่างฮ็อป (ล้อ mecanum ไม่สมดุล/พื้นลื่นไม่
        เท่ากัน แม้จะสั่ง ``x`` อย่างเดียวก็ตาม) ระยะสไลด์นั้นจะไปรวมเข้ากับ
        ``hypot`` ทำให้แตะเป้าหมาย ``CELL_SIZE_M`` "ครบ" ทั้งที่จริงยังไปข้างหน้า
        ไม่ถึงกึ่งกลางช่องถัดไป กลายเป็นหยุดเอียงออกข้าง แล้วตอนจอดคิดว่าตัวเอง
        มาถึงช่องใหม่แล้ว (บั๊กที่ทำให้เห็นหุ่นแกว่งวนไปมาระหว่าง 2 ช่องติดกัน)
        การโปรเจกต์แบบนี้เป็นแค่การ**วัด**เพื่อตัดสินใจว่าจะหยุดเมื่อไหร่ ไม่ใช่
        การปรับทิศ/ตำแหน่งระหว่างทาง (ยังไม่มี y/z ส่งออกไปเลยเหมือนเดิม) จึงไม่
        ขัดกฎ "ห้ามปรับระหว่างเดินหน้า"

        Returns:
            tuple: (ok, traveled_m, reason)
        """
        self.stop()
        time.sleep(SETTLE_S)
        self.hub.phase = "hop"

        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        fwd_yaw_rad = math.radians(start.yaw)
        fwd_dir_x, fwd_dir_y = math.cos(fwd_yaw_rad), math.sin(fwd_yaw_rad)

        def _forward_traveled(snap):
            return ((snap.pos_x - start_x) * fwd_dir_x
                     + (snap.pos_y - start_y) * fwd_dir_y)

        timeout = (CELL_SIZE_M / HOP_SPEED) * MOVE_TIMEOUT_RATIO
        deadline = time.time() + timeout
        reason = "timeout"

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                reason = "sensor_stale:" + snap.stale_reason
                break

            traveled = _forward_traveled(snap)
            if traveled >= CELL_SIZE_M:
                reason = "odometry"
                break

            if snap.emergency_trip():
                if snap.impact or snap.roll_over:
                    reason = "impact"
                elif ir_triggered(snap.ir_left) or ir_triggered(snap.ir_right):
                    reason = "ir_estop"
                else:
                    reason = "tof_emergency"
                break

            # คงที่ค่าเดียว ไม่มี y (strafe) ไม่มี z (yaw correction) เลย - ตาม
            # กฎ "ห้ามปรับระหว่างทาง" ที่ตกลงกันไว้
            self._drive(x=HOP_SPEED)
            time.sleep(CONTROL_DT)

        self.stop()
        time.sleep(0.15)

        snap = self.hub.snapshot()
        traveled = _forward_traveled(snap)
        ok = traveled >= CELL_SIZE_M * CELL_COMPLETE_RATIO
        print("[MOVE] {0} เดินได้ {1:.3f} m (เป้า {2:.2f}) เหตุที่จบ: {3} -> {4}"
              .format(DIR_NAMES[heading], traveled, CELL_SIZE_M, reason,
                      "สำเร็จ" if ok else "ไม่สำเร็จ"))
        return ok, traveled, reason

    def backup(self, distance_m):
        """ถอยกลับตามระยะที่กำหนด ด้วยความเร็วคงที่เช่นกัน (ไม่มีการปรับทิศ
        ระหว่างทาง เหมือน ``advance_one_cell``) เพื่อกลับไปยืนกลางช่องเดิมหลัง
        เดินไม่ผ่าน - x ติดลบเป็นพิกัดสัมพัทธ์กับตัวรถเองอยู่แล้ว ไม่ต้องรู้
        heading สัมบูรณ์
        """
        if distance_m < 0.03:
            return
        print("[BACK] ถอยกลับ {0:.3f} m เข้าช่องเดิม".format(distance_m))
        self.hub.phase = "backup"
        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        fwd_yaw_rad = math.radians(start.yaw)
        fwd_dir_x, fwd_dir_y = math.cos(fwd_yaw_rad), math.sin(fwd_yaw_rad)
        deadline = time.time() + (distance_m / BACKUP_SPEED) * 2.0 + 1.0

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                break
            # โปรเจกต์ลงแกนหน้า-หลังเหมือน advance_one_cell (กันสไลด์ข้างไป
            # รวมกับ hypot แล้วหยุดถอยก่อนถึงจริง)
            backed = -((snap.pos_x - start_x) * fwd_dir_x
                       + (snap.pos_y - start_y) * fwd_dir_y)
            if backed >= distance_m:
                break
            if snap.impact or snap.roll_over:
                print("[BACK] impact/roll_over ระหว่างถอย - หยุดทันที")
                break
            self._drive(x=-BACKUP_SPEED)
            time.sleep(CONTROL_DT)

        self.stop()
        time.sleep(0.15)


# =====================================================================
# ช่วยจัดการทิศ/timestamp + export ผลลัพธ์ (Map / Log / Trajectory)
# =====================================================================
def _timestamp():
    """str: timestamp ไว้ตั้งชื่อไฟล์ผลลัพธ์ของการวิ่งแต่ละรอบ"""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _edge_direction(cell_a, cell_b):
    """int: ทิศจาก cell_a ไป cell_b (ต้องเป็นช่องติดกัน)"""
    for direction in range(4):
        if (cell_a[0] + DX[direction], cell_a[1] + DY[direction]) == tuple(cell_b):
            return direction
    raise ValueError("{0} กับ {1} ไม่ได้ติดกัน".format(cell_a, cell_b))


def export_run(maze, ts, start_cell, start_heading, end_cell, end_heading,
                steps_rows, trajectory_rows):
    """เซฟผลการสำรวจเป็นไฟล์: แผนที่ (Map) / log การตัดสินใจ / trajectory / สรุป

    ไฟล์ทั้งหมดตั้งชื่อด้วย ``LOG_PREFIX`` และ timestamp เดียวกัน เก็บไว้โฟลเดอร์
    เดียวกับไฟล์นี้ เพื่อให้หาไฟล์ของการวิ่งแต่ละรอบง่าย ตรงกับ deliverable ข้อ
    1-3 ของโจทย์ (Map, Log ข้อมูลการสำรวจ, Robot Trajectory) ส่วนข้อ 4
    (Map Accuracy / Coverage เทียบ Ground Truth) อยู่ใน ``compare_ground_truth.py``
    แยกไฟล์ เพราะต้องมีไฟล์ ground truth ที่อาจารย์ให้วันจริงมาเทียบด้วย
    """
    map_txt_path = _log_path("{0}_map_{1}.txt".format(LOG_PREFIX, ts))
    with open(map_txt_path, "w", encoding="utf-8") as f:
        f.write(maze.render(robot=(end_cell[0], end_cell[1], end_heading), legend=True))
        f.write("\n")

    # แผนที่ occupancy แบบตาราง 0=สำรวจแล้ว/โล่ง 1=ตันรอบด้าน 0.5=ยังไม่เคยไป
    # ใช้เทียบกับ Ground Truth Map ตอนพรีเซนต์ได้ง่าย (รูปแบบเดียวกันทุกกลุ่ม)
    map_csv_path = _log_path("{0}_map_{1}.csv".format(LOG_PREFIX, ts))
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

    steps_csv_path = _log_path("{0}_steps_{1}.csv".format(LOG_PREFIX, ts))
    with open(steps_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "x", "y", "heading",
                          "tof_front_mm", "tof_right_mm", "tof_back_mm", "tof_left_mm",
                          "wall_N", "wall_E", "wall_S", "wall_W",
                          "adc_left", "adc_right", "center_iters", "center_err",
                          "kind", "target", "ok", "reason", "impact", "slip",
                          "cam_white_pct"])
        writer.writerows(steps_rows)

    traj_csv_path = _log_path("{0}_trajectory_{1}.csv".format(LOG_PREFIX, ts))
    with open(traj_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "step", "x", "y", "heading", "pos_x", "pos_y", "yaw"])
        writer.writerows(trajectory_rows)

    seen, total_edges = maze.edge_stats()
    summary_lines = [
        "เริ่มที่ {0} หัน {1}".format(start_cell, DIR_NAMES[start_heading]),
        "สิ้นสุดที่ {0} หัน {1}".format(end_cell, DIR_NAMES[end_heading]),
        "สำรวจ {0}/{1} ช่อง ({2:.1f}%)".format(
            len(maze.visited), maze.width * maze.height, maze.coverage_pct()),
        "ด้านที่ตรวจแล้ว {0}/{1}".format(seen, total_edges),
        "กำแพงที่เคยมาร์กแล้วถูกลบภายหลัง (evicted, ยืนยันโล่งซ้ำ >= {0} ครั้ง): {1}"
            .format(WALL_OPEN_CONFIRM_STREAK, len(maze.evicted_walls)),
    ]
    for x, y, direction, streak in maze.evicted_walls:
        summary_lines.append("  - ({0},{1}) ทิศ {2} (ยืนยันโล่ง {3} ครั้ง)"
                              .format(x, y, DIR_NAMES[direction], streak))
    summary_path = _log_path("{0}_summary_{1}.txt".format(LOG_PREFIX, ts))
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines) + "\n")

    # ไฟล์ state (JSON) - ต่างจาก 4 ไฟล์ข้างบนตรงที่โหลดกลับมาสร้าง Maze ใหม่ได้
    # ครบทุกบิต ใช้ตอนสั่ง --goto/--return-start ในภายหลัง โดยไม่ต้องสำรวจใหม่
    state_path = _log_path("{0}_state_{1}.json".format(LOG_PREFIX, ts))
    with open(state_path, "w", encoding="utf-8") as f:
        json.dump({
            "start_cell": list(start_cell), "start_heading": start_heading,
            "end_cell": list(end_cell), "end_heading": end_heading,
            "maze": maze.to_state(),
        }, f, ensure_ascii=False, indent=1)

    print("\n[EXPORT] บันทึกผลลัพธ์แล้ว:")
    for path in (map_txt_path, map_csv_path, steps_csv_path, traj_csv_path,
                 summary_path, state_path):
        print("  - {0}".format(path))


# =====================================================================
# State machine หลัก - สำรวจให้ครบทุกช่องที่ไปถึงได้ (full coverage)
# =====================================================================
def run_explore(hub, driver):
    """สำรวจแผนที่ที่ไม่รู้จักมาก่อนให้ครบทุกช่องที่ไปถึงได้ แล้ว export ผลลัพธ์

    ลำดับต่อ 1 ช่อง: จอดนิ่ง -> สแกน ToF ครบ 4 ทิศ (ความจริงเรื่องกำแพง) ->
    จัดกึ่งกลาง (ToF หน้า-หลัง, Sharp ซ้าย-ขวา) -> ตัดสินใจก้าวต่อไป -> เลี้ยว
    (chassis.move()) -> ยืนยัน ToF หน้าอีกครั้งสด ๆ -> เดินหน้า (drive_speed
    คงที่ ไม่มีการปรับใด ๆ ระหว่างทาง)

    Returns:
        bool: True เมื่อสำรวจจบแบบ "done" (ไม่เหลือช่องที่ไปถึงได้อีกแล้ว)
    """
    ts = _timestamp()
    t0 = time.time()
    maze = Maze(MAZE_W, MAZE_H, BORDER_OPENINGS)
    planner = ExplorePlanner(START_CELL)
    x, y = START_CELL
    heading = START_HEADING

    print("=" * 62)
    print("  SLAM EXPLORE RUN - สำรวจให้ครบทุกช่อง")
    print("  สนาม {0}x{1} ({2} ช่อง) ช่องละ {3:.2f} m | เริ่มที่ {4} หัน {5}"
          .format(MAZE_W, MAZE_H, MAZE_W * MAZE_H, CELL_SIZE_M, START_CELL,
                  DIR_NAMES[START_HEADING]))
    print("=" * 62)

    steps_rows = []
    trajectory_rows = []
    # นับความล้มเหลวซ้ำที่ (ช่อง, ทิศ) เดิม ใช้ตัดวงจรกรณีเดินไม่ผ่านแต่ ToF
    # ก็ไม่เห็นกำแพง (ล้อลื่น ติดขอบ ฯลฯ) ซึ่งถ้าไม่ตัดจะเลือกทิศเดิมซ้ำไปเรื่อย ๆ
    fail_key = None
    fail_count = 0
    done = False

    try:
        for step in range(MAX_STEPS):
            print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} ---"
                  .format(step, x, y, DIR_NAMES[heading]))

            driver.stop()
            time.sleep(SETTLE_S)
            hub.set_phase(step, "settle")
            snap = hub.snapshot()
            if not snap.fresh:
                print("[ERROR] เซนเซอร์ {0} ขาดการอัปเดต หยุดเพื่อความปลอดภัย"
                      .format(snap.stale_reason))
                break
            if snap.impact or snap.roll_over:
                print("[ERROR] impact/roll_over ค้างอยู่ - หยุดเพื่อความปลอดภัย")
                break

            walls = hub.read_walls_scan(heading)
            for direction, info in walls.items():
                maze.set_wall(x, y, direction, info["wall"])
            maze.mark_visited(x, y)
            print("ToF 4 ทิศ -> หน้า:{0} ขวา:{1} หลัง:{2} ซ้าย:{3} (mm)".format(
                walls[heading]["mm"], walls[(heading + 1) % 4]["mm"],
                walls[(heading + 2) % 4]["mm"], walls[(heading + 3) % 4]["mm"]))

            center = driver.center_in_cell(walls, heading)
            print(maze.render(robot=(x, y, heading), legend=True))

            hub.phase = "decide"
            snap = hub.snapshot()
            trajectory_rows.append([
                round(time.time() - t0, 3), step, x, y, DIR_NAMES[heading],
                round(snap.pos_x, 4), round(snap.pos_y, 4), round(snap.yaw, 2),
            ])
            wall_row = [int(maze.has_wall(x, y, d)) for d in (NORTH, EAST, SOUTH, WEST)]
            tof_row = [walls[heading]["mm"], walls[(heading + 1) % 4]["mm"],
                       walls[(heading + 2) % 4]["mm"], walls[(heading + 3) % 4]["mm"]]
            lateral = center["lateral"]
            entry_heading = heading  # heading จะถูกเปลี่ยนโดย turn_to ด้านล่าง - จับค่าไว้
                                      # ก่อน เพราะ _log_row ต้องรายงานทิศตอนสแกน/จัดกึ่งกลาง
                                      # ของช่องนี้ ไม่ใช่ทิศหลังเลี้ยวไปแล้ว

            def _log_row(kind, target, ok, reason, cam_white_pct=None):
                steps_rows.append([step, x, y, DIR_NAMES[entry_heading], *tof_row, *wall_row,
                                    snap.adc_left, snap.adc_right, lateral["iters"],
                                    lateral["final_err"], kind, target, ok, reason,
                                    snap.impact, snap.slip, cam_white_pct])

            kind, target = planner.next_move(maze, x, y, heading)
            if kind == "done":
                _log_row("done", "", True, "coverage_complete")
                print("\n[DONE] สำรวจครบทุกช่องที่ไปถึงได้แล้ว ใช้ไป {0} ก้าว".format(step))
                done = True
                break

            next_heading = _edge_direction((x, y), target)
            print("ตัดสินใจ -> {0} ไปช่อง {1} (ทาง {2})"
                  .format("เดินต่อ" if kind == "go" else "ย้อนกลับ/reroute",
                          target, DIR_NAMES[next_heading]))

            hub.phase = "turn"
            heading = driver.turn_to(heading, next_heading)

            # ยืนยัน ToF หน้าอีกครั้งสด ๆ หลังหมุนเสร็จ (gimbal ชี้หน้าแล้วจาก turn_to)
            # ก่อนออกตัวจริง เป็นด่านสุดท้ายที่กันไม่ให้พุ่งชนกำแพง แต่จะมาร์กเป็น
            # กำแพงในแผนที่ได้เฉพาะตอน kind=="go" (edge ที่ยังไม่เคยยืนยัน) เท่านั้น
            # ถ้าเป็น "back" (edge เคยยืนยันว่าโล่งจากการเดินผ่านจริงมาแล้ว) จะไม่แตะ
            # แผนที่เด็ดขาด เพราะเจอกำแพงตอนนี้น่าจะเป็นเพราะหุ่นหลงตำแหน่ง ไม่ใช่
            # กำแพงงอกใหม่ - การเรียก set_wall(present=True) มีผลทันทีเสมอ (ไม่มี
            # streak ป้องกันฝั่งเพิ่ม) จึงต้องกันด้วยเงื่อนไข kind ตรงนี้ ไม่ใช่ปล่อย
            # ให้ Maze ตัดสินใจเอง
            front_mm = hub.read_tof(GIMBAL_YAW_FRONT, phase="turn_verify")
            if tof_is_wall(front_mm, FRONT_STOP_MM):
                if kind == "go":
                    print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm - ยกเลิกการเดิน "
                          "แล้วมาร์กลงแผนที่".format(front_mm))
                    maze.set_wall(x, y, heading, True)
                    if planner.stack and planner.stack[-1] == target:
                        planner.stack.pop()
                else:
                    print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm บนทางที่เคยยืนยันว่าโล่ง "
                          "- ไม่แก้แผนที่ น่าจะหลงตำแหน่ง ลองใหม่รอบหน้า".format(front_mm))
                _log_row(kind, target, False, "safety_wall")
                continue

            # แก้ดริฟท์ด้านข้างจากการหมุน (ดู recenter_lateral docstring) - ยัง
            # จอดนิ่งอยู่ ยังไม่เริ่มเดินหน้า จึงไม่ขัดกฎ "ห้ามปรับระหว่างทาง"
            hub.phase = "center_lateral_post_turn"
            driver.recenter_lateral(walls, heading)

            cam = hub.log_camera_frame("step{0:03d}".format(step))  # log-only, ไม่กระทบตัดสินใจ
            ok, traveled, reason = driver.advance_one_cell(heading)
            _log_row(kind, target, ok, reason, cam["white_pct"])

            if ok:
                x, y = target
                fail_key = None
                fail_count = 0
            else:
                key = (x, y, heading)
                fail_count = fail_count + 1 if key == fail_key else 1
                fail_key = key

                if kind == "go":
                    # ช่องใหม่ที่ยังไม่เคยยืนยัน - เชื่อ ToF หรือความล้มเหลวซ้ำได้
                    front_mm2 = hub.read_tof(GIMBAL_YAW_FRONT, phase="recover_verify")
                    if tof_is_wall(front_mm2, FRONT_STOP_MM):
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
                    # kind == "back": edge นี้เคยยืนยันว่าโล่งแล้วจริง ๆ ห้ามมาร์กเป็น
                    # กำแพง ถือเป็นปัญหาการเดิน (หลงตำแหน่ง/ล้อลื่น) ไม่ใช่ปัญหาแผนที่
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
    except KeyboardInterrupt:
        print("[STOP] ผู้ใช้สั่งหยุดกลางคัน (Ctrl+C) - จะ export สิ่งที่เก็บมาได้ทั้งหมดก่อน")
    except Exception as exc:                                # noqa: BLE001
        print("[CRASH] เกิดข้อผิดพลาดกลางคัน: {0} - จะ export สิ่งที่เก็บมาได้ทั้งหมดก่อน".format(exc))
        traceback.print_exc()

    driver.stop()
    seen, total_edges = maze.edge_stats()
    print("\n" + "=" * 62)
    print("  สรุปผล")
    print("  เริ่มที่ {0} หัน {1}".format(START_CELL, DIR_NAMES[START_HEADING]))
    print("  สิ้นสุดที่ {0} หัน {1}".format((x, y), DIR_NAMES[heading]))
    print("  สำรวจ {0}/{1} ช่อง ({2:.1f}%) | ด้านที่ตรวจแล้ว {3}/{4}"
          .format(len(maze.visited), MAZE_W * MAZE_H, maze.coverage_pct(),
                  seen, total_edges))
    print("=" * 62)

    export_run(maze, ts, START_CELL, START_HEADING, (x, y), heading,
               steps_rows, trajectory_rows)
    #: (bool, Maze, int, int, int): เผื่อผู้เรียก (main) อยากสั่งต่อทันทีแบบ
    #: ``--goto``/``--return-start`` โดยไม่ต้องสำรวจใหม่ - ตำแหน่ง/แผนที่ยังอยู่
    #: ในหน่วยความจำ ไม่ต้องโหลดจากไฟล์เลย (odometry ยัง live ต่อเนื่อง)
    return done, maze, x, y, heading


# =====================================================================
# โหมด --goto / --return-start : ใช้แผนที่ที่สำรวจไว้แล้วเดินไปช่องเป้าหมาย
# =====================================================================
def find_latest_state_file():
    """str or None: path ของไฟล์ state (``*_state_*.json``) ล่าสุดในโฟลเดอร์นี้

    ชื่อไฟล์ลงท้ายด้วย timestamp แบบ ``YYYYMMDD_HHMMSS`` ซึ่งเรียงเป็น string
    ได้ตรงกับเวลาจริงพอดี จึงหาไฟล์ล่าสุดด้วย ``max()`` ธรรมดาได้เลย
    """
    matches = sorted(glob.glob(_log_path("{0}_state_*.json".format(LOG_PREFIX))))
    return matches[-1] if matches else None


def load_state(path):
    """dict: โหลดไฟล์ state กลับมาเป็น Maze + ตำแหน่งเริ่ม/จบตอนสำรวจครั้งนั้น"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {
        "maze": Maze.from_state(data["maze"]),
        "start_cell": tuple(data["start_cell"]),
        "start_heading": data["start_heading"],
        "end_cell": tuple(data["end_cell"]),
        "end_heading": data["end_heading"],
    }


def run_goto(hub, driver, maze, x, y, heading, target_cell):
    """เดินตามแผนที่ที่รู้อยู่แล้ว (ไม่สำรวจใหม่) จากตำแหน่งปัจจุบันไปช่อง
    ``target_cell`` ใช้เส้นทางที่สั้นสุดผ่านเฉพาะด้านที่ยืนยันแล้วว่าโล่งจริง
    (``Maze.shortest_known_path`` - เข้มงวดกว่า optimistic flood fill ตอน
    สำรวจ เพราะตอนนี้ไม่ได้กำลังสำรวจแล้ว ต้องมั่นใจว่าเดินได้จริงทุกก้าว)

    ใช้ primitive การเดินชุดเดียวกับตอนสำรวจทุกอย่าง (``turn_to``,
    ``advance_one_cell``, ``center_in_cell``) ต่างกันแค่ไม่มาร์กกำแพงใหม่ลง
    แผนที่เลย (เส้นทางทุก edge ยืนยันโล่งอยู่แล้วจากการสำรวจครั้งก่อน ถ้าเดิน
    ไม่ผ่านตอนนี้คือปัญหาการเดิน/หลงตำแหน่ง ไม่ใช่ปัญหาแผนที่ - เหมือน kind
    "back" ตอนสำรวจ)

    Returns:
        tuple: (ok, x, y, heading) - ``ok`` เป็น False ถ้าไปไม่ถึงเป้าหมาย
        (ไม่ว่าเพราะแผนที่ไม่รู้จักเส้นทาง หรือเดินจริงไม่ผ่านซ้ำ ๆ)
    """
    if not maze.in_bounds(*target_cell):
        print("[GOTO] ช่องเป้าหมาย {0} อยู่นอกสนาม {1}x{2}"
              .format(target_cell, maze.width, maze.height))
        return False, x, y, heading

    path = maze.shortest_known_path((x, y), target_cell)
    if path is None:
        print("[GOTO] ไม่มีเส้นทางที่ยืนยันแล้วว่าโล่งไปช่อง {0} จากความรู้ปัจจุบัน "
              "(ถ้าเพิ่งสำรวจครบ 100% ไม่ควรเกิดกรณีนี้)".format(target_cell))
        return False, x, y, heading

    print("[GOTO] เส้นทาง {0} -> {1}: {2}".format((x, y), target_cell, path))
    for step_idx in range(len(path) - 1):
        cur, nxt = path[step_idx], path[step_idx + 1]
        next_heading = _edge_direction(cur, nxt)
        hub.set_phase(step_idx, "goto_turn")
        heading = driver.turn_to(heading, next_heading)

        front_mm = hub.read_tof(GIMBAL_YAW_FRONT, phase="goto_turn_verify")
        if tof_is_wall(front_mm, FRONT_STOP_MM):
            print("[GOTO] หันมาแล้วเจอกำแพงที่ {0}mm บนทางที่เคยยืนยันว่าโล่ง "
                  "- น่าจะหลงตำแหน่ง หยุดภารกิจ goto ไว้ก่อน".format(front_mm))
            return False, x, y, heading

        ok = False
        traveled = 0.0
        for attempt in range(BACK_RETRY):
            ok, traveled, reason = driver.advance_one_cell(heading)
            if ok:
                break
            print("[GOTO] เดิน {0}->{1} ไม่ผ่าน (ครั้งที่ {2}, เหตุผล {3}) "
                  "ถอยแล้วลองใหม่".format(cur, nxt, attempt + 1, reason))
            driver.backup(traveled)
        if not ok:
            print("[GOTO] เดิน {0}->{1} ไม่ผ่านซ้ำ {2} ครั้ง - หยุดภารกิจ goto ไว้ก่อน "
                  "ที่ช่อง {3}".format(cur, nxt, BACK_RETRY, cur))
            return False, x, y, heading

        x, y = nxt
        walls = hub.read_walls_scan(heading)
        driver.center_in_cell(walls, heading)
        print("[GOTO] ถึงช่อง {0} แล้ว หัน {1}".format((x, y), DIR_NAMES[heading]))

    print("[GOTO] ถึงเป้าหมาย {0} แล้ว".format(target_cell))
    return True, x, y, heading


# =====================================================================
# โหมด --calib : วัดค่าเซนเซอร์จริงจากหุ่นตัวนี้ในสนามนี้
# =====================================================================
def _collect(hub, samples=40, interval=0.05, label=""):
    """เก็บตัวอย่างค่าเซนเซอร์ตามจำนวนที่กำหนด แล้วคืนเป็น dict ของ list"""
    hub.phase = "calib_" + label if label else "calib"
    data = {"adc_l": [], "adc_r": [], "ir_l": [], "ir_r": [], "tof": []}
    for _ in range(samples):
        snap = hub.snapshot()
        if snap.adc_left is not None:
            data["adc_l"].append(snap.adc_left)
        if snap.adc_right is not None:
            data["adc_r"].append(snap.adc_right)
        if snap.ir_left is not None:
            data["ir_l"].append(int(snap.ir_left))
        if snap.ir_right is not None:
            data["ir_r"].append(int(snap.ir_right))
        if snap.tof_mm is not None:
            data["tof"].append(snap.tof_mm)
        time.sleep(interval)
    if label:
        print("      {0}: L={1} R={2}".format(label, _fmt(data["adc_l"]),
                                              _fmt(data["adc_r"])))
    return data


def _stats(values):
    """tuple: (mean, sd) หรือ (None, None) ถ้าไม่มีข้อมูล"""
    if not values:
        return None, None
    if len(values) == 1:
        return float(values[0]), 0.0
    return statistics.mean(values), statistics.pstdev(values)


def _fmt(values):
    """str: ข้อความสรุป mean/sd แบบสั้น"""
    mean, sd = _stats(values)
    if mean is None:
        return "ไม่มีข้อมูล"
    return "mean={0:6.1f} sd={1:5.1f}".format(mean, sd)


def _mode_value(values):
    """int or None: ค่าที่พบบ่อยที่สุดในลิสต์"""
    if not values:
        return None
    return max(set(values), key=values.count)


def run_calibration(hub):
    """ไล่วัดค่าเซนเซอร์ทีละขั้น แล้วพิมพ์บล็อก CONFIG ที่คัดลอกไปวางได้เลย

    หุ่นไม่ขยับเองตลอดกระบวนการนี้ (ผู้ใช้เป็นคนย้ายหุ่นเองตามคำสั่ง) ยกเว้น
    gimbal ที่หมุนไปมาเพื่ออ่าน ToF ทิศต่าง ๆ และ gimbal-yaw-sign self-test
    """
    print("\n" + "=" * 62)
    print("  โหมดคาลิเบรตเซนเซอร์")
    print("=" * 62)
    if not hub.use_adapter:
        print("[WARN] กำลังใช้ fallback get_adc ซึ่งช้า การเก็บตัวอย่างจะนานขึ้น")

    print("\n[gimbal] หา gimbal_yaw_sign อัตโนมัติ (หมุนทดสอบสั้น ๆ)...")
    hub.calibrate_gimbal_yaw_sign()

    print("\n[1/5] วางหุ่น 'กลางช่อง' ที่มีกำแพงทั้งซ้ายและขวา หันหน้าไปทางไหนก็ได้")
    print("      ตำแหน่งต้องอยู่กลางจริง ๆ เพราะค่านี้จะกลายเป็นจุดอ้างอิงที่")
    print("      ใช้จัดกึ่งกลางซ้าย-ขวาตอนจอดทุกครั้ง")
    input("      พร้อมแล้วกด Enter...")
    walls = _collect(hub, label="กำแพงสองข้าง")
    side_mm_left = hub.read_tof(GIMBAL_YAW_LEFT)
    side_mm_right = hub.read_tof(GIMBAL_YAW_RIGHT)
    hub.point_gimbal(GIMBAL_YAW_FRONT)
    print("      ToF ซ้าย={0}mm ขวา={1}mm".format(side_mm_left, side_mm_right))

    print("\n[2/5] ขยับหุ่นให้ชิดกำแพง 'ด้านซ้าย' มาก ๆ (ราว 5 ซม.) โดยยังอยู่")
    print("      แนวเดิม (ไม่หมุนตัว) - ขั้นนี้หาว่า ADC ซ้ายสูงขึ้นหรือต่ำลง")
    print("      เมื่อเข้าใกล้กำแพง")
    input("      พร้อมแล้วกด Enter...")
    close_left = _collect(hub, label="ชิดกำแพงซ้าย")

    print("\n[3/5] ขยับหุ่นให้ชิดกำแพง 'ด้านขวา' มาก ๆ แทน (ราว 5 ซม.)")
    input("      พร้อมแล้วกด Enter...")
    close_right = _collect(hub, label="ชิดกำแพงขวา")

    print("\n[4/6] วางหุ่น 'กลางช่อง' โดยหันหน้าชนกำแพง (ไม่ต้องมีกำแพงข้าง)")
    print("      ค่านี้จะกลายเป็นระยะที่หุ่นใช้หยุดตอนเข้าช่องที่มีกำแพงข้างหน้า")
    input("      พร้อมแล้วกด Enter...")
    front_mm = hub.read_tof(GIMBAL_YAW_FRONT)
    print("      ToF หน้า={0}mm".format(front_mm))
    hub.log_camera_frame("calib_front_wall")

    print("\n[5/6] (ถ้ามีกล้อง) หันหุ่นให้มองไปทางที่ 'ไม่มีกำแพงข้างหน้า' แทน "
          "(เช่น หันไปทางเดินโล่งๆ) - ขั้นนี้แค่เก็บรูป/สี % ขาวไว้เทียบ ไม่มีผล "
          "ต่อการตัดสินใจอะไรเลย")
    input("      พร้อมแล้วกด Enter (หรือ Enter เฉยๆ เพื่อข้าม)...")
    hub.log_camera_frame("calib_front_open")

    print("\n[6/6] เอามือบังหน้าเซนเซอร์ IR 45 องศา 'ทั้งสองตัว' พร้อมกันไว้")
    input("      พร้อมแล้วกด Enter...")
    ir_blocked = _collect(hub, samples=20)
    print("\n      เอามือออกให้ IR ทั้งสองตัวโล่ง")
    input("      พร้อมแล้วกด Enter...")
    ir_clear = _collect(hub, samples=20)

    # ---------- สรุปผล ----------
    print("\n" + "=" * 62)
    print("  ผลการคาลิเบรต")
    print("=" * 62)

    problems = []
    sharp_ref = {}
    sharp_polarity = {}
    for side, key, close_data, ref_name, pol_name in (
            ("ซ้าย", "adc_l", close_left, "SHARP_LEFT_REF", "SHARP_LEFT_CLOSER_IS_HIGHER"),
            ("ขวา", "adc_r", close_right, "SHARP_RIGHT_REF", "SHARP_RIGHT_CLOSER_IS_HIGHER")):
        ref_mean, ref_sd = _stats(walls[key])
        close_mean, close_sd = _stats(close_data[key])
        if ref_mean is None:
            problems.append("Sharp{0}: ไม่ได้รับค่า ADC เลย ตรวจการต่อสาย".format(side))
            continue
        sharp_ref[ref_name] = int(round(ref_mean))
        print("  Sharp{0}: กลางช่อง {1:.1f} (sd {2:.1f})".format(side, ref_mean, ref_sd))
        if close_mean is None:
            problems.append(
                "Sharp{0}: ขั้นชิดกำแพงไม่ได้ค่า - ตั้ง CLOSER_IS_HIGHER ไม่ได้ "
                "ต้องคาลิเบรตใหม่ก่อนใช้งานจริง".format(side))
            continue
        closer_is_higher = close_mean > ref_mean
        sharp_polarity[pol_name] = closer_is_higher
        gap = abs(close_mean - ref_mean)
        print("  Sharp{0}: ชิดมาก {1:.1f} (sd {2:.1f}) -> closer_is_higher={3} "
              "(ห่างจากกลางช่อง {4:.1f})"
              .format(side, close_mean, close_sd, closer_is_higher, gap))
        if gap < 15:
            problems.append(
                "Sharp{0}: ค่าตอนชิดกำแพงกับตอนกลางช่างต่างกันแค่ {1:.1f} ADC "
                "น้อยเกินจะแยกได้แน่นอน ลองขยับให้ชิดกำแพงกว่านี้แล้ววัดใหม่"
                .format(side, gap))

    ir_value = None
    for side, key in (("ซ้าย", "ir_l"), ("ขวา", "ir_r")):
        blocked = _mode_value(ir_blocked[key])
        clear = _mode_value(ir_clear[key])
        if blocked is None or clear is None:
            problems.append("IR{0}: ไม่ได้รับค่า IO ตรวจการต่อสาย".format(side))
        elif blocked == clear:
            problems.append(
                "IR{0}: บังกับไม่บังได้ค่าเท่ากัน ({1}) เซนเซอร์อาจไม่ทำงาน "
                "หรือ pot ตั้งไว้สั้น/ยาวเกินไป".format(side, blocked))
        else:
            print("  IR{0}: บัง={1} โล่ง={2} -> ค่าที่แปลว่าเจอสิ่งกีดขวางคือ {1}"
                  .format(side, blocked, clear))
            if ir_value is None:
                ir_value = blocked
            elif ir_value != blocked:
                problems.append(
                    "IR ซ้ายกับขวาใช้ตรรกะกลับด้านกัน โค้ดรองรับได้ค่าเดียว "
                    "ต้องตั้ง pot หรือสลับสายให้เหมือนกันก่อน")

    if front_mm is None:
        problems.append("ToF หน้า: ไม่ได้รับค่า ตรวจว่าเสียบโมดูลและ TOF_INDEX ถูกตัวหรือไม่")
    if side_mm_left is None or side_mm_right is None:
        problems.append("ToF ข้าง (ผ่าน gimbal): อ่านไม่ได้อย่างน้อย 1 ทิศ ตรวจว่ากำแพง"
                          "ทั้งสองข้างอยู่ในระยะจริงตอนขั้นที่ 1")

    print("\n" + "-" * 62)
    if problems:
        print("  พบปัญหาที่ควรแก้ก่อนลงสนามจริง:")
        for item in problems:
            print("    * {0}".format(item))
        print("-" * 62)

    print("\nคัดลอกบล็อกนี้ไปวางทับใน CONFIG ด้านบนของไฟล์:\n")
    def _show(name, value):
        print("{0:<30}= {1}".format(name, value))

    _show("FRONT_STOP_MM", int(round(front_mm)) if front_mm is not None else "None")
    side_vals = [v for v in (side_mm_left, side_mm_right) if v is not None]
    _show("SIDE_STOP_MM", int(round(sum(side_vals) / len(side_vals))) if side_vals else "None")
    _show("SHARP_LEFT_REF", sharp_ref.get("SHARP_LEFT_REF", "None"))
    _show("SHARP_RIGHT_REF", sharp_ref.get("SHARP_RIGHT_REF", "None"))
    _show("SHARP_LEFT_CLOSER_IS_HIGHER", sharp_polarity.get("SHARP_LEFT_CLOSER_IS_HIGHER", "None"))
    _show("SHARP_RIGHT_CLOSER_IS_HIGHER", sharp_polarity.get("SHARP_RIGHT_CLOSER_IS_HIGHER", "None"))
    _show("IR_TRIGGERED_VALUE", ir_value if ir_value is not None else "None")
    print()


# =====================================================================
# โหมด --sim : ทดสอบตรรกะสำรวจให้ครบ โดยไม่ต้องต่อหุ่น
# =====================================================================
#: list: คู่ช่องที่มีกำแพงกั้นระหว่างกันในเขาวงกตจำลอง (สนาม 4x5 = 20 ช่อง)
#: ออกแบบให้มีทางตัน (เช่น (3,1)) ต้องเดินเข้าไปแล้วถอยกลับมาสำรวจทางอื่น และ
#: มีมุมอับที่แถวบนสุด y=4 ที่เพิ่มเข้ามาใหม่ ทดสอบทั้ง backtrack และ reroute
#: ไปช่องไกล (ตอน stack ว่างแต่ยังสำรวจไม่ครบ) - ไม่มีช่องไหนถูกตัดขาดจริง
SIM_BLOCKED_EDGES = [
    ((0, 0), (1, 0)),
    ((0, 1), (0, 2)),
    ((1, 1), (1, 2)),
    ((2, 0), (2, 1)),
    ((1, 2), (2, 2)),
    ((3, 1), (3, 2)),
    ((0, 3), (0, 4)),
    ((2, 3), (2, 4)),
    ((1, 4), (2, 4)),
]


def _reachable_count(maze, start):
    """int: จำนวนช่องที่ไปถึงได้จาก ``start`` ตามกำแพงจริงทั้งหมดของ ``maze``

    ใช้เป็น "เฉลย" ตอน ``--sim`` เทียบว่าตรรกะสำรวจไปครบทุกช่องที่ควรไปถึง
    ได้จริงหรือไม่ (ต่างจาก ``flood_known`` ที่เดินผ่านได้เฉพาะด้านที่ ``known``
    เพราะที่นี่ ``maze`` คือเขาวงกตความจริงที่กำแพงครบทุกด้านอยู่แล้ว)
    """
    seen = {start}
    queue = deque([start])
    while queue:
        cx, cy = queue.popleft()
        for direction in range(4):
            if maze.has_wall(cx, cy, direction):
                continue
            nb = (cx + DX[direction], cy + DY[direction])
            if maze.in_bounds(*nb) and nb not in seen:
                seen.add(nb)
                queue.append(nb)
    return len(seen)


def run_sim():
    """เดินตรรกะสำรวจเดียวกับของจริง (``ExplorePlanner``) บนเขาวงกตจำลอง

    ใช้ ``Maze``/``ExplorePlanner`` ตัวเดียวกับที่หุ่นใช้จริง ต่างกันแค่แทนที่จะ
    อ่านเซนเซอร์ ก็ไปถามเขาวงกตความจริงตรง ๆ จึงยืนยันได้ว่าตรรกะสำรวจครบถูกต้อง
    ก่อนเอาไปเจอกับความไม่แน่นอนของเซนเซอร์และล้อในสนามจริง

    Returns:
        bool: True เมื่อสำรวจครบทุกช่องที่ไปถึงได้จริง
    """
    truth = Maze(MAZE_W, MAZE_H)
    for cell_a, cell_b in SIM_BLOCKED_EDGES:
        truth.set_wall(cell_a[0], cell_a[1], _edge_direction(cell_a, cell_b), True)
    expected = _reachable_count(truth, START_CELL)

    print("=" * 62)
    print("  โหมดจำลอง - สำรวจให้ครบทุกช่อง (ไม่ต้องต่อหุ่น)")
    print("=" * 62)
    print("\nเขาวงกตความจริง (หุ่นยังไม่รู้):")
    print(truth.render())
    print("ช่องที่ไปถึงได้จริงจาก {0} = {1}/{2}".format(
        START_CELL, expected, MAZE_W * MAZE_H))

    known = Maze(MAZE_W, MAZE_H)
    planner = ExplorePlanner(START_CELL)
    x, y = START_CELL
    heading = START_HEADING
    path = [(x, y)]

    for step in range(MAX_STEPS):
        front = truth.has_wall(x, y, heading)
        right = truth.has_wall(x, y, (heading + 1) % 4)
        left = truth.has_wall(x, y, (heading + 3) % 4)
        known.observe(x, y, heading, front, left, right)
        known.mark_visited(x, y)

        kind, target = planner.next_move(known, x, y, heading)
        print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} | {4} ---"
              .format(step, x, y, DIR_NAMES[heading], kind))

        if kind == "done":
            print("\n[DONE] สำรวจครบทุกช่องที่ไปถึงได้แล้วใน {0} ก้าว".format(step))
            print("เส้นทางที่เดินจริง: {0}".format(
                " -> ".join(str(cell) for cell in path)))
            print("\nแผนที่ที่หุ่นสร้างได้:")
            print(known.render(robot=(x, y, heading), legend=True))
            ok = len(known.visited) == expected
            print("[CHECK] สำรวจแล้ว {0}/{1} ช่องที่ไปถึงได้จริง -> {2}"
                  .format(len(known.visited), expected,
                          "ผ่าน" if ok else "ไม่ผ่าน (มีช่องตกหล่น)"))
            return ok

        next_heading = _edge_direction((x, y), target)
        # ตรวจความถูกต้องของตรรกะ: ทิศที่เลือกต้องไม่มีกำแพงอยู่จริง
        if truth.has_wall(x, y, next_heading):
            print("\n[BUG] เลือกเดินไปทาง {0} ทั้งที่มีกำแพงจริงอยู่"
                  .format(DIR_NAMES[next_heading]))
            return False

        heading = next_heading
        x, y = target
        path.append((x, y))

    print("\n[FAIL] ครบ {0} ก้าวแล้วยังสำรวจไม่ครบ".format(MAX_STEPS))
    return False


# =====================================================================
# MAIN
# =====================================================================
def _parse_cell(text):
    """tuple: แปลง 'x,y' เป็น (int, int) - raise ValueError พร้อมข้อความอ่านง่าย"""
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("'{0}' ไม่ใช่รูปแบบ x,y (เช่น 2,3)".format(text))
    try:
        return int(parts[0].strip()), int(parts[1].strip())
    except ValueError:
        raise ValueError("'{0}' ไม่ใช่รูปแบบ x,y (เช่น 2,3)".format(text))


def _prompt_current_pose(default_cell, default_heading):
    """tuple: ถามผู้ใช้ว่าหุ่นตอนนี้อยู่ช่องไหน หันทางไหนจริง ๆ

    จำเป็นเพราะ odometry รีเซ็ตทุกครั้งที่เชื่อมต่อหุ่นใหม่ - โปรแกรมจำ
    ตำแหน่งข้ามการเชื่อมต่อเองไม่ได้ ต้องให้คนที่วางหุ่นเป็นคนยืนยัน
    ค่าเริ่มต้นคือตำแหน่ง/ทิศที่บันทึกไว้ตอนจบการสำรวจครั้งก่อน (ที่ที่หุ่นน่าจะ
    อยู่จริง ถ้าไม่ได้ยกหุ่นไปวางที่อื่นหลังสำรวจเสร็จ)
    """
    print("\n[GOTO] หุ่นต้องอยู่กลางช่องจริง ๆ ก่อนเริ่ม (โปรแกรมจำตำแหน่งข้าม"
          "การเชื่อมต่อเองไม่ได้ ต้องยืนยันจากคนตอนนี้)")
    text = input("      ตอนนี้หุ่นอยู่ช่องไหน หันทางไหน [Enter = {0} หัน {1}]: "
                 .format(default_cell, DIR_NAMES[default_heading])).strip()
    if not text:
        return default_cell, default_heading
    cell_part, _, heading_part = text.partition(" ")
    cx, cy = _parse_cell(cell_part)
    heading_part = heading_part.strip().upper()
    heading = DIR_NAMES.index(heading_part) if heading_part else default_heading
    return (cx, cy), heading


class _Tee(object):
    """เขียนซ้ำทุกอย่างที่ ``print()``/stdout ส่งออกมา ทั้งจอและไฟล์ log พร้อมกัน

    ``flush()`` ทันทีทุกครั้งที่เขียน (ไม่รอ buffer) เพื่อให้ต่อให้โปรแกรมถูก
    kill กลางทาง (Ctrl+C ค้าง, แครช, ไฟดับ) ก็ยังมีไฟล์ log อยู่จนถึงบรรทัด
    สุดท้ายที่พิมพ์จริง - ไม่ต้องพึ่ง export_run ตอนจบเพียงอย่างเดียวเหมือนเดิม
    (ซึ่งจะไม่ทำงานเลยถ้าโปรแกรมไม่ได้จบทางปกติ)
    """

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            s.write(data)
            s.flush()

    def flush(self):
        for s in self._streams:
            s.flush()


def main():
    parser = argparse.ArgumentParser(
        description="SLAM สำรวจแผนที่ไม่รู้จักให้ครบ สำหรับ RoboMaster EP (Class Work 8)")
    parser.add_argument("--calib", action="store_true",
                        help="วัดค่าเซนเซอร์จริง หุ่นจะไม่ขยับ")
    parser.add_argument("--sim", action="store_true",
                        help="ทดสอบตรรกะสำรวจ โดยไม่ต้องต่อหุ่น")
    parser.add_argument("--goto", metavar="X,Y", default=None,
                        help="ไม่สำรวจใหม่ - โหลดแผนที่เก่า (--map-file) แล้วเดินไปช่องนี้")
    parser.add_argument("--return-start", action="store_true",
                        help="เหมือน --goto แต่ไปที่ START_CELL ของแผนที่ที่โหลดมา")
    parser.add_argument("--map-file", default=None,
                        help="ไฟล์ {0}_state_*.json ที่จะโหลด (ไม่ระบุ = เอาไฟล์ล่าสุดในโฟลเดอร์นี้)"
                            .format(LOG_PREFIX))
    parser.add_argument("--conn", default=CONN_TYPE,
                        choices=["ap", "sta", "rndis"],
                        help="วิธีเชื่อมต่อหุ่น (ค่าเริ่มต้น {0})".format(CONN_TYPE))
    args = parser.parse_args()

    # เปิดไฟล์ log ข้อความ (ทุกอย่างที่ print() ตั้งแต่บรรทัดนี้เป็นต้นไปจะถูก
    # เขียนซ้ำลงไฟล์นี้ด้วย real-time ไม่ต้องรอ export_run ตอนจบ) - ถ้าโปรแกรม
    # ถูก kill กลางทาง (Ctrl+C ค้าง/แครช) จะยังมีไฟล์นี้อยู่ครบถึงบรรทัดสุดท้าย
    # ที่พิมพ์จริง ต่างจาก CSV/map ที่เขียนแค่ตอนจบแบบปกติเท่านั้น
    ts = _timestamp()
    log_path = _log_path("{0}_run_{1}.log".format(LOG_PREFIX, ts))
    log_file = open(log_path, "w", encoding="utf-8")
    real_stdout = sys.stdout
    sys.stdout = _Tee(real_stdout, log_file)
    print("[LOG] ข้อความทั้งหมดตั้งแต่บรรทัดนี้จะถูกบันทึกไว้ที่ {0} ด้วย (real-time)"
          .format(log_path))

    try:
        if args.sim:
            return 0 if run_sim() else 1

        if robot is None:
            print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
            print("        ติดตั้ง SDK ก่อน หรือใช้ --sim เพื่อทดสอบเฉพาะตรรกะ")
            return 1

        goto_mode = args.goto is not None or args.return_start
        if not args.calib:
            require_calibration()

        ep_robot = robot.Robot()
        print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
        ep_robot.initialize(conn_type=args.conn)

        # ToF ติดอยู่บน gimbal - recenter ให้ชี้หน้าตรงตั้งแต่ต้น เป็นจุดอ้างอิง
        # 0 องศาของ point_gimbal() ทุกครั้งต่อจากนี้ (ต่างจากเวอร์ชันแรก เวอร์ชันนี้
        # จะสั่งหมุน gimbal จริงระหว่างภารกิจ เพื่อสแกน ToF 4 ทิศทุกช่อง)
        print("กำลัง recenter gimbal ...")
        ep_robot.gimbal.recenter().wait_for_completed()

        hub = SensorHub(ep_robot)
        success = False
        try:
            hub.start()
            sensorlog_path = _log_path("{0}_sensorlog_{1}.csv".format(LOG_PREFIX, ts))
            hub.attach_sensor_log(sensorlog_path)
            if args.calib:
                run_calibration(hub)
                success = True
            else:
                # initialize() ตั้ง FREE ให้อยู่แล้ว (robot.py reset) แต่สั่งซ้ำให้ชัดเจน
                # ว่าโค้ดนี้ต้องการให้แชสซีขยับอิสระจากกิมบอล
                ep_robot.set_robot_mode(robot.FREE)
                time.sleep(0.5)

                print("กำลังหา gimbal_yaw_sign ...")
                hub.calibrate_gimbal_yaw_sign()

                driver = Driver(ep_robot.chassis, hub)

                if goto_mode:
                    map_path = args.map_file or find_latest_state_file()
                    if map_path is None:
                        print("[ERROR] ไม่พบไฟล์ {0}_state_*.json ในโฟลเดอร์นี้ - ต้องสำรวจ "
                              "(รันแบบปกติไม่ใส่ --goto/--return-start) ให้จบอย่างน้อยหนึ่ง "
                              "ครั้งก่อน ถึงจะมีแผนที่ให้โหมดนี้ใช้".format(LOG_PREFIX))
                    else:
                        print("[GOTO] โหลดแผนที่จาก {0}".format(map_path))
                        state = load_state(map_path)
                        try:
                            target = (state["start_cell"] if args.return_start
                                      else _parse_cell(args.goto))
                        except ValueError as exc:
                            print("[ERROR] --goto {0}".format(exc))
                            target = None
                        if target is not None:
                            cur_cell, cur_heading = _prompt_current_pose(
                                state["end_cell"], state["end_heading"])
                            ok, _, _, _ = run_goto(hub, driver, state["maze"],
                                                   cur_cell[0], cur_cell[1],
                                                   cur_heading, target)
                            success = ok
                else:
                    done, maze, x, y, heading = run_explore(hub, driver)
                    success = done
                    # สำรวจครบแล้ว - สั่งต่อได้ทันทีในการเชื่อมต่อเดียวกันเลย โดยไม่
                    # ต้องโหลดไฟล์ใหม่ (ตำแหน่ง/แผนที่ยัง live อยู่ในหน่วยความจำ)
                    while done:
                        cmd = input(
                            "\n[NEXT] สำรวจครบแล้ว อยู่ช่อง {0} หัน {1}\n"
                            "      พิมพ์ 'start' กลับจุดเริ่มต้น, พิมพ์ 'x,y' ไปช่องนั้น, "
                            "หรือ Enter จบงาน: ".format((x, y), DIR_NAMES[heading])).strip()
                        if not cmd:
                            break
                        try:
                            target = START_CELL if cmd.lower() == "start" else _parse_cell(cmd)
                        except ValueError as exc:
                            print("[NEXT] {0}".format(exc))
                            continue
                        _, x, y, heading = run_goto(hub, driver, maze, x, y, heading, target)
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

    finally:
        sys.stdout = real_stdout
        try:
            log_file.close()
        except Exception:
            pass

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
