"""Move Accuracy Test - ทดสอบความแม่นยำของการเดิน/หมุน ล้วน ๆ ไม่พึ่งเซนเซอร์ใดเลย

จุดประสงค์
----------
แยกทดสอบเฉพาะความแม่นของ ``chassis.move()`` (ระยะเดินหน้า + มุมหมุน) ออกจาก
ตรรกะ SLAM/สำรวจทั้งหมด เพื่อเอาไว้ไล่ปรับ ``MOVE_XY_SPEED``/``MOVE_Z_SPEED``
(หรือชดเชย bias ที่วัดได้) ก่อนเอาไปใช้กับ ``SLAM.py`` จริง

**ไม่ใช้เซนเซอร์กันชนใด ๆ เลย** ไม่มี ToF, ไม่มี IR 45 องศา, ไม่มี Sharp -
ทั้งเดินหน้าและหมุนใช้ ``chassis.move()`` ปิดลูปตำแหน่งจากเฟิร์มแวร์เอง (แม่น
กว่าไล่คุมด้วย ``drive_speed`` จากภายนอก) แต่ ``wait_for_completed()`` ของ
คำสั่งเดินหน้าไม่น่าเชื่อถือ (พบจาก log ว่าแทบไม่เคยได้สัญญาณ "เสร็จ" เลย)
จึงอ่าน odometry จริงมาเช็คเองเสมอ ไม่เชื่อ True/False ที่ SDK คืนมาตรง ๆ -
และพบว่าเดินหน้าทันทีหลังหมุน (ไม่ว่าหมุนทิศไหน) มักสไลด์ด้านข้างจริง ไม่ใช่
แค่ขาดระยะตรง ๆ ท่าเดินหน้าจึงเป็นแบบ "เดินหลักแล้ววัดตำแหน่งจริง แก้ส่วนที่
เหลือซ้ำ" (ไม่เกิน ``FORWARD_MAX_ATTEMPTS`` ครั้ง) ไม่ใช่สั่งทีเดียวจบแบบ dead-
reckoning ล้วน ๆ อีกต่อไป (ดู ``move_forward`` docstring) - ก่อน/หลังทุกท่า
(เดิน/หมุน) จะรอจนกว่า yaw/ตำแหน่งจะนิ่งจริง ๆ (ดู ``_wait_until_still`` -
โพลเช็คจากข้อมูลจริงแทนการเดา sleep คงที่ เพราะพบว่าเวลาคงที่เดิมไม่พอเสมอ
ไปที่จะให้แรงหมุนค้างจากการเลี้ยวสงบตัวจริง กลายเป็นสไลด์ตอนเดินหน้าต่อ -
**ไม่สั่ง drive_speed แทรกด้วย** เพราะเคยลองแล้วทำให้ ``chassis.move()`` ถัด
ไปถูกเมิน ไม่ขยับเลย) ต้องเว้นพื้นที่ว่างรอบหุ่นเองก่อนรัน เพราะโปรแกรมจะ
**ไม่หยุดให้เอง**ถ้ามีของกีดขวาง

ลำดับท่าเริ่มต้น (ทำซ้ำได้ด้วย ``--repeat``)
--------------------------------------------
    เดินหน้า 1 ก้าว (STEP_M) -> หมุนขวา 90 องศา (TURN_DEG) -> เดินหน้า 1 ก้าว

ชุดที่วัดได้มาจาก ``chassis.sub_position()``/``sub_attitude()`` (odometry+IMU
ของหุ่นเอง) เอาไว้เทียบแนวโน้ม/ดูว่าหุ่นเชื่อตัวเองว่าไปถึงไหน - **ไม่ใช่ค่า
ความจริงภาคพื้นดิน** ความแม่นจริงต้องวัดด้วยตลับเมตร/ไม้โปรแทรกเตอร์ที่พื้น
สนามเอง แล้วเทียบกับ STEP_M/TURN_DEG ที่ตั้งไว้

วิธีใช้
------
    python move_accuracy_test.py                 วิ่ง 1 รอบ (เดิน-หมุนขวา-เดิน)
    python move_accuracy_test.py --repeat 3       ทำซ้ำแพทเทิร์นเดิม 3 รอบติดกัน
    python move_accuracy_test.py --step-m 0.5     ปรับระยะก้าวเป็น 0.5 ม.
    python move_accuracy_test.py --turn-deg 90    หมุนซ้ายแทน (+ = ซ้าย, ยืนยันจากการมองด้วยตาจริงแล้ว)
    python move_accuracy_test.py --xy-speed 0.8 --z-speed 60   ไล่ปรับความเร็ว

หลังรันเสร็จ ไฟล์ log (CSV) จะอยู่ในโฟลเดอร์เดียวกับไฟล์นี้เสมอ
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError as exc:
    robot = None
    ROBOT_IMPORT_ERROR = exc
else:
    ROBOT_IMPORT_ERROR = None


# =====================================================================
# CONFIG - แก้ค่าทั้งหมดที่นี่ที่เดียว (หรือส่งผ่าน --flag ตอนรันแทนก็ได้)
# =====================================================================
STEP_M = 0.60                   # ระยะเดินหน้าต่อก้าว (เมตร)
TURN_DEG = -90.0                 # มุมหมุนต่อครั้ง (+ = ซ้าย, - = ขวา) ตรงกับ
                                 # เอกสาร examples/02_chassis/01_move.py ที่
                                 # SLAM.py เคยอ้างไว้ทุกประการ - เคยลองสลับ
                                 # เครื่องหมายเป็น +90 ไปแล้วรอบหนึ่ง (ตอนนั้น
                                 # เข้าใจผิดว่าเอกสารกลับด้าน เพราะไปอ่าน log
                                 # จากรันที่หุ่นแทบไม่ขยับเลยจากบั๊ก drive_speed
                                 # ชนกับ move() - ค่าที่วัดได้ตอนนั้นเลยไม่มี
                                 # ความหมายอะไรทั้งนั้น) แต่มองด้วยตาจริงแล้ว
                                 # ยืนยันว่า +90=ซ้าย -90=ขวา ตามเอกสารเป๊ะ
                                 # จึงสลับกลับมาเป็น -90 (ขวา) ตามเดิม
MOVE_XY_SPEED = 0.5              # m/s - ค่าต่ำสุดที่ checker ของ chassis.move() ยอมรับจริง [0.5, 2.0]
MOVE_Z_SPEED = 45.0              # deg/s ตอนหมุน
MOVE_TIMEOUT_MARGIN_S = 3.0      # กัน wait_for_completed() ค้างตลอดกาล
STILL_CHECK_DT = 0.05            # คาบโพลตอนเช็คว่านิ่งจริงหรือยัง
STILL_POS_TOL_M = 0.002          # ตำแหน่งต้องขยับไม่เกินนี้ต่อรอบเช็คถึงจะนับว่านิ่ง
STILL_YAW_TOL_DEG = 0.3          # yaw ต้องขยับไม่เกินนี้ต่อรอบเช็คถึงจะนับว่านิ่ง
STILL_CONFIRM_COUNT = 4          # ต้องนิ่งติดกันกี่รอบเช็คถึงจะเชื่อว่านิ่งจริง
STILL_MAX_WAIT_S = 2.0           # เพดานเวลารอ กันค้างถ้าไม่มีทางนิ่งจริง ๆ สักที
FORWARD_TOLERANCE_M = 0.01       # ยอมรับคลาดเคลื่อนเท่านี้ถึงถือว่าเดินถึงเป้าแล้ว
FORWARD_MAX_ATTEMPTS = 3         # 1 เดินหลักตามระยะที่สั่ง + สูงสุด 2 รอบแก้ไข
                                 # ตำแหน่งจริงจาก odometry (ดู move_forward)
CONN_TYPE = "ap"                 # ap / sta / rndis
LOG_PREFIX = "movetest"

#: str: โฟลเดอร์ที่ไฟล์นี้อยู่ - log บันทึกไว้ที่นี่เสมอไม่ว่าจะรันจาก cwd ไหน
LOG_DIR = os.path.dirname(os.path.abspath(__file__))


def _log_path(name):
    return os.path.join(LOG_DIR, name)


def _timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


# =====================================================================
# ตัวติดตาม pose (odometry + IMU ของหุ่นเอง - ไม่ใช่ ground truth จริง)
# =====================================================================
class PoseTracker(object):
    """สมัคร sub_attitude/sub_position เก็บ pose ล่าสุดไว้ให้อ่านได้ทุกเมื่อ

    ไม่แตะ ToF/IR/Sharp/gimbal เลยตามที่ตกลง - มีแค่ IMU (yaw) กับ odometry
    ล้อ (x, y) ซึ่งเป็นของที่ ``chassis.move()`` ใช้ปิดลูปอยู่แล้วภายในตัวเอง
    """

    def __init__(self, chassis):
        self._chassis = chassis
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._att_t = 0.0
        self._pos_t = 0.0

    def _on_attitude(self, info):
        with self._lock:
            self._yaw = info[0]
            self._att_t = time.time()

    def _on_position(self, info):
        with self._lock:
            self._pos_x, self._pos_y = info[0], info[1]
            self._pos_t = time.time()

    def start(self):
        self._chassis.sub_attitude(freq=20, callback=self._on_attitude)
        self._chassis.sub_position(freq=20, callback=self._on_position)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            with self._lock:
                got = self._att_t > 0 and self._pos_t > 0
            if got:
                return
            time.sleep(0.05)
        raise RuntimeError(
            "ไม่ได้รับข้อมูล attitude/position จากหุ่น - ตรวจการเชื่อมต่อ")

    def stop(self):
        for fn in (self._chassis.unsub_attitude, self._chassis.unsub_position):
            try:
                fn()
            except Exception:                           # noqa: BLE001
                pass

    def pose(self):
        """tuple: (x, y, yaw) ล่าสุด"""
        with self._lock:
            return self._pos_x, self._pos_y, self._yaw


def wrap_deg(angle):
    """บีบมุมให้อยู่ในช่วง [-180, 180)"""
    return (angle + 180.0) % 360.0 - 180.0


# =====================================================================
# ท่าเดิน/หมุนพื้นฐาน
# =====================================================================
def _wait_until_still(tracker, label=""):
    """bool: รอจนกว่า yaw/ตำแหน่งจะ "นิ่งจริง" ติดกันหลายรอบ แทนการเดา sleep คงที่

    ก่อนหน้านี้เคยใช้ ``time.sleep(SETTLE_S)`` คงที่ (0.5 วิ) ก่อน/หลังทุกท่า
    เดิมเคยลองสั่ง ``chassis.drive_speed(0,0,0)`` แทรกด้วย แต่ผลคือแย่กว่าเดิม
    มาก (หุ่น**ไม่ขยับเลยสักท่า** ทั้งเดินและหมุน - น่าจะเพราะ ``drive_speed``
    กับ ``chassis.move()`` เป็นคนละโหมดคุมของเฟิร์มแวร์ สลับถี่เกินไปแล้วเฟิร์ม
    แวร์งงเมินคำสั่งถัดไป) จึงตัดออกไป เหลือแค่รอเฉย ๆ

    แต่พบว่าเดินหน้าทันทีหลังหมุน (หมุนไปทางไหนก็สไลด์ไปทางนั้น) ยังเกิดอยู่
    แม้จะรอแบบคงที่ไปแล้วก็ตาม - สมมติฐานที่น่าจะเป็นไปได้ที่สุด: การหมุนของ
    ``chassis.move(z=)`` อาจมี overshoot/แกว่งเล็กน้อยตอนเข้าใกล้มุมเป้าหมาย
    (พฤติกรรมทั่วไปของ PID ที่หน่วงไม่พอ) เวลาคงที่ตายตัวที่เดาไว้ (0.5 วิ)
    อาจไม่พอเสมอไปที่จะให้แกว่งนั้นสงบตัวจริง เหลือความเร็วเชิงมุมเล็ก ๆ
    ค้างอยู่ตอนเริ่มเดินหน้า กลายเป็นสไลด์ไปทางเดียวกับที่เพิ่งหมุน

    เช็คจากข้อมูลจริงแทน: โพล pose ซ้ำทุก ``STILL_CHECK_DT`` แล้วถือว่านิ่ง
    ก็ต่อเมื่อทั้ง yaw และตำแหน่งไม่ขยับเกิน tolerance ติดต่อกัน
    ``STILL_CONFIRM_COUNT`` รอบ (ไม่ใช่ ToF/IR/Sharp - ยังเป็น odometry/IMU
    ของหุ่นเองเหมือนเดิมทุกประการ)

    Returns:
        bool: True ถ้านิ่งจริงก่อนหมดเวลา False ถ้าหมดเวลาแล้วยังไม่นิ่ง
    """
    still_count = 0
    prev = tracker.pose()
    deadline = time.time() + STILL_MAX_WAIT_S
    while time.time() < deadline:
        time.sleep(STILL_CHECK_DT)
        cur = tracker.pose()
        dpos = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        dyaw = abs(wrap_deg(cur[2] - prev[2]))
        prev = cur
        if dpos <= STILL_POS_TOL_M and dyaw <= STILL_YAW_TOL_DEG:
            still_count += 1
            if still_count >= STILL_CONFIRM_COUNT:
                return True
        else:
            still_count = 0
    print("      [WARN] รอนิ่ง{0}ไม่ทันใน {1}s - ไปต่อเลย (yaw/ตำแหน่งยังขยับอยู่)"
          .format(" ({0})".format(label) if label else "", STILL_MAX_WAIT_S))
    return False


def move_forward(chassis, tracker, distance_m, xy_speed):
    """เดินไปยัง "จุดเป้าหมายจริง" (พิกัดโลกที่คำนวณจากทิศตอนเริ่มก่อนขยับ)
    ด้วย ``chassis.move()`` หลายก้อนต่อกันถ้าจำเป็น - ไม่ใช่สั่ง ``x=distance_m``
    ทีเดียวจบแบบเดิมอีกต่อไป

    พบจากการทดสอบจริงว่าเดินหน้าทันทีหลังหมุน (**ไม่ว่าจะหมุนไปทิศไหน**) มัก
    มีการสไลด์ด้านข้างจริง ไม่ใช่แค่ขาดระยะตรง ๆ (ล้อ mecanum ยังปรับตัวจาก
    แรงหมุนไม่ทันตอนเปลี่ยนมาเป็นเดินหน้า) เดินครั้งเดียวแบบ dead-reckoning
    (สั่งแค่ ``x=`` เฉย ๆ) แก้ปัญหานี้ไม่ได้เพราะไม่รู้เลยว่าสไลด์ไปเท่าไหร่ -
    ต้องวัดตำแหน่งจริงหลังคำสั่งจบ แล้วยิงก้อนเล็ก ๆ แก้ส่วนที่เหลือทั้งแนว
    หน้า-หลังและซ้าย-ขวาซ้ำ จนกว่าจะเข้าใกล้เป้าพอ (ยังไม่ใช้ ToF/IR/Sharp -
    ใช้ odometry ของหุ่นเองเท่านั้น เหมือนเดิม)

    ไม่รู้ทิศทางจริงของแกน ``y`` ใน ``chassis.move()`` ล่วงหน้า (แกน ``z`` เคย
    มีปัญหาเรื่องเครื่องหมายมาแล้ว) จึงเช็คเองทุกรอบว่าการแก้ไขครั้งก่อนทำให้
    ระยะห่างจากเป้าดีขึ้นจริงไหม ถ้าแย่ลงจะสลับเครื่องหมาย lateral ทันทีในรอบ
    ถัดไป - จำกัดไว้ไม่เกิน ``FORWARD_MAX_ATTEMPTS`` ครั้งกันวนไม่จบ

    ``wait_for_completed()`` ของ ``chassis.move()`` ไม่น่าเชื่อถือ (พบจาก log
    ว่าแทบไม่เคยได้สัญญาณ "เสร็จ" เลย) จึงไม่เอาค่า True/False ที่มันคืนมา
    ตัดสินผลตรง ๆ เลย - รอจนคำสั่งจบ (ไม่ว่าจะ ack หรือ timeout) แล้ววัด
    odometry จริงเองทุกครั้งแทน

    Returns:
        tuple: (ok, traveled_m) - ``traveled_m`` คือระยะจากจุดเริ่มถึงจุดสุดท้าย
    """
    start_x, start_y, start_yaw = tracker.pose()
    yaw0 = math.radians(start_yaw)
    target_x = start_x + distance_m * math.cos(yaw0)
    target_y = start_y + distance_m * math.sin(yaw0)

    ok = False
    lateral_sign = 1.0
    prev_remaining = None
    for attempt in range(FORWARD_MAX_ATTEMPTS):
        x, y, yaw = tracker.pose()
        err_x, err_y = target_x - x, target_y - y
        remaining = math.hypot(err_x, err_y)
        if remaining <= FORWARD_TOLERANCE_M:
            ok = True
            break
        if prev_remaining is not None and remaining > prev_remaining:
            lateral_sign *= -1.0
            print("      [CORRECT] รอบที่แล้วทำให้ระยะห่างเป้าแย่ลง สลับเครื่องหมาย lateral")
        prev_remaining = remaining

        yaw_rad = math.radians(yaw)
        fwd = err_x * math.cos(yaw_rad) + err_y * math.sin(yaw_rad)
        lat = (-err_x * math.sin(yaw_rad) + err_y * math.cos(yaw_rad)) * lateral_sign
        timeout = (remaining / xy_speed) * 2.0 + MOVE_TIMEOUT_MARGIN_S
        print("      [{0}] เหลืออีก {1:.3f}m -> สั่ง move(x={2:+.3f}, y={3:+.3f})"
              .format("เดินหลัก" if attempt == 0 else "แก้ไข", remaining, fwd, lat))
        action = chassis.move(x=fwd, y=lat, xy_speed=xy_speed)
        action.wait_for_completed(timeout=timeout)
        _wait_until_still(tracker, "เดินหน้า")

    x, y, _ = tracker.pose()
    traveled = math.hypot(x - start_x, y - start_y)
    return ok, traveled


def turn(chassis, delta_deg, z_speed):
    timeout = (abs(delta_deg) / z_speed) * 2.0 + MOVE_TIMEOUT_MARGIN_S
    action = chassis.move(z=delta_deg, z_speed=z_speed)
    return action.wait_for_completed(timeout=timeout)


def build_plan(step_m, turn_deg, repeat):
    """list: แผนท่าเดิน ``[("forward", ค่า), ("turn", ค่า), ...]``

    แพทเทิร์นเริ่มต้น (repeat=1): เดินหน้า 1 ก้าว -> หมุน 1 ครั้ง -> เดินหน้า
    1 ก้าว ตามที่ตกลงกันไว้ ถ้า ``repeat`` > 1 จะทำซ้ำทั้งแพทเทิร์นนี้ต่อกัน
    ไปเรื่อย ๆ (เอาไว้ดู bias สะสม เห็นง่ายกว่าดูรอบเดียว)
    """
    plan = []
    for _ in range(repeat):
        plan.append(("forward", step_m))
        plan.append(("turn", turn_deg))
        plan.append(("forward", step_m))
    return plan


def run_plan(chassis, tracker, plan, xy_speed, z_speed, writer, log_file):
    """เดินตามแผนทีละท่า log pose ก่อน/หลังทุกท่า

    Returns:
        bool: True ถ้าทุกท่า wait_for_completed() สำเร็จ (ไม่ timeout)
    """
    all_ok = True
    for i, (kind, value) in enumerate(plan):
        _wait_until_still(tracker, "ก่อนท่า {0}".format(i))  # ต้องนิ่งจริง ไม่ใช่แค่เดาเวลา
        before = tracker.pose()
        t0 = time.time()

        if kind == "forward":
            print("[{0:02d}] เดินหน้า {1:.3f} m ...".format(i, value))
            ok, _ = move_forward(chassis, tracker, value, xy_speed)
        else:
            print("[{0:02d}] หมุน {1:+.1f} องศา ({2}) ..."
                  .format(i, value, "ซ้าย" if value > 0 else "ขวา"))
            ok = turn(chassis, value, z_speed)

        elapsed = time.time() - t0
        _wait_until_still(tracker, "หลังท่า {0}".format(i))  # ต้องนิ่งจริงก่อนอ่าน pose
        after = tracker.pose()

        dx, dy = after[0] - before[0], after[1] - before[1]
        moved_m = math.hypot(dx, dy)
        yaw_delta = wrap_deg(after[2] - before[2])

        if kind == "forward":
            error = moved_m - value
            print("      odometry เดินได้ {0:.4f} m (สั่ง {1:.3f} m, error {2:+.4f} m) "
                  "{3}".format(moved_m, value, error, "" if ok else "[TIMEOUT]"))
        else:
            error = wrap_deg(yaw_delta - value)
            print("      IMU หมุนได้ {0:+.2f} องศา (สั่ง {1:+.1f} องศา, error {2:+.2f} "
                  "องศา) {3}".format(yaw_delta, value, error, "" if ok else "[TIMEOUT]"))

        if not ok:
            all_ok = False
            print("      [WARN] {0} - หมดเวลาก่อนถึงเป้าหมาย"
                  .format("odometry ไม่ถึงระยะที่สั่ง" if kind == "forward"
                          else "wait_for_completed() timeout"))

        writer.writerow([
            i, kind, value, round(before[0], 4), round(before[1], 4),
            round(before[2], 2), round(after[0], 4), round(after[1], 4),
            round(after[2], 2), round(moved_m, 4), round(yaw_delta, 2),
            round(error, 4), int(ok), round(elapsed, 3)])
        log_file.flush()

    return all_ok


def main():
    parser = argparse.ArgumentParser(
        description="ทดสอบความแม่นยำของการเดิน/หมุนล้วน ๆ ไม่ใช้เซนเซอร์กันชนใด ๆ")
    parser.add_argument("--step-m", type=float, default=STEP_M,
                        help="ระยะเดินหน้าต่อก้าว หน่วยเมตร (ค่าเริ่มต้น {0})".format(STEP_M))
    parser.add_argument("--turn-deg", type=float, default=TURN_DEG,
                        help="มุมหมุนต่อครั้ง + ซ้าย/- ขวา (ค่าเริ่มต้น {0}, ยืนยันทิศจากการมองด้วยตาจริง)"
                            .format(TURN_DEG))
    parser.add_argument("--repeat", type=int, default=1,
                        help="ทำซ้ำแพทเทิร์น (เดิน-หมุน-เดิน) กี่รอบติดกัน (ค่าเริ่มต้น 1)")
    parser.add_argument("--xy-speed", type=float, default=MOVE_XY_SPEED,
                        help="ความเร็วเดินหน้า m/s (ค่าเริ่มต้น {0}, ต้องอยู่ใน [0.5, 2.0] "
                            "เพราะเป็นพารามิเตอร์ของ chassis.move())".format(MOVE_XY_SPEED))
    parser.add_argument("--z-speed", type=float, default=MOVE_Z_SPEED,
                        help="ความเร็วหมุน deg/s (ค่าเริ่มต้น {0})".format(MOVE_Z_SPEED))
    parser.add_argument("--conn", default=CONN_TYPE, choices=["ap", "sta", "rndis"],
                        help="วิธีเชื่อมต่อหุ่น (ค่าเริ่มต้น {0})".format(CONN_TYPE))
    args = parser.parse_args()

    if robot is None:
        print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
        return 1

    plan = build_plan(args.step_m, args.turn_deg, args.repeat)

    print("=" * 62)
    print("  MOVE ACCURACY TEST - ไม่ใช้เซนเซอร์กันชนใด ๆ เลย (ไม่มี ToF/IR/Sharp)")
    print("  แพทเทิร์น: เดินหน้า {0:.2f}m -> หมุน {1:+.1f}องศา -> เดินหน้า {0:.2f}m "
          "x{2} รอบ ({3} ท่ารวม)".format(args.step_m, args.turn_deg, args.repeat, len(plan)))
    print("  ความเร็ว: xy={0} m/s, z={1} deg/s".format(args.xy_speed, args.z_speed))
    print("=" * 62)
    print("[WARN] โปรแกรมนี้ไม่หยุดให้เองถ้ามีของกีดขวางเลย (ไม่ได้ subscribe ToF/IR)")
    print("       ต้องเว้นพื้นที่ว่างรอบหุ่นเองให้พอก่อนเริ่ม")
    input("พร้อมแล้วกด Enter เพื่อเริ่ม (Ctrl+C เพื่อยกเลิก)...")

    ts = _timestamp()
    log_csv_path = _log_path("{0}_{1}.csv".format(LOG_PREFIX, ts))
    log_file = open(log_csv_path, "w", newline="", encoding="utf-8")
    writer = csv.writer(log_file)
    writer.writerow(["leg", "kind", "commanded", "before_x", "before_y", "before_yaw",
                      "after_x", "after_y", "after_yaw", "moved_m", "yaw_delta_deg",
                      "error", "move_ok", "elapsed_s"])
    log_file.flush()
    print("[LOG] บันทึกผลลงไปที่ {0}".format(log_csv_path))

    ep_robot = robot.Robot()
    print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
    ep_robot.initialize(conn_type=args.conn)
    chassis = ep_robot.chassis
    tracker = PoseTracker(chassis)
    all_ok = False

    try:
        tracker.start()
        start_pose = tracker.pose()
        print("[POSE] เริ่มที่ x={0:.4f} y={1:.4f} yaw={2:.2f}".format(*start_pose))

        all_ok = run_plan(chassis, tracker, plan, args.xy_speed, args.z_speed,
                          writer, log_file)

        end_pose = tracker.pose()
        print("\n" + "=" * 62)
        print("  สรุปผล (จาก odometry/IMU ของหุ่นเอง - ไม่ใช่ ground truth จริง)")
        print("  เริ่ม  x={0:.4f} y={1:.4f} yaw={2:.2f}".format(*start_pose))
        print("  จบ    x={0:.4f} y={1:.4f} yaw={2:.2f}".format(*end_pose))
        print("  ทุกท่าถึงเป้าหมาย (ระยะ/มุม) ทันเวลาหมด: {0}".format(all_ok))
        print("=" * 62)
        print("[NEXT] วัดตำแหน่ง/มุมจริงของหุ่นบนพื้นด้วยตลับเมตร/โปรแทรกเตอร์ "
              "เทียบกับที่คำนวณไว้ (step={0:.2f}m, turn={1:+.1f}องศา) แล้วปรับ "
              "--xy-speed/--z-speed หรือ error ที่เห็นใน log กลับไปชดเชยใน "
              "SLAM.py ต่อ".format(args.step_m, args.turn_deg))
    except KeyboardInterrupt:
        print("\n[STOP] ผู้ใช้สั่งหยุด")
    finally:
        try:
            chassis.drive_speed(x=0, y=0, z=0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))
        tracker.stop()
        log_file.close()
        try:
            ep_robot.close()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ปิดการเชื่อมต่อไม่สำเร็จ: {0}".format(exc))

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
