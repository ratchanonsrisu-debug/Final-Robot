# -*- coding: utf-8 -*-
"""
gimbal_pid_lab.py — Lab: ใช้ PID เล็ง gimbal ยิง IR ไปยังเป้า 3 เป้า

ลำดับงาน: ยิงเป้าซ้ายสุด -> กลาง -> ขวาสุด -> วนกลับมาซ้ายสุดอีกครั้ง (1,2,3,1)

โครงสร้างเป็น state machine ต่อเป้า เพื่อไม่ให้ PID (เล็งละเอียด) กับคำสั่งเคลื่อนที่หยาบ
แย่งกันสั่ง gimbal ตัวเดียวกันจนสั่น:
    SLEW  -> เคลื่อนที่หยาบไปตำแหน่งโดยประมาณของเป้าถัดไป (moveto แบบ open-loop, ปิดหู PID)
    AIM   -> เปิด PID เล็งละเอียดจากตำแหน่ง marker ในภาพ (x,y) จนนิ่งในโทเลอแรนซ์ต่อเนื่อง
    FIRE  -> หยุด PID แล้วยิง IR
    (จำมุม yaw/pitch ที่ยิงสำเร็จไว้ใน target.learned_* เพื่อให้รอบถัดไปที่กลับมาเป้าเดิม
     SLEW ตรงไปจุดเดิมได้เลย ไม่ต้องเดาใหม่)

การเลือกว่า marker ตัวไหนในเฟรมคือ "เป้าปัจจุบัน" ใช้ตำแหน่งในเฟรม (ใกล้กึ่งกลางที่สุด)
ไม่ใช้ชื่อ/สัญลักษณ์บนป้าย (info) เพราะเป้าบางอันซ้ำรูปกัน (เช่น หัวใจ 2 ป้าย) แยกด้วยชื่อไม่ได้
"""

import csv
import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import cv2

from robomaster import blaster
from robomaster import robot

# =============================================================================
#  การเชื่อมต่อ
# =============================================================================
CONN_TYPE = "ap"      # "ap" = ต่อ Wi-Fi ของหุ่นยนต์โดยตรง, "sta" = หุ่น+PC ต่อ router เดียวกัน

# =============================================================================
#  หน้าต่างแสดงภาพกล้อง debug -- มี crosshair กลางจอ + กรอบ marker ที่เห็น
#  ให้ดูด้วยตาได้เลยว่าตอนยิงป้ายซ้อนกับ crosshair แค่ไหน (ปิดได้ถ้าไม่อยากให้จอเด้งตอนอัดคลิปจริง)
# =============================================================================
DEBUG_SHOW_CAMERA = True

# =============================================================================
#  เรขาคณิตของสนาม (ตามโจทย์: เป้าห่างกัน 0.6m, หุ่นยนต์ห่างเป้ากลาง 1m)
# =============================================================================
TARGET_SPACING_M = 0.6
TARGET_DISTANCE_M = 1.0

# =============================================================================
#  ทิศทาง (+/-) ของแกนควบคุม -- ยังไม่ยืนยันจากของจริง ถ้าลองแล้ว gimbal เล็งสวนทาง
#  (เช่น marker อยู่ขวาแต่ gimbal หมุนซ้าย) ให้สลับเครื่องหมายตัวที่ผิดเป็น -1
# =============================================================================
YAW_SIGN = 1      # +1: marker อยู่ขวาเฟรม (x>0.5) -> หมุนขวา (yaw_speed บวก)
PITCH_SIGN = -1   # -1: marker อยู่ล่างเฟรม (y>0.5) -> ก้มลง (pitch_speed ลบ)

# =============================================================================
#  PID สำหรับ AIM (error เป็นสัดส่วนเฟรม -0.5..0.5, output เป็น deg/s ป้อน drive_speed)
#  ผลทดสอบจริงรอบก่อน (P อย่างเดียว) เจอ steady-state error ค้าง ~1-2% ไม่ไหลเข้ากลางป้ายต่อ
#  (สังเกตจาก error หยุดนิ่งที่ค่าคงที่หลายลูปติดต่อกันก่อนครบ dwell) เลยเติม I มาไล่ error ค้างนี้
#  และเติม D นิดหน่อยกันไม่ให้ I ดันจน overshoot/แกว่ง
# =============================================================================
KP_YAW, KI_YAW, KD_YAW = 80.0, 20.0, 6.0
KP_PITCH, KI_PITCH, KD_PITCH = 80.0, 20.0, 6.0

MAX_YAW_SPEED = 200.0        # deg/s เพดานความเร็ว yaw ตอนเล็ง
MAX_PITCH_SPEED = 150.0      # deg/s เพดานความเร็ว pitch ตอนเล็ง
INTEGRAL_LIMIT = 0.3         # anti-windup: จำกัดค่าสะสม integral (หน่วยเดียวกับ error)

# =============================================================================
#  เกณฑ์ตัดสินว่า "เล็งนิ่งพอจะยิง" และ timeout กันค้าง
#  ป้ายจริงกว้าง 17cm ที่ระยะ 1m -- ครึ่งความกว้างป้าย = 8.5cm ทดสอบจริงที่ 1.5% (~6-7cm)
#  ยังเยื้องได้ถึง 4-5cm บางนัด (ยังไม่เข้ากลางเป๊ะ) เลยบีบลงอีกเป็น 0.8% (~3-4cm) ให้ใกล้กลางกว่าเดิม
# =============================================================================
AIM_TOLERANCE = 0.005        # |error| ต้องต่ำกว่านี้ (สัดส่วนเฟรม) ทั้ง x และ y
AIM_DWELL_S = 0.25           # ต้องนิ่งในโทเลอแรนซ์ต่อเนื่องกี่วินาทีก่อนยิง (กันสั่นผ่าน-เข้าเฉียดๆ)
AIM_TIMEOUT_S = 3.5          # เล็งนานเกินนี้ยังไม่นิ่ง -> ยิงตามสภาพที่ดีที่สุด ณ ตอนนั้น (กันค้างทั้งโปรแกรม)
MARKER_TIMEOUT_S = 0.4       # ไม่มีข้อมูล marker ใหม่นานเกินนี้ -> ถือว่าเป้าหลุดเฟรม หยุดนิ่งรอ

# กล้องมุมกว้างมาก บางจังหวะเห็นเป้าข้างเคียง (เช่น MID) โผล่เข้าเฟรมพร้อมกับเป้าที่กำลังเล็งอยู่
# (LEFT) -- ถ้า marker ใกล้ตำแหน่งล่าสุดที่สุด ยังห่างเกินนี้ (สัดส่วนเฟรม) ถือว่าเป็นเป้าอื่น ไม่ใช่ของเรา
MAX_MARKER_JUMP = 0.10

SLEW_SPEED = 200             # deg/s ความเร็วตอนเคลื่อนที่หยาบ (SLEW)
CONTROL_PERIOD_S = 0.05      # คาบลูปควบคุมตอน AIM (20 Hz)

# =============================================================================
#  ขนาดป้ายเป้าจริง (ซม.) -- ใช้แปลง error ที่เป็น % ของเฟรม เป็นระยะเยื้องจริงหน่วยซม.
#  โดยไม่ต้องรู้ FOV กล้อง (ใช้ขนาด marker ที่กล้องเห็นจริง ณ ตอนยิงมาคำนวณสัดส่วนแทน)
# =============================================================================
TARGET_WIDTH_CM = 17.0
TARGET_HEIGHT_CM = 18.0


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class PID:
    """PID แกนเดียว ใช้แยก instance สำหรับ yaw กับ pitch"""

    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def update(self, error, dt):
        if dt <= 0:
            dt = 1e-3
        self._integral = clamp(self._integral + error * dt,
                                -self.integral_limit, self.integral_limit)
        derivative = (error - self._prev_error) / dt if self._has_prev else 0.0
        self._prev_error = error
        self._has_prev = True
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return clamp(output, -self.output_limit, self.output_limit)


@dataclass
class Target:
    id: str
    guess_yaw: float
    guess_pitch: float = 0.0
    learned_yaw: float = None
    learned_pitch: float = None

    def slew_angle(self):
        yaw = self.learned_yaw if self.learned_yaw is not None else self.guess_yaw
        pitch = self.learned_pitch if self.learned_pitch is not None else self.guess_pitch
        return yaw, pitch


class VisionState:
    """เก็บผลตรวจจับ marker ล่าสุดจาก callback แบบ thread-safe"""

    def __init__(self):
        self._lock = threading.Lock()
        self._markers = []
        self._last_update_ts = 0.0

    def update(self, markers):
        with self._lock:
            self._markers = markers
            self._last_update_ts = time.monotonic()

    def snapshot(self):
        with self._lock:
            return list(self._markers), self._last_update_ts


class GimbalState:
    """เก็บมุม gimbal ล่าสุดจาก sub_angle callback แบบ thread-safe"""

    def __init__(self):
        self._lock = threading.Lock()
        self._pitch_angle = 0.0
        self._yaw_angle = 0.0

    def update(self, pitch_angle, yaw_angle):
        with self._lock:
            self._pitch_angle = pitch_angle
            self._yaw_angle = yaw_angle

    def snapshot(self):
        with self._lock:
            return self._pitch_angle, self._yaw_angle


class ResponseLogger:
    """log (เวลา, phase, มุม, error, คำสั่ง) ต่อแถว สำหรับ plot time response ทำรายงาน"""

    def __init__(self, path):
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "t_s", "phase", "target_id", "yaw_angle_deg", "pitch_angle_deg",
            "marker_x", "marker_y", "marker_w", "marker_h", "yaw_error", "pitch_error",
            "yaw_speed_cmd", "pitch_speed_cmd",
        ])
        self._t0 = time.monotonic()

    def log(self, phase, target_id, yaw_angle, pitch_angle,
            marker_x="", marker_y="", marker_w="", marker_h="", yaw_error="", pitch_error="",
            yaw_speed_cmd="", pitch_speed_cmd=""):
        t = time.monotonic() - self._t0
        self._writer.writerow([t, phase, target_id, yaw_angle, pitch_angle,
                                marker_x, marker_y, marker_w, marker_h, yaw_error, pitch_error,
                                yaw_speed_cmd, pitch_speed_cmd])
        self._file.flush()

    def close(self):
        self._file.close()


def select_target_marker(markers, reference_x=0.5, max_jump=MAX_MARKER_JUMP):
    """เลือก marker ที่ x ใกล้ "ตำแหน่งอ้างอิง" ที่สุด แทนการอิงชื่อ/สัญลักษณ์บนป้าย

    reference_x ปกติคือตำแหน่งของเป้าที่เพิ่งล็อกไว้เฟรมก่อนหน้า (ไม่ใช่ 0.5 กลางจอตายตัว)
    เพื่อไม่ให้หลุดไปเลือก marker ของเป้าข้างเคียงที่กล้องมุมกว้างเห็นโผล่เข้ามาในเฟรมเดียวกัน
    ตอนเป้าจริงหลุดเฟรมไปแวบเดียว (พบจริงจากการทดสอบ: เป้า MID โผล่มาแทนตอนเล็งเป้า LEFT)
    ถ้า marker ที่ใกล้ที่สุดยังห่างจาก reference_x เกิน max_jump ถือว่าไม่ใช่เป้าเดิม ข้ามเฟรมนั้นไปเลย
    """
    if not markers:
        return None
    best = min(markers, key=lambda m: abs(m[0] - reference_x))
    if abs(best[0] - reference_x) > max_jump:
        return None
    return best


def debug_camera_view(ep_camera, vision_state, stop_event):
    """เปิดหน้าต่างภาพกล้องสด วาด crosshair กลางจอ + กรอบ marker ที่เห็น
    ให้ตรวจสอบด้วยตาได้ว่าตอนยิงป้ายอยู่ตรงกลางจอแค่ไหน ทำงานแยก thread ไม่ยุ่งกับจังหวะ PID
    """
    while not stop_event.is_set():
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        if img is None:
            continue
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS,
                        markerSize=30, thickness=2)

        markers, _ = vision_state.snapshot()
        for x, y, mw, mh, info in markers:
            pt1 = (int((x - mw / 2) * w), int((y - mh / 2) * h))
            pt2 = (int((x + mw / 2) * w), int((y + mh / 2) * h))
            cv2.rectangle(img, pt1, pt2, (255, 255, 255), 2)
            cv2.putText(img, str(info), (pt1[0], max(0, pt1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("Gimbal PID debug (กด q เพื่อปิดหน้าต่างนี้เฉยๆ ไม่หยุดโปรแกรม)", img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()


def slew_to_target(ep_gimbal, target, gimbal_state, logger):
    """เคลื่อนที่หยาบแบบ open-loop ไปตำแหน่งโดยประมาณของเป้า (ไม่ยุ่งกับ PID/vision เลย)"""
    yaw, pitch = target.slew_angle()
    pitch_angle, yaw_angle = gimbal_state.snapshot()
    logger.log("SLEW_START", target.id, yaw_angle, pitch_angle)

    ep_gimbal.moveto(pitch=pitch, yaw=yaw,
                      pitch_speed=SLEW_SPEED, yaw_speed=SLEW_SPEED).wait_for_completed()

    pitch_angle, yaw_angle = gimbal_state.snapshot()
    logger.log("SLEW_END", target.id, yaw_angle, pitch_angle)


def aim_and_fire(ep_gimbal, ep_blaster, vision_state, gimbal_state, target, logger):
    """เปิด PID เล็งละเอียดจนนิ่งในโทเลอแรนซ์ต่อเนื่อง แล้วหยุด PID ก่อนยิง IR"""
    yaw_pid = PID(KP_YAW, KI_YAW, KD_YAW, MAX_YAW_SPEED, INTEGRAL_LIMIT)
    pitch_pid = PID(KP_PITCH, KI_PITCH, KD_PITCH, MAX_PITCH_SPEED, INTEGRAL_LIMIT)

    start_t = time.monotonic()
    last_t = start_t
    in_tolerance_since = None
    last_yaw_err, last_pitch_err = None, None
    last_marker_w, last_marker_h = None, None
    reference_x = 0.5   # ตำแหน่งอ้างอิงสำหรับเลือก marker -- อัปเดตเป็นตำแหน่งล่าสุดที่ล็อกได้จริงทุกเฟรม

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        markers, marker_ts = vision_state.snapshot()
        pitch_angle, yaw_angle = gimbal_state.snapshot()
        marker_fresh = (now - marker_ts) < MARKER_TIMEOUT_S
        marker = select_target_marker(markers, reference_x=reference_x) if marker_fresh else None

        if marker is None:
            # เป้าหลุดเฟรม/ยังไม่มีข้อมูลใหม่/หรือมีแต่ marker ของเป้าอื่นที่กระโดดเข้ามา -> หยุดนิ่งรอ
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            in_tolerance_since = None
            logger.log("AIM_LOST", target.id, yaw_angle, pitch_angle)
        else:
            x, y, w, h = marker[0], marker[1], marker[2], marker[3]
            reference_x = x
            yaw_err = x - 0.5
            pitch_err = y - 0.5
            last_yaw_err, last_pitch_err = yaw_err, pitch_err
            last_marker_w, last_marker_h = w, h

            yaw_speed = YAW_SIGN * yaw_pid.update(yaw_err, dt)
            pitch_speed = PITCH_SIGN * pitch_pid.update(pitch_err, dt)
            ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)

            logger.log("AIM", target.id, yaw_angle, pitch_angle,
                       marker_x=x, marker_y=y, marker_w=w, marker_h=h,
                       yaw_error=yaw_err, pitch_error=pitch_err,
                       yaw_speed_cmd=yaw_speed, pitch_speed_cmd=pitch_speed)

            within_tol = abs(yaw_err) < AIM_TOLERANCE and abs(pitch_err) < AIM_TOLERANCE
            if within_tol:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                elif now - in_tolerance_since >= AIM_DWELL_S:
                    break  # นิ่งพอแล้ว ออกไปยิง
            else:
                in_tolerance_since = None

        if now - start_t > AIM_TIMEOUT_S:
            print("[WARN] target {0}: aim timeout, firing at best-effort aim".format(target.id))
            break

        time.sleep(CONTROL_PERIOD_S)

    ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
    time.sleep(0.05)  # กันแรงเฉื่อยตกค้างก่อนยิง

    if last_yaw_err is None:
        print("-> ยิง {0}: ไม่มีข้อมูล marker เลยตอนจะยิง (เล็งแบบ best-effort)".format(target.id))
    else:
        miss_x_cm = last_yaw_err * TARGET_WIDTH_CM / last_marker_w
        miss_y_cm = last_pitch_err * TARGET_HEIGHT_CM / last_marker_h
        print("-> ยิง {0}: เยื้องจากกลางป้าย x={1:+.1%} ({2:+.1f} cm) "
              "y={3:+.1%} ({4:+.1f} cm)".format(
                  target.id, last_yaw_err, miss_x_cm, last_pitch_err, miss_y_cm))

    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)

    pitch_angle, yaw_angle = gimbal_state.snapshot()
    target.learned_yaw = yaw_angle
    target.learned_pitch = pitch_angle
    logger.log("FIRE", target.id, yaw_angle, pitch_angle)


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type=CONN_TYPE)

    vision_state = VisionState()
    gimbal_state = GimbalState()

    def on_detect_marker(marker_info):
        vision_state.update(list(marker_info))

    def on_gimbal_angle(angle_info):
        pitch_angle, yaw_angle, _pitch_ground, _yaw_ground = angle_info
        gimbal_state.update(pitch_angle, yaw_angle)

    log_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(
        log_dir, "gimbal_pid_log_{0}.csv".format(datetime.now().strftime("%Y%m%d_%H%M%S")))
    logger = ResponseLogger(log_path)

    # มุม yaw โดยประมาณจากเป้ากลางไปเป้าซ้าย/ขวา คำนวณจากเรขาคณิต (สปีดตั้งต้นก่อนมี learned angle)
    half_angle_deg = math.degrees(math.atan2(TARGET_SPACING_M / 2.0, TARGET_DISTANCE_M))
    target_left = Target(id="LEFT", guess_yaw=-half_angle_deg)
    target_mid = Target(id="MID", guess_yaw=0.0)
    target_right = Target(id="RIGHT", guess_yaw=half_angle_deg)
    sequence = [target_left, target_mid, target_right, target_left]

    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    debug_stop = threading.Event()
    debug_thread = None

    try:
        ep_camera.start_video_stream(display=False)
        ep_gimbal.sub_angle(freq=50, callback=on_gimbal_angle)
        ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

        if DEBUG_SHOW_CAMERA:
            debug_thread = threading.Thread(
                target=debug_camera_view, args=(ep_camera, vision_state, debug_stop),
                daemon=True)
            debug_thread.start()

        print("recenter...")
        ep_gimbal.recenter().wait_for_completed()
        time.sleep(0.3)  # รอ callback แรกของ angle/vision เข้ามาก่อนเริ่ม loop

        mission_start = time.monotonic()
        for target in sequence:
            print("-> target {0}".format(target.id))
            slew_to_target(ep_gimbal, target, gimbal_state, logger)
            aim_and_fire(ep_gimbal, ep_blaster, vision_state, gimbal_state, target, logger)

        elapsed = time.monotonic() - mission_start
        print("เสร็จภารกิจใน {0:.2f} วินาที".format(elapsed))
        print("log: {0}".format(log_path))

    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        debug_stop.set()
        if debug_thread is not None:
            debug_thread.join(timeout=1.0)
        ep_vision.unsub_detect_info(name="marker")
        ep_gimbal.unsub_angle()
        ep_camera.stop_video_stream()
        logger.close()
        ep_robot.close()


if __name__ == "__main__":
    main()
