import csv
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from robomaster import robot, blaster
#cd "d:\ROBOT-Final\Final-Robot\PIDGimball"
#python detectcolor.py fire        (ยิง IR ปลอดภัย)
#python detectcolor.py fire_live   (ยิงกระสุนเจลจริง ต้องพิมพ์ยืนยันก่อน)
#python detectcolor.py colorscan


# โฟลเดอร์เก็บไฟล์ภาพทั้งหมด = โฟลเดอร์นี้ (PIDGimball) เสมอ ไม่ว่าจะรันจากที่ไหน
SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
ACCURACY_LOG_PATH = os.path.join(SAVE_DIR, "fire_accuracy_log.csv")
COLORSCAN_LOG_PATH = os.path.join(SAVE_DIR, "colorscan_log.csv")

# --------------------------------------------------
# โหมดปลอดภัย: เลือกจากคำสั่งตอนรัน ไม่ใช่แก้ค่าคงที่ในไฟล์นี้แล้ว (กันลืมเปลี่ยนกลับ)
#   python detectcolor.py fire       -> ล็อกเป้าสำเร็จแล้วยิงด้วย "แสง IR" เท่านั้น ไม่มีกระสุนเจลออกมา ปลอดภัยเสมอ
#   python detectcolor.py fire_live  -> ล็อกเป้าสำเร็จแล้วยิงกระสุนเจลจริง (WATER_FIRE) ต้องพิมพ์ยืนยันในหน้าจอก่อนเริ่มทุกครั้ง
# --------------------------------------------------


# --------------------------------------------------
# คลาสสำหรับคำนวณ PID Controller
# --------------------------------------------------
class PIDController:
    def __init__(self, kp, ki, kd, limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_limit, self.max_limit = limits

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.01  # ป้องกันการหารด้วยศูนย์

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term (สะสมค่า Error)
        self.integral += error * dt
        i_term = self.ki * self.integral

        # 3. Derivative term (อัตราการเปลี่ยนแปลงของ Error)
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # รวมค่าความเร็ว
        output = p_term + i_term + d_term

        # จำกัดความเร็วสูงสุด/ต่ำสุด (Output Clamping)
        output = max(self.min_limit, min(self.max_limit, output))

        # บันทึกค่าไว้ใช้ในรอบถัดไป
        self.last_error = error
        self.last_time = now

        return output

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


# --------------------------------------------------
# ตั้งค่าระบบ PID สำหรับ Yaw และ Pitch
# --------------------------------------------------
# ลด limits ลงจากเดิม (-180,180)/(-120,120) เพราะ loop อ่านภาพ+ประมวลผลต่อรอบช้ากว่าจะหมุนจริง
# ถ้า error ยังเยอะแล้วสั่งความเร็วสูงมาก ระหว่างรอเฟรมถัดไป gimbal อาจแกว่งเลยจุดที่ควรจะหยุดไปไกล
# (สงสัยว่าเป็นสาเหตุที่ยิงป้ายซ้ายสุดพลาด - ยังหมุนไปไม่ถึงตำแหน่งจริงแต่ error ที่อ่านได้ดันต่ำ)
pid_yaw = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-70, 70))
pid_pitch = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-50, 50))

# --------------------------------------------------
# ค่าความแม่นยำตอนเล็ง+ยิง (ปรับได้ตามผลเทสจริง)
# --------------------------------------------------
# log เดิมพบว่า err ตอนล็อกติดอยู่ที่ราวๆ 0.029-0.030 ทุกครั้ง (คือพอต่ำกว่า 0.03 ปุ๊บก็ยิงเลยทันที
# ไม่ได้เล็งต่อจนแม่นจริง) เลยลด threshold ลง + เพิ่มเงื่อนไขต้องนิ่งในเกณฑ์ต่อเนื่องกี่เฟรมก่อนค่อยยิงจริง
# 0.015 (~19px, ~3.3ซม.ที่ระยะ 100ซม.) ยังหลวมกว่าค่า AIM_OFFSET_X ที่กำลัง calibrate กันอยู่ (1-5ซม.) เกือบเท่าตัว
# ทำให้แต่ละนัดที่ "ล็อกสำเร็จ" สุ่มตำแหน่งได้เองในช่วงนี้อยู่แล้ว ผลทดสอบเลยดูไม่นิ่ง/กลับไปกลับมาทั้งที่ offset ไม่เปลี่ยนทิศ
# บีบแน่นลงอีก + เพิ่มจำนวนเฟรมนิ่งก่อนยิง ให้กิมบอลหยุดสนิทจริงๆ (กันโมเมนตัมตกค้างจากการเบรกกะทันหัน) ก่อนลั่นไก
LOCK_ERROR_THRESHOLD = 0.008     # เดิม 0.015 (~19px/~3.3ซม.) บีบเหลือ ~10px/~1.7ซม. ที่ระยะ 100ซม.
LOCK_DWELL_FRAMES = 5            # เดิม 3 เพิ่มเป็น 5 ให้มีเวลาสั่ง speed=0 ค้างไว้นานขึ้นก่อนยิง (กันโมเมนตัมตกค้าง)

# จุดเล็งแนวตั้งบนตัวป้าย: 0 = ขอบบนสุด, 0.5 = กึ่งกลาง, 1 = ขอบล่างสุด
# เล็งใกล้ขอบบนป้ายแทนกึ่งกลาง เพราะยิงโดนด้านบนทำให้ป้ายล้มง่ายกว่า
AIM_POINT_Y_RATIO = 0.2

# ชดเชยผลต่างระหว่างตำแหน่งกล้องกับป้อมปืน (parallax) แนวนอน แยกค่าตาม "ลำดับป้าย" (0=ซ้ายสุด, 1=ถัดมา, 2=ขวาสุด ...)
# เพราะ parallax จริงเป็นระยะทางกายภาพคงที่ระหว่างกล้องกับลำกล้อง แต่พอแปลงเป็นสัดส่วนพิกเซลที่ต้องชดเชยแล้ว "ไม่คงที่" ตามมุมที่ป้อมปืนหันไป
# ยิ่งหันไปทางซ้าย/ขวามาก มุมเบี่ยงยิ่งมากกว่าตอนป้ายอยู่กลางจอ ค่าคงที่ตัวเดียวแบบเดิมเลยเอาไม่อยู่ทุกป้าย (ทดสอบแล้วป้ายซ้ายสุดคลาดหนักสุด)
# ค่าบวก = สั่งเล็ง/ยิงเยื้องไปทางขวาเพิ่มจากที่กล้องเห็น, ค่าลบ = เยื้องไปทางซ้ายเพิ่ม (สัดส่วนความกว้างเฟรม 0-1)
# วิธี calibrate: ยิงป้ายนั้นแล้วดูว่ากระสุน/แสงคลาดไปทางซ้าย(-)/ขวา(+) กี่ px เทียบความกว้างภาพ หารด้วยความกว้างเฟรมได้ค่าที่ต้องปรับ
# แก้ค่าของป้ายตำแหน่งนั้นทีละนิดจนแม่น (ทำแยกทีละตำแหน่ง อย่าใช้ค่าเดียวกันทุกป้ายอีก)
# ทดสอบยิงจริงรอบที่ 1 (offset ทุกป้าย=0): เข้ากรอบครบทุกป้ายแล้ว แต่ยังคลาดขวาอยู่ ไม่เท่ากันแต่ละป้าย
# ซ้ายสุด คลาดขวานิดหน่อย / กลาง คลาดขวาชัดเจน / ขวาสุด คลาดขวาหนักสุด -> ทุกป้ายต้องดันซ้ายเพิ่ม (ค่าลบ) ปริมาณไม่เท่ากัน
# ทดสอบยิงจริงรอบที่ 2 (0/-0.025/-0.045): ซ้ายสุดแม่นแล้ว (ไม่แก้ต่อ) แต่กลาง/ขวาสุด overshoot ไปทางซ้ายเกินแล้ว
# ทดสอบยิงจริงรอบที่ 3 (-0.01/-0.012/-0.022): กลาง/ขวาสุด ยัง overshoot ซ้ายอยู่ (ขวาสุด overshoot หนักกว่ากลาง)
# -> ความสัมพันธ์มุม<->offset ไม่เป็นเชิงเส้นกับลำดับป้าย/ระยะห่างจริง เลยต้อง bisect แยกทีละป้ายต่อไปเรื่อยๆ (ไม่ใช้สัดส่วนคงที่)
# ทดสอบรอบ 4 (~0/~0): กลับกลายเป็นคลาดซ้ายแทน (ตรงข้ามรอบ 1 ที่ offset=0 แล้วคลาดขวา) ทั้งที่ offset แทบไม่เปลี่ยน
# -> ตัวแปรกวนคือ LOCK_ERROR_THRESHOLD เดิม 0.015 หลวมกว่า offset ที่กำลัง bisect กันเกือบเท่าตัว (ดูคอมเมนต์บรรทัด 92)
# ทำให้แต่ละนัดสุ่มตำแหน่งเองอยู่แล้วไม่ว่า offset จะเป็นเท่าไหร่ -> บีบ threshold+เพิ่ม dwell ก่อน (แก้ไปแล้วบรรทัด 92-93)
# แล้วค่อยทดสอบค่าปัจจุบัน (เกือบ 0 ทั้งคู่) ซ้ำอีกรอบก่อน ถ้านิ่งขึ้น/สม่ำเสมอขึ้นแล้วค่อย bisect ต่อจากค่าจริงที่วัดได้
# ทดสอบรอบ 5 (หลังบีบ threshold): ผลนิ่งขึ้นแล้ว เข้ากรอบครบ เกือบตรงกลางทุกป้าย เหลือแค่ต้องขยับขวาอีกนิด
# ขวาสุดต้องขยับมากสุด / กลางปานกลาง / ซ้ายสุดน้อยสุด -> fine-tune ทีละนิด (ก้าวเล็กกว่ารอบก่อนๆ เพราะใกล้จุดแล้ว)
# ทดสอบรอบ 6: ซ้ายสุด (-0.007) กับกลาง (+0.006) โอเคแล้ว ไม่แก้ต่อ / ขวาสุด (+0.015) แรงไป เกือบหลุดกรอบ (ขวาเกิน)
# -> bisect ขวาสุดระหว่าง 0 กับ +0.015
AIM_OFFSET_X_BY_INDEX = {
    0: -0.007,  # ป้ายซ้ายสุด (ลำดับยิงที่ 1) - โอเคแล้วจากรอบ 6 ไม่ต้องแก้
    1: 0.006,   # ป้ายถัดมา (ลำดับยิงที่ 2) - แม่นแล้วจากรอบ 6 ไม่ต้องแก้
    2: 0.008,   # ป้ายขวาสุด/ป้ายที่ 3 (ลำดับยิงที่ 3) - รอบ 6 ขวาเกินไปที่ +0.015 bisect ระหว่าง 0 กับ +0.015
}
AIM_OFFSET_X_DEFAULT = 0.0  # เผื่อมีป้ายมากกว่าที่ระบุไว้ในดิกต์ด้านบน

# ชดเชยกระสุนตก (projectile drop) เฉพาะตอนยิงกระสุนเจลจริง (โหมด fire_live, safe_mode=False) เท่านั้น
# IR (โหมด fire, safe_mode=True) เป็นแสง ไม่มีการตกจากแรงโน้มถ่วง เลยไม่ต้องชดเชย
# ค่าเป็นสัดส่วนความสูงเฟรม (normalize 0-1) ยิ่งมาก = เล็งสูงขึ้นชดเชยเยอะขึ้น
# วิธี calibrate: ยิงกระสุนเจลจริง 1 นัดที่ป้ายตรงกึ่งกลางจอปกติ (ค่านี้ = 0 ก่อน) แล้วดูว่ากระสุนไปโดนต่ำกว่าจุดเล็งกี่ % ของความสูงภาพ
# แล้วเอาค่านั้นมาใส่ตรงนี้ (ยังไม่ได้ calibrate จริง ตั้งไว้ 0 ก่อน ห้ามเดาเองมั่วๆ)
PROJECTILE_DROP_COMPENSATION = 0.0

ep_gimbal = None
ep_blaster = None
ep_camera = None

# --------------------------------------------------
# Log ต่อเนื่องของมุม gimbal (yaw/pitch) เทียบเวลา ไว้ plot time response ตอนเขียนรายงาน
# (fire_accuracy_log.csv เดิมมีแค่ 1 แถวต่อ 1 นัดที่ยิงสำเร็จ ไม่พอวาดกราฟ time response แบบต่อเนื่อง)
# --------------------------------------------------
TIME_RESPONSE_LOG_PATH = os.path.join(SAVE_DIR, "gimbal_time_response_log.csv")
_gimbal_angle = {"pitch": 0.0, "yaw": 0.0}  # อัปเดตจาก callback ของ ep_gimbal.sub_angle()


def _on_gimbal_angle(angle_info):
    """callback รับมุม gimbal ปัจจุบัน (หน่วยองศา) ความถี่ตามที่ sub_angle(freq=...) กำหนด"""
    pitch_angle, yaw_angle, _pitch_ground_angle, _yaw_ground_angle = angle_info
    _gimbal_angle["pitch"] = pitch_angle
    _gimbal_angle["yaw"] = yaw_angle


def log_time_response(elapsed_sec, target_idx, color, err_x, err_y, fired):
    """บันทึก 1 แถวต่อเฟรม: เวลา, มุม yaw/pitch ปัจจุบัน, error, และธงว่ายิงตรงเฟรมนี้มั้ย
    เอาไว้ plot time response ของมุม gimbal เทียบเวลา (เหมือนกราฟในรายงาน PID เดิม)"""
    _append_csv_row(
        TIME_RESPONSE_LOG_PATH,
        ["elapsed_sec", "target_idx", "color", "yaw_angle_deg", "pitch_angle_deg", "err_x", "err_y", "fired"],
        [f"{elapsed_sec:.3f}", target_idx, color,
         f"{_gimbal_angle['yaw']:.2f}", f"{_gimbal_angle['pitch']:.2f}",
         f"{err_x:.4f}" if err_x is not None else "-", f"{err_y:.4f}" if err_y is not None else "-",
         int(fired)],
    )


def save_frame(prefix, frame=None):
    """ถ่าย/บันทึกภาพปัจจุบันจากกล้อง ลงโฟลเดอร์ PIDGimball พร้อม timestamp"""
    global ep_camera

    if frame is None:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
    if frame is None:
        print("[WARN] ไม่สามารถอ่านภาพจากกล้องเพื่อบันทึกได้")
        return None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.jpg"
    filepath = os.path.join(SAVE_DIR, filename)
    cv2.imwrite(filepath, frame)
    print(f"[บันทึกภาพ] -> {filename}")
    return frame


def _append_csv_row(path, header, row):
    is_new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(header)
        writer.writerow(row)


def draw_overlay(frame, bbox, locked=False, info_lines=None):
    """วาด crosshair กึ่งกลางจอ + กรอบ/เส้นจำลองแนวเล็ง (เหมือนเลเซอร์) ไปยังป้าย + ข้อความบอกค่าต่างๆ"""
    fh, fw = frame.shape[:2]
    cx, cy = fw // 2, fh // 2

    cv2.drawMarker(frame, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS,
                    markerSize=24, thickness=2)

    if bbox is not None:
        x, y, w, h = bbox
        x1, y1 = int((x - w / 2) * fw), int((y - h / 2) * fh)
        x2, y2 = int((x + w / 2) * fw), int((y + h / 2) * fh)
        tx, ty = int(x * fw), int(y * fh)
        color = (0, 0, 255) if locked else (0, 255, 0)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        # เส้นจำลองแนวยิง/เลเซอร์ จากกึ่งกลางจอไปยังจุดกึ่งกลางป้าย
        cv2.line(frame, (cx, cy), (tx, ty), color, 1)
        cv2.circle(frame, (tx, ty), 4, color, -1)

    y0 = 30
    for line in (info_lines or []):
        cv2.putText(frame, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y0 += 26

    return frame


# --------------------------------------------------
# โหมด "fire": คุม Gimbal เองด้วย WASD จนเจอป้ายที่จะยิงครบในกรอบ ROI แล้วกด Enter/Space
# เพื่อเริ่มยิงอัตโนมัติทีละป้าย เรียงจากซ้ายสุด -> ขวาสุด (ใช้ detect_signs() ตัวเดียวกับ colorscan
# ไม่ใช้ vision.MARKER แล้ว เพราะป้ายเล็กแค่ 6x9/9x6 ซม. ระบบจดจำ marker ของ DJI อ่านสัญลักษณ์ไม่ออก)
# --------------------------------------------------
def log_accuracy(target_id, err_x, err_y, frame, yaw_speed, pitch_speed, time_to_lock, bbox_px=None):
    """บันทึกค่าความแม่นยำตอนล็อกเป้าสำเร็จ (ระยะเบี่ยงเบนจากกึ่งกลางจอ เทียบเท่าจุดที่เลเซอร์/กระสุนจะไปโดน)
    เก็บ bbox_px (ตำแหน่งจริงบนจอตอนยิง) ไว้ด้วย เพื่อเช็คย้อนหลังว่ายิงโดนจุดไหนบนภาพจริงๆ (เช่นเจอว่ายิงโดนของที่ติดกับตัวหุ่นเองแทนป้ายจริง)"""
    fh, fw = frame.shape[:2]
    err_x_px = err_x * fw
    err_y_px = err_y * fh
    pixel_distance = (err_x_px ** 2 + err_y_px ** 2) ** 0.5

    _append_csv_row(
        ACCURACY_LOG_PATH,
        ["timestamp", "target_id", "err_x", "err_y", "err_x_px", "err_y_px",
         "pixel_distance", "yaw_speed", "pitch_speed", "time_to_lock_sec", "bbox_px", "frame_w", "frame_h"],
        [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), target_id,
         f"{err_x:.4f}", f"{err_y:.4f}", f"{err_x_px:.1f}", f"{err_y_px:.1f}",
         f"{pixel_distance:.1f}", f"{yaw_speed:.1f}", f"{pitch_speed:.1f}",
         f"{time_to_lock:.2f}", str(bbox_px) if bbox_px is not None else "-", fw, fh],
    )
    print(f"[LOG] ความแม่นยำ -> เบี่ยงจากกึ่งกลาง {pixel_distance:.1f}px "
          f"(x={err_x_px:.1f}px, y={err_y_px:.1f}px), bbox_px={bbox_px}, ใช้เวลา {time_to_lock:.2f}s -> {os.path.basename(ACCURACY_LOG_PATH)}")


def run_fire_mode(ep_robot, safe_mode):
    """คุม Gimbal เองด้วย WASD จนเจอป้ายครบในกรอบ ROI แล้วกด Enter/Space
    -> รีเซ็ต PID + ล็อกลำดับป้ายซ้าย->ขวา แล้วเล็ง+ยิงอัตโนมัติทีละป้ายเรียงซ้าย->ขวาจนครบ
    safe_mode=True (โหมด "fire") ยิง IR เท่านั้นเสมอ
    safe_mode=False (โหมด "fire_live") ยิงกระสุนเจลจริง เรียกจาก main() หลังผู้ใช้พิมพ์ยืนยันในหน้าจอแล้วเท่านั้น"""
    global ep_gimbal, ep_blaster, ep_camera

    reset_sign_tracker()
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    # เปิด log มุม gimbal ต่อเนื่องเทียบเวลา (สำหรับ plot time response) เริ่มนับเวลา 0 ตอนนี้เลย
    _gimbal_angle["pitch"] = 0.0
    _gimbal_angle["yaw"] = 0.0
    try:
        ep_gimbal.sub_angle(freq=20, callback=_on_gimbal_angle)
    except Exception as e:
        print(f"[WARN] subscribe มุม gimbal ไม่สำเร็จ ({e}) จะไม่มี log time response ในรอบนี้")
    run_start_time = time.time()

    window_name = "RoboMaster Camera - fire"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        # พยายามดันหน้าต่างขึ้นมาบนสุด+รับโฟกัส กันปัญหาคีย์บอร์ดไม่เข้าหน้าต่าง
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
    except cv2.error:
        pass

    print("=" * 60)
    print("โหมด fire")
    print("คุม Gimbal เอง: W=เงยขึ้น, S=ก้มลง, A=หันซ้าย, D=หันขวา")
    print("เล็งให้ป้ายที่จะยิงอยู่ในกรอบเหลือง (ROI) ให้ครบ แล้วกด Enter หรือ Space")
    print("-> รีเซ็ต PID + ล็อกลำดับป้ายซ้าย->ขวา แล้วเล็ง+ยิงอัตโนมัติทีละป้ายจนครบ จบโปรแกรมเอง")
    print(f"safe_mode={safe_mode} ({'ยิง IR เท่านั้น ปลอดภัย ไม่มีกระสุนเจลออก' if safe_mode else 'ยิงกระสุนเจลจริง!!'})")
    print("สำคัญ: ต้องคลิกที่หน้าต่างภาพ 'RoboMaster Camera - fire' ให้โฟกัสก่อน ถึงจะกดคีย์ได้")
    print("กด ESC หรือ q เพื่อยกเลิก")
    print("=" * 60)

    last_yaw_key_time = 0.0
    last_pitch_key_time = 0.0
    yaw_dir = 0
    pitch_dir = 0
    live_signs = []

    targets = None      # list ของสีป้าย เรียงซ้าย->ขวา หลังกด Enter ล็อกลำดับแล้ว
    target_idx = 0
    target_start_time = 0.0
    lock_dwell_count = 0  # นับเฟรมที่อยู่ในเกณฑ์ล็อกต่อเนื่อง ก่อนค่อยยิงจริง

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            now = time.time()

            if frame is not None:
                live_signs = detect_signs(frame)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                print(f"[DEBUG] key pressed: {key}")  # เช็คว่าคีย์เข้าถึงหน้าต่างภาพจริงมั้ย (ลบทิ้งได้ถ้าไม่ใช้แล้ว)

            # A/D คุมหันซ้าย-ขวา (yaw), W/S คุมเงย-ก้ม (pitch) ใช้ตอนยังไม่ล็อกลำดับยิง
            if targets is None:
                if key in (ord('a'), ord('A')):
                    yaw_dir, last_yaw_key_time = -1, now
                elif key in (ord('d'), ord('D')):
                    yaw_dir, last_yaw_key_time = 1, now
                if now - last_yaw_key_time > MANUAL_KEY_HOLD_TIMEOUT:
                    yaw_dir = 0

                if key in (ord('w'), ord('W')):
                    pitch_dir, last_pitch_key_time = 1, now
                elif key in (ord('s'), ord('S')):
                    pitch_dir, last_pitch_key_time = -1, now
                if now - last_pitch_key_time > MANUAL_KEY_HOLD_TIMEOUT:
                    pitch_dir = 0

                ep_gimbal.drive_speed(pitch_speed=pitch_dir * MANUAL_TURN_SPEED,
                                       yaw_speed=yaw_dir * MANUAL_TURN_SPEED)

                if key in (13, 32):  # Enter/Space -> รีเซ็ต PID + ล็อกลำดับป้าย เรียงซ้าย->ขวา แล้วเริ่มยิง
                    if live_signs:
                        ordered = sorted(live_signs, key=lambda s: s["bbox_px"][0] + s["bbox_px"][2] / 2.0)
                        # เก็บ track_id ของป้ายใบที่ตั้งใจจะยิงไว้เป๊ะๆ (ไม่ใช่แค่สี) กันสับสนถ้ามีสีซ้ำ/ป้ายปลอมสีเดียวกันโผล่มาทีหลัง
                        targets = [(s["color"], s.get("track_id")) for s in ordered]
                        target_idx = 0
                        target_start_time = now
                        lock_dwell_count = 0
                        pid_yaw.reset()
                        pid_pitch.reset()
                        names = ', '.join(f"{c}#{tid}" for c, tid in targets)
                        print(f"\n>> รีเซ็ต PID แล้ว ล็อกลำดับยิง {len(targets)} ป้าย ซ้าย->ขวา: {names}")
                    else:
                        print("[WARN] ยังไม่เจอป้ายในกรอบ ROI เลย ลองขยับกล้องก่อนกด Enter")

                info_lines = ["WASD=aim  ENTER/SPACE=start firing  ESC/Q=cancel",
                              f"signs found: {len(live_signs)}"]

            else:
                # เฟสยิงอัตโนมัติ: หาป้ายใบเดิม (track_id เป๊ะๆ) ที่ต้องการในบรรดาป้ายที่เห็นตอนนี้ แล้ว PID ล็อก+ยิง
                if target_idx >= len(targets):
                    print("\n>> ยิงครบทุกป้ายแล้ว จบโปรแกรม")
                    break

                want_color, want_track_id = targets[target_idx]
                # เจาะจงยึด track_id เดิมก่อน ถ้าป้ายนั้นหลุดหายไปนานจนถูกลืม (เกิน MAX_STABLE_MISSES) ค่อย fallback ไปหาด้วยสีแทน
                match = next((s for s in live_signs if s.get("track_id") == want_track_id), None)
                if match is None:
                    match = next((s for s in live_signs if s["color"] == want_color), None)
                info_lines = [f"target {target_idx + 1}/{len(targets)}: {want_color}#{want_track_id}"]

                if match is None or frame is None:
                    ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                    lock_dwell_count = 0
                    info_lines.append("searching...")
                    log_time_response(now - run_start_time, target_idx, want_color, None, None, fired=False)
                else:
                    fh, fw = frame.shape[:2]
                    x, y, w, h = match["bbox_px"]
                    cx_n = (x + w / 2.0) / fw
                    # เล็งจุดใกล้ขอบบนป้าย (ไม่ใช่กึ่งกลาง) ให้ล้มง่ายกว่า + ชดเชยกระสุนตกถ้ายิงจริง
                    aim_y_n = (y + h * AIM_POINT_Y_RATIO) / fh
                    if not safe_mode:
                        aim_y_n -= PROJECTILE_DROP_COMPENSATION
                    # ชดเชย parallax กล้อง-ป้อมปืนแนวนอน แยกค่าตามลำดับป้าย (ซ้าย/กลาง/ขวา คลาดไม่เท่ากัน)
                    aim_offset_x = AIM_OFFSET_X_BY_INDEX.get(target_idx, AIM_OFFSET_X_DEFAULT)
                    err_x = (cx_n - 0.5) + aim_offset_x
                    err_y = 0.5 - aim_y_n
                    info_lines.append(f"err_x={err_x:.3f} err_y={err_y:.3f} offset_x={aim_offset_x:+.3f} "
                                       f"dwell={lock_dwell_count}/{LOCK_DWELL_FRAMES}")
                    # log ทุกเฟรมลง terminal ด้วย (ไม่ใช่แค่บนจอ) ไว้ไล่ดูย้อนหลังว่าตอนยิงจริงเล็งไปถึงไหนแล้ว
                    print(f"[AIM] target_idx={target_idx} {want_color}#{want_track_id} err_x={err_x:+.3f} err_y={err_y:+.3f} "
                          f"offset_x={aim_offset_x:+.3f} dwell={lock_dwell_count}/{LOCK_DWELL_FRAMES}")

                    log_time_response(now - run_start_time, target_idx, want_color, err_x, err_y, fired=False)

                    yaw_speed = pid_yaw.compute(err_x)
                    pitch_speed = pid_pitch.compute(err_y)

                    if abs(err_x) < LOCK_ERROR_THRESHOLD and abs(err_y) < LOCK_ERROR_THRESHOLD:
                        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                        lock_dwell_count += 1
                    else:
                        lock_dwell_count = 0
                        ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)

                    if lock_dwell_count >= LOCK_DWELL_FRAMES:
                        time_to_lock = now - target_start_time

                        fire_type = blaster.INFRARED_FIRE if safe_mode else blaster.WATER_FIRE
                        print(f"\n>> ล็อกป้าย {want_color} สำเร็จด้วย PID! -> ยิง! (fire_type={fire_type}"
                              f"{' ปลอดภัย ไม่มีกระสุนเจลออก' if safe_mode else ''})")
                        ep_blaster.fire(fire_type=fire_type, times=1)
                        log_time_response(time.time() - run_start_time, target_idx, want_color, err_x, err_y, fired=True)

                        locked_frame = save_frame(f"fire_{want_color}")
                        if locked_frame is not None:
                            log_accuracy(want_color, err_x, err_y, locked_frame, yaw_speed, pitch_speed, time_to_lock,
                                         bbox_px=match["bbox_px"])

                        pid_yaw.reset()
                        pid_pitch.reset()
                        lock_dwell_count = 0
                        target_idx += 1
                        target_start_time = time.time()
                        info_lines.append("LOCKED! FIRED!")

            if frame is not None:
                y0 = 30
                for line in info_lines:
                    cv2.putText(frame, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    y0 += 26
                draw_signs_overlay(frame, live_signs)
                if targets is not None and 0 <= target_idx < len(targets):
                    hl_color, hl_track_id = targets[target_idx]
                    match = next((s for s in live_signs if s.get("track_id") == hl_track_id), None)
                    if match is None:
                        match = next((s for s in live_signs if s["color"] == hl_color), None)
                    if match is not None:
                        fh, fw = frame.shape[:2]
                        x, y, w, h = match["bbox_px"]
                        cx, cy = fw // 2, fh // 2
                        tx, ty = x + w // 2, y + h // 2
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                        cv2.line(frame, (cx, cy), (tx, ty), (0, 0, 255), 2)
                cv2.imshow(window_name, frame)

            if key == 27 or key == ord('q'):
                print("\nยกเลิกโหมด fire")
                break
    except KeyboardInterrupt:
        print("\nหยุดโหมด fire")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        try:
            ep_gimbal.unsub_angle()
        except Exception:
            pass
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        print(f"log time response -> {os.path.basename(TIME_RESPONSE_LOG_PATH)}")


# --------------------------------------------------
# โหมด "colorscan": คุม Gimbal เองด้วย WASD + หาป้ายสี่เหลี่ยม 4 สีในเฟรมทั้งหมด
# (ไม่ใช้ vision.MARKER แล้ว เพราะป้ายสีเฉยๆ ที่ไม่มีสัญลักษณ์ marker ของ DJI จะไม่ถูกตรวจจับ
#  เปลี่ยนมาหา "ก้อนสี" รูปสี่เหลี่ยมในภาพเองด้วย contour + ประมาณขนาดจริงเป็น ซม. เทียบกับขนาดที่รู้อยู่แล้ว)
# --------------------------------------------------
# ชื่อสี key เป็นภาษาอังกฤษล้วน เพราะ cv2.putText วาดตัวอักษรไทยไม่ได้ (จะขึ้นเป็น "?")
COLOR_RANGES = {
    "RED":    {"th": "แดง",         "ranges": [((0, 90, 60), (8, 255, 255)), ((170, 90, 60), (180, 255, 255))]},
    "ORANGE": {"th": "ส้ม",         "ranges": [((9, 90, 60), (20, 255, 255))]},
    "YELLOW": {"th": "เหลือง",      "ranges": [((21, 150, 100), (33, 255, 255))]},  # S/V สูงขึ้นมาก กันผนังสีเหลืองซีดๆ (ป้ายจริงเข้มกว่าผนังเยอะ)
    "GREEN":  {"th": "เขียว",       "ranges": [((36, 70, 25), (85, 255, 255))]},  # S>=70 กันพื้นกระเบื้องจางๆ, V ลดเหลือ 25 ให้จับเขียวโดนเงา/มืดได้ (S ที่สูงยังกันของจางๆ อยู่)
    "BLUE":   {"th": "ฟ้า/น้ำเงิน",  "ranges": [((86, 60, 40), (135, 255, 255))]},
    "PURPLE": {"th": "ม่วง",        "ranges": [((136, 60, 40), (169, 255, 255))]},
    "WHITE":  {"th": "ขาว",         "ranges": [((0, 0, 200), (180, 40, 255))]},
    "BLACK":  {"th": "ดำ",          "ranges": [((0, 0, 0), (180, 255, 45))]},
}

# ตอนนี้มีป้ายวงกลมสีเหลืองปนอยู่ด้วย เลยต้องหาเหลืองด้วย (ระบบตัดสินรูปทรงเองจาก contour ไม่ได้ผูกสีตายตัว)
TARGET_SIGN_COLORS = ["RED", "GREEN", "BLUE", "YELLOW"]

# บังคับว่าบางสีต้องเป็นรูปทรงที่กำหนดเท่านั้น ไม่ตรงตัดทิ้งเลย (ไม่ต้องรอให้ HSV แยกของปลอมออกได้สมบูรณ์แบบ)
# ชุดทดสอบตอนนี้มีแค่ป้ายวงกลมที่เป็นสีเหลือง ไม่มีป้ายสี่เหลี่ยมสีเหลือง เลยล็อกว่าเหลืองต้องเป็นวงกลมเท่านั้น
# กันของปลอมสีใกล้เคียงกัน (เช่น กล่อง/พื้นสีน้ำตาลอ่อน) ที่ดันมี H/S/V ใกล้เหลืองแต่ contour ไม่กลม
COLOR_SHAPE_LOCK = {"YELLOW": "CIRCLE"}

# ขนาดป้ายสี่เหลี่ยมที่รู้อยู่แล้ว (กว้าง x สูง หน่วยซม.) ไว้เทียบหาความคลาดเคลื่อน
KNOWN_SIGN_SIZES_CM = {
    "VERTICAL": (6.0, 9.0),     # ป้ายแนวตั้ง 6x9 ซม.
    "HORIZONTAL": (9.0, 6.0),   # ป้ายแนวนอน 9x6 ซม.
}

# ป้ายวงกลม: รัศมี 3.5 ซม. = เส้นผ่านศูนย์กลาง 7 ซม.
KNOWN_CIRCLE_DIAMETER_CM = 7.0

# ตัดสินว่า contour เป็นวงกลมมั้ยจากค่าความกลม (circularity = 4*pi*Area/Perimeter^2, วงกลมสมบูรณ์ = 1.0)
# สี่เหลี่ยม 6:9 ของเราคำนวณตามทฤษฎีได้ราวๆ 0.75 ส่วนวงกลมจริงๆ มักได้ 0.85-0.95 (มีความเบี้ยวจาก pixel)
CIRCLE_MIN_CIRCULARITY = 0.82

# ลู่ขนาดที่รายงาน/ใช้เล็งเข้าหาขนาดจริงตามทฤษฎี (กันขอบสีเบลอ/บวมทำให้วัดได้ใหญ่กว่าจริง)
# 0 = เชื่อขนาดที่วัดจาก contour ทั้งหมด, 1 = บังคับให้เท่ากับขนาดจริงตามทฤษฎีทั้งหมด (ยังใช้จุดศูนย์กลางที่วัดได้จริงเป๊ะเหมือนเดิม)
BOX_SIZE_BLEND = 0.5

# ค่าประมาณสำหรับแปลงพิกเซล -> ซม. จริง (มุมมองกล้อง EP โดยประมาณ + ระยะห่างตอนวัด)
# CAMERA_HFOV_DEG=95 ได้จาก calibrate ย้อนกลับจากป้ายจริง 3 ใบที่วัดได้ใหญ่เกินจริง ~1.6 เท่าตอนใช้ 120
# ถ้ายังคลาดเคลื่อนเยอะอยู่ ปรับเลขนี้ต่อได้ (คลาดเคลื่อน + = ตรวจได้ใหญ่กว่าจริง -> ลด HFOV ลงอีก)
CAMERA_HFOV_DEG = 95.0    # มุมมองแนวนอนกล้อง EP (calibrate จากป้ายจริงแล้ว)
DISTANCE_CM = 100.0       # ระยะห่างหุ่น <-> ป้ายตอนวัด (ถ้าเปลี่ยนระยะ ให้แก้เลขนี้ตาม)

# ที่ระยะ 1 เมตร ป้าย 6x9 ซม. จะเล็กในภาพ (ราวๆ 20x30 พิกเซล) เลยตั้ง MIN_SIGN_AREA_PX ไว้ต่ำ
MIN_SIGN_AREA_PX = 150    # พื้นที่เล็กสุด (พิกเซล^2) ถึงจะนับว่าเป็นป้าย ไม่ใช่สัญญาณรบกวน
MIN_FILL_RATIO = 0.55     # พื้นที่ก้อนสี เทียบกับพื้นที่กรอบสี่เหลี่ยมล้อมรอบ ต้องไม่ต่ำกว่านี้ (คัดของที่ไม่ใช่รูปสี่เหลี่ยม)

# กรอบ ROI กลางจอ (สัดส่วนของทั้งเฟรม) - ตรวจป้ายเฉพาะในกรอบนี้เท่านั้น ตัดขอบโต๊ะ/ผนัง/พื้นไกลๆ ทิ้งไปเลย
ROI_W_RATIO = 0.7
ROI_H_RATIO = 0.55

# เผื่อสัดส่วนกว้าง/สูงที่ตรวจจับได้คลาดเคลื่อนจากอัตราส่วนจริง (6:9 หรือ 9:6) ได้กี่ % ก่อนตัดทิ้ง
# (กันของที่รูปทรงเพี้ยนไปเลย เช่นแถบยาวๆ หรือสี่เหลี่ยมจัตุรัส ไม่ใช่ป้าย)
ASPECT_TOLERANCE = 0.3
_VERT_RATIO = KNOWN_SIGN_SIZES_CM["VERTICAL"][0] / KNOWN_SIGN_SIZES_CM["VERTICAL"][1]      # 6/9
_HORIZ_RATIO = KNOWN_SIGN_SIZES_CM["HORIZONTAL"][0] / KNOWN_SIGN_SIZES_CM["HORIZONTAL"][1]  # 9/6
ASPECT_VERT_RANGE = (_VERT_RATIO * (1 - ASPECT_TOLERANCE), _VERT_RATIO * (1 + ASPECT_TOLERANCE))
ASPECT_HORIZ_RANGE = (_HORIZ_RATIO * (1 - ASPECT_TOLERANCE), _HORIZ_RATIO * (1 + ASPECT_TOLERANCE))

# เผื่อขนาดพิกเซลจริงที่วัดได้ คลาดเคลื่อนจากขนาดที่ "ควรจะเป็น" ตามระยะ DISTANCE_CM ได้กี่เท่า ก่อนตัดทิ้ง
# (กันของที่ใหญ่/เล็กเกินกว่าจะเป็นป้าย เช่นขอบโต๊ะทั้งแผ่น หรือจุดเล็กๆ บนผนัง)
# เข้มขึ้นจากเดิม (0.5-2.0) เพราะของปลอมที่เจอ (จุดเหลืองมุมจอ, แสงสะท้อนบนขาโต๊ะ) มีขนาดเบี้ยวไปทางเล็ก/ใหญ่กว่าป้ายจริงชัดเจน
SIZE_MIN_FACTOR = 0.65
SIZE_MAX_FACTOR = 1.5

# --------------------------------------------------
# กรองป้ายปลอม/ไม่นิ่งด้วยความต่อเนื่อง (temporal stability):
# ของจริงจะอยู่นิ่งๆ ในตำแหน่งเดิมหลายเฟรมติด ส่วนแสงสะท้อน/สัญญาณรบกวนมักหลุดๆ ติดๆ
# เลยต้องเจอ "สีเดิม ตำแหน่งใกล้เดิม" ต่อเนื่องกันสัก MIN_STABLE_HITS เฟรม ถึงจะถือว่าเป็นป้ายจริง
# --------------------------------------------------
STABLE_MATCH_DIST_PX = 40   # ระยะพิกเซลที่ยังถือว่าเป็นป้ายเดิม (ไม่ขยับเกินนี้)
MIN_STABLE_HITS = 4         # ต้องเจอต่อเนื่องกี่เฟรมถึงจะเริ่มนับว่าเป็นป้ายจริง (โผล่มาแวบเดียวจะไม่นับ)
STABLE_MISS_GRACE = 2       # หายไปไม่เกินกี่เฟรมติด ไม่หักคะแนนเลย (ทนต่อการหลุดสั้นๆ ของสีเขียว)
MAX_STABLE_MISSES = 12      # หายไปติดกันเกินกี่เฟรม ให้ลืมป้ายนี้ทิ้งไปเลย (เพิ่มจาก 5 เพราะสีเขียวหลุดเป็นช่วงสั้นๆ บ่อย)

_sign_tracker = []  # เก็บสถานะความต่อเนื่องของป้ายที่เจอ ข้ามเฟรม
_next_track_id = 1  # เลข id ไม่ซ้ำของแต่ละป้ายที่ tracker เจอ (ไว้ยึดติดตามป้ายใบเดิมเป๊ะๆ ระหว่างยิงอัตโนมัติ)

# คุม Gimbal เองด้วย WASD: W=เงย, S=ก้ม, A=หันซ้าย, D=หันขวา
MANUAL_TURN_SPEED = 60          # องศา/วินาที
MANUAL_KEY_HOLD_TIMEOUT = 0.15  # วินาที ถ้าไม่มีคีย์ซ้ำในช่วงนี้ถือว่าปล่อยคีย์แล้ว ให้หยุดหมุน


def reset_sign_tracker():
    """ล้างประวัติความต่อเนื่องของป้าย เรียกตอนเริ่มรอบตรวจจับใหม่"""
    global _sign_tracker, _next_track_id
    _sign_tracker = []
    _next_track_id = 1


def _update_sign_tracker(raw_signs):
    """จับคู่ป้ายที่เจอรอบนี้กับ track เดิม (สี+ตำแหน่งใกล้กัน) แล้วคืนเฉพาะป้ายที่เจอต่อเนื่องพอจะเชื่อถือได้
    แต่ละป้ายจะมี track_id ติดตัวคงที่ ไว้ยึดตามป้าย "ใบเดิม" เป๊ะๆ แม้จะมีป้ายสีซ้ำกันหลายใบในเฟรม"""
    global _sign_tracker, _next_track_id

    matched = set()
    for sign in raw_signs:
        x, y, w, h = sign["bbox_px"]
        cx, cy = x + w / 2.0, y + h / 2.0

        best_i, best_dist = None, None
        for i, tr in enumerate(_sign_tracker):
            if i in matched or tr["color"] != sign["color"]:
                continue
            dist = math.hypot(tr["cx"] - cx, tr["cy"] - cy)
            if dist <= STABLE_MATCH_DIST_PX and (best_dist is None or dist < best_dist):
                best_i, best_dist = i, dist

        if best_i is not None:
            tr = _sign_tracker[best_i]
            sign_with_id = dict(sign, track_id=tr["id"])
            tr.update(cx=cx, cy=cy, hits=tr["hits"] + 1, misses=0, sign=sign_with_id)
            matched.add(best_i)
        else:
            sign_with_id = dict(sign, track_id=_next_track_id)
            _sign_tracker.append({"id": _next_track_id, "color": sign["color"], "cx": cx, "cy": cy,
                                   "hits": 1, "misses": 0, "sign": sign_with_id})
            _next_track_id += 1
            matched.add(len(_sign_tracker) - 1)

    still_alive = []
    for i, tr in enumerate(_sign_tracker):
        if i not in matched:
            tr["misses"] += 1
            if tr["misses"] > STABLE_MISS_GRACE:
                tr["hits"] = max(tr["hits"] - 1, 0)
            if tr["misses"] > MAX_STABLE_MISSES:
                continue
        still_alive.append(tr)
    _sign_tracker = still_alive

    return [tr["sign"] for tr in _sign_tracker if tr["hits"] >= MIN_STABLE_HITS]


def _px_to_cm_scale(frame_w):
    """cm ต่อ 1 พิกเซล ที่ระยะ DISTANCE_CM โดยประมาณจาก CAMERA_HFOV_DEG (สมมติ pixel เป็นสี่เหลี่ยมจัตุรัส)"""
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    view_width_cm = 2 * DISTANCE_CM * math.tan(hfov_rad / 2)
    return view_width_cm / frame_w


def roi_rect_px(frame_w, frame_h):
    """กรอบ ROI กลางจอเป็นพิกเซล (x, y, w, h) จากสัดส่วน ROI_W_RATIO/ROI_H_RATIO"""
    rw = int(frame_w * ROI_W_RATIO)
    rh = int(frame_h * ROI_H_RATIO)
    rx = (frame_w - rw) // 2
    ry = (frame_h - rh) // 2
    return rx, ry, rw, rh


def detect_signs(frame):
    """หาป้าย (สี่เหลี่ยม 6x9/9x6 หรือวงกลม เส้นผ่านศูนย์กลาง 7ซม.) เฉพาะในกรอบ ROI กลางจอ
    ตัดสินรูปทรงเองจาก contour (ไม่ผูกกับสีตายตัว) คืน list ของ dict: สี, รูปทรง, ขนาดที่ตรวจจับได้, ความคลาดเคลื่อน"""
    fh, fw = frame.shape[:2]
    cm_per_px = _px_to_cm_scale(fw)

    rx, ry, rw, rh = roi_rect_px(fw, fh)
    roi = frame[ry:ry + rh, rx:rx + rw]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    erode_kernel = np.ones((3, 3), np.uint8)

    signs = []
    for label in TARGET_SIGN_COLORS:
        spec = COLOR_RANGES[label]
        mask_total = None
        for lower, upper in spec["ranges"]:
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = mask if mask_total is None else (mask_total | mask)

        # ลบจุดสัญญาณรบกวนเล็กๆ + ปิดรูเล็กๆ ในก้อนสี แล้วกัดขอบนิดหน่อยกันสีบวม/เบลอเกินขอบจริง
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)
        mask_total = cv2.erode(mask_total, erode_kernel, iterations=1)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_SIGN_AREA_PX:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if area / float(w * h) < MIN_FILL_RATIO:
                continue  # รูปทรงไม่เข้ารูปเลย (เช่นสีกระจัดกระจาย) ตัดทิ้ง

            # ใช้เส้นรอบวงจาก convex hull แทน contour ดิบ เพราะขอบป้ายจริงมักมีรอยหยักเล็กๆ
            # จาก noise ของกล้อง/morphology ทำให้ perimeter ดิบสูงเกินจริงเป็นบางเฟรม (พื้นที่ area ไม่เปลี่ยนแต่ perimeter พุ่ง)
            # -> circularity แกว่งหลุดเกณฑ์เป็นพักๆ ทั้งที่เนื้อสีตรงกลางป้ายยังเต็มดวงเหมือนเดิมทุกเฟรม (สาเหตุที่ป้ายวงกลมติดๆดับๆ)
            # convex hull ตัดรอยเว้าเล็กๆ พวกนี้ทิ้ง ทำให้ค่า circularity นิ่งกว่าเดิมมาก
            hull = cv2.convexHull(cnt)
            perimeter = cv2.arcLength(hull, True)
            circularity = (4 * math.pi * area / (perimeter ** 2)) if perimeter > 0 else 0.0

            if circularity >= CIRCLE_MIN_CIRCULARITY:
                # ---------- ป้ายวงกลม ----------
                (ccx, ccy), radius_px = cv2.minEnclosingCircle(cnt)
                diameter_px = 2 * radius_px
                expected_diameter_px = KNOWN_CIRCLE_DIAMETER_CM / cm_per_px
                if not (expected_diameter_px * SIZE_MIN_FACTOR <= diameter_px <= expected_diameter_px * SIZE_MAX_FACTOR):
                    continue  # ใหญ่/เล็กเกินไปสำหรับวงกลมที่ควรจะเป็น ตัดทิ้ง

                sign_type = "CIRCLE"
                known_w_cm = known_h_cm = KNOWN_CIRCLE_DIAMETER_CM
                # ลู่ขนาดเข้าหาค่าจริงตามทฤษฎี แต่ยังใช้จุดศูนย์กลางที่วัดได้จริง
                refined_d = diameter_px * (1 - BOX_SIZE_BLEND) + expected_diameter_px * BOX_SIZE_BLEND
                box_x = int(ccx - refined_d / 2)
                box_y = int(ccy - refined_d / 2)
                box_w = box_h = int(refined_d)
            else:
                # ---------- ป้ายสี่เหลี่ยม ----------
                aspect = w / float(h)
                is_vert = ASPECT_VERT_RANGE[0] <= aspect <= ASPECT_VERT_RANGE[1]
                is_horiz = ASPECT_HORIZ_RANGE[0] <= aspect <= ASPECT_HORIZ_RANGE[1]
                if not is_vert and not is_horiz:
                    continue  # สัดส่วนกว้าง/สูงไม่ใกล้ 6:9 หรือ 9:6 เลย (เช่นแถบยาว) ตัดทิ้ง

                if is_vert and (not is_horiz or abs(aspect - _VERT_RATIO) <= abs(aspect - _HORIZ_RATIO)):
                    sign_type = "VERTICAL"
                else:
                    sign_type = "HORIZONTAL"
                known_w_cm, known_h_cm = KNOWN_SIGN_SIZES_CM[sign_type]

                # เช็กว่าขนาดพิกเซลจริงใกล้เคียงขนาดที่ "ควรจะเป็น" ตามระยะที่ตั้งไว้มั้ย ถ้าใหญ่/เล็กเกินไปตัดทิ้ง
                expected_w_px = known_w_cm / cm_per_px
                expected_h_px = known_h_cm / cm_per_px
                if not (expected_w_px * SIZE_MIN_FACTOR <= w <= expected_w_px * SIZE_MAX_FACTOR):
                    continue
                if not (expected_h_px * SIZE_MIN_FACTOR <= h <= expected_h_px * SIZE_MAX_FACTOR):
                    continue

                # ใช้จุดศูนย์ถ่วง (moments) แทนจุดกึ่งกลางกรอบ เพราะทนต่อขอบสีเบี้ยว/บวมด้านใดด้านหนึ่งได้ดีกว่า
                # (กรอบสี่เหลี่ยมล้อมรอบไวต่อ pixel เบี้ยวข้างเดียวมาก ทำให้จุดกึ่งกลางเพี้ยนได้ง่าย)
                moments = cv2.moments(cnt)
                if moments["m00"] > 0:
                    ccx = moments["m10"] / moments["m00"]
                    ccy = moments["m01"] / moments["m00"]
                else:
                    ccx, ccy = x + w / 2.0, y + h / 2.0
                refined_w = w * (1 - BOX_SIZE_BLEND) + expected_w_px * BOX_SIZE_BLEND
                refined_h = h * (1 - BOX_SIZE_BLEND) + expected_h_px * BOX_SIZE_BLEND
                box_x = int(ccx - refined_w / 2)
                box_y = int(ccy - refined_h / 2)
                box_w, box_h = int(refined_w), int(refined_h)

            required_shape = COLOR_SHAPE_LOCK.get(label)
            if required_shape and sign_type != required_shape:
                continue  # สีนี้ถูกล็อกว่าต้องเป็นรูปทรงที่กำหนดเท่านั้น (เช่น เหลือง=วงกลม) ไม่ตรงตัดทิ้ง

            detected_w_cm = box_w * cm_per_px
            detected_h_cm = box_h * cm_per_px

            signs.append({
                "color": label,
                "sign_type": sign_type,
                "bbox_px": (box_x + rx, box_y + ry, box_w, box_h),  # แปลงกลับเป็นพิกัดบนเฟรมเต็ม
                "detected_w_cm": detected_w_cm,
                "detected_h_cm": detected_h_cm,
                "known_w_cm": known_w_cm,
                "known_h_cm": known_h_cm,
                "error_w_cm": detected_w_cm - known_w_cm,
                "error_h_cm": detected_h_cm - known_h_cm,
                "error_w_pct": (detected_w_cm - known_w_cm) / known_w_cm * 100.0,
                "error_h_pct": (detected_h_cm - known_h_cm) / known_h_cm * 100.0,
            })

    # กรองด้วยความต่อเนื่อง (temporal stability) - ต้องเจอสีเดิม/ตำแหน่งใกล้เดิมหลายเฟรมติดถึงจะเชื่อถือได้
    return _update_sign_tracker(signs)


def draw_signs_overlay(frame, signs):
    """วาด crosshair กึ่งกลางจอ + กรอบ ROI (เหลือง) + กรอบรอบป้ายแต่ละใบที่เจอ (เขียว) พร้อมป้ายกำกับสี/ประเภท/ขนาด"""
    fh, fw = frame.shape[:2]
    cx, cy = fw // 2, fh // 2
    cv2.drawMarker(frame, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)

    rx, ry, rw, rh = roi_rect_px(fw, fh)
    cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)

    for sign in signs:
        x, y, w, h = sign["bbox_px"]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        line1 = f"{sign['color']} {sign['sign_type']}"
        line2 = f"{sign['detected_w_cm']:.1f}x{sign['detected_h_cm']:.1f}cm (real {sign['known_w_cm']:.0f}x{sign['known_h_cm']:.0f})"
        cv2.putText(frame, line1, (x, max(y - 26, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame, line2, (x, max(y - 6, 38)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame


def log_colorscan(capture_id, signs):
    """บันทึกผลป้ายที่เจอแต่ละใบ (คนละแถวต่อป้าย) ลงไฟล์ csv เก็บไว้เป็นหลักฐาน"""
    header = ["timestamp", "capture_id", "color", "sign_type", "bbox_px",
              "detected_w_cm", "detected_h_cm", "known_w_cm", "known_h_cm",
              "error_w_cm", "error_h_cm", "error_w_pct", "error_h_pct"]
    if not signs:
        _append_csv_row(COLORSCAN_LOG_PATH, header,
                         [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), capture_id, "-", "-", "-",
                          "-", "-", "-", "-", "-", "-", "-", "-"])
        return

    for sign in signs:
        _append_csv_row(COLORSCAN_LOG_PATH, header, [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"), capture_id, sign["color"], sign["sign_type"],
            str(sign["bbox_px"]), f"{sign['detected_w_cm']:.2f}", f"{sign['detected_h_cm']:.2f}",
            f"{sign['known_w_cm']:.1f}", f"{sign['known_h_cm']:.1f}",
            f"{sign['error_w_cm']:.2f}", f"{sign['error_h_cm']:.2f}",
            f"{sign['error_w_pct']:.1f}", f"{sign['error_h_pct']:.1f}",
        ])


def run_colorscan_mode(ep_robot):
    global ep_camera

    reset_sign_tracker()
    ep_gimbal = ep_robot.gimbal
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    window_name = "RoboMaster Camera - colorscan"
    print("คุม Gimbal เองที่หน้าต่างภาพ: W=เงยขึ้น, S=ก้มลง, A=หันซ้าย, D=หันขวา")
    print(f"สมมติระยะห่างหุ่น<->ป้าย = {DISTANCE_CM:.0f} ซม. (ถ้าวางระยะอื่น แก้ค่า DISTANCE_CM ในไฟล์นี้ก่อนรัน)")
    print("เล็งให้ป้ายอยู่ในกรอบเหลือง (ROI) เท่านั้นถึงจะถูกตรวจ - นอกกรอบนี้จะไม่สนใจเลย กันขอบโต๊ะ/ผนัง/พื้นไกลๆ")
    print("เจอป้ายในกรอบจะขึ้นกรอบเขียว + ประเภท/สี/ขนาดที่ตรวจจับได้ทันที กด Enter หรือ Space เพื่อแคปภาพ + บันทึก")
    print("ขนาด/ค่าคลาดเคลื่อนที่ได้เป็นค่าประมาณจาก CAMERA_HFOV_DEG (calibrate ไว้แล้วคร่าวๆ) "
          "ถ้าคลาดเคลื่อนเยอะผิดปกติ ลองปรับค่านี้ในไฟล์แล้วรันใหม่จนค่า error ใกล้ 0%")
    print("กด ESC หรือ q ที่หน้าต่างภาพ เพื่อออกจากโปรแกรม (ต้องคลิกที่หน้าต่างภาพให้โฟกัสก่อนกดคีย์)")

    last_yaw_key_time = 0.0
    last_pitch_key_time = 0.0
    yaw_dir = 0
    pitch_dir = 0
    capture_count = 0
    live_signs = []

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            now = time.time()

            if frame is not None:
                live_signs = detect_signs(frame)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                print(f"[DEBUG] key pressed: {key}")  # เช็คว่าคีย์เข้าถึงหน้าต่างภาพจริงมั้ย (ลบทิ้งได้ถ้าไม่ใช้แล้ว)

            # A/D คุมหันซ้าย-ขวา (yaw), W/S คุมเงย-ก้ม (pitch) แบบกดค้างไว้ยังหมุนต่อ
            if key in (ord('a'), ord('A')):
                yaw_dir, last_yaw_key_time = -1, now
            elif key in (ord('d'), ord('D')):
                yaw_dir, last_yaw_key_time = 1, now
            if now - last_yaw_key_time > MANUAL_KEY_HOLD_TIMEOUT:
                yaw_dir = 0

            if key in (ord('w'), ord('W')):
                pitch_dir, last_pitch_key_time = 1, now
            elif key in (ord('s'), ord('S')):
                pitch_dir, last_pitch_key_time = -1, now
            if now - last_pitch_key_time > MANUAL_KEY_HOLD_TIMEOUT:
                pitch_dir = 0

            ep_gimbal.drive_speed(pitch_speed=pitch_dir * MANUAL_TURN_SPEED,
                                   yaw_speed=yaw_dir * MANUAL_TURN_SPEED)

            if key in (13, 32):  # Enter หรือ Space -> แคปภาพ
                if frame is not None:
                    capture_count += 1
                    capture_id = f"{capture_count:03d}"
                    print(f"\n>> แคปภาพครั้งที่ {capture_id}")
                    if live_signs:
                        for sign in live_signs:
                            th_name = COLOR_RANGES[sign["color"]]["th"]
                            print(f"   - [{sign['color']}/{th_name}] ประเภท {sign['sign_type']} "
                                  f"ขนาดที่ตรวจจับได้ {sign['detected_w_cm']:.1f}x{sign['detected_h_cm']:.1f} ซม. "
                                  f"(ของจริง {sign['known_w_cm']:.0f}x{sign['known_h_cm']:.0f} ซม., "
                                  f"คลาดเคลื่อน {sign['error_w_cm']:+.1f}/{sign['error_h_cm']:+.1f} ซม. "
                                  f"= {sign['error_w_pct']:+.0f}%/{sign['error_h_pct']:+.0f}%)")
                    else:
                        print("   - ไม่เจอป้ายในเฟรม (ลองขยับป้ายให้เข้าจอ หรือปรับแสง)")
                    save_frame(f"colorscan_{capture_id}", frame=frame)
                    log_colorscan(capture_id, live_signs)
                else:
                    print("[WARN] ยังอ่านภาพจากกล้องไม่ได้ ลองใหม่อีกครั้ง")

            if frame is not None:
                # ใช้ข้อความอังกฤษล้วนบนภาพ เพราะ cv2.putText วาดตัวอักษรไทยไม่ได้
                info_lines = ["WASD=move  ENTER/SPACE=capture  ESC/Q=quit", f"signs found: {len(live_signs)}"]
                draw_overlay(frame, None, locked=False, info_lines=info_lines)
                draw_signs_overlay(frame, live_signs)
                cv2.imshow(window_name, frame)

            if key == 27 or key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nหยุดโหมด colorscan")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()


# --------------------------------------------------
# main: เลือกโหมดจากคำสั่งที่พิมพ์ตอนรัน
#   python detectcolor.py fire       -> เล็ง + ยิงป้ายด้วย PID (ยิง IR เท่านั้น ปลอดภัยเสมอ ไม่มีกระสุนเจลออก)
#   python detectcolor.py fire_live  -> เหมือน fire ทุกอย่าง แต่ยิงกระสุนเจลจริง (WATER_FIRE) ต้องพิมพ์ยืนยันก่อนเริ่มทุกครั้ง
#   python detectcolor.py colorscan  -> ตรวจจับป้าย + สีทั้งหมดบนป้าย
# --------------------------------------------------
def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("fire", "fire_live", "colorscan"):
        print("วิธีใช้:")
        print("  python detectcolor.py fire       -> เล็ง + ยิงป้ายด้วย PID (ยิง IR เท่านั้น ปลอดภัย ไม่มีกระสุนเจลออก)")
        print("  python detectcolor.py fire_live  -> เหมือน fire แต่ยิงกระสุนเจลจริง (ต้องพิมพ์ยืนยันก่อนเริ่ม)")
        print("  python detectcolor.py colorscan  -> ตรวจจับป้าย + หาสีทั้งหมดบนป้าย")
        return

    mode = sys.argv[1]

    if mode == "fire_live":
        print("=" * 60)
        print("!!! คำเตือน: fire_live จะยิงกระสุนเจลจริงออกจากป้อมปืนทุกครั้งที่ล็อกเป้าสำเร็จ !!!")
        print("เช็คก่อนเริ่ม: มีที่กันกระสุน/แผ่นกันหลังเป้าแล้ว, ไม่มีคน/สัตว์เลี้ยงอยู่ในแนวยิงหรือแนวสะท้อน, บรรจุกระสุนเจลถูกต้องแล้ว")
        print("=" * 60)
        confirm = input("พิมพ์ YES (ตัวพิมพ์ใหญ่ทั้งหมด) เพื่อยืนยันว่าพร้อมยิงจริง: ").strip()
        if confirm != "YES":
            print("ยกเลิก ไม่ได้ยืนยัน ไม่มีการยิงเกิดขึ้น")
            return

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    try:
        if mode in ("fire", "fire_live"):
            run_fire_mode(ep_robot, safe_mode=(mode == "fire"))
        elif mode == "colorscan":
            run_colorscan_mode(ep_robot)
    finally:
        ep_robot.close()


if __name__ == "__main__":
    main()
