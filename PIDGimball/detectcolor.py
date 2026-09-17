import csv
import os
import sys
import threading
import time
from datetime import datetime

import cv2
from robomaster import robot, vision, blaster
#cd "d:\ROBOT-Final\Final-Robot\PIDGimball"
#python detectcolor.py fire
#python detectcolor.py colorscan


# โฟลเดอร์เก็บไฟล์ภาพทั้งหมด = โฟลเดอร์นี้ (PIDGimball) เสมอ ไม่ว่าจะรันจากที่ไหน
SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
ACCURACY_LOG_PATH = os.path.join(SAVE_DIR, "fire_accuracy_log.csv")
COLORSCAN_LOG_PATH = os.path.join(SAVE_DIR, "colorscan_log.csv")

# --------------------------------------------------
# โหมดปลอดภัย: ตอนนี้ในรังเพลิงมีกระสุนเจลอยู่ -> ห้ามยิงจริง
#   True  = ล็อกเป้าสำเร็จแล้ว "ไม่ยิง" กระพริบไฟที่หัวยิงแทน แล้วถ่ายภาพเก็บไว้ตรวจสอบ
#   False = ล็อกเป้าสำเร็จแล้วยิงจริง (เอากระสุนเจลออกก่อน หรือแน่ใจแล้วค่อยเปลี่ยนเป็น False)
# --------------------------------------------------
SAFE_MODE = True


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
pid_yaw = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-180, 180))
pid_pitch = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-120, 120))

target_locked = False
ep_gimbal = None
ep_blaster = None
ep_camera = None

# สถานะล่าสุดของป้ายที่กำลังเล็ง ไว้ใช้วาด overlay บนหน้าจอกล้อง
current_target = {"bbox": None, "err": None}
first_seen_at = {}  # marker_id -> เวลาที่เห็นครั้งแรก (ไว้คำนวณเวลากว่าจะล็อกเป้าติด)


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
# โหมด "fire": เล็งป้ายหมายเลข 1 ด้วย PID แล้วยิง
# --------------------------------------------------
def log_accuracy(marker_id, err_x, err_y, frame, yaw_speed, pitch_speed, time_to_lock):
    """บันทึกค่าความแม่นยำตอนล็อกเป้าสำเร็จ (ระยะเบี่ยงเบนจากกึ่งกลางจอ เทียบเท่าจุดที่เลเซอร์/กระสุนจะไปโดน)"""
    fh, fw = frame.shape[:2]
    err_x_px = err_x * fw
    err_y_px = err_y * fh
    pixel_distance = (err_x_px ** 2 + err_y_px ** 2) ** 0.5

    _append_csv_row(
        ACCURACY_LOG_PATH,
        ["timestamp", "marker_id", "err_x", "err_y", "err_x_px", "err_y_px",
         "pixel_distance", "yaw_speed", "pitch_speed", "time_to_lock_sec"],
        [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), marker_id,
         f"{err_x:.4f}", f"{err_y:.4f}", f"{err_x_px:.1f}", f"{err_y_px:.1f}",
         f"{pixel_distance:.1f}", f"{yaw_speed:.1f}", f"{pitch_speed:.1f}",
         f"{time_to_lock:.2f}"],
    )
    print(f"[LOG] ความแม่นยำ -> เบี่ยงจากกึ่งกลาง {pixel_distance:.1f}px "
          f"(x={err_x_px:.1f}px, y={err_y_px:.1f}px), ใช้เวลา {time_to_lock:.2f}s -> {os.path.basename(ACCURACY_LOG_PATH)}")


def on_detect_marker_fire(marker_info):
    global target_locked, ep_gimbal, ep_blaster, pid_yaw, pid_pitch

    if target_locked:
        return

    for marker in marker_info:
        x, y, w, h, info = marker
        print("[DEBUG] เห็น marker: info={0} x={1:.2f} y={2:.2f}".format(info, x, y))

        # เล็งล็อกเป้าหมายป้ายหมายเลข "1"
        if info == "1":
            first_seen_at.setdefault(info, time.time())

            # คำนวณ Error สัมพัทธ์กับจุดกึ่งกลางจอ (0.5)
            err_x = x - 0.5
            err_y = 0.5 - y
            current_target["bbox"] = (x, y, w, h)
            current_target["err"] = (err_x, err_y)

            # คำนวณความเร็วด้วย PID
            yaw_speed = pid_yaw.compute(err_x)
            pitch_speed = pid_pitch.compute(err_y)

            # เช็กเงื่อนไข ล็อกเป้าเมื่อ Error ต่ำกว่า 3% (|Error| < 0.03)
            if abs(err_x) < 0.03 and abs(err_y) < 0.03:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                target_locked = True
                time_to_lock = time.time() - first_seen_at.pop(info, time.time())

                if SAFE_MODE:
                    print(">> ล็อกเป้าสำเร็จด้วย PID! -> (SAFE MODE) ไม่ยิงจริง กระพริบไฟแทน + ถ่ายภาพเก็บไว้")
                    ep_blaster.set_led(brightness=255, effect=blaster.LED_ON)
                    time.sleep(0.3)
                    ep_blaster.set_led(brightness=0, effect=blaster.LED_OFF)
                else:
                    print(">> ล็อกเป้าสำเร็จด้วย PID! -> ยิง!")
                    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)

                locked_frame = save_frame(f"fire_{info}")
                if locked_frame is not None:
                    log_accuracy(info, err_x, err_y, locked_frame, yaw_speed, pitch_speed, time_to_lock)

                # รีเซ็ตค่าสะสมของ PID เมื่อยิงเสร็จ
                pid_yaw.reset()
                pid_pitch.reset()
            else:
                # สั่ง Gimbal เคลื่อนที่ตามความเร็ว PID
                ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
            break


def run_fire_mode(ep_robot):
    global ep_gimbal, ep_blaster, ep_camera, target_locked

    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    print("เริ่มค้นหา Vision Marker หมายเลข 1 ด้วย PID Auto-Aim...")
    ep_vision.sub_detect_info(name=vision.MARKER, callback=on_detect_marker_fire)

    window_name = "RoboMaster Camera - fire"
    lock_reset_at = None

    print("แสดงภาพกล้องสด (กด ESC หรือ q ที่หน้าต่างภาพ หรือ Ctrl+C เพื่อหยุด)")
    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            if frame is not None:
                err = current_target["err"]
                info_lines = [f"SAFE_MODE={SAFE_MODE}"]
                if err is not None:
                    info_lines.append(f"err_x={err[0]:.3f} err_y={err[1]:.3f}")
                if target_locked:
                    info_lines.append("LOCKED!")
                draw_overlay(frame, current_target["bbox"], locked=target_locked, info_lines=info_lines)
                cv2.imshow(window_name, frame)

            # รอ 2 วิหลังล็อกเป้าติด แล้วค่อยรีเซ็ตไปหาเป้าถัดไป โดยไม่บล็อกการอัปเดตภาพ
            if target_locked and lock_reset_at is None:
                lock_reset_at = time.time() + 2.0
            if lock_reset_at is not None and time.time() >= lock_reset_at:
                target_locked = False
                current_target["bbox"] = None
                current_target["err"] = None
                lock_reset_at = None
                print(">> พร้อมค้นหาเป้าหมายถัดไป...")

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nหยุดโหมด fire")
    finally:
        ep_vision.unsub_detect_info(name=vision.MARKER)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()


# --------------------------------------------------
# โหมด "colorscan": ตรวจจับป้าย + หาว่าบนป้ายมีสีอะไรบ้าง
# --------------------------------------------------
# ช่วงสี HSV (H: 0-180 แบบ OpenCV) ของแต่ละสีที่จะเช็กบนป้าย
COLOR_RANGES = {
    "แดง": [((0, 90, 60), (8, 255, 255)), ((170, 90, 60), (180, 255, 255))],
    "ส้ม": [((9, 90, 60), (20, 255, 255))],
    "เหลือง": [((21, 90, 60), (33, 255, 255))],
    "เขียว": [((34, 60, 40), (85, 255, 255))],
    "ฟ้า/น้ำเงิน": [((86, 60, 40), (135, 255, 255))],
    "ม่วง": [((136, 60, 40), (169, 255, 255))],
    "ขาว": [((0, 0, 200), (180, 40, 255))],
    "ดำ": [((0, 0, 0), (180, 255, 45))],
}

MIN_COLOR_RATIO = 0.03  # นับว่าเจอสีนี้บนป้าย ถ้าสัดส่วนพิกเซล >= 3%

# --------------------------------------------------
# ตั้งค่าการหมุน Gimbal หาป้ายเองอัตโนมัติ (โหมด colorscan)
# --------------------------------------------------
SEARCH_YAW_SPEED = 40        # องศา/วินาที ความเร็วหมุนตอนหาป้าย
SEARCH_REVERSE_INTERVAL = 3.5  # วินาที ก่อนสลับทิศหมุนซ้าย<->ขวา
SIGN_LOST_TIMEOUT = 1.0      # ถ้าไม่เห็นป้ายนี้เกินกี่วินาที ถือว่าหลุดจากจอแล้ว เริ่มหมุนหาใหม่

captured_markers = set()  # ป้ายที่แคปภาพไปแล้ว (กด Enter แล้ว) จะไม่หยุดรอที่ป้ายเดิมอีก
current_scan = {"bbox": None, "marker_id": None, "colors": [], "last_seen": 0.0}  # สถานะล่าสุด ไว้วาด overlay + คุมการหมุนหา


def detect_colors_on_sign(frame, bbox):
    """bbox = (x, y, w, h) แบบ normalize 0-1 (x,y คือจุดกึ่งกลาง) จาก vision marker"""
    fh, fw = frame.shape[:2]
    x, y, w, h = bbox
    x1 = max(int((x - w / 2) * fw), 0)
    y1 = max(int((y - h / 2) * fh), 0)
    x2 = min(int((x + w / 2) * fw), fw)
    y2 = min(int((y + h / 2) * fh), fh)

    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return []

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    total_pixels = roi.shape[0] * roi.shape[1]

    found_colors = []
    for name, ranges in COLOR_RANGES.items():
        mask_total = None
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = mask if mask_total is None else (mask_total | mask)
        ratio = cv2.countNonZero(mask_total) / total_pixels
        if ratio >= MIN_COLOR_RATIO:
            found_colors.append((name, ratio))

    found_colors.sort(key=lambda item: item[1], reverse=True)
    return found_colors


def log_colorscan(marker_id, colors):
    """บันทึกผลสีที่เจอบนป้ายแต่ละใบลงไฟล์ csv เก็บไว้เป็นหลักฐาน"""
    colors_text = "; ".join(f"{name}:{ratio * 100:.1f}%" for name, ratio in colors) or "-"
    _append_csv_row(
        COLORSCAN_LOG_PATH,
        ["timestamp", "marker_id", "colors"],
        [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), marker_id, colors_text],
    )


def on_detect_marker_colorscan(marker_info):
    """แค่จำตำแหน่งป้ายล่าสุดไว้ ไม่เซฟ/ไม่ยิ่งอะไรที่นี่ - การแคปภาพทำตอนกด Enter เท่านั้น"""
    if not marker_info:
        return

    x, y, w, h, info = marker_info[0]
    current_scan["bbox"] = (x, y, w, h)
    current_scan["marker_id"] = info
    current_scan["last_seen"] = time.time()


def listen_for_enter(capture_event, stop_event):
    while not stop_event.is_set():
        try:
            input()
        except EOFError:
            return
        capture_event.set()


def run_colorscan_mode(ep_robot):
    global ep_camera

    ep_gimbal = ep_robot.gimbal
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    window_name = "RoboMaster Camera - colorscan"
    print("เริ่มโหมด colorscan: หมุนหาป้ายเอง เจอแล้วจะหยุดค้างรอ")
    print("กด Enter ที่ terminal นี้ -> แคปภาพ + บันทึกสีที่เจอ, กด Ctrl+C -> ออกจากโปรแกรม")
    ep_vision.sub_detect_info(name=vision.MARKER, callback=on_detect_marker_colorscan)

    capture_event = threading.Event()
    stop_event = threading.Event()
    input_thread = threading.Thread(target=listen_for_enter, args=(capture_event, stop_event), daemon=True)
    input_thread.start()

    search_dir = 1
    last_dir_switch = time.time()

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            now = time.time()

            # ถ้าไม่เห็นป้ายนี้เกิน timeout แล้ว ถือว่าหลุดจากจอ เคลียร์สถานะเพื่อกลับไปหมุนหาใหม่
            if current_scan["bbox"] is not None and now - current_scan["last_seen"] > SIGN_LOST_TIMEOUT:
                current_scan["bbox"] = None
                current_scan["marker_id"] = None
                current_scan["colors"] = []

            has_new_sign = (
                current_scan["bbox"] is not None
                and current_scan["marker_id"] not in captured_markers
            )

            if has_new_sign:
                # เจอป้ายที่ยังไม่เคยแคป -> หยุดหมุนนิ่งๆ แล้วตรวจสีสดๆ ทุกเฟรม รอกด Enter
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                if frame is not None:
                    current_scan["colors"] = detect_colors_on_sign(frame, current_scan["bbox"])
            else:
                # ไม่เจอป้ายใหม่ (หรือป้ายนี้แคปไปแล้ว) -> หมุน Gimbal ซ้าย-ขวาหาไปเรื่อยๆ เอง
                if now - last_dir_switch >= SEARCH_REVERSE_INTERVAL:
                    search_dir *= -1
                    last_dir_switch = now
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=search_dir * SEARCH_YAW_SPEED)

            if capture_event.is_set():
                capture_event.clear()
                if has_new_sign and frame is not None:
                    marker_id = current_scan["marker_id"]
                    colors = current_scan["colors"]
                    print(f"\n>> แคปภาพป้ายหมายเลข {marker_id}")
                    if colors:
                        for name, ratio in colors:
                            print(f"   - {name}: {ratio * 100:.1f}%")
                    else:
                        print("   - ไม่พบสีที่ตรงเงื่อนไขที่ตั้งไว้")
                    save_frame(f"colorscan_{marker_id}", frame=frame)
                    log_colorscan(marker_id, colors)
                    captured_markers.add(marker_id)
                    print(">> บันทึกแล้ว กำลังหมุนหาป้ายถัดไป (กด Ctrl+C เพื่อออกโปรแกรม)")
                else:
                    print("[WARN] ตอนนี้ยังไม่เจอป้ายให้แคป")

            if frame is not None:
                info_lines = [f"{name}: {ratio * 100:.0f}%" for name, ratio in current_scan["colors"]]
                if has_new_sign:
                    info_lines.insert(0, "เจอป้ายแล้ว - กด Enter เพื่อแคปภาพ")
                else:
                    info_lines.insert(0, "กำลังหมุนหาป้าย...")
                draw_overlay(frame, current_scan["bbox"], locked=has_new_sign, info_lines=info_lines)
                cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nหยุดโหมด colorscan")
    finally:
        stop_event.set()
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        ep_vision.unsub_detect_info(name=vision.MARKER)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()


# --------------------------------------------------
# main: เลือกโหมดจากคำสั่งที่พิมพ์ตอนรัน
#   python detectcolor.py fire       -> เล็ง + ยิงป้ายด้วย PID
#   python detectcolor.py colorscan  -> ตรวจจับป้าย + สีทั้งหมดบนป้าย
# --------------------------------------------------
def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("fire", "colorscan"):
        print("วิธีใช้:")
        print("  python detectcolor.py fire       -> เล็ง + ยิงป้ายหมายเลข 1 ด้วย PID")
        print("  python detectcolor.py colorscan  -> ตรวจจับป้าย + หาสีทั้งหมดบนป้าย")
        return

    mode = sys.argv[1]

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    try:
        if mode == "fire":
            run_fire_mode(ep_robot)
        elif mode == "colorscan":
            run_colorscan_mode(ep_robot)
    finally:
        ep_robot.close()


if __name__ == "__main__":
    main()
