import cv2
import time
import numpy as np
import threading
from robomaster import robot, blaster

# =============================================================================
#  ตั้งค่าระบบ PID
# =============================================================================
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
            dt = 0.01

        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.last_error) / dt

        output = p_term + i_term + d_term
        output = max(self.min_limit, min(self.max_limit, output))

        self.last_error = error
        self.last_time = now
        return output

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

# =============================================================================
#  ตั้งค่าเป้าหมายที่ต้องการตรวจจับ (Shape & Color)
# =============================================================================
# ตัวเลือกสี: "red", "blue", "yellow", "green"
# ตัวเลือกรูปทรง: "circle", "square", "rectangle"
TARGET_COLOR = "red"
TARGET_SHAPE = "rectangle"

# =============================================================================
#  ฟังก์ชันตรวจจับสี (HSV Range)
# =============================================================================
def get_color_mask(hsv_img, color):
    if color == "red":
        mask1 = cv2.inRange(hsv_img, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv_img, np.array([170, 120, 70]), np.array([180, 255, 255]))
        return cv2.bitwise_or(mask1, mask2)
    elif color == "blue":
        return cv2.inRange(hsv_img, np.array([100, 150, 0]), np.array([140, 255, 255]))
    elif color == "yellow":
        return cv2.inRange(hsv_img, np.array([20, 100, 100]), np.array([30, 255, 255]))
    elif color == "green":
        return cv2.inRange(hsv_img, np.array([35, 100, 100]), np.array([85, 255, 255]))
    return np.zeros(hsv_img.shape[:2], dtype=np.uint8)

# =============================================================================
#  ฟังก์ชันตรวจจับรูปร่าง
# =============================================================================
def detect_shape(contour):
    """
    จำแนกรูปทรงเป้าหมาย (Circle, Square, Rectangle) ด้วย Feature Extraction
    ใช้อัลกอริทึม cv2.minAreaRect (Minimum Area Bounding Box) และ cv2.fitEllipse (Direct Least Squares)
    ร่วมกับ Solidity, Extent, Circularity เพื่อการจำแนกที่แม่นยำแม้เป้าหมายหมุนเอียง
    """
    shape = "unidentified"
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    area = cv2.contourArea(contour)
    if area < 500 or peri == 0:
        return shape, approx
        
    (x, y, w, h) = cv2.boundingRect(contour)
    is_close = (area > 12000) or (w > 200) or (h > 200)

    # 1. Minimum Area Bounding Box (cv2.minAreaRect)
    rect = cv2.minAreaRect(contour)
    (cx, cy), (rw, rh), angle = rect
    if rw == 0 or rh == 0:
        return shape, approx
    
    min_rect_area = rw * rh
    extent_min_rect = area / float(min_rect_area) if min_rect_area > 0 else 0.0
    ar_min_rect = rw / float(rh) if rh > 0 else 0.0
    norm_ar = max(rw, rh) / min(rw, rh) if min(rw, rh) > 0 else 1.0

    # 2. Convex Hull & Solidity
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / float(hull_area) if hull_area > 0 else 0.0

    # 3. Direct Least Squares Ellipse Fitting (cv2.fitEllipse)
    ellipse_ratio = 0.0
    if len(contour) >= 5:
        try:
            ellipse = cv2.fitEllipse(contour)
            (ecx, ecy), (ma, MA), e_angle = ellipse
            ellipse_area = (np.pi / 4.0) * ma * MA
            ellipse_ratio = area / float(ellipse_area) if ellipse_area > 0 else 0.0
        except Exception:
            ellipse_ratio = 0.0

    # 4. Circularity
    circularity = (4.0 * np.pi * area) / (peri * peri)

    # 5. Target Shape Classification Logic
    min_circ = 0.42 if is_close else 0.58
    if (circularity > min_circ and norm_ar < 1.45) or (ellipse_ratio > 0.78 and norm_ar < 1.35 and len(approx) > 4):
        shape = "circle"
    elif (extent_min_rect > 0.60 or is_close) and (0.75 <= ar_min_rect <= 1.25 or norm_ar <= 1.25) and solidity > 0.75:
        shape = "square"
    elif (extent_min_rect > 0.50 or is_close) and (norm_ar > 1.25 or 0.3 <= ar_min_rect <= 0.75 or 1.25 <= ar_min_rect <= 3.5) and solidity > 0.70:
        shape = "rectangle"
    elif len(approx) == 4:
        ar_box = w / float(h) if h > 0 else 0
        if 0.75 <= ar_box <= 1.25:
            shape = "square"
        else:
            shape = "rectangle"

    return shape, approx

# =============================================================================
#  Main Process
# =============================================================================
def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera = ep_robot.camera

    print("เปิดกล้องและเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    # PID Controllers (ปรับจูนค่าตามความเหมาะสมของหุ่น)
    pid_yaw = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-150, 150))
    pid_pitch = PIDController(kp=80.0, ki=0.0, kd=0.0, limits=(-100, 100))

    target_locked_frames = 0
    TARGET_LOCK_THRESHOLD = 5  # ต้องตรวจเจอต่อเนื่องกี่เฟรมถึงจะยิง
    
    print(f"กำลังค้นหาเป้าหมาย: รูปทรง {TARGET_SHAPE} สี {TARGET_COLOR}")

    try:
        while True:
            img = ep_camera.read_cv2_image(strategy="newest")
            if img is None:
                time.sleep(0.01)
                continue

            h, w = img.shape[:2]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # 1. กรองสี
            mask = get_color_mask(hsv, TARGET_COLOR)
            
            # ลด Noise
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # 2. หา Contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            target_found = False
            best_contour = None
            max_area = 0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # กรองขนาดที่เล็กเกินไปทิ้ง
                    shape, approx = detect_shape(cnt)
                    if shape == TARGET_SHAPE and area > max_area:
                        max_area = area
                        best_contour = cnt

            # วาดเส้นกากบาทกลางจอ
            cv2.drawMarker(img, (w//2, h//2), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

            if best_contour is not None:
                # คำนวณจุดกึ่งกลางของวัตถุ
                M = cv2.moments(best_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # คำนวณ Error (อิงจาก 0.5 คือกึ่งกลางจอ)
                    err_x = (cx / w) - 0.5
                    err_y = 0.5 - (cy / h) # แกน y ของภาพกลับหัวกับแกน pitch

                    # วาดกรอบและจุดกึ่งกลางเป้าหมายที่เลือก
                    cv2.drawContours(img, [best_contour], -1, (0, 255, 0), 2)
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(img, f"{TARGET_COLOR} {TARGET_SHAPE}", (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # คำนวณความเร็ว Gimbal
                    yaw_speed = pid_yaw.compute(err_x)
                    pitch_speed = pid_pitch.compute(err_y)
                    
                    # เล็งเป้าให้ Error น้อยกว่า 3% 
                    if abs(err_x) < 0.03 and abs(err_y) < 0.03:
                        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                        target_locked_frames += 1
                        print(f"เล็งเป้าสำเร็จ! (นิ่ง {target_locked_frames}/{TARGET_LOCK_THRESHOLD})")
                        
                        if target_locked_frames >= TARGET_LOCK_THRESHOLD:
                            print(">> ยิงงงงงง! <<")
                            ep_blaster.fire(fire_type=blaster.IR_FIRE, times=1)
                            target_locked_frames = 0
                            pid_yaw.reset()
                            pid_pitch.reset()
                            time.sleep(1.5) # พักแปบนึงหลังยิง
                    else:
                        ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
                        target_locked_frames = 0
                        
                    target_found = True
            
            if not target_found:
                # ถ้าไม่เจอเป้า ให้หยุดหัน
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                target_locked_frames = 0
                pid_yaw.reset()
                pid_pitch.reset()

            # แสดงภาพสำหรับการ Debug
            cv2.imshow("Robot Camera - Shape & Color Detection", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("ยกเลิกการทำงาน...")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()
        ep_robot.close()

if __name__ == "__main__":
    main()
