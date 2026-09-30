"""
vision_shooter.py
=================
ระบบประมวลผลภาพและการยิงเป้าหมายแบบ Multi-Threaded Complete Suite
รองรับ:
  - 4 รูปแบบเป้าหมาย: Circle, Horizontal Rectangle, Vertical Rectangle, Square
  - 4 เฉดสีเป้าหมาย: Red, Blue, Yellow, Green (และโหมด Any)
  - ระบบตัดภาพเหนือขอบโฟมสนาม 11.5 นิ้ว (Foam Height Cutoff Filter)
  - UI Control Window & HUD Camera Overlay แสดงสถานะและปรับจูนสดๆ
  - Drop Old Frames Strategy (queue.Queue maxsize=1) อ่านเฟรมสด 0ms ดีเลย์
  - Global Hardware Lock (hardware_cmd_lock) ป้องกัน Socket ชนกัน
"""

import cv2
import time
import queue
import numpy as np
import threading
from robomaster import blaster

# =============================================================================
#  Global Constants & Preset Lists
# =============================================================================
COLOR_PRESETS = ["red", "blue", "yellow", "green", "any"]
SHAPE_PRESETS = ["circle", "rectangle_horizontal", "rectangle_vertical", "square", "any"]

# Global Hardware Port Command Lock
hardware_cmd_lock = threading.Lock()

def safe_gimbal_drive(ep_gimbal, pitch_speed, yaw_speed):
    """ส่งคำสั่งคุมความเร็ว Gimbal อย่างปลอดภัย ป้องกัน Socket Collision"""
    with hardware_cmd_lock:
        try:
            if ep_gimbal is not None:
                ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
        except Exception:
            pass

def safe_blaster_fire(ep_blaster):
    """ส่งคำสั่งยิง IR Blaster อย่างปลอดภัย ป้องกัน Socket Collision"""
    with hardware_cmd_lock:
        try:
            if ep_blaster is not None:
                ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
        except Exception as e:
            print(f"[Blaster] Fire error: {e}")

# =============================================================================
#  ระบบควบคุม PID (Proportional-Integral-Derivative Controller)
# =============================================================================
class PIDController:
    def __init__(self, kp=80.0, ki=0.0, kd=10.0, limits=(-150, 150)):
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
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-40.0, min(40.0, self.integral))
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
#  ฟังก์ชันตรวจหาขอบโฟมสีขาว 11.5 นิ้ว (Foam Height Cutoff)
# =============================================================================
def detect_foam_top_border(hsv_img):
    """
    ตรวจหาขอบโฟมสีขาวบนสุดของสนาม (ความสูงโฟม 11.5 นิ้ว / ~29.2 cm)
    เพื่อใช้ตัดการจับภาพวัตถุใดๆ ที่อยู่เหนือขอบโฟม (เพดาน/ไฟห้อง/สิ่งรบกวนภายนอกสนาม)
    """
    white_mask = cv2.inRange(hsv_img, np.array([0, 0, 140]), np.array([180, 70, 255]))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    y_coords = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 800:
            x, y, w, h = cv2.boundingRect(cnt)
            if w > 50 and w > 1.2 * h:
                y_coords.append(y + h)
                
    return min(y_coords) if y_coords else None

# =============================================================================
#  ฟังก์ชันกรองสีและจำแนกรูปทรงเป้าหมาย
# =============================================================================
def get_color_mask(hsv_img, color, s_min=70, v_min=40):
    """กรองเฉพาะสีเป้าหมายด้วย HSV Range พร้อม Morphological Filter"""
    if color == "red":
        mask1 = cv2.inRange(hsv_img, np.array([0, s_min, v_min]), np.array([12, 255, 255]))
        mask2 = cv2.inRange(hsv_img, np.array([168, s_min, v_min]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "blue":
        mask = cv2.inRange(hsv_img, np.array([95, s_min, v_min]), np.array([145, 255, 255]))
    elif color == "yellow":
        mask = cv2.inRange(hsv_img, np.array([18, s_min, v_min]), np.array([38, 255, 255]))
    elif color == "green":
        mask = cv2.inRange(hsv_img, np.array([35, s_min, v_min]), np.array([88, 255, 255]))
    elif color == "any":
        # รวมทั้ง 4 เฉดสีเป้าหมาย
        m_r1 = cv2.inRange(hsv_img, np.array([0, s_min, v_min]), np.array([12, 255, 255]))
        m_r2 = cv2.inRange(hsv_img, np.array([168, s_min, v_min]), np.array([180, 255, 255]))
        m_b = cv2.inRange(hsv_img, np.array([95, s_min, v_min]), np.array([145, 255, 255]))
        m_y = cv2.inRange(hsv_img, np.array([18, s_min, v_min]), np.array([38, 255, 255]))
        m_g = cv2.inRange(hsv_img, np.array([35, s_min, v_min]), np.array([88, 255, 255]))
        mask = cv2.bitwise_or(cv2.bitwise_or(m_r1, m_r2), cv2.bitwise_or(cv2.bitwise_or(m_b, m_y), m_g))
    else:
        mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def detect_shape(contour):
    """
    จำแนกรูปทรงเป้าหมาย 4 แบบตามภาพตัวอย่าง:
      1. circle: วงกลม
      2. rectangle_horizontal: สี่เหลี่ยมผืนผ้าแนวนอน (กว้าง > ยาว)
      3. rectangle_vertical: สี่เหลี่ยมผืนผ้าแนวตั้ง (ยาว > กว้าง)
      4. square: สี่เหลี่ยมจัตุรัส
    """
    shape = "unidentified"
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if area < 1500 or peri == 0:
        return shape, None
        
    (x, y, w, h) = cv2.boundingRect(contour)
    rect = cv2.minAreaRect(contour)
    (cx, cy), (rw, rh), angle = rect
    if rw == 0 or rh == 0:
        return shape, None
        
    min_rect_area = rw * rh
    extent = area / float(min_rect_area) if min_rect_area > 0 else 0.0
    norm_ar = max(rw, rh) / min(rw, rh) if min(rw, rh) > 0 else 1.0

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / float(hull_area) if hull_area > 0 else 0.0

    circularity = (4.0 * np.pi * area) / (peri * peri)

    # 1. วงกลม (Circle)
    if circularity > 0.52 and norm_ar < 1.38:
        shape = "circle"
    # 2. สี่เหลี่ยมจัตุรัส (Square) - อัตราส่วนด้านใกล้เคียง 1.0 (0.80 ถึง 1.25)
    elif extent > 0.55 and solidity > 0.70 and 0.80 <= (w / float(h)) <= 1.25:
        shape = "square"
    # 3. สี่เหลี่ยมผืนผ้าแนวนอน (Horizontal Rectangle) - กว้าง > สูง
    elif extent > 0.45 and (w / float(h)) > 1.25:
        shape = "rectangle_horizontal"
    # 4. สี่เหลี่ยมผืนผ้าแนวตั้ง (Vertical Rectangle) - สูง > กว้าง
    elif extent > 0.45 and (w / float(h)) < 0.80:
        shape = "rectangle_vertical"
    else:
        # Fallback กรณีทั่วไป
        if w > 1.25 * h:
            shape = "rectangle_horizontal"
        elif h > 1.25 * w:
            shape = "rectangle_vertical"
        else:
            shape = "square"
            
    return shape, None

def dummy_callback(val):
    pass

# =============================================================================
#  Thread 2: Vision & Target Detection Thread (Thread-Safe Producer)
# =============================================================================
class VisionThread:
    """
    Thread 2: Vision & Target Detection Thread
    ==========================================
    เธรดประมวลผลภาพกล้อง 30 FPS ดึงเฟรมสด (Drop Old Queue) และตัดภาพเหนือโฟม 11.5 นิ้ว
    รองรับ Multi-Target Tracking & List Mode (ยิงหลายเป้า/หลายทรง/หลายสีในการวิ่งรอบเดียว)
    """
    def __init__(self, ep_camera, target_color="any", target_shape="any", target_list=None):
        self.ep_camera = ep_camera
        self.target_color = target_color
        self.target_shape = target_shape
        # target_list: รายการเป้าหมาย เช่น [{"color": "red", "shape": "circle"}, {"color": "blue", "shape": "square"}]
        self.target_list = target_list if target_list is not None else []
        
        self.frame_queue = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.running = False
        
        # Shared State Variables
        self.target_detected = False
        self.target_center = None       # (cx, cy)
        self.target_area = 0.0
        self.detected_shape = "unidentified"
        self.detected_color = "unidentified"
        self.spotted_frames = 0
        self.latest_frame = None
        self.frame_size = (640, 360)
        self.foam_top_y = None

        # Multi-Target Memory: บันทึกประวัติเป้าหมายที่ยิงแล้วเพื่อป้องกันยิงเป้าเดิมซ้ำ
        self.shot_targets = []  # List of dict: [{"color": ..., "shape": ..., "center": (x,y), "time": timestamp}]
        self.shot_cooldown_sec = 15.0  # Cooldown สำหรับเป้าเดิม
        
        # Trackbars Controls
        self.win_name = "Robot Vision (Thread 2 - Camera Engine)"
        self.ctrl_win = "Vision Controls & Calibration"
        self.trackbars_created = False
        
        self.thread = None

    def start(self):
        if not self.running:
            self.running = True
            print(f"[VisionThread] 📷 เริ่มทำงานเธรดประมวลผลภาพ (Multi-Target Mode | Filter: {self.target_color} {self.target_shape})...")
            try:
                self.ep_camera.start_video_stream(display=False)
            except Exception:
                pass
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        try:
            self.ep_camera.stop_video_stream()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("[VisionThread] 🛑 หยุดการทำงานเธรดภาพเรียบร้อย")

    def get_target_info(self):
        """Thread-safe Method สำหรับเธรดอื่นดึงข้อมูลเป้าหมายล่าสุด"""
        with self.lock:
            return (
                self.target_detected,
                self.target_center,
                self.detected_shape,
                self.target_area,
                self.frame_size[0],
                self.frame_size[1],
                self.detected_color
            )

    def set_target_filter(self, color, shape, target_list=None):
        with self.lock:
            self.target_color = color
            self.target_shape = shape
            if target_list is not None:
                self.target_list = target_list

    def mark_target_as_shot(self, color, shape, center):
        """บันทึกเป้าหมายที่ยิงสำเร็จลงความจำเพื่อไม่ให้ยิงซ้ำในการวิ่งรอบนี้"""
        with self.lock:
            record = {
                "color": color,
                "shape": shape,
                "center": center,
                "time": time.time()
            }
            self.shot_targets.append(record)
            print(f"[VisionThread] 🎯 บันทึกเป้าหมายที่ยิงแล้ว: สี={color}, ทรง={shape}, ตำแหน่ง={center}")

    def clear_shot_history(self):
        with self.lock:
            self.shot_targets.clear()

    def _is_already_shot(self, color, shape, center, threshold_px=80):
        """ตรวจสอบว่าเป้าหมายนี้ถูกยิงไปแล้วหรือยัง (เช็คสี, ทรง และพิกัดใกล้เคียง)"""
        now = time.time()
        for shot in self.shot_targets:
            if (now - shot["time"]) > self.shot_cooldown_sec:
                continue
            # ถ้าเป็นสีและทรงเดียวกัน หรือมีพิกัดภาพใกล้เคียงกัน ให้ถือว่าเป็นเป้าเดิมที่ยิงแล้ว
            same_type = (shot["color"] == color and shot["shape"] == shape)
            close_position = False
            if center and shot["center"]:
                dist = np.hypot(center[0] - shot["center"][0], center[1] - shot["center"][1])
                if dist < threshold_px:
                    close_position = True
            if same_type or close_position:
                return True
        return False

    def _setup_trackbars(self):
        try:
            cv2.namedWindow(self.ctrl_win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.ctrl_win, 420, 260)
            
            c_idx = COLOR_PRESETS.index(self.target_color) if self.target_color in COLOR_PRESETS else 0
            s_idx = SHAPE_PRESETS.index(self.target_shape) if self.target_shape in SHAPE_PRESETS else 4
            
            cv2.createTrackbar("Color Filter", self.ctrl_win, c_idx, len(COLOR_PRESETS)-1, dummy_callback)
            cv2.createTrackbar("Shape Filter", self.ctrl_win, s_idx, len(SHAPE_PRESETS)-1, dummy_callback)
            cv2.createTrackbar("Cutoff Active", self.ctrl_win, 1, 1, dummy_callback)
            cv2.createTrackbar("Foam Top Line (%)", self.ctrl_win, 18, 50, dummy_callback)
            cv2.createTrackbar("Min Area (x100)", self.ctrl_win, 18, 100, dummy_callback)
            self.trackbars_created = True
        except Exception as e:
            print(f"[VisionThread] Trackbar note: {e}")

    def _run_loop(self):
        last_time = time.time()
        fps = 0.0

        while self.running:
            try:
                raw_img = self.ep_camera.read_cv2_image(strategy="newest", timeout=0.2)
            except Exception:
                time.sleep(0.01)
                continue
                
            if raw_img is None:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = now - last_time
            if dt > 0:
                fps = 1.0 / dt
            last_time = now

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self.frame_queue.put_nowait(raw_img)
            except queue.Full:
                pass

            try:
                img = self.frame_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            h, w = img.shape[:2]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            if not self.trackbars_created:
                self._setup_trackbars()

            cutoff_active = True
            foam_top_pct = 0.18
            min_area = 1800
            if self.trackbars_created:
                try:
                    c_i = cv2.getTrackbarPos("Color Filter", self.ctrl_win)
                    s_i = cv2.getTrackbarPos("Shape Filter", self.ctrl_win)
                    cutoff_active = cv2.getTrackbarPos("Cutoff Active", self.ctrl_win) == 1
                    foam_line_val = cv2.getTrackbarPos("Foam Top Line (%)", self.ctrl_win)
                    if foam_line_val >= 0:
                        foam_top_pct = foam_line_val / 100.0
                    min_area = cv2.getTrackbarPos("Min Area (x100)", self.ctrl_win) * 100
                    
                    with self.lock:
                        self.target_color = COLOR_PRESETS[min(c_i, len(COLOR_PRESETS)-1)]
                        self.target_shape = SHAPE_PRESETS[min(s_i, len(SHAPE_PRESETS)-1)]
                except Exception:
                    pass

            with self.lock:
                cur_color = self.target_color
                cur_shape = self.target_shape
                cur_target_list = list(self.target_list)

            # -------------------------------------------------------------
            #  เส้นขอบโฟม 11.5 นิ้วแบบคงที่ (Static Foam Cutoff Line)
            # -------------------------------------------------------------
            foam_top_y = int(h * foam_top_pct) if cutoff_active else None
            self.foam_top_y = foam_top_y

            mask = get_color_mask(hsv, cur_color)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            roi_top = int(h * 0.08)
            roi_bottom = int(h * 0.78)
            roi_left = int(w * 0.02)
            roi_right = int(w * 0.98)

            best_cnt = None
            max_area = 0.0
            found_shape = "unidentified"
            found_color = "unidentified"
            found_center = None

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area >= min_area:
                    x, y, cw, ch = cv2.boundingRect(cnt)
                    cx_cnt = x + cw // 2
                    cy_cnt = y + ch // 2
                    cnt_center = (cx_cnt, cy_cnt)

                    # ⛔ กรองตัดวัตถุเหนือขอบโฟม 11.5 นิ้วออกเด็ดขาด (เหนือเส้นตัดคงที่)
                    if cutoff_active and foam_top_y is not None:
                        if y < foam_top_y or cy_cnt < foam_top_y:
                            cv2.putText(img, "IGNORED (Above Foam Line)", (x, y - 5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                            continue

                    # กรองตัดวัตถุขนาดใหญ่ผิดปกติ (ยอมรับเป้าหมายระยะใกล้ได้สูงถึง 120000 area)
                    if area > 120000 or cw > 630:
                        continue

                    # ยอมรับเป้าหมายระดับต่ำบริเวณส่วนล่างของภาพเมื่อก้มกล้องลง
                    if cy_cnt < roi_top or cx_cnt < roi_left or cx_cnt > roi_right:
                        continue

                    shape, _ = detect_shape(cnt)
                    
                    # จำแนกสีเฉพาะของ Contour (กรณี cur_color == "any")
                    if cur_color == "any":
                        roi_hsv = hsv[y:y+ch, x:x+cw]
                        c_scores = {c: cv2.countNonZero(get_color_mask(roi_hsv, c)) for c in ["red", "blue", "yellow", "green"]}
                        cnt_color = max(c_scores, key=c_scores.get) if max(c_scores.values()) > 50 else "any"
                    else:
                        cnt_color = cur_color

                    # ตรวจสอบว่าถูกยิงไปแล้วในรอบนี้หรือไม่ (Multi-Target Memory)
                    if self._is_already_shot(cnt_color, shape, cnt_center):
                        cv2.rectangle(img, (x, y), (x + cw, y + ch), (128, 128, 128), 1)
                        cv2.putText(img, f"[SHOT] {cnt_color} {shape}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
                        continue

                    # วาดป้ายชื่อและทรงรอบ Contour ทุกอันที่ผ่านเกณฑ์
                    cv2.rectangle(img, (x, y), (x + cw, y + ch), (255, 200, 0), 1)
                    cv2.putText(img, f"{cnt_color} {shape} (a:{int(area)})", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                    # ตรวจสอบเงื่อนไขกับ Target List หรือ Color/Shape Filter
                    matched = False
                    if cur_target_list:
                        for target_spec in cur_target_list:
                            req_c = target_spec.get("color", "any")
                            req_s = target_spec.get("shape", "any")
                            c_ok = (req_c == "any" or req_c == cnt_color)
                            s_ok = (req_s == "any" or req_s == shape or (req_s == "rectangle" and "rectangle" in shape))
                            if c_ok and s_ok:
                                matched = True
                                break
                    else:
                        c_ok = (cur_color == "any" or cur_color == cnt_color)
                        s_ok = (cur_shape == "any" or cur_shape == shape or (cur_shape == "rectangle" and "rectangle" in shape))
                        matched = c_ok and s_ok

                    if matched and area > max_area:
                        max_area = area
                        best_cnt = cnt
                        found_shape = shape
                        found_color = cnt_color
                        rect = cv2.minAreaRect(cnt)
                        found_center = (int(rect[0][0]), int(rect[0][1]))

            with self.lock:
                self.frame_size = (w, h)
                if best_cnt is not None:
                    self.spotted_frames += 1
                    if self.spotted_frames >= 2:
                        self.target_detected = True
                        self.target_center = found_center
                        self.target_area = max_area
                        self.detected_shape = found_shape
                        self.detected_color = found_color
                        
                    cv2.drawContours(img, [best_cnt], -1, (0, 255, 0), 2)
                    if found_center:
                        cv2.circle(img, found_center, 6, (255, 0, 0), -1)
                else:
                    self.spotted_frames = 0
                    self.target_detected = False
                    self.target_center = None
                    self.target_area = 0.0
                    self.detected_shape = "unidentified"
                    self.detected_color = "unidentified"

                self.latest_frame = img.copy()

            # -------------------------------------------------------------
            #  แสดงผลกล้องและ HUD Overlay บ่งบอกสถานะการทำงาน
            # -------------------------------------------------------------
            # วาดเส้นขอบโฟม 11.5 นิ้ว (สีขาวขีดจางๆ)
            if foam_top_y is not None:
                cv2.line(img, (0, foam_top_y), (w, foam_top_y), (255, 255, 255), 2)
                cv2.putText(img, "--- 11.5\" FOAM WALL TOP LINE ---", (10, foam_top_y - 6), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # วาด Scope ROI
            cv2.rectangle(img, (roi_left, roi_top), (roi_right, roi_bottom), (255, 255, 0), 1)
            
            # แถบสเปกตรัมสถานะบนภาพ
            cv2.rectangle(img, (0, 0), (w, 35), (20, 20, 20), -1)
            shot_cnt = len(self.shot_targets)
            status_txt = f"FPS:{fps:.1f} | Filter: [{cur_color.upper()}] [{cur_shape.upper()}] | Shot: {shot_cnt} | Target: {self.detected_color} {self.detected_shape}"
            status_color = (0, 255, 0) if self.target_detected else (0, 255, 255)
            cv2.putText(img, status_txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)

            cv2.imshow(self.win_name, img)
            cv2.waitKey(1)

# =============================================================================
#  Thread 3: Gimbal Tracking & Fire Control Thread (Visual Servoing Consumer)
# =============================================================================
class GimbalTrackingThread:
    """
    Thread 3: Gimbal Tracking & Fire Control Thread
    ===============================================
    เธรดควบคุมป้อมปืนและการยิงเป้าหมาย (Consumer)
    - ใช้ safe_gimbal_drive และ safe_blaster_fire ร่วมกับ hardware_cmd_lock ป้องกันคำสั่งพอร์ตชนกัน
    """
    def __init__(self, ep_gimbal, ep_blaster, vision_thread, ep_led=None):
        self.ep_gimbal = ep_gimbal
        self.ep_blaster = ep_blaster
        self.ep_led = ep_led
        self.vision_thread = vision_thread
        
        self.pid_yaw = PIDController(kp=80.0, ki=0.0, kd=10.0, limits=(-150, 150))
        self.pid_pitch = PIDController(kp=80.0, ki=0.0, kd=10.0, limits=(-100, 100))
        
        self.target_offset_x = 0.50  # 50% width
        self.target_offset_y = 0.40  # 40% height
        
        self.is_aiming = False
        self.has_fired = False
        self.running = False
        
        self.aim_trigger_event = threading.Event()
        self.aim_completed_event = threading.Event()
        
        self.thread = None

    def start(self):
        if not self.running:
            self.running = True
            print("[GimbalTrackingThread] 🎯 เริ่มทำงานเธรดควบคุมการยิง (Thread 3)...")
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        self.aim_trigger_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.stop_gimbal()
        print("[GimbalTrackingThread] 🛑 หยุดการทำงานเธรดป้อมปืนเรียบร้อย")

    def stop_gimbal(self):
        safe_gimbal_drive(self.ep_gimbal, pitch_speed=0, yaw_speed=0)

    def trigger_shoot(self, timeout=4.0):
        self.has_fired = False
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        self.aim_completed_event.clear()
        self.is_aiming = True
        
        self.aim_trigger_event.set()
        completed = self.aim_completed_event.wait(timeout=timeout)
        if not completed:
            print("[GimbalTrackingThread] ⚠️ เล็งยิงนานเกินกำหนด (Timeout)")
            
        self.is_aiming = False
        self.stop_gimbal()
        return self.has_fired

    def _do_fire(self, target_color="any", target_shape="any", center=None):
        self.has_fired = True
        print(f"[GimbalTrackingThread] 💥💥 ยิงสำเร็จเรียบร้อย! (FIRE CONFIRMED: {target_color} {target_shape})")
        # บันทึกประวัติเป้าหมายที่ยิงสำเร็จลง VisionThread
        if self.vision_thread:
            self.vision_thread.mark_target_as_shot(target_color, target_shape, center)

        if self.ep_led is not None:
            with hardware_cmd_lock:
                try:
                    from robomaster import led
                    self.ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_FLASH)
                except Exception:
                    pass
        safe_blaster_fire(self.ep_blaster)
            
        time.sleep(0.4)
        if self.ep_led is not None:
            with hardware_cmd_lock:
                try:
                    from robomaster import led
                    self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
                except Exception:
                    pass

    def _run_loop(self):
        while self.running:
            self.aim_trigger_event.wait()
            if not self.running:
                break
                
            aim_start_time = time.time()
            locked_frames = 0
            lost_frames = 0
            
            while self.running and self.is_aiming:
                detected, center, shape, area, fw, fh, color = self.vision_thread.get_target_info()
                
                if detected and center is not None:
                    lost_frames = 0
                    cx, cy = center
                    
                    err_x = (cx / float(fw)) - self.target_offset_x
                    err_y = self.target_offset_y - (cy / float(fh))
                    
                    yaw_speed = self.pid_yaw.compute(err_x)
                    pitch_speed = self.pid_pitch.compute(err_y)
                    
                    # ปรับผ่อนปรน Lock Zone (±6% กว้าง, ±7% สูง) เพื่อให้ล็อกยิงได้แม่นยำและรวดเร็ว
                    is_locked = abs(err_x) < 0.06 and abs(err_y) < 0.07
                    
                    # Fallback: เล็งเกิน 1.2 วินาทีแล้วเป้าใกล้จุดศูนย์กลาง (±12%) ให้สั่งยิงทันที
                    elapsed = time.time() - aim_start_time
                    force_lock = (elapsed > 1.2) and (abs(err_x) < 0.12 and abs(err_y) < 0.12)
                    
                    if is_locked or force_lock:
                        self.stop_gimbal()
                        locked_frames += 1
                        if locked_frames >= 2 or force_lock:
                            self._do_fire(target_color=color, target_shape=shape, center=center)
                            self.is_aiming = False
                            break
                    else:
                        locked_frames = 0
                        safe_gimbal_drive(self.ep_gimbal, pitch_speed=pitch_speed, yaw_speed=yaw_speed)
                else:
                    lost_frames += 1
                    self.stop_gimbal()
                    if lost_frames >= 10:
                        print("[GimbalTrackingThread] ⚠️ เป้าหมายหายไปขณะเล็งยิง")
                        self.is_aiming = False
                        break
                        
                time.sleep(0.02)
                
            self.stop_gimbal()
            self.aim_trigger_event.clear()
            self.aim_completed_event.set()

# =============================================================================
#  Thread 4: State Machine / Coordinator Thread (Finite State Machine - FSM)
# =============================================================================
class RobotState:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    TARGET_SPOTTED = "TARGET_SPOTTED"
    AIM_AND_SHOOT = "AIM_AND_SHOOT"
    COOLDOWN = "COOLDOWN"
    MISSION_COMPLETE = "MISSION_COMPLETE"

class RobotCoordinator:
    def __init__(self, vision_thread, tracking_thread):
        self.vision_thread = vision_thread
        self.tracking_thread = tracking_thread
        
        self.lock = threading.Lock()
        self.current_state = RobotState.IDLE
        self.cooldown_until = 0.0
        self.shot_target_cells = set()
        
        self.running = False
        self.thread = None

    def start(self):
        if not self.running:
            self.running = True
            print("[RobotCoordinator] ⚙️ เริ่มทำงานเธรดประสานงาน FSM (Thread 4)...")
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("[RobotCoordinator] 🛑 หยุดการทำงาน Coordinator เรียบร้อย")

    def get_state(self):
        with self.lock:
            return self.current_state

    def set_state(self, new_state):
        with self.lock:
            if self.current_state != new_state:
                print(f"[Coordinator FSM] 🔄 เปลี่ยนสถานะ: {self.current_state} ──► {new_state}")
                self.current_state = new_state

    def is_in_cooldown(self):
        return time.time() < self.cooldown_until

    def mark_target_shot(self, cell):
        with self.lock:
            self.shot_target_cells.add(cell)
            self.cooldown_until = time.time() + 20.0

    def _run_loop(self):
        while self.running:
            state = self.get_state()
            
            if state == RobotState.NAVIGATING:
                if not self.is_in_cooldown():
                    detected, center, shape, area, w, h, color = self.vision_thread.get_target_info()
                    if detected:
                        self.set_state(RobotState.TARGET_SPOTTED)
                        
            time.sleep(0.05)

# =============================================================================
#  AutoShooter Orchestrator (Multi-Thread Integration Container)
# =============================================================================
class AutoShooter:
    def __init__(self, ep_camera, ep_gimbal, ep_blaster, color="any", shape="any", target_list=None, ep_led=None):
        self.vision_thread = VisionThread(ep_camera, target_color=color, target_shape=shape, target_list=target_list)
        self.tracking_thread = GimbalTrackingThread(ep_gimbal, ep_blaster, self.vision_thread, ep_led=ep_led)
        self.coordinator = RobotCoordinator(self.vision_thread, self.tracking_thread)

    @property
    def target_detected(self):
        status, _, _, _, _, _, _ = self.vision_thread.get_target_info()
        return status

    def set_target_filter(self, color="any", shape="any", target_list=None):
        self.vision_thread.set_target_filter(color, shape, target_list=target_list)

    def clear_shot_history(self):
        self.vision_thread.clear_shot_history()

    def start(self):
        self.vision_thread.start()
        self.tracking_thread.start()
        self.coordinator.start()

    def stop(self):
        self.coordinator.stop()
        self.tracking_thread.stop()
        self.vision_thread.stop()

    def stop_aiming(self):
        self.tracking_thread.stop_gimbal()

    def execute_shoot(self, timeout=4.0):
        self.coordinator.set_state(RobotState.AIM_AND_SHOOT)
        fired = self.tracking_thread.trigger_shoot(timeout=timeout)
        self.coordinator.cooldown_until = time.time() + 10.0
        self.coordinator.set_state(RobotState.NAVIGATING)
        return fired
