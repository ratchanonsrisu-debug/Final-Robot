"""
test_detection.py — เครื่องมือทดสอบและปรับแต่งระบบตรวจจับเป้าหมาย (Target Detection & Aiming Test Bench)

การใช้งาน:
    python test_detection.py                   # พยายามเชื่อมต่อหุ่น RoboMaster (ถ้าไม่เจอ จะสลับไปกล้อง Webcam/Sim อัตโนมัติ)
    python test_detection.py --webcam 0         # ใช้กล้องคอมพิวเตอร์ (Webcam index 0)
    python test_detection.py --file video.mp4   # ทดสอบกับไฟล์วิดีโอหรือรูปภาพ
    python test_detection.py --sim              # จำลองภาพเป้าหมายเคลื่อนที่ (Synthetic Test Pattern)
    python test_detection.py --color red --shape rectangle # กำหนดเป้าหมายเริ่มต้น

ปุ่มคีย์บอร์ดลัดขณะรัน:
    [c]       : สลับสีเป้าหมาย (red -> blue -> yellow -> green -> custom)
    [s]       : สลับทรงเป้าหมาย (rectangle -> square -> circle -> any)
    [m]       : เปลี่ยนโหมดมุมมอง (Overlay / Side-by-Side / Mask Only)
    [a]       : เปิด/ปิด การหัน Gimbal ตามเป้าหมาย (กรณีต่อหุ่นจริง)
    [SPACE]   : สั่งยิงอินฟราเรด IR Fire (กรณีต่อหุ่นจริง)
    [r]       : Recenter หัน Gimbal กลับตรงกลาง (กรณีต่อหุ่นจริง)
    [p]       : พิมพ์ค่า HSV Calibration ปัจจุบันออกทาง Console (นำไปแปะในโค้ดได้เลย)
    [q] / ESC : ออกจากโปรแกรม
"""

import argparse
import sys
import time
import math
import cv2
import numpy as np

# นำเข้าฟังก์ชันและคลาสจาก vision_shooter
try:
    from vision_shooter import PIDController, get_color_mask, detect_shape, detect_foam_borders
except ImportError:
    # เผื่อกรณีรันจากไดเรกทอรีอื่น
    from .vision_shooter import PIDController, get_color_mask, detect_shape, detect_foam_borders

# นำเข้า SDK robomaster ถ้ามี
HAS_ROBOMASTER = False
try:
    from robomaster import robot, blaster
    HAS_ROBOMASTER = True
except ImportError:
    HAS_ROBOMASTER = False

COLOR_PRESETS = {
    "red": {"h_min": 0, "h_max": 10, "s_min": 90, "s_max": 255, "v_min": 50, "v_max": 255, "h_min2": 170, "h_max2": 180},
    "blue": {"h_min": 95, "h_max": 145, "s_min": 80, "s_max": 255, "v_min": 45, "v_max": 255},
    "yellow": {"h_min": 20, "h_max": 35, "s_min": 90, "s_max": 255, "v_min": 50, "v_max": 255},
    "green": {"h_min": 35, "h_max": 85, "s_min": 80, "s_max": 255, "v_min": 45, "v_max": 255},
}

SHAPE_LIST = ["rectangle", "square", "circle", "any"]
COLOR_LIST = ["red", "blue", "yellow", "green", "custom"]


def create_synthetic_frame(t, target_color="red", target_shape="rectangle"):
    """สร้างภาพจำลองเป้าหมายเคลื่อนที่สำหรับทดสอบโดยไม่ต้องใช้กล้อง"""
    img = np.full((480, 640, 3), (40, 40, 40), dtype=np.uint8)
    
    # วาดพื้นหลังเส้นตาราง
    for x in range(0, 640, 40):
        cv2.line(img, (x, 0), (x, 480), (55, 55, 55), 1)
    for y in range(0, 480, 40):
        cv2.line(img, (0, y), (640, y), (55, 55, 55), 1)

    # คำนวณตำแหน่งเป้าหมายเคลื่อนที่แบบ Sine wave
    cx = int(320 + 200 * math.sin(t * 1.5))
    cy = int(240 + 100 * math.cos(t * 2.0))
    
    # เลือกสี BGR สำหรับวาดเป้าหมาย
    color_bgr_map = {
        "red": (30, 30, 220),
        "blue": (220, 80, 30),
        "yellow": (30, 220, 220),
        "green": (50, 200, 50),
        "custom": (30, 30, 220)
    }
    bgr = color_bgr_map.get(target_color, (30, 30, 220))

    w, h = 120, 70
    if target_shape == "square":
        w = h = 90
    elif target_shape == "circle":
        w = h = 90

    if target_shape == "circle":
        cv2.circle(img, (cx, cy), w // 2, bgr, -1)
        cv2.circle(img, (cx, cy), w // 2, (255, 255, 255), 2)
    else:
        x1, y1 = cx - w // 2, cy - h // 2
        x2, y2 = cx + w // 2, cy + h // 2
        cv2.rectangle(img, (x1, y1), (x2, y2), bgr, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # วาดเป้าหลอกรบกวน (Distractor target - เช่น ทรงกลมสีฟ้า)
    dist_x = int(320 - 180 * math.sin(t * 1.2))
    dist_y = int(240 - 80 * math.cos(t * 1.8))
    cv2.circle(img, (dist_x, dist_y), 35, (200, 50, 30), -1)

    # เพิ่มตัวอักษรบนเป้าหมาย
    cv2.putText(img, "TEST TARGET", (cx - 45, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return img


def get_mask_with_custom_hsv(hsv_img, color_preset, h_min, h_max, s_min, s_max, v_min, v_max):
    """สร้าง Mask จากค่า HSV ปัจจุบันหรือจาก Preset"""
    if color_preset != "custom" and color_preset in COLOR_PRESETS:
        return get_color_mask(hsv_img, color_preset)
    
    # Custom HSV range
    if h_min <= h_max:
        return cv2.inRange(hsv_img, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))
    else:
        # Hue wrap-around (เช่น สีแดง 170-180 รวมกับ 0-10)
        mask1 = cv2.inRange(hsv_img, np.array([0, s_min, v_min]), np.array([h_max, s_max, v_max]))
        mask2 = cv2.inRange(hsv_img, np.array([h_min, s_min, v_min]), np.array([180, s_max, v_max]))
        return cv2.bitwise_or(mask1, mask2)


def nothing(x):
    pass


def main():
    parser = argparse.ArgumentParser(description="Target Detection Test & Calibration Tool")
    parser.add_argument("--webcam", type=int, default=None, help="Index ของกล้อง Webcam (เช่น 0, 1)")
    parser.add_argument("--file", type=str, default=None, help="พาธไฟล์วิดีโอหรือรูปภาพสำหรับทดสอบ")
    parser.add_argument("--sim", action="store_true", help="โหมดจำลองเป้าหมายเคลื่อนที่ (Synthetic Pattern)")
    parser.add_argument("--color", type=str, default="red", choices=COLOR_LIST, help="สีเป้าหมายเริ่มต้น")
    parser.add_argument("--shape", type=str, default="rectangle", choices=SHAPE_LIST, help="ทรงเป้าหมายเริ่มต้น")
    args = parser.parse_args()

    print("=" * 65)
    print(" 🎯 RoboMaster Target Detection Test & Calibration System")
    print("=" * 65)

    # 1. จัดเตรียมแหล่งที่มาของภาพ (Video Source)
    cap = None
    ep_robot = None
    ep_camera = None
    ep_gimbal = None
    ep_blaster = None
    source_name = "Unknown"

    if args.sim:
        source_name = "Simulated Motion Pattern"
        print("💡 ใช้งานโหมดจำลอง Synthetic Target")
    elif args.file:
        source_name = f"File: {args.file}"
        cap = cv2.VideoCapture(args.file)
        if not cap.isOpened():
            print(f"❌ ไม่สามารถเปิดไฟล์: {args.file}")
            sys.exit(1)
        print(f"📁 โหลดไฟล์สื่อ: {args.file}")
    elif args.webcam is not None:
        source_name = f"Webcam #{args.webcam}"
        cap = cv2.VideoCapture(args.webcam)
        if not cap.isOpened():
            print(f"❌ ไม่สามารถเปิด Webcam index {args.webcam}")
            sys.exit(1)
        print(f"📷 โหลด Webcam #{args.webcam}")
    else:
        # พยายามเชื่อมต่อหุ่นยนต์จริงก่อน
        if HAS_ROBOMASTER:
            print("🤖 กำลังค้นหาและเชื่อมต่อหุ่นยนต์ RoboMaster EP...")
            try:
                ep_robot = robot.Robot()
                ep_robot.initialize(conn_type="ap")
                ep_camera = ep_robot.camera
                ep_gimbal = ep_robot.gimbal
                ep_blaster = ep_robot.blaster
                ep_camera.start_video_stream(display=False)
                source_name = "RoboMaster EP Camera"
                print("✅ เชื่อมต่อหุ่นยนต์และเริ่ม Video Stream สำเร็จ!")
            except Exception as e:
                print(f"⚠️ ไม่พบหุ่นยนต์จริง ({e}) -> สลับไปใช้ Webcam #0 หรือ Sim อัตโนมัติ")
                ep_robot = None

        if ep_robot is None:
            # ลองเปิด Webcam 0
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                source_name = "Webcam #0 (Fallback)"
                print("📷 ใช้กล้อง Webcam #0 สำหรับการทดสอบ")
            else:
                source_name = "Simulated Target (Fallback)"
                print("💡 ไม่พบกล้อง สลับเข้าโหมด Synthetic Simulation")

    # 2. สร้างหน้าต่างควบคุม Trackbars
    ctrl_win = "Detection Parameters & Tuning"
    cv2.namedWindow(ctrl_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(ctrl_win, 450, 480)

    color_idx = COLOR_LIST.index(args.color) if args.color in COLOR_LIST else 0
    shape_idx = SHAPE_LIST.index(args.shape) if args.shape in SHAPE_LIST else 0

    cv2.createTrackbar("Color Preset", ctrl_win, color_idx, len(COLOR_LIST) - 1, nothing)
    cv2.createTrackbar("Target Shape", ctrl_win, shape_idx, len(SHAPE_LIST) - 1, nothing)
    cv2.createTrackbar("H Min", ctrl_win, 0, 180, nothing)
    cv2.createTrackbar("H Max", ctrl_win, 10, 180, nothing)
    cv2.createTrackbar("S Min", ctrl_win, 90, 255, nothing)
    cv2.createTrackbar("S Max", ctrl_win, 255, 255, nothing)
    cv2.createTrackbar("V Min", ctrl_win, 50, 255, nothing)
    cv2.createTrackbar("V Max", ctrl_win, 255, 255, nothing)
    cv2.createTrackbar("Min Area (x100)", ctrl_win, 20, 200, nothing)  # 20 = 2000 px
    cv2.createTrackbar("Erode/Dilate", ctrl_win, 2, 5, nothing)
    cv2.createTrackbar("View Mode", ctrl_win, 0, 2, nothing)  # 0: Overlay, 1: Side-by-Side, 2: Mask Only
    cv2.createTrackbar("Auto Aim Gimbal", ctrl_win, 0, 1, nothing)

    # ตัวแปรระบบเล็งและ PID
    pid_yaw = PIDController(kp=40.0, ki=0.0, kd=10.0, limits=(-100, 100))
    pid_pitch = PIDController(kp=40.0, ki=0.0, kd=10.0, limits=(-80, 80))
    
    spotted_frames = 0
    locked_frames = 0
    start_time = time.time()
    prev_frame_time = time.time()

    print("\nคำสั่งลัดบนหน้าต่างภาพ:")
    print("  [c] : เปลี่ยน Preset สี")
    print("  [s] : เปลี่ยน Preset ทรง")
    print("  [m] : เปลี่ยนโหมดมุมมอง")
    print("  [a] : เปิด/ปิด Auto Aiming Gimbal")
    print("  [SPACE] : ทดสอบยิง IR")
    print("  [p] : พิมพ์ค่า HSV แปะโค้ด")
    print("  [q] : ออกจากโปรแกรม\n")

    main_win = "Target Detection Test Bench"
    cv2.namedWindow(main_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(main_win, 960, 540)

    try:
        while True:
            t = time.time() - start_time
            fps = 1.0 / max(0.001, time.time() - prev_frame_time)
            prev_frame_time = time.time()

            # อ่านภาพจากแหล่งที่มา
            img = None
            if ep_camera is not None:
                try:
                    img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                except Exception:
                    img = None
            elif cap is not None and cap.isOpened():
                ret, img = cap.read()
                if not ret:
                    # ถ้าเป็นวิดีโอ แล้วเล่นจบ ให้เล่นวนลูป
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, img = cap.read()
                    if not ret:
                        print("ไม่สามารถอ่านภาพจากวิดีโอ")
                        break

            # ถ้าไม่มีกล้องหรือเปิดอ่านไม่ได้ ให้ใช้ภาพจำลอง Synthetic Pattern
            if img is None:
                c_preset = COLOR_LIST[cv2.getTrackbarPos("Color Preset", ctrl_win)]
                s_preset = SHAPE_LIST[cv2.getTrackbarPos("Target Shape", ctrl_win)]
                img = create_synthetic_frame(t, target_color=c_preset, target_shape=s_preset)

            h, w = img.shape[:2]

            # อ่านค่าตั้งค่าจาก Trackbars
            c_idx = cv2.getTrackbarPos("Color Preset", ctrl_win)
            s_idx = cv2.getTrackbarPos("Target Shape", ctrl_win)
            cur_color = COLOR_LIST[c_idx]
            cur_shape = SHAPE_LIST[s_idx]

            h_min = cv2.getTrackbarPos("H Min", ctrl_win)
            h_max = cv2.getTrackbarPos("H Max", ctrl_win)
            s_min = cv2.getTrackbarPos("S Min", ctrl_win)
            s_max = cv2.getTrackbarPos("S Max", ctrl_win)
            v_min = cv2.getTrackbarPos("V Min", ctrl_win)
            v_max = cv2.getTrackbarPos("V Max", ctrl_win)
            min_area = cv2.getTrackbarPos("Min Area (x100)", ctrl_win) * 100
            morph_iters = cv2.getTrackbarPos("Erode/Dilate", ctrl_win)
            view_mode = cv2.getTrackbarPos("View Mode", ctrl_win)
            auto_aim = cv2.getTrackbarPos("Auto Aim Gimbal", ctrl_win) == 1

            # -------------------------------------------------------------
            #  ประมวลผลระบบมองภาพ (Vision Processing Pipeline)
            # -------------------------------------------------------------
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = get_mask_with_custom_hsv(hsv, cur_color, h_min, h_max, s_min, s_max, v_min, v_max)

            if morph_iters > 0:
                mask = cv2.erode(mask, None, iterations=morph_iters)
                mask = cv2.dilate(mask, None, iterations=morph_iters)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # กำหนดขอบเขต Scope ROI (Region of Interest)
            roi_top = int(h * 0.08)
            roi_bottom = int(h * 0.78)
            roi_left = int(w * 0.02)
            roi_right = int(w * 0.98)

            best_contour = None
            max_area = 0
            detected_info = ""

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue

                x, y, cw, ch = cv2.boundingRect(cnt)
                cx_cnt = x + cw // 2
                cy_cnt = y + ch // 2
                is_large = (area > 12000) or (cw > 200) or (ch > 200)

                # ตรวจสอบว่าเป้าหมายอยู่นอก Scope หรือไม่
                if not is_large and (cy_cnt < roi_top or cy_cnt > roi_bottom or cx_cnt < roi_left or cx_cnt > roi_right):
                    cv2.putText(img, "IGNORED (Scope)", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    continue

                shape, approx = detect_shape(cnt)

                # วาดป้ายชื่อและทรงที่ตรวจจับได้รอบๆ Contour ทุกอันที่ผ่าน min_area
                cv2.rectangle(img, (x, y), (x + cw, y + ch), (255, 200, 0), 1)
                cv2.putText(img, f"{shape} (a:{int(area)})", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                if (cur_shape == "any" or shape == cur_shape) and area > max_area:
                    max_area = area
                    best_contour = cnt
                    detected_info = f"{shape} area:{int(area)} ar:{cw/max(1,ch):.2f}"

            # วาดกรอบ Scope (ROI) และ Crosshair ตรงกลาง
            cv2.rectangle(img, (roi_left, roi_top), (roi_right, roi_bottom), (255, 255, 0), 1)
            cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=1)

            status_text = "SEARCHING"
            status_color = (200, 200, 200)
            err_x, err_y = 0.0, 0.0
            yaw_sp, pitch_sp = 0.0, 0.0

            if best_contour is not None:
                spotted_frames += 1
                M = cv2.moments(best_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # คำนวณ Error สัญญาณควบคุม (err_x > 0 เมื่อเป้าอยู่ขวา -> yaw_speed > 0 หันขวา)
                    err_x = (cx / w) - 0.5
                    err_y = 0.4 - (cy / h)

                    yaw_sp = pid_yaw.compute(err_x)
                    pitch_sp = pid_pitch.compute(err_y)

                    is_locked = abs(err_x) < 0.03 and abs(err_y) < 0.03

                    if is_locked:
                        locked_frames += 1
                        status_text = f"🎯 LOCKED! ({locked_frames}/5)"
                        status_color = (0, 255, 0)
                    else:
                        locked_frames = 0
                        status_text = f"👀 SPOTTED ({spotted_frames}f) - Aiming..."
                        status_color = (0, 165, 255)

                    # วาดจุด Centroid และ Contour หลัก
                    cv2.drawContours(img, [best_contour], -1, status_color, 2)
                    cv2.circle(img, (cx, cy), 6, (255, 0, 0), -1)
                    cv2.line(img, (w // 2, h // 2), (cx, cy), (0, 255, 255), 1)

                    # สั่ง Gimbal ถ้าเปิด Auto Aim และต่อหุ่นจริงอยู่
                    if auto_aim and ep_gimbal is not None:
                        if is_locked and locked_frames >= 5:
                            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                        else:
                            ep_gimbal.drive_speed(pitch_speed=pitch_sp, yaw_speed=yaw_sp)
            else:
                spotted_frames = 0
                locked_frames = 0
                pid_yaw.reset()
                pid_pitch.reset()
                if auto_aim and ep_gimbal is not None:
                    ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

            # -------------------------------------------------------------
            #  สร้างภาพผลลัพธ์และการแสดงผล (HUD Display)
            # -------------------------------------------------------------
            # แบนเนอร์หัวข้อบนภาพ
            cv2.rectangle(img, (0, 0), (w, 38), (20, 20, 20), -1)
            hud_info = f"Src: {source_name} | FPS: {fps:.1f} | Target: [{cur_color.upper()}] [{cur_shape.upper()}]"
            cv2.putText(img, hud_info, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            # แถบสถานะด้านล่าง
            cv2.rectangle(img, (0, h - 35), (w, h), (20, 20, 20), -1)
            cv2.putText(img, f"Status: {status_text}", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
            if detected_info:
                cv2.putText(img, detected_info, (w - 320, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            # แปลง Mask ให้เป็น 3 ช่องสีสำหรับทำ Side-by-Side
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            if view_mode == 1:
                # โหมด Side-by-Side (ภาพจริงคู่กับ Mask)
                combined = np.hstack((img, mask_bgr))
            elif view_mode == 2:
                # โหมด Mask อย่างเดียว
                combined = mask_bgr
            else:
                # โหมด Overlay ปกติ
                combined = img

            cv2.imshow(main_win, combined)

            # -------------------------------------------------------------
            #  การจัดการคีย์บอร์ดและคำสั่งควบคุม
            # -------------------------------------------------------------
            key = cv2.waitKey(15) & 0xFF
            if key == 27 or key == ord('q'):
                print("👋 ออกจากระบบทดสอบ")
                break
            elif key == ord('c'):
                # สลับสี
                new_c = (c_idx + 1) % len(COLOR_LIST)
                cv2.setTrackbarPos("Color Preset", ctrl_win, new_c)
                print(f"🎨 สลับสีเป็น: {COLOR_LIST[new_c]}")
            elif key == ord('s'):
                # สลับทรง
                new_s = (s_idx + 1) % len(SHAPE_LIST)
                cv2.setTrackbarPos("Target Shape", ctrl_win, new_s)
                print(f"📐 สลับทรงเป็น: {SHAPE_LIST[new_s]}")
            elif key == ord('m'):
                # เปลี่ยนมุมมอง
                new_v = (view_mode + 1) % 3
                cv2.setTrackbarPos("View Mode", ctrl_win, new_v)
            elif key == ord('a'):
                # สลับ Auto Aim
                new_a = 0 if auto_aim else 1
                cv2.setTrackbarPos("Auto Aim Gimbal", ctrl_win, new_a)
                print(f"🎯 Auto Aiming: {'ON' if new_a else 'OFF'}")
            elif key == 32:  # SPACE bar
                if ep_blaster is not None:
                    print("💥 [FIRE!] สั่งยิง IR Blaster")
                    ep_blaster.fire(fire_type=blaster.IR_FIRE, times=1)
                else:
                    print("💥 [FIRE SIMULATED] ยิง IR (โหมดจำลอง)")
            elif key == ord('r'):
                if ep_gimbal is not None:
                    print("🔄 Recenter Gimbal")
                    ep_gimbal.recenter().wait_for_completed()
            elif key == ord('p'):
                # พิมพ์โค้ด HSV สำหรับนำไปแปะใช้งาน
                print("\n" + "=" * 50)
                print(f"📋 HSV Calibration Output ({cur_color.upper()}):")
                if cur_color == "custom":
                    print(f"np.array([{h_min}, {s_min}, {v_min}]), np.array([{h_max}, {s_max}, {v_max}])")
                else:
                    print(f"Preset: '{cur_color}' -> {COLOR_PRESETS.get(cur_color)}")
                print(f"Min Contour Area: {min_area}")
                print("=" * 50 + "\n")

    except KeyboardInterrupt:
        print("\nถูกยกเลิกโดยผู้ใช้")

    # คืนทรัพยากร
    if ep_gimbal is not None:
        try:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        except Exception:
            pass
    if ep_camera is not None:
        try:
            ep_camera.stop_video_stream()
        except Exception:
            pass
    if ep_robot is not None:
        try:
            ep_robot.close()
        except Exception:
            pass
    if cap is not None:
        cap.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
