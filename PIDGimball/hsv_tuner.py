import json
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from robomaster import robot

import detectcolor as dc
# cd "d:\ROBOT-Final\Final-Robot\PIDGimball"
# python hsv_tuner.py            -> เริ่มจากช่วง HSV เต็ม (0-179 / 0-255 / 0-255)
# python hsv_tuner.py YELLOW     -> โหลดช่วง HSV ปัจจุบันของ YELLOW จาก detectcolor.COLOR_RANGES มาตั้งต้น

# --------------------------------------------------
# เครื่องมือปรับ HSV แบบสดๆ ผ่านกล้องจริง (แยกจาก detectcolor.py ไม่กระทบของเดิม)
# ใช้ตอนป้ายสีตรวจจับติดๆดับๆ / อยากรู้ระยะจริงที่กล้องเห็นป้าย ก่อนจะเอาค่าไปแก้ COLOR_RANGES จริง
#
# ขนาดป้ายที่รู้อยู่แล้ว (ใช้ตัวเดียวกับ detectcolor.py ไม่ประกาศซ้ำ):
#   สี่เหลี่ยมแนวตั้ง  6x9 ซม., สี่เหลี่ยมแนวนอน 9x6 ซม., วงกลม เส้นผ่านศูนย์กลาง 7 ซม. (รัศมี 3.5 ซม.)
# ระยะที่รับรู้ (แสดงบนจอเป็น ~xxcm) คำนวณจากขนาดพิกเซลที่วัดได้เทียบกับขนาดจริงข้างต้น
# ผ่านมุมมองกล้อง CAMERA_HFOV_DEG (ค่าเดียวกับใน detectcolor.py ถ้าจะแก้มุมมองกล้องให้ไปแก้ที่ไฟล์นั้นที่เดียว)
# --------------------------------------------------

PRESETS_PATH = os.path.join(dc.SAVE_DIR, "hsv_presets.json")

CONTROL_WIN = "HSV Controls"
LIVE_WIN = "HSV Tuner - Live"
MASK_WIN = "HSV Tuner - Mask (raw | ที่ใช้จริงหลัง morphology)"


def _nothing(_):
    pass


def create_trackbars(init_vals):
    h1min, h1max, smin, smax, vmin, vmax, wrap, h2min, h2max = init_vals
    cv2.namedWindow(CONTROL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WIN, 420, 320)
    cv2.createTrackbar("H1 min", CONTROL_WIN, h1min, 179, _nothing)
    cv2.createTrackbar("H1 max", CONTROL_WIN, h1max, 179, _nothing)
    cv2.createTrackbar("S min", CONTROL_WIN, smin, 255, _nothing)
    cv2.createTrackbar("S max", CONTROL_WIN, smax, 255, _nothing)
    cv2.createTrackbar("V min", CONTROL_WIN, vmin, 255, _nothing)
    cv2.createTrackbar("V max", CONTROL_WIN, vmax, 255, _nothing)
    cv2.createTrackbar("Wrap(red) 0/1", CONTROL_WIN, wrap, 1, _nothing)
    cv2.createTrackbar("H2 min", CONTROL_WIN, h2min, 179, _nothing)
    cv2.createTrackbar("H2 max", CONTROL_WIN, h2max, 179, _nothing)


def read_trackbars():
    g = lambda name: cv2.getTrackbarPos(name, CONTROL_WIN)
    return (g("H1 min"), g("H1 max"), g("S min"), g("S max"), g("V min"), g("V max"),
            g("Wrap(red) 0/1"), g("H2 min"), g("H2 max"))


def initial_trackbar_values():
    """โหลดช่วง HSV เริ่มต้นจากชื่อสีที่ระบุตอนรัน (ถ้ามี) มาตั้งเป็นจุดเริ่มปรับ แทนที่จะเริ่มจากศูนย์ทุกครั้ง"""
    if len(sys.argv) >= 2:
        key = sys.argv[1].upper()
        if key in dc.COLOR_RANGES:
            ranges = dc.COLOR_RANGES[key]["ranges"]
            (h1min, smin, vmin), (h1max, smax, vmax) = ranges[0]
            if len(ranges) > 1:
                (h2min, _, _), (h2max, _, _) = ranges[1]
                wrap = 1
            else:
                h2min, h2max, wrap = 0, 179, 0
            print(f"[โหลดค่าเริ่มต้นจาก COLOR_RANGES['{key}']]")
            return (h1min, h1max, smin, smax, vmin, vmax, wrap, h2min, h2max)
        print(f"[WARN] ไม่รู้จักสี '{sys.argv[1]}' (มี: {', '.join(dc.COLOR_RANGES.keys())}) เริ่มจากช่วงเต็มแทน")
    return (0, 179, 0, 255, 0, 255, 0, 0, 179)


def build_mask(hsv, vals):
    h1min, h1max, smin, smax, vmin, vmax, wrap, h2min, h2max = vals
    mask = cv2.inRange(hsv, (h1min, smin, vmin), (h1max, smax, vmax))
    if wrap:
        mask = mask | cv2.inRange(hsv, (h2min, smin, vmin), (h2max, smax, vmax))
    return mask


def ranges_from_vals(vals):
    h1min, h1max, smin, smax, vmin, vmax, wrap, h2min, h2max = vals
    ranges = [[[h1min, smin, vmin], [h1max, smax, vmax]]]
    if wrap:
        ranges.append([[h2min, smin, vmin], [h2max, smax, vmax]])
    return ranges


def print_paste_snippet(name, ranges):
    ranges_str = ", ".join(
        f"(({r[0][0]}, {r[0][1]}, {r[0][2]}), ({r[1][0]}, {r[1][1]}, {r[1][2]}))" for r in ranges
    )
    print("วางแทนในไฟล์ detectcolor.py ที่ COLOR_RANGES ได้เลย:")
    print(f'  "{name}": {{"th": "???", "ranges": [{ranges_str}]}},')


def load_presets():
    if os.path.exists(PRESETS_PATH):
        with open(PRESETS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def save_preset(vals):
    name = input("\nตั้งชื่อ preset (เช่น YELLOW_v2): ").strip()
    if not name:
        print("[ยกเลิก] ไม่ได้ตั้งชื่อ")
        return
    ranges = ranges_from_vals(vals)
    presets = load_presets()
    presets[name] = {"timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), "ranges": ranges}
    with open(PRESETS_PATH, "w", encoding="utf-8") as f:
        json.dump(presets, f, ensure_ascii=False, indent=2)
    print(f"[บันทึก] preset '{name}' -> {os.path.basename(PRESETS_PATH)}")
    print_paste_snippet(name, ranges)


def classify_shape(cnt, w, h, area):
    """ตัดสินรูปทรงจากความกลม/สัดส่วนกว้างสูง เหมือน detect_signs() ใน detectcolor.py (รวม fix convex hull เดียวกัน)
    แต่ไม่กรองด้วยขนาดพิกเซลที่ "ควรจะเป็น" ตามระยะสมมติ เพราะตอนจูนสีเราไม่รู้ระยะจริงล่วงหน้า"""
    hull = cv2.convexHull(cnt)
    hull_perimeter = cv2.arcLength(hull, True)
    circularity = (4 * math.pi * area / (hull_perimeter ** 2)) if hull_perimeter > 0 else 0.0
    if circularity >= dc.CIRCLE_MIN_CIRCULARITY:
        return "CIRCLE", circularity

    aspect = w / float(h) if h > 0 else 0.0
    is_vert = dc.ASPECT_VERT_RANGE[0] <= aspect <= dc.ASPECT_VERT_RANGE[1]
    is_horiz = dc.ASPECT_HORIZ_RANGE[0] <= aspect <= dc.ASPECT_HORIZ_RANGE[1]
    if is_vert and (not is_horiz or abs(aspect - dc._VERT_RATIO) <= abs(aspect - dc._HORIZ_RATIO)):
        return "VERTICAL", circularity
    if is_horiz:
        return "HORIZONTAL", circularity
    return "UNKNOWN", circularity


def estimate_distance_cm(pixel_size_px, real_size_cm, frame_w):
    """ประมาณระยะห่างจริง (ซม.) จากขนาดพิกเซลที่วัดได้ เทียบกับขนาดจริงที่รู้อยู่แล้ว ผ่านมุมมองกล้อง (pinhole model)
    สูตรกลับด้านจาก _px_to_cm_scale() ใน detectcolor.py ซึ่งสมมติระยะไว้ตายตัว - ตัวนี้คำนวณระยะจริงแทน"""
    if pixel_size_px <= 0:
        return None
    hfov_rad = math.radians(dc.CAMERA_HFOV_DEG)
    return (real_size_cm * frame_w) / (2 * pixel_size_px * math.tan(hfov_rad / 2))


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_gimbal = ep_robot.gimbal
    ep_camera = ep_robot.camera

    print("เปิดใช้งานกล้อง และเซ็ต Gimbal...")
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    init_vals = initial_trackbar_values()
    create_trackbars(init_vals)
    cv2.namedWindow(LIVE_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WIN, cv2.WINDOW_NORMAL)

    mouse_pos = {"x": -1, "y": -1, "clicked": False}

    def on_mouse(event, x, y, flags, param):
        mouse_pos["x"], mouse_pos["y"] = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_pos["clicked"] = True

    cv2.setMouseCallback(LIVE_WIN, on_mouse)

    print("=" * 60)
    print("HSV Tuner")
    print("WASD ที่หน้าต่างภาพ = หมุนกล้องเล็งไปที่ป้าย")
    print("ปรับสไลด์ที่หน้าต่าง 'HSV Controls' จนป้ายในภาพ 'Mask' ติดนิ่ง ไม่กระพริบ")
    print("เอาเมาส์วางบนภาพ = โชว์ค่า HSV ตรงจุดนั้นแบบสด, คลิกซ้าย = print ค่านั้นออก terminal ด้วย")
    print("กด p = print ค่า HSV ปัจจุบันแบบ paste ใส่ detectcolor.py ได้เลย (ไม่บันทึกไฟล์)")
    print("กด o = บันทึกค่าปัจจุบันเป็น preset ลงไฟล์ hsv_presets.json (ตั้งชื่อใน terminal)")
    print("กด ESC หรือ q = ออก")
    print("=" * 60)

    last_yaw_key_time = last_pitch_key_time = 0.0
    yaw_dir = pitch_dir = 0

    kernel = np.ones((5, 5), np.uint8)
    erode_kernel = np.ones((3, 3), np.uint8)

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            now = time.time()
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('a'), ord('A')):
                yaw_dir, last_yaw_key_time = -1, now
            elif key in (ord('d'), ord('D')):
                yaw_dir, last_yaw_key_time = 1, now
            if now - last_yaw_key_time > dc.MANUAL_KEY_HOLD_TIMEOUT:
                yaw_dir = 0

            if key in (ord('w'), ord('W')):
                pitch_dir, last_pitch_key_time = 1, now
            elif key in (ord('s'), ord('S')):
                pitch_dir, last_pitch_key_time = -1, now
            if now - last_pitch_key_time > dc.MANUAL_KEY_HOLD_TIMEOUT:
                pitch_dir = 0

            ep_gimbal.drive_speed(pitch_speed=pitch_dir * dc.MANUAL_TURN_SPEED,
                                   yaw_speed=yaw_dir * dc.MANUAL_TURN_SPEED)

            if frame is None:
                if key in (27, ord('q')):
                    break
                continue

            fh, fw = frame.shape[:2]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            vals = read_trackbars()
            mask_raw = build_mask(hsv, vals)
            mask = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.erode(mask, erode_kernel, iterations=1)

            display = frame.copy()
            rx, ry, rw, rh = dc.roi_rect_px(fw, fh)
            cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            found_count = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < dc.MIN_SIGN_AREA_PX:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if w == 0 or h == 0 or area / float(w * h) < dc.MIN_FILL_RATIO:
                    continue

                shape_type, circularity = classify_shape(cnt, w, h, area)
                found_count += 1

                if shape_type == "CIRCLE":
                    (ccx, ccy), radius = cv2.minEnclosingCircle(cnt)
                    diameter_px = 2 * radius
                    dist = estimate_distance_cm(diameter_px, dc.KNOWN_CIRCLE_DIAMETER_CM, fw)
                    size_label = f"d={diameter_px:.0f}px"
                elif shape_type in ("VERTICAL", "HORIZONTAL"):
                    real_w_cm, _ = dc.KNOWN_SIGN_SIZES_CM[shape_type]
                    dist = estimate_distance_cm(w, real_w_cm, fw)
                    size_label = f"{w}x{h}px"
                else:
                    dist = None
                    size_label = f"{w}x{h}px"

                color = (0, 255, 0) if shape_type != "UNKNOWN" else (150, 150, 150)
                cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                line1 = f"{shape_type} {size_label} circ={circularity:.2f}"
                line2 = f"~{dist:.0f}cm" if dist is not None else "dist=?"
                cv2.putText(display, line1, (x, max(y - 24, 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(display, line2, (x, max(y - 6, 34)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            mx, my = mouse_pos["x"], mouse_pos["y"]
            if 0 <= mx < fw and 0 <= my < fh:
                h_, s_, v_ = hsv[my, mx]
                cv2.drawMarker(display, (mx, my), (255, 255, 255), markerType=cv2.MARKER_CROSS,
                                markerSize=14, thickness=1)
                cv2.putText(display, f"HSV@cursor=({h_},{s_},{v_})", (10, fh - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if mouse_pos["clicked"]:
                    print(f"[คลิก] ({mx},{my}) -> HSV=({h_},{s_},{v_})")
                    mouse_pos["clicked"] = False

            info_lines = [
                "WASD=aim  p=print snippet  o=save preset  ESC/q=quit",
                f"signs found: {found_count}   wrap(red)={'ON' if vals[6] else 'off'}",
            ]
            y0 = 24
            for line in info_lines:
                cv2.putText(display, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                y0 += 22

            cv2.imshow(LIVE_WIN, display)
            mask_raw_bgr = cv2.cvtColor(mask_raw, cv2.COLOR_GRAY2BGR)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_raw_bgr, "raw", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(mask_bgr, "morph (ใช้จริง)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(MASK_WIN, np.hstack([mask_raw_bgr, mask_bgr]))

            if key == ord('p'):
                print_paste_snippet("PREVIEW", ranges_from_vals(vals))
            elif key == ord('o'):
                save_preset(vals)
            elif key in (27, ord('q')):
                print("\nออกจาก HSV Tuner")
                break
    except KeyboardInterrupt:
        print("\nหยุด HSV Tuner")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()


if __name__ == "__main__":
    main()
