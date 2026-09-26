# -*- coding: utf-8 -*-
"""
laser_calibrate.py -- Calibrate aim_offset_x (parallax กล้อง<->ป้อมปืน) ด้วยโหมด IR ของป้อมปืนเอง
ไม่ต้องยิงกระสุนเจลจริงเลย (ไม่มีอุปกรณ์เลเซอร์แยกต่างหาก -- "เลเซอร์" ที่ใช้คือ blaster.fire(fire_type=
INFRARED_FIRE) ตัวเดียวกับที่ detectcolor.py ใช้ในโหมด "fire" ปลอดภัยนั่นแหละ ไม่มีคำสั่งเปิดเลเซอร์แยกใน SDK)

วิธีใช้:
  1. แปะจุดอ้างอิง (เช่นจุด X เล็กๆ) บนผนัง/เป้า ที่ระยะ DISTANCE_CM (ต้องตรงกับค่าที่ตั้งไว้ใน
     detectcolor.py ด้วย) -- ไม่ต้องเป็นป้ายจริงก็ได้ ผนังเปล่าก็ใช้ได้ ขอแค่มีจุดอ้างอิงที่มองเห็นชัด
  2. WASD เล็ง gimbal จนจุดอ้างอิงนั้นอยู่ตรงกากบาทเหลืองกลางจอพอดี (จำลองสถานะตอนโค้ด detectcolor.py
     ล็อกเป้าสำเร็จด้วย err=0, offset=0 -- คือสมมติฐานตั้งต้นก่อนชดเชย parallax)
  3. กด f เพื่อ "เปิดค้าง" -- โปรแกรมจะสั่งยิง IR ซ้ำๆ ทุก ~0.25 วินาทีให้เอง (ปลอดภัย ไม่มีกระสุนเจลออก)
     ให้เห็นจุดต่อเนื่องในภาพ กด f อีกครั้งเพื่อปิด
  4. คลิกซ้ายที่ "จุด IR" ที่เห็นในภาพ (ตำแหน่งที่กระสุนจริงจะไปตกจริงถ้ายิงจากมุมนี้) โปรแกรมคำนวณ offset ให้ทันที
  5. กด 0-9 เพื่อเลือกว่ากำลัง calibrate "ป้ายลำดับที่เท่าไหร่" (ตรงกับ key ของ AIM_OFFSET_X_BY_INDEX ใน
     detectcolor.py, 0=ซ้ายสุด) แล้วกด Enter/Space เพื่อบันทึกค่า -- ทำซ้ำได้หลายตำแหน่ง/มุม
  6. กด ESC หรือ q เพื่อจบ โปรแกรมจะสรุปเป็น dict พร้อมก็อปวาง AIM_OFFSET_X_BY_INDEX ในไฟล์ detectcolor.py

*** สำคัญ: ใช้ได้เฉพาะแนวนอน (offset_x) เท่านั้น ***
IR เดินทางเป็นเส้นตรง แต่กระสุนเจลจริงตกท้องช้าง (projectile drop) ระหว่างบินเพราะแรงโน้มถ่วง
ค่า offset แนวตั้งที่เครื่องมือนี้คำนวณให้ (offset_y) จึง "ไม่ใช่" ค่าเดียวกับ PROJECTILE_DROP_COMPENSATION
ใน detectcolor.py -- ห้ามเอาไปแทนตรงๆ เด็ดขาด ค่าแนวตั้งสำหรับกระสุนจริงยังต้องใช้ fire_calibrate.py
(ยิงจริง+คลิกรอยกระสุน) ในโฟลเดอร์เดียวกันนี้แทน (offset_y ที่เห็นในนี้มีไว้ดูเฉยๆ ว่า IR เพี้ยนแนวตั้งแค่ไหน)

python laser_calibrate.py
"""

import os
import sys
import csv
import time
import math
from datetime import datetime

import cv2
from robomaster import robot, blaster

IR_FIRE_INTERVAL_S = 0.25  # ความถี่ยิง IR ซ้ำตอนกด f เปิดค้าง (วินาทีต่อนัด)

SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIB_LOG_PATH = os.path.join(SAVE_DIR, "laser_calibration_log.csv")

# ต้องตรงกับค่าที่ใช้ใน detectcolor.py ถึงจะแปลงเป็น ซม. ได้ถูกต้อง (ใช้แค่โชว์บนจอ ไม่กระทบผลลัพธ์ offset)
CAMERA_HFOV_DEG = 95.0
DISTANCE_CM = 100.0

MANUAL_TURN_SPEED = 60
MANUAL_KEY_HOLD_TIMEOUT = 0.15

_click_px = None  # (x, y) พิกเซลล่าสุดที่คลิก หรือ None ถ้ายังไม่คลิก


def _on_mouse(event, x, y, flags, param):
    global _click_px
    if event == cv2.EVENT_LBUTTONDOWN:
        _click_px = (x, y)


def _px_to_cm_scale(frame_w):
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    view_width_cm = 2 * DISTANCE_CM * math.tan(hfov_rad / 2)
    return view_width_cm / frame_w


def main():
    global _click_px
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_gimbal = ep_robot.gimbal
    ep_camera = ep_robot.camera
    ep_blaster = ep_robot.blaster

    print("=" * 70)
    print("laser_calibrate.py -- calibrate AIM_OFFSET_X_BY_INDEX ด้วยโหมด IR (ไม่ยิงกระสุนเจลจริง)")
    print("WASD เล็งให้จุดอ้างอิงอยู่ตรงกากบาทเหลืองกลางจอ แล้วกด f เพื่อเปิดค้างยิง IR ซ้ำๆ ให้เห็นจุด")
    print("คลิกซ้ายที่จุด IR ที่เห็นในภาพ, กด 0-9 เลือกลำดับป้าย, Enter/Space = บันทึกค่า")
    print("c = ล้างจุดคลิก, f = ปิด/เปิด IR สลับ, ESC/q = จบ+สรุปผล")
    print("*** offset_y ที่เห็นในนี้ใช้กับกระสุนจริงไม่ได้ (IR ไม่ตกท้องช้างเหมือนกระสุนเจล) ***")
    print("=" * 70)

    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    window_name = "RoboMaster Camera - laser_calibrate"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, _on_mouse)
    try:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
    except cv2.error:
        pass

    last_yaw_key_time = 0.0
    last_pitch_key_time = 0.0
    yaw_dir = 0
    pitch_dir = 0
    target_index = 0
    samples = []  # list ของ dict: {target_index, offset_x, offset_y_raw, timestamp}
    ir_beam_on = False
    last_ir_fire_time = 0.0

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            now = time.time()
            key = cv2.waitKey(1) & 0xFF

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

            if ord('0') <= key <= ord('9'):
                target_index = key - ord('0')
                print(f"[SELECT] กำลัง calibrate target_index={target_index}")

            if key == ord('c'):
                _click_px = None

            if key == ord('f'):
                ir_beam_on = not ir_beam_on
                print(f"\n[IR] {'เปิดค้าง (ยิง IR ซ้ำ)' if ir_beam_on else 'ปิดแล้ว'}")

            if ir_beam_on and (now - last_ir_fire_time) > IR_FIRE_INTERVAL_S:
                ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
                last_ir_fire_time = now

            offset_x_n = offset_y_n = None
            if frame is not None and _click_px is not None:
                fh, fw = frame.shape[:2]
                cx, cy = _click_px
                offset_x_n = 0.5 - (cx / fw)
                offset_y_n = 0.5 - (cy / fh)

            if key in (13, 32) and offset_x_n is not None:  # Enter/Space -> บันทึกค่า
                sample = {
                    "target_index": target_index,
                    "offset_x": offset_x_n,
                    "offset_y_raw": offset_y_n,
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                }
                samples.append(sample)
                is_new = not os.path.exists(CALIB_LOG_PATH)
                with open(CALIB_LOG_PATH, "a", newline="", encoding="utf-8-sig") as f:
                    writer = csv.writer(f)
                    if is_new:
                        writer.writerow(["timestamp", "target_index", "offset_x", "offset_y_raw_NOT_for_real_ammo"])
                    writer.writerow([sample["timestamp"], target_index, f"{offset_x_n:.4f}", f"{offset_y_n:.4f}"])
                print(f"\n>> บันทึกแล้ว target_index={target_index}  offset_x={offset_x_n:+.4f}  "
                      f"(offset_y_raw={offset_y_n:+.4f}, ใช้กับกระสุนจริงไม่ได้)")

            if frame is not None:
                fh, fw = frame.shape[:2]
                cx_px, cy_px = fw // 2, fh // 2
                cv2.drawMarker(frame, (cx_px, cy_px), (0, 255, 255), markerType=cv2.MARKER_CROSS,
                                markerSize=24, thickness=2)

                info_lines = ["WASD=aim  f=IR on/off  0-9=target idx  click=mark IR  ENTER=save  c=clear  ESC/Q=quit",
                              f"target_index={target_index}  IR={'ON (ยิงซ้ำ)' if ir_beam_on else 'off'}"]

                if _click_px is not None:
                    cv2.drawMarker(frame, _click_px, (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS,
                                    markerSize=20, thickness=2)
                    cv2.line(frame, (cx_px, cy_px), _click_px, (0, 0, 255), 1)
                    cm_per_px = _px_to_cm_scale(fw)
                    dx_cm = (_click_px[0] - cx_px) * cm_per_px
                    dy_cm = (_click_px[1] - cy_px) * cm_per_px
                    info_lines.append(f"offset_x={offset_x_n:+.4f}  (~{-dx_cm:+.1f}cm)")
                    info_lines.append(f"offset_y_raw={offset_y_n:+.4f} (~{-dy_cm:+.1f}cm) -- NOT for real ammo!")
                else:
                    info_lines.append("ยังไม่คลิกจุด IR (กด f เปิดค้างก่อน)")

                y0 = 30
                for line in info_lines:
                    cv2.putText(frame, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    y0 += 26

                cv2.imshow(window_name, frame)

            if key == 27 or key == ord('q'):
                print("\nจบการ calibrate")
                break
    except KeyboardInterrupt:
        print("\nหยุด laser_calibrate")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

    if samples:
        print("\n" + "=" * 70)
        print("สรุปผล -- ก็อปวางแทน AIM_OFFSET_X_BY_INDEX ใน detectcolor.py ได้เลย (เฉลี่ยถ้ามีหลายตัวอย่างต่อ index เดียวกัน)")
        by_index = {}
        for s in samples:
            by_index.setdefault(s["target_index"], []).append(s["offset_x"])
        print("AIM_OFFSET_X_BY_INDEX = {")
        for idx in sorted(by_index):
            vals = by_index[idx]
            avg = sum(vals) / len(vals)
            n_note = f" (เฉลี่ยจาก {len(vals)} ตัวอย่าง)" if len(vals) > 1 else ""
            print(f"    {idx}: {avg:+.4f},{n_note}")
        print("}")
        print(f"log ทั้งหมด -> {os.path.basename(CALIB_LOG_PATH)}")
        print("*** อย่าลืม: offset_y_raw ในไฟล์ log ใช้แทน PROJECTILE_DROP_COMPENSATION ไม่ได้ ***")
        print("=" * 70)
    else:
        print("\nไม่มีตัวอย่างที่บันทึกไว้เลย")


if __name__ == "__main__":
    main()
