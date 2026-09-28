# -*- coding: utf-8 -*-
"""
fire_calibrate.py -- Calibrate AIM_OFFSET_X_BY_INDEX + PROJECTILE_DROP_COMPENSATION
ด้วยการยิงกระสุนเจลจริง 1 นัดแล้วคลิกจุดที่โดนจริงบนภาพ (ไม่มีเลเซอร์แยกต่างหาก เลยใช้รอยกระสุนจริงแทน)

*** ยิงกระสุนเจลจริงทุกครั้งที่กด f -- ต้องมีที่กันกระสุน/แผ่นเป้าอยู่ตรงหน้าเสมอ ***

วิธีใช้:
  1. วางป้าย/เป้าไว้ที่ระยะ DISTANCE_CM (ต้องตรงกับค่าที่ตั้งไว้ใน detectcolor.py ด้วย)
  2. WASD เล็ง gimbal จน "จุดกึ่งกลางป้าย" อยู่ตรงกากบาทเหลืองกลางจอพอดี (สำคัญ: เล็งด้วยตาให้ตรงกลางจริงๆ
     ไม่ต้องพึ่งระบบตรวจจับสีเลย เพราะจุดนี้คือ "จุดอ้างอิง offset=0" ที่ค่า offset ในไฟล์นี้อิงตาม)
  3. กด f เพื่อยิง 1 นัด (มีคำถามยืนยัน YES ครั้งเดียวตอนเปิดโปรแกรม)
  4. มองรอยกระสุน/คราบเจลที่ป้าย (กล้องนิ่งอยู่ที่เดิม รอยควรอยู่ในเฟรมเดิมเลย) แล้วคลิกซ้ายตรงรอยนั้นในภาพ
     โปรแกรมคำนวณ offset_x และ drop_compensation ให้ทันที
  5. กด 0-9 เลือกว่ากำลัง calibrate "ป้ายลำดับที่เท่าไหร่" (ตรงกับ key ของ AIM_OFFSET_X_BY_INDEX ใน
     detectcolor.py, 0=ซ้ายสุด) แล้วกด Enter/Space เพื่อบันทึกค่า -- ทำซ้ำได้หลายตำแหน่ง/มุม
  6. กด ESC หรือ q เพื่อจบ โปรแกรมจะสรุปเป็นค่าพร้อมก็อปวางใน detectcolor.py

สูตรที่ใช้ (ตรงกับที่ detectcolor.py ใช้เล็ง/ยิงจริง อิงตามคอมเมนต์ calibrate เดิมในไฟล์นั้น):
  AIM_OFFSET_X_BY_INDEX[idx] = 0.5 - click_x_normalized
  PROJECTILE_DROP_COMPENSATION = click_y_normalized - 0.5   (ค่าเดียวรวม ไม่แยกตาม index)

python fire_calibrate.py
"""

import os
import sys
import csv
import time
import math
from datetime import datetime

import cv2
from robomaster import robot, blaster

SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIB_LOG_PATH = os.path.join(SAVE_DIR, "fire_calibration_log.csv")

# ต้องตรงกับค่าที่ใช้ใน detectcolor.py ถึงจะแปลงเป็น ซม. บนจอได้ถูกต้อง (ใช้แค่โชว์ผล ไม่กระทบสูตร offset)
CAMERA_HFOV_DEG = 95.0
DISTANCE_CM = 100.0

MANUAL_TURN_SPEED = 60
MANUAL_KEY_HOLD_TIMEOUT = 0.15

_click_px = None  # (x, y) พิกเซลล่าสุดที่คลิก (รอยกระสุน) หรือ None ถ้ายังไม่คลิก


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

    print("=" * 70)
    print("!!! คำเตือน: fire_calibrate.py ยิงกระสุนเจลจริงทุกครั้งที่กด f !!!")
    print("เช็คก่อนเริ่ม: มีที่กันกระสุน/แผ่นกันหลังเป้าแล้ว, ไม่มีคน/สัตว์เลี้ยงอยู่ในแนวยิงหรือแนวสะท้อน")
    print("=" * 70)
    confirm = input("พิมพ์ YES (ตัวพิมพ์ใหญ่ทั้งหมด) เพื่อยืนยันว่าพร้อมยิงจริง: ").strip()
    if confirm != "YES":
        print("ยกเลิก ไม่ได้ยืนยัน ไม่มีการยิงเกิดขึ้น")
        return

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_gimbal = ep_robot.gimbal
    ep_camera = ep_robot.camera
    ep_blaster = ep_robot.blaster

    print("=" * 70)
    print("WASD เล็งให้จุดกึ่งกลางป้ายอยู่ตรงกากบาทเหลืองกลางจอ (เล็งด้วยตา ไม่ต้องพึ่งระบบตรวจจับสี)")
    print("f = ยิง 1 นัด (กระสุนจริง) -- แล้วคลิกซ้ายตรงรอยที่โดนบนภาพ")
    print("0-9 เลือกลำดับป้ายที่กำลัง calibrate, Enter/Space = บันทึกค่า, c = ล้างจุดคลิก, ESC/q = จบ+สรุปผล")
    print("=" * 70)

    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    window_name = "RoboMaster Camera - fire_calibrate"
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
    shots_fired = 0
    samples = []  # list ของ dict: {target_index, offset_x, drop_comp, timestamp}

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
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)  # หยุดนิ่งสนิทก่อนยิง กันโมเมนตัมตกค้าง
                time.sleep(0.1)
                print("\n>> ยิง 1 นัด! มองรอยที่โดนบนป้ายแล้วคลิกซ้ายตรงจุดนั้นในภาพ")
                ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=1)
                shots_fired += 1
                _click_px = None  # บังคับให้คลิกใหม่ทุกครั้งหลังยิง กันเผลอใช้จุดคลิกค้างจากนัดก่อน

            offset_x_n = drop_comp_n = None
            if frame is not None and _click_px is not None:
                fh, fw = frame.shape[:2]
                cx, cy = _click_px
                offset_x_n = 0.5 - (cx / fw)
                drop_comp_n = (cy / fh) - 0.5

            if key in (13, 32) and offset_x_n is not None:  # Enter/Space -> บันทึกค่า
                sample = {
                    "target_index": target_index,
                    "offset_x": offset_x_n,
                    "drop_comp": drop_comp_n,
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                }
                samples.append(sample)
                is_new = not os.path.exists(CALIB_LOG_PATH)
                with open(CALIB_LOG_PATH, "a", newline="", encoding="utf-8-sig") as f:
                    writer = csv.writer(f)
                    if is_new:
                        writer.writerow(["timestamp", "target_index", "offset_x", "drop_comp"])
                    writer.writerow([sample["timestamp"], target_index, f"{offset_x_n:.4f}", f"{drop_comp_n:.4f}"])
                print(f"\n>> บันทึกแล้ว target_index={target_index}  offset_x={offset_x_n:+.4f}  "
                      f"drop_comp={drop_comp_n:+.4f}")

            if frame is not None:
                fh, fw = frame.shape[:2]
                cx_px, cy_px = fw // 2, fh // 2
                cv2.drawMarker(frame, (cx_px, cy_px), (0, 255, 255), markerType=cv2.MARKER_CROSS,
                                markerSize=24, thickness=2)

                info_lines = ["WASD=aim  f=FIRE  0-9=target idx  click=mark hit  ENTER=save  c=clear  ESC/Q=quit",
                              f"target_index={target_index}  shots_fired={shots_fired}"]

                if _click_px is not None:
                    cv2.drawMarker(frame, _click_px, (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS,
                                    markerSize=20, thickness=2)
                    cv2.line(frame, (cx_px, cy_px), _click_px, (0, 0, 255), 1)
                    cm_per_px = _px_to_cm_scale(fw)
                    dx_cm = (_click_px[0] - cx_px) * cm_per_px
                    dy_cm = (_click_px[1] - cy_px) * cm_per_px
                    info_lines.append(f"offset_x={offset_x_n:+.4f}  (miss {dx_cm:+.1f}cm L/R)")
                    info_lines.append(f"drop_comp={drop_comp_n:+.4f}  (miss {dy_cm:+.1f}cm U/D)")
                else:
                    info_lines.append("ยังไม่คลิกรอยกระสุน (กด f ยิงก่อน แล้วคลิกรอยที่เห็น)")

                y0 = 30
                for line in info_lines:
                    cv2.putText(frame, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    y0 += 26

                cv2.imshow(window_name, frame)

            if key == 27 or key == ord('q'):
                print("\nจบการ calibrate")
                break
    except KeyboardInterrupt:
        print("\nหยุด fire_calibrate")
    finally:
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

    if samples:
        print("\n" + "=" * 70)
        print("สรุปผล -- ก็อปวางใน detectcolor.py ได้เลย")

        by_index = {}
        all_drop_comp = []
        for s in samples:
            by_index.setdefault(s["target_index"], []).append(s["offset_x"])
            all_drop_comp.append(s["drop_comp"])

        print("\nAIM_OFFSET_X_BY_INDEX = {")
        for idx in sorted(by_index):
            vals = by_index[idx]
            avg = sum(vals) / len(vals)
            n_note = f" (เฉลี่ยจาก {len(vals)} ตัวอย่าง)" if len(vals) > 1 else ""
            print(f"    {idx}: {avg:+.4f},{n_note}")
        print("}")

        avg_drop = sum(all_drop_comp) / len(all_drop_comp)
        print(f"\nPROJECTILE_DROP_COMPENSATION = {avg_drop:+.4f}  "
              f"(เฉลี่ยจากทั้งหมด {len(all_drop_comp)} นัด, ค่าเดียวรวม ไม่แยกตาม index)")

        print(f"\nยิงทั้งหมด {shots_fired} นัด, log -> {os.path.basename(CALIB_LOG_PATH)}")
        print("=" * 70)
    else:
        print("\nไม่มีตัวอย่างที่บันทึกไว้เลย")


if __name__ == "__main__":
    main()
