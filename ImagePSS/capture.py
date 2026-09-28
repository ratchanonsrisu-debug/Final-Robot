# -*- coding: utf-8 -*-
"""Capture still images (1280x720) from the RoboMaster EP camera on Enter, with a live preview."""

import os
import threading
import time
from datetime import datetime

import cv2
from robomaster import robot, camera

CONN_TYPE = "ap"          # เปลี่ยนเป็น "sta" ถ้าต่อผ่าน router
WARMUP_SEC = 3.0          # เวลารอให้ auto light balance (auto exposure/white balance) นิ่งก่อนเริ่มถ่าย
SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
TARGET_SIZE = (1280, 720)  # (width, height)
WINDOW_NAME = "RoboMaster Camera"


def listen_for_enter(capture_event, stop_event):
    while not stop_event.is_set():
        try:
            input()
        except EOFError:
            return
        capture_event.set()


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type=CONN_TYPE)
    ep_camera = ep_robot.camera

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

    print(f"กำลังรอ Auto Light Balance นิ่งก่อน ({WARMUP_SEC:.0f} วินาที) ...")
    time.sleep(WARMUP_SEC)

    print("พร้อมบันทึกภาพแล้ว (พ้นช่วง Auto Light Balance แล้ว)")
    print("กด Enter เพื่อถ่ายภาพ (ย้ายหุ่นแล้วกดต่อได้เลย) / กด Ctrl+C เพื่อออก")

    capture_event = threading.Event()
    stop_event = threading.Event()
    input_thread = threading.Thread(
        target=listen_for_enter, args=(capture_event, stop_event), daemon=True
    )
    input_thread.start()

    count = 0
    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
            if frame is None:
                cv2.waitKey(1)
                continue

            if (frame.shape[1], frame.shape[0]) != TARGET_SIZE:
                frame = cv2.resize(frame, TARGET_SIZE)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if capture_event.is_set():
                capture_event.clear()
                count += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"capture_{count:03d}_{timestamp}.jpg"
                filepath = os.path.join(SAVE_DIR, filename)
                cv2.imwrite(filepath, frame)
                print(f"บันทึกภาพที่ {count} -> {filename}")

            if key == 27:  # ESC บนหน้าต่างภาพ ออกได้เช่นกัน
                break
    except KeyboardInterrupt:
        print(f"\nยกเลิกโปรแกรม (Ctrl+C) บันทึกไปทั้งหมด {count} ภาพ")
    finally:
        stop_event.set()
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()


if __name__ == "__main__":
    main()
