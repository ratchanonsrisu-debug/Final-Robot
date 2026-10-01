"""ถอดรหัสภาพกล้อง EP ใน process แยก (config.CAM_ISOLATE)

ทำไม: ตัวถอดรหัส H264 ของ SDK (libmedia_codec -> ffmpeg avcodec/avutil) พังแบบ native (access violation ใน
msvcrt.dll offset 0x7b2dc ; Windows Event Log 29/9-1/10 เกิด 9 ครั้ง) -> python ตายทั้งโปรแกรมทันที จับ exception ไม่ได้
-> คำสั่ง drive_speed ล่าสุดค้างที่หุ่น (timeout ของ SDK เป็น timer ฝั่งคอม ตายไปด้วย) หุ่นพุ่งชนกำแพง (1/10 04:39)
แยกมาถอดรหัสที่นี่: พังก็ตายแค่ process นี้ โปรแกรมหลัก (เดิน/หยุดล้อ) ทำงานต่อ แล้วเปิดตัวนี้ใหม่

รับ: python camproc.py <robot_ip> <video_port> <shared_memory_name> <parent_pid>
shared memory: [0:32] float64 x4 = seq (คี่ = กำลังเขียน), w, h, time ; [32:] ภาพ BGR (เหมือน read_cv2_image)
"""
import ctypes
import faulthandler
import socket
import sys
import time

import numpy as np
from multiprocessing import shared_memory

HDR = 32


def _parent_alive(pid):
    k = ctypes.windll.kernel32
    h = k.OpenProcess(0x00100000, False, pid)            # SYNCHRONIZE
    if not h:
        return False
    try:
        return k.WaitForSingleObject(h, 0) == 0x102      # WAIT_TIMEOUT = ยังไม่จบ
    finally:
        k.CloseHandle(h)


def main(ip, port, name, ppid):
    faulthandler.enable()                                # พังแบบ native -> เขียน stack ลง stderr (ไฟล์ log)
    import libmedia_codec
    shm = shared_memory.SharedMemory(name=name)
    hdr = np.ndarray((4,), dtype=np.float64, buffer=shm.buf[:HDR])
    img = shm.buf[HDR:]
    dec = libmedia_codec.H264Decoder()
    sock = socket.create_connection((ip, port), timeout=3)
    sock.settimeout(1.0)
    seq = int(hdr[0]) // 2 + 1
    t_chk = time.time()
    while True:
        if time.time() - t_chk > 1.0:
            t_chk = time.time()
            if not _parent_alive(ppid):
                return 0                                 # โปรแกรมหลักปิด/ตาย -> เลิก (ไม่ค้างถือ socket วิดีโอ)
        try:
            data = sock.recv(65536)
        except socket.timeout:
            continue
        if not data:
            return 2                                     # หุ่นปิดการเชื่อมต่อวิดีโอ
        for f, w, h, _ls in dec.decode(data):
            n = w * h * 3
            if not f or n > len(img) or len(f) < n:
                continue
            hdr[0] = 2 * seq - 1
            img[:n] = f[:n]
            hdr[1], hdr[2], hdr[3] = w, h, time.time()
            hdr[0] = 2 * seq
            seq += 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])))
