# -*- coding: utf-8 -*-
"""
testsen.py -- เช็คเซนเซอร์ที่ต่อกับ RoboMaster EP แบบเรียลไทม์

ฮาร์ดแวร์:
  - Sharp (analog IR distance) x2  -> sensor_adaptor.get_adc(id, port)
  - IR digital obstacle sensor x2  -> sensor_adaptor.get_io(id, port)
    (รวม 4 พอร์ตบน sensor adaptor บอร์ด)
  - ToF distance module            -> ติดอยู่บน gimbal, อ่านผ่าน ep_robot.sensor.sub_distance()

แก้ id/port/type ใน SENSOR_PORTS ด้านล่างให้ตรงกับการต่อสายจริงของหุ่นตัวนี้
(ถ้าไม่แน่ใจว่า id/port ไหนตอบสนองจริง ให้ไล่ค่าดูจาก terminal ตอนบัง/ปล่อยเซนเซอร์แต่ละตัว)

การใช้งาน:
    python testsen.py                  # conn_type=ap (ต่อ WiFi ตรงหุ่น), รีเฟรชทุก 0.2s
    python testsen.py --conn sta --interval 0.1
"""

import sys
import time
import argparse

from robomaster import robot


# ----------------------------------------------------------------------------
# ตั้งค่าพอร์ต sensor_adaptor ที่ใช้งานจริง (แก้ตรงนี้ให้ตรงกับสายที่ต่อ)
#   type: "sharp" -> analog, อ่านด้วย get_adc (ค่า 0-1023)
#         "ir"    -> digital, อ่านด้วย get_io  (ค่า 0/1)
# ----------------------------------------------------------------------------
SENSOR_PORTS = [
    {"id": 1, "port": 1, "type": "sharp", "label": "Sharp-FrontLeft"},
    {"id": 1, "port": 2, "type": "ir",    "label": "IR45-FrontLeft"},
    {"id": 2, "port": 1, "type": "ir",    "label": "IR-FrontRight"},
    {"id": 3, "port": 1, "type": "sharp", "label": "Sharp-RearLeft"},
]


def build_parser():
    p = argparse.ArgumentParser(description="Realtime sensor check: Sharp/IR (sensor_adaptor) + ToF (gimbal)")
    p.add_argument("--conn", default="ap", choices=["ap", "sta", "rndis"], help="ประเภทการเชื่อมต่อ")
    p.add_argument("--interval", type=float, default=0.2, help="วินาทีต่อการรีเฟรชหนึ่งบรรทัด")
    p.add_argument("--scan", action="store_true",
                   help="ไล่หา id (1-8) / port (1-2) ที่ตอบสนองจริงก่อน แล้วจบโปรแกรม "
                        "(ใช้ตอนไม่รู้ว่าบอร์ด sensor adaptor ตั้ง DIP switch id ไว้เท่าไหร่)")
    return p


def scan_ports(adaptor):
    """ไล่ id=1..8, port=1..2 หาว่าตัวไหน get_io ตอบสนองจริง (ไม่ None/ไม่ error)"""
    print("[Scan] ไล่หา id/port ที่ตอบสนอง (id=1..8, port=1..2)...\n")
    found = []
    for bid in range(1, 9):
        for port in (1, 2):
            try:
                io_val = adaptor.get_io(id=bid, port=port)
            except Exception as e:
                io_val = None
            try:
                adc_val = adaptor.get_adc(id=bid, port=port)
            except Exception:
                adc_val = None
            tag = "  <-- ตอบสนอง!" if io_val is not None else ""
            print("  id=%d port=%d  IO=%s  ADC=%s%s" % (bid, port, io_val, adc_val, tag))
            if io_val is not None:
                found.append((bid, port))
    print("\n[Scan] สรุป id/port ที่ตอบสนอง:", found if found else "ไม่พบเลย")
    if found:
        print("[Scan] แก้ SENSOR_PORTS ด้านบนไฟล์ให้ใช้ id/port เหล่านี้ตามที่รู้ว่าเซนเซอร์ไหนต่อพอร์ตไหน")
    else:
        print("[Scan] ไม่พบบอร์ดตอบสนองเลย -- เช็คว่า sensor adaptor เสียบแน่น/มีไฟเลี้ยง และแอป RoboMaster เห็นบอร์ดมั้ย")


def read_port(adaptor, port_cfg):
    """อ่านค่า sensor_adaptor -- แสดงทั้ง IO และ ADC เสมอ (ไม่ใช่แค่ตาม type ที่ตั้งไว้)
    เพื่อวินิจฉัยได้ว่าเซนเซอร์ตัวนี้จริงๆ ขยับค่าที่ช่องไหน (บางตัวมีขา AO ให้ค่าต่อเนื่อง
    แม้จะติดตั้งไว้เป็น IO ก็ตาม, หรือ IO ค้างเพราะ sensitivity trimmer/ระยะตรวจจับ)"""
    bid = port_cfg["id"]
    port = port_cfg["port"]
    try:
        io_val = adaptor.get_io(id=bid, port=port)
    except Exception as e:
        io_val = "ERR"
    try:
        adc_val = adaptor.get_adc(id=bid, port=port)
    except Exception:
        adc_val = "ERR"
    return "IO=%3s ADC=%4s" % (
        io_val if io_val is not None else "NA",
        adc_val if adc_val is not None else "NA")


def main():
    args = build_parser().parse_args()

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type=args.conn)

    adaptor = ep_robot.sensor_adaptor
    gimbal = ep_robot.gimbal
    sensor = ep_robot.sensor

    if args.scan:
        scan_ports(adaptor)
        ep_robot.close()
        return

    tof_mm = [None]

    def on_distance(sub_info):
        tof_mm[0] = sub_info[0]

    print("[Gimbal] ล็อก pitch=0 yaw=0 (ToF หันตรงไปข้างหน้า)...")
    gimbal.recenter().wait_for_completed()

    print("[ToF] เปิด subscribe...")
    sensor.sub_distance(freq=10, callback=on_distance)

    print("[Sensor] เริ่มอ่านค่าเรียลไทม์ (Ctrl+C เพื่อหยุด)\n")
    time.sleep(0.3)

    try:
        while True:
            parts = []
            for cfg in SENSOR_PORTS:
                reading = read_port(adaptor, cfg)
                parts.append("%s[id%s.p%s %s]=%s" % (
                    cfg["label"], cfg["id"], cfg["port"], cfg["type"].upper(), reading))

            tof_str = "%.1fcm" % (tof_mm[0] / 10.0) if tof_mm[0] is not None else "NA"
            parts.append("ToF(gimbal)=%s" % tof_str)

            line = " | ".join(parts)
            print("\r" + line + "     ", end="", flush=True)
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[Sensor] หยุดโดยผู้ใช้")

    finally:
        try:
            sensor.unsub_distance()
        except Exception:
            pass
        ep_robot.close()
        print("[Sensor] ปิดการเชื่อมต่อแล้ว")


if __name__ == "__main__":
    main()
