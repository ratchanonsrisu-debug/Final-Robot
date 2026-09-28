"""
sensor_calib.py — บันทึกค่า Sharp ซ้าย/ขวา + ToF ที่ระยะจริง 5..30 cm แล้วฟิตสูตรแปลงค่า

วิธีใช้:
    python sensor_calib.py                 # ต่อหุ่นจริง (AP mode)
    python sensor_calib.py --fit FILE.csv  # ฟิตใหม่จาก CSV เดิม ไม่ต้องต่อหุ่น

วัดทีละตัวตามลำดับ: ToF -> Sharp ซ้าย -> Sharp ขวา  (แต่ละตัวไล่ระยะ 5..30 cm)

ระหว่างบันทึก (พิมพ์แล้วกด Enter):
    <Enter>     บันทึก 1 แถว (เฉพาะเซนเซอร์ที่กำลังวัด) ที่ระยะปัจจุบัน (กดรัวๆ ได้เลย)
    n / b       ไประยะถัดไป / ย้อนระยะก่อนหน้า  (n ที่ระยะสุดท้าย = ไปเซนเซอร์ตัวถัดไป)
    <ตัวเลข>    ตั้งระยะเอง (cm) เช่น 12.5
    s           ข้ามไปเซนเซอร์ตัวถัดไปเลย
    u           ลบแถวล่าสุด
    q           จบ -> บันทึก CSV + ฟิตสูตร (output/sensor_calib_*.csv / *_fit.txt)

ระยะ = ระยะจาก "หน้าเซนเซอร์" ถึงแผ่นวัด (cm) - วัดแบบเดียวกันทุกครั้ง
"""

import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time

# ---------------- ตั้งค่า ----------------
DISTANCES_CM = [5, 10, 15, 20, 25, 30]
SHARP_LEFT = (3, 1)     # (hub_id, port) - ตรงกับ Classwork8/SLAM.py
SHARP_RIGHT = (2, 2)
TOF_INDEX = 0
ADC_SAMPLES = 3         # อ่าน ADC กี่ครั้งต่อ 1 Enter แล้วเอา median
TOF_SAMPLES = 3
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

# ลำดับการวัด (tag, คอลัมน์ CSV, ชื่อที่แสดง)
SENSORS = [("T", "tof_mm", "ToF (หน้า gimbal)"),
           ("L", "sharp_left_adc", "Sharp ซ้าย"),
           ("R", "sharp_right_adc", "Sharp ขวา")]


# ======================================================================
# อ่านเซนเซอร์
# ======================================================================
class Sensors:
    def __init__(self):
        from robomaster import robot       # noqa: PLC0415
        self.ep = robot.Robot()
        print("กำลังเชื่อมต่อ RoboMaster (AP mode) ...")
        self.ep.initialize(conn_type="ap")
        self.ep.gimbal.recenter().wait_for_completed()   # ToF ชี้ตรงหน้า
        self._lock = threading.Lock()
        self._tof = []
        self.ep.sensor.sub_distance(freq=20, callback=self._on_tof)
        time.sleep(0.5)
        print("เชื่อมต่อสำเร็จ")

    def _on_tof(self, info):
        with self._lock:
            self._tof.append(info[TOF_INDEX])
            self._tof = self._tof[-10:]

    def _adc(self, hub_port):
        vals = []
        for _ in range(ADC_SAMPLES):
            try:
                vals.append(self.ep.sensor_adaptor.get_adc(id=hub_port[0], port=hub_port[1]))
            except Exception as e:                      # noqa: BLE001
                print(f"  [adc] hub {hub_port} อ่านไม่ได้: {e}")
        vals = [v for v in vals if v is not None]
        return statistics.median(vals) if vals else None

    def read(self, tag):
        """อ่านเฉพาะเซนเซอร์ tag ('T' / 'L' / 'R')"""
        if tag == "L":
            return self._adc(SHARP_LEFT)
        if tag == "R":
            return self._adc(SHARP_RIGHT)
        with self._lock:
            tof = self._tof[-TOF_SAMPLES:]
        return statistics.median(tof) if tof else None

    def close(self):
        try:
            self.ep.sensor.unsub_distance()
        except Exception:                               # noqa: BLE001
            pass
        self.ep.close()


# ======================================================================
# ฟิตสูตร
# ======================================================================
def _linfit(xs, ys):
    """y = k*x + c (least squares)"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return k, my - k * mx


def _interp(table, v):
    """แปลงค่าเซนเซอร์ v -> cm ด้วย lookup table [(value, cm), ...] แบบ piecewise linear"""
    pts = sorted(table)
    if v <= pts[0][0]:
        (x0, y0), (x1, y1) = pts[0], pts[1]
    elif v >= pts[-1][0]:
        (x0, y0), (x1, y1) = pts[-2], pts[-1]
    else:
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            if x0 <= v <= x1:
                break
    return y0 if x1 == x0 else y0 + (v - x0) * (y1 - y0) / (x1 - x0)


def fit(rows):
    out = []
    for tag, col, _ in SENSORS:
        use = [(float(r["true_cm"]), float(r[col])) for r in rows
               if r.get(col) not in (None, "", "None") and r["target"] in ("ALL", tag)]
        out.append(f"===== {col} ({len(use)} แถว) =====")
        by_d = {}
        for d, v in use:
            by_d.setdefault(d, []).append(v)
        if len(by_d) < 2:
            out.append("  ข้อมูลไม่พอ (ต้องมีอย่างน้อย 2 ระยะ)\n")
            continue

        out.append("  cm      median     min      max    std   n")
        table = []
        for d in sorted(by_d):
            vs = by_d[d]
            med = statistics.median(vs)
            sd = statistics.pstdev(vs) if len(vs) > 1 else 0.0
            table.append((med, d))
            out.append(f"  {d:5.1f}  {med:8.1f}  {min(vs):7.1f}  {max(vs):7.1f}  {sd:5.1f}  {len(vs)}")

        meds = [m for m, _ in sorted(table, key=lambda t: t[1])]
        diffs = [b - a for a, b in zip(meds, meds[1:])]
        if not (all(x > 0 for x in diffs) or all(x < 0 for x in diffs)):
            out.append("  !! ค่าไม่ไล่ทางเดียวกัน (non-monotonic) - ระยะที่ค่ากลับทางใช้แยกระยะไม่ได้"
                       " (Sharp มักอ่านผิดที่ระยะใกล้กว่าช่วงใช้งาน) ควรตัดระยะนั้นทิ้ง")

        if col == "tof_mm":
            lf = _linfit([v for _, v in use], [d * 10 for d, _ in use])
            if lf:
                k, c = lf
                out.append(f"  สูตร: true_mm = {k:.4f} * raw + {c:.1f}")
                err = [abs(k * v + c - d * 10) for d, v in use]
                out.append(f"  error เฉลี่ย {statistics.mean(err):.1f} mm, สูงสุด {max(err):.1f} mm")
        else:
            pw = [(math.log(v), math.log(d)) for d, v in use if v > 0]
            lf = _linfit([p[0] for p in pw], [p[1] for p in pw]) if len(pw) >= 2 else None
            if lf:
                b, ln_a = lf
                a = math.exp(ln_a)
                out.append(f"  สูตร power law: cm = {a:.4g} * adc ^ {b:.4f}")
                err = [abs(a * v ** b - d) for d, v in use if v > 0]
                out.append(f"    error เฉลี่ย {statistics.mean(err):.2f} cm, สูงสุด {max(err):.2f} cm")
            err = [abs(_interp(table, v) - d) for d, v in use]
            out.append(f"  lookup table (แม่นกว่าในช่วงที่วัด): error เฉลี่ย {statistics.mean(err):.2f} cm")
        pairs = ", ".join(f"({m:.0f}, {d:g})" for m, d in sorted(table))
        out.append(f"  TABLE_{tag} = [{pairs}]   # (ค่าดิบ, cm)\n")
    return "\n".join(out)


# ======================================================================
# โหมดบันทึก
# ======================================================================
FIELDS = ["time_s", "true_cm", "target", "sharp_left_adc", "sharp_right_adc", "tof_mm"]


def record():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUTPUT_DIR, f"sensor_calib_{stamp}.csv")
    s = Sensors()
    rows, t0 = [], time.time()
    si, i = 0, 0                    # index เซนเซอร์ / index ระยะ
    dist = DISTANCES_CM[0]

    def save_all():
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)

    def announce():
        tag, _, name = SENSORS[si]
        print(f"\n===== [{si + 1}/{len(SENSORS)}] {name} =====")
        if tag == "T":
            print("วางแผ่นวัดตรงหน้า ToF (gimbal หันหน้าตรง)")
        else:
            print(f"วางแผ่นวัดด้าน{name.split()[1]}ของหุ่น ตั้งฉากกับ Sharp")

    print(__doc__.split("วัดทีละตัว")[1].split("ระยะ =")[0])
    announce()
    try:
        while si < len(SENSORS):
            tag, col, name = SENSORS[si]
            n_here = sum(1 for r in rows if r["true_cm"] == dist and r["target"] == tag)
            cmd = input(f"[{name} | {dist:g} cm | {n_here} แถว] > ").strip().lower()
            next_sensor = False
            if cmd == "":
                v = s.read(tag)
                row = {k: "" for k in FIELDS}
                row.update(time_s=round(time.time() - t0, 2), true_cm=dist, target=tag)
                row[col] = v
                rows.append(row)
                save_all()
                print(f"   {name} = {v}")
            elif cmd == "n":
                if i == len(DISTANCES_CM) - 1:
                    next_sensor = True
                else:
                    i += 1
                    dist = DISTANCES_CM[i]
            elif cmd == "b":
                i = max(i - 1, 0)
                dist = DISTANCES_CM[i]
            elif cmd == "s":
                next_sensor = True
            elif cmd == "u":
                if rows:
                    print(f"   ลบ: {rows.pop()}")
                    save_all()
            elif cmd == "q":
                break
            else:
                try:
                    dist = float(cmd)
                except ValueError:
                    print("   ไม่รู้จักคำสั่ง")
            if next_sensor:
                si += 1
                i, dist = 0, DISTANCES_CM[0]
                if si < len(SENSORS):
                    announce()
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        save_all()
        s.close()
    print(f"บันทึก {len(rows)} แถว -> {csv_path}")
    if rows:
        write_fit(csv_path, rows)


def write_fit(csv_path, rows):
    txt = fit(rows)
    print("\n" + txt)
    fit_path = csv_path[:-4] + "_fit.txt"
    with open(fit_path, "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    print(f"ผลฟิต -> {fit_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--fit", help="ฟิตใหม่จาก CSV ที่บันทึกไว้แล้ว")
    a = ap.parse_args()
    if a.fit:
        with open(a.fit, encoding="utf-8") as fh:
            rs = list(csv.DictReader(fh))
        for r in rs:
            r["true_cm"] = float(r["true_cm"])
        write_fit(a.fit, rs)
    else:
        record()
    sys.exit(0)
