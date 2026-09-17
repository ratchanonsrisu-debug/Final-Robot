# -*-coding:utf-8-*-
# อ่านไฟล์ gimbal_pid_log_*.csv (สร้างจาก gimbal_pid_lab.py หลังจบการทดลอง)
# แล้ว plot time response ของมุม yaw/pitch สำหรับแนบในรายงาน
#
# ใช้งาน: python plot_response.py [ชื่อไฟล์.csv]
# ถ้าไม่ระบุไฟล์ จะใช้ไฟล์ gimbal_pid_log_*.csv ล่าสุดในโฟลเดอร์นี้

import csv
import glob
import os
import sys

import matplotlib.pyplot as plt

# ฟอนต์เริ่มต้นของ matplotlib ไม่มีตัวอักษรไทย (ขึ้นเป็นกล่อง □) ต้องสั่งใช้ฟอนต์ที่มีไทยก่อน
plt.rcParams["font.family"] = ["Tahoma", "Leelawadee UI", "Angsana New", "Arial"]
plt.rcParams["axes.unicode_minus"] = False  # บางฟอนต์ไทยไม่มีสัญลักษณ์ลบแบบ unicode

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ต้องตรงกับ AIM_TOLERANCE ที่ใช้จริงตอนรัน gimbal_pid_lab.py (ดูค่าปัจจุบันในไฟล์นั้น)
AIM_TOLERANCE = 0.005


def find_latest_log():
    files = glob.glob(os.path.join(SCRIPT_DIR, "gimbal_pid_log_*.csv"))
    if not files:
        raise FileNotFoundError("ไม่พบไฟล์ gimbal_pid_log_*.csv ในโฟลเดอร์นี้ ลองรัน gimbal_pid_lab.py ก่อน")
    return max(files, key=os.path.getmtime)


if __name__ == '__main__':
    log_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_log()
    print("อ่าน log: {0}".format(log_path))

    t, yaw, pitch, phase = [], [], [], []
    fire_events = []  # (t, yaw, pitch) ตอนยิงแต่ละนัด
    err_t, yaw_err, pitch_err = [], [], []  # เฉพาะช่วง AIM ที่มีข้อมูล error จริง

    with open(log_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t_s"]))
            yaw.append(float(row["yaw_angle_deg"]))
            pitch.append(float(row["pitch_angle_deg"]))
            phase.append(row["phase"])
            if row["phase"] == "FIRE":
                fire_events.append((float(row["t_s"]), float(row["yaw_angle_deg"]),
                                     float(row["pitch_angle_deg"])))
            if row["phase"] == "AIM" and row["yaw_error"] != "":
                err_t.append(float(row["t_s"]))
                yaw_err.append(float(row["yaw_error"]) * 100)      # แปลงเป็น % ของเฟรม
                pitch_err.append(float(row["pitch_error"]) * 100)

    fig, (ax_yaw, ax_pitch, ax_err) = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    ax_yaw.plot(t, yaw, "-b", linewidth=1.2, label="yaw angle")
    for i, (ft, fy, _fp) in enumerate(fire_events):
        ax_yaw.axvline(ft, color="r", linestyle="--", linewidth=1)
        ax_yaw.plot(ft, fy, "rv", markersize=8, label="ยิง" if i == 0 else None)
    ax_yaw.set_ylabel("yaw (deg)")
    ax_yaw.set_title("Time response ของมุม Gimbal (yaw)")
    ax_yaw.grid(True)
    ax_yaw.legend()

    ax_pitch.plot(t, pitch, "-g", linewidth=1.2, label="pitch angle")
    for i, (ft, _fy, fp) in enumerate(fire_events):
        ax_pitch.axvline(ft, color="r", linestyle="--", linewidth=1)
        ax_pitch.plot(ft, fp, "rv", markersize=8, label="ยิง" if i == 0 else None)
    ax_pitch.set_xlabel("time (s)")
    ax_pitch.set_ylabel("pitch (deg)")
    ax_pitch.set_title("Time response ของมุม Gimbal (pitch)")
    ax_pitch.grid(True)
    ax_pitch.legend()

    tol_pct = AIM_TOLERANCE * 100
    ax_err.axhspan(-tol_pct, tol_pct, color="green", alpha=0.15,
                    label="เกณฑ์ยอมรับ ({0:.2f}%)".format(tol_pct))
    ax_err.axhline(0, color="k", linewidth=0.8)
    ax_err.plot(err_t, yaw_err, ".-b", markersize=3, linewidth=1, label="yaw error")
    ax_err.plot(err_t, pitch_err, ".-g", markersize=3, linewidth=1, label="pitch error")
    for i, (ft, _fy, _fp) in enumerate(fire_events):
        ax_err.axvline(ft, color="r", linestyle="--", linewidth=1)
        ax_err.plot(ft, 0, "rv", markersize=8, label="ยิง" if i == 0 else None)
    ax_err.set_xlabel("time (s)")
    ax_err.set_ylabel("error (% ของเฟรม)")
    ax_err.set_title("Error response ของ PID (แสดงการลู่เข้าศูนย์กลางเป้า -- rise time / overshoot / settling)")
    ax_err.grid(True)
    ax_err.legend()

    plt.tight_layout()
    out_png = os.path.splitext(log_path)[0] + ".png"
    plt.savefig(out_png, dpi=150)
    print("บันทึกกราฟ: {0}".format(out_png))
    plt.show()
