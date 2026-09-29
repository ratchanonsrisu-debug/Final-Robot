"""
run.py
  python run.py              เปิด dashboard
  python run.py --selftest   ทดสอบทั้งระบบในฉากจำลอง (calibrate + ยิง) ไม่ต้องใช้หุ่น
  python run.py --calib-folder logs/calib_session_xxx/raw [--z 100] [--apply]
                             calibrate จากป้าย 4 ใบด้วยภาพที่เก็บไว้ (ไม่ต้องต่อหุ่น)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def selftest():
    """ฉากจำลอง: แสง IR ติด 0.25 วิ, ภาพช้า 0.3 วิ -> calibrate อัตโนมัติ 3 ระยะ -> รายงาน -> ทดสอบก่อน/หลัง -> ยิงจริง 5 ป้าย"""
    import time
    import copy
    import settings, vision, robot_io, aiming, calibration as CAL
    # ใช้ค่าเริ่มต้นเสมอ (settings.json เป็นค่าของกล้องจริง: สี/ROI/โซนตัดทิ้ง ไม่ตรงกับฉากจำลอง)
    S = copy.deepcopy(settings.DEFAULTS); S["aim"]["settle_s"] = 0.1
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "selftest")
    sim = robot_io.SimEP(seed=7, latency_s=0.3, flash_s=0.25)
    g = robot_io.Grabbed(sim); time.sleep(1.0)
    m = aiming.HitModel(S)
    S["hit_model"]["ir"]["samples"] = []
    for z in (60, 90, 130):
        sim.scene = []; sim.wall_z = z; time.sleep(0.4)
        CAL.auto_calibrate(g, m, z, "ir", shots=3, out_dir=out)
    f = g.peek()
    print(CAL.fit_report(m, "ir", f.shape[1], f.shape[0], out_dir=out))
    sim.wall_z = 102; sim.scene = [("RED", "VERTICAL", 0, 0, 100)]; time.sleep(0.4)
    aim = aiming.Aimer(g, vision.SignDetector(S), vision.Tracker(), S, log=lambda *a: None)
    print(CAL.accuracy_test(aim, g, trials=4, ammo="ir", out_dir=out))
    sim.scene = robot_io.SimEP.default_scene(); sim.wall_z = 180; sim.hits = []; sim.recenter()
    aim.log = print
    res = aim.engage_all(fire=True, ammo="ir")
    ok = [h for h in sim.hits if h]
    print(f"[ir] ยิง {len(res)} ป้าย โดน {len(ok)}: {ok}")
    print(f"ผลทั้งหมดอยู่ที่ {out}")
    g.close()


def calib_folder(argv):
    """calibrate จากป้าย 4 ใบ โดยใช้ภาพที่เก็บไว้แล้ว (เช่น logs/calib_session_xxx/raw) ไม่ต้องต่อหุ่น
    python run.py --calib-folder <โฟลเดอร์หรือไฟล์ภาพ> [--z 100] [--apply]"""
    import cv2
    import settings, session_calib as SC
    path = argv[argv.index("--calib-folder") + 1]
    z = float(argv[argv.index("--z") + 1]) if "--z" in argv else None
    S = settings.load()
    if os.path.isdir(path):
        frames = SC.load_folder(path)
    else:
        img = cv2.imread(path)
        frames = [] if img is None else [(0.0, img, None)]
    r = SC.analyze(frames, S, z_cm=z, apply=True, save_raw=not os.path.isdir(path))
    if r and "--apply" in argv:
        settings.save(S)
        print(f"บันทึกลง {settings.SETTINGS_PATH} แล้ว")
    elif r:
        print("(ยังไม่บันทึกลง settings.json — ใส่ --apply ถ้าต้องการ หรือคัดลอก learned_settings.json)")


if __name__ == "__main__":
    if "--calib-folder" in sys.argv:
        calib_folder(sys.argv)
    elif "--selftest" in sys.argv:
        selftest()
    else:
        from dashboard import Dashboard
        Dashboard().run()
