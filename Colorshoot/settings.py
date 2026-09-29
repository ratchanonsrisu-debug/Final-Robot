"""
settings.py — ค่าตั้งค่าทั้งหมด เก็บเป็นไฟล์ settings.json (dashboard แก้แล้วกดบันทึกได้)
ทุกค่าที่เป็นตำแหน่งในภาพ เก็บแบบ normalize 0..1 เพื่อไม่ผูกกับความละเอียดภาพ
"""
import copy
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_PATH = os.path.join(HERE, "settings.json")

# ขนาดเฉพาะ "ส่วนที่มีสี" ของป้าย (กว้าง, สูง) หน่วย ซม.
SIGN_SIZES_CM = {
    "VERTICAL":   (6.0, 9.0),
    "HORIZONTAL": (9.0, 6.0),
    "SQUARE":     (7.0, 7.0),
    "CIRCLE":     (7.0, 7.0),    # รัศมี 3.5 ซม. -> เส้นผ่านศูนย์กลาง 7 ซม.
}

DEFAULTS = {
    "camera": {
        "resolution": "720p",
        "hfov_deg": 96.0,          # มุมมองแนวนอน (ใช้คำนวณ focal) — calibrate ได้ใน dashboard
    },
    # ช่วง HSV เริ่มต้น (OpenCV: H 0-180, S/V 0-255) — ควร calibrate จากภาพป้ายจริงด้วยปุ่ม "เรียนสีจากกรอบ"
    "colors": {
        "RED":    {"ranges": [[[0, 110, 70], [8, 255, 255]], [[170, 110, 70], [180, 255, 255]]], "draw": [0, 0, 255]},
        "YELLOW": {"ranges": [[[20, 165, 140], [34, 255, 255]]], "draw": [0, 230, 255]},   # กำแพงสนามจริงสีเหลือง S≈115-135
        "GREEN":  {"ranges": [[[40, 90, 50], [85, 255, 255]]], "draw": [0, 200, 0]},
        "BLUE":   {"ranges": [[[95, 100, 50], [130, 255, 255]]], "draw": [255, 120, 0]},
    },
    "vision": {
        "clahe": True,             # ปรับแสงด้วย CLAHE บนช่อง L ก่อนแปลง HSV
        "glare_v_min": 225,        # จุดสะท้อนแสงบนอะคริลิก: V สูง
        "glare_s_max": 70,         #                          S ต่ำ
        "glare_fill": True,        # เติมจุดสะท้อนที่อยู่ติดก้อนสีกลับเข้าไปในป้าย
        "min_area_px": 80,
        "min_rectangularity": 0.78,
        "circle_min_circularity": 0.80,   # (ไม่ใช้แล้ว เก็บไว้ให้ settings.json เก่าโหลดได้)
        "circle_min_fill": 0.74,   # วงกลม = พื้นที่ ÷ วงกลมล้อมรอบ ≥ ค่านี้ (วงกลม≈0.9 จัตุรัส≈0.64)
        "dist_min_cm": 15,
        "dist_max_cm": 320,
        "size_consistency": 0.35,  # ระยะจากความกว้าง vs ความสูง ต่างกันได้ไม่เกินกี่ %
        "bg_ratio": 1.0,           # ใช้เมื่อเรียนสีพื้นหลังแล้ว: P(ป้าย) ต้อง > ratio * P(พื้นหลัง)
    },
    # พื้นที่ตรวจจับ (normalize) และโซนที่ตัดทิ้ง
    "roi": {"x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 1.0},
    "exclude": [],                 # [[x0,y0,x1,y1], ...]
    "gimbal": {
        "home_pitch": 0.0,         # มุมล็อคตอนค้นหา/เริ่มต้น
        "home_yaw": 0.0,
        "yaw_min": -120.0, "yaw_max": 120.0,
        "pitch_min": -20.0, "pitch_max": 25.0,
        "yaw_sign": 1, "pitch_sign": 1,   # ระบบตรวจทิศเองอัตโนมัติ แล้วบันทึกค่า
        "manual_speed": 40,
    },
    "aim": {
        "tolerance_deg": 0.35,     # เล็งจนคลาดไม่เกินกี่องศา (0.35° ≈ 0.7 ซม. ที่ 1.2 ม.)
        "max_iters": 8,
        "gain": 0.9,
        "settle_s": 0.35,          # รอ gimbal นิ่ง + ภาพจากกล้องตามทัน (stream มี delay)
        "aim_y_ratio": 0.5,        # 0=ขอบบนสีของป้าย 0.5=กึ่งกลาง 1=ขอบล่าง
        "confirm_frames": 2,
        "ammo": "ir",              # ir หรือ gel
        # ยืนยันด้วยภาพว่าป้ายล้ม (เฉพาะเจล): ป้อนกระสุน EP ไม่สม่ำเสมอ สั่ง ~3 ครั้งออก 1 นัด
        "confirm_hit_gel": True, "max_triggers_gel": 8,
        "confirm_hit_ir": False, "max_triggers_ir": 1,
        "burst_gel": 3, "burst_ir": 1,   # สั่งยิง 1 ครั้ง = รัวกี่นัด (SDK 1-8) — max_triggers คือจำนวน "ชุด"
        "fire_hold_s_per_shot": 0.25,    # หลังสั่งยิง gimbal นิ่งอยู่ = นัด × ค่านี้ (ให้ยิงรัวครบก่อนหมุน)
        "confirm_wait_s": 0.8,     # รอป้ายล้ม/ภาพตามทันหลังยิง
        "confirm_knock_frames": 3, # ต้องไม่เห็นป้ายติดกันกี่เฟรมจึงถือว่าล้ม (กันตรวจจับหลุดเฟรมเดียว)
    },
    # calibrate จากป้ายจริง 4 ใบ (แท็บ "Calib 4 ป้าย")
    "calib_session": {
        "duration_s": 15.0,        # เก็บภาพกี่วินาที
        "interval_s": 0.3,         # เก็บ 1 ภาพทุกกี่วินาที
        "z_cm": 0.0,               # ระยะจากกล้องถึงแนวป้าย (0 = ไม่ทราบ -> ไม่ปรับ HFOV)
        # ป้ายที่วางไว้ [[สี, รูปทรง], ...] ใช้ตรวจว่ารูปทรงถูกไหม — ว่าง = ไม่ระบุ (ป้ายสีละ 1 ใบ รูปทรงอะไรก็ได้)
        "expected": [],
    },
    # โมเดลจุดกระสุนตกในภาพ:  u = cx + Ax + Bx/Z ,  v = cy + Ay + By/Z + Cy*Z   (px ที่ความกว้างภาพ base_w)
    "hit_model": {
        "base_w": 1280,
        "ir":  {"Ax": 0.0, "Bx": 0.0, "Ay": 0.0, "By": 0.0, "Cy": 0.0, "samples": []},
        # ค่าเริ่มต้นจากการทดสอบจริง 29 ก.ย. (วัดรอยน้ำมันด้วยไม้บรรทัด 1 นัด): เล็ง crosshair กลางป้าย 6x9 ที่ ~60 ซม.
        # รอยอยู่ ซ้าย 1.0 ซม., ต่ำ 2.6-2.9 ซม. (ใช้ 2.75) จากกึ่งกลาง  -> f=576px@1280, Z≈64 ซม. (กล้อง-ป้าย)
        #   dx = -1.0*576/64 = -9.0 px , dy = 2.75*576/64 = +24.8 px   (ระยะเดียว -> คลาดคงที่เป็น ซม. : B = d*Z)
        "gel": {"Ax": 0.0, "Bx": -576.0, "Ay": 0.0, "By": 1587.0, "Cy": 0.0, "samples": [[64.0, -9.0, 24.8]]},
    },
    "targets": {                   # สเปคเป้าที่อนุญาตให้ยิง
        "colors": ["RED", "YELLOW", "GREEN", "BLUE"],
        "shapes": ["VERTICAL", "HORIZONTAL", "SQUARE", "CIRCLE"],
        "pairs": [],               # ถ้าไม่ว่าง ยิงเฉพาะคู่ [สี, รูปทรง] ที่ระบุ
        "order": "left_to_right",
        "max_dist_cm": 0,          # ไม่ยิงป้ายที่ไกลกว่านี้ (กติกา ≤ 2 กระเบื้อง) 0 = ไม่จำกัด
    },
}


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def load(path=SETTINGS_PATH):
    s = copy.deepcopy(DEFAULTS)
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            _merge(s, json.load(f))
    return s


def save(s, path=SETTINGS_PATH):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(s, f, indent=1, ensure_ascii=False)
