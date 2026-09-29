"""
dashboard.py — หน้าจอควบคุมการตรวจจับ/เล็ง/ยิงป้ายสี (Tkinter)

เมาส์บนภาพ (เลือกโหมดทางขวา):
  • ลาก ROI              : กำหนดพื้นที่ตรวจจับ
  • ลากโซนตัดทิ้ง         : ลากกรอบทับจุดที่ไม่ต้องการ (คลิกขวาที่โซนเพื่อลบ)
  • ลากเรียนสีป้าย        : ลากกรอบ "ในเนื้อสี" ของป้าย -> เรียนช่วง HSV + histogram ของสีที่เลือก
  • ลากเรียนสีพื้นหลัง    : ลากกรอบบนกำแพง/ของที่ชอบโดนจับผิด -> ใช้ตัดออก (back-projection)
  • คลิกยิงป้ายนี้         : คลิกที่ป้ายในภาพ -> เล็ง+ยิงป้ายนั้น
  • คลิกจุด IR            : คลิกจุดที่กระสุน/IR ไปโดนจริง -> เป็นตัวอย่าง calibrate
คีย์บอร์ด: W/A/S/D หมุน gimbal, Space = หยุดทุกอย่าง
"""
import csv
import datetime
import math
import os
import queue
import threading
import time
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

import cv2
import numpy as np
from PIL import Image, ImageTk

import settings as ST
import vision
from aiming import Aimer, HitModel, pixel_to_angle, target_point
from robot_io import Grabbed
import calibration as CAL
from vision import focal_px

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
CALIB_DIR = os.path.join(LOG_DIR, "calib")
SHAPES = ["VERTICAL", "HORIZONTAL", "SQUARE", "CIRCLE"]


class Dashboard:
    def __init__(self):
        self.S = ST.load()
        self.root = tk.Tk()
        self.root.title("RoboMaster EP — Color Sign Shooter")
        self.q = queue.Queue()
        self.src = None
        self.det = vision.SignDetector(self.S)
        self.trk = vision.Tracker()
        self.model = HitModel(self.S)
        self.stop_event = threading.Event()
        self.worker = None
        self.aimer = None
        self.dets = []
        self.drag = None
        self.disp_scale = 1.0
        self.frame_size = (1280, 720)
        self.key_t = {}
        self.ir_pending = None
        self._showmask = False
        os.makedirs(LOG_DIR, exist_ok=True)
        self._build()
        threading.Thread(target=self._vision_loop, daemon=True).start()
        self.root.after(60, self._tick)

    # ================================================================ UI
    def _build(self):
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.CW = max(560, min(960, sw - 470))
        self.CH = int(self.CW * 9 / 16)
        main = ttk.Frame(self.root); main.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(main); left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)
        right = ttk.Frame(main, width=440); right.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self.status = tk.StringVar(value="ยังไม่เชื่อมต่อ — เลือกแหล่งภาพแล้วกด เชื่อมต่อ")
        ttk.Label(left, textvariable=self.status, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.cv = tk.Canvas(left, width=self.CW, height=self.CH, bg="black", highlightthickness=0)
        self.cv.pack()
        self.cv.bind("<ButtonPress-1>", self._m_down)
        self.cv.bind("<B1-Motion>", self._m_move)
        self.cv.bind("<ButtonRelease-1>", self._m_up)
        self.cv.bind("<Button-3>", self._m_right)
        self.logbox = ScrolledText(left, height=8, font=("Consolas", 9))
        self.logbox.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        for k in "wasdWASD":
            self.root.bind(f"<KeyPress-{k}>", self._key)
        self.root.bind("<space>", lambda e: self.stop())

        nb = ttk.Notebook(right); nb.pack(fill=tk.BOTH, expand=True)
        t1, t2, t3, t4, t5 = (ttk.Frame(nb, padding=6) for _ in range(5))
        nb.add(t1, text="ยิง"); nb.add(t2, text="สี / ROI"); nb.add(t3, text="Gimbal"); nb.add(t4, text="Calibrate")
        nb.add(t5, text="Calib 4 ป้าย")
        self._build_session_tab(t5)

        # ---------------- tab ยิง ----------------
        f = ttk.LabelFrame(t1, text="แหล่งภาพ", padding=4); f.pack(fill=tk.X)
        self.v_src = tk.StringVar(value="sim")
        for v, t in (("robot", "หุ่นจริง (Wi-Fi AP)"), ("sim", "จำลอง"), ("file", "ไฟล์ภาพ/วิดีโอ/โฟลเดอร์")):
            ttk.Radiobutton(f, text=t, value=v, variable=self.v_src).pack(anchor="w")
        r = ttk.Frame(f); r.pack(fill=tk.X, pady=2)
        ttk.Button(r, text="เชื่อมต่อ", command=self.connect).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(r, text="ภาพถัดไป ▶", command=lambda: self._file_step(1)).pack(side=tk.LEFT)

        f = ttk.LabelFrame(t1, text="กระสุน", padding=4); f.pack(fill=tk.X, pady=4)
        self.v_ammo = tk.StringVar(value=self.S["aim"]["ammo"])
        ttk.Radiobutton(f, text="IR (ปลอดภัย)", value="ir", variable=self.v_ammo).pack(side=tk.LEFT)
        ttk.Radiobutton(f, text="กระสุนเจล", value="gel", variable=self.v_ammo).pack(side=tk.LEFT)
        self.v_gel_ok = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="ยืนยันใช้เจล", variable=self.v_gel_ok).pack(side=tk.LEFT)

        f = ttk.LabelFrame(t1, text="เป้าที่อนุญาตให้ยิง", padding=4); f.pack(fill=tk.X)
        self.v_col = {}
        rc = ttk.Frame(f); rc.pack(fill=tk.X)
        for c in self.S["colors"]:
            self.v_col[c] = tk.BooleanVar(value=c in self.S["targets"]["colors"])
            ttk.Checkbutton(rc, text=c, variable=self.v_col[c], command=self._targets_changed).pack(side=tk.LEFT)
        self.v_shp = {}
        rs = ttk.Frame(f); rs.pack(fill=tk.X)
        for s, t in zip(SHAPES, ("ตั้ง 6x9", "นอน 9x6", "จัตุรัส 7x7", "วงกลม ø7")):
            self.v_shp[s] = tk.BooleanVar(value=s in self.S["targets"]["shapes"])
            ttk.Checkbutton(rs, text=t, variable=self.v_shp[s], command=self._targets_changed).pack(side=tk.LEFT)
        ttk.Label(f, text="คู่เฉพาะ (เว้นว่าง=ทุกคู่ที่ติ๊ก) เช่น RED:HORIZONTAL, BLUE:CIRCLE").pack(anchor="w")
        self.v_pairs = tk.StringVar(value=", ".join(f"{a}:{b}" for a, b in self.S["targets"]["pairs"]))
        e = ttk.Entry(f, textvariable=self.v_pairs); e.pack(fill=tk.X)
        e.bind("<FocusOut>", lambda ev: self._targets_changed())
        self.v_order = tk.StringVar(value=self.S["targets"]["order"])
        ro = ttk.Frame(f); ro.pack(fill=tk.X)
        ttk.Radiobutton(ro, text="ซ้าย→ขวา", value="left_to_right", variable=self.v_order, command=self._targets_changed).pack(side=tk.LEFT)
        ttk.Radiobutton(ro, text="ใกล้สุดก่อน", value="nearest", variable=self.v_order, command=self._targets_changed).pack(side=tk.LEFT)
        rd = ttk.Frame(f); rd.pack(fill=tk.X)
        ttk.Label(rd, text="ยิงเฉพาะป้ายที่ใกล้กว่า (ซม., 0=ไม่จำกัด)").pack(side=tk.LEFT)
        self.v_maxd = tk.DoubleVar(value=self.S["targets"].get("max_dist_cm", 0))
        ttk.Spinbox(rd, from_=0, to=400, increment=10, textvariable=self.v_maxd, width=6,
                    command=self._targets_changed).pack(side=tk.LEFT)

        f = ttk.LabelFrame(t1, text="สั่งงาน", padding=4); f.pack(fill=tk.X, pady=4)
        ttk.Button(f, text="🎯 ยิงทุกเป้าที่ตรงสเปคในภาพ", command=lambda: self.engage(True)).pack(fill=tk.X)
        ttk.Button(f, text="เล็งอย่างเดียว (ไม่ยิง)", command=lambda: self.engage(False)).pack(fill=tk.X, pady=2)
        ttk.Button(f, text="■ STOP (Space)", command=self.stop).pack(fill=tk.X)
        ra = ttk.Frame(f); ra.pack(fill=tk.X, pady=2)
        ttk.Label(ra, text="จุดเล็งบนป้าย (0=บน .5=กลาง 1=ล่าง)").pack(side=tk.LEFT)
        self.v_aimy = tk.DoubleVar(value=self.S["aim"]["aim_y_ratio"])
        ttk.Spinbox(ra, from_=0, to=1, increment=0.1, textvariable=self.v_aimy, width=5).pack(side=tk.LEFT)
        rt = ttk.Frame(f); rt.pack(fill=tk.X)
        ttk.Label(rt, text="tolerance (องศา)").pack(side=tk.LEFT)
        self.v_tol = tk.DoubleVar(value=self.S["aim"]["tolerance_deg"])
        ttk.Spinbox(rt, from_=0.1, to=2, increment=0.05, textvariable=self.v_tol, width=5).pack(side=tk.LEFT)
        rg = ttk.Frame(f); rg.pack(fill=tk.X)
        self.v_confirm = tk.BooleanVar(value=self.S["aim"].get("confirm_hit_gel", True))
        ttk.Checkbutton(rg, text="เจล: ยิงซ้ำจนป้ายล้ม สูงสุด", variable=self.v_confirm).pack(side=tk.LEFT)
        self.v_maxtrig = tk.IntVar(value=self.S["aim"].get("max_triggers_gel", 8))
        ttk.Spinbox(rg, from_=1, to=15, textvariable=self.v_maxtrig, width=4).pack(side=tk.LEFT)
        ttk.Label(rg, text="ชุด").pack(side=tk.LEFT)
        rb = ttk.Frame(f); rb.pack(fill=tk.X)
        ttk.Label(rb, text="เจล: ยิงรัวชุดละ").pack(side=tk.LEFT)
        self.v_burst = tk.IntVar(value=self.S["aim"].get("burst_gel", 3))
        ttk.Spinbox(rb, from_=1, to=8, textvariable=self.v_burst, width=4).pack(side=tk.LEFT)
        ttk.Label(rb, text="นัด").pack(side=tk.LEFT)
        ttk.Button(t1, text="💾 บันทึกค่าทั้งหมด (settings.json)", command=self.save_settings).pack(fill=tk.X, pady=6)

        # ---------------- tab สี / ROI ----------------
        f = ttk.LabelFrame(t2, text="โหมดเมาส์บนภาพ", padding=4); f.pack(fill=tk.X)
        self.v_mouse = tk.StringVar(value="shoot")
        for v, t in (("roi", "ลาก ROI (พื้นที่ตรวจจับ)"), ("exclude", "ลากโซนตัดทิ้ง (คลิกขวา=ลบโซน)"),
                     ("learn", "ลากเรียนสีป้าย (ในเนื้อสี)"), ("learn_bg", "ลากเรียนสีพื้นหลัง/ของปลอม"),
                     ("shoot", "คลิกยิงป้ายนี้"), ("ir", "คลิกจุด IR/กระสุนที่เห็น (calibrate)")):
            ttk.Radiobutton(f, text=t, value=v, variable=self.v_mouse).pack(anchor="w")
        r = ttk.Frame(f); r.pack(fill=tk.X, pady=2)
        ttk.Button(r, text="ROI เต็มจอ", command=self.reset_roi).pack(side=tk.LEFT)
        ttk.Button(r, text="ลบโซนทั้งหมด", command=self.clear_exclude).pack(side=tk.LEFT)
        ttk.Button(r, text="ล้างพื้นหลัง", command=self.clear_bg).pack(side=tk.LEFT)

        f = ttk.LabelFrame(t2, text="สีที่เลือก", padding=4); f.pack(fill=tk.X, pady=4)
        r = ttk.Frame(f); r.pack(fill=tk.X)
        self.v_color = tk.StringVar(value="RED")
        self.cb_color = ttk.Combobox(r, values=list(self.S["colors"]), textvariable=self.v_color, width=10, state="readonly")
        self.cb_color.pack(side=tk.LEFT)
        self.cb_color.bind("<<ComboboxSelected>>", lambda e: self._load_sliders())
        ttk.Button(r, text="+ สีใหม่", command=self.add_color).pack(side=tk.LEFT)
        ttk.Button(r, text="ล้าง histogram", command=self.clear_hist).pack(side=tk.LEFT)
        self.v_learn_add = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="เรียนเพิ่ม (รวมกับที่เรียนไว้ เช่น ป้ายเดิมหลายมุมแสง)", variable=self.v_learn_add).pack(anchor="w")
        self.sl = {}
        for key, lo, hi in (("H lo", 0, 180), ("H hi", 0, 180), ("S lo", 0, 255), ("V lo", 0, 255)):
            rr = ttk.Frame(f); rr.pack(fill=tk.X)
            ttk.Label(rr, text=key, width=5).pack(side=tk.LEFT)
            v = tk.IntVar()
            s = tk.Scale(rr, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=v, showvalue=True, length=260,
                         command=lambda _=None: self._sliders_changed())
            s.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sl[key] = v
        self.v_showmask = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="ดู mask ของสีนี้ (ขาว = ผ่าน)", variable=self.v_showmask).pack(anchor="w")
        self.lbl_colorinfo = ttk.Label(f, text="", wraplength=400); self.lbl_colorinfo.pack(anchor="w")

        f = ttk.LabelFrame(t2, text="ตัวกรอง", padding=4); f.pack(fill=tk.X)
        self.v_vis = {}
        for key, t, a, b, inc in (("glare_v_min", "แสงสะท้อน V ≥", 150, 255, 5), ("glare_s_max", "แสงสะท้อน S ≤", 0, 150, 5),
                                  ("min_area_px", "พื้นที่ขั้นต่ำ px", 20, 2000, 10), ("dist_max_cm", "ระยะไกลสุด ซม.", 50, 500, 10),
                                  ("size_consistency", "ขนาดกว้าง/สูงคลาดได้", 0.1, 1.0, 0.05), ("bg_ratio", "ความเข้มตัดพื้นหลัง", 0.2, 5, 0.1)):
            rr = ttk.Frame(f); rr.pack(fill=tk.X)
            ttk.Label(rr, text=t, width=20).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=self.S["vision"][key])
            sp = ttk.Spinbox(rr, from_=a, to=b, increment=inc, textvariable=v, width=7,
                             command=self._vision_changed)
            sp.pack(side=tk.LEFT)
            sp.bind("<FocusOut>", lambda e: self._vision_changed())
            self.v_vis[key] = v
        self.v_clahe = tk.BooleanVar(value=self.S["vision"]["clahe"])
        ttk.Checkbutton(f, text="CLAHE ปรับแสง", variable=self.v_clahe, command=self._vision_changed).pack(anchor="w")
        r = ttk.Frame(t2); r.pack(fill=tk.X, pady=4)
        self.v_freeze = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="หยุดภาพ (freeze)", variable=self.v_freeze, command=self._freeze).pack(side=tk.LEFT)
        ttk.Button(r, text="📷 เซฟภาพ", command=self.snapshot).pack(side=tk.LEFT, padx=4)
        self._load_sliders()

        # ---------------- tab Gimbal ----------------
        G = self.S["gimbal"]
        f = ttk.LabelFrame(t3, text="มุมล็อค (มุมเริ่มต้น/ค้นหา)", padding=4); f.pack(fill=tk.X)
        self.v_g = {}
        for key, t in (("home_pitch", "pitch (+เงย)"), ("home_yaw", "yaw (+ขวา)")):
            rr = ttk.Frame(f); rr.pack(fill=tk.X)
            ttk.Label(rr, text=t, width=14).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=G[key]); self.v_g[key] = v
            ttk.Spinbox(rr, from_=-250, to=250, increment=1, textvariable=v, width=7).pack(side=tk.LEFT)
        r = ttk.Frame(f); r.pack(fill=tk.X, pady=2)
        ttk.Button(r, text="ไปมุมล็อค", command=self.go_home).pack(side=tk.LEFT)
        ttk.Button(r, text="ใช้มุมปัจจุบันเป็นมุมล็อค", command=self.set_home_here).pack(side=tk.LEFT)
        ttk.Button(r, text="recenter", command=lambda: self.run_bg(lambda: self.src.recenter(), "recenter")).pack(side=tk.LEFT)
        f = ttk.LabelFrame(t3, text="ขอบเขตการหมุน (กันหมุนไปยิงนอกสนาม)", padding=4); f.pack(fill=tk.X, pady=4)
        for key, t in (("yaw_min", "yaw ต่ำสุด"), ("yaw_max", "yaw สูงสุด"), ("pitch_min", "pitch ต่ำสุด"), ("pitch_max", "pitch สูงสุด")):
            rr = ttk.Frame(f); rr.pack(fill=tk.X)
            ttk.Label(rr, text=t, width=14).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=G[key]); self.v_g[key] = v
            ttk.Spinbox(rr, from_=-250, to=250, increment=1, textvariable=v, width=7).pack(side=tk.LEFT)
        f = ttk.LabelFrame(t3, text="หมุนด้วยมือ (หรือ W A S D)", padding=4); f.pack(fill=tk.X)
        pad = ttk.Frame(f); pad.pack()
        ttk.Button(pad, text="▲", width=4, command=lambda: self.nudge(2, 0)).grid(row=0, column=1)
        ttk.Button(pad, text="◀", width=4, command=lambda: self.nudge(0, -2)).grid(row=1, column=0)
        ttk.Button(pad, text="▶", width=4, command=lambda: self.nudge(0, 2)).grid(row=1, column=2)
        ttk.Button(pad, text="▼", width=4, command=lambda: self.nudge(-2, 0)).grid(row=2, column=1)
        ttk.Button(f, text="ตรวจทิศการหมุน gimbal อัตโนมัติ", command=self.verify_dirs).pack(fill=tk.X, pady=4)
        self.lbl_g = ttk.Label(t3, text=""); self.lbl_g.pack(anchor="w")

        # ---------------- tab Calibrate ----------------
        f = ttk.LabelFrame(t4, text="1) มุมมองกล้อง (HFOV) — ใช้คำนวณระยะ", padding=4); f.pack(fill=tk.X)
        rr = ttk.Frame(f); rr.pack(fill=tk.X)
        ttk.Label(rr, text="HFOV (องศา)").pack(side=tk.LEFT)
        self.v_hfov = tk.DoubleVar(value=self.S["camera"]["hfov_deg"])
        sp = ttk.Spinbox(rr, from_=40, to=140, increment=0.5, textvariable=self.v_hfov, width=7,
                         command=self._hfov_changed)
        sp.pack(side=tk.LEFT); sp.bind("<FocusOut>", lambda e: self._hfov_changed())
        rr = ttk.Frame(f); rr.pack(fill=tk.X)
        ttk.Label(rr, text="วางป้าย 1 ใบที่ระยะจริง (ซม.)").pack(side=tk.LEFT)
        self.v_known = tk.DoubleVar(value=100.0)
        ttk.Entry(rr, textvariable=self.v_known, width=7).pack(side=tk.LEFT)
        ttk.Button(rr, text="คำนวณ HFOV", command=self.calib_hfov).pack(side=tk.LEFT)

        f = ttk.LabelFrame(t4, text="2) จุดกระทบ (IR / เจล)  u = A + B/Z (+C·Z)", padding=4); f.pack(fill=tk.X, pady=4)
        ttk.Label(f, text="ตั้งกระดานขาวด้าน/กำแพงเรียบ ตั้งฉากกับหุ่นที่ระยะ Z แล้วกด Auto\n"
                          "โปรแกรมยิงเอง + บันทึกทุกเฟรม + หาแสงแฟลชเอง (ไม่ต้องกดทัน)\n"
                          "ทำ 3 ระยะขึ้นไป เช่น 60 / 90 / 120 ซม. แล้วกด สรุปผล",
                  justify=tk.LEFT).pack(anchor="w")
        rr = ttk.Frame(f); rr.pack(fill=tk.X)
        ttk.Label(rr, text="Z (ซม.)").pack(side=tk.LEFT)
        self.v_calz = tk.DoubleVar(value=90.0)
        ttk.Entry(rr, textvariable=self.v_calz, width=7).pack(side=tk.LEFT)
        ttk.Button(rr, text="ใช้ระยะป้ายที่เห็น", command=self.z_from_sign).pack(side=tk.LEFT)
        ttk.Label(rr, text=" นัด").pack(side=tk.LEFT)
        self.v_nshot = tk.IntVar(value=5)
        ttk.Spinbox(rr, from_=1, to=20, textvariable=self.v_nshot, width=4).pack(side=tk.LEFT)
        self.v_zsign = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="วัด Z จากป้ายที่แปะบนกระดานทุกนัด (ไม่ต้องใช้ตลับเมตร)", variable=self.v_zsign).pack(anchor="w")
        ttk.Button(f, text="▶ Auto: ยิง N นัดที่ระยะนี้ + จับภาพเอง", command=self.calib_auto).pack(fill=tk.X, pady=2)
        ttk.Button(f, text="📊 สรุปผล (Fit + รายงาน + กราฟ)", command=self.calib_fit).pack(fill=tk.X)
        ttk.Button(f, text="🎯 ทดสอบความแม่น ก่อน/หลัง (ติดป้าย 1 ใบบนกระดาน)", command=self.calib_test).pack(fill=tk.X, pady=2)
        ttk.Button(f, text="สำรอง: ยิง 1 นัด แล้วคลิกจุดเอง (ภาพหยุดที่เฟรมแฟลช)", command=self.calib_shot_click).pack(fill=tk.X)
        rr = ttk.Frame(f); rr.pack(fill=tk.X, pady=2)
        ttk.Button(rr, text="ลบตัวล่าสุด", command=self.calib_undo).pack(side=tk.LEFT)
        ttk.Button(rr, text="ล้างทั้งหมด", command=self.calib_clear).pack(side=tk.LEFT)
        ttk.Button(rr, text="เปิดโฟลเดอร์ผล", command=self.open_calib_dir).pack(side=tk.LEFT)
        self.lbl_cal = ttk.Label(f, text="", wraplength=400, justify=tk.LEFT); self.lbl_cal.pack(anchor="w")
        f = ttk.LabelFrame(t4, text="2b) วัดรอยกระสุนเจลจริงด้วยไม้บรรทัด (เจลไม่มีแสง)", padding=4); f.pack(fill=tk.X, pady=4)
        ttk.Label(f, text="แปะกระดาษ/ป้ายบนกระดานที่ระยะ Z → กดปุ่มด้านล่าง (เล็งกลางป้ายด้วย crosshair แบบแอป DJI แล้วยิง)\n"
                          "วัดว่ารอยห่างจากกลางป้ายกี่ ซม. (ขวา=+, ลง=+) แล้วกดเพิ่ม ทำ 2-3 ระยะ แล้วกด 📊 สรุปผล",
                  justify=tk.LEFT, wraplength=420).pack(anchor="w")
        ttk.Button(f, text="เล็ง crosshair กลางป้าย + ยิง 1 ครั้ง", command=self.crosshair_shot).pack(fill=tk.X)
        rr = ttk.Frame(f); rr.pack(fill=tk.X, pady=2)
        self.v_rz = tk.DoubleVar(value=60.0); self.v_rdx = tk.DoubleVar(value=0.0); self.v_rdy = tk.DoubleVar(value=0.0)
        for t, v in (("Z ซม.", self.v_rz), ("ขวา ซม.", self.v_rdx), ("ลง ซม.", self.v_rdy)):
            ttk.Label(rr, text=t).pack(side=tk.LEFT)
            ttk.Entry(rr, textvariable=v, width=6).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(rr, text="เพิ่ม", command=self.add_ruler_sample).pack(side=tk.LEFT)
        f = ttk.LabelFrame(t4, text="3) ปรับละเอียดหลังยิงจริง", padding=4); f.pack(fill=tk.X)
        ttk.Label(f, text="ถ้ายังคลาดทุกนัดไปทางเดียวกัน ปรับ A (px ที่ภาพกว้าง 1280)\n+Ax = เลื่อนจุดตกไปขวา, +Ay = ลง",
                  justify=tk.LEFT).pack(anchor="w")
        rr = ttk.Frame(f); rr.pack(fill=tk.X)
        for t, dx, dy in (("◀ 2px", -2, 0), ("▶ 2px", 2, 0), ("▲ 2px", 0, -2), ("▼ 2px", 0, 2)):
            ttk.Button(rr, text=t, command=lambda dx=dx, dy=dy: self.trim(dx, dy)).pack(side=tk.LEFT)
        self._update_cal_label()

    def _build_session_tab(self, t5):
        C = self.S["calib_session"]
        f = ttk.LabelFrame(t5, text="Calibrate จากป้ายจริง 4 ใบ (อัตโนมัติ)", padding=4); f.pack(fill=tk.X)
        ttk.Label(f, text="1. วางป้าย 4 ใบ (ครบทุกสี/รูปทรง) เรียงหน้ากล้อง ให้เห็นครบในภาพ\n"
                          "2. หุ่นและ gimbal อยู่นิ่ง ระหว่างเก็บภาพห้ามเดินผ่านหน้ากล้อง\n"
                          "3. กดเริ่ม โปรแกรมเก็บภาพเอง -> เรียนสี + เช็ครูปทรง + (ถ้าใส่ระยะ) หา HFOV\n"
                          "ผล + ภาพทั้งหมดอยู่ที่ logs/calib_session_<เวลา>/ (raw, annotated, masks, csv, summary)",
                  justify=tk.LEFT, wraplength=420).pack(anchor="w")
        self.v_ss = {}
        for key, t, a, b, inc in (("duration_s", "เก็บภาพนาน (วินาที)", 3, 120, 1),
                                  ("interval_s", "เก็บ 1 ภาพทุก (วินาที)", 0.1, 5, 0.1),
                                  ("z_cm", "ระยะกล้อง→แนวป้าย ซม. (0=ไม่ทราบ)", 0, 400, 5)):
            rr = ttk.Frame(f); rr.pack(fill=tk.X)
            ttk.Label(rr, text=t, width=30).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=C[key]); self.v_ss[key] = v
            ttk.Spinbox(rr, from_=a, to=b, increment=inc, textvariable=v, width=7).pack(side=tk.LEFT)
        ttk.Label(f, text="ป้ายที่วาง (สี:รูปทรง) เว้นว่าง = ไม่เช็ครูปทรง (แนะนำ)").pack(anchor="w")
        self.v_ss_exp = tk.StringVar(value=", ".join(f"{a}:{b}" for a, b in C["expected"]))
        ttk.Entry(f, textvariable=self.v_ss_exp).pack(fill=tk.X)
        self.v_ss_apply = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="ใช้ค่าสี/เกณฑ์รูปทรงที่ได้ทันที", variable=self.v_ss_apply).pack(anchor="w")
        self.v_ss_hfov = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="ปรับ HFOV จากระยะที่ใส่ (ถ้าป้ายทุกใบให้ผลตรงกัน)", variable=self.v_ss_hfov).pack(anchor="w")
        ttk.Button(f, text="▶ เริ่มเก็บภาพ + calibrate", command=self.session_calib).pack(fill=tk.X, pady=(4, 2))
        rr = ttk.Frame(f); rr.pack(fill=tk.X)
        ttk.Button(rr, text="↩ ย้อนค่าก่อน calibrate", command=self.session_undo).pack(side=tk.LEFT)
        ttk.Button(rr, text="ใช้ ROI ที่แนะนำ", command=self.session_use_roi).pack(side=tk.LEFT)
        ttk.Button(rr, text="เปิดโฟลเดอร์ผล", command=self.session_open).pack(side=tk.LEFT)
        ttk.Button(f, text="💾 บันทึกค่าทั้งหมด (settings.json)", command=self.save_settings).pack(fill=tk.X, pady=4)
        self.lbl_ss = ttk.Label(f, text="", wraplength=420, justify=tk.LEFT); self.lbl_ss.pack(anchor="w")
        self._session = None

    # ---------------- calib 4 ป้าย ----------------
    def _session_params(self):
        C = self.S["calib_session"]
        for k, v in self.v_ss.items():
            C[k] = float(v.get())
        exp = []
        for tok in self.v_ss_exp.get().replace(";", ",").split(","):
            if ":" in tok:
                a, b = [x.strip().upper() for x in tok.split(":", 1)]
                if a in self.S["colors"] and b in SHAPES:
                    exp.append([a, b])
        C["expected"] = exp          # ว่าง = ไม่ระบุรูปทรง
        return C

    def session_calib(self):
        C = self._session_params()
        self.v_freeze.set(False); self._freeze()
        apply, apply_hfov = bool(self.v_ss_apply.get()), bool(self.v_ss_hfov.get())
        def job():
            import session_calib as SC
            r = SC.run_session(self.src, self.S, z_cm=C["z_cm"] or None, duration_s=C["duration_s"],
                               interval_s=C["interval_s"], expected=C["expected"], apply=apply,
                               apply_hfov=apply_hfov, log=self.log, stop=self.stop_event)
            if r is None:
                return
            self._session = r
            keys = [k for k in r["detection_new"] if not k.startswith("_")]
            ok = [k for k in keys if r["detection_new"][k]["rate"] >= 0.9]
            self.q.put(("session", f"ล่าสุด: เจอ ≥90% ของภาพ {len(ok)}/{len(keys)} ป้าย  "
                                   f"HFOV {r['hfov_after']}°\n{r['out_dir']}"))
            self.q.put(("reload", None))
        self.run_bg(job, f"CALIB 4 ป้าย ({C['duration_s']:.0f} วิ)")

    def session_undo(self):
        if not self._session:
            self.log("ยังไม่มี session ให้ย้อน"); return
        b = self._session["backup"]
        for k in ("colors", "vision", "camera", "roi"):
            self.S[k] = b[k]
        self.log("ย้อนค่าสี/เกณฑ์/HFOV/ROI กลับเป็นก่อน calibrate แล้ว")
        self._session = None
        self.q.put(("reload", None))

    def session_use_roi(self):
        roi = self._session and self._session.get("roi_suggested")
        if not roi:
            self.log("ยังไม่มี ROI ที่แนะนำ (calibrate ก่อน)"); return
        self.S["roi"] = dict(roi)
        self.log(f"ROI = {roi}  (ดูภาพใน annotated/ ก่อนใช้จริง ป้ายที่ระยะอื่นอาจอยู่สูง/ต่ำกว่านี้)")

    def session_open(self):
        d = self._session["out_dir"] if self._session else LOG_DIR
        try:
            os.startfile(d)
        except Exception:
            self.log(f"ผลอยู่ที่: {d}")

    def _reload_ui(self):
        """ค่าใน S ถูกแก้จาก thread อื่น -> อัปเดตช่องต่างๆ บนหน้าจอให้ตรง"""
        self.v_hfov.set(self.S["camera"]["hfov_deg"])
        for k, v in self.v_vis.items():
            v.set(self.S["vision"][k])
        self.v_clahe.set(self.S["vision"]["clahe"])
        self._load_sliders()

    # ================================================================ helpers
    def log(self, s):
        self.q.put(s)

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def run_bg(self, fn, name):
        if self.src is None:
            messagebox.showwarning("", "ยังไม่เชื่อมต่อแหล่งภาพ"); return
        if self.busy():
            messagebox.showwarning("", "กำลังทำงานอยู่ (กด STOP ก่อน)"); return
        self.stop_event.clear()
        self._sync_settings()

        def wrap():
            try:
                fn()
            except Exception as e:
                self.log(f"ERROR {name}: {e!r}\n{traceback.format_exc()}")
        self.log(f"--- {name} ---")
        self.worker = threading.Thread(target=wrap, daemon=True)
        self.worker.start()

    def _sync_settings(self):
        A = self.S["aim"]
        A["aim_y_ratio"] = float(self.v_aimy.get()); A["tolerance_deg"] = float(self.v_tol.get())
        A["ammo"] = self.v_ammo.get()
        A["confirm_hit_gel"] = bool(self.v_confirm.get()); A["max_triggers_gel"] = int(self.v_maxtrig.get())
        A["burst_gel"] = max(1, min(8, int(self.v_burst.get())))
        for k, v in self.v_g.items():
            self.S["gimbal"][k] = float(v.get())
        self._targets_changed()

    def _targets_changed(self):
        T = self.S["targets"]
        T["colors"] = [c for c, v in self.v_col.items() if v.get()]
        T["shapes"] = [s for s, v in self.v_shp.items() if v.get()]
        pairs = []
        for tok in self.v_pairs.get().replace(";", ",").split(","):
            if ":" in tok:
                a, b = [x.strip().upper() for x in tok.split(":", 1)]
                if a in self.S["colors"] and b in SHAPES:
                    pairs.append([a, b])
        T["pairs"] = pairs
        T["order"] = self.v_order.get()
        try:
            T["max_dist_cm"] = float(self.v_maxd.get())
        except Exception:
            pass

    def save_settings(self):
        self._sync_settings()
        ST.save(self.S)
        self.log(f"บันทึก settings -> {ST.SETTINGS_PATH}")

    # ================================================================ connect
    def connect(self):
        if self.busy():
            return
        if self.src is not None:
            self.src.close(); self.src = None
        kind = self.v_src.get()
        try:
            from robot_io import SimEP, FileSource, RealEP
            if kind == "sim":
                raw = SimEP(hfov=self.S["camera"]["hfov_deg"])
            elif kind == "file":
                p = filedialog.askopenfilename(title="เลือกรูปหรือวิดีโอ (ยกเลิก = เลือกโฟลเดอร์)")
                if not p:
                    p = filedialog.askdirectory(title="เลือกโฟลเดอร์รูป")
                if not p:
                    return
                raw = FileSource(p)
            else:
                self.status.set("กำลังเชื่อมต่อหุ่น…")
                self.root.update()
                raw = RealEP(self.S["camera"]["resolution"], log=self.log)
        except Exception as e:
            messagebox.showerror("เชื่อมต่อไม่สำเร็จ", f"{e!r}\n\nหุ่นจริง: ต่อ Wi-Fi ของหุ่น (AP) + pip install robomaster")
            return
        self.src = Grabbed(raw)
        self.aimer = Aimer(self.src, self.det, self.trk, self.S, log=self.log, stop_flag=self.stop_event)
        self.aimer.shot_dir = os.path.join(LOG_DIR, "shots")     # ภาพทุกนัด (ก่อนยิง/หลังยิง)
        self.log(f"เชื่อมต่อ: {raw.name}")

    def _file_step(self, k):
        if self.src is not None and hasattr(self.src.src, "next_image"):
            self.src.src.next_image(k)

    # ================================================================ actions
    def _gel_guard(self):
        if self.v_ammo.get() == "gel" and not self.v_gel_ok.get():
            messagebox.showwarning("กระสุนเจล", "ติ๊ก 'ยืนยันใช้เจล' ก่อน (สวมแว่น ห้ามหันเข้าหาคน)")
            return False
        return True

    def engage(self, fire):
        if fire and not self._gel_guard():
            return
        ammo = self.v_ammo.get()
        def job():
            res = self.aimer.engage_all(fire=fire, ammo=ammo)
            self._log_shots(res, ammo)
            n = sum(r["fired"] for r in res)
            msg = f"[engage] เสร็จ: {'ยิง' if fire else 'เล็ง'}สำเร็จ {n if fire else sum(1 for r in res if 'err_yaw' in r)}/{len(res)}"
            if fire and any(r.get("knocked") is not None for r in res):
                msg += f"  ป้ายล้ม {sum(1 for r in res if r.get('knocked'))}/{len(res)}"
            self.log(msg)
        self.run_bg(job, "ENGAGE" if fire else "AIM ONLY")

    def stop(self):
        self.stop_event.set()
        try:
            if self.src is not None:
                self.src.gimbal_speed(0, 0)
        except Exception:
            pass
        self.log("STOP")

    def _log_shots(self, res, ammo):
        path = os.path.join(LOG_DIR, "shots.csv")
        header = ["time", "color", "shape", "fired", "triggers", "burst", "knocked", "dist_cm", "err_yaw_deg", "err_pitch_deg",
                  "iters", "time_s", "gimbal_yaw_abs", "gimbal_pitch_abs", "ammo", "images"]
        if os.path.exists(path):
            with open(path, encoding="utf-8-sig") as f:
                old_header = f.readline().strip().split(",")
            if old_header != header:          # ไฟล์รูปแบบเก่า -> เก็บแยกไว้ ไม่ต่อท้ายคนละคอลัมน์
                os.replace(path, path.replace(".csv", "_old_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"))
        new = not os.path.exists(path)
        with open(path, "a", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            if new:
                w.writerow(header)
            for r in res:
                kn = r.get("knocked")
                w.writerow([datetime.datetime.now().strftime("%H:%M:%S"), r["color"], r["shape"], int(r["fired"]),
                            r.get("triggers", 0), r.get("burst", 1), "" if kn is None else int(kn),
                            round(r.get("dist_cm", 0), 1), round(r.get("err_yaw", 0), 3), round(r.get("err_pitch", 0), 3),
                            r.get("iters"), r.get("time_s"), round(r.get("yaw_abs", 0), 2), round(r.get("pitch_abs", 0), 2),
                            ammo, " ".join(x for x in r.get("images", []) if x)])

    def shoot_clicked(self, fx, fy):
        hit = [d for d in self.dets if d["bbox"][0] - 6 <= fx <= d["bbox"][0] + d["bbox"][2] + 6
               and d["bbox"][1] - 6 <= fy <= d["bbox"][1] + d["bbox"][3] + 6]
        if not hit:
            self.log("คลิกไม่โดนป้ายที่ตรวจเจอ"); return
        if not self._gel_guard():
            return
        d = hit[0]
        w, h = self.frame_size
        ammo = self.v_ammo.get()
        def job():
            G = self.S["gimbal"]
            f = focal_px(w, self.S["camera"]["hfov_deg"])
            dyaw = math.degrees(math.atan2(d["cx"] - w / 2, f))
            dpit = -math.degrees(math.atan2(d["cy"] - h / 2, f))
            self.aimer._clamped_move(dyaw, dpit)
            r = self.aimer.aim_and_fire(d["color"], d["shape"], start_xy="center", ammo=ammo)
            self._log_shots([r], ammo)
        self.run_bg(job, f"SHOOT {d['color']}/{d['shape']}")

    def go_home(self):
        self._sync_settings()
        G = self.S["gimbal"]
        self.run_bg(lambda: self.src.gimbal_moveto(G["home_pitch"], G["home_yaw"]), "ไปมุมล็อค")

    def set_home_here(self):
        a = self.src.gimbal_angles() if self.src else None
        if a is None:
            return
        self.v_g["home_pitch"].set(round(a[0], 1)); self.v_g["home_yaw"].set(round(a[1], 1))
        self.log(f"มุมล็อค = pitch {a[0]:.1f}, yaw {a[1]:.1f}")

    def nudge(self, dp, dy):
        if self.src is None or self.busy():
            return
        G = self.S["gimbal"]
        threading.Thread(target=lambda: self.aimer._clamped_move(dy, dp), daemon=True).start()

    def _key(self, e):
        if self.src is None or self.busy() or isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry, ttk.Spinbox)):
            return
        k = e.keysym.lower()
        self.key_t[k] = time.time()

    def verify_dirs(self):
        self.run_bg(lambda: self.aimer.verify_directions(), "ตรวจทิศ gimbal")

    # ---------------- colour ----------------
    def _load_sliders(self):
        c = self.S["colors"].get(self.v_color.get())
        if not c:
            return
        lo, hi = c["ranges"][0]
        self._slider_lock = True
        self.sl["H lo"].set(lo[0]); self.sl["H hi"].set(hi[0]); self.sl["S lo"].set(lo[1]); self.sl["V lo"].set(lo[2])
        self._slider_lock = False
        rngs = "; ".join(f"H{lo[0]}-{hi[0]} S≥{lo[1]} V≥{lo[2]}" for lo, hi in c["ranges"])
        self.lbl_colorinfo.config(text=f"ช่วงปัจจุบัน: {rngs}   histogram: {'มี' if c.get('hist') else 'ไม่มี'}"
                                       f"   พื้นหลังที่เรียน: {'มี' if self.S.get('bg_hist') else 'ไม่มี'}")

    def _sliders_changed(self):
        if getattr(self, "_slider_lock", False):
            return
        c = self.S["colors"][self.v_color.get()]
        lo, hi = c["ranges"][0]
        lo[0], hi[0], lo[1], lo[2] = self.sl["H lo"].get(), self.sl["H hi"].get(), self.sl["S lo"].get(), self.sl["V lo"].get()
        for rlo, rhi in c["ranges"][1:]:     # ช่วงที่สอง (สีแดงฝั่ง 180) ใช้ S/V เดียวกัน
            rlo[1], rlo[2] = lo[1], lo[2]

    def add_color(self):
        from tkinter import simpledialog
        n = simpledialog.askstring("สีใหม่", "ชื่อสี (ภาษาอังกฤษ เช่น ORANGE)")
        if not n:
            return
        n = n.strip().upper()
        self.S["colors"][n] = {"ranges": [[[0, 100, 60], [10, 255, 255]]], "draw": [200, 200, 200]}
        self.cb_color.config(values=list(self.S["colors"])); self.v_color.set(n)
        self.log(f"เพิ่มสี {n} — ลากเรียนสีจากป้ายจริงได้เลย (ติ๊กเป้าในแท็บยิงหลังรีสตาร์ท)")
        self._load_sliders()

    def clear_hist(self):
        self.S["colors"][self.v_color.get()].pop("hist", None); self._load_sliders()

    def clear_bg(self):
        self.S.pop("bg_hist", None); self._load_sliders(); self.log("ล้างสีพื้นหลังแล้ว")

    def learn_patch(self, x0, y0, x1, y1, bg=False):
        f = self.src.peek() if self.src else None
        if f is None:
            return
        x0, x1 = sorted((int(x0), int(x1))); y0, y1 = sorted((int(y0), int(y1)))
        if x1 - x0 < 3 or y1 - y0 < 3:
            self.log("กรอบเล็กเกินไป"); return
        hsv = self.det.preprocess(f)[y0:y1, x0:x1]
        V = self.S["vision"]
        if bg:
            px = hsv.reshape(-1, 3).astype(np.float32)
            self.S["bg_hist"] = vision.add_hist(self.S.get("bg_hist"), vision.hs_hist(px), 0.5 if self.S.get("bg_hist") else 0)
            self.log(f"เรียนสีพื้นหลังจาก {len(px)} px (สะสมได้หลายครั้ง)")
            self._load_sliders(); return
        name = self.v_color.get()
        try:
            ranges, hist, stats = vision.learn_color_from_patch(hsv, V["glare_v_min"], V["glare_s_max"])
        except ValueError as e:
            self.log(str(e)); return
        c = self.S["colors"][name]
        if self.v_learn_add.get() and c.get("hist"):
            # ขยายช่วงเดิมให้ครอบคลุมทั้งสองชุด (ทำง่ายๆ: เอาขอบนอกสุด)
            if len(ranges) == len(c["ranges"]):
                for (lo, hi), (nlo, nhi) in zip(c["ranges"], ranges):
                    lo[0] = min(lo[0], nlo[0]); hi[0] = max(hi[0], nhi[0])
                    lo[1] = min(lo[1], nlo[1]); lo[2] = min(lo[2], nlo[2])
            else:
                c["ranges"] = ranges
            c["hist"] = vision.add_hist(c["hist"], hist)
        else:
            c["ranges"], c["hist"] = ranges, hist
        mean_bgr = cv2.cvtColor(np.uint8([[np.median(hsv.reshape(-1, 3), 0)]]), cv2.COLOR_HSV2BGR)[0, 0]
        c["draw"] = [int(v) for v in mean_bgr]
        self.log(f"เรียนสี {name}: {c['ranges']}  (H median {stats['h_med']}, n={stats['n']})")
        self._load_sliders()

    def _vision_changed(self):
        for k, v in self.v_vis.items():
            try:
                val = float(v.get())
            except Exception:
                continue
            self.S["vision"][k] = int(val) if k in ("glare_v_min", "glare_s_max", "min_area_px") else val
        self.S["vision"]["clahe"] = bool(self.v_clahe.get())

    def _freeze(self):
        if self.src is not None:
            self.src.freeze = bool(self.v_freeze.get())

    def snapshot(self):
        f = self.src.peek() if self.src else None
        if f is None:
            return
        p = os.path.join(LOG_DIR, "snap_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png")
        cv2.imwrite(p, f); self.log(f"เซฟภาพ -> {p}")

    def reset_roi(self):
        self.S["roi"] = {"x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 1.0}

    def clear_exclude(self):
        self.S["exclude"] = []

    # ---------------- calibration ----------------
    def _hfov_changed(self):
        try:
            self.S["camera"]["hfov_deg"] = float(self.v_hfov.get())
        except Exception:
            pass

    def calib_hfov(self):
        if not self.dets:
            self.log("ไม่เห็นป้ายในภาพ"); return
        d = max(self.dets, key=lambda d: d["area"])
        W, H = ST.SIGN_SIZES_CM[d["shape"]]
        x, y, bw, bh = d["bbox"]
        Z = float(self.v_known.get())
        f_est = (0.35 * bw / W + 0.65 * bh / H) * Z
        w = self.frame_size[0]
        hfov = 2 * math.degrees(math.atan((w / 2) / f_est))
        self.v_hfov.set(round(hfov, 2)); self._hfov_changed()
        self.log(f"HFOV = {hfov:.2f}° (จาก {d['color']}/{d['shape']} กว้าง {bw}px สูง {bh}px ที่ {Z} ซม.)")

    def z_from_sign(self):
        if not self.dets:
            self.log("ไม่เห็นป้าย"); return
        d = max(self.dets, key=lambda d: d["area"])
        self.v_calz.set(round(d["dist_cm"], 1))

    def calib_auto(self):
        ammo = self.v_ammo.get()
        if not self._gel_guard():
            return
        z = None if self.v_zsign.get() else float(self.v_calz.get())
        n = int(self.v_nshot.get())
        self.v_freeze.set(False); self._freeze()
        def job():
            out = CAL.auto_calibrate(self.src, self.model, z, ammo, shots=n, out_dir=CALIB_DIR, log=self.log,
                                     stop=self.stop_event, detector=self.det)
            self._show_last_calib()
            self._update_cal_label(f"ระยะ {out.get('z') or 0:.0f} ซม.: ได้ {out['ok']} นัด")
        self.run_bg(job, f"AUTO CALIB Z={z if z else 'จากป้าย'} x{n}")

    def _show_last_calib(self):
        try:
            files = sorted((os.path.join(CALIB_DIR, f) for f in os.listdir(CALIB_DIR) if f.endswith(".jpg")),
                           key=os.path.getmtime)
            if files:
                self._show_img = (cv2.imread(files[-1]), time.time() + 2.5)
        except Exception:
            pass

    def calib_shot_click(self):
        if not self._gel_guard():
            return
        self.v_freeze.set(False); self._freeze()
        ammo = self.v_ammo.get()
        self.ir_pending = float(self.v_calz.get())
        self.v_mouse.set("ir")
        def job():
            time.sleep(0.6)
            pre = self.src.recent(6)
            self.src.start_record()
            self.src.fire(ammo)
            time.sleep(1.5)
            post = self.src.stop_record()
            if not post or len(pre) < 2:
                self.log("ไม่มีภาพ"); return
            r = CAL.find_flash(pre, post, min_peak=5, min_snr=0,      # เอาเฟรมที่เปลี่ยนมากสุด (ใน ROI)
                               roi=self.det.roi_mask(*pre[0].shape[:2]))
            f = post[r["idx"]][1] if r else post[len(post) // 3][1]
            self.src.freeze = True
            self.src.latest = f
            self.q.put(("freeze_on", None))
            self.log("ภาพหยุดที่เฟรมที่แสงแฟลชเด่นสุดแล้ว — คลิกตรงจุดที่โดน")
        self.run_bg(job, "CALIB SHOT (click)")

    def add_ruler_sample(self):
        ammo = self.v_ammo.get()
        z, dx, dy = float(self.v_rz.get()), float(self.v_rdx.get()), float(self.v_rdy.get())
        self.model.add_sample_cm(z, dx, dy, ammo)
        self.log(f"เพิ่มตัวอย่าง [{ammo}] Z={z} ซม. รอยอยู่ ขวา {dx:+.1f} ลง {dy:+.1f} ซม. จากจุดที่ crosshair เล็ง")
        self._update_cal_label()

    def crosshair_shot(self):
        if not self._gel_guard():
            return
        self._sync_settings()
        ammo = self.v_ammo.get()
        if not self.dets:
            self.log("ไม่เห็นป้าย — หันกล้องไปที่ป้าย/กระดาษที่มีสี"); return
        d = max(self.dets, key=lambda d: d["area"])
        self.v_rz.set(round(d["dist_cm"], 1))
        def job():
            S = self.S
            backup = dict(S["hit_model"][ammo]); aimy = S["aim"]["aim_y_ratio"]
            conf = S["aim"].get("confirm_hit_" + ammo)
            S["hit_model"][ammo] = dict(backup, Ax=0.0, Bx=0.0, Ay=0.0, By=0.0, Cy=0.0)
            S["aim"]["aim_y_ratio"] = 0.5; S["aim"]["confirm_hit_" + ammo] = False
            try:
                r = self.aimer.aim_and_fire(d["color"], d["shape"], start_xy="center", fire=True, ammo=ammo)
            finally:
                S["hit_model"][ammo] = backup; S["aim"]["aim_y_ratio"] = aimy; S["aim"]["confirm_hit_" + ammo] = conf
            self.log(f"ยิงแล้ว (เล็งกลางป้ายด้วย crosshair) ระยะ {r.get('dist_cm', 0):.0f} ซม. — "
                     "ถ้ากระสุนไม่ออกกดใหม่ได้ วัดรอยแล้วใส่ ขวา/ลง ซม. แล้วกด เพิ่ม")
        self.run_bg(job, "CROSSHAIR SHOT")

    def calib_test(self):
        ammo = self.v_ammo.get()
        if not self._gel_guard():
            return
        self.v_freeze.set(False); self._freeze()
        def job():
            txt = CAL.accuracy_test(self.aimer, self.src, trials=6, ammo=ammo, compare=True, out_dir=CALIB_DIR,
                                    log=self.log, stop=self.stop_event)
            self.log(txt)
            self._show_last_calib()
        self.run_bg(job, "ACCURACY TEST")

    def open_calib_dir(self):
        os.makedirs(CALIB_DIR, exist_ok=True)
        try:
            os.startfile(CALIB_DIR)          # Windows
        except Exception:
            self.log(f"ผลอยู่ที่: {CALIB_DIR}")

    def add_ir_click(self, fx, fy):
        z = self.ir_pending if self.ir_pending else float(self.v_calz.get())
        w, h = self.frame_size
        self.model.add_sample(w, h, z, fx, fy, self.v_ammo.get())
        self.log(f"เพิ่มตัวอย่าง Z={z} จุด=({fx:.0f},{fy:.0f}) [{self.v_ammo.get()}]")
        self.ir_pending = None
        self.v_freeze.set(False); self._freeze()
        self._update_cal_label()

    def calib_fit(self):
        w, h = self.frame_size
        txt = CAL.fit_report(self.model, self.v_ammo.get(), w, h, out_dir=CALIB_DIR)
        self.log(txt)
        self.log(f"รายงาน+กราฟ -> {CALIB_DIR}  (กด 💾 บันทึกค่า เพื่อใช้ครั้งหน้า)")
        self._update_cal_label(txt.splitlines()[1] if "\n" in txt else txt)

    def calib_undo(self):
        s = self.model.params(self.v_ammo.get())["samples"]
        if s:
            s.pop()
        self._update_cal_label()

    def calib_clear(self):
        if messagebox.askyesno("", f"ล้างตัวอย่าง {self.v_ammo.get()} ทั้งหมด?"):
            self.model.params(self.v_ammo.get())["samples"].clear()
            self._update_cal_label()

    def trim(self, dx, dy):
        p = self.model.params(self.v_ammo.get())
        p["Ax"] += dx; p["Ay"] += dy
        self._update_cal_label()

    def _update_cal_label(self, extra=""):
        self.q.put(("cal", extra))          # คำนวณข้อความใน main thread (_tick)

    def _cal_text(self, extra):
        ammo = self.v_ammo.get()
        p = self.S["hit_model"][ammo]
        txt = (f"[{ammo}] ตัวอย่าง {len(p['samples'])}  Ax={p['Ax']:.1f} Bx={p['Bx']:.0f} "
               f"Ay={p['Ay']:.1f} By={p['By']:.0f} Cy={p['Cy']:.3f}")
        return txt + ("\n" + extra if extra else "")

    # ================================================================ mouse
    def _to_frame(self, ex, ey):
        return ex / self.disp_scale, ey / self.disp_scale

    def _m_down(self, e):
        self.drag = (e.x, e.y, e.x, e.y)
        mode = self.v_mouse.get()
        fx, fy = self._to_frame(e.x, e.y)
        if mode == "shoot":
            self.drag = None
            self.shoot_clicked(fx, fy)
        elif mode == "ir":
            self.drag = None
            self.add_ir_click(fx, fy)

    def _m_move(self, e):
        if self.drag:
            self.drag = (self.drag[0], self.drag[1], e.x, e.y)

    def _m_up(self, e):
        if not self.drag:
            return
        x0, y0, _, _ = self.drag
        self.drag = None
        w, h = self.frame_size
        (fx0, fy0), (fx1, fy1) = self._to_frame(x0, y0), self._to_frame(e.x, e.y)
        if abs(fx1 - fx0) < 3 or abs(fy1 - fy0) < 3:
            return
        n = [min(fx0, fx1) / w, min(fy0, fy1) / h, max(fx0, fx1) / w, max(fy0, fy1) / h]
        n = [min(1.0, max(0.0, v)) for v in n]
        mode = self.v_mouse.get()
        if mode == "roi":
            self.S["roi"] = {"x0": n[0], "y0": n[1], "x1": n[2], "y1": n[3]}
            self.log(f"ROI = {[round(v, 3) for v in n]}")
        elif mode == "exclude":
            self.S["exclude"].append(n); self.log(f"เพิ่มโซนตัดทิ้ง {[round(v, 3) for v in n]}")
        elif mode in ("learn", "learn_bg"):
            self.learn_patch(fx0, fy0, fx1, fy1, bg=(mode == "learn_bg"))

    def _m_right(self, e):
        fx, fy = self._to_frame(e.x, e.y)
        w, h = self.frame_size
        nx, ny = fx / w, fy / h
        for z in list(self.S["exclude"]):
            if min(z[0], z[2]) <= nx <= max(z[0], z[2]) and min(z[1], z[3]) <= ny <= max(z[1], z[3]):
                self.S["exclude"].remove(z); self.log("ลบโซนตัดทิ้ง"); return

    # ================================================================ loops
    def _vision_loop(self):
        """thread ตรวจจับต่อเนื่องบนเฟรมล่าสุด (ให้ภาพใน dashboard ลื่น)"""
        last_t = 0
        while True:
            src = self.src
            if src is None or src.latest is None or src.t == last_t:
                time.sleep(0.02); continue
            last_t = src.t
            f = src.latest
            try:
                dets = self.det.detect(f, keep_masks=self._showmask)
                self.dets = dets
                self._last_frame = f
            except Exception as e:
                self.log(f"vision error {e!r}")
                time.sleep(0.2)

    def _manual_keys(self):
        now = time.time()
        if self.src is None or self.busy():
            return
        G = self.S["gimbal"]
        yaw = (1 if now - self.key_t.get("d", 0) < 0.25 else 0) - (1 if now - self.key_t.get("a", 0) < 0.25 else 0)
        pit = (1 if now - self.key_t.get("w", 0) < 0.25 else 0) - (1 if now - self.key_t.get("s", 0) < 0.25 else 0)
        sp = G["manual_speed"]
        state = (yaw, pit)
        if state != getattr(self, "_key_state", (0, 0)):
            try:
                self.src.gimbal_speed(pit * sp * G["pitch_sign"], yaw * sp * G["yaw_sign"])
            except Exception:
                pass
            self._key_state = state
        elif state != (0, 0) and self.src.name == "Simulator":
            self.src.gimbal_speed(pit * sp * G["pitch_sign"], yaw * sp * G["yaw_sign"])

    def _tick(self):
        while True:
            try:
                m = self.q.get_nowait()
            except queue.Empty:
                break
            if isinstance(m, tuple) and m[0] == "cal":
                self.lbl_cal.config(text=self._cal_text(m[1]))
            elif isinstance(m, tuple) and m[0] == "freeze_on":
                self.v_freeze.set(True)
            elif isinstance(m, tuple) and m[0] == "reload":
                self._reload_ui()
            elif isinstance(m, tuple) and m[0] == "session":
                self.lbl_ss.config(text=m[1])
            else:
                self.logbox.insert(tk.END, str(m) + "\n"); self.logbox.see(tk.END)
        self._manual_keys()
        self._showmask = bool(self.v_showmask.get())
        f = getattr(self, "_last_frame", None)
        show = getattr(self, "_show_img", None)
        showing = show is not None and show[0] is not None and time.time() < show[1]
        if showing:
            f = show[0]
        if f is not None:
            h, w = f.shape[:2]
            self.frame_size = (w, h)
            if self.v_showmask.get() and self.v_color.get() in self.det.last_masks:
                disp = cv2.cvtColor(self.det.last_masks[self.v_color.get()], cv2.COLOR_GRAY2BGR)
            else:
                disp = f.copy()
            dets = [] if showing else list(self.dets)
            hit = None
            if dets:
                big = max(dets, key=lambda d: d["area"])
                hit = self.model.hit_px(w, h, big["dist_cm"], self.v_ammo.get())
            vision.draw(disp, dets, self.S, hit_px=hit)
            st = self.aimer.state if (self.aimer and self.busy()) else {}
            if st.get("aim") and st.get("hit"):
                a, b = st["aim"], st["hit"]
                cv2.line(disp, (int(b[0]), int(b[1])), (int(a[0]), int(a[1])), (0, 0, 255), 2)
                cv2.circle(disp, (int(a[0]), int(a[1])), 5, (255, 255, 255), 2)
            self.disp_scale = min(self.CW / w, self.CH / h)
            disp = cv2.resize(disp, (int(w * self.disp_scale), int(h * self.disp_scale)))
            img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)))
            self.cv.delete("all")
            self.cv.create_image(0, 0, anchor="nw", image=img)
            self.cv._img = img
            if self.drag:
                x0, y0, x1, y1 = self.drag
                col = {"roi": "yellow", "exclude": "gray", "learn": "cyan", "learn_bg": "orange"}.get(self.v_mouse.get(), "white")
                self.cv.create_rectangle(x0, y0, x1, y1, outline=col, width=2, dash=(4, 2))
            ang = self.src.gimbal_angles() if self.src else None
            angs = f"gimbal pitch {ang[0]:+.1f} yaw {ang[1]:+.1f}" if ang else "gimbal -"
            self.lbl_g.config(text=angs + f"   sign yaw={self.S['gimbal']['yaw_sign']} pitch={self.S['gimbal']['pitch_sign']}")
            desc = ", ".join(f"{d['color'][0]}{d['shape'][0]}@{d['dist_cm']:.0f}" for d in sorted(dets, key=lambda d: d["cx"]))
            self.status.set(f"{self.src.name if self.src else ''}  {w}x{h}  เจอ {len(dets)} ป้าย: {desc}   |  {angs}"
                            + ("   [กำลังทำงาน]" if self.busy() else ""))
        self.root.after(50, self._tick)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.mainloop()

    def _close(self):
        self.stop_event.set()
        try:
            if self.src:
                self.src.close()
        finally:
            self.root.destroy()


if __name__ == "__main__":
    Dashboard().run()
