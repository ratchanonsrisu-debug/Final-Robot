"""
gui.py — หน้าจอควบคุม (Tkinter)

โหมดคลิกบนแผนที่:
  • ไปยังช่องที่คลิก   : หุ่นวางแผน A* แล้วเดินไป (ใช้แผนที่ที่สำรวจ/โหลดมา)
  • ตั้งตำแหน่งหุ่น     : บอกโปรแกรมว่าตอนนี้หุ่นวางอยู่ช่องไหน (หลังโหลดแผนที่)
  • แก้แผนที่           : คลิกเส้นขอบเพื่อสลับ กำแพง/โล่ง/ไม่รู้
  • แก้ Ground Truth     : คลิกเส้นขอบเพื่อวาดแผนที่จริงของอาจารย์ (ใช้คำนวณ Accuracy)
"""
import datetime
import os
import queue
import threading
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

import config as C
from grid_map import GridMap, random_maze, DIRS, DX, DY, WALL, OPEN, UNKNOWN

HERE = os.path.dirname(os.path.abspath(__file__))
CANVAS = 620
PAD = 30


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RoboMaster EP — Grid SLAM Explorer (Class Work 8)")
        global CANVAS
        CANVAS = max(380, min(620, self.root.winfo_screenheight() - 300))
        self.q = queue.Queue()
        #: บันทึกข้อความ log() ทุกบรรทัดลงไฟล์จริง ๆ ทันที (ไม่ใช่แค่ในกล่อง log บนจอ ซึ่ง
        #: หายไปเมื่อปิดโปรแกรม) เปิดตั้งแต่ต้นโปรแกรม ก่อนกด Apply ด้วยซ้ำ เพื่อให้ตรวจย้อนหลังได้
        #: เสมอว่าตอนเชื่อมต่อ/คาลิเบรต gimbal ได้ค่าอะไร แม้ยังไม่ได้เริ่มสำรวจ
        self._log_lock = threading.Lock()
        os.makedirs(os.path.join(HERE, C.OUTPUT_DIR), exist_ok=True)
        self._session_log_path = os.path.join(
            HERE, C.OUTPUT_DIR, "gui_session_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
        self._session_log_file = open(self._session_log_path, "w", encoding="utf-8")
        self.robot = None
        self.explorer = None
        self.logger = None
        self.worker = None
        self.stop_event = threading.Event()
        self.gt = None
        self.out_dir = None
        #: str or None: "sim"/"real" ตามหุ่นที่ self.robot เชื่อมต่ออยู่จริงตอนนี้ (ไม่ใช่
        #: ค่าที่เลือกไว้ใน v_mode) ใช้กันสับสนเวลาสลับปุ่มวิทยุแล้วลืมกด Apply ซ้ำ
        self.robot_kind = None
        self.m = GridMap(C.GRID_W, C.GRID_H, C.CELL_M)
        self.last_eval = None
        self._build()
        self.log(C.build_info())   # เช็คได้ทันทีว่ารันโค้ดชุดที่แก้ล่าสุดจริงไหม
        self._new_sim_world()
        self.redraw()
        self.root.after(100, self._poll)

    # ================================================================ UI
    def _build(self):
        left = ttk.Frame(self.root, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(self.root, padding=6)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ---- settings ----
        f = ttk.LabelFrame(left, text="1) ตั้งค่าสนาม / หุ่น", padding=6)
        f.pack(fill=tk.X)
        self.v_w = tk.IntVar(value=C.GRID_W)
        self.v_h = tk.IntVar(value=C.GRID_H)
        self.v_cell = tk.DoubleVar(value=C.CELL_M)
        self.v_sx = tk.IntVar(value=C.START_X)
        self.v_sy = tk.IntVar(value=C.START_Y)
        self.v_hd = tk.StringVar(value=C.START_HEADING)
        self.v_mode = tk.StringVar(value="sim")
        self.v_seed = tk.StringVar(value="")
        self.v_move = tk.StringVar(value=C.MOVE_MODE)
        self.v_color = tk.StringVar(value="red")
        self.v_shape = tk.StringVar(value="rectangle")
        self.v_shooter_enabled = tk.BooleanVar(value=getattr(C, "ENABLE_SHOOTER", True))
        rows = [("กว้าง (ช่อง)", ttk.Spinbox(f, from_=2, to=C.MAX_GRID, textvariable=self.v_w, width=5)),
                ("สูง (ช่อง)", ttk.Spinbox(f, from_=2, to=C.MAX_GRID, textvariable=self.v_h, width=5)),
                ("ขนาดช่อง (m)", ttk.Entry(f, textvariable=self.v_cell, width=7)),
                ("เริ่ม x", ttk.Spinbox(f, from_=0, to=C.MAX_GRID - 1, textvariable=self.v_sx, width=5)),
                ("เริ่ม y", ttk.Spinbox(f, from_=0, to=C.MAX_GRID - 1, textvariable=self.v_sy, width=5)),
                ("หน้าหุ่นหันทิศ", ttk.Combobox(f, values=DIRS, textvariable=self.v_hd, width=4, state="readonly")),
                ("การเดิน", ttk.Combobox(f, values=["strafe", "rotate"], textvariable=self.v_move, width=7, state="readonly")),
                ("sim seed", ttk.Entry(f, textvariable=self.v_seed, width=7)),
                ("สีเป้าหมาย", ttk.Combobox(f, values=["red", "blue", "yellow", "green"], textvariable=self.v_color, width=7, state="readonly")),
                ("ทรงเป้าหมาย", ttk.Combobox(f, values=["rectangle", "circle", "square"], textvariable=self.v_shape, width=7, state="readonly"))]
        for i, (lab, w) in enumerate(rows):          # 2 คู่ต่อแถว ให้หน้าจอไม่สูงเกิน
            r, c = divmod(i, 2)
            ttk.Label(f, text=lab).grid(row=r, column=2 * c, sticky="w", padx=(0 if c == 0 else 8, 2))
            w.grid(row=r, column=2 * c + 1, sticky="w", pady=1)
        nr = (len(rows) + 1) // 2
        mf = ttk.Frame(f)
        mf.grid(row=nr, column=0, columnspan=4, sticky="w", pady=3)
        ttk.Radiobutton(mf, text="จำลอง (Sim)", value="sim", variable=self.v_mode).pack(side=tk.LEFT)
        ttk.Radiobutton(mf, text="หุ่นจริง", value="real", variable=self.v_mode).pack(side=tk.LEFT, padx=(2, 6))
        ttk.Checkbutton(mf, text="🎯 เปิดยิงเป้า", variable=self.v_shooter_enabled).pack(side=tk.LEFT)
        ttk.Button(f, text="Apply / เชื่อมต่อหุ่น", command=self.apply_settings).grid(
            row=nr + 1, column=0, columnspan=4, sticky="ew")

        # ---- mission ----
        f2 = ttk.LabelFrame(left, text="2) ภารกิจ", padding=6)
        f2.pack(fill=tk.X, pady=4)
        self.btn_explore = ttk.Button(f2, text="▶ เริ่มสำรวจ (Explore)", command=self.start_explore)
        self.btn_explore.grid(row=0, column=0, sticky="ew")
        ttk.Button(f2, text="■ หยุด (Stop)", command=self.stop).grid(row=0, column=1, sticky="ew")
        ttk.Button(f2, text="สแกนตรงนี้", command=self.scan_here).grid(row=1, column=0, sticky="ew")
        ttk.Button(f2, text="ฉันอยู่ไหน?", command=self.where_am_i).grid(row=1, column=1, sticky="ew")
        ttk.Button(f2, text="กลับจุดเริ่มต้น", command=self.go_home).grid(row=2, column=0, sticky="ew")
        ttk.Button(f2, text="Calibrate ToF", command=self.calibrate).grid(row=2, column=1, sticky="ew")
        ttk.Button(f2, text="🚀 Run 2 (วิ่งยิงเป้าด่วน)", command=self.start_run2).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(2, 2))
        ttk.Button(f2, text="🎯 เทสการ Detect Target", command=self.open_detection_test).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(0, 0))
        f2.columnconfigure(0, weight=1); f2.columnconfigure(1, weight=1)

        # ---- click mode ----
        f3 = ttk.LabelFrame(left, text="3) คลิกบนแผนที่เพื่อ…", padding=6)
        f3.pack(fill=tk.X, pady=4)
        self.v_click = tk.StringVar(value="goto")
        for val, txt in (("goto", "ไปยังช่องที่คลิก"), ("pose", "ตั้งตำแหน่งหุ่น (ช่อง + ทิศด้านบน)"),
                         ("edit", "แก้แผนที่ (คลิกเส้นขอบ)"), ("gt", "วาด Ground Truth (คลิกเส้นขอบ)")):
            ttk.Radiobutton(f3, text=txt, value=val, variable=self.v_click,
                            command=self.redraw).pack(anchor="w")

        # ---- files ----
        f4 = ttk.LabelFrame(left, text="4) ไฟล์ / ประเมินผล", padding=6)
        f4.pack(fill=tk.X, pady=4)
        g = ttk.Frame(f4); g.pack(fill=tk.X)
        ttk.Button(g, text="บันทึกแผนที่", command=self.save_map).grid(row=0, column=0, sticky="ew")
        ttk.Button(g, text="โหลดแผนที่", command=self.load_map).grid(row=0, column=1, sticky="ew")
        ttk.Button(g, text="บันทึก GT", command=self.save_gt).grid(row=1, column=0, sticky="ew")
        ttk.Button(g, text="โหลด GT", command=self.load_gt).grid(row=1, column=1, sticky="ew")
        ttk.Button(g, text="ล้าง GT (เหลือขอบนอก)", command=self.clear_gt).grid(row=2, column=0, columnspan=2, sticky="ew")
        g.columnconfigure(0, weight=1); g.columnconfigure(1, weight=1)
        ttk.Button(f4, text="📊 ประเมินผล + Export ไฟล์ส่งงาน", command=self.evaluate_export).pack(fill=tk.X, pady=3)
        self.v_showgt = tk.BooleanVar(value=False)
        ttk.Checkbutton(f4, text="แสดง GT ซ้อน (ขอบที่ผิดเป็นสีแดง)", variable=self.v_showgt,
                        command=self.redraw).pack(anchor="w")

        # ---- canvas + log ----
        self.status = tk.StringVar(value="พร้อม")
        ttk.Label(right, textvariable=self.status, font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.cv = tk.Canvas(right, width=CANVAS, height=CANVAS, bg="white", highlightthickness=1,
                            highlightbackground="#bbb")
        self.cv.pack()
        self.cv.bind("<Button-1>", self.on_click)
        self.logbox = ScrolledText(right, height=9, font=("Consolas", 9))
        self.logbox.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

    # ================================================================ helpers
    def log(self, s):
        with self._log_lock:
            try:
                self._session_log_file.write(s + "\n")
                self._session_log_file.flush()
            except Exception:
                pass
        self.q.put(("log", s))

    def _poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "log":
                    self.logbox.insert(tk.END, val + "\n"); self.logbox.see(tk.END)
                elif kind == "redraw":
                    pass
                elif kind == "done":
                    self.btn_explore.state(["!disabled"])
                elif kind == "click_goto":
                    self.v_click.set("goto")
                elif kind == "info":
                    messagebox.showinfo("ผลลัพธ์", val)
        except queue.Empty:
            pass
        self.redraw()
        self.root.after(150, self._poll)

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def run_bg(self, fn, name):
        if self.busy():
            messagebox.showwarning("กำลังทำงาน", "หุ่นกำลังทำงานอยู่ กด Stop ก่อน")
            return
        if self.robot is None:
            messagebox.showwarning("ยังไม่เชื่อมต่อ", "กด Apply / เชื่อมต่อหุ่น ก่อน")
            return
        if self.robot_kind != self.v_mode.get():
            messagebox.showwarning(
                "โหมดไม่ตรงกับหุ่นที่เชื่อมต่ออยู่",
                f"ตอนนี้ต่ออยู่กับหุ่น{'จำลอง (Sim)' if self.robot_kind == 'sim' else 'จริง'} "
                f"แต่เลือกโหมด{'จำลอง (Sim)' if self.v_mode.get() == 'sim' else 'หุ่นจริง'} ไว้\n\n"
                "กด 'Apply / เชื่อมต่อหุ่น' ซ้ำอีกครั้งก่อน เพื่อสลับไปหุ่นที่เลือกจริง ๆ")
            return
        self.stop_event.clear()
        self._ensure_logger()
        from explorer import Explorer
        self.explorer = Explorer(self.robot, self.m, self.logger, stop_event=self.stop_event, echo=self.log)
        def wrap():
            try:
                fn()
            except Exception as e:
                self.log("ERROR: " + repr(e))
                self.log(traceback.format_exc())
            finally:
                self.q.put(("done", None))
        self.log(f"--- {name} ---")
        self.worker = threading.Thread(target=wrap, daemon=True)
        self.worker.start()

    def _ensure_logger(self):
        if self.logger is None:
            from explorer import RunLogger
            self.out_dir = os.path.join(HERE, C.OUTPUT_DIR,
                                        "run_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.logger = RunLogger(self.out_dir, echo=self.log)
            self.log(f"log -> {self.out_dir}")

    def _read_settings(self):
        w, h = int(self.v_w.get()), int(self.v_h.get())
        if not (2 <= w <= C.MAX_GRID and 2 <= h <= C.MAX_GRID):
            raise ValueError(f"ขนาดกริดต้องอยู่ระหว่าง 2..{C.MAX_GRID}")
        sx, sy = int(self.v_sx.get()), int(self.v_sy.get())
        if not (0 <= sx < w and 0 <= sy < h):
            raise ValueError("จุดเริ่มต้นอยู่นอกสนาม")
        return w, h, float(self.v_cell.get()), sx, sy, self.v_hd.get(), self.v_color.get(), self.v_shape.get()

    def _new_sim_world(self):
        seed = self.v_seed.get().strip()
        self.sim_world = random_maze(self.m.w, self.m.h, seed=int(seed) if seed else None)

    # ================================================================ actions
    def apply_settings(self):
        if self.busy():
            messagebox.showwarning("กำลังทำงาน", "กด Stop ก่อน"); return
        try:
            w, h, cell, sx, sy, hd, color, shape = self._read_settings()
        except Exception as e:
            messagebox.showerror("ตั้งค่าผิด", str(e)); return
        C.MOVE_MODE = self.v_move.get()
        C.ENABLE_SHOOTER = self.v_shooter_enabled.get()
        if self.robot:
            self.robot.close(); self.robot = None
        self.robot_kind = None    # ยังไม่มีหุ่นต่ออยู่ จนกว่าจะเชื่อมต่อสำเร็จด้านล่าง
        if self.logger:
            self.logger.close(); self.logger = None
        self.m = GridMap(w, h, cell)
        if self.gt is not None and (self.gt.w, self.gt.h) != (w, h):
            self.gt = None
        self.last_eval = None
        if self.v_mode.get() == "sim":
            from robots import SimRobot
            if self.gt is not None and not getattr(self, "gt_is_random", False):
                world = self.gt
                self.log("Sim ใช้ Ground Truth ที่วาด/โหลดไว้เป็นสนามจำลอง")
            else:
                self._new_sim_world(); world = self.sim_world
                self.log("Sim ใช้เขาวงกตสุ่ม (ติ๊ก 'แสดง GT' เพื่อดูเฉลย)")
            seed = self.v_seed.get().strip()
            self.robot = SimRobot(world, sx, sy, hd, cell, seed=int(seed) if seed else None)
            self.robot_kind = "sim"
            if world is self.sim_world:
                self.gt = world
                self.gt_is_random = True
            self.log(f"[SIM] Sim robot พร้อม: {w}x{h} ช่อง, ช่องละ {cell} m, เริ่ม ({sx},{sy}) หัน {hd}")
        else:
            self.status.set("กำลังเชื่อมต่อ RoboMaster…")
            def conn():
                try:
                    from robots import RealRobot
                    r = RealRobot(sx, sy, hd, cell, log=self.log, color=color, shape=shape, enable_shooter=self.v_shooter_enabled.get())
                    self.robot = r
                    self.robot_kind = "real"
                    self.log(f"[REAL] เชื่อมต่อหุ่นจริงสำเร็จ: {w}x{h}, เริ่ม ({sx},{sy}) หัน {hd} | Shooter: {'เปิด' if C.ENABLE_SHOOTER else 'ปิด'}")
                except Exception as e:
                    self.robot = None
                    self.robot_kind = None
                    self.log("เชื่อมต่อไม่สำเร็จ: " + repr(e))
                    self.log("ตรวจ: ต่อ Wi-Fi ของหุ่น (AP mode), pip install robomaster, Python 3.6-3.8")
            threading.Thread(target=conn, daemon=True).start()

    def start_explore(self):
        if self.busy():
            messagebox.showwarning("กำลังทำงาน", "กด Stop ก่อน"); return
        # เริ่มสำรวจใหม่ทุกครั้งด้วยแผนที่เปล่าเสมอ - ไม่งั้นความรู้ (รวมถึงกำแพงที่เคย
        # ตรวจพลาด) จากความพยายามครั้งก่อนจะค้างอยู่ใน self.m ข้ามการกด "เริ่มสำรวจ" แต่ละ
        # ครั้ง (self.m ถูกสร้างใหม่เฉพาะตอนกด Apply เท่านั้น) ทำให้ครั้งถัดไปแก้กำแพงผิดที่
        # เดิมไม่ได้ง่ายๆ (log-odds ต้องเจอ "โล่ง" ยืนยันซ้ำหลายครั้งกว่าจะลบกำแพงเก่าออก)
        self.m = GridMap(self.m.w, self.m.h, self.m.cell_m)
        self.last_eval = None

        def job():
            vis, tot = self.explorer.explore()
            self.log(f"สำรวจเสร็จ: {vis}/{tot} ช่อง  เริ่ม {self.m.start}  จบ {self.m.end}")
            self._export(auto=True)
            if not self.stop_event.is_set():
                self.q.put(("click_goto", None))
                self.log("จอดรอคำสั่ง: คลิกช่องบนแผนที่เพื่อสั่งหุ่นไป (แผนที่บันทึกแล้ว)")
        self.run_bg(job, "EXPLORE")

    def start_run2(self):
        if self.busy():
            messagebox.showwarning("กำลังทำงาน", "กด Stop ก่อน"); return
        def job():
            shot_cnt, tot_cnt = self.explorer.run2_shoot_targets()
            self.log(f"ภารกิจ Run 2 เสร็จสิ้น: ยิงสำเร็จ {shot_cnt}/{tot_cnt} เป้าหมาย")
            self._export(auto=True)
        self.run_bg(job, "RUN 2")

    def stop(self):
        self.stop_event.set()
        self.log("STOP requested (หุ่นจะหยุดหลังจบก้าวปัจจุบัน)")

    def scan_here(self):
        self.run_bg(lambda: (self.explorer._sense_here(), None), "SCAN")

    def where_am_i(self):
        def job():
            obs, cands = self.explorer.where_am_i()
            seen = "".join("W" if obs[d] == WALL else "." for d in range(4))
            best = [c for m_, c in cands if m_ == 0]
            self.log(f"เห็นกำแพง N,E,S,W = {seen}  -> ช่องที่ลายตรงกับแผนที่: {best if best else cands[:3]}")
            self.log(f"ตำแหน่งที่หุ่นเชื่อว่าอยู่: {self.robot.cell}  (ถ้าไม่ตรง ใช้โหมด 'ตั้งตำแหน่งหุ่น')")
        self.run_bg(job, "RELOCALIZE")

    def go_home(self):
        if not self.m.start:
            messagebox.showinfo("", "ยังไม่มีจุดเริ่มต้น (ยังไม่ได้สำรวจ)"); return
        tgt = tuple(self.m.start[:2])
        self.run_bg(lambda: self.explorer.return_home(), f"GO HOME {tgt}")

    def calibrate(self):
        def job():
            detailed = self.robot.scan_all_detailed()
            for d in range(4):
                samples_txt = ", ".join(f"{v:.0f}" for v in detailed[d]["samples"])
                lo, hi = min(detailed[d]["samples"]), max(detailed[d]["samples"])
                self.log(f"[CALIBRATE] {DIRS[d]}: [{samples_txt}]mm "
                         f"-> median={detailed[d]['median']:.0f}mm (ต่ำสุด-สูงสุด {lo:.0f}-{hi:.0f}mm)")
            tof = {d: detailed[d]["median"] for d in range(4)}
            near = {d: v for d, v in tof.items() if v < 600}
            # กำแพงอ้างอิงคิดจาก "คู่กำแพงตรงข้าม" เท่านั้น: (ซ้าย+ขวา)/2 ไม่ขึ้นกับว่าหุ่นอยู่
            # กลางช่องเป๊ะไหม (เดิมเฉลี่ยทุกด้านที่ใกล้ -> หุ่นเยื้อง 4cm ก็ได้ EXPECT 226 แทน 184
            # แล้ว recenter พาหุ่นไปจอดเยื้องตามค่าผิดนั้นทุกช่อง)
            pairs = [(near[d] + near[d + 2]) / 2 for d in (0, 1) if d in near and d + 2 in near]
            if pairs:
                e = sum(pairs) / len(pairs)
                if abs(e - C.WALL_EXPECT_MM) > 60:
                    self.log(f"[CALIBRATE] !! ค่าใหม่ {e:.0f}mm ต่างจากเดิม {C.WALL_EXPECT_MM}mm มาก "
                             f"- เช็คว่ากำแพงตั้งตรง/หุ่นหันตรงกริด")
                C.WALL_EXPECT_MM = round(e)
                C.WALL_THRESHOLD_MM = round(e + C.WALL_THRESHOLD_FRAC * self.m.cell_m * 1000)
                self.log(f"ตั้ง WALL_EXPECT_MM={C.WALL_EXPECT_MM}, WALL_THRESHOLD_MM={C.WALL_THRESHOLD_MM} "
                         f"(mm จริง, จากคู่กำแพงตรงข้าม {len(pairs)} คู่ — ใช้ในรอบนี้)")
            else:
                self.log(f"[CALIBRATE] ไม่มีคู่กำแพงตรงข้าม (ซ้าย+ขวา หรือ หน้า+หลัง) - คงค่าเดิม "
                         f"WALL_EXPECT_MM={C.WALL_EXPECT_MM} (วางหุ่นในช่องที่มีกำแพงสองฝั่งตรงข้าม)")
            # Sharp: เทียบกับ ToF ฝั่งเดียวกัน + ตั้งกำแพงอ้างอิง (ชดเชยการเยื้องด้วยคู่ ToF)
            disagree, sides = 0, 0
            for d in range(4):
                sh = self.robot.read_sharp(d)
                if sh is None:
                    continue
                sides += 1
                if (tof[d] < C.WALL_THRESHOLD_MM) != (sh[0] < C.SHARP_WALL_MM):
                    disagree += 1
                    self.log(f"[CALIBRATE] !! {DIRS[d]}: ToF {tof[d]:.0f}mm กับ Sharp {sh[0]:.0f}mm "
                             f"ตัดสินกำแพงไม่ตรงกัน")
                side = "R" if (d - self.robot.body_heading) % 4 == 1 else "L"
                od = (d + 2) % 4
                if sh[0] < C.SHARP_WALL_MM and d in near and od in near:
                    lean = (tof[od] - tof[d]) / 2.0      # + = หุ่นเยื้องเข้าหาฝั่ง d
                    ex = round(sh[0] + lean)
                    setattr(C, f"SHARP_{side}_EXPECT_MM", ex)
                    self.log(f"[CALIBRATE] Sharp {side} ({DIRS[d]}) = {sh[0]:.0f}mm, หุ่นเยื้อง {lean:+.0f}mm "
                             f"-> ตั้ง SHARP_{side}_EXPECT_MM={ex}")
                else:
                    self.log(f"[CALIBRATE] Sharp {side} ({DIRS[d]}) = {sh[0]:.0f}mm "
                             f"- คงค่า SHARP_{side}_EXPECT_MM={getattr(C, f'SHARP_{side}_EXPECT_MM')}")
            if sides == 2 and disagree == 2:
                self.log(f"[CALIBRATE] !! ToF ซ้าย/ขวาขัดกับ Sharp ทั้งสองฝั่ง = gimbal น่าจะหันสลับด้าน "
                         f"-> ลองตั้ง GIMBAL_RIGHT_YAW = {-C.GIMBAL_RIGHT_YAW} ใน config.py")
        self.run_bg(job, "CALIBRATE")

    def open_detection_test(self):
        import subprocess, sys
        color = self.v_color.get()
        shape = self.v_shape.get()
        mode = self.v_mode.get()
        cmd = [sys.executable, os.path.join(HERE, "test_detection.py"), "--color", color, "--shape", shape]
        if mode == "sim":
            cmd.append("--sim")
        self.log(f"[Vision Test] เปิดหน้าต่างทดสอบการตรวจจับเป้าหมาย ({color} {shape})...")
        try:
            subprocess.Popen(cmd)
        except Exception as e:
            self.log(f"[Vision Test] ไม่สามารถเปิดระบบทดสอบได้: {e}")

    # ---------------------------------------------------------------- click
    def _geom(self):
        s = (CANVAS - 2 * PAD) / max(self.m.w, self.m.h)
        ox = (CANVAS - s * self.m.w) / 2
        oy = (CANVAS - s * self.m.h) / 2
        return s, ox, oy

    def _px(self, gx, gy):
        s, ox, oy = self._geom()
        return ox + gx * s, CANVAS - (oy + gy * s)

    def on_click(self, ev):
        s, ox, oy = self._geom()
        gx = (ev.x - ox) / s
        gy = (CANVAS - ev.y - oy) / s
        mode = self.v_click.get()
        cx, cy = int(gx // 1), int(gy // 1)
        if mode in ("goto", "pose"):
            if not self.m.in_bounds(cx, cy):
                return
            if mode == "goto":
                if self.robot is None:
                    messagebox.showwarning("", "กด Apply / เชื่อมต่อหุ่น ก่อน"); return
                self.run_bg(lambda: self.explorer.go_to((cx, cy)) and self.explorer.park(),
                            f"GOTO ({cx},{cy})")
            else:
                if self.busy():
                    return
                if self.robot:
                    self.robot.set_pose(cx, cy, self.v_hd.get())
                self.v_sx.set(cx); self.v_sy.set(cy)
                self.log(f"ตั้งตำแหน่งหุ่น = ({cx},{cy}) หัน {self.v_hd.get()}")
            return
        # edge edit
        fx, fy = gx - cx, gy - cy
        cands = [(fy, 2), (1 - fy, 0), (fx, 3), (1 - fx, 1)]   # distance to S,N,W,E edge
        dist, d = min(cands)
        if dist > 0.25:
            return
        x, y = cx, cy
        if not self.m.in_bounds(x, y):   # คลิกที่ขอบนอกด้านบน/ขวา
            x, y = min(max(cx, 0), self.m.w - 1), min(max(cy, 0), self.m.h - 1)
            d = 0 if cy >= self.m.h else 1 if cx >= self.m.w else 2 if cy < 0 else 3
        target = self.m if mode == "edit" else self._gt_for_edit()
        cur = target.state(x, y, d)
        if mode == "edit":      # WALL -> OPEN -> UNKNOWN -> WALL
            nxt = {WALL: OPEN, OPEN: UNKNOWN, UNKNOWN: WALL}[cur]
        else:                   # GT: WALL <-> OPEN
            nxt = OPEN if cur == WALL else WALL
        target.set_state(x, y, d, nxt)
        if target.is_boundary(x, y, d):
            target.boundary_walls = False
        self.redraw()

    def _gt_for_edit(self):
        if self.gt is None or (self.gt.w, self.gt.h) != (self.m.w, self.m.h):
            self.clear_gt()
        return self.gt

    # ---------------------------------------------------------------- files
    def save_map(self):
        p = filedialog.asksaveasfilename(defaultextension=".json", initialdir=HERE,
                                         initialfile="my_map.json", filetypes=[("Map JSON", "*.json")])
        if p:
            self.m.save(p); self.log(f"บันทึกแผนที่ -> {p}")

    def load_map(self):
        if self.busy():
            return
        p = filedialog.askopenfilename(initialdir=HERE, filetypes=[("Map JSON", "*.json")])
        if not p:
            return
        m = GridMap.load(p)
        self.m = m
        self.v_w.set(m.w); self.v_h.set(m.h); self.v_cell.set(m.cell_m)
        self.log(f"โหลดแผนที่ {m.w}x{m.h} จาก {p}")
        if self.robot is not None and self.explorer is not None:
            self.explorer.m = m
        self.log("วางหุ่นที่ช่องใดก็ได้ -> เลือกโหมด 'ตั้งตำแหน่งหุ่น' แล้วคลิกช่องนั้น -> "
                 "เลือก 'ไปยังช่องที่คลิก' แล้วคลิกเป้าหมาย")
        if self.robot is None:
            self.log("(ยังไม่ได้เชื่อมต่อหุ่น: ตั้งค่าเริ่ม x,y/ทิศ แล้วกด Apply — แผนที่ที่โหลดจะถูกเก็บไว้)")
            self._pending_map = m

    def save_gt(self):
        if self.gt is None:
            messagebox.showinfo("", "ยังไม่มี Ground Truth"); return
        p = filedialog.asksaveasfilename(defaultextension=".json", initialdir=HERE,
                                         initialfile="ground_truth.json", filetypes=[("JSON", "*.json")])
        if p:
            self.gt.save(p); self.log(f"บันทึก GT -> {p}")

    def load_gt(self):
        p = filedialog.askopenfilename(initialdir=HERE, filetypes=[("JSON", "*.json")])
        if p:
            g = GridMap.load(p)
            if (g.w, g.h) != (self.m.w, self.m.h):
                messagebox.showerror("", f"GT ขนาด {g.w}x{g.h} ไม่ตรงกับแผนที่ {self.m.w}x{self.m.h}"); return
            self.gt = g; self.gt_is_random = False
            self.v_showgt.set(True); self.log(f"โหลด GT จาก {p}")

    def clear_gt(self):
        g = GridMap(self.m.w, self.m.h, self.m.cell_m, boundary_walls=True)
        for x in range(g.w):
            for y in range(g.h):
                for d in range(4):
                    if not g.is_boundary(x, y, d):
                        g.set_state(x, y, d, OPEN)
        self.gt = g
        self.gt_is_random = False
        self.v_showgt.set(True)
        self.log("GT ใหม่: มีแค่ขอบนอก — เลือกโหมด 'วาด Ground Truth' แล้วคลิกเส้นขอบที่เป็นกำแพงจริง")

    def evaluate_export(self):
        self._export(auto=False)

    def _export(self, auto):
        from evaluation import export_all
        try:
            self._ensure_logger()
            gt = self.gt if (self.gt is not None and (self.gt.w, self.gt.h) == (self.m.w, self.m.h)) else None
            res, text = export_all(self.m, self.logger.traj, self.out_dir, gt)
            self.last_eval = res
            self.log(text)
            self.log(f"ไฟล์ทั้งหมดอยู่ที่: {self.out_dir}")
            if not auto:
                msg = f"Coverage {res['coverage_pct']}%"
                if "map_accuracy_pct" in res:
                    msg += f"\nMap Accuracy {res['map_accuracy_pct']}%"
                self.q.put(("info", msg + f"\n\nบันทึกที่ {self.out_dir}"))
        except Exception as e:
            self.log("export error: " + repr(e))

    # ================================================================ draw
    def redraw(self):
        if hasattr(self, "_pending_map") and self.robot is not None:
            self.m = self._pending_map
            del self._pending_map
        cv = self.cv
        cv.delete("all")
        m = self.m
        s, ox, oy = self._geom()
        mode = self.v_click.get()
        show = self.gt if mode == "gt" else m
        cell_ok = None
        if self.last_eval and "cell_ok" in self.last_eval and mode != "gt":
            if len(self.last_eval["cell_ok"]) == m.w:
                cell_ok = self.last_eval["cell_ok"]
        for x in range(m.w):
            for y in range(m.h):
                x0, y0 = self._px(x, y + 1); x1, y1 = self._px(x + 1, y)
                fc = "#e8f5e9" if (m.visited[x][y] and mode != "gt") else "#fafafa"
                if cell_ok is not None:
                    fc = "#c8e6c9" if cell_ok[x][y] else "#ffcdd2"
                cv.create_rectangle(x0, y0, x1, y1, fill=fc, outline="#e0e0e0")
                cv.create_text(x0 + 4, y1 - 4, text=f"{x},{y}", anchor="sw", fill="#aaa", font=("Arial", 8))
        # planned path
        if self.explorer and self.explorer.plan and mode != "gt":
            pts = []
            for (x, y) in self.explorer.plan:
                pts += list(self._px(x + 0.5, y + 0.5))
            if len(pts) >= 4:
                cv.create_line(*pts, fill="#42a5f5", width=4, dash=(6, 4), arrow=tk.LAST)
        # trajectory
        if self.logger and self.logger.traj and mode != "gt":
            pts = []
            for p in self.logger.traj[-600:]:
                pts += list(self._px(p[1] / m.cell_m, p[2] / m.cell_m))
            if len(pts) >= 4:
                cv.create_line(*pts, fill="#1565c0", width=2)
        # edges
        if show is not None:
            for x in range(m.w):
                for y in range(m.h):
                    for d in range(4):
                        if d in (0, 1) or (d == 2 and y == 0) or (d == 3 and x == 0):
                            self._draw_edge(show, x, y, d, mode)
        # start / robot
        if m.start:
            px, py = self._px(m.start[0] + 0.5, m.start[1] + 0.5)
            cv.create_oval(px - 16, py - 16, px + 16, py + 16, outline="#2e7d32", width=3)
            cv.create_text(px, py - 24, text="START", fill="#2e7d32", font=("Arial", 9, "bold"))
        if m.end:
            px, py = self._px(m.end[0] + 0.5, m.end[1] + 0.5)
            cv.create_text(px, py + 24, text="END", fill="#c62828", font=("Arial", 9, "bold"))
        if self.robot is not None:
            ex, ey = self.robot.est_xy()
            px, py = self._px(ex / m.cell_m, ey / m.cell_m)
            h = self.robot.heading
            r = s * 0.28
            fx, fy = DX[h], -DY[h]
            lx, ly = -fy, fx
            pts = [px + fx * r, py + fy * r,
                   px - fx * r * 0.7 + lx * r * 0.7, py - fy * r * 0.7 + ly * r * 0.7,
                   px - fx * r * 0.7 - lx * r * 0.7, py - fy * r * 0.7 - ly * r * 0.7]
            cv.create_polygon(*pts, fill="#ff7043", outline="#bf360c", width=2)
            if self.explorer and self.explorer.last_tof:
                for d, v in self.explorer.last_tof.items():
                    if not isinstance(d, int):
                        continue
                    L = min(v, 2500) / 1000.0 / m.cell_m * s
                    cv.create_line(px, py, px + DX[d] * L, py - DY[d] * L, fill="#ffab91", width=1)
        vis, tot = m.counts()
        acc = ""
        if self.last_eval and "map_accuracy_pct" in self.last_eval:
            acc = f"   Accuracy {self.last_eval['map_accuracy_pct']}%"
        if self.robot:
            kind_txt = "🖥 SIM" if self.robot_kind == "sim" else "🤖 หุ่นจริง"
            pos = f"   [{kind_txt}] อยู่ช่อง {self.robot.cell} หัน {DIRS[self.robot.heading]}"
            if self.robot_kind != self.v_mode.get():
                pos += "  ⚠ โหมดที่เลือกไม่ตรง กด Apply ซ้ำ!"
        else:
            pos = "   (ยังไม่เชื่อมต่อหุ่น)"
        state = "  [กำลังทำงาน]" if self.busy() else ""
        mode_txt = {"goto": "คลิก = ไปยังช่อง", "pose": "คลิก = ตั้งตำแหน่งหุ่น",
                    "edit": "คลิกเส้น = แก้แผนที่", "gt": "คลิกเส้น = วาด Ground Truth"}[mode]
        self.status.set(f"{m.w}x{m.h}  สำรวจ {vis}/{tot} ({100*vis/tot:.0f}%){acc}{pos}{state}   | {mode_txt}")

    def _draw_edge(self, g, x, y, d, mode):
        if d == 0:
            a, b = self._px(x, y + 1), self._px(x + 1, y + 1)
        elif d == 2:
            a, b = self._px(x, y), self._px(x + 1, y)
        elif d == 1:
            a, b = self._px(x + 1, y), self._px(x + 1, y + 1)
        else:
            a, b = self._px(x, y), self._px(x, y + 1)
        st = g.state(x, y, d)
        wrong = (mode != "gt" and self.v_showgt.get() and self.gt is not None
                 and (self.gt.w, self.gt.h) == (g.w, g.h) and self.gt.state(x, y, d) != st)
        if st == WALL:
            self.cv.create_line(*a, *b, fill="#d32f2f" if wrong else "#111", width=7, capstyle=tk.ROUND)
        elif st == UNKNOWN:
            self.cv.create_line(*a, *b, fill="#ff9800", width=2, dash=(4, 4))
        elif wrong:
            self.cv.create_line(*a, *b, fill="#d32f2f", width=3, dash=(2, 3))
        # เงา GT
        if mode != "gt" and self.v_showgt.get() and self.gt is not None and \
                (self.gt.w, self.gt.h) == (g.w, g.h) and self.gt.state(x, y, d) == WALL and st != WALL:
            self.cv.create_line(*a, *b, fill="#90a4ae", width=3)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.mainloop()

    def _close(self):
        self.stop_event.set()
        try:
            if self.robot:
                self.robot.close()
            if self.logger:
                self.logger.close()
        finally:
            try:
                self._session_log_file.close()
            except Exception:
                pass
            self.root.destroy()


if __name__ == "__main__":
    App().run()
