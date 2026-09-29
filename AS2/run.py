"""
run.py — จุดเริ่มโปรแกรม (GUI / headless simulation / ประเมินผลย้อนหลัง)

    python run.py                 เปิด GUI
    python run.py --sim --headless --seed 3
    python run.py --sim --headless --w 7 --h 7 --sx 3 --sy 3 --heading E
    python run.py --eval output/run_xxx/map.json --gt my_gt.json   คำนวณผลย้อนหลัง
    python run.py --sim --headless --seed 3 --shoot [--round2]      จำลองสำรวจ+ยิงป้าย (+รอบสอง)
"""
import argparse
import datetime
import os

import config as C

HERE = os.path.dirname(os.path.abspath(__file__))


def run_headless(args):
    from grid_map import GridMap, random_maze
    from robots import SimRobot
    from explorer import Explorer, RunLogger
    from evaluation import export_all

    w = args.w or C.GRID_W
    h = args.h or C.GRID_H
    sx = args.sx if args.sx is not None else C.START_X
    sy = args.sy if args.sy is not None else C.START_Y
    heading = args.heading or C.START_HEADING
    cell = args.cell or C.CELL_M

    world = random_maze(w, h, seed=args.seed)
    m = GridMap(w, h, cell)
    bot = SimRobot(world, sx, sy, heading, cell, seed=args.seed)

    out_dir = os.path.join(HERE, C.OUTPUT_DIR,
                           "run_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    logger = RunLogger(out_dir, echo=print)
    hunter = None
    if args.shoot:
        from shooter import SimShooterIO, TargetHunter, make_settings, random_sim_targets
        truth = random_sim_targets(world, C.SIM_TARGETS, avoid={(sx, sy)}, seed=args.seed)
        print("ป้ายจริงในสนามจำลอง: " + ", ".join(
            f"{t['color']}/{t['shape']}@({t['x']:.2f},{t['y']:.2f})" for t in truth))
        hunter = TargetHunter(SimShooterIO(bot, world, truth, seed=args.seed or 1), make_settings(sim=True),
                              log=print, sim=True)
        if args.targets is not None:
            from shooter import parse_spec
            hunter.spec = parse_spec(args.targets)
        hunter.set_out_dir(out_dir)
    explorer = Explorer(bot, m, logger, echo=print, hunter=hunter)
    vis, tot = explorer.explore()
    print(f"สำรวจเสร็จ: {vis}/{tot} ช่อง  เริ่ม {m.start}  จบ {m.end}")
    if hunter:
        _sim_report(hunter, "รอบ 1")

    res, text = export_all(m, logger.traj, out_dir, world,
                           targets=hunter.export() if hunter else None)
    print()
    print(text)
    if hunter and args.round2:
        from shooter import run_round2
        for t in truth:                       # กรรมการตั้งป้ายใหม่
            t.pop("knocked", None)
        n1 = len(hunter.io.shots)
        hunter.load(hunter.export())
        shot, total = run_round2(explorer, hunter)
        print(f"รอบ 2: ยิง {shot}/{total}")
        _sim_report(hunter, "รอบ 2", since=n1)
        export_all(m, logger.traj, out_dir, world, targets=hunter.export(), tag="_round2")
    logger.close()
    bot.close()
    return 0 if vis == tot else 1


def _sim_report(hunter, name, since=0):
    """ตรวจกติกาจากความจริงของสนามจำลอง: ยิงผิดเป้า / ทแยง / เกิน 2 ช่อง / ช่องเดียวกัน"""
    from shooter import inline_dir
    shots = hunter.io.shots[since:]
    bad = []
    for s in shots:
        if s["hit"] is None:
            continue
        if not hunter.spec_ok(*s["hit"]):
            bad.append(f"ยิงผิดเป้า {s['hit']}")
        il = inline_dir(s["from"], s["target_cell"])
        if il is None or il[1] > C.SHOOT_MAX_TILES:
            bad.append(f"ยิง {s['hit']} จาก {s['from']} ไปช่อง {s['target_cell']} ผิดกติกา")
    need = [t for t in hunter.io.targets if hunter.spec_ok(t["color"], t["shape"])]
    hit = {id(t) for t in need if t.get("knocked")}
    print(f"[sim {name}] สั่งยิง {len(shots)} ครั้ง โดน {sum(1 for s in shots if s['hit'])} ; "
          f"เป้าที่กำหนด {len(need)} ใบ ล้ม {len(hit)} ; ผิดกติกา {len(bad)} {bad}")


def run_eval(args):
    from grid_map import GridMap
    from evaluation import evaluate, save_comparison_png

    m = GridMap.load(args.eval)
    gt = GridMap.load(args.gt) if args.gt else None
    res = evaluate(m, gt)
    for k, v in res.items():
        if k != "cell_ok":
            print(f"{k:20s}: {v}")
    if gt is not None:
        out_path = os.path.join(os.path.dirname(os.path.abspath(args.eval)), "comparison.png")
        save_comparison_png(m, gt, res, out_path)
        print(f"บันทึกภาพเปรียบเทียบ -> {out_path}")
    return 0


def main():
    p = argparse.ArgumentParser(
        description="RoboMaster EP — Assignment 2: SLAM explore + shoot color targets")
    p.add_argument("--sim", action="store_true", help="ทดสอบตรรกะจำลอง (ไม่ต่อหุ่นจริง)")
    p.add_argument("--headless", action="store_true", help="ไม่เปิดหน้าต่าง GUI (ใช้คู่กับ --sim)")
    p.add_argument("--seed", type=int, default=None, help="sim seed (ไม่ใส่ = สุ่ม)")
    p.add_argument("--w", type=int, default=None, help="ความกว้างกริด (ช่อง)")
    p.add_argument("--h", type=int, default=None, help="ความสูงกริด (ช่อง)")
    p.add_argument("--sx", type=int, default=None, help="ตำแหน่งเริ่ม x")
    p.add_argument("--sy", type=int, default=None, help="ตำแหน่งเริ่ม y")
    p.add_argument("--heading", default=None, choices=["N", "E", "S", "W"], help="ทิศเริ่มต้น")
    p.add_argument("--cell", type=float, default=None, help="ขนาดช่อง (เมตร)")
    p.add_argument("--eval", metavar="MAP_JSON", default=None,
                   help="คำนวณ Accuracy/Coverage ย้อนหลังจากไฟล์ map.json ที่ export ไว้แล้ว")
    p.add_argument("--gt", metavar="GT_JSON", default=None,
                   help="ไฟล์ Ground Truth JSON คู่กับ --eval")
    p.add_argument("--shoot", action="store_true", help="(sim headless) วางป้ายสุ่ม + หา/ยิงป้ายระหว่างสำรวจ")
    p.add_argument("--round2", action="store_true", help="(ใช้คู่ --shoot) สำรวจเสร็จแล้วรันรอบสองต่อ")
    p.add_argument("--targets", default=None, help='ป้ายที่ต้องยิง เช่น "RED:CIRCLE, BLUE" (ว่าง = ทุกป้าย)')
    args = p.parse_args()
    print(C.build_info())

    if args.eval:
        return run_eval(args)
    if args.sim and args.headless:
        return run_headless(args)

    from gui import App
    App().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
