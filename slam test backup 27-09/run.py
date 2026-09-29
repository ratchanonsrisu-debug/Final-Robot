"""
run.py — จุดเริ่มโปรแกรม (GUI / headless simulation / ประเมินผลย้อนหลัง)

    python run.py                 เปิด GUI
    python run.py --sim --headless --seed 3
    python run.py --sim --headless --w 7 --h 7 --sx 3 --sy 3 --heading E
    python run.py --eval output/run_xxx/map.json --gt my_gt.json   คำนวณผลย้อนหลัง
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
    explorer = Explorer(bot, m, logger, echo=print)
    vis, tot = explorer.explore()
    print(f"สำรวจเสร็จ: {vis}/{tot} ช่อง  เริ่ม {m.start}  จบ {m.end}")

    res, text = export_all(m, logger.traj, out_dir, world)
    print()
    print(text)
    logger.close()
    bot.close()
    return 0 if vis == tot else 1


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
        description="RoboMaster EP — Grid SLAM Explorer (Class Work 8)")
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
    p.add_argument("--no-shooter", action="store_true", help="ปิดระบบการมองหาและยิงเป้าหมาย (เดินสำรวจอย่างเดียว)")
    args = p.parse_args()
    if args.no_shooter:
        C.ENABLE_SHOOTER = False
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
