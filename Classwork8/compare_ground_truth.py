# -*- coding: utf-8 -*-
"""Class Work 8 - เปรียบเทียบแผนที่ที่หุ่นสร้างได้กับ Ground Truth Map (deliverable ข้อ 4)

คำนวณตรงตามสูตรที่โจทย์กำหนด:
    Map Accuracy = (จำนวน Cell ที่ทายถูก / จำนวน Cell ทั้งหมด) x 100
    Coverage     = (จำนวน Cell ที่สำรวจแล้ว / จำนวน Cell ทั้งหมด) x 100

อินพุต
------
1. Map CSV ที่ ``SLAM.py`` export ออกมาหลังวิ่งจริง (``slam8_map_<timestamp>.csv``)
   ทุกช่องมีค่าเป็น 0 (สำรวจแล้ว/หุ่นเคยไปยืน), 1 (ไม่เคยไปแต่กำแพงล้อมรอบครบ
   ทุกด้าน = ไปไม่ถึง/ตัน), หรือ 0.5 (ไม่เคยไปแต่ยังไม่ตัน = ไม่รู้)
2. Ground Truth CSV ที่อาจารย์ให้ในวันทดสอบ - รูปแบบเดียวกัน (แถวบนสุด = y
   มากสุด ตรงกับที่ ``Maze.render``/``export_run`` เขียนไว้) ค่าที่ใช้ได้คือ
   0 (ช่องเดินได้/โล่ง) หรือ 1 (ช่องตัน/ผ่านไม่ได้) เท่านั้น (ไม่มี 0.5 เพราะ
   Ground Truth คือความจริงทั้งหมด ไม่มีคำว่า "ไม่รู้")

หมายเหตุสำคัญ: ถ้า Ground Truth ที่ได้จริงในวันสอบมาในรูปแบบอื่น (เช่น สลับแถว/
สลับความหมาย 0-1 หรือเป็นขนาดช่องย่อยกว่าตาราง MAZE_W x MAZE_H) ให้แก้ที่ฟังก์ชัน
``load_grid`` หรือ ``_cell_matches`` เท่านั้น ไม่ต้องแตะส่วนคำนวณสูตร

วิธีใช้
------
    python compare_ground_truth.py --map slam8_map_20260928_101500.csv --gt ground_truth.csv
"""

import argparse
import csv
import sys


def load_grid(path):
    """list: อ่าน CSV เป็นตาราง float สองมิติ (แถว 0 = แถวบนสุดของไฟล์)"""
    grid = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if not row:
                continue
            grid.append([float(v) for v in row])
    return grid


def _cell_matches(predicted, truth):
    """bool: ช่องนี้ "ทายถูก" ไหม

    Ground Truth ตีความตรง ๆ: 0 = โล่ง/เดินได้, 1 = ตัน/ผ่านไม่ได้
    Map ของเราตีความ: 0 = สำรวจแล้ว (โล่งแน่นอน เพราะหุ่นเคยไปยืนจริง) จึงถือ
    ว่า "ทาย = โล่ง"; 1 = ล้อมกำแพงครบ (ไม่เคยไปถึง) จึงถือว่า "ทาย = ตัน";
    0.5 = ไม่เคยไปและยังไม่ตัน (สถานะ "ไม่รู้") - นับเป็น "ทายผิด" เสมอตามหลัก
    ระมัดระวัง (ไม่รู้ = เดาไม่ได้ว่าตรงหรือไม่ ไม่ควรนับเป็นถูกฟรี ๆ)
    """
    if predicted == 0.5:
        return False
    predicted_wall = predicted >= 1.0
    truth_wall = truth >= 1.0
    return predicted_wall == truth_wall


def compare(map_grid, gt_grid):
    """dict: คำนวณ Map Accuracy% และ Coverage% ตามสูตรของโจทย์เป๊ะ ๆ"""
    h = min(len(map_grid), len(gt_grid))
    if len(map_grid) != len(gt_grid):
        print("[WARN] จำนวนแถวไม่เท่ากัน (map={0}, gt={1}) จะเทียบแค่ {2} แถวแรก"
              .format(len(map_grid), len(gt_grid), h))

    total = 0
    correct = 0
    explored = 0
    mismatches = []
    for y in range(h):
        row_map = map_grid[y]
        row_gt = gt_grid[y]
        w = min(len(row_map), len(row_gt))
        if len(row_map) != len(row_gt):
            print("[WARN] แถว {0}: จำนวนคอลัมน์ไม่เท่ากัน (map={1}, gt={2}) "
                  "จะเทียบแค่ {3} คอลัมน์แรก"
                  .format(y, len(row_map), len(row_gt), w))
        for x in range(w):
            total += 1
            predicted = row_map[x]
            truth = row_gt[x]
            if predicted == 0:
                explored += 1
            if _cell_matches(predicted, truth):
                correct += 1
            else:
                mismatches.append((x, y, predicted, truth))

    if total == 0:
        raise ValueError("ไม่มี Cell ให้เทียบเลย ตรวจไฟล์ CSV ทั้งสองไฟล์")

    return {
        "total_cells": total,
        "map_accuracy_pct": 100.0 * correct / total,
        "coverage_pct": 100.0 * explored / total,
        "correct_cells": correct,
        "explored_cells": explored,
        "mismatches": mismatches,
    }


def main():
    parser = argparse.ArgumentParser(
        description="เปรียบเทียบแผนที่ที่หุ่นสร้างได้กับ Ground Truth Map (Class Work 8 ข้อ 4)")
    parser.add_argument("--map", required=True, help="path ไปยัง slam8_map_<ts>.csv ที่ export ออกมา")
    parser.add_argument("--gt", required=True, help="path ไปยัง Ground Truth Map CSV")
    args = parser.parse_args()

    map_grid = load_grid(args.map)
    gt_grid = load_grid(args.gt)
    result = compare(map_grid, gt_grid)

    print("=" * 50)
    print("  ผลการเปรียบเทียบกับ Ground Truth Map")
    print("=" * 50)
    print("  Cell ทั้งหมด        : {0}".format(result["total_cells"]))
    print("  Cell ที่สำรวจแล้ว    : {0}".format(result["explored_cells"]))
    print("  Cell ที่ทายถูก       : {0}".format(result["correct_cells"]))
    print("  Map Accuracy         : {0:.2f}%".format(result["map_accuracy_pct"]))
    print("  Coverage             : {0:.2f}%".format(result["coverage_pct"]))

    if result["mismatches"]:
        print("\n  Cell ที่ทายผิด ({0} ช่อง):".format(len(result["mismatches"])))
        for x, y, predicted, truth in result["mismatches"]:
            print("    ({0},{1}) ทาย={2} จริง={3}".format(x, y, predicted, truth))

    return 0


if __name__ == "__main__":
    sys.exit(main())
