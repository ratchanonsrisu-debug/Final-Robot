"""
evaluation.py — วัดความถูกต้อง + วาดภาพแผนที่ / trajectory + export ไฟล์ส่งงาน

Map Accuracy = (จำนวน Cell ที่ทายถูก / จำนวน Cell ทั้งหมด) x 100
    Cell "ทายถูก" = กำแพงทั้ง 4 ด้านของช่องนั้น ตรงกับ Ground Truth (UNKNOWN = ผิด)
Coverage     = (จำนวน Cell ที่สำรวจแล้ว / จำนวน Cell ทั้งหมด) x 100
"""
import csv
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from grid_map import DIRS, DX, DY, WALL, OPEN, UNKNOWN, STATE_NAME


def evaluate(m, gt):
    tot = m.w * m.h
    vis = sum(m.visited[x][y] for x in range(m.w) for y in range(m.h))
    res = {"total_cells": tot, "visited_cells": vis,
           "coverage_pct": round(100.0 * vis / tot, 2)}
    if gt is None:
        return res
    if (gt.w, gt.h) != (m.w, m.h):
        raise ValueError("ground truth size != map size")
    correct = 0
    cell_ok = [[False] * m.h for _ in range(m.w)]
    e_ok = e_tot = 0
    seen_edges = set()
    for x in range(m.w):
        for y in range(m.h):
            ok = True
            for d in range(4):
                a, b = m.state(x, y, d), gt.state(x, y, d)
                if a != b:
                    ok = False
                key = m._edge_ref(x, y, d)
                if key not in seen_edges:
                    seen_edges.add(key)
                    e_tot += 1
                    e_ok += (a == b)
            cell_ok[x][y] = ok
            correct += ok
    gt_walls = sum(1 for k in seen_edges if _edge_state(gt, k) == WALL)
    det_walls = sum(1 for k in seen_edges if _edge_state(m, k) == WALL)
    tp = sum(1 for k in seen_edges if _edge_state(gt, k) == WALL and _edge_state(m, k) == WALL)
    res.update({"correct_cells": correct,
                "map_accuracy_pct": round(100.0 * correct / tot, 2),
                "edge_accuracy_pct": round(100.0 * e_ok / e_tot, 2),
                "gt_walls": gt_walls, "detected_walls": det_walls, "walls_matched": tp,
                "cell_ok": cell_ok})
    return res


def _edge_state(m, key):
    k, a, b = key
    lo = m.Hz[a][b] if k == "H" else m.V[a][b]
    return m._st(lo)


# ------------------------------------------------------------------ drawing
def draw_map(ax, m, title="", cell_ok=None, show_unknown=True, gt=None):
    c = m.cell_m
    for x in range(m.w):
        for y in range(m.h):
            fc = "#e8f5e9" if m.visited[x][y] else "#f4f4f4"
            if cell_ok is not None:
                fc = "#c8e6c9" if cell_ok[x][y] else "#ffcdd2"
            ax.add_patch(Rectangle((x * c, y * c), c, c, fc=fc, ec="#dddddd", lw=0.6))
            ax.text((x + 0.5) * c, (y + 0.5) * c - 0.22 * c, f"({x},{y})",
                    ha="center", va="center", fontsize=7, color="#999999")
    def seg(k, a, b):
        if k == "V":
            return [a * c, a * c], [b * c, (b + 1) * c]
        return [a * c, (a + 1) * c], [b * c, b * c]
    edges = [("V", a, b) for a in range(m.w + 1) for b in range(m.h)] + \
            [("H", a, b) for a in range(m.w) for b in range(m.h + 1)]
    for key in edges:
        st = _edge_state(m, key)
        xs, ys = seg(*key)
        wrong = gt is not None and _edge_state(gt, key) != st
        if st == WALL:
            ax.plot(xs, ys, color="#d32f2f" if wrong else "#111111", lw=5, solid_capstyle="round")
        elif st == UNKNOWN and show_unknown:
            ax.plot(xs, ys, color="#ff9800", lw=1.5, ls=(0, (3, 3)))
        elif wrong:
            ax.plot(xs, ys, color="#d32f2f", lw=2, ls=(0, (1, 2)))
    ax.set_xlim(-0.05, m.w * c + 0.05)
    ax.set_ylim(-0.05, m.h * c + 0.05)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    if title:
        ax.set_title(title, fontsize=11)


def _mark_start_end(ax, m):
    c = m.cell_m
    if m.start:
        ax.plot((m.start[0] + 0.5) * c, (m.start[1] + 0.5) * c, "o", ms=16, mfc="none",
                mec="#2e7d32", mew=3, zorder=6)
        ax.text((m.start[0] + 0.5) * c, (m.start[1] + 0.5) * c + 0.2 * c, "START",
                ha="center", color="#2e7d32", fontsize=9, weight="bold", zorder=6)
    if m.end:
        ax.plot((m.end[0] + 0.5) * c, (m.end[1] + 0.5) * c, "X", ms=14, color="#c62828", zorder=6)
        ax.text((m.end[0] + 0.5) * c, (m.end[1] + 0.5) * c - 0.33 * c, "END",
                ha="center", color="#c62828", fontsize=9, weight="bold", zorder=6)


def save_map_png(m, path):
    fig, ax = plt.subplots(figsize=(1.4 * m.w + 2, 1.4 * m.h + 1.5))
    vis, tot = m.counts()
    draw_map(ax, m, f"Explored Map {m.w}x{m.h}  (cell {m.cell_m:.2f} m)  visited {vis}/{tot}")
    _mark_start_end(ax, m)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def save_trajectory_png(m, traj, path):
    fig, ax = plt.subplots(figsize=(1.4 * m.w + 2, 1.4 * m.h + 1.5))
    draw_map(ax, m, "Robot Trajectory", show_unknown=False)
    ex = [p[1] for p in traj]; ey = [p[2] for p in traj]
    ax.plot(ex, ey, "-", color="#1565c0", lw=2, label="estimated pose (grid + ToF)", zorder=4)
    ax.plot(ex, ey, ".", color="#1565c0", ms=4, zorder=4)
    ox = [p[3] for p in traj if p[3] is not None]; oy = [p[4] for p in traj if p[4] is not None]
    if ox:
        ax.plot(ox, oy, "--", color="#8e24aa", lw=1.2, alpha=0.8, label="odometry", zorder=3)
    # ลูกศรทิศทางการเดิน
    moves = [p for p in traj if p[7] in ("move", "start")]
    for a, b in zip(moves, moves[1:]):
        ax.annotate("", xy=(b[1], b[2]), xytext=(a[1], a[2]),
                    arrowprops=dict(arrowstyle="->", color="#1565c0", lw=1.2, alpha=0.6), zorder=4)
    _mark_start_end(ax, m)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=2)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def save_comparison_png(m, gt, res, path):
    fig, axs = plt.subplots(1, 2, figsize=(2 * (1.3 * m.w + 1.5), 1.3 * m.h + 2))
    draw_map(axs[0], gt, "Ground Truth", show_unknown=False)
    draw_map(axs[1], m, f"Explored (green=cell correct, red=wrong)\n"
             f"Accuracy {res['map_accuracy_pct']}%   Coverage {res['coverage_pct']}%",
             cell_ok=res["cell_ok"], gt=gt)
    for a in axs:
        _mark_start_end(a, m)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


# ------------------------------------------------------------------ export
def export_all(m, traj, out_dir, gt=None, extra_lines=()):
    os.makedirs(out_dir, exist_ok=True)
    m.save(os.path.join(out_dir, "map.json"))
    with open(os.path.join(out_dir, "walls_data.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["type", "i", "j", "state", "logodds"])
        w.writerows(m.walls_rows())
    with open(os.path.join(out_dir, "map_cells.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["x", "y", "visited", "N", "E", "S", "W"])
        for y in range(m.h):
            for x in range(m.w):
                w.writerow([x, y, int(m.visited[x][y])] +
                           [STATE_NAME[m.state(x, y, d)] for d in range(4)])
    with open(os.path.join(out_dir, "trajectory.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "est_x_m", "est_y_m", "odom_x_m", "odom_y_m", "cell_x", "cell_y", "event"])
        for p in traj:
            w.writerow([p[0], round(p[1], 3), round(p[2], 3),
                        "" if p[3] is None else round(p[3], 3),
                        "" if p[4] is None else round(p[4], 3), p[5], p[6], p[7]])
    save_map_png(m, os.path.join(out_dir, "map.png"))
    save_trajectory_png(m, traj, os.path.join(out_dir, "trajectory.png"))
    res = evaluate(m, gt)
    if gt is not None:
        gt.save(os.path.join(out_dir, "ground_truth.json"))
        save_comparison_png(m, gt, res, os.path.join(out_dir, "comparison.png"))
    lines = ["===== Class Work 8 : SLAM - Explore the Unknown World =====",
             f"Grid           : {m.w} x {m.h}  (cell {m.cell_m:.2f} m)",
             f"Start          : cell {tuple(m.start[:2]) if m.start else '-'}  heading {m.start[2] if m.start else '-'}",
             f"End            : cell {tuple(m.end[:2]) if m.end else '-'}",
             f"Visited cells  : {res['visited_cells']} / {res['total_cells']}",
             f"Coverage       : {res['coverage_pct']} %"]
    if gt is not None:
        lines += [f"Correct cells  : {res['correct_cells']} / {res['total_cells']}",
                  f"Map Accuracy   : {res['map_accuracy_pct']} %",
                  f"Edge accuracy  : {res['edge_accuracy_pct']} %  (per wall segment)",
                  f"Walls GT/detected/matched : {res['gt_walls']} / {res['detected_walls']} / {res['walls_matched']}"]
        wrong = [(x, y) for x in range(m.w) for y in range(m.h) if not res["cell_ok"][x][y]]
        lines.append(f"Wrong cells    : {wrong if wrong else 'none'}")
    else:
        lines.append("Map Accuracy   : (no ground truth yet — enter it in the GUI and press Evaluate)")
    lines += list(extra_lines)
    lines.append("")
    lines.append("ASCII map ( | --- = wall, ? = unknown, . = visited ):")
    lines.append(m.ascii())
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    res_out = {k: v for k, v in res.items() if k != "cell_ok"}
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(res_out, f, indent=1)
    return res, "\n".join(lines)


def load_traj_csv(path):
    traj = []
    with open(path) as f:
        for r in csv.DictReader(f):
            traj.append((float(r["t_s"]), float(r["est_x_m"]), float(r["est_y_m"]),
                         float(r["odom_x_m"]) if r["odom_x_m"] else None,
                         float(r["odom_y_m"]) if r["odom_y_m"] else None,
                         int(r["cell_x"]), int(r["cell_y"]), r["event"]))
    return traj
