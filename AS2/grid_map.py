"""
grid_map.py — แผนที่แบบกริด เก็บสถานะ "ขอบช่อง" (กำแพงระหว่างช่อง) ด้วย log-odds

ขอบแนวตั้ง  V[x][y]  x=0..W  : กำแพงด้านซ้ายของช่อง (x,y)  (V[W][y] = ขอบขวาสุด)
ขอบแนวนอน   H[x][y]  y=0..H  : กำแพงด้านล่างของช่อง (x,y)  (H[x][H] = ขอบบนสุด)
"""
import json
import heapq
from collections import deque

import config as C

DIRS = ["N", "E", "S", "W"]
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
UNKNOWN, OPEN, WALL = 0, 1, 2
STATE_NAME = {UNKNOWN: "UNKNOWN", OPEN: "OPEN", WALL: "WALL"}


def dir_index(d):
    return DIRS.index(d) if isinstance(d, str) else int(d) % 4


class GridMap:
    def __init__(self, w=C.GRID_W, h=C.GRID_H, cell_m=C.CELL_M,
                 boundary_walls=C.ASSUME_BOUNDARY_WALLS):
        if not (1 <= w <= C.MAX_GRID and 1 <= h <= C.MAX_GRID):
            raise ValueError(f"grid must be 1..{C.MAX_GRID}")
        self.w, self.h, self.cell_m = w, h, cell_m
        self.V = [[0.0] * h for _ in range(w + 1)]
        self.Hz = [[0.0] * (h + 1) for _ in range(w)]
        self.visited = [[False] * h for _ in range(w)]
        self.boundary_walls = boundary_walls
        if boundary_walls:
            for y in range(h):
                self.V[0][y] = C.L_CLAMP
                self.V[w][y] = C.L_CLAMP
            for x in range(w):
                self.Hz[x][0] = C.L_CLAMP
                self.Hz[x][h] = C.L_CLAMP
        self.start = None      # (x, y, heading)
        self.end = None
        self.meta = {}

    # ---------- edge access ----------
    def in_bounds(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def _edge_ref(self, x, y, d):
        d = dir_index(d)
        if d == 0:
            return ("H", x, y + 1)
        if d == 2:
            return ("H", x, y)
        if d == 1:
            return ("V", x + 1, y)
        return ("V", x, y)

    def get_lo(self, x, y, d):
        k, a, b = self._edge_ref(x, y, d)
        return self.Hz[a][b] if k == "H" else self.V[a][b]

    def mark_passed(self, x, y, d):
        """หุ่นเดินผ่านขอบนี้มาแล้วจริง (ถึงช่องใหม่ยืนยันแล้ว) -> โล่งแน่ ค่าเซนเซอร์ทีหลังห้ามเปลี่ยนเป็นกำแพง
        (log 1/10 06:34: หุ่นเยื้องที่ (5,5) อ่าน ToF W 415 / Sharp 190 -> ใส่กำแพง (4,5)|(5,5) ที่เพิ่งเดินผ่านมา
        -> กลับบ้านไม่ได้ แผนที่ผิดครึ่งสนาม)"""
        if not hasattr(self, "passed"):
            self.passed = set()
        self.passed.add(self._edge_ref(x, y, d))
        self.set_lo(x, y, d, -C.L_CLAMP, force=True)

    def is_passed(self, x, y, d):
        return self._edge_ref(x, y, d) in getattr(self, "passed", ())

    def set_lo(self, x, y, d, val, force=False):
        k, a, b = self._edge_ref(x, y, d)
        val = max(-C.L_CLAMP, min(C.L_CLAMP, val))
        if not force and (k, a, b) in getattr(self, "passed", ()) and val > -C.L_WALL_TH:
            return                  # ขอบที่เดินผ่านมาแล้ว: ไม่รับค่าที่จะทำให้ไม่โล่ง
        if k == "H":
            self.Hz[a][b] = val
        else:
            self.V[a][b] = val

    def is_boundary(self, x, y, d):
        d = dir_index(d)
        return not self.in_bounds(x + DX[d], y + DY[d])

    def update(self, x, y, d, is_wall):
        """เพิ่มหลักฐานจากเซนเซอร์ (Bayesian log-odds update)"""
        if self.boundary_walls and self.is_boundary(x, y, d):
            return
        lo = self.get_lo(x, y, d) + (C.L_HIT if is_wall else C.L_MISS)
        self.set_lo(x, y, d, lo)

    def state(self, x, y, d):
        lo = self.get_lo(x, y, d)
        if lo > C.L_WALL_TH:
            return WALL
        if lo < -C.L_WALL_TH:
            return OPEN
        return UNKNOWN

    def set_state(self, x, y, d, st):
        """แก้ด้วยมือ (manual edit) ; แก้มือชนะเสมอ (ล้างสถานะ "เดินผ่านแล้ว" ของขอบนี้)"""
        getattr(self, "passed", set()).discard(self._edge_ref(x, y, d))
        self.set_lo(x, y, d, {WALL: C.L_CLAMP, OPEN: -C.L_CLAMP, UNKNOWN: 0.0}[st], force=True)

    def can_move(self, x, y, d, allow_unknown=False):
        d = dir_index(d)
        nx, ny = x + DX[d], y + DY[d]
        if not self.in_bounds(nx, ny):
            return False
        s = self.state(x, y, d)
        return s == OPEN or (allow_unknown and s == UNKNOWN)

    # ---------- planning ----------
    def neighbors(self, x, y, allow_unknown=False):
        for d in range(4):
            if self.can_move(x, y, d, allow_unknown):
                yield d, x + DX[d], y + DY[d]

    def astar(self, start, goal, heading=None, allow_unknown=False, turn_cost=0.0):
        """A* บนกริด คืนลิสต์ทิศ [d0, d1, ...] หรือ None"""
        if start == goal:
            return []
        h0 = dir_index(heading) if heading is not None else -1
        def hfun(c):
            return abs(c[0] - goal[0]) + abs(c[1] - goal[1])
        openq = [(hfun(start), 0.0, start, h0)]
        came = {(start, h0): None}
        g = {(start, h0): 0.0}
        while openq:
            f, gc, cur, hd = heapq.heappop(openq)
            if cur == goal:
                path = []
                key = (cur, hd)
                while came[key] is not None:
                    prev, d = came[key]
                    path.append(d)
                    key = prev
                return path[::-1]
            if gc > g.get((cur, hd), 1e9):
                continue
            for d, nx, ny in self.neighbors(cur[0], cur[1], allow_unknown):
                cost = 1.0 + (turn_cost if (hd != -1 and d != hd) else 0.0)
                if allow_unknown and self.state(cur[0], cur[1], d) == UNKNOWN:
                    cost += 0.5
                nk = ((nx, ny), d)
                ng = gc + cost
                if ng < g.get(nk, 1e9):
                    g[nk] = ng
                    came[nk] = ((cur, hd), d)
                    heapq.heappush(openq, (ng + hfun((nx, ny)), ng, (nx, ny), d))
        return None

    def nearest_frontier(self, start):
        """BFS หาช่องที่ยังไม่เคยไป ซึ่งเดินถึงได้ผ่านขอบที่รู้ว่าโล่ง (ใกล้ที่สุด)"""
        q = deque([start])
        dist = {start: 0}
        best = None
        while q:
            c = q.popleft()
            if best is not None and dist[c] > dist[best[1]]:
                break
            if not self.visited[c[0]][c[1]]:
                # ใกล้เท่ากัน -> เลือกช่องที่รู้กำแพงรอบมากกว่า (น่าจะเป็นทางตัน/ซอก เก็บให้จบก่อน
                # ไม่ต้องย้อนกลับมาทีหลัง) sim 120 เขาวงกต: ก้าวลด ~2-4%
                w = sum(self.state(c[0], c[1], d) == WALL for d in range(4))
                if best is None or w > best[0]:
                    best = (w, c)
                continue
            for d, nx, ny in self.neighbors(c[0], c[1], allow_unknown=False):
                if (nx, ny) not in dist:
                    dist[(nx, ny)] = dist[c] + 1
                    q.append((nx, ny))
        return best[1] if best else None

    def reachable_cells(self, start):
        q = deque([start]); seen = {start}
        while q:
            c = q.popleft()
            for d, nx, ny in self.neighbors(c[0], c[1]):
                if (nx, ny) not in seen:
                    seen.add((nx, ny)); q.append((nx, ny))
        return seen

    # ---------- localisation helper ----------
    def signature(self, x, y):
        return tuple(self.state(x, y, d) for d in range(4))

    def match_candidates(self, observed_world):
        """observed_world: tuple 4 ค่า (WALL/OPEN) ตามทิศโลก N,E,S,W
        คืนช่องที่ลายกำแพงตรงกับที่สังเกต (ใช้ยืนยันตำแหน่ง / global relocalisation)"""
        out = []
        for x in range(self.w):
            for y in range(self.h):
                sig = self.signature(x, y)
                mism = sum(1 for a, b in zip(sig, observed_world)
                           if a != UNKNOWN and b != UNKNOWN and a != b)
                out.append((mism, (x, y)))
        out.sort()
        return out

    # ---------- stats ----------
    def counts(self):
        vis = sum(v for col in self.visited for v in col)
        return vis, self.w * self.h

    # ---------- IO ----------
    def to_dict(self):
        return {
            "grid_w": self.w, "grid_h": self.h, "cell_m": self.cell_m,
            "boundary_walls": self.boundary_walls,
            "V_logodds": self.V, "H_logodds": self.Hz,
            "visited": self.visited,
            "start": self.start, "end": self.end,
            "cells": [
                {"x": x, "y": y, "visited": self.visited[x][y],
                 **{d: STATE_NAME[self.state(x, y, i)] for i, d in enumerate(DIRS)}}
                for y in range(self.h) for x in range(self.w)
            ],
            "meta": self.meta,
        }

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=1, ensure_ascii=False)

    @classmethod
    def from_dict(cls, d):
        m = cls(d["grid_w"], d["grid_h"], d.get("cell_m", C.CELL_M),
                d.get("boundary_walls", True))
        if "V_logodds" in d:
            m.V = d["V_logodds"]; m.Hz = d["H_logodds"]
            m.visited = d.get("visited", m.visited)
        else:   # ไฟล์ที่เก็บเป็น cells แบบง่าย (เช่น ground truth ที่เขียนเอง)
            for c in d["cells"]:
                for i, dn in enumerate(DIRS):
                    if dn in c:
                        st = {"WALL": WALL, "OPEN": OPEN}.get(c[dn], UNKNOWN)
                        m.set_state(c["x"], c["y"], i, st)
        m.start = d.get("start"); m.end = d.get("end")
        m.meta = d.get("meta", {})
        return m

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def walls_rows(self):
        """สำหรับ walls_data.csv"""
        rows = []
        for x in range(self.w + 1):
            for y in range(self.h):
                rows.append(("V", x, y, STATE_NAME[self._st(self.V[x][y])], round(self.V[x][y], 2)))
        for x in range(self.w):
            for y in range(self.h + 1):
                rows.append(("H", x, y, STATE_NAME[self._st(self.Hz[x][y])], round(self.Hz[x][y], 2)))
        return rows

    @staticmethod
    def _st(lo):
        return WALL if lo > C.L_WALL_TH else OPEN if lo < -C.L_WALL_TH else UNKNOWN

    def ascii(self, robot=None):
        lines = []
        for y in range(self.h, -1, -1):
            s = "+"
            for x in range(self.w):
                st = self._st(self.Hz[x][y])
                s += {WALL: "---", OPEN: "   ", UNKNOWN: " ? "}[st] + "+"
            lines.append(s)
            if y == 0:
                break
            yy = y - 1
            s = ""
            for x in range(self.w + 1):
                st = self._st(self.V[x][yy])
                s += {WALL: "|", OPEN: " ", UNKNOWN: "?"}[st]
                if x < self.w:
                    if robot and (robot[0], robot[1]) == (x, yy):
                        s += " R "
                    else:
                        s += " . " if self.visited[x][yy] else "   "
            lines.append(s)
        return "\n".join(lines)


def random_maze(w, h, loops=0.15, seed=None):
    """สร้างเขาวงกตสุ่ม (ใช้เป็น ground truth ในโหมดจำลอง)"""
    import random
    rnd = random.Random(seed)
    m = GridMap(w, h, boundary_walls=True)
    for x in range(w):
        for y in range(h):
            for d in range(4):
                if not m.is_boundary(x, y, d):
                    m.set_state(x, y, d, WALL)
    stack = [(0, 0)]; seen = {(0, 0)}
    while stack:
        x, y = stack[-1]
        opts = [d for d in range(4) if m.in_bounds(x + DX[d], y + DY[d]) and (x + DX[d], y + DY[d]) not in seen]
        if not opts:
            stack.pop(); continue
        d = rnd.choice(opts)
        m.set_state(x, y, d, OPEN)
        n = (x + DX[d], y + DY[d]); seen.add(n); stack.append(n)
    for x in range(w):
        for y in range(h):
            for d in (0, 1):
                if not m.is_boundary(x, y, d) and m.state(x, y, d) == WALL and rnd.random() < loops:
                    m.set_state(x, y, d, OPEN)
    return m
