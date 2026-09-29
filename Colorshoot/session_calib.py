"""
session_calib.py — calibrate จากป้ายจริง 4 ใบ (สี/รูปทรงครบ) วางเรียงหน้ากล้อง หุ่น + gimbal อยู่นิ่ง

ขั้นตอน
  1) เก็บภาพจากกล้องต่อเนื่อง duration วินาที (1 ภาพทุก interval วินาที)
  2) หาก้อนสีอิ่ม (S สูง) บนพื้นหลังขาว/เทา -> จัดเข้าสีที่ใกล้ที่สุดด้วย hue
     -> รวมพิกเซลเนื้อสีจากทุกเฟรม แล้วเรียนช่วง HSV + histogram (ทนแสงกระพริบ/noise กว่าการลากกรอบภาพเดียว)
  3) วัดรูปทรงของก้อนสี (rectangularity, circularity, aspect) -> ถ้าป้ายจริงไม่ผ่านเกณฑ์ เสนอเกณฑ์ใหม่
  4) ตรวจจับทุกเฟรมด้วยค่าเดิม เทียบกับค่าใหม่: อัตราเจอ, รูปทรงถูกไหม, ระยะ, การสั่นของตำแหน่ง
  5) (ถ้าใส่ระยะจริง) หา HFOV จากความสูงป้าย + เช็คว่าขนาดป้ายใน settings ตรงของจริงไหม
  6) บันทึกทั้งหมดไว้ที่ logs/calib_session_<เวลา>/
       raw/            ภาพดิบ PNG (เปิดซ้ำได้ด้วยโหมด "ไฟล์ภาพ/วิดีโอ/โฟลเดอร์" หรือ run.py --calib-folder)
       annotated/      ภาพที่วาดผลตรวจจับ + ROI ปัจจุบัน + ROI ที่แนะนำ (ใช้ดูตอนตั้ง ROI)
       masks/          mask ของแต่ละสี (ทุก 10 เฟรม)
       compare_old_new.jpg  เฟรมแรก ค่าเดิม (ซ้าย) vs ค่าใหม่ (ขวา)
       detections.csv  ทุกป้ายที่ตรวจเจอทุกเฟรม (ค่าเดิม/ค่าใหม่)
       summary.txt / summary.json   สรุปผล
       learned_settings.json        settings ทั้งชุดหลัง calibrate (คัดลอกเป็น settings.json ได้)
"""
import copy
import csv
import datetime
import glob
import json
import math
import os
import time

import cv2
import numpy as np

import vision
from settings import SIGN_SIZES_CM
from vision import focal_px

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")


# ------------------------------------------------------------------ capture
def capture(grab, duration_s=15.0, interval_s=0.3, log=print, stop=None):
    """เก็บภาพจาก Grabbed -> list (t, frame, gimbal_angles)"""
    grab.freeze = False
    frames = []
    t0 = time.time()
    last = -1e9
    next_log = 2.0
    while time.time() - t0 < duration_s:
        if stop is not None and stop.is_set():
            log("  [session] หยุดเก็บภาพ (STOP)")
            break
        f = grab.read_frame()
        now = time.time()
        if f is None or now - last < interval_s:
            continue
        last = now
        frames.append((round(now - t0, 3), f, grab.gimbal_angles()))
        if now - t0 >= next_log:
            log(f"  [session] เก็บภาพแล้ว {len(frames)} ภาพ ({now - t0:.0f}/{duration_s:.0f} วิ)")
            next_log += 2.0
    return frames


def load_folder(path):
    """โหลดภาพจากโฟลเดอร์ (เช่น raw/ ของ session ก่อน) -> list (t, frame, None)"""
    files = sorted(sum([glob.glob(os.path.join(path, e)) for e in ("*.png", "*.jpg", "*.jpeg", "*.bmp")], []))
    out = []
    for i, p in enumerate(files):
        img = cv2.imread(p)
        if img is not None:
            out.append((float(i), img, None))
    return out


# ------------------------------------------------------------------ colour blobs
def _circ_mean_hue(h):
    ang = h.astype(np.float64) * (2 * np.pi / 180.0)
    return (math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) % (2 * np.pi)) * 180.0 / (2 * np.pi)


def _hue_dist(a, b):
    d = abs(a - b) % 180
    return min(d, 180 - d)


def hue_centers(settings):
    """hue กลางของแต่ละสีจากช่วงใน settings (สีแดงมี 2 ช่วงรอบ 0/180 -> เฉลี่ยแบบวงกลม)"""
    out = {}
    for name, c in settings["colors"].items():
        s = c_ = 0.0
        for lo, hi in c["ranges"]:
            wgt = max(1.0, hi[0] - lo[0])
            a = (lo[0] + hi[0]) / 2.0 * (2 * np.pi / 180.0)
            s += wgt * math.sin(a); c_ += wgt * math.cos(a)
        out[name] = (math.atan2(s, c_) % (2 * np.pi)) * 180.0 / (2 * np.pi)
    return out


def _geometry(cnt, w, h, f):
    """ค่ารูปทรงแบบเดียวกับ SignDetector.classify"""
    return vision.shape_metrics(cnt)


def find_color_blobs(det, bgr, centers, s_mins=(80, 120, 160), v_min=40, min_area_ratio=0.0003,
                     max_area_ratio=0.08, max_hue_dist=22):
    """ก้อนสีอิ่มที่ใหญ่ที่สุดของแต่ละสี -> {สี: dict(mask, bbox, hue, geometry)}
    ลองหลายเกณฑ์ S: กำแพงสีอ่อน (เช่นกำแพงเหลืองสนาม S≈115-135) ผ่านเกณฑ์ต่ำแล้วรวมเป็นก้อนใหญ่กับป้าย
    -> ก้อนแบบนั้นถูกตัด (ใหญ่เกิน/แตะขอบภาพ/ไม่ทึบ) แล้วเกณฑ์ที่สูงขึ้นจะแยกป้ายออกมาได้เอง"""
    hsv = det.preprocess(bgr)
    h, w = hsv.shape[:2]
    roi = det.roi_mask(h, w)
    f = focal_px(w, det.S["camera"]["hfov_deg"])
    best = {}
    wide = {}                                # ก้อนใหญ่/แตะขอบ (กำแพง) ของแต่ละสี ที่เกณฑ์ S ต่ำสุด
    for s_min in s_mins:
        m = ((hsv[..., 1] >= s_min) & (hsv[..., 2] >= v_min)).astype(np.uint8) * 255
        m &= roi
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
        for k in range(1, n):
            area = int(stats[k, cv2.CC_STAT_AREA])
            if area < min_area_ratio * w * h:
                continue
            x, y, bw, bh = (int(v) for v in stats[k, :4])
            comp = lab == k
            hue = _circ_mean_hue(hsv[..., 0][comp])
            name, dist = min(((c, _hue_dist(hue, hc)) for c, hc in centers.items()), key=lambda t: t[1])
            if dist > max_hue_dist:
                continue
            if area > max_area_ratio * w * h or x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                if s_min == s_mins[0]:
                    wide[name] = wide.get(name, np.zeros((h, w), bool)) | comp
                continue
            if name in best and best[name]["area"] >= area:
                continue
            cm = comp.astype(np.uint8) * 255
            cnts, _ = cv2.findContours(cm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            geo = _geometry(max(cnts, key=cv2.contourArea), w, h, f)
            if geo is None or geo["solidity"] < 0.8:          # ป้ายเป็นก้อนทึบ กำแพง/เงาเป็นก้อนแหว่ง
                continue
            best[name] = {"mask": cm, "area": area, "bbox": (x, y, bw, bh), "hue": hue, "geo": geo,
                          "s_min": s_min}
    # ค่า S ของ "กำแพง" สีเดียวกับป้าย (ตัดส่วนที่เป็นป้ายออก) -> ใช้ตั้งขอบล่าง S ให้อยู่เหนือกำแพง
    signs = np.zeros((h, w), np.uint8)
    for b in best.values():
        signs |= b["mask"]
    signs = cv2.dilate(signs, np.ones((9, 9), np.uint8)) > 0
    wall_s = {}
    for name, wm in wide.items():
        s = hsv[..., 1][wm & ~signs]
        if len(s) > 500:
            wall_s[name] = s[:: max(1, len(s) // 5000)]
    return hsv, best, wall_s


def learn_colors(det, frames, log=print, max_px_per_frame=4000):
    """เรียนสีจากก้อนสีทุกเฟรม -> (learned {สี: dict}, geo {สี: list}, seen {สี: จำนวนเฟรมที่เจอ})"""
    centers = hue_centers(det.S)
    V = det.S["vision"]
    rng = np.random.default_rng(0)
    px, geo, seen, walls = {}, {}, {}, {}
    for _, f, _ in frames:
        hsv, blobs, wall_s = find_color_blobs(det, f, centers)
        for name, s in wall_s.items():
            walls.setdefault(name, []).append(s)
        for name, b in blobs.items():
            x, y, bw, bh = b["bbox"]
            k = max(3, int(0.2 * min(bw, bh)) | 1)           # ตัดขอบก้อนทิ้ง ~20% เอาเฉพาะเนื้อสี
            inner = cv2.erode(b["mask"], np.ones((k, k), np.uint8)) > 0
            p = hsv[inner]
            # เก็บเฉพาะพิกเซลที่ hue ใกล้สีนี้มากกว่าสีอื่น (ขอบที่ผสมกับพื้นหลังทำให้ช่วง hue กว้างจนทับสีอื่น)
            dh = {c: np.minimum(np.abs(p[:, 0].astype(np.float32) - hc) % 180,
                                180 - np.abs(p[:, 0].astype(np.float32) - hc) % 180) for c, hc in centers.items()}
            p = p[np.all([dh[name] <= dh[c] for c in centers if c != name], axis=0)] if len(centers) > 1 else p
            if len(p) < 30:
                continue
            if len(p) > max_px_per_frame:
                p = p[rng.choice(len(p), max_px_per_frame, replace=False)]
            px.setdefault(name, []).append(p)
            seen[name] = seen.get(name, 0) + 1
            if b["geo"]:
                geo.setdefault(name, []).append(b["geo"])
    learned = {}
    for name, lst in px.items():
        allp = np.concatenate(lst)
        try:
            ranges, hist, st = vision.learn_color_from_patch(allp.reshape(-1, 1, 3), V["glare_v_min"], V["glare_s_max"])
        except ValueError as e:
            log(f"  [session] เรียนสี {name} ไม่ได้: {e}")
            continue
        for lo, _ in ranges:        # พิกเซลมืดมาก hue/S เชื่อไม่ได้ (เงา เทปดำ) -> ไม่ให้ V ต่ำกว่า 30
            lo[2] = max(lo[2], 30)
        if name in walls:           # มีกำแพง/พื้นที่กว้างสีเดียวกัน -> ขอบล่าง S อยู่กึ่งกลางระหว่างกำแพงกับป้าย
            ws = float(np.percentile(np.concatenate(walls[name]), 95))
            ps = float(np.percentile(allp[:, 1], 5))
            st["wall_s_p95"], st["sign_s_p5"] = round(ws, 1), round(ps, 1)
            if ps > ws + 10:
                s_cut = int(round((ws + ps) / 2))
                for lo, _ in ranges:
                    lo[1] = max(lo[1], s_cut)
                st["s_cut_from_wall"] = s_cut
            else:
                st["wall_overlap"] = True
        med = np.median(allp.astype(np.float32), 0)
        draw = cv2.cvtColor(np.uint8([[med]]), cv2.COLOR_HSV2BGR)[0, 0]
        st.update({"frames": seen[name], "s_p50": round(float(np.median(allp[:, 1])), 1),
                   "v_p50": round(float(np.median(allp[:, 2])), 1)})
        learned[name] = {"ranges": ranges, "hist": hist, "draw": [int(v) for v in draw], "stats": st}
    return learned, geo, seen


# ------------------------------------------------------------------ evaluation
def _targets(expected, colors):
    """รายการป้ายที่ประเมิน: [(key, สี, รูปทรงหรือ None)]
    ไม่ระบุรูปทรง (expected ว่าง) -> ประเมินต่อสี ป้ายสีละ 1 ใบ รูปทรงอะไรก็ได้"""
    if expected:
        return [(f"{c}/{s}", c, s) for c, s in expected]
    return [(c, c, None) for c in colors]


def evaluate(settings, frames, expected, keep_masks_every=0, colors=None):
    """ตรวจจับทุกเฟรม -> (rows สำหรับ csv, stats ต่อป้าย, dets ต่อเฟรม, masks {เฟรม: masks})
    ของปลอม = ป้ายที่ไม่ตรงรายการ (ระบุรูปทรง) หรือป้ายสีเดียวกันเกิน 1 ใบในภาพ (ไม่ระบุรูปทรง)"""
    det = vision.SignDetector(settings)
    tg = _targets(expected, colors or list(settings["colors"]))
    rows, per_frame, masks = [], [], {}
    st = {k: {"found": 0, "shapes": {}, "cx": [], "cy": [], "dist": [], "zw": [], "zh": [],
              "bearing": [], "elev": []} for k, _, _ in tg}
    fp = 0
    exp_set = {(c, s) for c, s in expected} if expected else None
    for i, (t, f, ang) in enumerate(frames):
        km = bool(keep_masks_every) and i % keep_masks_every == 0
        dets = det.detect(f, keep_masks=km)
        if km:
            masks[i] = dict(det.last_masks)
        per_frame.append(dets)
        for d in dets:
            x, y, bw, bh = d["bbox"]
            ok = (d["color"], d["shape"]) in exp_set if exp_set else d["color"] in st
            rows.append([i, t, d["color"], d["shape"], int(ok),
                         round(d["cx"], 1), round(d["cy"], 1), x, y, bw, bh, round(d["dist_cm"], 1),
                         round(d["zw"], 1), round(d["zh"], 1), round(d["aspect"], 3), round(d["rectangularity"], 3),
                         round(d["circularity"], 3), round(d["conf"], 3),
                         "" if ang is None else round(ang[0], 2), "" if ang is None else round(ang[1], 2)])
            if exp_set and not ok:
                fp += 1
        if not exp_set:
            for c in st:
                fp += max(0, sum(1 for d in dets if d["color"] == c) - 1)
        for key, c, s in tg:
            e = st[key]
            same = [d for d in dets if d["color"] == c and (s is None or d["shape"] == s)]
            if same:
                d = max(same, key=lambda d: d["area"])
                e["found"] += 1
                e["shapes"][d["shape"]] = e["shapes"].get(d["shape"], 0) + 1
                for k, v in (("cx", d["cx"]), ("cy", d["cy"]), ("dist", d["dist_cm"]), ("zw", d["zw"]),
                             ("zh", d["zh"]), ("bearing", d["bearing_deg"]), ("elev", d["elev_deg"])):
                    e[k].append(v)
            else:
                for d in dets:
                    if d["color"] == c:
                        e["shapes"]["ผิด:" + d["shape"]] = e["shapes"].get("ผิด:" + d["shape"], 0) + 1
    n = max(1, len(frames))
    summary = {}
    for key, e in st.items():
        a = {k: np.array(v, float) for k, v in e.items() if isinstance(v, list)}
        good = {k: v for k, v in e["shapes"].items() if not k.startswith("ผิด:")}
        summary[key] = {
            "rate": round(e["found"] / n, 3), "found": e["found"], "shapes": e["shapes"],
            "shape": max(good, key=good.get) if good else None,
            "dist_mean": round(float(a["dist"].mean()), 1) if len(a["dist"]) else None,
            "dist_std": round(float(a["dist"].std()), 2) if len(a["dist"]) else None,
            "cx_std_px": round(float(a["cx"].std()), 2) if len(a["cx"]) else None,
            "cy_std_px": round(float(a["cy"].std()), 2) if len(a["cy"]) else None,
            "_raw": {k: v.tolist() for k, v in a.items()},
        }
    summary["_false_positives"] = fp
    return rows, summary, per_frame, masks


def hfov_from_distance(summary, z_cm, frame_w, hfov_now):
    """ใช้ความสูงป้าย (ไม่เปลี่ยนเมื่อป้ายหันซ้าย-ขวา) หา focal -> HFOV ต่อป้าย + ขนาดป้ายที่ควรเป็น"""
    f_now = focal_px(frame_w, hfov_now)
    per, fs = {}, {}
    for key, sm in summary.items():
        if key.startswith("_") or not sm.get("shape"):
            continue
        s = sm["shape"]
        r = sm.get("_raw", {})
        if not r or not len(r.get("zh", [])):
            continue
        zh, zw = np.array(r["zh"]), np.array(r["zw"])
        tx, ty = np.radians(r["bearing"]), np.radians(r["elev"])
        z_exp = z_cm / (np.cos(ty) ** 2 * np.cos(tx))     # ระยะตามแนวสายตาที่ detector ควรวัดได้ (ป้ายขนานกล้อง)
        f_h = float(np.median(f_now * z_exp / zh)); f_w = float(np.median(f_now * z_exp / zw))
        fs[key] = (f_h, f_w)
        per[key] = {
            "shape": s,
            "hfov": round(2 * math.degrees(math.atan((frame_w / 2.0) / f_h)), 2),
            "dist_measured": round(float(np.median(0.35 * zw + 0.65 * zh)), 1),
            "dist_expected": round(float(np.median(z_exp)), 1),
            "size_cm_setting": list(SIGN_SIZES_CM[s]),
        }
    if not per:
        return None
    vals = np.array([p["hfov"] for p in per.values()])
    hfov = float(np.median(vals))
    f_fit = (frame_w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    for k, (f_h, f_w) in fs.items():
        # ถ้าใช้ HFOV ที่ fit ได้ ป้ายใบนี้ต้องมีขนาดจริงเท่าไหร่ถึงจะวัดระยะได้ตรง (ต่างจาก setting = ขนาดใน settings ผิด)
        W, H = per[k]["size_cm_setting"]
        per[k]["size_cm_implied"] = [round(W * f_fit / f_w, 2), round(H * f_fit / f_h, 2)]
    return {"hfov": round(hfov, 2), "hfov_spread": round(float(vals.max() - vals.min()), 2), "per_sign": per}


def suggest_vision(geo, expected, V, failing=()):
    """ถ้ารูปทรงของป้ายจริงไม่ผ่านเกณฑ์ -> เสนอเกณฑ์ใหม่ (ไม่ต่ำกว่าขอบที่ยังแยกรูปทรงได้)
    เสนอเฉพาะสีใน failing (ป้ายที่ตรวจไม่เจอจริงหลังเรียนสีแล้ว) เพราะก้อนสีที่นี่ยังไม่ได้เติมแสงสะท้อน
    จึงดูแหว่งกว่าที่ detector เห็น"""
    shape_of = dict((c, s) for c, s in expected)
    sug, info = {}, {}
    for name, lst in geo.items():
        rect = np.array([g["rectangularity"] for g in lst]); circ = np.array([g["circle_fill"] for g in lst])
        asp = np.array([g["aspect"] for g in lst])
        if name not in shape_of:            # ไม่ได้ระบุรูปทรง -> เดาจากก้อนสี (วงกลมเต็มวงล้อมรอบ ≈0.9 สี่เหลี่ยม ≤0.64)
            am = float(np.median(asp))
            shape_of[name] = ("CIRCLE" if float(np.median(circ)) > 0.78 else
                              "SQUARE" if vision.ASPECT_VERT <= am <= vision.ASPECT_HORZ else
                              "VERTICAL" if am < vision.ASPECT_VERT else "HORIZONTAL")
        info[name] = {"shape": shape_of.get(name), "rect_p10": round(float(np.percentile(rect, 10)), 3),
                      "rect_med": round(float(np.median(rect)), 3), "circ_p10": round(float(np.percentile(circ, 10)), 3),
                      "circ_med": round(float(np.median(circ)), 3), "aspect_med": round(float(np.median(asp)), 3)}
        shp = shape_of.get(name)
        if name not in failing:
            continue
        if shp in ("VERTICAL", "HORIZONTAL", "SQUARE") and info[name]["rect_p10"] < V["min_rectangularity"]:
            cand = max(0.70, info[name]["rect_p10"] - 0.02)
            sug["min_rectangularity"] = round(min(sug.get("min_rectangularity", 1.0), cand), 3)
        if shp == "CIRCLE" and info[name]["circ_p10"] < V.get("circle_min_fill", 0.74):
            cand = max(0.70, info[name]["circ_p10"] - 0.02)       # ยังห่างจากจัตุรัส (0.64)
            sug["circle_min_fill"] = round(min(sug.get("circle_min_fill", 1.0), cand), 3)
    return sug, info


def suggest_roi(per_frame, targets, w, h):
    """กรอบรวมของป้ายที่ตรวจเจอทุกเฟรม ขยายขึ้น 1.5 เท่าความสูงป้าย ลง 0.5 เท่า (ตัดเงาบนพื้น) กว้างเต็มภาพ
    (แนะนำเท่านั้น: ป้ายที่ระยะอื่นจะอยู่สูง/ต่ำต่างไปในภาพ)"""
    ys0, ys1, hs = [], [], []
    for dets in per_frame:
        for _, c, s in targets:            # ใช้เฉพาะป้ายใหญ่สุดของแต่ละรายการ (ไม่เอาของปลอม)
            same = [d for d in dets if d["color"] == c and (s is None or d["shape"] == s)]
            if same:
                x, y, bw, bh = max(same, key=lambda d: d["area"])["bbox"]
                ys0.append(y); ys1.append(y + bh); hs.append(bh)
    if not ys0:
        return None
    sh = float(np.median(hs))
    y0 = max(0.0, (min(ys0) - 1.5 * sh) / h); y1 = min(1.0, (max(ys1) + 0.5 * sh) / h)
    return {"x0": 0.0, "y0": round(y0, 3), "x1": 1.0, "y1": round(y1, 3)}


# ------------------------------------------------------------------ images
def _annotate(frame, dets, settings, text, roi_sug=None):
    img = vision.draw(frame.copy(), dets, settings)
    h, w = img.shape[:2]
    if roi_sug:
        p0 = (int(roi_sug["x0"] * w), int(roi_sug["y0"] * h)); p1 = (int(roi_sug["x1"] * w) - 1, int(roi_sug["y1"] * h) - 1)
        for x in range(p0[0], p1[0], 16):                      # เส้นประสีม่วง = ROI ที่แนะนำ
            cv2.line(img, (x, p0[1]), (min(x + 8, p1[0]), p0[1]), (255, 0, 255), 2)
            cv2.line(img, (x, p1[1]), (min(x + 8, p1[0]), p1[1]), (255, 0, 255), 2)
    vision.put_text(img, text, (10, 26), 0.6)
    return img


def _mask_grid(masks, settings):
    tiles = []
    for name, m in masks.items():
        t = cv2.cvtColor(cv2.resize(m, (m.shape[1] // 2, m.shape[0] // 2)), cv2.COLOR_GRAY2BGR)
        col = tuple(int(v) for v in settings["colors"][name]["draw"])
        cv2.rectangle(t, (0, 0), (t.shape[1] - 1, t.shape[0] - 1), col, 3)
        cv2.putText(t, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
        tiles.append(t)
    if not tiles:
        return None
    while len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    return np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])


# ------------------------------------------------------------------ main
def analyze(frames, S, z_cm=None, expected=None, apply=True, apply_hfov=True, out_dir=None, log=print,
            save_raw=True):
    """วิเคราะห์ภาพที่เก็บมา -> บันทึกผล + (apply) แก้ S ในที่ -> คืน dict สรุป (มี backup ของค่าเดิม)"""
    if not frames:
        log("[session] ไม่มีภาพ")
        return None
    # expected ว่าง = ไม่ระบุรูปทรง -> ประเมินต่อสี (ป้ายแต่ละชุดรูปทรงไม่เหมือนกัน)
    if expected is None:
        expected = S.get("calib_session", {}).get("expected") or []
    expected = [list(e) for e in expected]
    targets = _targets(expected, list(S["colors"]))
    out_dir = out_dir or os.path.join(LOG_DIR, "calib_session_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    for sub in ("raw", "annotated", "masks"):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=True)
    h, w = frames[0][1].shape[:2]
    before = copy.deepcopy(S)
    log(f"[session] วิเคราะห์ {len(frames)} ภาพ ({w}x{h}) -> {out_dir}")
    if save_raw:
        for i, (_, f, _) in enumerate(frames):
            cv2.imwrite(os.path.join(out_dir, "raw", f"frame_{i:03d}.png"), f)

    # 1) ค่าเดิม
    rows_old, sum_old, dets_old, _ = evaluate(before, frames, expected)

    # 2) เรียนสี + เกณฑ์รูปทรง
    cand = copy.deepcopy(before)
    learned, geo, seen = learn_colors(vision.SignDetector(before), frames, log=log)
    for name, L in learned.items():
        c = cand["colors"].setdefault(name, {"draw": L["draw"]})
        c["ranges"], c["hist"], c["draw"] = L["ranges"], L["hist"], L["draw"]
    _, sum_col, _, _ = evaluate(cand, frames, expected)
    failing = {c for key, c, s in targets if sum_col[key]["rate"] < 0.9}
    sug, geo_info = suggest_vision(geo, expected, cand["vision"], failing)
    cand["vision"].update(sug)
    warn = []
    for c in dict.fromkeys(c for _, c, _ in targets):
        if c not in learned:
            warn.append(f"ไม่เจอก้อนสี {c} ในภาพเลย — เช็คว่าป้ายอยู่ในภาพ/ROI และไม่มืดเกินไป")
    for name, L in learned.items():
        if L["stats"].get("wall_overlap"):
            warn.append(f"{name}: ความอิ่มสี (S) ของป้ายซ้อนกับพื้นที่กว้างสีเดียวกัน (กำแพง?) "
                        f"S กำแพง p95={L['stats']['wall_s_p95']} ป้าย p5={L['stats']['sign_s_p5']} "
                        "-> ลากเรียนสีพื้นหลังบนกำแพง หรือใช้ ROI/โซนตัดทิ้ง")
    y = learned.get("YELLOW")
    if y and "wall_s_p95" not in y["stats"] and y["ranges"][0][0][1] < 140:
        warn.append(f"YELLOW เรียนได้ S≥{y['ranges'][0][0][1]} ซึ่งต่ำกว่า 140: กำแพงสนามสีเหลือง (S≈115–135) อาจหลุดเข้ามา "
                    "-> ที่สนามให้ลากเรียนสีพื้นหลังบนกำแพง หรือเลื่อนแถบ S lo ของ YELLOW ขึ้นไป ~150")

    # 3) HFOV จากระยะจริง
    _, sum_mid, _, _ = evaluate(cand, frames, expected)
    hf = hfov_from_distance(sum_mid, z_cm, w, cand["camera"]["hfov_deg"]) if z_cm else None
    if hf and apply_hfov:
        if hf["hfov_spread"] > 6:
            warn.append(f"HFOV จากแต่ละป้ายต่างกัน {hf['hfov_spread']}° — ขนาดป้ายบางใบใน settings น่าจะไม่ตรงของจริง "
                        "(ดู size_cm_implied) จึงไม่ปรับ HFOV อัตโนมัติ")
        else:
            cand["camera"]["hfov_deg"] = hf["hfov"]

    # 4) ค่าใหม่ (ชุดสุดท้าย)
    rows_new, sum_new, dets_new, masks = evaluate(cand, frames, expected, keep_masks_every=10)
    roi_sug = suggest_roi(dets_new, targets, w, h)

    # ---------- บันทึกภาพ
    for i, (t, f, _) in enumerate(frames):
        n_ok = sum(1 for _, c, s in targets if any(d["color"] == c and (s is None or d["shape"] == s) for d in dets_new[i]))
        cv2.imwrite(os.path.join(out_dir, "annotated", f"frame_{i:03d}.jpg"),
                    _annotate(f, dets_new[i], cand, f"#{i} t={t:.1f}s  new settings  {n_ok}/{len(targets)} signs", roi_sug),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
    for i, m in masks.items():
        g = _mask_grid(m, cand)
        if g is not None:
            cv2.imwrite(os.path.join(out_dir, "masks", f"frame_{i:03d}_masks.jpg"), g, [cv2.IMWRITE_JPEG_QUALITY, 85])
    f0 = frames[0][1]
    a = _annotate(f0, dets_old[0], before, "OLD settings")
    b = _annotate(f0, dets_new[0], cand, "NEW settings", roi_sug)
    cv2.imwrite(os.path.join(out_dir, "compare_old_new.jpg"), np.hstack([a, b]), [cv2.IMWRITE_JPEG_QUALITY, 90])

    # ---------- บันทึกข้อมูล
    hdr = ["frame", "t_s", "color", "shape", "expected", "cx", "cy", "x", "y", "w", "h", "dist_cm", "zw", "zh",
           "aspect", "rectangularity", "circularity", "conf", "gimbal_pitch", "gimbal_yaw"]
    with open(os.path.join(out_dir, "detections.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        wr = csv.writer(fh)
        wr.writerow(["settings"] + hdr)
        for tag, rows in (("old", rows_old), ("new", rows_new)):
            for r in rows:
                wr.writerow([tag] + r)

    def strip(sm):
        return {k: ({kk: vv for kk, vv in v.items() if kk != "_raw"} if isinstance(v, dict) else v) for k, v in sm.items()}

    applied = []
    if apply:
        for name in learned:
            S["colors"][name] = cand["colors"][name]
        S["vision"].update(sug)
        applied.append("สีที่เรียนได้: " + ", ".join(learned))
        if sug:
            applied.append(f"เกณฑ์รูปทรง: {sug}")
        if S["camera"]["hfov_deg"] != cand["camera"]["hfov_deg"]:
            S["camera"]["hfov_deg"] = cand["camera"]["hfov_deg"]
            applied.append(f"HFOV = {cand['camera']['hfov_deg']}°")
    result = {
        "time": datetime.datetime.now().isoformat(timespec="seconds"), "out_dir": out_dir,
        "n_frames": len(frames), "frame_size": [w, h], "z_cm": z_cm, "expected": expected,
        "learned_colors": {k: {"ranges": v["ranges"], "stats": v["stats"]} for k, v in learned.items()},
        "blob_geometry": geo_info, "vision_suggestions": sug,
        "hfov_before": before["camera"]["hfov_deg"], "hfov_analysis": hf, "hfov_after": cand["camera"]["hfov_deg"],
        "detection_old": strip(sum_old), "detection_new": strip(sum_new),
        "roi_current": before["roi"], "roi_suggested": roi_sug, "warnings": warn,
        "applied": applied if apply else [],
    }
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=1, ensure_ascii=False)
    with open(os.path.join(out_dir, "learned_settings.json"), "w", encoding="utf-8") as fh:
        json.dump(cand, fh, indent=1, ensure_ascii=False)
    text = report_text(result)
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    log(text)
    result["backup"] = before
    return result


def report_text(r):
    L = ["===== Calibrate จากป้าย 4 ใบ =====",
         f"{r['time']}  ภาพ {r['n_frames']} ภาพ  {r['frame_size'][0]}x{r['frame_size'][1]}  "
         f"ระยะจริง: {str(r['z_cm']) + ' ซม.' if r['z_cm'] else 'ไม่ได้ใส่'}", "",
         "สีที่เรียนได้ (รวมทุกเฟรม):"]
    for k, v in r["learned_colors"].items():
        st = v["stats"]
        rng = "; ".join(f"H{lo[0]}-{hi[0]} S≥{lo[1]} V≥{lo[2]}" for lo, hi in v["ranges"])
        L.append(f"  {k:7s} {rng}   (H กลาง {st['h_med']}, S กลาง {st['s_p50']}, V กลาง {st['v_p50']}, "
                 f"เจอใน {st['frames']}/{r['n_frames']} ภาพ)")
        if "s_cut_from_wall" in st:
            L.append(f"          เจอพื้นที่กว้างสีเดียวกัน (S p95={st['wall_s_p95']}) ป้าย S p5={st['sign_s_p5']} "
                     f"-> ตั้ง S≥{st['s_cut_from_wall']} กันกำแพงหลุดเข้ามา")
    L += ["", "รูปทรงของก้อนสีจริง (p10 = ค่าต่ำสุดเกือบทั้งหมด):"]
    for k, g in r["blob_geometry"].items():
        L.append(f"  {k:7s} [{g['shape']}] rect p10={g['rect_p10']} กลาง={g['rect_med']}  "
                 f"เต็มวงกลม p10={g['circ_p10']} กลาง={g['circ_med']}  aspect={g['aspect_med']}")
    if r["vision_suggestions"]:
        L.append(f"  -> ปรับเกณฑ์: {r['vision_suggestions']}")
    L += ["", "อัตราตรวจจับ (ค่าเดิม -> ค่าใหม่):"]
    for key in r["detection_new"]:
        if key.startswith("_"):
            continue
        o, n = r["detection_old"][key], r["detection_new"][key]
        extra = ""
        if n["dist_mean"] is not None:
            extra = f"  ระยะ {n['dist_mean']}±{n['dist_std']} ซม.  สั่น ({n['cx_std_px']},{n['cy_std_px']}) px"
        if n["shapes"]:
            extra += f"  รูปทรงที่จับได้ {n['shapes']}"
        L.append(f"  {key:18s} {o['rate'] * 100:5.1f}% -> {n['rate'] * 100:5.1f}%{extra}")
    fp_what = "ป้ายที่ไม่ตรงรายการ" if r["expected"] else "ป้ายสีเดียวกันเกิน 1 ใบในภาพ"
    L.append(f"  ของปลอม ({fp_what}): {r['detection_old']['_false_positives']} -> "
             f"{r['detection_new']['_false_positives']} ครั้ง")
    hf = r["hfov_analysis"]
    if hf:
        L += ["", f"HFOV จากระยะจริง: {hf['hfov']}° (ต่างกันระหว่างป้าย {hf['hfov_spread']}°)  "
                  f"เดิม {r['hfov_before']}° -> ใช้ {r['hfov_after']}°"]
        for k, p in hf["per_sign"].items():
            L.append(f"  {k:18s} HFOV {p['hfov']}°  ระยะวัดได้ (HFOV เดิม) {p['dist_measured']} / ควรเป็น "
                     f"{p['dist_expected']} ซม.  ขนาดใน settings {p['size_cm_setting']} ซม. -> ที่ HFOV ใหม่ "
                     f"ขนาดจริงน่าจะ {p['size_cm_implied']} ซม. (กว้าง, สูง)")
    L += ["", f"ROI ปัจจุบัน {r['roi_current']}", f"ROI ที่แนะนำ (เส้นประม่วงในภาพ annotated) {r['roi_suggested']}"]
    if r["warnings"]:
        L += ["", "⚠ ข้อควรระวัง:"] + [f"  - {x}" for x in r["warnings"]]
    L += ["", ("ใช้ค่าแล้ว: " + " | ".join(r["applied"]) + "  (กด 💾 บันทึกค่าทั้งหมด เพื่อเก็บลง settings.json)")
          if r["applied"] else "ยังไม่ได้ใช้ค่า (ดู learned_settings.json)",
          f"ผลทั้งหมดอยู่ที่ {r['out_dir']}"]
    return "\n".join(L)


def run_session(grab, S, z_cm=None, duration_s=15.0, interval_s=0.3, expected=None, apply=True, apply_hfov=True,
                log=print, stop=None):
    log(f"[session] เก็บภาพ {duration_s:.0f} วิ (ทุก {interval_s} วิ) — อย่าขยับหุ่น/ป้าย")
    frames = capture(grab, duration_s, interval_s, log=log, stop=stop)
    return analyze(frames, S, z_cm=z_cm, expected=expected, apply=apply, apply_hfov=apply_hfov, log=log)
