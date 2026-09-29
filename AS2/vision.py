"""
vision.py — ตรวจจับป้ายสี + แยกรูปทรง + ประมาณระยะจากขนาดจริง + ติดตามข้ามเฟรม

ขั้นตอนต่อเฟรม
  1) CLAHE บนช่อง L (Lab) ลดผลของแสงไม่สม่ำเสมอ
  2) ต่อสี: mask จากช่วง HSV  ∧  (ถ้าเรียนสีแล้ว) back-projection: P(สีป้าย) > ratio·P(สีพื้นหลัง)
     -> ตัดสีกำแพงที่ hue ใกล้กันแต่ความอิ่ม/ความสว่างต่างกันออก
  3) จุดสะท้อนแสง (V สูง S ต่ำ) ที่อยู่ติดก้อนสี เติมกลับเข้าไป + เติมรูในก้อน (ป้ายอะคริลิกเงา)
  4) ตัด ROI / โซนที่ไม่ต้องการ
  5) contour -> รูปทรง (วงกลม / สี่เหลี่ยมตั้ง / นอน / จัตุรัส) จาก circularity, rectangularity, aspect
  6) ระยะ Z = f · ขนาดจริง / ขนาดในภาพ  (ใช้ทั้งกว้างและสูง แล้วเช็คว่าสอดคล้องกัน -> กรองของปลอม)
"""
import math

import cv2
import numpy as np

from settings import SIGN_SIZES_CM

H_BINS, S_BINS = 36, 32


def focal_px(frame_w, hfov_deg):
    return (frame_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


ASPECT_VERT, ASPECT_HORZ = 0.82, 1.22     # ≈ sqrt(0.667), sqrt(1.5)


def put_text(img, text, org, scale=0.6, color=(255, 255, 255), bg=(0, 0, 0)):
    """ข้อความบนกล่องพื้นทึบ (OpenCV 5 วางตัวอักษรเส้นหนา/บางไม่ตรงกัน การทำขอบด้วยการเขียนซ้ำ 2 ชั้นจึงซ้อนเพี้ยน)"""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(img, (x - 2, y - th - 3), (x + tw + 2, y + base), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def shape_metrics(cnt):
    """ค่ารูปทรงของก้อนสี (ใช้ทั้ง detector และ calibrate)
    aspect = ความกว้างกลาง ÷ ความสูงกลาง: วัดความกว้างทุกแถว / ความสูงทุกคอลัมน์ในก้อนแล้วเอาค่ากลาง
      - ป้ายหันเฉียง (สี่เหลี่ยมด้านขนาน) ทุกแถวกว้างเท่าด้านจริง ไม่ใช่เท่ากรอบ bbox
      - มุมมน/รอยแสงสะท้อนกระทบแค่ไม่กี่แถว ค่ากลางจึงไม่เปลี่ยน
      (ไม่ชดเชยเลนส์: ข้อมูลจริง 29 ก.ย. ที่ขอบภาพ ~28° ค่าดิบตรงขนาดจริงกว่าค่าที่ชดเชย)"""
    area = cv2.contourArea(cnt)
    per = cv2.arcLength(cnt, True)
    if per <= 0 or area <= 0:
        return None
    x, y, bw, bh = cv2.boundingRect(cnt)
    m = np.zeros((bh, bw), np.uint8)
    cv2.drawContours(m, [cnt - np.array([x, y])], -1, 1, -1)
    rows = m.sum(1); cols = m.sum(0)
    w_med = float(np.median(rows[rows > 0])); h_med = float(np.median(cols[cols > 0]))
    (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
    (_, _), r_enc = cv2.minEnclosingCircle(cnt)
    return {"area": area,
            "circularity": 4 * math.pi * area / (per * per),
            "rectangularity": area / float(max(rw * rh, 1)),
            # เต็มวงกลมล้อมรอบแค่ไหน: วงกลม ≈0.9, จัตุรัส 2/π≈0.64, สี่เหลี่ยม 6x9 ≈0.59
            # (circularity ของวงกลมเล็กๆ ต่ำเพราะขอบเป็นขั้นบันได จึงแยกจากจัตุรัสไม่ได้)
            "circle_fill": area / max(math.pi * r_enc * r_enc, 1.0),
            "aspect": w_med / max(h_med, 1.0),
            "aspect_box": bw / float(max(bh, 1)),
            "solidity": area / max(cv2.contourArea(cv2.convexHull(cnt)), 1)}


# ------------------------------------------------------------------ colour learning
def learn_color_from_patch(hsv_patch, glare_v=225, glare_s=70, margin=(6, 40, 45)):
    """เรียนช่วง HSV + histogram จากกรอบที่ผู้ใช้ลากบนป้าย (ตัดจุดสะท้อนออกก่อน)"""
    px = hsv_patch.reshape(-1, 3).astype(np.float32)
    keep = ~((px[:, 2] >= glare_v) & (px[:, 1] <= glare_s))
    px = px[keep]
    if len(px) < 20:
        raise ValueError("กรอบเล็กเกินไป หรือมีแต่แสงสะท้อน")
    h = px[:, 0]
    # hue วนรอบ (สีแดงอยู่ทั้ง 0 และ 180) -> หา median แบบวงกลม
    ang = h * (2 * np.pi / 180.0)
    med = (math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) % (2 * np.pi)) * 180.0 / (2 * np.pi)
    dh = ((h - med + 90) % 180) - 90
    lo_h = np.percentile(dh, 2) - margin[0]
    hi_h = np.percentile(dh, 98) + margin[0]
    s_lo = max(0, np.percentile(px[:, 1], 3) - margin[1])
    v_lo = max(0, np.percentile(px[:, 2], 3) - margin[2])
    ranges = []
    a, b = med + lo_h, med + hi_h
    if a < 0:
        ranges += [[[0, s_lo, v_lo], [b, 255, 255]], [[180 + a, s_lo, v_lo], [180, 255, 255]]]
    elif b > 180:
        ranges += [[[a, s_lo, v_lo], [180, 255, 255]], [[0, s_lo, v_lo], [b - 180, 255, 255]]]
    else:
        ranges += [[[a, s_lo, v_lo], [b, 255, 255]]]
    ranges = [[[int(round(x)) for x in lo], [int(round(x)) for x in hi]] for lo, hi in ranges]
    hist = hs_hist(px)
    stats = {"h_med": round(med, 1), "s_p3": round(float(np.percentile(px[:, 1], 3)), 1),
             "v_p3": round(float(np.percentile(px[:, 2], 3)), 1), "n": int(len(px))}
    return ranges, hist, stats


def hs_hist(px):
    """px: Nx3 HSV -> histogram H-S normalize (list)"""
    arr = px[:, :2].reshape(-1, 1, 2).astype(np.float32)
    hist = cv2.calcHist([arr], [0, 1], None, [H_BINS, S_BINS], [0, 180, 0, 256])
    hist = cv2.GaussianBlur(hist, (3, 3), 0)
    hist /= max(hist.sum(), 1e-6)
    return hist.tolist()


def add_hist(old, new, w_old=0.5):
    if not old:
        return new
    a = np.array(old, np.float32); b = np.array(new, np.float32)
    c = a * w_old + b * (1 - w_old)
    c /= max(c.sum(), 1e-6)
    return c.tolist()


# ------------------------------------------------------------------ detector
class SignDetector:
    def __init__(self, settings):
        self.S = settings
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.last_masks = {}

    # ---------- preprocessing ----------
    def preprocess(self, bgr):
        if self.S["vision"].get("clahe", True):
            lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
            bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    def roi_mask(self, h, w):
        r = self.S["roi"]
        m = np.zeros((h, w), np.uint8)
        m[int(r["y0"] * h):int(r["y1"] * h), int(r["x0"] * w):int(r["x1"] * w)] = 255
        for x0, y0, x1, y1 in self.S.get("exclude", []):
            m[int(min(y0, y1) * h):int(max(y0, y1) * h), int(min(x0, x1) * w):int(max(x0, x1) * w)] = 0
        return m

    def color_mask(self, hsv, name, glare, roi):
        c = self.S["colors"][name]
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in c["ranges"]:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        # back-projection ตัดสีพื้นหลังที่เรียนไว้
        bg = self.S.get("bg_hist")
        if c.get("hist") and bg:
            fg_h = np.array(c["hist"], np.float32)
            bg_h = np.array(bg, np.float32)
            ratio = float(self.S["vision"].get("bg_ratio", 1.0))
            # คะแนน = fg/(fg+ratio*bg)  -> ยอมรับเมื่อ > 0.5
            score = fg_h / (fg_h + ratio * bg_h + 1e-9)
            lut = np.clip(score * 255, 0, 255).astype(np.float32)
            bp = cv2.calcBackProject([hsv], [0, 1], lut, [0, 180, 0, 256], 1)
            m &= (bp >= 128).astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m &= roi
        if self.S["vision"].get("glare_fill", True) and m.any():
            # จุดสะท้อนแสงที่อยู่ "ภายใน" convex hull ของก้อนสีเท่านั้นที่เติมกลับ
            # (กันไม่ให้ไปรวมเสาสีขาว/ผนังสว่างที่อยู่ติดขอบป้าย)
            grp = cv2.dilate(m, np.ones((7, 7), np.uint8))
            cnts, _ = cv2.findContours(grp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hullmask = np.zeros_like(m)
            for g in cnts:
                x, y, bw, bh = cv2.boundingRect(g)
                sub = m[y:y + bh, x:x + bw]
                ys, xs = np.nonzero(sub)
                if len(xs) < 5:
                    continue
                pts = np.stack([xs + x, ys + y], 1).astype(np.int32)
                cv2.fillConvexPoly(hullmask, cv2.convexHull(pts), 255)
            m |= (glare & hullmask)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        return m

    # ---------- shape ----------
    def classify(self, cnt, kx=1.0, ky=1.0):
        """kx, ky ไม่ได้ใช้คิดรูปทรงแล้ว (เก็บ signature เดิมไว้)"""
        V = self.S["vision"]
        g = shape_metrics(cnt)
        if g is None or g["solidity"] < 0.85:
            return None
        rect_ratio, aspect = g["rectangularity"], g["aspect"]
        shape = None
        if (0.75 <= g["aspect_box"] <= 1.33 and g["circle_fill"] >= V.get("circle_min_fill", 0.74)
                and rect_ratio <= 0.88 and g["circularity"] >= 0.6):
            shape = "CIRCLE"
        elif rect_ratio >= V["min_rectangularity"]:
            # เกณฑ์อยู่กึ่งกลาง (แบบ log) ระหว่าง ตั้ง 6:9=0.667 | จัตุรัส 1.0 | นอน 9:6=1.5
            if ASPECT_VERT <= aspect <= ASPECT_HORZ:
                shape = "SQUARE"
            elif 0.45 <= aspect < ASPECT_VERT:
                shape = "VERTICAL"
            elif ASPECT_HORZ < aspect <= 2.2:
                shape = "HORIZONTAL"
        if shape is None:
            return None
        x, y, bw, bh = cv2.boundingRect(cnt)
        return dict(g, shape=shape, bbox=(x, y, bw, bh))

    # ---------- main ----------
    def detect(self, bgr, keep_masks=False):
        h, w = bgr.shape[:2]
        V = self.S["vision"]
        f = focal_px(w, self.S["camera"]["hfov_deg"])
        hsv = self.preprocess(bgr)
        glare = cv2.inRange(hsv, (0, 0, V["glare_v_min"]), (180, V["glare_s_max"], 255))
        roi = self.roi_mask(h, w)
        dets = []
        masks = {}
        for name in self.S["colors"]:
            m = self.color_mask(hsv, name, glare, roi)
            if keep_masks:
                masks[name] = m
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < V["min_area_px"]:
                    continue
                # เลนส์มุมกว้าง: วัตถุที่มุม θ จากแกนกล้อง ถูกยืดตามแนวรัศมี 1/cos²θ (แนวตั้งฉาก 1/cosθ)
                M = cv2.moments(c)
                if M["m00"] <= 0:
                    continue
                cx = M["m10"] / M["m00"]; cy = M["m01"] / M["m00"]
                tx = math.atan2(cx - w / 2.0, f); ty = math.atan2(cy - h / 2.0, f)
                kx = math.cos(tx) ** 2 * math.cos(ty)
                ky = math.cos(ty) ** 2 * math.cos(tx)
                info = self.classify(c, kx, ky)
                if info is None:
                    continue
                x, y, bw, bh = info["bbox"]
                bw_c, bh_c = bw * kx, bh * ky
                # ตัดป้ายที่ถูกขอบภาพ/ROI/โซนตัดทิ้งตัด (วัดขนาด -> ระยะผิด -> จุดกระทบผิด)
                if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                    continue
                if roi[max(0, y - 2):y + bh + 2, max(0, x - 2):x + bw + 2].min() == 0:
                    continue
                W_cm, H_cm = SIGN_SIZES_CM[info["shape"]]
                if info["shape"] == "CIRCLE":
                    (ccx, ccy), rad = cv2.minEnclosingCircle(c)
                    d_px = (bw_c + bh_c) / 2.0
                    zw = zh = f * W_cm / d_px
                else:
                    zw = f * W_cm / bw_c
                    zh = f * H_cm / bh_c
                z = 0.35 * zw + 0.65 * zh          # ความสูงเชื่อได้กว่า (ป้ายหมุนซ้าย-ขวาทำให้ความกว้างหด)
                incons = abs(zw - zh) / max(z, 1e-6)
                if not (V["dist_min_cm"] <= z <= V["dist_max_cm"]):
                    continue
                if incons > V["size_consistency"] and info["shape"] != "CIRCLE":
                    continue
                conf = max(0.0, 1.0 - incons) * min(1.0, info["solidity"])
                dets.append({
                    "color": name, "shape": info["shape"], "cx": cx, "cy": cy,
                    "bbox": (x, y, bw, bh), "area": info["area"], "dist_cm": z,
                    "zw": zw, "zh": zh, "conf": conf, "contour": c,
                    "box": cv2.boxPoints(cv2.minAreaRect(c)),
                    "circularity": info["circularity"], "rectangularity": info["rectangularity"],
                    "aspect": info["aspect"],
                    # มุมของป้ายเทียบแกนกล้อง (ใช้ทำแผนที่ตำแหน่งเป้า)
                    "bearing_deg": math.degrees(math.atan2(cx - w / 2.0, f)),
                    "elev_deg": -math.degrees(math.atan2(cy - h / 2.0, f)),
                })
        # ป้ายเดียวกันถูกจับหลายสี -> เก็บอันที่มั่นใจ/ใหญ่กว่า
        dets.sort(key=lambda d: -d["area"])
        out = []
        for d in dets:
            if all(_iou(d["bbox"], o["bbox"]) < 0.3 for o in out):
                out.append(d)
        # เงาสะท้อนบนพื้นมัน: สีเดียวกัน อยู่ใต้ป้ายจริงในแนวเดียวกัน และกว้างเท่ากัน -> ตัดอันล่างทิ้ง
        # (ป้ายจริงอีกใบที่อยู่คนละระยะจะกว้างในภาพไม่เท่ากัน จึงไม่โดนตัด)
        keep = []
        for d in out:
            x, y, bw, bh = d["bbox"]
            refl = False
            for o in out:
                if o is d or o["color"] != d["color"]:
                    continue
                ox, oy, ow, oh = o["bbox"]
                overlap = max(0, min(x + bw, ox + ow) - max(x, ox)) / float(min(bw, ow))
                same_w = abs(bw - ow) <= 0.3 * max(bw, ow)
                if overlap > 0.5 and same_w and oy + oh <= y + 0.3 * bh and y - (oy + oh) < 2.5 * oh:
                    refl = True
                    break
            if not refl:
                keep.append(d)
        out = keep
        if keep_masks:
            self.last_masks = masks
        return out


def _iou(a, b):
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    return inter / float(aw * ah + bw * bh - inter + 1e-9)


# ------------------------------------------------------------------ tracker
class Tracker:
    """ติดตามป้ายข้ามเฟรม: จับคู่ด้วยสี+รูปทรง+ระยะใกล้สุด, กรองค่ากลางด้วย EMA,
    ต้องเจอ >= min_hits เฟรมถึงยืนยัน (กันของโผล่แวบเดียว)"""

    def __init__(self, match_px=70, min_hits=3, max_miss=8, alpha=0.5):
        self.tracks = {}
        self.next_id = 1
        self.match_px, self.min_hits, self.max_miss, self.alpha = match_px, min_hits, max_miss, alpha

    def reset(self):
        self.tracks.clear()

    def shift_all(self, dx, dy):
        """gimbal หมุน -> ตำแหน่งในภาพเลื่อน (ช่วยให้จับคู่ได้หลังหมุน)"""
        for t in self.tracks.values():
            t["cx"] += dx; t["cy"] += dy

    def update(self, dets):
        used = set()
        for tid, t in list(self.tracks.items()):
            best, bd = None, self.match_px
            for i, d in enumerate(dets):
                if i in used or d["color"] != t["color"] or d["shape"] != t["shape"]:
                    continue
                dist = math.hypot(d["cx"] - t["cx"], d["cy"] - t["cy"])
                if dist < bd:
                    best, bd = i, dist
            if best is None:
                t["miss"] += 1
                if t["miss"] > self.max_miss:
                    del self.tracks[tid]
                continue
            d = dets[best]; used.add(best)
            a = self.alpha
            for k in ("cx", "cy", "dist_cm"):
                t[k] = a * d[k] + (1 - a) * t[k]
            t.update({"det": d, "hits": t["hits"] + 1, "miss": 0})
        for i, d in enumerate(dets):
            if i in used:
                continue
            self.tracks[self.next_id] = {"id": self.next_id, "color": d["color"], "shape": d["shape"],
                                         "cx": d["cx"], "cy": d["cy"], "dist_cm": d["dist_cm"],
                                         "det": d, "hits": 1, "miss": 0}
            self.next_id += 1
        return self.confirmed()

    def confirmed(self):
        return [t for t in self.tracks.values() if t["hits"] >= self.min_hits and t["miss"] == 0]


# ------------------------------------------------------------------ drawing
def draw(frame, dets_or_tracks, settings, hit_px=None, highlight_id=None, show_roi=True):
    h, w = frame.shape[:2]
    if show_roi:
        r = settings["roi"]
        cv2.rectangle(frame, (int(r["x0"] * w), int(r["y0"] * h)), (int(r["x1"] * w) - 1, int(r["y1"] * h) - 1),
                      (0, 255, 255), 1)
        for x0, y0, x1, y1 in settings.get("exclude", []):
            p0 = (int(min(x0, x1) * w), int(min(y0, y1) * h)); p1 = (int(max(x0, x1) * w), int(max(y0, y1) * h))
            ov = frame.copy()
            cv2.rectangle(ov, p0, p1, (60, 60, 60), -1)
            cv2.addWeighted(ov, 0.45, frame, 0.55, 0, frame)
            cv2.rectangle(frame, p0, p1, (0, 0, 0), 1)
    for o in dets_or_tracks:
        d = o.get("det", o)
        col = tuple(int(v) for v in settings["colors"][d["color"]]["draw"])
        thick = 3 if (highlight_id is not None and o.get("id") == highlight_id) else 2
        if d["shape"] == "CIRCLE":
            (ccx, ccy), rad = cv2.minEnclosingCircle(d["contour"])
            cv2.circle(frame, (int(ccx), int(ccy)), int(rad), col, thick)
        else:
            cv2.drawContours(frame, [np.intp(d["box"])], 0, col, thick)
        x, y, bw, bh = d["bbox"]
        label = f"{d['color'][0]}-{d['shape'][:3]} {d['dist_cm']:.0f}cm"
        if "id" in o:
            label = f"#{o['id']} " + label
        light = 0.114 * col[0] + 0.587 * col[1] + 0.299 * col[2] > 140
        put_text(frame, label, (x, max(14, y - 6)), 0.45, (0, 0, 0) if light else (255, 255, 255), bg=col)
        cv2.circle(frame, (int(d["cx"]), int(d["cy"])), 3, col, -1)
    # crosshair กลางภาพ + จุดที่กระสุนจะตก (จากโมเดล calibrate)
    cv2.drawMarker(frame, (w // 2, h // 2), (200, 200, 200), cv2.MARKER_CROSS, 18, 1)
    if hit_px is not None:
        cv2.drawMarker(frame, (int(hit_px[0]), int(hit_px[1])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
    return frame
