"""
calibration.py — calibrate จุดกระทบ IR/เจลแบบอัตโนมัติ + รายงานผล + ทดสอบความแม่นก่อน/หลัง

ปัญหา: แสง IR ติดแวบเดียว (1-3 เฟรม) และภาพจากหุ่นมาช้ากว่าจริง ~0.2-0.4 วิ -> กดเองไม่ทัน
วิธี:  1) เก็บภาพ "ก่อนยิง" หลายเฟรม -> median เป็นภาพพื้นหลัง (ตัด noise)
       2) สั่งยิง แล้วบันทึก "ทุกเฟรม" ต่อเนื่อง 1.5 วิ (ครอบคลุม delay ของ stream)
       3) ทุกเฟรม: ภาพต่าง = เฟรม − พื้นหลัง  หาจุดที่สว่างขึ้นมากที่สุด
          ต้องเด่นกว่า noise (SNR) และเป็นจุดเล็ก (ถ้าทั้งภาพเปลี่ยน = แสงห้อง/กล้องสั่น -> ไม่นับ)
       4) เลือกเฟรมที่จุดเด่นที่สุด -> หาจุดศูนย์กลางแบบถ่วงน้ำหนักความสว่าง (ละเอียดกว่า 1 px)
       5) ยิงหลายนัดต่อระยะ ตัดนัดที่หลุดกลุ่ม (outlier) แล้วค่อยใช้ fit โมเดล
"""
import math
import os
import time

import cv2
import numpy as np

from vision import focal_px


# ------------------------------------------------------------------ flash detection
def _diff_map(frame, bg):
    d = frame.astype(np.int16) - bg
    d = np.clip(d.max(axis=2), 0, 255).astype(np.uint8)       # ช่องสีที่สว่างขึ้นมากที่สุด
    d = cv2.GaussianBlur(d, (5, 5), 0)
    med = int(np.median(d))
    if med:
        d = cv2.subtract(d, med)                              # ชดเชยแสงทั้งภาพเปลี่ยนเล็กน้อย
    return d


def find_flash(pre_frames, post, border=0.02, min_peak=25, min_snr=8.0, max_area_ratio=0.015, roi=None):
    """pre_frames: list ภาพก่อนยิง, post: list (t, frame) หลังยิง
    roi: mask uint8 (255 = ค้นหา) — ตัดส่วนที่มีคน/ของขยับ (เช่นเหนือกำแพง) และลำกล้องออก
    คืน dict(pt, peak, snr, area, idx, t) ของเฟรมที่เจอจุดเด่นที่สุด หรือ None"""
    bg = np.median(np.stack(pre_frames).astype(np.int16), axis=0).astype(np.int16)
    h, w = bg.shape[:2]
    bx, by = int(w * border), int(h * border)
    best = None
    for i, (t, f) in enumerate(post):
        d = _diff_map(f, bg)
        d[:by] = 0; d[-by:] = 0; d[:, :bx] = 0; d[:, -bx:] = 0
        if roi is not None:
            d[roi == 0] = 0
        _, peak, _, loc = cv2.minMaxLoc(d)
        if peak < min_peak:
            continue
        mad = float(np.median(np.abs(d.astype(np.float32) - np.median(d)))) * 1.4826
        noise = max(mad, float(d.std()) * 0.5, 1.0)
        snr = peak / noise
        m = (d >= peak * 0.5).astype(np.uint8)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
        k = lab[loc[1], loc[0]]
        area = int(stats[k, cv2.CC_STAT_AREA])
        if area > max_area_ratio * w * h or snr < min_snr:
            continue
        # จุดศูนย์กลางถ่วงน้ำหนักความสว่าง
        ys, xs = np.nonzero(lab == k)
        wts = d[ys, xs].astype(np.float64)
        pt = (float((xs * wts).sum() / wts.sum()), float((ys * wts).sum() / wts.sum()))
        if best is None or snr > best["snr"]:
            best = {"pt": pt, "peak": int(peak), "snr": round(snr, 1), "area": area, "idx": i, "t": t}
    return best


def capture_shot(grab, ammo="ir", post_s=1.5, settle_s=0.6, log=print, detector=None):
    """ยิง 1 นัด + จับภาพอัตโนมัติ -> dict(pt, frame, pre, delay_s, ...) หรือ None
    detector: ถ้าให้มา จะค้นหาแสงเฉพาะใน ROI (ไม่รวมโซนตัดทิ้ง) ของมัน"""
    grab.freeze = False
    time.sleep(settle_s)                      # รอให้นิ่ง + แสงจากนัดก่อนหายหมด
    pre = grab.recent(6)
    if len(pre) < 2:
        time.sleep(0.5); pre = grab.recent(6)
    if len(pre) < 2:
        log("  [capture] ยังไม่มีภาพจากกล้อง"); return None
    grab.start_record()
    t_fire = time.time()
    grab.fire(ammo)
    time.sleep(post_s)
    post = grab.stop_record()
    roi = detector.roi_mask(*pre[0].shape[:2]) if detector is not None else None
    res = find_flash(pre, post, roi=roi)
    if res is None:
        log(f"  [capture] ไม่เจอแสงในภาพ ({len(post)} เฟรมใน {post_s}s) — ลองลดแสงห้อง/ใช้กระดานสีขาวด้าน/เข้าใกล้ขึ้น")
        return None
    res.update({"frame": post[res["idx"]][1], "pre": pre[-1], "delay_s": round(res["t"] - t_fire, 3),
                "n_frames": len(post)})
    return res


def annotate(res, text, path):
    img = res["frame"].copy()
    x, y = res["pt"]
    h, w = img.shape[:2]
    cv2.drawMarker(img, (w // 2, h // 2), (200, 200, 200), cv2.MARKER_CROSS, 24, 1)
    cv2.circle(img, (int(round(x)), int(round(y))), 14, (0, 0, 255), 2)
    cv2.line(img, (w // 2, h // 2), (int(x), int(y)), (0, 0, 255), 1)
    from vision import put_text
    put_text(img, text, (10, 26), 0.6)
    cv2.imwrite(path, img)


# ------------------------------------------------------------------ calibration session
def auto_calibrate(grab, model, z_cm, ammo="ir", shots=5, out_dir="logs/calib", log=print, stop=None,
                   detector=None):
    """ยิง shots นัดที่ระยะ z_cm (กระดานตั้งฉาก) -> เพิ่มตัวอย่างเฉพาะนัดที่อยู่ในกลุ่ม"""
    os.makedirs(out_dir, exist_ok=True)
    pts, zs = [], []
    w = h = None
    for k in range(shots):
        if stop is not None and stop.is_set():
            break
        r = capture_shot(grab, ammo, log=log, detector=detector)
        if r is None:
            continue
        h, w = r["frame"].shape[:2]
        z = z_cm
        if z_cm is None and detector is not None:            # ใช้ระยะจากป้ายบนกระดาน
            ds = detector.detect(r["pre"])
            if not ds:
                log("  ไม่เห็นป้ายบนกระดานสำหรับวัดระยะ"); continue
            z = max(ds, key=lambda d: d["area"])["dist_cm"]
        pts.append(r["pt"]); zs.append(z)
        dx, dy = r["pt"][0] - w / 2, r["pt"][1] - h / 2
        log(f"  นัด {k + 1}/{shots}: จุด ({r['pt'][0]:.1f},{r['pt'][1]:.1f}) ห่างกลางภาพ ({dx:+.1f},{dy:+.1f})px "
            f"SNR={r['snr']} delay={r['delay_s']}s")
        annotate(r, f"{ammo} Z={z:.0f}cm shot{k + 1} d=({dx:+.1f},{dy:+.1f})px",
                 os.path.join(out_dir, f"{ammo}_Z{int(round(z))}_shot{k + 1}.jpg"))
    if not pts:
        return {"ok": 0, "z": z_cm}
    P = np.array(pts)
    med = np.median(P, axis=0)
    dist = np.linalg.norm(P - med, axis=1)
    mad = np.median(dist) * 1.4826
    keep = dist <= max(3 * mad, 6.0)
    for (u, v), z, kp in zip(pts, zs, keep):
        if kp:
            model.add_sample(w, h, z, u, v, ammo)
    f = focal_px(w, model.S["camera"]["hfov_deg"])
    zmean = float(np.mean(np.array(zs)[keep]))
    spread_px = float(np.std(P[keep], axis=0).mean()) if keep.sum() > 1 else 0.0
    out = {"ok": int(keep.sum()), "rejected": int((~keep).sum()), "z": zmean,
           "mean_px": (float(P[keep, 0].mean() - w / 2), float(P[keep, 1].mean() - h / 2)),
           "spread_px": spread_px, "spread_cm": spread_px * zmean / f}
    log(f"[calib] Z={zmean:.0f}cm ใช้ได้ {out['ok']} นัด (ตัดทิ้ง {out['rejected']}) "
        f"จุดเฉลี่ยห่างกลาง ({out['mean_px'][0]:+.1f},{out['mean_px'][1]:+.1f})px  "
        f"กระจาย ±{spread_px:.1f}px (≈±{out['spread_cm']:.2f} ซม.)")
    return out


# ------------------------------------------------------------------ report
def fit_report(model, ammo, frame_w, frame_h, out_dir="logs/calib"):
    """fit + เขียนรายงาน + กราฟ ; คืนข้อความสรุป"""
    os.makedirs(out_dir, exist_ok=True)
    fit_msg = model.fit(ammo)
    p = model.params(ammo)
    S = model.S
    s = np.array(p["samples"], float)
    if len(s) == 0:
        return fit_msg
    k = frame_w / float(S["hit_model"]["base_w"])
    f = focal_px(frame_w, S["camera"]["hfov_deg"])
    z, du, dv = s[:, 0], s[:, 1] * k, s[:, 2] * k           # px ที่ความละเอียดจริง
    pu = k * (p["Ax"] + p["Bx"] / z)
    pv = k * (p["Ay"] + p["By"] / z + p["Cy"] * z)
    # "ก่อน" = เล็งด้วย crosshair กลางภาพ -> คลาดเท่ากับระยะจุดจากกลางภาพ
    before_cm = np.hypot(du, dv) * z / f
    after_cm = np.hypot(du - pu, dv - pv) * z / f
    lines = ["===== IR/Gel Hit-point Calibration Report =====",
             fit_msg, "",
             f"ความละเอียดภาพ {frame_w}x{frame_h}  HFOV {S['camera']['hfov_deg']:.1f}°  f={f:.0f}px", "",
             "ระยะ(ซม.) | จุดจริงห่างกลาง(px) | โมเดลทำนาย(px) | คลาดถ้าเล็ง crosshair(ซม.) | คลาดหลัง calibrate(ซม.)"]
    for zi in sorted(set(np.round(z, 0))):
        m = np.abs(z - zi) < 0.6
        lines.append(f"{zi:8.0f}  | ({du[m].mean():+6.1f},{dv[m].mean():+6.1f})       | "
                     f"({pu[m].mean():+6.1f},{pv[m].mean():+6.1f})    | {before_cm[m].mean():8.2f}"
                     f"                  | {after_cm[m].mean():6.2f}")
    lines += ["",
              f"เฉลี่ยทุกนัด: เล็ง crosshair คลาด {before_cm.mean():.2f} ซม.  ->  หลัง calibrate คลาด {after_cm.mean():.2f} ซม.",
              f"(ป้ายเล็กสุด 6 ซม. = ต้องคลาดไม่เกิน ~3 ซม. จึงยังโดน)", "",
              "ทำนายจุดกระทบที่ระยะต่างๆ (ใช้ตอนยิงจริง):"]
    for zz in (40, 60, 80, 100, 120, 150, 200):
        u = k * (p["Ax"] + p["Bx"] / zz); v = k * (p["Ay"] + p["By"] / zz + p["Cy"] * zz)
        lines.append(f"  Z={zz:4d} ซม.: จุดกระทบอยู่ห่างกลางภาพ ({u:+6.1f},{v:+6.1f}) px "
                     f"= ({u * zz / f:+5.2f},{v * zz / f:+5.2f}) ซม. บนเป้า")
    text = "\n".join(lines)
    with open(os.path.join(out_dir, f"calib_report_{ammo}.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
        zz = np.linspace(max(20, z.min() * 0.8), z.max() * 1.25, 100)
        ax[0].scatter(z, du, c="tab:red", label="measured dx")
        ax[0].scatter(z, dv, c="tab:blue", label="measured dy")
        ax[0].plot(zz, k * (p["Ax"] + p["Bx"] / zz), "r-", label="model dx = A+B/Z")
        ax[0].plot(zz, k * (p["Ay"] + p["By"] / zz + p["Cy"] * zz), "b-", label="model dy")
        ax[0].axhline(0, color="gray", lw=0.8)
        ax[0].set_xlabel("distance Z (cm)"); ax[0].set_ylabel("hit point - image centre (px)")
        ax[0].set_title(f"Hit point vs distance ({ammo})"); ax[0].legend(fontsize=8)
        order = np.argsort(z)
        ax[1].plot(z[order], before_cm[order], "o-", c="tab:orange", label="aim with crosshair (before)")
        ax[1].plot(z[order], after_cm[order], "o-", c="tab:green", label="aim with model (after)")
        ax[1].axhline(3.0, color="gray", ls="--", lw=0.8, label="half of 6 cm sign")
        ax[1].set_xlabel("distance Z (cm)"); ax[1].set_ylabel("miss distance (cm)")
        ax[1].set_title("Expected miss: before vs after"); ax[1].legend(fontsize=8)
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, f"calib_plot_{ammo}.png"), dpi=150); plt.close(fig)
    except Exception as e:
        text += f"\n(วาดกราฟไม่ได้: {e})"
    return text


# ------------------------------------------------------------------ accuracy test
def accuracy_test(aimer, grab, trials=6, ammo="ir", compare=True, out_dir="logs/calib", log=print, stop=None):
    """ทดสอบจริง: เล็งป้ายบนกระดาน -> ยิง IR -> จับจุด -> วัดว่าคลาดจากจุดเล็งกี่ ซม.
    compare=True สลับ 'ไม่ใช้โมเดล (crosshair)' กับ 'ใช้โมเดล' ทีละนัด เพื่อเทียบก่อน/หลัง"""
    os.makedirs(out_dir, exist_ok=True)
    S = aimer.S
    model_backup = dict(S["hit_model"][ammo])
    rows = []
    ds = []
    for _ in range(6):                      # ลองหลายเฟรม
        frame0 = grab.read_frame()
        ds = aimer.det.detect(frame0) if frame0 is not None else []
        if ds:
            break
    if not ds:
        return "ไม่เห็นป้ายในภาพ — ติดป้าย 1 ใบบนกระดานแล้วหันกล้องไปทางนั้น"
    tgt = max(ds, key=lambda d: d["area"])
    color, shape = tgt["color"], tgt["shape"]
    log(f"[test] เป้า {color}/{shape} ระยะ {tgt['dist_cm']:.0f} ซม.")
    for i in range(trials):
        if stop is not None and stop.is_set():
            break
        use_model = (i % 2 == 1) if compare else True
        if not use_model:
            S["hit_model"][ammo] = dict(model_backup, Ax=0.0, Bx=0.0, Ay=0.0, By=0.0, Cy=0.0)
        try:
            r = aimer.aim_and_fire(color, shape, start_xy="center", fire=False, ammo=ammo)
        finally:
            S["hit_model"][ammo] = model_backup
        if "err_yaw" not in r:
            log("  เล็งไม่ได้ ข้าม"); continue
        # จุดเล็งจากภาพนิ่งก่อนยิงจริง
        time.sleep(0.3)
        pre = grab.recent(1)[-1]
        d = aimer.find_target(aimer.det.detect(pre), (color, shape, None))
        if d is None:
            continue
        from aiming import target_point
        ax, ay = target_point(d, S["aim"]["aim_y_ratio"])
        cap = capture_shot(grab, ammo, settle_s=0.1, log=log, detector=aimer.det)
        if cap is None:
            continue
        h, w = pre.shape[:2]
        f = focal_px(w, S["camera"]["hfov_deg"])
        ex, ey = cap["pt"][0] - ax, cap["pt"][1] - ay
        err_cm = math.hypot(ex, ey) * d["dist_cm"] / f
        x, y, bw, bh = d["bbox"]
        inside = x <= cap["pt"][0] <= x + bw and y <= cap["pt"][1] <= y + bh
        mode = "model" if use_model else "crosshair"
        rows.append((mode, d["dist_cm"], ex, ey, err_cm, inside))
        log(f"  นัด {i + 1} [{mode:9s}] คลาด ({ex:+.1f},{ey:+.1f})px = {err_cm:.2f} ซม.  {'โดนป้าย' if inside else 'หลุดป้าย'}")
        annotate(cap, f"test {mode} err={err_cm:.2f}cm {'HIT' if inside else 'MISS'}",
                 os.path.join(out_dir, f"test_{i + 1}_{mode}.jpg"))
    lines = ["===== Accuracy Test =====", f"เป้า {color}/{shape}  ระยะ ~{tgt['dist_cm']:.0f} ซม.  กระสุน {ammo}"]
    for mode in ("crosshair", "model"):
        rr = [r for r in rows if r[0] == mode]
        if rr:
            e = np.array([r[4] for r in rr])
            lines.append(f"{mode:9s}: {len(rr)} นัด  คลาดเฉลี่ย {e.mean():.2f} ซม. (สูงสุด {e.max():.2f})  "
                         f"โดนป้าย {sum(r[5] for r in rr)}/{len(rr)}")
    text = "\n".join(lines)
    with open(os.path.join(out_dir, "accuracy_test.txt"), "a", encoding="utf-8") as fh:
        fh.write(time.strftime("%Y-%m-%d %H:%M:%S") + "\n" + text + "\n\n")
    with open(os.path.join(out_dir, "accuracy_test.csv"), "a", encoding="utf-8") as fh:
        for r in rows:
            fh.write(",".join(str(round(v, 3)) if isinstance(v, float) else str(v) for v in r) + "\n")
    return text
