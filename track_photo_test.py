#!/usr/bin/env python3
"""Piece tracking with the REAL YOLO model: a virtual drone flies over labelled aerial photos.

Each photo (UAVVaste, labels in COCO format) is treated as a patch of ground. A virtual camera flies two passes
over it (north, then south with the image upside down, partly overlapping), cutting out what the camera would
see every 1/fps seconds. YOLO runs on each cut-out; the poses get wandering GPS like track_sim.py. Then the old
GPS-only merge and the new tracker are scored against the labelled litter.

  python track_photo_test.py --images DIR --annotations annotations.json [--model models/litter_ground_drone_1280.pt]

Limits: a still photo has no motion blur, lighting change or tilt, so detections are steadier than in flight.
"""
import argparse, json, logging, math, os, shutil, tempfile, time

import cv2
import numpy as np

import survey_logger as sl
import track_sim


def fly_photo(img, gts, model, a, rng):
    """img: photo (north = up after rotation); gts: [x, y, w, h] boxes. Returns logged frames + per-frame stats."""
    H, W = img.shape[:2]
    hfov, vfov = sl.CAMERAS[a.camera]
    across = 2 * a.alt * math.tan(math.radians(hfov) / 2)   # metres the frame covers side to side
    fw = int(W * a.frame_frac)                                # frame width in photo pixels
    fh = int(fw * sl.DETECT[1] / sl.DETECT[0])
    s = across / fw                                           # metres per photo pixel
    pad = max(fw, fh)
    big = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=[int(c) for c in img.reshape(-1, 3).mean(0)])
    centres = np.array([[x + w / 2, y + h / 2] for x, y, w, h in gts])
    halfs = np.array([[max(w, 16) * 0.65, max(h, 16) * 0.65] for x, y, w, h in gts])

    spacing = fw * (1 - a.overlap)
    passes = [(W / 2 - spacing / 2, True), (W / 2 + spacing / 2, False)]
    lat0, lon0 = 40.7402, -73.8410
    k = sl.R_EARTH * math.cos(math.radians(lat0))
    gps, vb = np.zeros(2), np.zeros(2)
    frames, t, dt = [], 0.0, 1 / a.fps
    sx, sy = sl.CAPTURE[0] / sl.DETECT[0], sl.CAPTURE[1] / sl.DETECT[1]
    turn_s = 4.0
    # timeline: (cx, cy, yaw, velocity north, velocity east, look) at time t; positions in photo pixels
    def where(t):
        L = (H + fh) * s / a.speed                            # seconds per pass
        if t < L:                                             # pass 1, northbound
            return passes[0][0], H + fh / 2 - a.speed * t / s, 0.0, a.speed, 0.0, True
        if t < L + turn_s:                                    # turn: slide east, rotate
            f = (t - L) / turn_s
            return passes[0][0] + f * spacing, -fh / 2, f * math.pi, 0.0, spacing * s / turn_s, False
        if t < 2 * L + turn_s:                                # pass 2, southbound
            return passes[1][0], -fh / 2 + a.speed * (t - L - turn_s) / s, math.pi, -a.speed, 0.0, True
        return None

    while (w := where(t)) is not None:
        cx, cy, yaw, tvn, tve, look = w
        north = yaw < math.pi / 2
        dets = []
        if look:
            crop = big[int(cy - fh / 2) + pad:int(cy - fh / 2) + pad + fh, int(cx - fw / 2) + pad:int(cx - fw / 2) + pad + fw]
            if not north:
                crop = cv2.rotate(crop, cv2.ROTATE_180)          # camera top points south
            small = cv2.resize(crop, sl.DETECT, interpolation=cv2.INTER_AREA)
            r = model(small, imgsz=a.imgsz, conf=a.conf, verbose=False)[0]
            for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
                u, v = (x1 + x2) / 2 * fw / sl.DETECT[0], (y1 + y2) / 2 * fh / sl.DETECT[1]   # in crop pixels
                if not north:
                    u, v = fw - u, fh - v
                px, py = cx - fw / 2 + u, cy - fh / 2 + v                                       # in photo pixels
                inside = np.all(np.abs(centres - [px, py]) <= halfs, axis=1)
                item = int(np.argmin(np.hypot(*(centres - [px, py]).T) + 1e9 * ~inside)) if inside.any() else None
                dets.append({"box": [x1 * sx, y1 * sy, x2 * sx, y2 * sy], "conf": float(c), "item": item})
        # true drone position in metres (north up = photo y down); readings keep coming during the turn
        n, e = -(cy - H / 2) * s, (cx - W / 2) * s
        gps = gps * (1 - dt / a.gps_tau) + rng.normal(0, a.gps * math.sqrt(2 * dt / a.gps_tau), 2)
        vb = vb * (1 - dt / 30) + rng.normal(0, 0.05 * math.sqrt(2 * dt / 30), 2)
        lat, lon = lat0 + math.degrees((n + gps[0]) / sl.R_EARTH), lon0 + math.degrees((e + gps[1]) / k)
        pose = {"t": t, "lat": lat, "lon": lon, "alt": a.alt + rng.normal(0, 0.05),
                "roll": rng.normal(0, math.radians(0.5)), "pitch": rng.normal(0, math.radians(0.5)),
                "yaw": sl.wrap(yaw + rng.normal(0, math.radians(1.5))),
                "vx": tvn + vb[0] + rng.normal(0, 0.1), "vy": tve + vb[1] + rng.normal(0, 0.1)}
        frames.append({"t": t, "pose": pose, "dets": dets})
        t += dt + rng.uniform(-0.03, 0.03)
    return frames, s


def load(path, anns, img_id):
    img = cv2.imread(path, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)   # labels are on raw pixels
    gts = [x["bbox"] for x in anns if x["image_id"] == img_id]
    H, W = img.shape[:2]
    if W > H:   # fly along the long side: rotate so it points north
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)          # (x, y) -> (H - 1 - y, x)
        gts = [[H - y - h, x, h, w] for x, y, w, h in gts]
    return img, gts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True)
    ap.add_argument("--annotations", required=True)
    ap.add_argument("--model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "litter_ground_drone_1280.pt"))
    ap.add_argument("--imgsz", type=int, nargs=2, default=[736, 1280])
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--camera", default="cm3", choices=sl.CAMERAS)
    ap.add_argument("--alt", type=float, default=4.6)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--frame-frac", type=float, default=1.0, help="camera frame width as a share of the photo's short side")
    ap.add_argument("--overlap", type=float, default=0.4, help="side overlap of the two passes")
    ap.add_argument("--gps", type=float, default=2.5)
    ap.add_argument("--gps-tau", type=float, default=8.0)
    ap.add_argument("--min-sightings", type=int, default=3)
    ap.add_argument("--merge-m", type=float, default=2.0)
    ap.add_argument("--nested", type=float, default=0.0)
    ap.add_argument("--cache", help="save/load YOLO results here (json) so other settings can be tried quickly")
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    from ultralytics import YOLO
    model = YOLO(a.model, task="detect")

    coco = json.load(open(a.annotations))
    byname = {im["file_name"]: im for im in coco["images"]}
    keys = ["pieces", "found", "findable", "duplicates", "lost", "false"]
    tot = {"old": dict.fromkeys(keys, 0), "new": dict.fromkeys(keys, 0)}
    n_gt = n_det_once = 0
    cache = json.load(open(a.cache)) if a.cache and os.path.exists(a.cache) else {}
    print(f"{'photo':<26}{'labelled':>9}{'frames':>7}{'method':>7}" + "".join(f"{k:>11}" for k in keys))
    for i, name in enumerate(sorted(f for f in os.listdir(a.images) if f in byname)):
        img, gts = load(os.path.join(a.images, name), coco["annotations"], byname[name]["id"])
        t0 = time.time()
        key = f"{name}|{a.camera}|{a.alt}|{a.speed}|{a.fps}|{a.frame_frac}|{a.overlap}|{a.gps}|{a.gps_tau}|{a.conf}|{i}"
        if key in cache:
            frames, s = cache[key]
        else:
            frames, s = cache[key] = fly_photo(img, gts, model, a, np.random.default_rng(i))
        seen = {d["item"] for f in frames for d in f["dets"] if d["item"] is not None}
        n_gt += len(gts)
        n_det_once += len(seen)
        old = sl.gps_merge(frames, a.camera, merge_m=3.0, min_hits=a.min_sightings)
        d = tempfile.mkdtemp()
        tr = sl.retrack(frames, d, a.camera, min_hits=a.min_sightings, merge_m=a.merge_m, nested=a.nested)
        new = [[tag for o in p["tracks"] for tag in o.tags] or [None] for p in tr.pieces.items]
        shutil.rmtree(d, ignore_errors=True)
        for meth, groups in (("old", [g["tags"] for g in old]), ("new", new)):
            sc = track_sim.score(groups, frames, a.min_sightings)
            for k in keys:
                tot[meth][k] += sc[k]
            print(f"{name[:25]:<26}{len(gts):>9}{len(frames):>7}{meth:>7}" + "".join(f"{sc[k]:>11}" for k in keys))
        print(f"{'':<26}  ({time.time() - t0:.0f} s, {s * 1000:.1f} mm per photo pixel, "
              f"{len(seen)}/{len(gts)} labelled pieces detected at least once)")
    if a.cache:
        json.dump(cache, open(a.cache, "w"))
    for meth in ("old", "new"):
        print(f"{'ALL':<26}{n_gt:>9}{'':>7}{meth:>7}" + "".join(f"{tot[meth][k]:>11}" for k in keys))
    print(f"YOLO detected {n_det_once} of {n_gt} labelled pieces at least once. findable = detected in >= "
          f"{a.min_sightings} frames; found = got its own map entry; lost = findable but no entry; false = entry with no real piece")


if __name__ == "__main__":
    main()
