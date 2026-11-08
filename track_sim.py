#!/usr/bin/env python3
"""Fake survey flight to test piece tracking without a drone: old GPS-only merge vs the new tracker.

Scatters litter (including close pairs), flies a lawnmower path over it with turns, wind tilt and wandering GPS,
fakes YOLO boxes (with misses and false detections) and logs them like detections.jsonl. Then runs both methods
and counts, using the true item behind each box: pieces on the map, real items found, duplicates, items lost.

  python track_sim.py                 # default: 3 seeds
  python track_sim.py --seeds 10 --gps 4 --speed 3
"""
import argparse, logging, math, shutil, tempfile
from collections import Counter

import numpy as np

import survey_logger as sl


def fly(rng, a):
    W, H = sl.CAPTURE
    hfov, vfov = sl.CAMERAS[a.camera]
    fx, fy = (W / 2) / math.tan(math.radians(hfov) / 2), (H / 2) / math.tan(math.radians(vfov) / 2)
    lat0, lon0 = 40.7402, -73.8410
    k = sl.R_EARTH * math.cos(math.radians(lat0))
    to_ll = lambda n, e: (lat0 + math.degrees(n / sl.R_EARTH), lon0 + math.degrees(e / k))

    # litter: random pieces plus close pairs
    span_e = a.spacing * (a.passes - 1)
    items = [(rng.uniform(0, a.length), rng.uniform(-1, span_e + 1), rng.uniform(0.08, 0.3)) for _ in range(a.items)]
    for _ in range(a.pairs):
        n, e, gap, ang = rng.uniform(2, a.length - 2), rng.uniform(0, span_e), rng.uniform(0.4, 1.5), rng.uniform(0, math.pi)
        items += [(n, e, rng.uniform(0.08, 0.2)), (n + gap * math.cos(ang), e + gap * math.sin(ang), rng.uniform(0.08, 0.2))]

    # path: passes north/south joined by turns; positions at 50 Hz
    pts, t = [], 0.0
    dt = 0.02
    for p in range(a.passes):
        north = p % 2 == 0
        e = p * a.spacing
        for i in range(int((a.length + 6) / a.speed / dt)):
            s = -3 + i * a.speed * dt
            pts.append((t, s if north else a.length - s, e, 0.0 if north else math.pi)); t += dt
        if p < a.passes - 1:   # turn: slide one pass spacing east while rotating 180 degrees
            nt = int(a.turn_s / dt)
            for i in range(nt):
                f = i / nt
                pts.append((t, (a.length + 3) if north else -3, e + f * a.spacing, (0.0 if north else math.pi) + f * math.pi)); t += dt
    P = np.array(pts)
    vel = np.gradient(P[:, 1:3], dt, axis=0)

    # wandering GPS error (slow drift + a little fast noise), wind tilt
    gps = np.zeros((len(P), 2))
    for i in range(1, len(P)):
        gps[i] = gps[i - 1] * (1 - dt / a.gps_tau) + rng.normal(0, a.gps * math.sqrt(2 * dt / a.gps_tau), 2)
    vbias = np.zeros((len(P), 2))   # velocity error that wanders slowly (adds up along the smooth path)
    for i in range(1, len(P)):
        vbias[i] = vbias[i - 1] * (1 - dt / 30) + rng.normal(0, a.vel_err * math.sqrt(2 * dt / 30), 2)
    roll = np.radians(3) * np.sin(P[:, 0] * 0.9 + 1) + np.radians(1.5) * np.sin(P[:, 0] * 2.3)
    pitch = np.radians(3) * np.sin(P[:, 0] * 0.7) + np.radians(1.5) * np.sin(P[:, 0] * 1.9 + 2)

    frames, tf = [], 0.5
    while tf < P[-1, 0] - 0.1:
        i = int(tf / dt)
        tn, te, yaw = P[i, 1], P[i, 2], P[i, 3]
        true = sl.Pose(tf, 0, 0, a.alt, roll[i], pitch[i], yaw)
        R = sl.body_to_ned(true).T
        dets = []
        for idx, (n, e, size) in enumerate(items):
            b = R @ np.array([n - tn, e - te, a.alt])        # forward, right, down from the camera
            u, v = W / 2 + b[1] / b[2] * fx + rng.normal(0, 4), H / 2 - b[0] / b[2] * fy + rng.normal(0, 4)
            half = size * fx / b[2] / 2 * rng.uniform(0.85, 1.15)
            if half < u < W - half and half < v < H - half and rng.random() < a.detect_p:
                dets.append({"box": [u - half, v - half, u + half, v + half], "conf": float(rng.uniform(0.38, 0.9)), "item": idx})
        for _ in range(rng.poisson(a.false_rate)):          # one-off false detections
            u, v, half = rng.uniform(40, W - 40), rng.uniform(40, H - 40), rng.uniform(15, 40)
            dets.append({"box": [u - half, v - half, u + half, v + half], "conf": float(rng.uniform(0.36, 0.5)), "item": None})
        lat, lon = to_ll(tn + gps[i, 0] + rng.normal(0, 0.2), te + gps[i, 1] + rng.normal(0, 0.2))
        pose = {"t": tf, "lat": lat, "lon": lon, "alt": a.alt + rng.normal(0, 0.05),
                "roll": roll[i] + rng.normal(0, math.radians(0.5)), "pitch": pitch[i] + rng.normal(0, math.radians(0.5)),
                "yaw": sl.wrap(yaw + rng.normal(0, math.radians(1.5))),
                "vx": vel[i, 0] + vbias[i, 0] + rng.normal(0, 0.1), "vy": vel[i, 1] + vbias[i, 1] + rng.normal(0, 0.1)}
        frames.append({"t": tf, "pose": pose, "dets": dets})
        tf += 1 / a.fps + rng.uniform(-0.03, 0.03)
    return frames, items


def score(groups, frames, min_hits):
    """groups: list of tag lists (one per map entry). Uses the true item behind each box."""
    seen = Counter(d["item"] for f in frames for d in f["dets"] if d["item"] is not None)
    findable = {i for i, n in seen.items() if n >= min_hits}
    majority = [Counter(g).most_common(1)[0][0] for g in groups]
    real = [m for m in majority if m is not None]
    found = set(real)
    return {"pieces": len(groups), "found": len(found & findable), "findable": len(findable),
            "duplicates": len(real) - len(found), "false": majority.count(None), "lost": len(findable - found)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--camera", default="cm3", choices=sl.CAMERAS)
    ap.add_argument("--alt", type=float, default=4.6)
    ap.add_argument("--speed", type=float, default=2.5, help="m/s")
    ap.add_argument("--fps", type=float, default=5.0, help="detections per second")
    ap.add_argument("--length", type=float, default=30.0, help="pass length, m")
    ap.add_argument("--passes", type=int, default=5)
    ap.add_argument("--spacing", type=float, default=4.5, help="m between passes (frame is ~6 m wide at 4.6 m)")
    ap.add_argument("--turn-s", type=float, default=2.0)
    ap.add_argument("--items", type=int, default=40)
    ap.add_argument("--pairs", type=int, default=8, help="extra pairs of pieces 0.4-1.5 m apart")
    ap.add_argument("--gps", type=float, default=2.5, help="GPS error size, m")
    ap.add_argument("--gps-tau", type=float, default=8.0, help="how slowly the GPS error wanders, s")
    ap.add_argument("--vel-err", type=float, default=0.05, help="slowly wandering velocity error, m/s")
    ap.add_argument("--detect-p", type=float, default=0.85)
    ap.add_argument("--false-rate", type=float, default=0.15, help="false boxes per frame")
    ap.add_argument("--merge-m", type=float, default=2.0, help="new method only; old is always 3 m")
    ap.add_argument("--gate-m", type=float, default=0.5)
    ap.add_argument("--min-sightings", type=int, default=3)
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)

    keys = ["pieces", "found", "findable", "duplicates", "lost", "false"]
    tot = {"old": Counter(), "new": Counter()}
    print(f"{'seed':>4} {'method':<7}" + "".join(f"{k:>11}" for k in keys))
    for seed in range(a.seeds):
        frames, items = fly(np.random.default_rng(seed), a)
        old = sl.gps_merge(frames, a.camera, merge_m=3.0, min_hits=a.min_sightings)   # as it was
        d = tempfile.mkdtemp()
        tr = sl.retrack(frames, d, a.camera, merge_m=a.merge_m, min_hits=a.min_sightings, gate_m=a.gate_m)
        new = [[tag for o in p["tracks"] for tag in o.tags] or [None] for p in tr.pieces.items]
        shutil.rmtree(d, ignore_errors=True)
        for name, groups in (("old", [g["tags"] for g in old]), ("new", new)):
            s = score(groups, frames, a.min_sightings)
            tot[name].update(s)
            print(f"{seed:>4} {name:<7}" + "".join(f"{s[k]:>11}" for k in keys))
    print(f"{'all':>4} {'old':<7}" + "".join(f"{tot['old'][k]:>11}" for k in keys))
    print(f"{'all':>4} {'new':<7}" + "".join(f"{tot['new'][k]:>11}" for k in keys))
    print("found = real items on the map; duplicates = extra entries for an item; lost = item never got its own entry")


if __name__ == "__main__":
    main()
