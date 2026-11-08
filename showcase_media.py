"""Showcase media from our litter model on UAVVaste drone photos the model never trained on (test split).

  media/drone-camera.mp4   mock flight: a 16:9 window glides over each photo (ground moving down, like flying
                           forward); the model runs on every frame, boxes + HUD drawn like the drone's recording
  media/find-1..4.jpg      close-ups of the most confident finds

Same settings as the drone: frames resized to 1280x720, imgsz 736x1280, confidence 0.35.
Run:  PYTHONPATH=~/litterbug/.pylib ~/unslop-rmbg-eval/venv/bin/python showcase_media.py
"""
import json, os, subprocess, sys
import cv2, numpy as np
from ultralytics import YOLO

ROOT = os.path.expanduser("~/datasets/uavvaste_src/UAVVaste")
OUT = os.path.expanduser("~/litterbug-site/media")
MODEL = os.path.expanduser("~/litter-model/full1280_all/best.pt")
DETECT, IMGSZ, CONF = (1280, 720), (736, 1280), 0.35
FPS, SECONDS_PER_PHOTO, N_PHOTOS = 24, 6, 4
ORANGE, WHITE, INK = (61, 138, 255), (255, 255, 255), (48, 53, 59)

model = YOLO(MODEL)
# EXIF rotation must NOT be applied: UAVVaste labels are on the raw pixels.
read = lambda name: cv2.imread(os.path.join(ROOT, "images", name), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
test = json.load(open(os.path.join(ROOT, "annotations", "train_val_test_distribution_file.json")))["test"]


def detect(img):
    """Boxes in img's pixels, the drone's way (shrink to 1280x720 first)."""
    h, w = img.shape[:2]
    small = cv2.resize(img, DETECT, interpolation=cv2.INTER_AREA)
    r = model(small, imgsz=IMGSZ, conf=CONF, verbose=False)[0]
    sx, sy = w / DETECT[0], h / DETECT[1]
    return [((x1 * sx, y1 * sy, x2 * sx, y2 * sy), c) for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())]


# 1. Score every test photo on the whole frame; keep ones with several confident finds.
scored = []
for i, name in enumerate(test):
    img = read(name)
    if img is None or img.shape[1] < img.shape[0]:
        continue   # landscape only, like the drone camera
    dets = detect(img)
    good = [d for d in dets if d[1] >= 0.5]
    scored.append((len(good) + sum(c for _, c in good), name, dets))
    print(f"\r{i + 1}/{len(test)} photos", end="", flush=True)
print()
scored.sort(reverse=True)
picks = [s for s in scored if 2 <= len([d for d in s[2] if d[1] >= 0.5]) <= 12][:N_PHOTOS]
print("video photos:", [p[1] for p in picks])

# 2. Close-ups: the most confident finds, at most one per photo, square with some ground around them.
crops = []
for _, name, dets in scored[:40]:
    img = read(name)
    (x1, y1, x2, y2), c = max(dets, key=lambda d: d[1]) if dets else ((0, 0, 0, 0), 0)
    if c < 0.6:
        continue
    cx, cy, side = (x1 + x2) / 2, (y1 + y2) / 2, max(x2 - x1, y2 - y1) * 2.2
    side = max(side, 260)
    h, w = img.shape[:2]
    X1, Y1 = int(max(0, min(w - side, cx - side / 2))), int(max(0, min(h - side, cy - side / 2)))
    crop = img[Y1:Y1 + int(side), X1:X1 + int(side)]
    if crop.shape[0] < 150 or crop.shape[0] != crop.shape[1]:
        continue
    crops.append((c, name, cv2.resize(crop, (640, 640), interpolation=cv2.INTER_AREA)))
crops.sort(key=lambda t: -t[0])
for k, (c, name, crop) in enumerate(crops[:4], 1):
    cv2.imwrite(os.path.join(OUT, f"find-{k}.jpg"), crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(f"find-{k}.jpg  {c:.2f}  {name}")


# 3. Mock flight video.
def label(frame, text, x, y, bg, fg=WHITE, scale=0.6):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, scale, 1)
    cv2.rectangle(frame, (x, y - th - 10), (x + tw + 12, y), bg, -1)
    cv2.putText(frame, text, (x + 6, y - 6), cv2.FONT_HERSHEY_DUPLEX, scale, fg, 1, cv2.LINE_AA)


W, H = DETECT
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
                       "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "24",
                       "-pix_fmt", "yuv420p", "-movflags", "+faststart", os.path.join(OUT, "drone-camera.mp4")],
                      stdin=subprocess.PIPE)
found, seen, t = 0, set(), 0.0
for p_i, (_, name, full_dets) in enumerate(picks):
    img = read(name)
    h, w = img.shape[:2]
    win_w = int(w * 0.62); win_h = int(win_w * 9 / 16)
    x0 = (w - win_w) // 2
    n = FPS * SECONDS_PER_PHOTO
    for f in range(n):
        y0 = int((h - win_h) * (1 - f / (n - 1)))   # window moves up the photo = ground moves down the frame
        frame = cv2.resize(img[y0:y0 + win_h, x0:x0 + win_w], (W, H), interpolation=cv2.INTER_AREA)
        dets = detect(frame)
        for (x1, y1, x2, y2), c in dets:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), ORANGE, 3, cv2.LINE_AA)
            label(frame, f"litter {c:.2f}", int(x1), max(22, int(y1)), ORANGE, scale=0.55)
        # "pieces found": each whole-photo find counts once, when it first comes into view
        for k, ((X1, Y1, X2, Y2), c) in enumerate(full_dets):
            if c >= CONF and (p_i, k) not in seen and x0 <= (X1 + X2) / 2 <= x0 + win_w and y0 <= (Y1 + Y2) / 2 <= y0 + win_h:
                seen.add((p_i, k)); found += 1
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (W, 44), INK, -1)
        frame = cv2.addWeighted(overlay, 0.75, frame, 0.25, 0)
        mm, ss = divmod(int(t), 60)
        cv2.putText(frame, f"LITTERBUG   {mm:02d}:{ss:02d}   15 ft   2.0 m/s   pass {p_i + 1}", (14, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.7, WHITE, 1, cv2.LINE_AA)
        label(frame, f"pieces found: {found}", W - 230, 36, ORANGE, scale=0.7)
        if f < 10 and p_i:   # quick fade between passes
            frame = cv2.addWeighted(frame, f / 10, np.zeros_like(frame), 1 - f / 10, 0)
        ff.stdin.write(frame.tobytes())
        t += 1 / FPS
    print(f"pass {p_i + 1}/{len(picks)} done, {found} pieces so far")
ff.stdin.close(); ff.wait()
print("video:", os.path.join(OUT, "drone-camera.mp4"), f"{os.path.getsize(os.path.join(OUT, 'drone-camera.mp4')) / 1e6:.1f} MB")
