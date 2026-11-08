#!/usr/bin/env python3
"""Survey logger for the Pi on the drone.

camera -> YOLO -> drone position -> distinct litter pieces with GPS, saved for upload.
The drone flies the path (QGroundControl / Mission Planner); this script only listens to it.

  # real flight: read position from the flight controller over MAVLink
  python survey_logger.py --mavlink /dev/ttyAMA0 --baud 57600
  # bench test without a drone: fixed fake position "lat,lon,height_m,heading_deg"
  python survey_logger.py --sim 40.7402,-73.8410,4.6,0
  # check the map math without a camera
  python survey_logger.py --selftest

Each run writes ~/surveys/<date_time>/
  survey.db          SQLite: one row per distinct piece (lat, lon, sightings, confidence, crop, confirmed, uploaded)
  pieces.geojson     confirmed pieces, for viewing on a map
  crops/<id>.jpg     best close-up of each piece (from the full-resolution frame)
  frames/            a full frame every --save-every seconds (training data for later)
  detections.jsonl   every frame: time, drone position, boxes, camera exposure (for debugging/replay)
  log.txt
"""
import argparse, collections, json, logging, math, os, shutil, signal, sqlite3, threading, time, uuid
from dataclasses import dataclass, asdict

import numpy as np

R_EARTH = 6378137.0
# Field of view (horizontal, vertical) in degrees for the full 16:9 view.
CAMERAS = {"cm3": (66.0, 41.0), "cm3wide": (102.0, 67.0)}
CAPTURE = (2304, 1296)   # full field of view, 2x2 binned: used for crops and saved frames
DETECT = (1280, 720)     # what YOLO gets
LIVE_SIZE = (640, 360)   # live view for the app: every camera frame, shrunk by the camera's own hardware


@dataclass
class Pose:
    t: float
    lat: float
    lon: float
    alt: float          # metres above takeoff
    roll: float = 0.0   # radians
    pitch: float = 0.0
    yaw: float = 0.0    # radians, 0 = north, clockwise


# ---------------------------------------------------------------- map math
def pixel_to_ground(u, v, W, H, hfov, vfov, pose, mount_yaw_deg=0.0):
    """Where pixel (u, v) of a W x H frame lands on flat ground, as (lat, lon, north_m, east_m).
    Camera points straight down with the top of the image toward the drone's nose (mount_yaw rotates that)."""
    fx = (W / 2) / math.tan(math.radians(hfov) / 2)
    fy = (H / 2) / math.tan(math.radians(vfov) / 2)
    xn, yn = (u - W / 2) / fx, (v - H / 2) / fy
    d = np.array([-yn, xn, 1.0])                    # body frame: x forward, y right, z down
    if mount_yaw_deg:
        m = math.radians(mount_yaw_deg)
        d = np.array([[math.cos(m), -math.sin(m), 0], [math.sin(m), math.cos(m), 0], [0, 0, 1]]) @ d
    cr, sr = math.cos(pose.roll), math.sin(pose.roll)
    cp, sp = math.cos(pose.pitch), math.sin(pose.pitch)
    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    n = Rz @ Ry @ Rx @ d                            # north, east, down
    if n[2] < 0.2 or pose.alt <= 0:                 # ray too close to the horizon
        return None
    s = pose.alt / n[2]
    north, east = s * n[0], s * n[1]
    lat = pose.lat + math.degrees(north / R_EARTH)
    lon = pose.lon + math.degrees(east / (R_EARTH * math.cos(math.radians(pose.lat))))
    return lat, lon, north, east


def metres_between(lat1, lon1, lat2, lon2):
    k = math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(math.radians(lat2 - lat1) * R_EARTH, math.radians(lon2 - lon1) * R_EARTH * k)


# ---------------------------------------------------------------- drone position
class Telemetry(threading.Thread):
    def __init__(self, a, log):
        super().__init__(daemon=True)
        self.a, self.log = a, log
        self.hist = collections.deque(maxlen=500)
        self.lock = threading.Lock()
        self.stop = False
        self.status = "starting"
        self._att = (0.0, 0.0, 0.0)

    def _push(self, p):
        with self.lock:
            self.hist.append(p)

    def at(self, t, max_gap=0.5):
        """Drone pose closest in time to t, or None if there's none within max_gap seconds."""
        with self.lock:
            h = list(self.hist)
        if not h:
            return None
        p = min(h, key=lambda q: abs(q.t - t))
        return p if abs(p.t - t) <= max_gap else None

    def run(self):
        if self.a.sim:
            lat, lon, alt, hdg = map(float, self.a.sim.split(","))
            self.status = "sim"
            while not self.stop:
                self._push(Pose(time.time(), lat, lon, alt, 0.0, 0.0, math.radians(hdg)))
                time.sleep(0.1)
            return
        from pymavlink import mavutil
        while not self.stop:
            try:
                self.status = "connecting"
                m = mavutil.mavlink_connection(self.a.mavlink, baud=self.a.baud, source_system=254)
                if not m.wait_heartbeat(timeout=30):
                    raise RuntimeError("no heartbeat from the flight controller")
                self.log.info("MAVLink connected (system %s)", m.target_system)
                for sid in (mavutil.mavlink.MAV_DATA_STREAM_POSITION, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
                    m.mav.request_data_stream_send(m.target_system, m.target_component, sid, 10, 1)
                self.status, last = "ok", time.time()
                while not self.stop:
                    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=True, timeout=1)
                    if msg is None:
                        if time.time() - last > 5:
                            raise RuntimeError("no position for 5 s")
                        continue
                    last = time.time()
                    if msg.get_type() == "ATTITUDE":
                        self._att = (msg.roll, msg.pitch, msg.yaw)
                    else:
                        r, p, y = self._att
                        self._push(Pose(last, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0, r, p, y))
            except Exception as e:
                self.status = f"lost ({e})"
                self.log.warning("MAVLink: %s; retrying", e)
                time.sleep(2)


# ---------------------------------------------------------------- camera
class Camera(threading.Thread):
    """Captures continuously in its own thread; the detector always takes the newest frame."""

    def __init__(self, a, log):
        super().__init__(daemon=True)
        self.a, self.log = a, log
        self.lock = threading.Lock()
        self.latest, self.seq, self.stop, self.status = None, 0, False, "starting"
        self.want_main = threading.Event()   # the detector sets this when it's ready for a new full frame
        self.want_main.set()
        self.on_live = None                  # called with every small live frame (YUV420) and its time

    def _open(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        cfg = cam.create_video_configuration(main={"size": CAPTURE, "format": "RGB888"}, lores={"size": LIVE_SIZE, "format": "YUV420"},
                                             sensor={"output_size": CAPTURE},
                                             controls={"FrameDurationLimits": (int(1e6 / self.a.camfps),) * 2}, buffer_count=4)
        cam.configure(cfg)
        cam.start()
        avail, ctrls = cam.camera_controls, {}
        if self.a.shutter_us:   # fast shutter: fixed exposure time, the camera raises gain to compensate
            if "ExposureTimeMode" in avail:
                ctrls.update(ExposureTimeMode=1, ExposureTime=self.a.shutter_us, AnalogueGainMode=0)
            else:
                ctrls.update(ExposureTime=self.a.shutter_us)
        if self.a.focus_m and "AfMode" in avail:   # locked focus (no hunting mid-flight); lens position is in dioptres
            ctrls.update(AfMode=0, LensPosition=1.0 / self.a.focus_m)
        if ctrls:
            cam.set_controls(ctrls)
        self.log.info("Camera started %sx%s at %s fps, controls %s", *CAPTURE, self.a.camfps, ctrls)
        return cam

    def run(self):
        while not self.stop:
            cam = None
            try:
                cam = self._open()
                self.status = "ok"
                while not self.stop:
                    req = cam.capture_request()
                    t = time.time()
                    try:
                        md = req.get_metadata()
                        live = req.make_array("lores") if self.on_live else None
                        # copying the 3 MP frame is the expensive part, so only do it when the detector is ready
                        arr = req.make_array("main") if self.want_main.is_set() else None   # BGR order
                    finally:
                        req.release()
                    if live is not None:
                        self.on_live(live, t)
                    if arr is not None:
                        self.want_main.clear()
                        with self.lock:
                            self.latest, self.seq = (arr, t, md), self.seq + 1
            except Exception as e:
                self.status = f"error ({e})"
                self.log.warning("Camera: %s; restarting", e)
                time.sleep(2)
            finally:
                if cam is not None:
                    try:
                        cam.stop(); cam.close()
                    except Exception:
                        pass


# ---------------------------------------------------------------- distinct pieces
class Pieces:
    """Merges sightings that land within merge_m of each other into one piece.
    A piece is 'confirmed' once it has been seen min_sightings times."""

    def __init__(self, run_dir, merge_m, min_sightings, log):
        self.dir, self.merge_m, self.min_s, self.log = run_dir, merge_m, min_sightings, log
        self.items = []
        self.db = sqlite3.connect(os.path.join(run_dir, "survey.db"), check_same_thread=False, timeout=10)
        self.db.execute("PRAGMA journal_mode=WAL")   # the mission server reads and the uploader writes while we fly
        # fb_status: pending -> new (added to Firebase) or known (already there from an earlier flight)
        self.db.execute("""CREATE TABLE IF NOT EXISTS pieces (id TEXT PRIMARY KEY, lat REAL, lon REAL, sightings INTEGER,
                           max_conf REAL, first_seen REAL, last_seen REAL, crop TEXT, confirmed INTEGER, uploaded INTEGER DEFAULT 0,
                           fb_status TEXT DEFAULT 'pending', fb_id TEXT)""")
        self.db.commit()

    def add(self, lat, lon, conf, t, crop):
        best, bd = None, self.merge_m
        for it in self.items:
            dist = metres_between(lat, lon, it["lat"], it["lon"])
            if dist <= bd:
                best, bd = it, dist
        if best is None:
            best = {"id": uuid.uuid4().hex[:12], "lat": lat, "lon": lon, "w": conf, "sightings": 1, "max_conf": conf,
                    "first_seen": t, "last_seen": t, "crop": None, "confirmed": False}
            self.items.append(best)
            status = "new"
        else:
            w = best["w"]
            best["lat"] = (best["lat"] * w + lat * conf) / (w + conf)   # confidence-weighted average position
            best["lon"] = (best["lon"] * w + lon * conf) / (w + conf)
            best["w"] += conf
            best["sightings"] += 1
            best["last_seen"] = t
            status = "seen"
        if crop is not None and (best["crop"] is None or conf >= best["max_conf"]):
            import cv2
            best["crop"] = f"crops/{best['id']}.jpg"
            cv2.imwrite(os.path.join(self.dir, best["crop"]), crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
        best["max_conf"] = max(best["max_conf"], conf)
        if not best["confirmed"] and best["sightings"] >= self.min_s:
            best["confirmed"] = True
            status = "confirmed"
            self.log.info("Confirmed piece %s at %.6f, %.6f (conf %.2f)", best["id"], best["lat"], best["lon"], best["max_conf"])
        # update in place so the uploader's columns (uploaded, fb_status, fb_id) are kept
        self.db.execute("""INSERT INTO pieces (id, lat, lon, sightings, max_conf, first_seen, last_seen, crop, confirmed)
                           VALUES (?,?,?,?,?,?,?,?,?)
                           ON CONFLICT(id) DO UPDATE SET lat=excluded.lat, lon=excluded.lon, sightings=excluded.sightings,
                             max_conf=excluded.max_conf, last_seen=excluded.last_seen, crop=excluded.crop, confirmed=excluded.confirmed""",
                        (best["id"], best["lat"], best["lon"], best["sightings"], best["max_conf"], best["first_seen"],
                         best["last_seen"], best["crop"], int(best["confirmed"])))
        self.db.commit()   # visible to the mission server / uploader right away
        return best, status

    def commit(self):
        self.db.commit()

    def write_geojson(self):
        feats = [{"type": "Feature", "geometry": {"type": "Point", "coordinates": [it["lon"], it["lat"]]},
                  "properties": {k: it[k] for k in ("id", "sightings", "max_conf", "first_seen", "last_seen", "crop")}}
                 for it in self.items if it["confirmed"]]
        json.dump({"type": "FeatureCollection", "features": feats}, open(os.path.join(self.dir, "pieces.geojson"), "w"))


class Recorder(threading.Thread):
    """Writes an annotated video (boxes, confidence, piece id, status bar) in its own thread.
    If it falls behind, frames are dropped from the video rather than slowing detection."""

    def __init__(self, path, fps, log):
        super().__init__(daemon=True)
        import queue
        self.q, self.path, self.fps, self.log = queue.Queue(maxsize=8), path, fps, log
        self.dropped = self.written = 0

    def put(self, small, dets, sx, sy, hud):
        try:
            self.q.put_nowait((small, dets, sx, sy, hud))
        except Exception:
            self.dropped += 1

    def run(self):
        import cv2
        vw = None
        while True:
            item = self.q.get()
            if item is None:
                break
            img = draw_overlay(*item)
            if vw is None:
                h, w = img.shape[:2]
                vw = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            vw.write(img)
            self.written += 1
        if vw is not None:
            vw.release()
        self.log.info("Video saved: %s (%d frames, %d skipped)", self.path, self.written, self.dropped)


def draw_overlay(img, dets, sx, sy, hud):
    """Boxes (colour = map status), labels and a status bar on a copy of the YOLO-sized frame."""
    import cv2
    img = img.copy()
    k = img.shape[1] / 1280   # text and lines scale with the image width
    colors = {"new": (0, 200, 255), "seen": (255, 160, 40), "confirmed": (60, 220, 60)}
    for d in dets:
        x1, y1, x2, y2 = (int(d["box"][0] / sx), int(d["box"][1] / sy), int(d["box"][2] / sx), int(d["box"][3] / sy))
        col = colors.get(d.get("status"), (60, 60, 255))   # red = not placed on the map (no position / too low)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        label = f"{d['conf']:.2f}" + (f" {d['piece'][:4]} {d['status']}" if "piece" in d else "")
        cv2.putText(img, label, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, max(0.4, 0.6 * k), (0, 0, 0), 4)
        cv2.putText(img, label, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, max(0.4, 0.6 * k), col, 2 if k > 0.7 else 1)
    if hud:
        bar = max(16, int(24 * k))
        cv2.rectangle(img, (0, 0), (img.shape[1], bar), (0, 0, 0), -1)
        cv2.putText(img, hud, (6, int(bar * 0.7)), cv2.FONT_HERSHEY_SIMPLEX, max(0.35, 0.5 * k), (255, 255, 255), 1)
    return img


class LiveWriter(threading.Thread):
    """Live view for the app: every small camera frame (up to 30 fps) with the newest boxes drawn on,
    kept as a JPEG on disk (in RAM under /dev/shm). Boxes update whenever YOLO finishes (~5 per second)."""

    def __init__(self, path, max_fps=30.0, video_path=None):
        super().__init__(daemon=True)
        self.path, self.min_gap, self.fps = path, 1.0 / max_fps, max_fps
        self.frame, self.cv, self.stop, self.last = None, threading.Condition(), False, 0.0
        self.dets, self.hud = [], ""
        self.video_path, self.vw, self.t0, self.written = video_path, None, None, 0   # 30 fps replay saved on the Pi

    def close(self):
        self.stop = True
        with self.cv:
            self.cv.notify()
        self.join(timeout=10)
        if self.vw is not None:
            self.vw.release()

    def put_frame(self, yuv, t):       # from the camera thread: keep only the newest
        with self.cv:
            self.frame = yuv
            self.cv.notify()

    def set_overlay(self, dets, hud):  # from the detector: boxes are in full-frame (CAPTURE) pixels
        self.dets, self.hud = dets, hud

    def run(self):
        import cv2
        sx, sy = CAPTURE[0] / LIVE_SIZE[0], CAPTURE[1] / LIVE_SIZE[1]
        while not self.stop:
            with self.cv:
                while self.frame is None and not self.stop:
                    self.cv.wait(timeout=1)
                yuv, self.frame = self.frame, None
            if yuv is None:
                continue
            wait = self.min_gap - (time.time() - self.last)
            if wait > 0:
                time.sleep(wait)
            img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            img = draw_overlay(img, self.dets, sx, sy, self.hud)
            if self.video_path:   # keep real-time pacing: repeat a frame if the camera/encoder fell behind
                now = time.time()
                if self.vw is None:
                    h, w = img.shape[:2]
                    self.vw = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
                    self.t0 = now
                target = int((now - self.t0) * self.fps) + 1
                while self.written < target:
                    self.vw.write(img)
                    self.written += 1
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                tmp = self.path + ".tmp"
                with open(tmp, "wb") as f:
                    f.write(buf.tobytes())
                os.replace(tmp, self.path)   # readers never see a half-written file
            self.last = time.time()


def crop_box(frame, x1, y1, x2, y2, pad=0.6, min_side=96):
    H, W = frame.shape[:2]
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    side = max(min_side, (x2 - x1) * (1 + pad), (y2 - y1) * (1 + pad))
    a, b = int(max(0, cx - side / 2)), int(max(0, cy - side / 2))
    return frame[b:int(min(H, cy + side / 2)), a:int(min(W, cx + side / 2))].copy()


# ---------------------------------------------------------------- self test (no camera needed)
def selftest():
    import tempfile
    W, H = CAPTURE
    hfov, vfov = CAMERAS["cm3"]
    p = Pose(0, 40.0, -73.0, 4.6)
    c = pixel_to_ground(W / 2, H / 2, W, H, hfov, vfov, p)
    assert abs(c[2]) < 1e-6 and abs(c[3]) < 1e-6, "centre of frame should be straight below"
    r = pixel_to_ground(W, H / 2, W, H, hfov, vfov, p)
    want = 4.6 * math.tan(math.radians(hfov / 2))
    assert abs(r[3] - want) < 0.01 and abs(r[2]) < 1e-6, "right edge should be east when heading north"
    t = pixel_to_ground(W / 2, 0, W, H, hfov, vfov, Pose(0, 40.0, -73.0, 4.6, yaw=math.radians(90)))
    assert abs(t[3] - 4.6 * math.tan(math.radians(vfov / 2))) < 0.01, "top of frame should point along the heading"
    tilt = pixel_to_ground(W / 2, H / 2, W, H, hfov, vfov, Pose(0, 40.0, -73.0, 4.6, pitch=math.radians(-15)))
    assert abs(tilt[2] + 4.6 * math.tan(math.radians(15))) < 0.01, "nose-down tilt should move the centre point backward"
    print(f"map math ok: frame covers {2 * want:.2f} m across at 4.6 m; 15 deg tilt shifts the centre {abs(tilt[2]):.2f} m")
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "crops"))
    pc = Pieces(d, 3.0, 3, logging.getLogger("t"))
    for i in range(5):    # one piece seen 5 times with ~1 m of GPS jitter
        pc.add(40.0 + (i % 2) * 9e-6, -73.0, 0.6, i, None)
    pc.add(40.0 + 9e-5, -73.0, 0.6, 9, None)   # another piece 10 m away
    assert len(pc.items) == 2 and pc.items[0]["confirmed"] and not pc.items[1]["confirmed"]
    print("merging ok: 5 nearby sightings -> 1 confirmed piece; a sighting 10 m away -> separate, unconfirmed")
    shutil.rmtree(d)


# ---------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mavlink", help="flight controller connection, e.g. /dev/ttyAMA0 or udpin:0.0.0.0:14550")
    src.add_argument("--sim", help="fake fixed position 'lat,lon,height_m,heading_deg' for bench tests")
    src.add_argument("--selftest", action="store_true")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--model", default=os.path.expanduser("~/yolo/all1280_rect_ncnn_model"))
    ap.add_argument("--imgsz", type=int, nargs=2, default=[736, 1280])
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--camera", choices=CAMERAS, default="cm3")
    ap.add_argument("--mount-yaw", type=float, default=0.0, help="degrees the image top is rotated from the drone's nose")
    ap.add_argument("--camfps", type=float, default=30, help="camera frame rate (the live view gets every frame)")
    ap.add_argument("--shutter-us", type=int, default=1000, help="exposure time in microseconds (1000 = 1/1000 s); 0 = auto")
    ap.add_argument("--focus-m", type=float, default=4.6, help="lock focus at this distance in metres; 0 = autofocus")
    ap.add_argument("--min-alt", type=float, default=2.0, help="ignore detections below this height (takeoff/landing)")
    ap.add_argument("--max-alt", type=float, default=30.0)
    ap.add_argument("--merge-m", type=float, default=3.0, help="sightings closer than this are the same piece")
    ap.add_argument("--min-sightings", type=int, default=3)
    ap.add_argument("--save-every", type=float, default=2.0, help="save a full frame every N seconds (0 = off)")
    ap.add_argument("--record", action="store_true", help="save video.mp4 with the boxes drawn, for checking tests")
    ap.add_argument("--record-fps", type=float, default=5.0, help="playback speed of the video (match the detection rate)")
    ap.add_argument("--live", default="", help="keep the newest annotated frame as a small JPEG at this path (for the app's live view)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = run until Ctrl+C)")
    ap.add_argument("--out", default=os.path.expanduser("~/surveys"))
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not (a.mavlink or a.sim):
        ap.error("give --mavlink for a real flight or --sim for a bench test")

    import cv2
    from ultralytics import YOLO
    run_dir = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S"))
    for sub in ("crops", "frames"):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        handlers=[logging.FileHandler(os.path.join(run_dir, "log.txt")), logging.StreamHandler()])
    log = logging.getLogger("survey")
    log.info("Run folder %s", run_dir)
    json.dump(vars(a), open(os.path.join(run_dir, "settings.json"), "w"), indent=2)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    model = YOLO(a.model, task="detect")
    model(np.zeros((DETECT[1], DETECT[0], 3), np.uint8), imgsz=a.imgsz, verbose=False)   # warm-up
    tel, cam = Telemetry(a, log), Camera(a, log)
    tel.start(); cam.start()
    pieces = Pieces(run_dir, a.merge_m, a.min_sightings, log)
    jl = open(os.path.join(run_dir, "detections.jsonl"), "a")
    hfov, vfov = CAMERAS[a.camera]
    live = None
    if a.live:
        live = LiveWriter(a.live, video_path=os.path.join(run_dir, "live_30fps.mp4"))
        live.start()
        cam.on_live = live.put_frame
    rec = None
    if a.record:
        rec = Recorder(os.path.join(run_dir, "video.mp4"), a.record_fps, log)
        rec.start()

    t0 = t_stat = time.time()
    t_save, last_seq = 0.0, -1
    n_frames = n_boxes = n_nopose = 0
    win = collections.deque(maxlen=50)
    try:
        while not stop.is_set() and not (a.seconds and time.time() - t0 > a.seconds):
            with cam.lock:
                item, seq = cam.latest, cam.seq
            if item is None or seq == last_seq:
                time.sleep(0.003)
                continue
            last_seq = seq
            frame, t, md = item
            H, W = frame.shape[:2]
            pose = tel.at(t)
            flying = pose is not None and a.min_alt <= pose.alt <= a.max_alt

            small = cv2.resize(frame, DETECT, interpolation=cv2.INTER_AREA)
            r = model(small, imgsz=a.imgsz, conf=a.conf, verbose=False)[0]
            sx, sy = W / DETECT[0], H / DETECT[1]
            dets = []
            for (x1, y1, x2, y2), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
                X1, Y1, X2, Y2 = x1 * sx, y1 * sy, x2 * sx, y2 * sy
                d = {"box": [round(X1), round(Y1), round(X2), round(Y2)], "conf": round(c, 3)}
                if flying:
                    g = pixel_to_ground((X1 + X2) / 2, (Y1 + Y2) / 2, W, H, hfov, vfov, pose, a.mount_yaw)
                    if g:
                        it, st = pieces.add(g[0], g[1], c, t, crop_box(frame, X1, Y1, X2, Y2))
                        d.update(lat=round(g[0], 7), lon=round(g[1], 7), piece=it["id"], status=st)
                dets.append(d)
            cam.want_main.set()   # ready for the next full frame
            n_frames += 1
            n_boxes += len(dets)
            n_nopose += pose is None
            win.append(time.time())
            jl.write(json.dumps({"t": round(t, 3), "pose": asdict(pose) if pose else None, "dets": dets,
                                 "exposure_us": md.get("ExposureTime"), "gain": round(md.get("AnalogueGain", 0), 2)}) + "\n")
            if rec or live:
                fps_now = (len(win) - 1) / (win[-1] - win[0]) if len(win) > 1 else 0
                where = f"{pose.lat:.6f},{pose.lon:.6f} h={pose.alt:.1f}m hdg={math.degrees(pose.yaw) % 360:.0f}" if pose else "NO POSITION"
                hud = (f"{time.strftime('%H:%M:%S', time.localtime(t))}  {fps_now:.1f} fps  {where}  "
                       f"pieces {sum(i['confirmed'] for i in pieces.items)}/{len(pieces.items)}  exp {md.get('ExposureTime')}us")
                if rec:
                    rec.put(small, dets, sx, sy, hud)
                if live:
                    live.set_overlay(dets, hud)

            if a.save_every and t - t_save >= a.save_every and shutil.disk_usage(run_dir).free > 1e9:
                cv2.imwrite(os.path.join(run_dir, "frames", f"{t:.2f}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                t_save = t

            if time.time() - t_stat >= 5:
                t_stat = time.time()
                fps = (len(win) - 1) / (win[-1] - win[0]) if len(win) > 1 else 0
                conf_n = sum(it["confirmed"] for it in pieces.items)
                pos = f"{pose.lat:.6f},{pose.lon:.6f} h={pose.alt:.1f}m" if pose else "no position"
                log.info("%.1f fps | frames %d | boxes %d | pieces %d confirmed / %d seen | camera %s exp %sus gain %.1f | drone %s (%s)",
                         fps, n_frames, n_boxes, conf_n, len(pieces.items), cam.status, md.get("ExposureTime"),
                         md.get("AnalogueGain", 0), pos, tel.status)
                jl.flush()
                pieces.commit()
                pieces.write_geojson()
    finally:
        tel.stop = cam.stop = True
        if live:
            live.close()   # finishes the 30 fps replay file
        if rec:
            rec.q.put(None)
            rec.join(timeout=30)
        pieces.commit()
        pieces.write_geojson()
        jl.close()
        el = time.time() - t0
        log.info("Done: %.0f s, %d frames (%.1f fps), %d boxes, %d frames without position, %d confirmed pieces of %d seen",
                 el, n_frames, n_frames / max(el, 1e-9), n_boxes, n_nopose,
                 sum(it["confirmed"] for it in pieces.items), len(pieces.items))


if __name__ == "__main__":
    main()
