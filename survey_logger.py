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
import argparse, bisect, collections, json, logging, math, os, shutil, signal, sqlite3, threading, time, uuid
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
    vx: float = 0.0     # ground speed north, m/s
    vy: float = 0.0     # ground speed east, m/s


def wrap(a):
    """Angle in radians folded into -pi..pi."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def interp_pose(a, b, t):
    """Pose at time t between two readings a and b."""
    f = (t - a.t) / (b.t - a.t)
    lin = lambda x, y: x + (y - x) * f
    ang = lambda x, y: x + wrap(y - x) * f
    return Pose(t, lin(a.lat, b.lat), lin(a.lon, b.lon), lin(a.alt, b.alt), ang(a.roll, b.roll), ang(a.pitch, b.pitch),
                wrap(ang(a.yaw, b.yaw)), lin(a.vx, b.vx), lin(a.vy, b.vy))


# ---------------------------------------------------------------- map math
def body_to_ned(pose):
    """Rotation from the drone's body frame (forward, right, down) to north, east, down."""
    cr, sr = math.cos(pose.roll), math.sin(pose.roll)
    cp, sp = math.cos(pose.pitch), math.sin(pose.pitch)
    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


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
    n = body_to_ned(pose) @ d                       # north, east, down
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
        """Drone pose at time t (blended between the readings just before and after it),
        or None if there's no reading within max_gap seconds."""
        with self.lock:
            h = list(self.hist)
        if not h:
            return None
        i = bisect.bisect_left([q.t for q in h], t)
        a, b = h[max(i - 1, 0)], h[min(i, len(h) - 1)]
        if a is not b and t - max_gap <= a.t <= t <= b.t <= t + max_gap:
            return interp_pose(a, b, t)
        p = min((a, b), key=lambda q: abs(q.t - t))
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
                        r, p, y = self._att   # vx, vy: ground speed north/east in cm/s
                        self._push(Pose(last, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0, r, p, y,
                                        msg.vx / 100.0, msg.vy / 100.0))
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


# ---------------------------------------------------------------- following each piece from frame to frame
@dataclass
class Obs:
    """One box in one frame, placed on the ground."""
    lat: float
    lon: float
    north: float          # metres from the spot straight below the drone (height, tilt and heading included)
    east: float
    conf: float
    size: float           # longest side of the box, in metres on the ground
    crop: object = None   # close-up image (BGR array) or None
    tag: object = None    # passed through untouched (replay/simulation: which real item this box was)


def make_obs(box, conf, pose, W, H, hfov, vfov, mount_yaw=0.0, crop=None):
    """Box (x1, y1, x2, y2 in W x H frame pixels) -> Obs, or None if its centre doesn't land on the ground."""
    x1, y1, x2, y2 = box
    g = pixel_to_ground((x1 + x2) / 2, (y1 + y2) / 2, W, H, hfov, vfov, pose, mount_yaw)
    if g is None:
        return None
    m_per_px = 2 * pose.alt * math.tan(math.radians(hfov) / 2) / W
    return Obs(g[0], g[1], g[2], g[3], conf, max(x2 - x1, y2 - y1) * m_per_px, crop)


def drop_nested(boxes, confs, share):
    """Indexes of boxes to keep: YOLO sometimes boxes a whole piece and part of it separately. When the smaller
    of two boxes lies at least `share` inside the other, only the more confident one is kept (0 = keep all)."""
    if not share:
        return list(range(len(boxes)))
    area = lambda b: max(1e-9, (b[2] - b[0]) * (b[3] - b[1]))
    inter = lambda a, b: max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    keep = []
    for i in sorted(range(len(boxes)), key=lambda i: -confs[i]):
        if all(inter(boxes[i], boxes[j]) / min(area(boxes[i]), area(boxes[j])) < share for j in keep):
            keep.append(i)
    return sorted(keep)


def assign(cost, limit):
    """One-to-one (row, col) pairs with the lowest total cost; pairs costing >= limit are left out."""
    if cost.size == 0:
        return []
    try:
        from scipy.optimize import linear_sum_assignment   # comes with ultralytics
        rows, cols = linear_sum_assignment(cost)
        return [(i, j) for i, j in zip(rows, cols) if cost[i, j] < limit]
    except ImportError:   # cheapest pair first
        pairs, used_r, used_c = [], set(), set()
        for k in np.argsort(cost, axis=None):
            i, j = divmod(int(k), cost.shape[1])
            if cost[i, j] >= limit:
                break
            if i not in used_r and j not in used_c:
                pairs.append((i, j)); used_r.add(i); used_c.add(j)
        return pairs


class Track:
    """One piece followed through consecutive frames."""

    def __init__(self, o, t, pose, odo):
        self.id = uuid.uuid4().hex[:12]
        self.first = t
        self.hits, self.w, self.wlat, self.wlon, self.wn, self.we, self.max_conf = 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.crop, self.crop_conf = None, -1.0
        self.size = o.size
        self.piece = None    # the map entry, once confirmed
        self.active = True
        self.tags = []
        self.add(o, t, pose, odo)

    def add(self, o, t, pose, odo):
        self.last, self.north, self.east = t, o.north, o.east
        self.v, self.yaw = (pose.vx, pose.vy), pose.yaw
        self.size = 0.7 * self.size + 0.3 * o.size
        self.hits += 1
        self.w += o.conf                        # pin = confidence-weighted average of the GPS positions
        self.wlat += o.lat * o.conf
        self.wlon += o.lon * o.conf
        self.wn += (odo[0] + o.north) * o.conf  # position on the drone's own smooth path (for other passes)
        self.we += (odo[1] + o.east) * o.conf
        self.max_conf = max(self.max_conf, o.conf)
        if o.crop is not None and o.crop.size and o.conf > self.crop_conf:
            self.crop, self.crop_conf = o.crop, o.conf
        if o.tag is not None:
            self.tags.append(o.tag)

    def overlaps(self, other):
        return self.first <= other.last and other.first <= self.last


class Tracker:
    """Decides which boxes are the same physical piece.

    Every box becomes a ground offset from the drone (north/east metres; height, tilt and heading included).
    Between frames the ground slides under the drone by -velocity x time, so each open track predicts where
    its piece should be now, and this frame's boxes are paired one-to-one with the nearest predictions inside
    a gate. Two boxes in the same frame are never the same piece. GPS (which jitters 2-5 m) only sets where
    the pin goes; frame-to-frame motion from the flight controller's velocity is good to a few cm.
    A track becomes a map entry after min_hits sightings and closes when unseen for timeout_s.

    For pieces seen again on a later pass, positions are also kept on the drone's own smooth path: velocity
    added up over time, pulled gently toward GPS (ODO_TAU) so it can't drift far. Over the minute or so between
    passes that path is much steadier than GPS, whose error wanders by metres."""

    ODO_TAU = 60.0   # seconds; how slowly the smooth path is pulled toward GPS

    def __init__(self, pieces, gate_m=0.5, min_hits=3, timeout_s=1.5, log=None):
        self.pieces, self.gate_m, self.min_hits, self.timeout = pieces, gate_m, min_hits, timeout_s
        self.log = log or logging.getLogger("survey")
        self.active, self.n_tracks, self.n_dropped = [], 0, 0
        self.origin, self.odo, self.odo_t, self.odo_v = None, None, None, None

    def _odometry(self, t, pose):
        """Drone position (north, east metres from the first fix) on the smooth path."""
        if self.origin is None:
            self.origin = (pose.lat, pose.lon)
        gn = math.radians(pose.lat - self.origin[0]) * R_EARTH
        ge = math.radians(pose.lon - self.origin[1]) * R_EARTH * math.cos(math.radians(self.origin[0]))
        dt = t - self.odo_t if self.odo_t is not None else None
        if dt is None or not 0 <= dt <= 2.0:   # first fix, or a gap in the position data: restart from GPS
            self.odo = (gn, ge)
        else:
            n = self.odo[0] + (self.odo_v[0] + pose.vx) / 2 * dt
            e = self.odo[1] + (self.odo_v[1] + pose.vy) / 2 * dt
            k = min(1.0, dt / self.ODO_TAU)
            self.odo = (n + (gn - n) * k, e + (ge - e) * k)
        self.odo_t, self.odo_v = t, (pose.vx, pose.vy)
        return self.odo

    def update(self, t, pose, obs):
        """Feed one frame (pose may be None when there's no position; obs is then empty).
        Returns (id, status) per obs: status 'new' (first sighting), 'seen' (followed, not on the map yet)
        or 'confirmed' (on the map; id is the map entry's id)."""
        self.expire(t)
        if pose is None:
            return []
        odo = self._odometry(t, pose)
        if not obs:
            return []
        tracks, far = self.active, 1e9
        cost = np.full((len(tracks), len(obs)), far)
        for i, tr in enumerate(tracks):
            dt = t - tr.last
            vn, ve = (tr.v[0] + pose.vx) / 2, (tr.v[1] + pose.vy) / 2
            pn, pe = tr.north - vn * dt, tr.east - ve * dt          # ground moves opposite to the drone
            turning = 1 + min(1.0, abs(wrap(pose.yaw - tr.yaw)) / max(dt, 1e-3) / 0.5)   # wider gate in fast turns
            for j, o in enumerate(obs):
                gate = (max(self.gate_m, 1.5 * max(tr.size, o.size)) + 0.2 * math.hypot(vn, ve) * dt) * turning
                d = math.hypot(o.north - pn, o.east - pe)
                if d <= gate:
                    cost[i, j] = d / gate
        hit = [None] * len(obs)
        for i, j in assign(cost, far):
            tracks[i].add(obs[j], t, pose, odo)
            hit[j] = tracks[i]
        for j, o in enumerate(obs):
            if hit[j] is None:
                hit[j] = Track(o, t, pose, odo)
                self.active.append(hit[j])
                self.n_tracks += 1
        out = []
        for tr in hit:
            if tr.piece is None and tr.hits >= self.min_hits:
                self.pieces.confirm(tr)
            elif tr.piece is not None:
                self.pieces.save(tr.piece)
            st = "confirmed" if tr.piece else ("new" if tr.hits == 1 else "seen")
            out.append((tr.piece["id"] if tr.piece else tr.id, st))
        return out

    def expire(self, t):
        old = [tr for tr in self.active if t - tr.last > self.timeout]
        if old:
            self.active = [tr for tr in self.active if t - tr.last <= self.timeout]
            for tr in old:
                self._close(tr)

    def close_all(self):
        old, self.active = self.active, []
        for tr in old:
            self._close(tr)

    def _close(self, tr):
        tr.active = False
        if tr.piece is not None:
            self.pieces.closed(tr)
        else:
            self.n_dropped += 1   # seen fewer than min_hits times: probably a false detection
        tr.crop = None


# ---------------------------------------------------------------- map entries
class Pieces:
    """The map entries. Each confirmed track becomes one, unless it is a piece already on the map seen again
    on another pass: within merge_m on the drone's smooth path (see Tracker), nearest first, and never on
    screen at the same time as it."""

    def __init__(self, run_dir, merge_m, log):
        self.dir, self.merge_m, self.log = run_dir, merge_m, log
        self.items, self.n_resighted = [], 0
        self.db = sqlite3.connect(os.path.join(run_dir, "survey.db"), check_same_thread=False, timeout=10)
        self.db.execute("PRAGMA journal_mode=WAL")   # the mission server reads and the uploader writes while we fly
        # fb_status: pending -> new (added to Firebase) or known (already there from an earlier flight)
        self.db.execute("""CREATE TABLE IF NOT EXISTS pieces (id TEXT PRIMARY KEY, lat REAL, lon REAL, sightings INTEGER,
                           max_conf REAL, first_seen REAL, last_seen REAL, crop TEXT, confirmed INTEGER, uploaded INTEGER DEFAULT 0,
                           fb_status TEXT DEFAULT 'pending', fb_id TEXT)""")
        self.db.commit()

    def _match(self, tracks, n, e, skip=None):
        """Nearest map entry within merge_m of (n, e) on the smooth path that none of these tracks was ever on screen with."""
        best, bd = None, self.merge_m
        for p in self.items:
            if p is skip or any(o.active or any(o.overlaps(tr) for tr in tracks) for o in p["tracks"]):
                continue
            d = math.hypot(n - p["n"], e - p["e"])
            if d <= bd:
                best, bd = p, d
        return best

    def confirm(self, tr):
        p = self._match([tr], tr.wn / tr.w, tr.we / tr.w)
        if p is None:
            p = {"id": tr.id, "tracks": [], "crop": None, "crop_conf": -1.0, "confirmed": True}
            self.items.append(p)
        else:
            self.n_resighted += 1
            self.log.info("Piece %s seen again (another pass)", p["id"])
        p["tracks"].append(tr)
        tr.piece = p
        self.save(p)
        if len(p["tracks"]) == 1:
            self.log.info("Confirmed piece %s at %.6f, %.6f (conf %.2f)", p["id"], p["lat"], p["lon"], p["max_conf"])

    def closed(self, tr):
        """A track ended: with its full set of sightings, check once more whether its piece was already on the map."""
        p = tr.piece
        if any(o.active for o in p["tracks"]):
            return
        q = self._match(p["tracks"], p["n"], p["e"], skip=p)
        if q is None:
            return
        keep, gone = (q, p) if q["first_seen"] <= p["first_seen"] else (p, q)
        keep["tracks"] += gone["tracks"]
        for o in gone["tracks"]:
            o.piece = keep
        if gone["crop"]:
            src = os.path.join(self.dir, gone["crop"])
            if gone["crop_conf"] > keep["crop_conf"]:
                keep["crop"], keep["crop_conf"] = f"crops/{keep['id']}.jpg", gone["crop_conf"]
                os.replace(src, os.path.join(self.dir, keep["crop"]))
            elif os.path.exists(src):
                os.remove(src)
        self.items.remove(gone)
        self.db.execute("DELETE FROM pieces WHERE id = ?", (gone["id"],))
        self.n_resighted += 1
        self.log.info("Piece %s is %s seen again (another pass); merged", gone["id"], keep["id"])
        self.save(keep)

    def save(self, p):
        trs = p["tracks"]
        w = sum(o.w for o in trs)
        p.update(lat=sum(o.wlat for o in trs) / w, lon=sum(o.wlon for o in trs) / w,
                 n=sum(o.wn for o in trs) / w, e=sum(o.we for o in trs) / w, sightings=sum(o.hits for o in trs),
                 max_conf=max(o.max_conf for o in trs), first_seen=min(o.first for o in trs), last_seen=max(o.last for o in trs))
        best = max(trs, key=lambda o: o.crop_conf)
        if best.crop is not None and best.crop_conf > p["crop_conf"]:
            import cv2
            p["crop"], p["crop_conf"] = f"crops/{p['id']}.jpg", best.crop_conf
            cv2.imwrite(os.path.join(self.dir, p["crop"]), best.crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
        # update in place so the uploader's columns (uploaded, fb_status, fb_id) are kept
        self.db.execute("""INSERT INTO pieces (id, lat, lon, sightings, max_conf, first_seen, last_seen, crop, confirmed)
                           VALUES (?,?,?,?,?,?,?,?,1)
                           ON CONFLICT(id) DO UPDATE SET lat=excluded.lat, lon=excluded.lon, sightings=excluded.sightings,
                             max_conf=excluded.max_conf, first_seen=excluded.first_seen, last_seen=excluded.last_seen,
                             crop=excluded.crop, confirmed=1""",
                        (p["id"], p["lat"], p["lon"], p["sightings"], p["max_conf"], p["first_seen"], p["last_seen"], p["crop"]))
        self.db.commit()   # visible to the mission server right away

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


# ---------------------------------------------------------------- replay a recorded run (no camera, no drone)
def fill_velocity(frames, span=0.5):
    """Old runs logged no velocity: estimate it from the logged positions about +-span seconds around each frame."""
    ps = [(f["t"], f["pose"]) for f in frames if f.get("pose")]
    if not ps or all("vx" in p for _, p in ps):
        return
    ts = [t for t, _ in ps]
    for t, p in ps:
        a, b = ps[bisect.bisect_left(ts, t - span)][1], ps[bisect.bisect_right(ts, t + span) - 1][1]
        dt = b["t"] - a["t"]
        if dt < 0.2:
            p["vx"], p["vy"] = 0.0, 0.0
        else:
            p["vx"] = math.radians(b["lat"] - a["lat"]) * R_EARTH / dt
            p["vy"] = math.radians(b["lon"] - a["lon"]) * R_EARTH * math.cos(math.radians(p["lat"])) / dt


def frame_obs(frames, camera, mount_yaw, min_alt, max_alt, nested=0.0):
    """(t, pose or None, [Obs]) for each logged frame; boxes are in full-frame (CAPTURE) pixels."""
    hfov, vfov = CAMERAS[camera]
    fields = Pose.__dataclass_fields__
    for f in frames:
        pose = Pose(**{k: v for k, v in f["pose"].items() if k in fields}) if f.get("pose") else None
        if pose is None or not (min_alt <= pose.alt <= max_alt):
            yield f["t"], None, []
            continue
        obs, dets = [], f["dets"]
        dets = [dets[i] for i in drop_nested([d["box"] for d in dets], [d["conf"] for d in dets], nested)]
        for d in dets:
            o = make_obs(d["box"], d["conf"], pose, *CAPTURE, hfov, vfov, mount_yaw)
            if o is not None:
                o.tag = d.get("item")
                obs.append(o)
        yield f["t"], pose, obs


def retrack(frames, out_dir, camera="cm3", mount_yaw=0.0, min_alt=2.0, max_alt=30.0,
            merge_m=2.0, min_hits=3, gate_m=0.5, timeout_s=1.5, nested=0.0, log=None):
    """Runs the tracker over logged frames (rows of detections.jsonl). Returns the Tracker (.pieces = map entries)."""
    log = log or logging.getLogger("replay")
    os.makedirs(os.path.join(out_dir, "crops"), exist_ok=True)
    if os.path.exists(os.path.join(out_dir, "survey.db")):
        os.remove(os.path.join(out_dir, "survey.db"))
    fill_velocity(frames)
    tracker = Tracker(Pieces(out_dir, merge_m, log), gate_m, min_hits, timeout_s, log)
    for t, pose, obs in frame_obs(frames, camera, mount_yaw, min_alt, max_alt, nested):
        tracker.update(t, pose, obs)
    tracker.close_all()
    tracker.pieces.write_geojson()
    tracker.pieces.db.close()
    return tracker


def gps_merge(frames, camera="cm3", mount_yaw=0.0, min_alt=2.0, max_alt=30.0, merge_m=3.0, min_hits=3):
    """The old method, for comparison: every box joins the nearest piece within merge_m by GPS alone.
    Returns confirmed pieces as dicts (lat, lon, n, tags)."""
    items = []
    for t, pose, obs in frame_obs(frames, camera, mount_yaw, min_alt, max_alt):
        for o in obs:
            best, bd = None, merge_m
            for it in items:
                d = metres_between(o.lat, o.lon, it["lat"], it["lon"])
                if d <= bd:
                    best, bd = it, d
            if best is None:
                items.append({"lat": o.lat, "lon": o.lon, "w": o.conf, "n": 1, "tags": [o.tag]})
            else:
                w = best["w"]
                best["lat"], best["lon"] = (best["lat"] * w + o.lat * o.conf) / (w + o.conf), (best["lon"] * w + o.lon * o.conf) / (w + o.conf)
                best["w"] += o.conf
                best["n"] += 1
                best["tags"].append(o.tag)
    return [it for it in items if it["n"] >= min_hits]


def replay(a):
    run = a.replay
    path = run if run.endswith(".jsonl") else os.path.join(run, "detections.jsonl")
    run_dir = os.path.dirname(os.path.abspath(path))
    st = {}
    if os.path.exists(os.path.join(run_dir, "settings.json")):
        st = json.load(open(os.path.join(run_dir, "settings.json")))
    cam, my = st.get("camera", a.camera), st.get("mount_yaw", a.mount_yaw)
    lo, hi = st.get("min_alt", a.min_alt), st.get("max_alt", a.max_alt)
    frames = [json.loads(line) for line in open(path) if line.strip()]
    out = os.path.join(run_dir, "retrack")
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    tr = retrack(frames, out, cam, my, lo, hi, a.merge_m, a.min_sightings, a.gate_m, a.track_timeout, a.nested)
    old = gps_merge(frames, cam, my, lo, hi, 3.0, a.min_sightings)
    n_box = sum(len(f["dets"]) for f in frames)
    print(f"{path}: {len(frames)} frames, {n_box} boxes (camera {cam}, mount yaw {my})")
    print(f"  old (GPS within 3 m only): {len(old)} pieces")
    print(f"  new (tracking):                {len(tr.pieces.items)} pieces  "
          f"({tr.n_tracks} tracks, {tr.n_dropped} too short, {tr.pieces.n_resighted} re-sightings on other passes merged)")
    print(f"  written to {out}/ (survey.db, pieces.geojson)")


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
    # tracking: 3 pieces (two of them only 0.4 m apart), flown over northbound then southbound 30 s later,
    # with 2 m of random GPS jitter on every frame and the GPS 1.5 m off on the way back
    d = tempfile.mkdtemp()
    lg = logging.getLogger("t")
    rng = np.random.default_rng(1)
    ll = lambda n, e: (40.0 + math.degrees(n / R_EARTH), -73.0 + math.degrees(e / (R_EARTH * math.cos(math.radians(40)))))
    pcs = Pieces(d, 3.0, lg)
    trk = Tracker(pcs, 0.5, 3, 1.5, lg)
    items = [(0.0, 0.0), (0.0, 0.4), (10.0, 0.0)]

    def fly(t0, n0, vn, secs, gps_off):
        for i in range(int(secs * 5)):
            t = t0 + i * 0.2
            dn = n0 + vn * (t - t0)
            pose = Pose(t, *ll(dn, 0), 4.6, yaw=0.0 if vn > 0 else math.pi, vx=vn + rng.normal(0, 0.05))
            obs = [Obs(*ll(n + rng.normal(0, 2.0) + gps_off, e + rng.normal(0, 2.0)),
                       n - dn + rng.normal(0, 0.02), e + rng.normal(0, 0.02), 0.6, 0.15)
                   for n, e in items if abs(n - dn) < 1.7]
            for o, (_, st) in zip(obs, trk.update(t, pose, obs)):
                assert st in ("new", "seen", "confirmed")

    fly(0, -3, 2.5, 6.4, 0.0)
    fly(30, 14, -2.5, 6.4, 1.5)
    trk.close_all()
    assert len(pcs.items) == 3 and pcs.n_resighted == 3, (len(pcs.items), pcs.n_resighted, trk.n_tracks)
    rows = pcs.db.execute("SELECT COUNT(*) FROM pieces WHERE confirmed = 1").fetchone()[0]
    assert rows == 3, rows
    pcs.db.close()
    print("tracking ok: 3 pieces (two 0.4 m apart) over 2 passes with 2 m GPS jitter -> 3 map entries, "
          "each seen again on the second pass")
    shutil.rmtree(d, ignore_errors=True)


# ---------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mavlink", help="flight controller connection, e.g. /dev/ttyAMA0 or udpin:0.0.0.0:14550")
    src.add_argument("--sim", help="fake fixed position 'lat,lon,height_m,heading_deg' for bench tests")
    src.add_argument("--selftest", action="store_true")
    src.add_argument("--replay", metavar="RUN_DIR", help="re-run piece tracking on a recorded run's detections.jsonl "
                     "(writes RUN_DIR/retrack/, compares with the old GPS-only merge)")
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
    ap.add_argument("--gate-m", type=float, default=0.5,
                    help="frame to frame: a box this far (or 1.5x its size) from where a piece was predicted is a different piece")
    ap.add_argument("--nested", type=float, default=0.0,
                    help="drop the less confident of two boxes when the smaller lies this much inside the other (0 = off)")
    ap.add_argument("--track-timeout", type=float, default=1.5, help="stop following a piece after this many seconds unseen")
    ap.add_argument("--merge-m", type=float, default=2.0,
                    help="other passes: a piece within this GPS distance of one already on the map (never seen together) is the same piece")
    ap.add_argument("--min-sightings", type=int, default=3, help="frames a piece must be followed for before it goes on the map")
    ap.add_argument("--save-every", type=float, default=2.0, help="save a full frame every N seconds (0 = off)")
    ap.add_argument("--record", action="store_true", help="save video.mp4 with the boxes drawn, for checking tests")
    ap.add_argument("--record-fps", type=float, default=5.0, help="playback speed of the video (match the detection rate)")
    ap.add_argument("--live", default="", help="keep the newest annotated frame as a small JPEG at this path (for the app's live view)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = run until Ctrl+C)")
    ap.add_argument("--out", default=os.path.expanduser("~/surveys"))
    ap.add_argument("--note", default="", help="test name / notes, saved in settings.json and shown in the app")
    ap.add_argument("--no-upload", action="store_true", help="never upload this run's litter to Firebase (field tests)")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.replay:
        return replay(a)
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
    pieces = Pieces(run_dir, a.merge_m, log)
    tracker = Tracker(pieces, a.gate_m, a.min_sightings, a.track_timeout, log)
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
            dets, placed, obs = [], [], []
            xyxy, confs = r.boxes.xyxy.tolist(), r.boxes.conf.tolist()
            keep = drop_nested(xyxy, confs, a.nested)   # one box per piece
            for (x1, y1, x2, y2), c in zip([xyxy[i] for i in keep], [confs[i] for i in keep]):
                X1, Y1, X2, Y2 = x1 * sx, y1 * sy, x2 * sx, y2 * sy
                d = {"box": [round(X1), round(Y1), round(X2), round(Y2)], "conf": round(c, 3)}
                if flying:
                    o = make_obs((X1, Y1, X2, Y2), c, pose, W, H, hfov, vfov, a.mount_yaw, crop_box(frame, X1, Y1, X2, Y2))
                    if o:
                        d.update(lat=round(o.lat, 7), lon=round(o.lon, 7))
                        placed.append(d)
                        obs.append(o)
                dets.append(d)
            # which boxes are the same piece as in earlier frames (also closes pieces no longer seen)
            for d, (pid, st) in zip(placed, tracker.update(t, pose if flying else None, obs)):
                d.update(piece=pid, status=st)
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
                       f"pieces {len(pieces.items)} (following {len(tracker.active)})  exp {md.get('ExposureTime')}us")
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
                pos = f"{pose.lat:.6f},{pose.lon:.6f} h={pose.alt:.1f}m v={math.hypot(pose.vx, pose.vy):.1f}m/s" if pose else "no position"
                log.info("%.1f fps | frames %d | boxes %d | pieces %d on map, following %d | camera %s exp %sus gain %.1f | drone %s (%s)",
                         fps, n_frames, n_boxes, len(pieces.items), len(tracker.active), cam.status, md.get("ExposureTime"),
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
        tracker.close_all()   # last chance to recognise pieces seen again on another pass
        pieces.commit()
        pieces.write_geojson()
        jl.close()
        el = time.time() - t0
        log.info("Done: %.0f s, %d frames (%.1f fps), %d boxes, %d frames without position, %d pieces "
                 "(%d tracks, %d too short to count, %d re-sightings on other passes merged)",
                 el, n_frames, n_frames / max(el, 1e-9), n_boxes, n_nopose, len(pieces.items),
                 tracker.n_tracks, tracker.n_dropped, pieces.n_resighted)


if __name__ == "__main__":
    main()
