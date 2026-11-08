#!/usr/bin/env python3
"""Mission server for the Pi on the drone.

A phone (or any browser) on the same network sends an area; the Pi plans the lawnmower path,
uploads it to the flight controller, starts the mission and runs the litter logger.

  python mission_server.py --mavlink /dev/ttyAMA0 --baud 57600       # on the drone
  python mission_server.py --mavlink udpin:0.0.0.0:14550              # e.g. telemetry over Wi-Fi / a simulator
  then open http://<pi-address>:8080 for the operator page

API (all JSON):
  GET  /api/status                 drone + mission + logger state
  POST /api/plan     {polygon:[[lat,lon],...], alt, overlap, speed, camera}   -> path preview (no drone needed)
  POST /api/upload                 send the last plan to the flight controller
  POST /api/start    {confirm:true}  arm, take off and fly the mission; starts the logger
  POST /api/pause | /api/resume | /api/rtl | /api/land
  POST /api/logger/start {sim:"lat,lon,h,hdg"} | /api/logger/stop   (manual, for bench tests)
  GET  /api/pieces                 litter found in the current/last run (GeoJSON)
  GET  /stream                     live camera view with boxes (MJPEG, ~5 fps, only while the logger runs)
  GET  /live.jpg                   newest live frame
  GET  /api/runs                   past flights / camera tests, newest first
  GET  /runs/<run>/pieces          that run's litter (GeoJSON)
  GET  /runs/<run>/video           that run's 5 fps detection recording (H.264 once converted; supports seeking)
  GET  /runs/<run>/replay          that run's smooth 30 fps replay (the live view, saved)
  GET  /runs/<run>/crops/<file>    photo of one piece

The flight controller's MAVLink stream is also forwarded to udp 127.0.0.1:14551 for the logger.
"""
import argparse, glob, json, logging, math, os, queue, re, signal, sqlite3, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_path  # noqa: E402

log = logging.getLogger("mission")
FWD = "127.0.0.1:14551"
LIVE = "/dev/shm/litterbug_live.jpg" if os.path.isdir("/dev/shm") else os.path.join(HERE, "live.jpg")


class Drone(threading.Thread):
    """Owns the MAVLink connection: keeps the latest state, forwards everything to the logger,
    and does mission upload / mode changes / arming."""

    def __init__(self, url, baud):
        super().__init__(daemon=True)
        self.url, self.baud = url, baud
        self.m = None
        self.lock = threading.Lock()          # one command sequence at a time
        self.mq = queue.Queue()               # mission-protocol replies
        self.acks = {}                        # command id -> (result, time)
        self.s = {"link": "connecting", "armed": False, "mode": None, "lat": None, "lon": None, "alt": None,
                  "heading": None, "speed": None, "gps_fix": 0, "sats": 0, "battery": None, "voltage": None,
                  "mission_seq": None, "mission_count": None, "messages": [], "last_heartbeat": 0,
                  "home_lat": None, "home_lon": None}

    def run(self):
        from pymavlink import mavutil
        self.mavutil = mavutil
        fwd = mavutil.mavlink_connection("udpout:" + FWD)
        while True:
            try:
                self.s["link"] = "connecting"
                m = mavutil.mavlink_connection(self.url, baud=self.baud, source_system=255, source_component=190)
                hb = m.wait_heartbeat(timeout=30)
                if hb is None:
                    raise RuntimeError("no heartbeat")
                self.m = m
                log.info("Flight controller connected (system %s, %s)", m.target_system, mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name)
                for sid, hz in ((mavutil.mavlink.MAV_DATA_STREAM_ALL, 4), (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
                                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)):
                    m.mav.request_data_stream_send(m.target_system, m.target_component, sid, hz, 1)
                self.s["link"] = "ok"
                t_home = 0
                while True:
                    if time.time() - t_home > 5:   # ask for the home position now and then (it's set/updated at arming)
                        t_home = time.time()
                        m.mav.command_long_send(m.target_system, m.target_component,
                                                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 242, 0, 0, 0, 0, 0, 0)
                    msg = m.recv_match(blocking=True, timeout=1)
                    if msg is None:
                        if time.time() - self.s["last_heartbeat"] > 5:
                            raise RuntimeError("heartbeat lost")
                        continue
                    t = msg.get_type()
                    if t == "BAD_DATA":
                        continue
                    try:
                        fwd.write(msg.get_msgbuf())
                    except Exception:
                        pass
                    self._handle(t, msg)
            except Exception as e:
                self.s["link"] = f"lost: {e}"
                self.m = None
                log.warning("MAVLink: %s; reconnecting", e)
                time.sleep(2)

    def _handle(self, t, msg):
        s, mu = self.s, self.mavutil.mavlink
        if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != mu.MAV_TYPE_GCS:
            s["last_heartbeat"] = time.time()
            s["armed"] = bool(msg.base_mode & mu.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = self.m.flightmode
        elif t == "GLOBAL_POSITION_INT":
            s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt=round(msg.relative_alt / 1000, 1),
                     heading=round(msg.hdg / 100) if msg.hdg != 65535 else None)
        elif t == "VFR_HUD":
            s["speed"] = round(msg.groundspeed, 1)
        elif t == "GPS_RAW_INT":
            s.update(gps_fix=msg.fix_type, sats=msg.satellites_visible)
        elif t == "SYS_STATUS":
            s.update(battery=msg.battery_remaining if msg.battery_remaining >= 0 else None, voltage=round(msg.voltage_battery / 1000, 2))
        elif t == "MISSION_CURRENT":
            s["mission_seq"] = msg.seq
        elif t == "STATUSTEXT":
            s["messages"] = (s["messages"] + [f"{time.strftime('%H:%M:%S')} {msg.text}"])[-12:]
            log.info("FC: %s", msg.text)
        elif t == "HOME_POSITION":   # where it took off / will return to
            s.update(home_lat=msg.latitude / 1e7, home_lon=msg.longitude / 1e7)
        elif t == "COMMAND_ACK":
            self.acks[msg.command] = (msg.result, time.time())
        elif t in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
            self.mq.put(msg)

    # ----- commands
    def _need(self):
        if self.m is None or self.s["link"] != "ok":
            raise RuntimeError("not connected to the flight controller")
        return self.m

    def command(self, cmd, *params, timeout=3):
        m = self._need()
        p = list(params) + [0] * (7 - len(params))
        self.acks.pop(cmd, None)
        m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cmd in self.acks:
                res = self.acks[cmd][0]
                if res != self.mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError(f"flight controller refused command {cmd} (result {res}); last message: "
                                       f"{self.s['messages'][-1] if self.s['messages'] else 'none'}")
                return
            time.sleep(0.05)
        raise RuntimeError(f"no reply to command {cmd}")

    def set_mode(self, name, timeout=3):
        m = self._need()
        mapping = m.mode_mapping() or {}
        if name not in mapping:
            raise RuntimeError(f"mode {name} not available")
        m.set_mode(mapping[name])
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.s["mode"] == name:
                return
            time.sleep(0.1)
        raise RuntimeError(f"flight controller didn't switch to {name} (still {self.s['mode']})")

    def arm(self, timeout=8):
        self.command(self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, timeout=timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.s["armed"]:
                return
            time.sleep(0.1)
        raise RuntimeError("didn't arm; check the flight controller messages (pre-arm checks)")

    def upload(self, items, home):
        """Upload mission items [(command, frame, params7)] with the MAVLink mission protocol.
        Item 0 is the home placeholder ArduPilot expects."""
        mu = self.mavutil.mavlink
        rows = [(16, 0, [0, 0, 0, 0, home[0], home[1], 0])] + list(items)
        with self.lock:
            m = self._need()
            while not self.mq.empty():
                self.mq.get_nowait()
            m.mav.mission_count_send(m.target_system, m.target_component, len(rows), mu.MAV_MISSION_TYPE_MISSION)
            sent, t_last = set(), time.time()
            while True:
                try:
                    msg = self.mq.get(timeout=1)
                except queue.Empty:
                    if time.time() - t_last > 6:
                        raise RuntimeError(f"mission upload timed out ({len(sent)}/{len(rows)} items sent)")
                    continue
                t_last = time.time()
                if msg.get_type() == "MISSION_ACK":
                    if msg.type != mu.MAV_MISSION_ACCEPTED:
                        raise RuntimeError(f"flight controller rejected the mission (code {msg.type})")
                    break
                seq = msg.seq
                cmd, frame, p = rows[seq]
                p = [0 if v is None else v for v in p]
                x, y = (int(p[4] * 1e7), int(p[5] * 1e7)) if frame in (0, 3) else (0, 0)
                m.mav.mission_item_int_send(m.target_system, m.target_component, seq, frame, cmd, 0, 1,
                                            p[0], p[1], p[2], p[3], x, y, float(p[6]), mu.MAV_MISSION_TYPE_MISSION)
                sent.add(seq)
            self.s["mission_count"] = len(rows)
            log.info("Mission uploaded: %d items", len(rows))

    def start_mission(self):
        mu = self.mavutil.mavlink
        with self.lock:
            if self.s["gps_fix"] < 3:
                raise RuntimeError(f"no 3D GPS fix yet ({self.s['sats']} satellites); wait outdoors")
            if not self.s["armed"]:
                self.set_mode("GUIDED")
                self.arm()
            self.set_mode("AUTO")
            self.command(mu.MAV_CMD_MISSION_START, 0, 0)


# ---------------------------------------------------------------- field-test settings (Settings tab on the operator page)
# Every survey_logger option the page can change. type: switch | number | optnum (switch + number; off = 0) | select | text.
# "arg" is the survey_logger flag. Saved in field_settings.json next to this file; used for every run started.
DEFAULT_MODEL = os.path.expanduser("~/yolo/all1280_rect_ncnn_model")
SETTINGS = [
    {"key": "note", "group": "This test", "label": "Test name / notes", "type": "text", "default": "", "arg": "--note",
     "help": "Saved with each run and shown under Flights, so you know what every test was."},
    {"key": "upload", "group": "This test", "label": "Upload litter to Firebase after landing", "type": "switch", "default": True,
     "help": "Off for practice and settings tests: those runs are never published, even if you turn this back on later."},

    {"key": "model", "group": "Detection", "label": "Model", "type": "select", "default": DEFAULT_MODEL, "arg": "--model",
     "help": "Which trained YOLO model runs. NCNN models are the fast ones on the Pi (~5 fps); .pt models are much slower there."},
    {"key": "conf", "group": "Detection", "label": "Confidence threshold", "type": "number", "default": 0.35,
     "min": 0.05, "max": 0.95, "step": 0.05, "arg": "--conf",
     "help": "Boxes less sure than this are ignored. Lower finds more litter but gives more false alarms."},
    {"key": "nested", "group": "Detection", "label": "Drop double boxes", "type": "optnum", "default": 0.0, "on": 0.85,
     "min": 0.3, "max": 1.0, "step": 0.05, "arg": "--nested",
     "help": "When one box sits mostly inside another (a bottle and its cap), keep only the surer one. "
             "The number is how far inside (0.85 = 85%). Fewer duplicates, but can merge pieces lying on top of each other."},

    {"key": "camera", "group": "Camera", "label": "Camera", "type": "select", "default": "cm3", "arg": "--camera",
     "options": [["cm3", "Camera Module 3 (regular)"], ["cm3wide", "Camera Module 3 Wide"]],
     "help": "Must match the camera on the drone. Sets the map maths and the spacing between passes."},
    {"key": "mount_yaw", "group": "Camera", "label": "Top of the image points to the drone's", "type": "select", "default": "0",
     "arg": "--mount-yaw", "options": [["0", "nose (normal)"], ["90", "right side"], ["180", "tail"], ["270", "left side"]],
     "help": "Check with a camera test: walk forward holding the drone; the ground should move down the picture. Wrong = tracking breaks."},
    {"key": "shutter_us", "group": "Camera", "label": "Fast shutter", "type": "optnum", "default": 1000, "on": 1000,
     "min": 100, "max": 20000, "step": 100, "unit": "µs", "int": True, "arg": "--shutter-us",
     "help": "A short exposure stops motion blur (1000 µs = 1/1000 s). Off = the camera picks (blurrier when moving, better in dim light)."},
    {"key": "focus_m", "group": "Camera", "label": "Lock focus", "type": "optnum", "default": 4.6, "on": 4.6,
     "min": 0.5, "max": 30, "step": 0.1, "unit": "m", "arg": "--focus-m",
     "help": "Fixes focus at this distance: set it to the flying height. Off = autofocus, which can hunt mid-flight."},
    {"key": "camfps", "group": "Camera", "label": "Camera frame rate", "type": "select", "default": "30", "arg": "--camfps",
     "options": [["30", "30 fps (smooth live view)"], ["15", "15 fps (less load on the Pi)"]],
     "help": "Only affects the live view and replay; YOLO runs at ~5 fps either way."},

    {"key": "min_sightings", "group": "Tracking", "label": "Frames before a piece goes on the map", "type": "number",
     "default": 3, "min": 1, "max": 10, "step": 1, "int": True, "arg": "--min-sightings",
     "help": "Higher = fewer false alarms on the map, but small or briefly seen litter may be missed."},
    {"key": "gate_m", "group": "Tracking", "label": "Frame-to-frame match distance", "type": "number", "default": 0.5,
     "min": 0.1, "max": 3, "step": 0.1, "unit": "m", "arg": "--gate-m",
     "help": "How far from its predicted spot a box can be and still count as the same piece. "
             "Raise if one piece keeps getting new ids; lower if neighbouring pieces swap."},
    {"key": "track_timeout", "group": "Tracking", "label": "Forget a piece after", "type": "number", "default": 1.5,
     "min": 0.3, "max": 5, "step": 0.1, "unit": "s unseen", "arg": "--track-timeout",
     "help": "How long a piece can go undetected (missed frames) before it's treated as gone."},
    {"key": "merge_m", "group": "Tracking", "label": "Same piece on another pass if within", "type": "number", "default": 2.0,
     "min": 0, "max": 6, "step": 0.5, "unit": "m", "arg": "--merge-m",
     "help": "For pieces seen again on the next pass. Higher = fewer duplicates but close pieces may merge. 0 = never join passes."},

    {"key": "min_alt", "group": "Flight", "label": "Ignore detections below", "type": "number", "default": 2.0,
     "min": 0, "max": 30, "step": 0.5, "unit": "m", "arg": "--min-alt",
     "help": "Skips takeoff and landing, when the camera sees the launch spot up close."},
    {"key": "max_alt", "group": "Flight", "label": "Ignore detections above", "type": "number", "default": 30.0,
     "min": 1, "max": 120, "step": 1, "unit": "m", "arg": "--max-alt", "help": ""},

    {"key": "record", "group": "Recording", "label": "Save detection video (5 fps, with boxes)", "type": "switch",
     "default": True, "arg": "--record", "help": "The smooth 30 fps replay is always saved."},
    {"key": "save_every", "group": "Recording", "label": "Save full photos every", "type": "optnum", "default": 2.0, "on": 2.0,
     "min": 0.5, "max": 60, "step": 0.5, "unit": "s", "arg": "--save-every",
     "help": "Full-resolution photos for checking misses and training the model later. Uses about 1 MB each."},
]


def model_options():
    found = sorted(set(glob.glob(os.path.expanduser("~/yolo/*_ncnn_model")) + glob.glob(os.path.expanduser("~/yolo/*.pt"))
                       + glob.glob(os.path.join(HERE, "models", "*.pt"))))
    return [[p, os.path.basename(p) + ("  (fast on the Pi)" if p.endswith("_ncnn_model") else "")] for p in found]


def options(s):
    return model_options() if s["key"] == "model" else s.get("options", [])


class Settings:
    """Current settings plus named presets, kept in field_settings.json."""

    def __init__(self, path, camera):
        self.path, self.lock = path, threading.Lock()
        for s in SETTINGS:
            if s["key"] == "camera":
                s["default"] = camera
        try:
            d = json.load(open(path))
        except (OSError, ValueError):
            d = {}
        self.current, self.presets = self.clean(d.get("current", {})), d.get("presets", {})

    @staticmethod
    def defaults():
        return {s["key"]: s["default"] for s in SETTINGS}

    @staticmethod
    def clean(v):
        """Fills gaps with defaults and keeps every value inside its limits."""
        out = {}
        for s in SETTINGS:
            x, t = v.get(s["key"], s["default"]), s["type"]
            try:
                if t == "switch":
                    x = bool(x)
                elif t in ("number", "optnum"):
                    x = float(x)
                    if not (t == "optnum" and x == 0):
                        x = min(max(x, s["min"]), s["max"])
                    if s.get("int"):
                        x = int(round(x))
                elif t == "select":
                    x = str(x)
                    if x not in [o[0] for o in options(s)] and not (s["key"] == "model" and x == s["default"]):
                        x = s["default"]
                else:
                    x = " ".join(str(x).split())[:200]
            except (TypeError, ValueError):
                x = s["default"]
            out[s["key"]] = x
        return out

    def save(self):
        tmp = self.path + ".tmp"
        json.dump({"current": self.current, "presets": self.presets}, open(tmp, "w"), indent=1)
        os.replace(tmp, self.path)

    def update(self, values):
        with self.lock:
            self.current = self.clean({**self.current, **values})
            self.save()

    def preset(self, action, name):
        name = " ".join(str(name or "").split())[:40]
        with self.lock:
            if action == "save" and name:
                self.presets[name] = dict(self.current)
            elif action == "load" and name in self.presets:
                self.current = self.clean(self.presets[name])
            elif action == "delete":
                self.presets.pop(name, None)
            elif action == "defaults":
                self.current = self.clean({"note": self.current.get("note", "")})
            else:
                raise RuntimeError("unknown preset action or name")
            self.save()

    def logger_args(self):
        args = []
        for s in SETTINGS:
            x = self.current[s["key"]]
            if "arg" not in s:
                continue
            if s["type"] == "switch":
                args += [s["arg"]] if x else []
            elif s["type"] == "text":
                args += [s["arg"], x] if x else []
            else:
                args += [s["arg"], str(x)]   # optnum 0 is the logger's own "off"
        if not self.current["upload"]:
            args.append("--no-upload")
        return args

    def page(self):
        schema = [{**s, "options": options(s)} if s["type"] == "select" else s for s in SETTINGS]
        return {"schema": schema, "values": self.current, "defaults": self.defaults(), "presets": sorted(self.presets)}


class Logger:
    """Runs survey_logger.py as a separate process, fed by the forwarded MAVLink stream (or a fake position)."""

    def __init__(self, extra):
        self.p, self.extra, self.started = None, extra, None   # extra: function returning the logger's options

    def running(self):
        return self.p is not None and self.p.poll() is None

    def start(self, sim=None):
        if self.running():
            return
        src = ["--sim", sim] if sim else ["--mavlink", "udpin:" + FWD]
        cmd = [sys.executable, os.path.join(HERE, "survey_logger.py"), *src, "--live", LIVE, *self.extra()]
        self.p = subprocess.Popen(cmd, stdout=open(os.path.join(HERE, "logger_console.txt"), "a"), stderr=subprocess.STDOUT)
        self.started = time.time()
        log.info("Logger started (%s): %s", "sim" if sim else "flight", " ".join(cmd[2:]))

    def stop(self):
        if self.running():
            self.p.send_signal(signal.SIGINT)
            try:
                self.p.wait(timeout=40)
            except subprocess.TimeoutExpired:
                self.p.kill()
            log.info("Logger stopped")


RUN_RE = re.compile(r"^\d{8}_\d{6}$")


def run_info(run_dir):
    """Summary of one run folder for the Flights list."""
    name = os.path.basename(run_dir.rstrip("/\\"))
    info = {"run": name, "started": time.mktime(time.strptime(name, "%Y%m%d_%H%M%S")), "pieces": 0,
            "test": False, "video": None, "video_bytes": 0, "phone_video": False, "seconds": None,
            "replay": None, "replay_bytes": 0, "phone_replay": False, "note": "", "settings": {}, "upload": True}
    try:   # what the logger ran with (survey_logger writes its options to settings.json)
        st = json.load(open(os.path.join(run_dir, "settings.json")))
        info.update(test=bool(st.get("sim")), note=st.get("note", ""), upload=not st.get("no_upload"),
                    settings={k: st[k] for k in [x["key"] for x in SETTINGS] if k in st})
        info["settings"]["upload"] = info["upload"]
    except (OSError, ValueError):
        pass
    try:
        info["pieces"] = len(json.load(open(os.path.join(run_dir, "pieces.geojson")))["features"])
    except (OSError, ValueError, KeyError):
        pass
    for f, phone in (("live_30fps_h264.mp4", True), ("live_30fps.mp4", False)):   # smooth 30 fps replay
        fp = os.path.join(run_dir, f)
        if os.path.exists(fp):
            info.update(replay=f, replay_bytes=os.path.getsize(fp), phone_replay=phone)
            break
    for f, phone in (("video_h264.mp4", True), ("video.mp4", False)):
        fp = os.path.join(run_dir, f)
        if os.path.exists(fp):
            info.update(video=f, video_bytes=os.path.getsize(fp), phone_video=phone)
            break
    try:   # duration from the detection log: first and last frame times
        with open(os.path.join(run_dir, "detections.jsonl"), "rb") as fh:
            first = json.loads(fh.readline())["t"]
            fh.seek(max(0, os.path.getsize(fh.name) - 4096))
            last = json.loads(fh.read().splitlines()[-1])["t"]
            info["seconds"] = round(last - first)
    except Exception:
        pass
    return info


def run_pieces(run_dir, firebase_on):
    """Confirmed pieces of a run, newest first, with status:
    new = just added to Firebase, known = already there from an earlier flight, pending = not checked yet,
    local = no Firebase for this run (camera test or uploads off)."""
    dbp = os.path.join(run_dir, "survey.db")
    if not os.path.exists(dbp):
        return []
    try:   # camera tests and runs flown with uploads off never go to Firebase
        st = json.load(open(os.path.join(run_dir, "settings.json")))
        test = bool(st.get("sim") or st.get("no_upload"))
    except (OSError, ValueError):
        test = False
    con = sqlite3.connect(f"file:{dbp}?mode=ro", uri=True, timeout=5)
    cols = {r[1] for r in con.execute("PRAGMA table_info(pieces)")}
    st = "fb_status" if "fb_status" in cols else "'pending'"
    rows = con.execute(f"SELECT id, lat, lon, sightings, max_conf, first_seen, last_seen, crop, {st} FROM pieces "
                       "WHERE confirmed = 1 ORDER BY first_seen DESC").fetchall()
    con.close()
    feats = []
    for pid, lat, lon, n, conf, first, last, crop, status in rows:
        if test or not firebase_on:
            status = "local"
        feats.append({"type": "Feature", "geometry": {"type": "Point", "coordinates": [lon, lat]},
                      "properties": {"id": pid, "sightings": n, "max_conf": conf, "first_seen": first, "last_seen": last,
                                     "crop": crop, "status": status}})
    return feats


def retrack_run(run_dir, body):
    """Re-runs piece tracking on a finished run's saved detections with other tracking settings (no flying needed).
    Writes <run>/retrack/ and returns the new pieces plus how the old GPS-only method would have done."""
    import survey_logger as sl
    path = os.path.join(run_dir, "detections.jsonl")
    if not os.path.exists(path):
        raise RuntimeError("this run has no saved detections")
    st = json.load(open(os.path.join(run_dir, "settings.json")))
    v = Settings.clean({**st, **{k: body[k] for k in ("min_sightings", "gate_m", "track_timeout", "merge_m", "nested") if k in body}})
    frames = [json.loads(line) for line in open(path) if line.strip()]
    quiet = logging.getLogger("retrack")
    quiet.setLevel(logging.WARNING)
    args = (st.get("camera", "cm3"), st.get("mount_yaw", 0.0), st.get("min_alt", 2.0), st.get("max_alt", 30.0))
    tr = sl.retrack(frames, os.path.join(run_dir, "retrack"), *args, v["merge_m"], v["min_sightings"], v["gate_m"],
                    v["track_timeout"], v["nested"], log=quiet)
    old = sl.gps_merge(frames, *args, 3.0, v["min_sightings"])
    g = json.load(open(os.path.join(run_dir, "retrack", "pieces.geojson")))
    return {"pieces": g["features"], "new": len(tr.pieces.items), "old": len(old), "tracks": tr.n_tracks,
            "too_short": tr.n_dropped, "resighted": tr.pieces.n_resighted, "used": v}


def latest_run(out):
    runs = sorted(glob.glob(os.path.join(out, "*", "")))
    return runs[-1] if runs else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", help="flight controller connection; omit to run without a drone (plan + logger only)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--https", action="store_true", help="serve over https (self-signed) so browsers allow 'my location'")
    ap.add_argument("--surveys", default=os.path.expanduser("~/surveys"))
    ap.add_argument("--camera", default="cm3", help="default camera (the Settings tab can change it)")
    ap.add_argument("--logger-args", default="", help="extra arguments passed to survey_logger.py")
    ap.add_argument("--no-sync", action="store_true", help="don't upload finished runs to Firebase")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        handlers=[logging.FileHandler(os.path.join(HERE, "mission_server.log")), logging.StreamHandler()])

    drone = Drone(a.mavlink, a.baud) if a.mavlink else None
    if drone:
        drone.start()
    settings = Settings(os.path.join(HERE, "field_settings.json"), a.camera)
    logger = Logger(lambda: ["--out", a.surveys, *settings.logger_args(), *a.logger_args.split()])
    state = {"plan": None, "uploaded": False, "flying": False}

    live_on = logger.running

    sync_now = threading.Event()

    def convert_loop():   # convert finished recordings to H.264 so the phone can play them
        import firebase_sync
        while True:
            for run_dir in sorted(glob.glob(os.path.join(a.surveys, "*", ""))):
                if logger.running() and run_dir == latest_run(a.surveys):
                    continue
                try:
                    firebase_sync.convert_video(run_dir)
                except Exception as e:
                    log.warning("convert %s: %s", run_dir, e)
            time.sleep(30)
    threading.Thread(target=convert_loop, daemon=True).start()

    def sync_loop():   # upload finished runs to Firebase (after landing, and retry every minute if offline)
        import firebase_sync
        if not os.path.exists(firebase_sync.KEY):
            log.warning("No Firebase key at %s: uploads disabled", firebase_sync.KEY)
            return
        while True:
            sync_now.wait(timeout=60)
            sync_now.clear()
            try:
                current = latest_run(a.surveys) if logger.running() else None   # never upload a run that's still recording
                n = firebase_sync.sync_all(a.surveys, skip=current)
                if n:
                    log.info("Uploaded %d litter piece(s) to Firebase", n)
            except Exception as e:
                log.warning("Firebase sync: %s", e)
    firebase_on = not a.no_sync
    if firebase_on:
        threading.Thread(target=sync_loop, daemon=True).start()   # after landing: check against Firebase, upload only new items

    def watch():   # stop the logger after landing (disarm)
        was = False
        while True:
            armed = bool(drone and drone.s["armed"])
            if was and not armed and logger.running():
                log.info("Disarmed: stopping the logger in 3 s")
                time.sleep(3)
                logger.stop()
                state["flying"] = False
                sync_now.set()   # upload this flight's litter right away
            was = armed
            time.sleep(0.5)
    threading.Thread(target=watch, daemon=True).start()

    def status():
        run = latest_run(a.surveys)
        pieces = {"confirmed": 0}
        if run and os.path.exists(os.path.join(run, "pieces.geojson")):
            try:
                pieces["confirmed"] = len(json.load(open(os.path.join(run, "pieces.geojson")))["features"])
            except Exception:
                pass
        d = dict(drone.s) if drone else {"link": "no drone (plan / bench mode)"}
        d.update(plan=bool(state["plan"]), uploaded=state["uploaded"], logger=logger.running(), live=bool(live_on()),
                 run=os.path.basename(run.rstrip("/\\")) if run else None, pieces=pieces["confirmed"],
                 plan_minutes=state["plan"]["minutes"] if state["plan"] else None,
                 note=settings.current["note"], upload=settings.current["upload"] and firebase_on)
        return d

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def send(self, code, obj, ctype="application/json"):
            b = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(b)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST")
            self.end_headers()

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = open(os.path.join(HERE, "operator.html"), encoding="utf-8").read()
                kf = os.path.join(HERE, "secrets", "carto_key.txt")   # map tiles key, kept out of the repo
                page = page.replace("{{CARTO_KEY}}", open(kf).read().strip() if os.path.exists(kf) else "")
                return self.send(200, page.encode(), "text/html; charset=utf-8")
            if self.path == "/api/status":
                return self.send(200, status())
            if self.path == "/api/settings":
                return self.send(200, settings.page())
            if self.path == "/live.jpg":
                if live_on() and os.path.exists(LIVE):
                    return self.send(200, open(LIVE, "rb").read(), "image/jpeg")
                return self.send(404, {"error": "no live view (logger not running)"})
            if self.path == "/stream":
                return self.stream()
            if self.path == "/api/runs":
                runs = [run_info(d) for d in sorted(glob.glob(os.path.join(a.surveys, "*", "")), reverse=True)
                        if RUN_RE.match(os.path.basename(d.rstrip("/\\")))]
                current = latest_run(a.surveys) if logger.running() else None
                for r in runs:
                    r["recording"] = bool(current) and r["run"] == os.path.basename(current.rstrip("/\\"))
                return self.send(200, runs)
            m = re.match(r"^/runs/(\d{8}_\d{6})/(pieces|video|replay|crops/([\w.-]+))$", self.path.split("?")[0])
            if m:
                run_dir = os.path.join(a.surveys, m.group(1))
                if m.group(2) == "pieces":
                    f = os.path.join(run_dir, "pieces.geojson")
                    return self.send(200, json.load(open(f)) if os.path.exists(f) else {"type": "FeatureCollection", "features": []})
                if m.group(2) == "video":
                    info = run_info(run_dir) if os.path.isdir(run_dir) else {}
                    if info.get("video"):
                        return self.send_file(os.path.join(run_dir, info["video"]), "video/mp4")
                if m.group(2) == "replay":
                    info = run_info(run_dir) if os.path.isdir(run_dir) else {}
                    if info.get("replay"):
                        return self.send_file(os.path.join(run_dir, info["replay"]), "video/mp4")
                if m.group(3):
                    f = os.path.join(run_dir, "crops", os.path.basename(m.group(3)))
                    if os.path.exists(f):
                        return self.send(200, open(f, "rb").read(), "image/jpeg")
                return self.send(404, {"error": "not found"})
            if self.path == "/api/pieces":
                run = latest_run(a.surveys)
                feats = run_pieces(run, firebase_on) if run else []
                return self.send(200, {"type": "FeatureCollection", "features": feats})
            if self.path.startswith("/crops/"):
                run = latest_run(a.surveys)
                f = os.path.join(run or "", "crops", os.path.basename(self.path))
                if run and os.path.exists(f):
                    return self.send(200, open(f, "rb").read(), "image/jpeg")
            self.send(404, {"error": "not found"})

        def send_file(self, path, ctype):
            size = os.path.getsize(path)
            start, end = 0, size - 1
            rng = self.headers.get("Range")
            m = re.match(r"bytes=(\d*)-(\d*)", rng or "")
            if m:
                if m.group(1):
                    start = int(m.group(1))
                    end = int(m.group(2)) if m.group(2) else size - 1
                else:   # "bytes=-N": last N bytes
                    start = max(0, size - int(m.group(2)))
                end = min(end, size - 1)
                if start > end:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
            self.send_response(206 if m else 200)
            self.send_header("Content-Type", ctype)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(end - start + 1))
            if m:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            try:
                with open(path, "rb") as f:
                    f.seek(start)
                    left = end - start + 1
                    while left > 0:
                        chunk = f.read(min(256 * 1024, left))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        left -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def stream(self):
            """MJPEG: sends each new live frame as it appears. A slow connection just gets fewer frames."""
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last = 0.0
            try:
                while True:
                    m = os.path.getmtime(LIVE) if live_on() and os.path.exists(LIVE) else 0.0
                    if m and m != last:
                        last = m
                        b = open(LIVE, "rb").read()
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(b)).encode() + b"\r\n\r\n" + b + b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.008)   # check often enough to pass on 30 fps
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                p = self.path
                if p == "/api/plan":
                    poly = [tuple(map(float, x)) for x in body["polygon"]]
                    plan = survey_path.make_plan(poly, float(body.get("alt", 4.6)), settings.current["camera"],
                                                 float(body.get("overlap", 0.2)), float(body.get("speed", 2.0)), body.get("angle"))
                    state.update(plan=plan, uploaded=False)
                    return self.send(200, {k: plan[k] for k in ("polygon", "waypoints", "swaths", "stats", "minutes")})
                if p == "/api/settings":
                    settings.update(body.get("values", {}))
                    return self.send(200, settings.page())
                if p == "/api/settings/preset":
                    settings.preset(body.get("action"), body.get("name"))
                    return self.send(200, settings.page())
                m = re.match(r"^/api/runs/(\d{8}_\d{6})/retrack$", p)
                if m:
                    return self.send(200, retrack_run(os.path.join(a.surveys, m.group(1)), body))
                if p == "/api/logger/start":
                    logger.start(sim=body.get("sim"))
                    return self.send(200, status())
                if p == "/api/logger/stop":
                    logger.stop()
                    sync_now.set()
                    return self.send(200, status())
                if not drone:
                    raise RuntimeError("no drone connected (server started without --mavlink)")
                if p == "/api/upload":
                    if not state["plan"]:
                        raise RuntimeError("make a plan first")
                    if drone.s["armed"] and drone.s["mode"] == "AUTO":
                        raise RuntimeError("can't replace the mission while it's flying it")
                    drone.upload(state["plan"]["items"], state["plan"]["waypoints"][0])
                    state["uploaded"] = True
                elif p == "/api/start":
                    if not body.get("confirm"):
                        raise RuntimeError("send {\"confirm\": true} to start the mission")
                    if not state["uploaded"]:
                        raise RuntimeError("upload the mission first")
                    logger.start()
                    drone.start_mission()
                    state["flying"] = True
                elif p == "/api/pause":
                    drone.set_mode("BRAKE")
                elif p == "/api/resume":
                    drone.set_mode("AUTO")
                elif p == "/api/rtl":
                    drone.set_mode("RTL")
                elif p == "/api/land":
                    drone.set_mode("LAND")
                else:
                    return self.send(404, {"error": "unknown endpoint"})
                self.send(200, status())
            except Exception as e:
                log.warning("%s failed: %s", self.path, e)
                self.send(400, {"error": str(e)})

    srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
    if a.https:   # self-signed certificate, made once; browsers show a warning the first time
        import ssl
        cert, key = os.path.join(HERE, "secrets", "https_cert.pem"), os.path.join(HERE, "secrets", "https_key.pem")
        if not os.path.exists(cert):
            os.makedirs(os.path.dirname(cert), exist_ok=True)
            subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650", "-subj", "/CN=litterbug",
                            "-keyout", key, "-out", cert], check=True, capture_output=True)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
    log.info("Operator page on %s://0.0.0.0:%d  (drone: %s)", "https" if a.https else "http", a.port, a.mavlink or "none")
    try:
        srv.serve_forever()
    finally:
        logger.stop()


if __name__ == "__main__":
    main()
