#!/usr/bin/env python3
"""Upload finished survey runs to Firebase (Firestore) so the litter shows up for consumers.

For each run folder in ~/surveys/:
  - every confirmed piece becomes (or updates) a document in  litter/{id}
      lat, lon, geohash, status ("open"), confidence, sightings, firstSeen, lastSeen, flights, photo (small JPEG, base64)
    A piece within MERGE_M of an existing open one (e.g. seen again on a later flight) updates that one instead.
  - the run becomes a document in  flights/{run}  (time, piece count, where the video is kept)
  - the recording is converted to H.264 (video_h264.mp4) so phones can play it; it stays on the Pi
Pieces are marked uploaded in the run's survey.db, so re-running only sends what's new. If there's no
internet it just fails quietly and the next pass retries.

  python firebase_sync.py                 # sync every run once
  python firebase_sync.py --watch         # keep syncing every minute
"""
import argparse, base64, glob, json, logging, math, os, shutil, sqlite3, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
KEY = os.environ.get("LITTERBUG_FIREBASE_KEY", os.path.join(HERE, "firebase", "service-account.json"))
MERGE_M = 3.0
log = logging.getLogger("sync")

_B32 = "0123456789bcdefghjkmnpqrstuvwxyz"


def geohash(lat, lon, precision=9):
    """Standard geohash, so apps can query 'litter near me' by prefix."""
    lat_r, lon_r, bits, bit, ch, out, even = [-90.0, 90.0], [-180.0, 180.0], [16, 8, 4, 2, 1], 0, 0, [], True
    while len(out) < precision:
        rng, val = (lon_r, lon) if even else (lat_r, lat)
        mid = (rng[0] + rng[1]) / 2
        if val > mid:
            ch |= bits[bit]
            rng[0] = mid
        else:
            rng[1] = mid
        even = not even
        if bit < 4:
            bit += 1
        else:
            out.append(_B32[ch])
            bit, ch = 0, 0
    return "".join(out)


def metres(lat1, lon1, lat2, lon2):
    k = math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(math.radians(lat2 - lat1) * 6378137, math.radians(lon2 - lon1) * 6378137 * k)


def small_photo(path, max_side=320):
    """The drone's crop, shrunk to a small JPEG (base64) that fits comfortably in a Firestore document."""
    import cv2
    img = cv2.imread(path)
    if img is None:
        return None
    h, w = img.shape[:2]
    s = max_side / max(h, w)
    if s < 1:
        img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return base64.b64encode(buf.tobytes()).decode() if ok else None


def client():
    import firebase_admin
    from firebase_admin import credentials, firestore
    if not firebase_admin._apps:
        firebase_admin.initialize_app(credentials.Certificate(KEY))
    return firestore.client()


def convert_video(run_dir):
    """Convert the recordings (5 fps detection video and 30 fps replay) to H.264 for phones/browsers."""
    ok = convert_one(run_dir, "video")
    convert_one(run_dir, "live_30fps")
    return ok


def convert_one(run_dir, name):
    """mp4v -> H.264. Skipped if ffmpeg is missing, the logger is still writing, or already done."""
    src, dst = os.path.join(run_dir, name + ".mp4"), os.path.join(run_dir, name + "_h264.mp4")
    if not os.path.exists(src) or os.path.exists(dst) or not shutil.which("ffmpeg"):
        return os.path.exists(dst)
    if time.time() - os.path.getmtime(src) < 30:
        return False   # still being recorded
    tmp = dst + ".part.mp4"
    r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src, "-c:v", "libx264", "-preset", "veryfast",
                        "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", tmp], capture_output=True)
    if r.returncode == 0:
        os.replace(tmp, dst)
        log.info("Converted %s for phones", dst)
        return True
    log.warning("Video conversion failed: %s", r.stderr.decode()[-300:])
    return False


def sync_run(db, run_dir, source="drone", live=False):
    from firebase_admin import firestore
    from google.cloud.firestore_v1.base_query import FieldFilter
    run = os.path.basename(run_dir.rstrip("/"))
    dbp = os.path.join(run_dir, "survey.db")
    if not os.path.exists(dbp):
        return 0
    try:   # camera tests on the ground use a fake position: never publish those as litter
        if json.load(open(os.path.join(run_dir, "settings.json"))).get("sim"):
            return 0
    except (OSError, ValueError):
        pass
    con = sqlite3.connect(dbp, timeout=10)
    cols = {r[1] for r in con.execute("PRAGMA table_info(pieces)")}
    for c, decl in (("fb_status", "TEXT DEFAULT 'pending'"), ("fb_id", "TEXT")):   # older runs
        if c not in cols:
            con.execute(f"ALTER TABLE pieces ADD COLUMN {c} {decl}")
    rows = con.execute("SELECT id, lat, lon, sightings, max_conf, first_seen, last_seen, crop FROM pieces "
                       "WHERE confirmed = 1 AND uploaded = 0").fetchall()
    sent = 0
    for pid, lat, lon, sightings, conf, first, last, crop in rows:
        # same litter seen before (this or an earlier flight)? update it instead of adding a duplicate
        d = 0.00005   # ~5 m of latitude
        near = [s for s in db.collection("litter").where(filter=FieldFilter("lat", ">=", lat - d)).where(filter=FieldFilter("lat", "<=", lat + d)).stream()
                if s.to_dict().get("status") == "open" and metres(lat, lon, s.get("lat"), s.get("lon")) <= MERGE_M]
        from datetime import datetime, timezone
        ts_first = datetime.fromtimestamp(first, timezone.utc)
        ts_last = datetime.fromtimestamp(last, timezone.utc)
        if near:
            ref = near[0].reference
            old = near[0].to_dict()
            upd = {"lastSeen": ts_last, "sightings": firestore.Increment(sightings), "flights": firestore.ArrayUnion([run]),
                   "confidence": max(old.get("confidence", 0), conf)}
            if conf > old.get("confidence", 0) and crop:
                upd["photo"] = small_photo(os.path.join(run_dir, crop))
            ref.update(upd)
            fb_status, fb_id = "known", ref.id      # already on the map from an earlier flight
        else:
            db.collection("litter").document(pid).set({
                "lat": lat, "lon": lon, "geohash": geohash(lat, lon), "status": "open", "source": source,
                "confidence": round(conf, 3), "sightings": sightings, "firstSeen": ts_first, "lastSeen": ts_last,
                "flights": [run], "photo": small_photo(os.path.join(run_dir, crop)) if crop else None,
            })
            fb_status, fb_id = "new", pid
        con.execute("UPDATE pieces SET uploaded = 1, fb_status = ?, fb_id = ? WHERE id = ?", (fb_status, fb_id, pid))
        con.commit()
        sent += 1
    total = con.execute("SELECT COUNT(*) FROM pieces WHERE confirmed = 1").fetchone()[0]
    con.close()
    if live:   # mid-flight: just the litter; the flight record and video come after landing
        return sent
    has_video = convert_video(run_dir)
    db.collection("flights").document(run).set({
        "run": run, "pieces": total, "source": source, "syncedAt": firestore.SERVER_TIMESTAMP,
        "video": os.path.join(run_dir, "video_h264.mp4" if has_video else "video.mp4"),
    }, merge=True)
    if sent:
        log.info("%s: sent %d new piece(s) (%d confirmed in total)", run, sent, total)
    return sent


def sync_all(surveys, skip=None, source="drone"):
    """Sync every run except `skip` (e.g. the one still recording)."""
    db = client()
    n = 0
    for run_dir in sorted(glob.glob(os.path.join(surveys, "*", ""))):
        if skip and os.path.abspath(run_dir.rstrip("/")) == os.path.abspath(skip.rstrip("/")):
            continue
        try:
            n += sync_run(db, run_dir, source)
        except Exception as e:
            log.warning("%s: %s (will retry)", os.path.basename(run_dir.rstrip("/")), e)
            break   # most likely offline; try again next pass
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--surveys", default=os.path.expanduser("~/surveys"))
    ap.add_argument("--watch", action="store_true", help="keep syncing every --every seconds")
    ap.add_argument("--every", type=float, default=60)
    ap.add_argument("--source", default="drone")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    if not os.path.exists(KEY):
        sys.exit(f"Firebase key not found at {KEY}")
    while True:
        sync_all(a.surveys, source=a.source)
        if not a.watch:
            break
        time.sleep(a.every)


if __name__ == "__main__":
    main()
