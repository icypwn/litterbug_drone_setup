# TODO: track each piece of litter across frames

## The problem
The drone sees the same piece of litter in many frames (~5 detections/second, often 10–40 frames per piece).
We need to know which detections are the **same physical item**, so each item becomes exactly one map entry.

Right now (`survey_logger.py`, class `Pieces`) we only use **GPS**: every detection is turned into a lat/lon and
merged with any existing piece within **3 m** (`--merge-m`). That doesn't work well:
- **GPS jitter (2–5 m)**: one item can drift outside 3 m between frames and become two pieces (duplicates).
- **Items close together**: two real items under 3 m apart get merged into one (we lose items).
- Shrinking/growing the radius just trades one error for the other.
- It ignores the most precise information we have: where the item is **in the image**, frame to frame.

## Proposed fix (friend's idea): predict motion from speed + distance
Between two frames the ground moves under the camera by a predictable amount:
`ground distance moved = drone speed × time between frames`.
At a known height, that converts to a predictable **pixel shift** in the image. So for each item we're already
tracking, we can predict where it should appear in the next frame, and match new detections to those predictions.

### Steps
1. **Get drone velocity per frame.** Add `vx, vy` (north/east, m/s) to `Pose` from MAVLink `GLOBAL_POSITION_INT`
   (`vx`, `vy` are in cm/s) in `Telemetry`. Also keep heading/yaw and altitude (already there).
2. **Convert to expected pixel shift.** Ground motion in the camera frame (rotate north/east into
   forward/right with yaw) ÷ metres-per-pixel at the current height
   (`m_per_px = 2·alt·tan(HFOV/2) / image_width`). Ground moves opposite to the drone in the image.
3. **Predict.** For each active track: `predicted_xy = last_xy + expected_shift × (t_now − t_last)`.
4. **Match.** Pair this frame's boxes with predicted positions (nearest within a gate of e.g. 2–3× box size;
   use Hungarian/one-to-one assignment so two boxes can never both match one track).
5. **Rules.**
   - Two boxes in the **same frame** are always different items (never merge them).
   - Unmatched box → new track. Track unseen for ~1–2 s (or left the frame) → close it.
   - A track becomes a **confirmed item** after N matches (currently 3).
6. **Item position** = average (confidence-weighted) of the track's per-frame GPS positions; GPS only decides
   *where* the pin goes, not *which item* a detection belongs to.
7. **Across passes / flights** (item seen again much later): fall back to GPS distance, but with one-to-one
   matching, and never merge two tracks that were ever visible at the same time.

### Things to watch
- Drone **tilt** (pitch/roll) shifts the view: use attitude from `ATTITUDE` (already read) in the prediction, or
  run the prediction on geolocated ground coordinates instead of raw pixels.
- Turns at the end of each pass (yaw changes fast): widen the gate or reset tracks.
- Optional backup when telemetry is noisy: estimate the shift directly from the images (optical flow,
  e.g. OpenCV `calcOpticalFlowPyrLK` / phase correlation between consecutive frames).
- Ultralytics also has built-in trackers (ByteTrack/BoT-SORT via `model.track`); BoT-SORT already does
  camera-motion compensation and could be tried first as a quick baseline.

## Where to change / how to test
- Code: `survey_logger.py` (`Pose`, `Telemetry`, `Pieces`, main loop). `firebase_sync.py` dedupe stays as the
  final check after landing.
- Test offline without flying: every frame's detections + drone pose are saved in `detections.jsonl` in each run
  folder, so a new tracker can be replayed on old runs and compared (pieces found, duplicates, merges).
