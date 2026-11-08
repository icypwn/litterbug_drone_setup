# Tracking each piece of litter across frames

**Status (2026-11-07): built in `survey_logger.py`, tested in simulation only. Needs a real flight to confirm.**

## The problem
The drone sees the same piece of litter in many frames (~5 detections/second, often 10–40 frames per piece).
We need to know which detections are the **same physical item**, so each item becomes exactly one map entry.

The old code merged every detection with any piece within **3 m by GPS**. That failed both ways:
- **GPS jitter (2–5 m)**: one item drifted outside 3 m and became two pieces (duplicates).
- **Items close together**: two real items under 3 m apart were merged into one (lost items).

## What was built (`Tracker` and `Pieces` in `survey_logger.py`)
1. **Velocity**: `Pose` now has `vx, vy` (north/east m/s) from MAVLink `GLOBAL_POSITION_INT`. `Telemetry.at()`
   blends the readings just before and after each frame instead of taking the nearest one.
2. **Ground offsets instead of raw pixels**: each box becomes "metres north/east of the spot below the drone",
   using height, tilt (roll/pitch) and heading. Tilt and turns are handled by the geometry, not by guessing.
3. **Predict**: between frames the ground slides by `-velocity × time`, so each open track predicts where its
   piece should be now.
4. **Match one-to-one**: boxes are paired with predictions (scipy `linear_sum_assignment`, greedy fallback)
   inside a gate of `max(--gate-m 0.5 m, 1.5 × box size)`, widened for speed and fast turns.
   Two boxes in the same frame are never the same piece.
5. **Rules**: unmatched box → new track. Track unseen for `--track-timeout` 1.5 s → closed.
   A track goes on the map after `--min-sightings` 3 frames; shorter tracks are dropped as false detections.
6. **Pin position** = confidence-weighted average of the track's GPS positions. GPS decides *where*, not *which*.
7. **Seen again on another pass**: matched within `--merge-m` (now **2 m**) on the drone's own smooth path
   (velocity added up over time, pulled gently toward GPS over ~60 s), not raw GPS, which wanders by metres
   between passes. One-to-one, and never merges two tracks that were on screen at the same time.
   Checked when a track is confirmed and again when it closes.
8. Across flights: unchanged, `firebase_sync.py` still checks against Firestore (3 m) after landing.

Live view colours are unchanged: yellow = new, blue = followed (not on the map yet), green = on the map.

## Results in simulation (`track_sim.py`, 6 random fields, 5 passes, 20–25% overlap)
| | map entries | real items found (of 335) | duplicates | items lost |
|---|---|---|---|---|
| old (GPS 3 m) | 190 | 161 | 19 | 174 |
| new | 331 | 324 | 6 | 11 |

Also held up with worse velocity, worse GPS, 3.5 m/s, the wide camera, and 60% detection rate with many false boxes.
Caveat: it's my own simulation. Real GPS/velocity errors and YOLO misses may differ.

## Results with the real model (`track_photo_test.py`, 2026-11-08)
`litter_ground_drone_1280.pt` run on a virtual 2-pass flight over 10 UAVVaste **test** photos (125 labelled pieces;
YOLO saw 98 at least once, 93 in 3+ frames):
| | map entries | found (of 93) | duplicates | lost | entries with no labelled piece |
|---|---|---|---|---|---|
| old (GPS 3 m) | 46 | 28 | 3 | 65 | 15 |
| new | 145 | 85 | 19 | 8 | 41 |

- Duplicates all come from YOLO putting **two boxes on one labelled piece in the same frames** (bottle + its cap,
  part of a pile), or from one label box covering two objects. The tracker can't merge boxes seen together.
  `--nested 0.85` (drop a box mostly inside another) cuts duplicates 19 -> 14 for 1 fewer found; off by default.
- "No labelled piece" = YOLO kept seeing something with no label: a false alarm or unlabelled litter.
  The old method had fewer only because it lumped things together.
- Limits: still photos (no blur, lighting change or tilt), so detections are steadier than in flight.
- Current Ultralytics no longer installs `scipy`; without it the tracker uses its greedy fallback (the simulation
  results above used that). `pip install scipy` on the Pi for the best one-to-one matching.

## How to test
    python3 survey_logger.py --selftest           # map math + tracking checks
    python3 track_sim.py                          # fake flights: old vs new, with true answers
    python3 track_photo_test.py --images DIR --annotations annotations.json   # real model on UAVVaste photos
    python3 survey_logger.py --replay ~/surveys/<run>   # re-run tracking on a recorded flight -> <run>/retrack/

Old runs have no velocity in `detections.jsonl`; replay estimates it from GPS, so results there are a bit worse.

## Still to do
- Real flight: check the piece count against what's actually on the ground; tune `--gate-m` / `--merge-m`.
- Confirm the camera's image top points to the drone's nose (else `--mount-yaw`), or tracks will break on every frame.
- Frame time is when the Pi receives the frame, not when it was exposed; using the camera's `SensorTimestamp`
  would tighten the predictions a little.
- Not tried: optical flow from the images as a backup when telemetry is bad; Ultralytics BoT-SORT as a comparison.
