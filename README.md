# Litterbug (Pi side)

| File | What it does |
|---|---|
| `survey_path.py` | Area (corners) -> lawnmower flight path. Writes a QGroundControl `.plan`, a Mission Planner `.waypoints`, and a preview map. |
| `survey_logger.py` | On the drone: camera -> YOLO -> drone position -> distinct litter pieces with GPS, crops, optional video (`--record`). `--selftest` checks the map math. |
| `mission_server.py` | On the drone: web API + operator page. Takes an area, plans, uploads the mission, arms/starts it, runs the logger, stops it after landing. |
| `operator.html` | The operator page served by `mission_server.py` (draw area, preview, upload, start, pause, return home, live status, found litter). |

## Bench test (no drone)
    python3 survey_logger.py --selftest
    python3 survey_logger.py --sim 40.7402,-73.8410,4.6,0 --camera cm3wide --record --seconds 60
    python3 mission_server.py            # plan-only mode; open http://<pi>:8080

## With the drone
    python3 mission_server.py --mavlink /dev/ttyAMA0 --baud 57600

Logger output: `~/surveys/<date_time>/` (survey.db, pieces.geojson, crops/, frames/, video.mp4, detections.jsonl).

## Repo layout
- Pi scripts at the top level (`mission_server.py`, `survey_logger.py`, `firebase_sync.py`, `survey_path.py`, `operator.html` = web app)
- `operator-ios/` LB Operator iPhone app (`brew install xcodegen && xcodegen generate` in that folder, then open in Xcode)
- `models/` trained YOLO models (`litter_ground_drone_1280.pt` is the one in use; convert to NCNN on the Pi with
  `yolo export model=models/litter_ground_drone_1280.pt format=ncnn imgsz=736,1280`)
- `firebase/` Firestore rules (`firebase deploy --only firestore:rules --project litterbug-38a1f`)
- `HANDOFF.txt` status, decisions and to-do list

## Secrets
- In this PRIVATE repo: `firebase/service-account.json` (Pi uploads to Firebase) and `secrets/carto_key.txt` (web map tiles).
  Keep the repo private; anyone with access can write to the Firebase project.
- Not in the repo (get from Ethan): `operator-ios/Operator/Sources/Secrets.swift` (Mapbox token; see `Secrets.swift.example`)
