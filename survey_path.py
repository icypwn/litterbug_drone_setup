#!/usr/bin/env python3
"""Turn a survey area (polygon) into a lawnmower flight path for ArduCopter.

The passes are spaced so neighbouring camera views overlap, at the given height.
Outputs (next to --out):
  <out>.plan        QGroundControl mission (open it in QGC, check it, upload)
  <out>.waypoints   Mission Planner mission (same path)
  <out>.html        preview map: area, path, and what the camera covers

Area input, either:
  - a GeoJSON file with one Polygon (draw it at geojson.io and save), or
  - "lat,lon;lat,lon;lat,lon;..." corners in order

Example:
  python3 survey_path.py park.geojson --alt 4.6 --camera cm3 --overlap 0.2 --speed 2 --out park
"""
import argparse, json, math, sys

R_EARTH = 6378137.0
# Field of view (horizontal, vertical) in degrees for the full 16:9 view.
CAMERAS = {"cm3": (66.0, 41.0), "cm3wide": (102.0, 67.0)}


def load_polygon(arg):
    if arg.lower().endswith((".geojson", ".json")):
        g = json.load(open(arg))
        if g.get("type") == "FeatureCollection":
            g = g["features"][0]
        if g.get("type") == "Feature":
            g = g["geometry"]
        if g.get("type") != "Polygon":
            sys.exit(f"Need a Polygon, got {g.get('type')}")
        pts = [(lat, lon) for lon, lat in g["coordinates"][0]]
    else:
        pts = [tuple(map(float, p.split(","))) for p in arg.split(";") if p.strip()]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        sys.exit("The area needs at least 3 corners.")
    return pts


def to_local(pts, lat0, lon0):
    k = math.cos(math.radians(lat0))
    return [((lon - lon0) * math.pi / 180 * R_EARTH * k, (lat - lat0) * math.pi / 180 * R_EARTH) for lat, lon in pts]


def to_ll(x, y, lat0, lon0):
    return (lat0 + math.degrees(y / R_EARTH), lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(lat0)))))


def rot(x, y, a):
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))


def area_m2(xy):
    return abs(sum(xy[i][0] * xy[(i + 1) % len(xy)][1] - xy[(i + 1) % len(xy)][0] * xy[i][1] for i in range(len(xy)))) / 2


def longest_edge_angle(xy):
    best, ang = 0, 0.0
    for i in range(len(xy)):
        (x1, y1), (x2, y2) = xy[i], xy[(i + 1) % len(xy)]
        d = math.hypot(x2 - x1, y2 - y1)
        if d > best:
            best, ang = d, math.atan2(y2 - y1, x2 - x1)
    return ang


def lawnmower(xy, spacing, angle):
    """Parallel passes `spacing` metres apart, running along `angle`, clipped to the polygon.
    Returns a list of (start, end) points in local metres, in flying order (alternating direction)."""
    P = [rot(x, y, -angle) for x, y in xy]
    ys = [p[1] for p in P]
    y, k, lines = min(ys) + spacing / 2, 0, []
    while y < max(ys):
        xs = []
        for i in range(len(P)):
            (x1, y1), (x2, y2) = P[i], P[(i + 1) % len(P)]
            if (y1 <= y < y2) or (y2 <= y < y1):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        segs = [(xs[j], xs[j + 1]) for j in range(0, len(xs) - 1, 2)]
        if k % 2:
            segs = [(b, a) for a, b in reversed(segs)]
        lines += [((a, y), (b, y)) for a, b in segs]
        y += spacing
        k += 1
    return [(rot(*p0, angle), rot(*p1, angle)) for p0, p1 in lines]


def mission_items(wps, alt, speed):
    """(command, frame, params) in ArduPilot mission order. frame 3 = relative to home altitude."""
    lat0, lon0 = wps[0]
    items = [(22, 3, [0, 0, 0, None, lat0, lon0, alt]),            # take off
             (178, 2, [1, speed, -1, 0, 0, 0, 0])]                  # set ground speed
    items += [(16, 3, [0, 0, 0, None, lat, lon, alt]) for lat, lon in wps]  # waypoints
    items.append((20, 2, [0, 0, 0, 0, 0, 0, 0]))                    # return to launch
    return items


def write_plan(path, items, home, poly, fence):
    plan_items = [{"type": "SimpleItem", "autoContinue": True, "command": c, "doJumpId": i + 1, "frame": f, "params": p}
                  for i, (c, f, p) in enumerate(items)]
    plan = {"fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
            "mission": {"version": 2, "firmwareType": 3, "vehicleType": 2, "cruiseSpeed": 5, "hoverSpeed": 2,
                        "plannedHomePosition": [home[0], home[1], 0], "items": plan_items},
            "geoFence": {"version": 2, "circles": [],
                         "polygons": [{"inclusion": True, "version": 1, "polygon": [[a, b] for a, b in poly]}] if fence else []},
            "rallyPoints": {"version": 2, "points": []}}
    json.dump(plan, open(path, "w"), indent=2)


def write_waypoints(path, items, home):
    rows = ["QGC WPL 110", f"0\t1\t0\t16\t0\t0\t0\t0\t{home[0]:.8f}\t{home[1]:.8f}\t0\t1"]
    for i, (c, f, p) in enumerate(items, start=1):
        p = [0 if v is None else v for v in p]
        rows.append(f"{i}\t0\t{f}\t{c}\t" + "\t".join(f"{v:.8f}" if j >= 4 else f"{v:g}" for j, v in enumerate(p)) + "\t1")
    open(path, "w").write("\n".join(rows) + "\n")


def write_preview(path, poly, wps, swaths, stats):
    data = json.dumps({"poly": poly, "path": wps, "swaths": swaths})
    s = "".join(f"<div><b>{k}</b> {v}</div>" for k, v in stats.items())
    open(path, "w").write(f"""<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Survey path</title>
<link rel=stylesheet href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body{{margin:0;height:100%;font:14px system-ui}}#map{{height:100%}}
#info{{position:absolute;top:10px;right:10px;z-index:999;background:#fff;padding:10px 14px;border-radius:8px;box-shadow:0 2px 8px #0003;line-height:1.6}}</style>
<div id=map></div><div id=info>{s}<div style="color:#666;margin-top:6px">Green = area &middot; blue = path &middot; yellow = camera coverage</div></div>
<script>
const d = {data};
const map = L.map('map');
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{maxZoom: 21, maxNativeZoom: 19, attribution: '&copy; OpenStreetMap'}}).addTo(map);
d.swaths.forEach(s => L.polygon(s, {{color: '#e0b000', weight: 0, fillOpacity: 0.25}}).addTo(map));
const area = L.polygon(d.poly, {{color: '#1a9a3a', weight: 3, fillOpacity: 0.05}}).addTo(map);
L.polyline(d.path, {{color: '#1565ff', weight: 3}}).addTo(map);
L.circleMarker(d.path[0], {{radius: 7, color: '#1a9a3a', fillOpacity: 1}}).bindTooltip('start').addTo(map);
L.circleMarker(d.path[d.path.length - 1], {{radius: 7, color: '#d33', fillOpacity: 1}}).bindTooltip('end').addTo(map);
map.fitBounds(area.getBounds(), {{padding: [30, 30]}});
</script>""")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("area", help="GeoJSON file or 'lat,lon;lat,lon;...'")
    ap.add_argument("--alt", type=float, default=4.6, help="height above takeoff in metres (4.6 m = 15 ft)")
    ap.add_argument("--camera", choices=CAMERAS, default="cm3")
    ap.add_argument("--overlap", type=float, default=0.2, help="side overlap between passes, 0-0.9")
    ap.add_argument("--speed", type=float, default=2.0, help="ground speed in m/s")
    ap.add_argument("--angle", type=float, default=None, help="pass direction in degrees from east (default: along the longest edge)")
    ap.add_argument("--fence", action="store_true", help="add the area as a geofence (takeoff point must be inside it)")
    ap.add_argument("--out", default="survey")
    a = ap.parse_args()

    try:
        p = make_plan(load_polygon(a.area), a.alt, a.camera, a.overlap, a.speed, a.angle)
    except ValueError as e:
        sys.exit(str(e))
    home = p["waypoints"][0]
    write_plan(a.out + ".plan", p["items"], home, p["polygon"], a.fence)
    write_waypoints(a.out + ".waypoints", p["items"], home)
    write_preview(a.out + ".html", p["polygon"], p["waypoints"], p["swaths"], p["stats"])
    for k, v in p["stats"].items():
        print(f"{k:14} {v}")
    print(f"Wrote {a.out}.plan, {a.out}.waypoints, {a.out}.html")


def make_plan(poly, alt=4.6, camera="cm3", overlap=0.2, speed=2.0, angle_deg=None):
    """Area corners [(lat, lon), ...] -> lawnmower plan: waypoints, mission items, coverage swaths and stats."""
    if len(poly) < 3:
        raise ValueError("The area needs at least 3 corners.")
    if not (1.0 <= alt <= 120) or not (0 <= overlap <= 0.9) or not (0.3 <= speed <= 10):
        raise ValueError("Height must be 1-120 m, overlap 0-0.9, speed 0.3-10 m/s.")
    lat0 = sum(p[0] for p in poly) / len(poly)
    lon0 = sum(p[1] for p in poly) / len(poly)
    xy = to_local(poly, lat0, lon0)
    if area_m2(xy) > 200_000:
        raise ValueError("Area is over 20 hectares; split it into smaller surveys.")
    hfov, vfov = CAMERAS[camera]
    across = 2 * alt * math.tan(math.radians(hfov) / 2)   # camera mounted with the top of the image toward the nose
    along = 2 * alt * math.tan(math.radians(vfov) / 2)
    spacing = across * (1 - overlap)
    angle = math.radians(angle_deg) if angle_deg is not None else longest_edge_angle(xy)

    lines = lawnmower(xy, spacing, angle)
    if not lines:
        raise ValueError("No passes fit in this area; check the corners.")
    wps_xy = [p for seg in lines for p in seg]
    wps = [to_ll(x, y, lat0, lon0) for x, y in wps_xy]

    swaths = []
    for (x1, y1), (x2, y2) in lines:
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        nx, ny = -uy * across / 2, ux * across / 2
        ex, ey = ux * along / 2, uy * along / 2
        corners = [(x1 - ex + nx, y1 - ey + ny), (x2 + ex + nx, y2 + ey + ny), (x2 + ex - nx, y2 + ey - ny), (x1 - ex - nx, y1 - ey - ny)]
        swaths.append([to_ll(x, y, lat0, lon0) for x, y in corners])

    length = sum(math.dist(wps_xy[i], wps_xy[i + 1]) for i in range(len(wps_xy) - 1))
    minutes = (length / speed + 4 * len(wps_xy)) / 60   # ~4 s to slow, turn and speed up at each corner
    stats = {"Area": f"{area_m2(xy):,.0f} m²", "Height": f"{alt} m ({alt * 3.281:.0f} ft)",
             "Camera covers": f"{across:.1f} × {along:.1f} m", "Pass spacing": f"{spacing:.1f} m ({overlap:.0%} overlap)",
             "Passes": len(lines), "Path length": f"{length:,.0f} m", "Flight time": f"~{minutes:.1f} min at {speed} m/s"}
    return {"polygon": [list(p) for p in poly], "waypoints": [list(w) for w in wps], "swaths": swaths, "stats": stats,
            "items": mission_items(wps, alt, speed), "alt": alt, "speed": speed, "minutes": round(minutes, 1)}


if __name__ == "__main__":
    main()
