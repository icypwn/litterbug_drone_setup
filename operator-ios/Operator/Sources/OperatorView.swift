import SwiftUI
@_spi(Restricted) import MapboxMaps

/// Draw the survey area, preview the path, upload, fly, watch.
struct OperatorView: View {
    @Environment(OperatorModel.self) private var op
    @AppStorage(Pref.droneHost) private var host = Pref.defaultHost
    @AppStorage("opHeightFt") private var heightFt = 15.0
    @AppStorage("opSpeed") private var speed = 2.0
    @AppStorage("opOverlap") private var overlap = 20.0
    @State private var viewport: Viewport = .followPuck(zoom: 17.5, bearing: .constant(0))
    @State private var confirmStart = false
    @State private var centredOnDrone = false
    @State private var showSettings = false
    @State private var locationManager = CLLocationManager()
    @State private var mapFrame: CGRect = .zero          // map's position on screen, for converting drags to coordinates
    @State private var draggingCorner: Int?
    @State private var replanAfterDrag = false

    var body: some View {
        VStack(spacing: 12) {
            header
            map
            controls
        }
        .padding(.horizontal, 14)
        .padding(.bottom, 8)
        .background(Color.appBackground.ignoresSafeArea())
        .confirmationDialog("Start the mission?", isPresented: $confirmStart, titleVisibility: .visible) {
            Button("Arm and take off", role: .destructive) {
                Task { await op.send("start", host: host, confirm: true, ok: "Mission started: arming and taking off.") }
            }
        } message: {
            Text("The drone will take off to \(Int(heightFt)) ft and fly the uploaded path. Keep the controller in hand.")
        }
        .sheet(isPresented: $showSettings) { DroneSettingsSheet().presentationDetents([.medium]) }
        .onAppear {
            locationManager.requestWhenInUseAuthorization()
            #if DEBUG
            if ProcessInfo.processInfo.arguments.contains("-demoArea"), op.corners.isEmpty {   // screenshot/testing aid
                [(40.74020, -73.84150), (40.74060, -73.84070), (40.74010, -73.84030), (40.73975, -73.84105)]
                    .forEach { op.addCorner(.init(latitude: $0.0, longitude: $0.1)) }
                viewport = .camera(center: .init(latitude: 40.74018, longitude: -73.84090), zoom: 17.6)
                Task { await op.makePlan(host: host, heightFeet: heightFt, speed: speed, overlapPercent: overlap); if op.plan != nil && draggingCorner == nil { fitArea() } }
            }
            #endif
        }
        .task(id: host) {
            var tick = 0
            while !Task.isCancelled {
                await op.poll(host: host)
                if tick % 2 == 0 { await op.pollPieces(host: host) }   // new items show up within a couple of seconds
                if !centredOnDrone, let d = op.status?.position {
                    viewport = .camera(center: d, zoom: 18.5); centredOnDrone = true
                }
                tick += 1
                try? await Task.sleep(for: .seconds(1))
            }
        }
    }

    // MARK: header + status

    private var header: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("Operator").font(.largeTitle.bold()).foregroundStyle(.white)
                Spacer()
                Button { showSettings = true } label: {
                    Image(systemName: "gearshape.fill").font(.title3).foregroundStyle(.white)
                        .frame(width: 40, height: 40).background(.white.opacity(0.12), in: Circle())
                        .overlay(Circle().stroke(Color.ink, lineWidth: 2))
                }
            }
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) { chips }
            }
            if let s = op.status, s.lat != nil {
                Text(telemetry(s)).font(.caption.monospacedDigit()).foregroundStyle(.white.opacity(0.75))
            }
            if let last = op.status?.messages?.last {
                Text(last).font(.caption2).foregroundStyle(.white.opacity(0.6)).lineLimit(1)
            }
        }
        .padding(.top, 4)
    }

    @ViewBuilder private var chips: some View {
        if !op.reachable {
            Chip(text: "Pi not reachable · \(host)", color: .litter)
        } else if let s = op.status {
            Chip(text: s.connected ? "Drone connected" : (s.link.hasPrefix("no drone") ? "Pi online · no drone" : s.link),
                 color: s.connected ? .good : .caution)
            if let fix = s.gpsFix, let sats = s.sats {
                Chip(text: "GPS \(fix >= 3 ? "3D" : "no fix") · \(sats) sats", color: fix >= 3 && sats >= 8 ? .good : .caution)
            }
            if let b = s.battery { Chip(text: "Battery \(b)%", color: b > 30 ? .good : b > 15 ? .caution : .litter) }
            if let mode = s.mode { Chip(text: "\((s.armed ?? false) ? "ARMED" : "Disarmed") · \(mode)", color: (s.armed ?? false) ? .caution : .white) }
            Chip(text: s.logger ? "Detecting · \(s.pieces) found" : "Detection off", color: s.logger ? .good : .white.opacity(0.7))
        }
    }

    private func telemetry(_ s: DroneStatus) -> String {
        var t = "Height \(s.alt.map { String(format: "%.1f m", $0) } ?? "–") · \(s.speed.map { String(format: "%.1f m/s", $0) } ?? "–")"
        if let seq = s.missionSeq, let n = s.missionCount, n > 1 { t += " · waypoint \(seq) of \(n - 1)" }
        return t
    }

    // MARK: map

    private var map: some View {
        MapReader { proxy in
            Map(viewport: $viewport) {
                Puck2D()
                if let plan = op.plan {
                    ForEvery(Array(plan.swathPolygons.enumerated()), id: \.offset) { i, s in
                        PolygonAnnotation(id: "swath-\(i)", polygon: Polygon([s + [s[0]]]))
                            .fillColor(StyleColor(UIColor(red: 1.0, green: 0.85, blue: 0.3, alpha: 1)))
                            .fillOpacity(0.28)
                    }
                    PolylineAnnotation(id: "path", lineCoordinates: plan.path)
                        .lineColor(StyleColor(UIColor(Color.path)))
                        .lineWidth(4)
                }
                if op.corners.count >= 3 {
                    PolygonAnnotation(id: "area", polygon: Polygon([op.corners + [op.corners[0]]]))
                        .fillColor(StyleColor(UIColor(Color.litter)))
                        .fillOpacity(0.10)
                }
                if op.corners.count >= 2 {
                    PolylineAnnotation(id: "area-edge", lineCoordinates: op.corners + (op.corners.count >= 3 ? [op.corners[0]] : []))
                        .lineColor(StyleColor(UIColor(Color.ink)))
                        .lineWidth(2.5)
                }
                ForEvery(Array(op.corners.enumerated()), id: \.offset) { i, c in
                    MapViewAnnotation(coordinate: c) {
                        Text("\(i + 1)").font(.caption.bold()).foregroundStyle(.white)
                            .frame(width: 26, height: 26).background(draggingCorner == i ? Color.caution : Color.litter, in: Circle())
                            .overlay(Circle().stroke(.white, lineWidth: 2))
                            .overlay(Circle().inset(by: -2).stroke(Color.ink, lineWidth: 2))
                            .scaleEffect(draggingCorner == i ? 1.35 : 1)
                            .frame(width: 44, height: 44)          // bigger touch target than the badge
                            .contentShape(Circle())
                            .gesture(
                                DragGesture(minimumDistance: 2, coordinateSpace: .global)
                                    .onChanged { v in
                                        if draggingCorner == nil { draggingCorner = i; replanAfterDrag = op.plan != nil }
                                        let p = CGPoint(x: v.location.x - mapFrame.minX, y: v.location.y - mapFrame.minY)
                                        if let coord = proxy.map?.coordinate(for: p) { op.moveCorner(i, to: coord) }
                                    }
                                    .onEnded { _ in
                                        draggingCorner = nil
                                        if replanAfterDrag, op.corners.count >= 3 {
                                            Task { await op.makePlan(host: host, heightFeet: heightFt, speed: speed, overlapPercent: overlap); if op.plan != nil && draggingCorner == nil { fitArea() } }
                                        }
                                    }
                            )
                            .onLongPressGesture(minimumDuration: 0.6) { op.removeCorner(i) }
                    }
                    .allowOverlap(true).allowOverlapWithPuck(true)
                }
                if let start = op.plan?.path.first, let end = op.plan?.path.last, let home = launchPoint {
                    // trip out (your position -> first pass) and trip back (last pass -> where it took off), with arrowheads
                    PolylineAnnotation(id: "trip-out", lineCoordinates: [home, start])
                        .lineColor(StyleColor(UIColor(Color.ink))).lineWidth(2.5)
                    PolylineAnnotation(id: "trip-back", lineCoordinates: [end, home])
                        .lineColor(StyleColor(UIColor(Color.ink))).lineWidth(2.5)
                    ForEvery([(home, start, "out"), (end, home, "back")], id: \.2) { from, to, _ in
                        MapViewAnnotation(coordinate: along(from, to, 0.55)) { ArrowHead(degrees: bearing(from, to)) }
                            .allowOverlap(true).allowOverlapWithPuck(true)
                    }
                    MapViewAnnotation(coordinate: home) {
                        DoodleBadge(symbol: "house.fill", color: .good, size: 28, seed: "home").offset(y: -28)   // above your location dot
                    }
                    .allowOverlap(true).allowOverlapWithPuck(true)
                }
                if let end = op.plan?.path.last {
                    MapViewAnnotation(coordinate: end) {   // last pass ends here, then the drone flies back to where it took off
                        DoodleBadge(symbol: "flag.checkered", color: .ink, size: 26, seed: "end").offset(y: -26)
                    }
                    .allowOverlap(true).allowOverlapWithPuck(true)
                }
                if let start = op.plan?.path.first {
                    MapViewAnnotation(coordinate: start) {   // sits above the point so it doesn't hide a corner badge
                        DoodleBadge(symbol: "flag.fill", color: .good, size: 26, seed: "start").offset(y: -26)
                    }
                    .allowOverlap(true).allowOverlapWithPuck(true)
                }
                ForEvery(op.pieces) { p in   // every confirmed piece, live; greyed out once found to be a repeat after landing
                    MapViewAnnotation(coordinate: p.coordinate) {
                        DoodleBadge(symbol: "trash.fill", color: p.isKnown ? .gray : .litter, size: 24, seed: p.id).opacity(p.isKnown ? 0.6 : 1)
                    }
                        .allowOverlap(true).allowOverlapWithPuck(true)
                }
                if let d = op.status?.position {
                    MapViewAnnotation(coordinate: d) {
                        DoodleBadge(symbol: "airplane", color: .path, size: 36, seed: "drone")
                            .rotationEffect(.degrees(Double(op.status?.heading ?? 0) - 90))
                    }
                    .allowOverlap(true).allowOverlapWithPuck(true)
                }
            }
            .mapStyle(Doodle.style)
            .ornamentOptions(Self.ornaments)
            .gestureOptions(Self.gestures)
            .onMapTapGesture { context in
                // taps on a marker (home, flags, drone, an existing corner) shouldn't add a corner
                if !isNearMarker(context.point, proxy) { op.addCorner(context.coordinate) }
            }
            .onStyleLoaded { _ in
                try? proxy.map?.addImage(Hatch.image(color: UIColor(red: 0.30, green: 0.48, blue: 0.58, alpha: 0.35)), id: "hatch-blue")
            }
            .overlay(alignment: .topTrailing) {
                VStack(spacing: 8) {
                    mapButton("arrow.uturn.backward") { op.undo() }.disabled(op.corners.isEmpty)
                    mapButton("trash") { op.clear() }.disabled(op.corners.isEmpty)
                    Spacer().frame(height: 4)
                    mapButton("plus") { zoom(proxy, by: 1) }
                    mapButton("minus") { zoom(proxy, by: -1) }
                    mapButton("viewfinder") { fitArea() }
                    mapButton("location.fill") { withViewportAnimation(.easeOut(duration: 0.4)) { viewport = .followPuck(zoom: 17, bearing: .constant(0)) } }
                }
                .padding(12)
            }
        }
        .background(GeometryReader { g in
            Color.clear
                .onAppear { mapFrame = g.frame(in: .global) }
                .onChange(of: g.frame(in: .global)) { _, f in mapFrame = f }
        })
        .clipShape(RoundedRectangle(cornerRadius: 28, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 28, style: .continuous).stroke(Color.ink, lineWidth: 2))
        .shadow(color: .black.opacity(0.15), radius: 16, y: 8)
        .overlay(alignment: .bottom) {   // hint sits just under the map so it never covers a corner
            Text(op.corners.count < 3 ? "Tap the map to add corners (\(op.corners.count) of 3+)" : "\(op.corners.count) corners · drag to move · hold to delete")
                .font(.caption.weight(.semibold)).foregroundStyle(Color.ink)
                .padding(.horizontal, 10).padding(.vertical, 4)
                .background(.white, in: Capsule())
                .overlay(Capsule().stroke(Color.ink, lineWidth: 1.5))
                .offset(y: 12)
        }
        .padding(.bottom, 10)
    }

    /// No scale bar, Mapbox logo or attribution button (personal use; uses Mapbox's restricted option).
    /// Keep the map north-up so the arrows always point the right way.
    private static let gestures: GestureOptions = {
        var g = GestureOptions()
        g.rotateEnabled = false
        g.pitchEnabled = false
        return g
    }()

    static var ornamentsForOtherMaps: OrnamentOptions { ornaments }

    private static let ornaments: OrnamentOptions = {
        var logo = LogoViewOptions()
        logo.visibility = .hidden
        var attribution = AttributionButtonOptions()
        attribution.visibility = .hidden
        return OrnamentOptions(scaleBar: ScaleBarViewOptions(visibility: .hidden), logo: logo, attributionButton: attribution)
    }()

    /// Where the drone takes off and returns to: its real home once connected, otherwise your phone's location.
    private var launchPoint: CLLocationCoordinate2D? {
        op.status?.home ?? locationManager.location?.coordinate
    }

    private func along(_ a: CLLocationCoordinate2D, _ b: CLLocationCoordinate2D, _ f: Double) -> CLLocationCoordinate2D {
        .init(latitude: a.latitude + (b.latitude - a.latitude) * f, longitude: a.longitude + (b.longitude - a.longitude) * f)
    }

    /// Compass direction from a to b in degrees (0 = up/north, clockwise). The map is kept north-up.
    private func bearing(_ a: CLLocationCoordinate2D, _ b: CLLocationCoordinate2D) -> Double {
        let dx = (b.longitude - a.longitude) * cos(a.latitude * .pi / 180)
        let dy = b.latitude - a.latitude
        return atan2(dx, dy) * 180 / .pi
    }

    private func isNearMarker(_ pt: CGPoint, _ proxy: MapProxy) -> Bool {
        guard let map = proxy.map else { return false }
        var spots: [(CLLocationCoordinate2D, CGFloat)] = op.corners.map { ($0, 0) }      // (place, how far above it the badge is drawn)
        if let home = launchPoint, op.plan != nil { spots.append((home, -28)) }
        if let s = op.plan?.path.first { spots.append((s, -26)) }
        if let e = op.plan?.path.last { spots.append((e, -26)) }
        if let d = op.status?.position { spots.append((d, 0)) }
        return spots.contains { c, dy in
            let p = map.point(for: c)
            return hypot(p.x - pt.x, p.y + dy - pt.y) < 24
        }
    }

    private func zoom(_ proxy: MapProxy, by delta: Double) {
        guard let cam = proxy.map?.cameraState else { return }
        withViewportAnimation(.easeOut(duration: 0.25)) {
            viewport = .camera(center: cam.center, zoom: max(3, min(21, cam.zoom + delta)), bearing: cam.bearing, pitch: 0)
        }
    }

    /// Zoom to show the whole area and path (or your location if nothing's drawn yet).
    private func fitArea() {
        let pts = op.corners + (op.plan?.path ?? []) + (op.status?.position.map { [$0] } ?? []) + (op.plan != nil ? (launchPoint.map { [$0] } ?? []) : [])
        withViewportAnimation(.easeOut(duration: 0.4)) {
            if pts.count >= 2 {
                viewport = .overview(geometry: MultiPoint(pts), geometryPadding: EdgeInsets(top: 50, leading: 40, bottom: 40, trailing: 70))
            } else {
                viewport = .followPuck(zoom: 16, bearing: .constant(0))
            }
        }
    }

    private func mapButton(_ symbol: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: symbol).font(.system(size: 15, weight: .bold)).foregroundStyle(Color.ink)
                .frame(width: 36, height: 36).background(.white.opacity(0.92), in: Circle())
                .overlay(Circle().stroke(Color.ink, lineWidth: 1.5))
        }
    }

    // MARK: controls

    private var connected: Bool { op.reachable && (op.status?.connected ?? false) }

    private var controls: some View {
        InkCard {
            VStack(alignment: .leading, spacing: 10) {
                HStack(spacing: 8) {
                    Stepper(value: $heightFt, in: 6...100, step: 1) {
                        Text("\(Int(heightFt)) ft").font(.subheadline.weight(.semibold).monospacedDigit()).foregroundStyle(.white)
                    }
                    .fixedSize()
                    Spacer(minLength: 4)
                    Menu {
                        ForEach([1.0, 1.5, 2.0, 3.0, 4.0], id: \.self) { v in Button("\(v, specifier: "%.1f") m/s") { speed = v } }
                    } label: { Text("\(speed, specifier: "%.1f") m/s") }
                    .buttonStyle(.ink)
                    Menu {
                        ForEach([10.0, 20.0, 30.0, 40.0], id: \.self) { v in Button("\(Int(v))% overlap") { overlap = v } }
                    } label: { Text("\(Int(overlap))%") }
                    .buttonStyle(.ink)
                }

                Button {
                    Task { await op.makePlan(host: host, heightFeet: heightFt, speed: speed, overlapPercent: overlap); if op.plan != nil && draggingCorner == nil { fitArea() } }
                } label: { Label("Preview path", systemImage: "point.topleft.down.to.point.bottomright.curvepath").frame(maxWidth: .infinity) }
                .buttonStyle(.inkPrimary)
                .disabled(op.corners.count < 3 || !op.reachable || op.busy)

                if let plan = op.plan {
                    Text(["Passes", "Flight time", "Area", "Pass spacing"].compactMap { k in plan.stats[k].map { "\(k): \($0.display)" } }
                        .joined(separator: " · "))
                        .font(.caption).foregroundStyle(.white.opacity(0.8))
                }

                HStack(spacing: 8) {
                    Button { Task { await op.send("upload", host: host, ok: "Mission uploaded to the drone.") } } label: {
                        Label("Upload", systemImage: "arrow.up.circle").frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.ink).disabled(op.plan == nil || !connected || op.busy)
                    Button { confirmStart = true } label: {
                        Label("Start", systemImage: "play.fill").frame(maxWidth: .infinity)
                    }
                    .buttonStyle(.inkPrimary).disabled(!(op.status?.uploaded ?? false) || !connected || op.busy)
                }

                HStack(spacing: 8) {
                    Button("Pause") { Task { await op.send("pause", host: host, ok: "Paused: holding position.") } }.buttonStyle(.ink)
                    Button("Resume") { Task { await op.send("resume", host: host, ok: "Resuming the mission.") } }.buttonStyle(.ink)
                    Spacer(minLength: 0)
                    Button("Land") { Task { await op.send("land", host: host, ok: "Landing here.") } }.buttonStyle(.ink)
                    Button("Home") { Task { await op.send("rtl", host: host, ok: "Returning home.") } }.buttonStyle(.inkDanger)
                }
                .disabled(!connected)

                if let m = op.message {
                    Text(m).font(.caption).foregroundStyle(op.messageIsError ? Color.litter : .white.opacity(0.8))
                }
            }
        }
    }
}

struct DroneSettingsSheet: View {
    @AppStorage(Pref.droneHost) private var host = Pref.defaultHost
    @Environment(\.dismiss) private var dismiss
    var body: some View {
        NavigationStack {
            Form {
                Section {
                    TextField(Pref.defaultHost, text: $host)
                        .textInputAutocapitalization(.never).autocorrectionDisabled().keyboardType(.URL)
                } header: { Text("Drone address") } footer: {
                    Text("The Pi on the drone, on the same Wi-Fi as this phone. Default: \(Pref.defaultHost)")
                }
            }
            .scrollContentBackground(.hidden)
            .background(Color.appBackground)
            .navigationTitle("Drone").navigationBarTitleDisplayMode(.inline)
            .toolbar { ToolbarItem(placement: .topBarTrailing) { Button("Done") { dismiss() } } }
        }
    }
}

/// Doodle arrowhead pointing along a direction (degrees clockwise from up).
struct ArrowHead: View {
    let degrees: Double
    var body: some View {
        ZStack {
            Image(systemName: "arrowtriangle.up.fill").font(.system(size: 24, weight: .black)).foregroundStyle(.white)
            Image(systemName: "arrowtriangle.up.fill").font(.system(size: 17, weight: .black)).foregroundStyle(Color.ink)
        }
        .shadow(color: .black.opacity(0.25), radius: 1, x: 0, y: 1)
        .rotationEffect(.degrees(degrees))
    }
}
