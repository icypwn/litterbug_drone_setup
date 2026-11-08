import SwiftUI
import UIKit
import CoreLocation

/// Reads the Pi's MJPEG stream (/stream) and publishes the newest frame.
/// Reconnects by itself; if frames stop arriving it reports "no signal".
/// (Past footage is in the Flights tab, from the Pi's full recording.)
@MainActor @Observable
final class MJPEGStream: NSObject, URLSessionDataDelegate {
    var image: UIImage?
    var lastFrame: Date?
    var fps: Double = 0
    private var session: URLSession?
    private var task: URLSessionDataTask?
    private nonisolated(unsafe) var buffer = Data()
    private var stamps: [Date] = []
    private var url: URL?
    private var running = false

    var receiving: Bool { lastFrame.map { Date().timeIntervalSince($0) < 3 } ?? false }

    func start(url: URL) {
        stop()
        self.url = url
        running = true
        connect()
    }

    func stop() {
        running = false
        task?.cancel()
        session?.invalidateAndCancel()
        task = nil
        session = nil
    }

    private func connect() {
        guard running, let url else { return }
        let cfg = URLSessionConfiguration.ephemeral
        cfg.timeoutIntervalForRequest = 8
        cfg.requestCachePolicy = .reloadIgnoringLocalCacheData
        let s = URLSession(configuration: cfg, delegate: self, delegateQueue: nil)
        session = s
        buffer = Data()
        task = s.dataTask(with: url)
        task?.resume()
    }

    nonisolated func urlSession(_ session: URLSession, dataTask: URLSessionDataTask, didReceive data: Data) {
        buffer.append(data)
        // A JPEG starts with FF D8 and ends with FF D9: pull out every complete one, keep only the newest.
        var newest: Data?
        while let start = buffer.range(of: Data([0xFF, 0xD8])),
              let end = buffer.range(of: Data([0xFF, 0xD9]), in: start.upperBound..<buffer.endIndex) {
            newest = buffer.subdata(in: start.lowerBound..<end.upperBound)
            buffer.removeSubrange(buffer.startIndex..<end.upperBound)
        }
        if buffer.count > 4_000_000 { buffer.removeAll() }
        if let newest, let img = UIImage(data: newest) {
            Task { @MainActor in self.show(img) }
        }
    }

    nonisolated func urlSession(_ session: URLSession, task: URLSessionTask, didCompleteWithError error: Error?) {
        Task { @MainActor in
            guard self.running else { return }
            try? await Task.sleep(for: .seconds(2))
            self.connect()   // dropped or timed out: try again
        }
    }

    private func show(_ img: UIImage) {
        image = img
        let now = Date()
        lastFrame = now
        stamps = (stamps + [now]).filter { now.timeIntervalSince($0) < 3 }
        if let first = stamps.first, stamps.count > 1 { fps = Double(stamps.count - 1) / now.timeIntervalSince(first) }
    }
}

/// Live view from the drone's camera with the litter boxes drawn on.
struct CameraView: View {
    @Environment(OperatorModel.self) private var op
    @AppStorage(Pref.droneHost) private var host = Pref.defaultHost
    @State private var stream = MJPEGStream()
    @State private var locationManager = CLLocationManager()
    @State private var tick = Date()
    @State private var fullScreen = false

    private var detecting: Bool { op.status?.logger ?? false }
    private var armed: Bool { op.status?.armed ?? false }

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack {
                Text("Camera").font(.largeTitle.bold()).foregroundStyle(.white)
                Spacer()
                if stream.receiving { Chip(text: String(format: "LIVE · %.1f fps", stream.fps), color: .good) }
            }
            .padding(.top, 4)

            videoCard

            legend
                .padding(.horizontal, 4)

            InkCard {
                VStack(alignment: .leading, spacing: 12) {
                    HStack(spacing: 10) {
                        DoodleBadge(symbol: detecting ? "eye.fill" : "eye.slash.fill", color: detecting ? .good : .white.opacity(0.25), size: 34, seed: "det")
                        VStack(alignment: .leading, spacing: 2) {
                            Text(detecting ? "Detecting litter" : "Detection off").font(.headline).foregroundStyle(.white)
                            Text(statusLine).font(.caption).foregroundStyle(.white.opacity(0.75))
                        }
                        Spacer(minLength: 0)
                    }

                    Button { Task { await testCamera(start: !detecting) } } label: {
                        Label(detecting ? "Stop camera test" : "Start camera test",
                              systemImage: detecting ? "stop.fill" : "video.fill")
                            .frame(maxWidth: .infinity)
                    }
                    .buttonStyle(InkButtonStyle(fill: detecting ? .white.opacity(0.12) : .litter))
                    .disabled(!op.reachable || armed || op.busy)

                    if let m = op.message {
                        Text(m).font(.caption).foregroundStyle(op.messageIsError ? Color.litter : .white.opacity(0.8))
                    }
                }
            }
            Spacer(minLength: 0)
        }
        .padding(.horizontal, 14)
        .background(Color.appBackground.ignoresSafeArea())
        .onAppear {
            locationManager.requestWhenInUseAuthorization()
            if let u = DroneClient(host: host).baseURL?.appendingPathComponent("stream") { stream.start(url: u) }
            if ProcessInfo.processInfo.arguments.contains("-fullCamera") { fullScreen = true }   // testing aid
        }
        .onDisappear { if !fullScreen { stream.stop() } }
        .fullScreenCover(isPresented: $fullScreen) { FullScreenCamera(stream: stream) }
        .onChange(of: host) { _, h in
            if let u = DroneClient(host: h).baseURL?.appendingPathComponent("stream") { stream.start(url: u) }
        }
        .task(id: host) {
            while !Task.isCancelled {
                await op.poll(host: host)
                tick = Date()   // re-evaluates "receiving" so the no-signal state appears when frames stop
                try? await Task.sleep(for: .seconds(1))
            }
        }
    }

    private var statusLine: String {
        guard op.reachable, let s = op.status else { return "Pi not reachable (\(host))" }
        if armed { return [s.mode, s.alt.map { String(format: "%.1f m up", $0) }, "\(s.pieces) found"].compactMap { $0 }.joined(separator: " · ") }
        if detecting { return "Camera test · \(s.pieces) found · uses your location as the drone's" }
        return "Starts with the mission. Run a camera test to check it on the ground."
    }

    private var videoCard: some View {
        ZStack {
            Color.black.opacity(0.35)
            if let img = stream.image {
                Image(uiImage: img).resizable().scaledToFit()
                    .opacity(stream.receiving ? 1 : 0.35)
            }
            if !stream.receiving { noSignal }
        }
        .aspectRatio(16 / 9, contentMode: .fit)
        .overlay(alignment: .bottomTrailing) {
            Button { fullScreen = true } label: {
                Image(systemName: "arrow.up.left.and.arrow.down.right").font(.system(size: 15, weight: .bold)).foregroundStyle(Color.ink)
                    .frame(width: 38, height: 38).background(.white.opacity(0.92), in: Circle())
                    .overlay(Circle().stroke(Color.ink, lineWidth: 1.5))
            }
            .padding(12)
        }
        .onTapGesture { fullScreen = true }
        .clipShape(RoundedRectangle(cornerRadius: 28, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 28, style: .continuous).stroke(Color.ink, lineWidth: 2))
        .shadow(color: .black.opacity(0.15), radius: 16, y: 8)
    }

    private var noSignal: some View {
        VStack(spacing: 8) {
            Image(systemName: "video.slash.fill").font(.system(size: 34, weight: .bold))
            Text(!op.reachable ? "Pi not reachable" : (op.status?.live ?? false) ? "Waiting for video…" : "Camera not running")
                .font(.headline)
        }
        .foregroundStyle(.white.opacity(0.9))
        .multilineTextAlignment(.center)
        .padding()
    }

    private var legend: some View {
        HStack(spacing: 0) {
            legendItem(Color(red: 1, green: 0.78, blue: 0), "New")
            Spacer(minLength: 6)
            legendItem(Color(red: 0.16, green: 0.63, blue: 1), "Seen again")
            Spacer(minLength: 6)
            legendItem(Color(red: 0.24, green: 0.86, blue: 0.24), "On map")
            Spacer(minLength: 6)
            legendItem(Color(red: 1, green: 0.24, blue: 0.24), "No GPS")
        }
        .font(.caption.weight(.semibold))
        .foregroundStyle(.white.opacity(0.85))
    }

    private func legendItem(_ c: Color, _ text: String) -> some View {
        HStack(spacing: 5) { RoundedRectangle(cornerRadius: 3).stroke(c, lineWidth: 2.5).frame(width: 14, height: 11); Text(text) }
    }

    private func testCamera(start: Bool) async {
        if start {
            let c = locationManager.location?.coordinate ?? CLLocationCoordinate2D(latitude: 0, longitude: 0)
            await op.loggerTest(host: host, sim: String(format: "%.6f,%.6f,4.6,0", c.latitude, c.longitude))
        } else {
            await op.loggerTest(host: host, sim: nil)
        }
    }
}

/// Sideways, edge-to-edge live view. Tap anywhere to show or hide the controls.
struct FullScreenCamera: View {
    let stream: MJPEGStream
    @Environment(OperatorModel.self) private var op
    @Environment(\.dismiss) private var dismiss
    @State private var showControls = true

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            if let img = stream.image {
                Image(uiImage: img).resizable().scaledToFit().ignoresSafeArea()
                    .opacity(stream.receiving ? 1 : 0.35)
            }
            if !stream.receiving {
                Label("No signal", systemImage: "video.slash.fill").font(.headline).foregroundStyle(.white)
            }
            if showControls {
                VStack {
                    HStack(spacing: 8) {
                        if stream.receiving { Chip(text: String(format: "LIVE · %.1f fps", stream.fps), color: .good) }
                        if let s = op.status {
                            if s.logger { Chip(text: "\(s.pieces) found", color: .white) }
                            if let alt = s.alt { Chip(text: String(format: "%.1f m", alt), color: .white) }
                        }
                        Spacer()
                        Button { dismiss() } label: {
                            Image(systemName: "xmark").font(.system(size: 16, weight: .bold)).foregroundStyle(Color.ink)
                                .frame(width: 40, height: 40).background(.white.opacity(0.92), in: Circle())
                                .overlay(Circle().stroke(Color.ink, lineWidth: 1.5))
                        }
                    }
                    Spacer()
                }
                .padding(16)
                .transition(.opacity)
            }
        }
        .contentShape(Rectangle())
        .onTapGesture { withAnimation(.easeInOut(duration: 0.2)) { showControls.toggle() } }
        .statusBarHidden()
        .persistentSystemOverlays(.hidden)
        .onAppear { Orientation.set(.landscape) }
        .onDisappear { Orientation.set(.portrait) }
    }
}
