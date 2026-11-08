import SwiftUI
import AVKit
import Photos
import MapboxMaps

/// One past flight or camera test, from the Pi's /api/runs.
struct RunSummary: Decodable, Identifiable, Hashable {
    let run: String
    let started: Double
    let pieces: Int
    let test: Bool
    let video: String?
    let videoBytes: Int
    let phoneVideo: Bool
    let seconds: Int?
    let recording: Bool?
    let replay: String?
    let replayBytes: Int?
    let phoneReplay: Bool?

    var hasSmooth: Bool { phoneReplay == true }

    var id: String { run }
    var date: Date { Date(timeIntervalSince1970: started) }
    var durationText: String? {
        guard let s = seconds, s > 0 else { return nil }
        return s < 60 ? "\(s) s" : "\(s / 60) min \(s % 60) s"
    }
}

extension DroneClient {
    func runs() async throws -> [RunSummary] {
        guard let base = baseURL else { throw APIError(message: "Bad drone address") }
        let (data, _) = try await URLSession.shared.data(from: base.appendingPathComponent("api/runs"))
        let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
        return try d.decode([RunSummary].self, from: data)
    }

    func runPieces(_ run: String) async throws -> [LitterPiece] {
        guard let base = baseURL else { throw APIError(message: "Bad drone address") }
        let runBase = base.appendingPathComponent("runs/\(run)")
        let (data, _) = try await URLSession.shared.data(from: runBase.appendingPathComponent("pieces"))
        let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
        return try d.decode(PiecesResponse.self, from: data).pieces(base: runBase)
    }

    func videoURL(_ run: String, smooth: Bool = false) -> URL? { baseURL?.appendingPathComponent("runs/\(run)/\(smooth ? "replay" : "video")") }
}

/// Past flights stored on the Pi: litter found, the recording, save to Photos.
struct FlightsView: View {
    @AppStorage(Pref.droneHost) private var host = Pref.defaultHost
    @State private var runs: [RunSummary] = []
    @State private var error: String?
    @State private var loaded = false
    @State private var showTests = false
    @State private var path: [RunSummary] = []

    private var shown: [RunSummary] { showTests ? runs : runs.filter { !$0.test } }

    var body: some View {
        NavigationStack(path: $path) {
            ScrollView {
                VStack(alignment: .leading, spacing: 12) {
                    HStack {
                        Text("Flights").font(.largeTitle.bold()).foregroundStyle(.white)
                        Spacer()
                        Toggle("Tests", isOn: $showTests).toggleStyle(.button).buttonStyle(.ink).font(.caption)
                    }
                    .padding(.top, 4)

                    if let error {
                        InkCard { Label(error, systemImage: "wifi.exclamationmark").font(.subheadline).foregroundStyle(Color.litter).frame(maxWidth: .infinity, alignment: .leading) }
                    } else if loaded && shown.isEmpty {
                        InkCard {
                            VStack(alignment: .leading, spacing: 4) {
                                Text(runs.isEmpty ? "No flights yet" : "No flights yet (only camera tests)").font(.headline).foregroundStyle(.white)
                                Text("Each mission is saved on the drone's Pi and shows up here after landing.").font(.caption).foregroundStyle(.white.opacity(0.75))
                            }
                            .frame(maxWidth: .infinity, alignment: .leading)
                        }
                    }
                    ForEach(shown) { r in
                        NavigationLink(value: r) { FlightRow(run: r) }.buttonStyle(.plain)
                    }
                }
                .padding(.horizontal, 14)
                .padding(.bottom, 20)
            }
            .background(Color.appBackground.ignoresSafeArea())
            .refreshable { await load() }
            .navigationDestination(for: RunSummary.self) { FlightDetailView(run: $0) }
            .toolbar(.hidden, for: .navigationBar)
            .task(id: host) { await load() }
        }
    }

    private func load() async {
        do {
            runs = try await DroneClient(host: host).runs()
            error = nil
            if ProcessInfo.processInfo.arguments.contains("-openFirstFlight"), path.isEmpty, let first = shown.first { path = [first] }   // testing aid
        } catch {
            self.error = "Can't reach the Pi (\(host)) to load flights."
        }
        loaded = true
    }
}

struct FlightRow: View {
    let run: RunSummary
    var body: some View {
        InkCard {
            HStack(spacing: 12) {
                DoodleBadge(symbol: run.test ? "video.fill" : "airplane", color: run.test ? .path : .litter, size: 38, seed: run.run)
                VStack(alignment: .leading, spacing: 3) {
                    HStack(spacing: 6) {
                        Text(run.date.formatted(date: .abbreviated, time: .shortened)).font(.headline).foregroundStyle(.white)
                        if run.test { Chip(text: "Test", color: .white.opacity(0.8)) }
                        if run.recording == true { Chip(text: "Recording", color: .caution) }
                    }
                    Text([run.durationText, "\(run.pieces) litter found",
                          run.video == nil ? "no video" : (run.phoneVideo ? nil : "converting video…")]
                        .compactMap { $0 }.joined(separator: " · "))
                        .font(.caption).foregroundStyle(.white.opacity(0.75))
                }
                Spacer()
                Image(systemName: "chevron.right").foregroundStyle(.white.opacity(0.5))
            }
        }
    }
}

struct FlightDetailView: View {
    let run: RunSummary
    @AppStorage(Pref.droneHost) private var host = Pref.defaultHost
    @Environment(\.dismiss) private var dismiss
    @State private var pieces: [LitterPiece] = []
    @State private var player: AVPlayer?
    @State private var viewport: Viewport = .idle
    @State private var saving = false
    @State private var saveMessage: String?
    @State private var smooth = true   // 30 fps replay when available

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    Button { dismiss() } label: {
                        Image(systemName: "chevron.left").font(.system(size: 16, weight: .bold)).foregroundStyle(.white)
                            .frame(width: 40, height: 40).background(.white.opacity(0.12), in: Circle())
                            .overlay(Circle().stroke(Color.ink, lineWidth: 2))
                    }
                    VStack(alignment: .leading, spacing: 0) {
                        Text(run.date.formatted(date: .abbreviated, time: .shortened)).font(.title2.bold()).foregroundStyle(.white)
                        Text([run.test ? "Camera test" : "Flight", run.durationText, "\(run.pieces) litter found"].compactMap { $0 }.joined(separator: " · "))
                            .font(.caption).foregroundStyle(.white.opacity(0.75))
                    }
                    Spacer()
                }
                .padding(.top, 4)

                video
                if !pieces.isEmpty { map }
                if !pieces.isEmpty { pieceList }
            }
            .padding(.horizontal, 14)
            .padding(.bottom, 20)
        }
        .background(Color.appBackground.ignoresSafeArea())
        .toolbar(.hidden, for: .navigationBar)
        .task {
            loadPlayer()
            pieces = (try? await DroneClient(host: host).runPieces(run.run)) ?? []
            if pieces.count == 1 { viewport = .camera(center: pieces[0].coordinate, zoom: 19) }
            else if pieces.count > 1 { viewport = .overview(geometry: MultiPoint(pieces.map(\.coordinate)), geometryPadding: EdgeInsets(top: 40, leading: 40, bottom: 40, trailing: 40)) }
        }
        .onDisappear { player?.pause() }
    }

    private var useSmooth: Bool { smooth && run.hasSmooth }

    private func loadPlayer() {
        player?.pause()
        player = nil
        guard run.phoneVideo || run.hasSmooth, let u = DroneClient(host: host).videoURL(run.run, smooth: useSmooth || !run.phoneVideo) else { return }
        player = AVPlayer(url: u)
    }

    @ViewBuilder private var video: some View {
        if run.hasSmooth && run.phoneVideo {
            Picker("Video", selection: $smooth) {
                Text("Smooth · 30 fps").tag(true)
                Text("Detections · 5 fps").tag(false)
            }
            .pickerStyle(.segmented)
            .onChange(of: smooth) { _, _ in loadPlayer() }
        }
        ZStack {
            Color.black.opacity(0.35)
            if let player {
                VideoPlayer(player: player)
            } else {
                VStack(spacing: 6) {
                    Image(systemName: run.video == nil ? "video.slash.fill" : "hourglass").font(.system(size: 28, weight: .bold))
                    Text(run.video == nil ? "No recording for this run" : "Converting the video for iPhone…").font(.subheadline.weight(.semibold))
                    if run.video != nil { Text("Pull to refresh the Flights list in a minute").font(.caption).opacity(0.75) }
                }
                .foregroundStyle(.white)
            }
        }
        .aspectRatio(16 / 9, contentMode: .fit)
        .clipShape(RoundedRectangle(cornerRadius: 22, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 22, style: .continuous).stroke(Color.ink, lineWidth: 2))

        if run.phoneVideo || run.hasSmooth {
            HStack(spacing: 8) {
                Button { Task { await saveToPhotos() } } label: {
                    Label(saving ? "Saving…" : "Save to Photos", systemImage: "square.and.arrow.down").frame(maxWidth: .infinity)
                }
                .buttonStyle(.inkPrimary).disabled(saving)
                Text(ByteCountFormatter.string(fromByteCount: Int64(useSmooth ? (run.replayBytes ?? 0) : run.videoBytes), countStyle: .file))
                    .font(.caption).foregroundStyle(.white.opacity(0.7))
            }
            if let saveMessage { Text(saveMessage).font(.caption).foregroundStyle(.white.opacity(0.8)) }
        }
    }

    private var map: some View {
        Map(viewport: $viewport) {
            ForEvery(pieces) { p in
                MapViewAnnotation(coordinate: p.coordinate) { DoodleBadge(symbol: "trash.fill", size: 26, seed: p.id) }
                    .allowOverlap(true)
            }
        }
        .mapStyle(Doodle.style)
        .ornamentOptions(OperatorView.ornamentsForOtherMaps)
        .frame(height: 240)
        .clipShape(RoundedRectangle(cornerRadius: 22, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 22, style: .continuous).stroke(Color.ink, lineWidth: 2))
    }

    private var pieceList: some View {
        InkCard {
            VStack(alignment: .leading, spacing: 10) {
                Text("Litter found").font(.headline).foregroundStyle(.white)
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 10) {
                        ForEach(pieces) { p in
                            VStack(alignment: .leading, spacing: 4) {
                                AsyncImage(url: p.cropURL) { img in img.resizable().scaledToFill() } placeholder: { Color.black.opacity(0.3) }
                                    .frame(width: 96, height: 96).clipShape(RoundedRectangle(cornerRadius: 12))
                                    .overlay(RoundedRectangle(cornerRadius: 12).stroke(Color.ink, lineWidth: 2))
                                Text("\(Int(p.confidence * 100))% · seen \(p.sightings)×").font(.caption2).foregroundStyle(.white.opacity(0.8))
                            }
                        }
                    }
                }
            }
        }
    }

    private func saveToPhotos() async {
        guard let url = DroneClient(host: host).videoURL(run.run, smooth: useSmooth || !run.phoneVideo) else { return }
        saving = true; defer { saving = false }
        do {
            let (tmp, _) = try await URLSession.shared.download(from: url)
            let file = FileManager.default.temporaryDirectory.appendingPathComponent("litterbug-\(run.run).mp4")
            try? FileManager.default.removeItem(at: file)
            try FileManager.default.moveItem(at: tmp, to: file)
            let status = await PHPhotoLibrary.requestAuthorization(for: .addOnly)
            guard status == .authorized || status == .limited else { saveMessage = "Allow Photos access in Settings to save videos."; return }
            try await PHPhotoLibrary.shared().performChanges {
                PHAssetChangeRequest.creationRequestForAssetFromVideo(atFileURL: file)
            }
            try? FileManager.default.removeItem(at: file)
            saveMessage = "Saved to Photos."
        } catch {
            saveMessage = "Couldn't save: \(error.localizedDescription)"
        }
    }
}
