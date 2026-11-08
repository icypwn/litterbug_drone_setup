import Foundation
import CoreLocation

/// Talks to mission_server.py on the drone's Raspberry Pi.
struct DroneClient {
    var host: String   // e.g. "litterbug.local:8080"

    var baseURL: URL? {
        let h = host.trimmingCharacters(in: .whitespaces)
        return URL(string: h.hasPrefix("http") ? h : "http://\(h)")
    }

    private static let session: URLSession = {
        let c = URLSessionConfiguration.ephemeral
        c.timeoutIntervalForRequest = 4
        c.waitsForConnectivity = false
        return URLSession(configuration: c)
    }()

    private func request(_ path: String, body: [String: Any]? = nil, timeout: TimeInterval = 4) async throws -> Data {
        guard let base = baseURL, let url = URL(string: path, relativeTo: base) else { throw APIError(message: "Bad drone address") }
        var r = URLRequest(url: url, timeoutInterval: timeout)
        if let body {
            r.httpMethod = "POST"
            r.setValue("application/json", forHTTPHeaderField: "Content-Type")
            r.httpBody = try JSONSerialization.data(withJSONObject: body)
        }
        let (data, resp) = try await Self.session.data(for: r)
        if let http = resp as? HTTPURLResponse, http.statusCode >= 400 {
            let msg = (try? JSONSerialization.jsonObject(with: data) as? [String: Any])?["error"] as? String
            throw APIError(message: msg ?? "Drone returned error \(http.statusCode)")
        }
        return data
    }

    private var decoder: JSONDecoder {
        let d = JSONDecoder()
        d.keyDecodingStrategy = .convertFromSnakeCase
        return d
    }

    func status() async throws -> DroneStatus {
        try decoder.decode(DroneStatus.self, from: try await request("/api/status", timeout: 3))
    }

    func pieces() async throws -> [LitterPiece] {
        try decoder.decode(PiecesResponse.self, from: try await request("/api/pieces")).pieces(base: baseURL)
    }

    func plan(corners: [CLLocationCoordinate2D], altMeters: Double, speed: Double, overlap: Double) async throws -> PlanResponse {
        let body: [String: Any] = ["polygon": corners.map { [$0.latitude, $0.longitude] },
                                   "alt": altMeters, "speed": speed, "overlap": overlap]
        return try decoder.decode(PlanResponse.self, from: try await request("/api/plan", body: body))
    }

    /// Start the logger with a fake fixed position (camera test on the ground), or stop it.
    func logger(sim: String?) async throws {
        if let sim { _ = try await request("/api/logger/start", body: ["sim": sim], timeout: 10) }
        else { _ = try await request("/api/logger/stop", body: [:], timeout: 45) }
    }

    /// upload / start / pause / resume / rtl / land
    func command(_ name: String, confirm: Bool = false) async throws {
        _ = try await request("/api/\(name)", body: confirm ? ["confirm": true] : [:], timeout: name == "upload" || name == "start" ? 25 : 8)
    }
}
