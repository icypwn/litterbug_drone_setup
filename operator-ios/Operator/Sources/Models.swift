import Foundation
import CoreLocation

/// One distinct piece of litter found by the drone.
struct LitterPiece: Identifiable, Codable, Hashable {
    let id: String
    let latitude: Double
    let longitude: Double
    var sightings: Int
    var confidence: Double
    var cropURL: URL?
    var firstSeen: Date
    /// new = just added to Firebase, known = already there from an earlier flight, pending = not checked yet,
    /// local = no Firebase for this run (camera test). nil for older data.
    var status: String? = nil

    /// After landing the Pi checks Firebase: "known" means it was already logged on an earlier flight.
    var isKnown: Bool { status == "known" }

    var coordinate: CLLocationCoordinate2D { .init(latitude: latitude, longitude: longitude) }
    var location: CLLocation { .init(latitude: latitude, longitude: longitude) }
}

/// A completed pickup, kept on the phone for the points history.
struct Pickup: Identifiable, Codable, Hashable {
    let id: UUID
    let pieceID: String
    let date: Date
    let points: Int
    let distanceMeters: Double
}

/// GeoJSON from the drone's /api/pieces.
struct PiecesResponse: Decodable {
    struct Feature: Decodable {
        struct Geometry: Decodable { let coordinates: [Double] }
        struct Props: Decodable {
            let id: String
            let sightings: Int
            let maxConf: Double
            let firstSeen: Double
            let crop: String?
            let status: String?
        }
        let geometry: Geometry
        let properties: Props
    }
    let features: [Feature]

    func pieces(base: URL?) -> [LitterPiece] {
        features.compactMap { f in
            guard f.geometry.coordinates.count >= 2 else { return nil }
            return LitterPiece(id: f.properties.id, latitude: f.geometry.coordinates[1], longitude: f.geometry.coordinates[0],
                               sightings: f.properties.sightings, confidence: f.properties.maxConf,
                               cropURL: f.properties.crop.flatMap { c in base.map { $0.appendingPathComponent(c) } },
                               firstSeen: Date(timeIntervalSince1970: f.properties.firstSeen),
                               status: f.properties.status)
        }
    }
}

/// /api/status from the drone's mission server. Every drone field is optional:
/// without a drone connected only `link` and the plan/logger fields are present.
struct DroneStatus: Decodable {
    var link: String
    var armed: Bool?
    var mode: String?
    var lat: Double?
    var lon: Double?
    var alt: Double?
    var heading: Int?
    var speed: Double?
    var gpsFix: Int?
    var sats: Int?
    var battery: Int?
    var voltage: Double?
    var missionSeq: Int?
    var missionCount: Int?
    var messages: [String]?
    var plan: Bool
    var uploaded: Bool
    var logger: Bool
    var live: Bool?
    var run: String?
    var pieces: Int
    var planMinutes: Double?
    var homeLat: Double?
    var homeLon: Double?

    var connected: Bool { link == "ok" }
    var home: CLLocationCoordinate2D? {
        guard let homeLat, let homeLon else { return nil }
        return .init(latitude: homeLat, longitude: homeLon)
    }
    var position: CLLocationCoordinate2D? {
        guard let lat, let lon else { return nil }
        return .init(latitude: lat, longitude: lon)
    }
}

/// /api/plan response: the lawnmower path and what the camera covers.
struct PlanResponse: Decodable {
    enum Stat: Decodable {
        case text(String), number(Double)
        init(from d: Decoder) throws {
            let c = try d.singleValueContainer()
            if let s = try? c.decode(String.self) { self = .text(s) } else { self = .number(try c.decode(Double.self)) }
        }
        var display: String {
            switch self {
            case .text(let s): return s
            case .number(let n): return n == n.rounded() ? String(Int(n)) : String(format: "%.1f", n)
            }
        }
    }
    let polygon: [[Double]]
    let waypoints: [[Double]]
    let swaths: [[[Double]]]
    let stats: [String: Stat]
    let minutes: Double

    var path: [CLLocationCoordinate2D] { waypoints.map { .init(latitude: $0[0], longitude: $0[1]) } }
    var swathPolygons: [[CLLocationCoordinate2D]] { swaths.map { $0.map { .init(latitude: $0[0], longitude: $0[1]) } } }

    static let statOrder = ["Area", "Passes", "Pass spacing", "Flight time", "Path length", "Camera covers", "Height"]
}

struct APIError: LocalizedError {
    let message: String
    var errorDescription: String? { message }
}
