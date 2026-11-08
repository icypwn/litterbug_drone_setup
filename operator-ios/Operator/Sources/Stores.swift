import Foundation
import CoreLocation
import Observation

enum Pref {
    static let droneHost = "droneHost"
    static let defaultHost = "litterbug.local:8080"
}

/// The area being drawn, the planned path and the live drone status.
@MainActor @Observable
final class OperatorModel {
    var corners: [CLLocationCoordinate2D] = []
    var plan: PlanResponse?
    var status: DroneStatus?
    var reachable = false
    var pieces: [LitterPiece] = []
    var message: String?
    var messageIsError = false
    var busy = false

    func addCorner(_ c: CLLocationCoordinate2D) { corners.append(c); plan = nil; message = nil }
    func moveCorner(_ i: Int, to c: CLLocationCoordinate2D) { guard corners.indices.contains(i) else { return }; corners[i] = c; plan = nil; message = nil }
    func removeCorner(_ i: Int) { guard corners.indices.contains(i) else { return }; corners.remove(at: i); plan = nil; message = nil }
    func undo() { if !corners.isEmpty { corners.removeLast() }; plan = nil; message = nil }
    func clear() { corners = []; plan = nil; message = nil }

    private func say(_ text: String, error: Bool = false) { message = text; messageIsError = error }

    func poll(host: String) async {
        do {
            status = try await DroneClient(host: host).status()
            reachable = true
        } catch {
            reachable = false
        }
    }

    func pollPieces(host: String) async {
        if let p = try? await DroneClient(host: host).pieces() { pieces = p }
    }

    func makePlan(host: String, heightFeet: Double, speed: Double, overlapPercent: Double) async {
        guard corners.count >= 3 else { return say("Add at least 3 corners first.", error: true) }
        busy = true; defer { busy = false }
        do {
            plan = try await DroneClient(host: host).plan(corners: corners, altMeters: heightFeet / 3.281,
                                                          speed: speed, overlap: overlapPercent / 100)
            say("Path ready. Upload it to the drone next.")
        } catch { say(error.localizedDescription, error: true) }
    }

    func loggerTest(host: String, sim: String?) async {
        busy = true; defer { busy = false }
        do {
            try await DroneClient(host: host).logger(sim: sim)
            say(sim == nil ? "Camera test stopped." : "Camera test running: video appears in a few seconds.")
            await poll(host: host)
        } catch { say(error.localizedDescription, error: true) }
    }

    func send(_ command: String, host: String, confirm: Bool = false, ok: String) async {
        busy = true; defer { busy = false }
        do {
            try await DroneClient(host: host).command(command, confirm: confirm)
            say(ok)
            await poll(host: host)
        } catch { say(error.localizedDescription, error: true) }
    }
}
