import SwiftUI
import UIKit
import MapboxMaps

/// The app stays portrait; only the full-screen camera turns sideways.
enum Orientation {
    static var allowed: UIInterfaceOrientationMask = .portrait

    @MainActor static func set(_ mask: UIInterfaceOrientationMask) {
        allowed = mask
        guard let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene else { return }
        scene.windows.first?.rootViewController?.setNeedsUpdateOfSupportedInterfaceOrientations()
        scene.requestGeometryUpdate(.iOS(interfaceOrientations: mask)) { _ in }
    }
}

final class AppDelegate: NSObject, UIApplicationDelegate {
    func application(_ application: UIApplication, supportedInterfaceOrientationsFor window: UIWindow?) -> UIInterfaceOrientationMask {
        Orientation.allowed
    }
}

@main
struct OperatorApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) private var appDelegate
    @State private var op = OperatorModel()
    @State private var tab = ProcessInfo.processInfo.arguments.contains("-cameraTab") ? 1 : (ProcessInfo.processInfo.arguments.contains("-flightsTab") ? 2 : 0)   // launch args for testing

    init() {
        MapboxOptions.accessToken = Secrets.mapboxToken
    }

    var body: some Scene {
        WindowGroup {
            TabView(selection: $tab) {
                OperatorView()
                    .tabItem { Label("Map", systemImage: "map.fill") }.tag(0)
                CameraView()
                    .tabItem { Label("Camera", systemImage: "video.fill") }.tag(1)
                FlightsView()
                    .tabItem { Label("Flights", systemImage: "clock.arrow.circlepath") }.tag(2)
            }
                .environment(op)
                .tint(.white)
                .preferredColorScheme(.dark)
        }
    }
}
