import SwiftUI
import UIKit
import MapboxMaps

// Same look as the Litterbug app: dark green background, ink outlines, orange litter, doodle map.
extension Color {
    static let appBackground = Color(red: 28 / 255, green: 72 / 255, blue: 57 / 255)
    static let cardBackground = Color(red: 36 / 255, green: 88 / 255, blue: 70 / 255)
    static let ink = Color(red: 0.23, green: 0.21, blue: 0.19)
    static let litter = Color(red: 1.0, green: 0.36, blue: 0.2)
    static let path = Color(red: 0.30, green: 0.48, blue: 0.58)
    static let good = Color(red: 0.55, green: 0.85, blue: 0.45)
    static let caution = Color(red: 1.0, green: 0.78, blue: 0.3)
}

enum Doodle {
    static let style: MapStyle = {
        guard let url = Bundle.main.url(forResource: "DoodleStyle", withExtension: "json"),
              let json = try? String(contentsOf: url, encoding: .utf8) else { return .standard }
        return MapStyle(json: json)
    }()
}

enum Hatch {
    static func image(color: UIColor, size: CGFloat = 24, spacing: CGFloat = 8) -> UIImage {
        UIGraphicsImageRenderer(size: CGSize(width: size, height: size)).image { _ in
            let path = UIBezierPath()
            var c: CGFloat = -size
            while c <= size * 2 {
                path.move(to: CGPoint(x: c, y: 0))
                path.addLine(to: CGPoint(x: c - size, y: size))
                c += spacing
            }
            color.setStroke()
            path.lineWidth = 1.2
            path.stroke()
        }
    }
}

/// Round doodle badge: coloured circle, white ring, ink outline, slight random tilt, springs in.
struct DoodleBadge: View {
    let symbol: String
    var color: Color = .litter
    var size: CGFloat = 30
    var seed: String = ""
    @State private var appeared = false

    private var tilt: Double { Double(seed.unicodeScalars.reduce(0) { $0 + Int($1.value) * 7 } % 17 - 8) }

    var body: some View {
        Image(systemName: symbol)
            .font(.system(size: size * 0.44, weight: .bold))
            .foregroundStyle(.white)
            .frame(width: size, height: size)
            .background(color, in: Circle())
            .overlay(Circle().stroke(.white, lineWidth: size * 0.09))
            .overlay(Circle().inset(by: -2.5).stroke(Color.ink, lineWidth: 2))
            .rotationEffect(.degrees(tilt))
            .shadow(color: .black.opacity(0.3), radius: 2, x: 1, y: 2)
            .scaleEffect(appeared ? 1 : 0.1)
            .onAppear {
                withAnimation(.spring(response: 0.45, dampingFraction: 0.45).delay(.random(in: 0...0.2))) { appeared = true }
            }
    }
}

/// Card with the ink outline used around the map.
struct InkCard<Content: View>: View {
    var radius: CGFloat = 22
    @ViewBuilder var content: Content
    var body: some View {
        content
            .padding(12)
            .background(Color.cardBackground, in: RoundedRectangle(cornerRadius: radius, style: .continuous))
            .overlay(RoundedRectangle(cornerRadius: radius, style: .continuous).stroke(Color.ink, lineWidth: 2))
            .shadow(color: .black.opacity(0.15), radius: 10, y: 5)
    }
}

/// Chunky outlined button.
struct InkButtonStyle: ButtonStyle {
    var fill: Color = .white.opacity(0.12)
    var text: Color = .white
    func makeBody(configuration: Configuration) -> some View {
        InkButtonBody(configuration: configuration, fill: fill, text: text)
    }
}

private struct InkButtonBody: View {
    let configuration: ButtonStyleConfiguration
    let fill: Color
    let text: Color
    @Environment(\.isEnabled) private var enabled
    var body: some View {
        configuration.label
            .font(.subheadline.weight(.semibold))
            .foregroundStyle(text)
            .padding(.horizontal, 12).padding(.vertical, 9)
            .background(enabled ? fill : .white.opacity(0.06), in: Capsule())
            .overlay(Capsule().stroke(Color.ink, lineWidth: 2))
            .scaleEffect(configuration.isPressed ? 0.95 : 1)
            .opacity(enabled ? (configuration.isPressed ? 0.85 : 1) : 0.4)
    }
}

extension ButtonStyle where Self == InkButtonStyle {
    static var ink: InkButtonStyle { InkButtonStyle() }
    static var inkPrimary: InkButtonStyle { InkButtonStyle(fill: .litter) }
    static var inkDanger: InkButtonStyle { InkButtonStyle(fill: Color(red: 0.85, green: 0.25, blue: 0.22)) }
}

struct Chip: View {
    let text: String
    let color: Color
    var body: some View {
        Text(text).font(.caption.weight(.semibold)).lineLimit(1)
            .padding(.horizontal, 9).padding(.vertical, 4)
            .foregroundStyle(color)
            .background(color.opacity(0.14), in: Capsule())
            .overlay(Capsule().stroke(color.opacity(0.6), lineWidth: 1.5))
    }
}
