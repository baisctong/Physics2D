#include "demos/FallingBall.h"

#include "Renderer.h"

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Rot.h"

#include <imgui.h>

#include <cstdio>

namespace p2 {
namespace {

constexpr Color kBallColor{0.95f, 0.55f, 0.20f, 1.0f};
constexpr Color kBallCenterColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kSquareColor{0.35f, 0.62f, 1.00f, 1.0f};
constexpr Color kVelocityColor{0.35f, 0.90f, 0.40f, 1.0f};

// Half-width of the drawn ground line. The visible world at 1x zoom is 32 m wide
// on the default 1280 px window, so 16 m reaches both screen edges.
constexpr float kGroundHalfWidth = 16.0f;
constexpr int32_t kCircleSegments = 48;

} // namespace

const char* FallingBallDemo::Name() const noexcept {
    return "Falling Ball";
}

void FallingBallDemo::Setup(World& world) {
    m_squareAngle = 0.0f;
    m_spinSpeed = kDefaultSpinSpeed;
    m_resetCount = 0;
    CreateBall(world);
    std::printf("[FallingBall] one dynamic circle at (0, %.1f), mass 1 kg, gravity (0, %.1f); "
                "Week 1 has no contacts, so the ground line at y = %.1f is visual only and the "
                "scene auto-resets when the ball falls through it\n",
                kDropHeight, world.GetGravity().y, kGroundY);
}

void FallingBallDemo::Update(World& world, float dt) {
    // The rotating square is pure renderer state (D5 check), so it is advanced by
    // the same dt that the physics consumed; while paused the caller stops calling
    // Update, which freezes this animation together with the simulation.
    m_squareAngle += dt * m_spinSpeed;
    if (m_squareAngle > kTwoPi) {
        // Wrap instead of letting the angle grow unbounded: after minutes of
        // simulation large floats lose fractional precision and the square would
        // visibly stutter.
        m_squareAngle -= kTwoPi;
    }

    if (!world.IsValid(m_ball)) {
        return;
    }
    if (world.GetPosition(m_ball).y < kGroundY) {
        Reset(world);
        std::printf("[FallingBall] ball fell below y = %.1f - scene rebuilt (auto-reset #%d)\n", kGroundY,
                    m_resetCount);
    }
}

void FallingBallDemo::Draw(Renderer& renderer, const World& world) {
    // The ground is a visual reference for where Week 2's contact solver will stop
    // the ball; today the body simply passes through it.
    renderer.DrawSegment({-kGroundHalfWidth, kGroundY}, {kGroundHalfWidth, kGroundY}, kGroundColor);

    if (world.IsValid(m_ball)) {
        // Draw from the stable accessor, not from Body internals: the Week 4
        // hot/cold split will move the storage without changing this API.
        const Vec2 position = world.GetPosition(m_ball);
        renderer.DrawCircle(position, kBallRadius, kBallColor, kCircleSegments);
        renderer.DrawPoint(position, kBallCenterColor, 0.05f);
        // Velocity arrow: makes the semi-implicit Euler acceleration visible at a
        // glance (the arrow grows by g * dt every step until the reset).
        const Vec2 velocity = world.GetLinearVelocity(m_ball);
        renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
    }

    // Rotating square: the D5 renderer check. It exercises Rot/Mul and the polygon
    // path without involving the Physics: if the square spins smoothly while the
    // ball falls, rendering and simulation are wired together correctly.
    const Rot rotation(m_squareAngle);
    constexpr Vec2 kLocalCorners[4] = {{-kSquareHalfExtent, -kSquareHalfExtent},
                                       {kSquareHalfExtent, -kSquareHalfExtent},
                                       {kSquareHalfExtent, kSquareHalfExtent},
                                       {-kSquareHalfExtent, kSquareHalfExtent}};
    const Vec2 squareCenter{kSquareCenterX, kSquareCenterY};
    Vec2 worldCorners[4];
    for (int32_t cornerIndex = 0; cornerIndex < 4; ++cornerIndex) {
        worldCorners[cornerIndex] = Mul(rotation, kLocalCorners[cornerIndex]) + squareCenter;
    }
    renderer.DrawPolygon(worldCorners, 4, kSquareColor);
    renderer.DrawPoint(squareCenter, kSquareColor, 0.05f);
}

void FallingBallDemo::Ui(World& world) {
    ImGui::TextUnformatted("One dynamic circle under gravity (no shapes/contacts in Week 1).");
    if (world.IsValid(m_ball)) {
        const Vec2 position = world.GetPosition(m_ball);
        const Vec2 velocity = world.GetLinearVelocity(m_ball);
        ImGui::Text("Ball y: %.3f m   vy: %.3f m/s", position.y, velocity.y);
    } else {
        ImGui::TextDisabled("Ball body is missing");
    }
    ImGui::SliderFloat("Square spin (rad/s)", &m_spinSpeed, 0.0f, 6.0f, "%.2f");
    ImGui::Text("Auto-resets: %d", m_resetCount);
    if (ImGui::Button("Drop ball now")) {
        Reset(world);
    }
}

void FallingBallDemo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() also resets the
    // body handles, so the old m_ball becomes stale and is replaced below.
    world.Clear();
    m_squareAngle = 0.0f;
    CreateBall(world);
    ++m_resetCount;
}

void FallingBallDemo::CreateBall(World& world) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Dynamic;
    bodyDef.position = {0.0f, kDropHeight};
    bodyDef.mass = 1.0f;
    // Week 1 has no shape geometry, so mass properties are explicit. The disc
    // inertia I = 1/2 m r^2 is supplied instead of leaving the arbitrary default:
    // it becomes correct the moment a circle shape is attached in Week 2.
    bodyDef.inertia = 0.5f * bodyDef.mass * kBallRadius * kBallRadius;
    // allowSleep stays at its default (true): the brief asks for the default value
    // so the Week 7 sleeper is exercised by the very first scene.
    bodyDef.debugName = "falling_ball";
    m_ball = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ball)) {
        std::fprintf(stderr, "[FallingBall] World::CreateBody rejected the ball definition\n");
    }
}

} // namespace p2
