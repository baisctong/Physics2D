#include "demos/StressBenchmark.h"

#include "Renderer.h"
#include "SceneBuilders.h"

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace p2 {
namespace {

constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kCentroidColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr float kMillimetersPerMeter = 1000.0f;
// Velocity that maps to the "hot" end of the box colour ramp (m/s).
constexpr float kColorHotSpeed = 6.0f;

// Speed-keyed colour ramp: blue while a box is resting, orange while it is moving.
// In a 400-box stress field the eye cannot track individual wireframes, so motion is
// the useful visible signal.
[[nodiscard]] Color SpeedColor(float speed) noexcept {
    const float t = Clamp(speed / kColorHotSpeed, 0.0f, 1.0f);
    return Color{0.35f + 0.60f * t, 0.55f - 0.25f * t, 0.95f - 0.70f * t, 1.0f};
}

} // namespace

const char* StressBenchmarkDemo::Name() const noexcept {
    return "Stress Benchmark";
}

void StressBenchmarkDemo::Setup(World& world) {
    m_boxCount = kDefaultBoxCount;
    m_resetCount = 0;
    m_ground = BodyHandle{};
    m_boxes.clear();
    BuildScene(world);
}

void StressBenchmarkDemo::BuildScene(World& world) {
    m_ground = CreateGroundPlane(world, 0.0f);
    m_boxes = CreateRandomBoxField(world, m_boxCount, kFieldSeed, kFieldHalfExtent, kMaxBoxHalfExtent,
                                   kMaxInitialSpeed);

    std::printf("[StressBenchmark] static ground y=0 + %d random boxes (seed %u, sizes 0.2-1.0 m, random "
                "angles, initial speed <= %.1f m/s, spawn area +-%.0f m); broad-phase/solver stress scene\n",
                static_cast<int32_t>(m_boxes.size()), kFieldSeed, static_cast<double>(kMaxInitialSpeed),
                static_cast<double>(kFieldHalfExtent));
}

void StressBenchmarkDemo::SetFrameStats(const DemoFrameStats& stats) {
    m_frameStats = stats;
}

void StressBenchmarkDemo::Update(World& world, float /*dt*/) {
    // Safety net only: the ground is an infinite half-space, so a box below this line
    // means a physics regression (NaN / tunnel). Report it and rebuild the field.
    for (const BodyHandle handle : m_boxes) {
        if (!world.IsValid(handle)) {
            continue;
        }
        if (world.GetPosition(handle).y < kFallThroughResetY) {
            std::fprintf(
                stderr,
                "[StressBenchmark] a box fell below the ground plane (y < %.1f) - physics regression, "
                "rebuilding the field\n",
                kFallThroughResetY);
            Reset(world);
            return;
        }
    }
}

void StressBenchmarkDemo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, 0.0f}, {kGroundHalfWidth, 0.0f}, kGroundColor);

    for (const BodyHandle handle : m_boxes) {
        if (!world.IsValid(handle)) {
            continue;
        }
        const Body* body = world.GetBody(handle);
        if (body == nullptr || !world.IsValidShape(body->shape)) {
            continue;
        }
        const Shape* shape = world.GetShape(body->shape);
        if (shape == nullptr || shape->type != ShapeType::Polygon) {
            continue;
        }
        // Draw the stored polygon vertices, transformed by the current body transform.
        std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
        const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
        const Transform transform = world.GetTransform(handle);
        for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            worldVertices[static_cast<std::size_t>(vertexIndex)] =
                Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
        }
        const float speed = Length(world.GetLinearVelocity(handle));
        renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount), SpeedColor(speed));
        renderer.DrawPoint(Mul(transform, shape->polygon.centroid), kCentroidColor, 0.02f);
    }
}

void StressBenchmarkDemo::Ui(World& world) {
    ImGui::TextUnformatted("400 random OBBs: broad-phase comparison + solver stress scene.");
    ImGui::Text("Seed: %u   Boxes: %d   Bodies: %d   Shapes: %d", kFieldSeed,
                static_cast<int32_t>(m_boxes.size()), world.GetBodyCount(), world.GetShapeCount());

    int32_t touchingContacts = 0;
    int32_t contactPoints = 0;
    float maxPenetration = 0.0f;
    for (const Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingContacts;
        contactPoints += contact.manifold.pointCount;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }

    // The numbers the Week 6 milestone cares about: how many candidates the broad phase
    // hands to the narrow phase for this many bodies, and how much proxies it keeps.
    const IBroadPhase* broadPhase = world.GetBroadPhase();
    ImGui::Text("Broad phase: %s", broadPhase != nullptr ? broadPhase->Name() : "None (O(N^2) fallback)");
    ImGui::Text("Proxies: %d   Candidate pairs: %d", broadPhase != nullptr ? broadPhase->GetProxyCount() : 0,
                world.GetBroadPhasePairCount());
    ImGui::Text("Contacts: %d (points %d)   max penetration: %.2f mm", touchingContacts, contactPoints,
                maxPenetration * kMillimetersPerMeter);

    // Timings come from main.cpp's smoothed timers through Demo::SetFrameStats; a demo
    // cannot time World::Step itself because the host loop owns the step call.
    ImGui::Separator();
    ImGui::Text("FPS: %.1f   Step: %.3f ms", static_cast<double>(m_frameStats.framesPerSecond),
                static_cast<double>(m_frameStats.stepMilliseconds));
    ImGui::TextDisabled("Switch broad phase in Solver Settings -> Broad phase; the scene is not rebuilt.");

    ImGui::Separator();
    ImGui::SliderInt("Box count", &m_boxCount, kMinBoxCount, kMaxBoxCount, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) {
        // Explicit rebuild applies the requested count; the broad phase and solver
        // settings survive (World::Clear releases proxies but keeps the implementation).
        Reset(world);
    }
    if (ImGui::Button("Reset field")) {
        Reset(world);
    }
    ImGui::Text("Resets: %d", m_resetCount);
}

void StressBenchmarkDemo::Reset(World& world) {
    world.Clear();
    m_ground = BodyHandle{};
    m_boxes.clear();
    ++m_resetCount;
    BuildScene(world);
}

} // namespace p2
