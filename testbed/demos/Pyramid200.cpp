#include "demos/Pyramid200.h"

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

// Height-keyed colour ramp: the pyramid is 20 layers tall, and a per-height hue shift
// makes it obvious which layers moved relative to each other when the pile settles.
[[nodiscard]] Color HeightColor(float normalizedHeight) noexcept {
    const float t = Clamp(normalizedHeight, 0.0f, 1.0f);
    return Color{0.35f + 0.55f * t, 0.60f + 0.25f * (1.0f - t), 0.95f - 0.45f * t, 1.0f};
}

} // namespace

const char* Pyramid200Demo::Name() const noexcept {
    return "Pyramid 200";
}

void Pyramid200Demo::Setup(World& world) {
    // Fresh-scene entry point: the panel's layer count starts from the shipped default.
    m_baseCount = kDefaultBaseCount;
    m_resetCount = 0;
    m_ground = BodyHandle{};
    m_boxes.clear();
    BuildScene(world);
}

void Pyramid200Demo::BuildScene(World& world) {
    m_ground = CreateGroundPlane(world, 0.0f);
    m_boxes = CreateBoxPyramid(world, m_baseCount, kBoxHalfExtent);

    const int32_t expectedCount = m_baseCount * (m_baseCount + 1) / 2;
    const float sceneHeight = 2.0f * kBoxHalfExtent * static_cast<float>(m_baseCount);
    std::printf("[Pyramid200] static ground plane y=0 + perfect pyramid baseCount=%d -> %d boxes of 1x1 m "
                "(%d layers, %.1f m tall), default solver settings (8/3 iterations); name keeps the "
                "design-document ~200-box target\n",
                m_baseCount, expectedCount, m_baseCount, static_cast<double>(sceneHeight));
    if (static_cast<int32_t>(m_boxes.size()) != expectedCount) {
        std::fprintf(stderr, "[Pyramid200] expected %d boxes but the builder created %d (see stderr above)\n",
                     expectedCount, static_cast<int32_t>(m_boxes.size()));
    }
}

void Pyramid200Demo::Update(World& world, float /*dt*/) {
    // Safety net only: an infinite ground plane can never be tunnelled through in the
    // default configuration, so a box below this line means a physics regression. Report
    // it loudly and rebuild instead of letting the scene silently fall apart.
    for (const BodyHandle handle : m_boxes) {
        if (!world.IsValid(handle)) {
            continue;
        }
        if (world.GetPosition(handle).y < kFallThroughResetY) {
            std::fprintf(stderr,
                         "[Pyramid200] box fell through the ground plane (y < %.1f) - physics "
                         "regression, rebuilding the scene\n",
                         kFallThroughResetY);
            Reset(world);
            return;
        }
    }
}

void Pyramid200Demo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, 0.0f}, {kGroundHalfWidth, 0.0f}, kGroundColor);

    const float sceneHeight = 2.0f * kBoxHalfExtent * static_cast<float>(m_baseCount);
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
        // Draw the vertices the collision code actually stores (same rationale as
        // BoxStack): a construction bug must be visible instead of being masked by
        // recomputing corners from the half extents.
        std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
        const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
        const Transform transform = world.GetTransform(handle);
        for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            worldVertices[static_cast<std::size_t>(vertexIndex)] =
                Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
        }
        renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount),
                             HeightColor(world.GetPosition(handle).y / sceneHeight));
        renderer.DrawPoint(Mul(transform, shape->polygon.centroid), kCentroidColor, 0.03f);
    }
}

void Pyramid200Demo::Ui(World& world) {
    ImGui::TextUnformatted("Perfect 2D pyramid: layer k holds baseCount-k 1 m boxes, centred.");
    ImGui::TextUnformatted(
        "Name keeps the design-document target (~200 OBBs); default is 20 layers = 210 boxes.");

    const int32_t expectedCount = m_baseCount * (m_baseCount + 1) / 2;
    ImGui::Text("Boxes: %d / expected %d   Bodies: %d   Shapes: %d", static_cast<int32_t>(m_boxes.size()),
                expectedCount, world.GetBodyCount(), world.GetShapeCount());

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
    const int32_t candidatePairs = world.GetBroadPhasePairCount();
    const IBroadPhase* broadPhase = world.GetBroadPhase();
    ImGui::Text("Contacts: %d (points %d)   max penetration: %.2f mm", touchingContacts, contactPoints,
                maxPenetration * kMillimetersPerMeter);
    ImGui::Text("Broad phase: %s   candidates: %d   proxies: %d",
                broadPhase != nullptr ? broadPhase->Name() : "None", candidatePairs,
                broadPhase != nullptr ? broadPhase->GetProxyCount() : 0);

    // Aggregate standing metrics. The top height ratio uses the spawn centre height of
    // the apex box (baseCount - 0.5 m), so a settled pyramid that sank or slid reads
    // below 1.0 without needing the pristine layout.
    float maxAbsTiltDegrees = 0.0f;
    float highestCenterY = 0.0f;
    for (const BodyHandle handle : m_boxes) {
        if (!world.IsValid(handle)) {
            continue;
        }
        maxAbsTiltDegrees = std::max(maxAbsTiltDegrees, std::fabs(world.GetAngle(handle) * kRadToDeg));
        highestCenterY = std::max(highestCenterY, world.GetPosition(handle).y);
    }
    const float expectedTopCenterY = static_cast<float>(m_baseCount) - 0.5f;
    const float topHeightRatio = expectedTopCenterY > 0.0f ? highestCenterY / expectedTopCenterY : 0.0f;
    ImGui::Text("Max |tilt|: %.2f deg   top centre y: %.3f m (ratio %.3f)", maxAbsTiltDegrees, highestCenterY,
                topHeightRatio);

    const bool standing = static_cast<int32_t>(m_boxes.size()) == expectedCount && touchingContacts > 0 &&
                          maxPenetration < kStandingMaxPenetration &&
                          maxAbsTiltDegrees < kStandingMaxTiltDegrees &&
                          topHeightRatio >= kStandingMinTopHeightRatio;
    if (standing) {
        ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.40f, 1.0f), "Verdict: STANDING (default settings)");
    } else {
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.30f, 1.0f), "Verdict: NOT STABLE (see numbers above)");
    }

    ImGui::Separator();
    ImGui::SliderInt("Base count (layers)", &m_baseCount, kMinBaseCount, kMaxBaseCount, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) {
        // Rebuild is an explicit user action (not a per-frame body rebuild): it applies
        // the layer count, keeps the engine's default solver settings untouched.
        Reset(world);
    }
    ImGui::TextDisabled("Rebuild applies the layer count; solver settings stay at the engine defaults.");

    if (ImGui::Button("Reset pyramid")) {
        Reset(world);
    }
    ImGui::Text("Resets: %d", m_resetCount);
}

void Pyramid200Demo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() invalidates every
    // handle, so the ground and the pyramid are recreated below.
    world.Clear();
    m_ground = BodyHandle{};
    m_boxes.clear();
    ++m_resetCount;
    BuildScene(world);
}

} // namespace p2
