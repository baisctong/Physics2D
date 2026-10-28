#include "demos/BoxStack.h"

#include "Renderer.h"

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <string>

namespace p2 {
namespace {

constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kBoxCentroidColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr Color kImpactBallColor{0.95f, 0.30f, 0.30f, 1.0f};
constexpr Color kVelocityColor{0.35f, 0.90f, 0.40f, 1.0f};

// Half-width of the drawn ground line. The visible world at 1x zoom is 32 m wide on
// the default 1280 px window, so 16 m reaches both screen edges.
constexpr float kGroundHalfWidth = 16.0f;
constexpr float kMillimetersPerMeter = 1000.0f;
constexpr float kVelocityArrowScale = 0.10f;
// Arrows are skipped below this squared speed (m^2/s^2): a settling tower always has
// sub-cm/s correction noise and drawing it would fill the screen with meaningless
// arrows.
constexpr float kVelocityArrowThresholdSquared = 0.02f * 0.02f;

// Layer-colour ramp: the tower is tall, and a per-layer hue shift makes it obvious
// which boxes moved relative to each other when it starts to lean or topple. The layer
// count is passed in because the class constant stays private to the demo.
[[nodiscard]] Color LayerColor(int32_t layerIndex, int32_t layerCount) noexcept {
    const float t = static_cast<float>(layerIndex) / static_cast<float>(layerCount - 1);
    return Color{0.35f + 0.55f * t, 0.60f + 0.25f * (1.0f - t), 0.95f - 0.45f * t, 1.0f};
}

} // namespace

const char* BoxStackDemo::Name() const noexcept {
    return "Box Stack";
}

void BoxStackDemo::Setup(World& world) {
    m_resetCount = 0;
    m_boxes.fill(BodyHandle{});
    m_impactBalls.clear();
    CreateGround(world);
    CreateTower(world);
    std::printf("[BoxStack] static ground plane y=0 + %d dynamic boxes 1x1 m (hx=hy=0.5) at "
                "y=0.5..9.5, density 1, restitution 0. Week 4: no friction / warm starting yet - "
                "the tower is expected to collapse; Week 5 adds friction + warm starting\n",
                kLayerCount);
}

void BoxStackDemo::Update(World& /*world*/, float /*dt*/) {
    // Intentionally empty: no scripted motion and no auto-reset. A collapsing tower is
    // a real Week 4 result (and the W5 comparison baseline), not a state to hide.
}

void BoxStackDemo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, kGroundY}, {kGroundHalfWidth, kGroundY}, kGroundColor);

    for (int32_t layerIndex = 0; layerIndex < kLayerCount; ++layerIndex) {
        const BodyHandle handle = m_boxes[static_cast<std::size_t>(layerIndex)];
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

        // Draw the vertices the collision code actually stores, transformed by the
        // body's current transform. Reading them back instead of recomputing the
        // corners from half extents means a bug in the polygon construction is visible
        // on screen immediately.
        std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
        const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
        const Transform transform = world.GetTransform(handle);
        for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            worldVertices[static_cast<std::size_t>(vertexIndex)] =
                Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
        }
        renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount),
                             LayerColor(layerIndex, kLayerCount));
        renderer.DrawPoint(Mul(transform, shape->polygon.centroid), kBoxCentroidColor, 0.05f);

        const Vec2 velocity = world.GetLinearVelocity(handle);
        if (LengthSquared(velocity) > kVelocityArrowThresholdSquared) {
            const Vec2 position = world.GetPosition(handle);
            renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
        }
    }

    for (const BodyHandle ballHandle : m_impactBalls) {
        if (!world.IsValid(ballHandle)) {
            continue;
        }
        const Vec2 position = world.GetPosition(ballHandle);
        renderer.DrawCircle(position, kImpactBallRadius, kImpactBallColor, kImpactBallSegments);
        renderer.DrawPoint(position, kBoxCentroidColor, 0.05f);
        const Vec2 velocity = world.GetLinearVelocity(ballHandle);
        if (LengthSquared(velocity) > kVelocityArrowThresholdSquared) {
            renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
        }
    }
}

void BoxStackDemo::Ui(World& world) {
    ImGui::TextUnformatted("Ten 1x1 m boxes stacked on the ground plane (polygon SAT + clipping).");
    ImGui::TextUnformatted(
        "Week 4: no friction / warm starting yet - the tower is expected to collapse; Week 5 adds both.");

    int32_t touchingContacts = 0;
    float maxPenetration = 0.0f;
    for (const Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingContacts;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    ImGui::Text("Contacts: %d   max penetration: %.2f mm", touchingContacts,
                maxPenetration * kMillimetersPerMeter);

    // Per-layer readout: height and tilt. These are the two numbers that show the
    // Week 4 weakness directly (sinking and rocking), so they are printed per layer
    // instead of an aggregate.
    for (int32_t layerIndex = 0; layerIndex < kLayerCount; ++layerIndex) {
        const BodyHandle handle = m_boxes[static_cast<std::size_t>(layerIndex)];
        if (!world.IsValid(handle)) {
            ImGui::TextDisabled("Layer %2d: missing", layerIndex);
            continue;
        }
        const float centerY = world.GetPosition(handle).y;
        const float tiltDegrees = world.GetAngle(handle) * kRadToDeg;
        ImGui::Text("Layer %2d: y=%.3f m  tilt=%+.2f deg", layerIndex, centerY, tiltDegrees);
    }

    if (ImGui::Button("Reset tower")) {
        Reset(world);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(static_cast<int32_t>(m_impactBalls.size()) >= kMaxImpactBalls);
    if (ImGui::Button("Drop impact ball")) {
        CreateImpactBall(world);
    }
    ImGui::EndDisabled();
    ImGui::Text("Resets: %d   impact balls: %d", m_resetCount, static_cast<int32_t>(m_impactBalls.size()));
}

void BoxStackDemo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() invalidates every
    // handle, including the impact balls spawned during play.
    world.Clear();
    m_boxes.fill(BodyHandle{});
    m_impactBalls.clear();
    CreateGround(world);
    CreateTower(world);
    ++m_resetCount;
}

void BoxStackDemo::CreateGround(World& world) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Static;
    bodyDef.position = {0.0f, kGroundY};
    bodyDef.debugName = "ground";
    m_ground = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ground)) {
        std::fprintf(stderr, "[BoxStack] World::CreateBody rejected the ground body\n");
        return;
    }

    // PlaneShape::normal points out of the solid toward free space: (0,1) + offset 0
    // is the half-space y < 0, so the ground surface is exactly y = 0. This block is
    // duplicated across the early demos on purpose; the shared SceneBuilders.h the
    // design document schedules for Week 6 will absorb it.
    const ShapeHandle planeShape = world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f);
    if (!world.IsValidShape(planeShape) || !world.AttachShape(m_ground, planeShape)) {
        std::fprintf(stderr, "[BoxStack] failed to create/attach the ground plane shape\n");
    }
}

void BoxStackDemo::CreateTower(World& world) {
    for (int32_t layerIndex = 0; layerIndex < kLayerCount; ++layerIndex) {
        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        bodyDef.position = {0.0f, kFirstLayerCenterY + kLayerSpacing * static_cast<float>(layerIndex)};
        bodyDef.density = kDensity;
        // Restitution 0: a resting tower must not bounce; all energy loss in Week 4
        // comes from the solver's inelastic normal impulses.
        bodyDef.restitution = 0.0f;
        bodyDef.debugName = "box_" + std::to_string(layerIndex);
        const BodyHandle handle = world.CreateBody(bodyDef);
        if (!world.IsValid(handle)) {
            std::fprintf(stderr, "[BoxStack] World::CreateBody rejected box layer %d\n", layerIndex);
            continue;
        }

        // CreateBoxShape builds the CCW polygon and AttachShape derives mass = rho * area
        // (1 kg for a 1 m x 1 m box at density 1) and the box inertia.
        const ShapeHandle boxShape = world.CreateBoxShape(kHalfExtent, kHalfExtent);
        if (!world.IsValidShape(boxShape) || !world.AttachShape(handle, boxShape)) {
            std::fprintf(stderr, "[BoxStack] failed to create/attach box layer %d shape\n", layerIndex);
            continue;
        }
        m_boxes[static_cast<std::size_t>(layerIndex)] = handle;
    }
}

void BoxStackDemo::CreateImpactBall(World& world) {
    if (static_cast<int32_t>(m_impactBalls.size()) >= kMaxImpactBalls) {
        return;
    }
    BodyDef bodyDef;
    bodyDef.type = BodyType::Dynamic;
    bodyDef.position = {0.0f, kImpactBallSpawnY};
    bodyDef.density = kDensity;
    bodyDef.restitution = 0.0f;
    bodyDef.debugName = "impact_ball_" + std::to_string(static_cast<int32_t>(m_impactBalls.size()));
    const BodyHandle handle = world.CreateBody(bodyDef);
    if (!world.IsValid(handle)) {
        std::fprintf(stderr, "[BoxStack] World::CreateBody rejected the impact ball\n");
        return;
    }
    const ShapeHandle circleShape = world.CreateCircleShape(kImpactBallRadius);
    if (!world.IsValidShape(circleShape) || !world.AttachShape(handle, circleShape)) {
        std::fprintf(stderr, "[BoxStack] failed to create/attach the impact ball shape\n");
        return;
    }
    m_impactBalls.push_back(handle);
}

} // namespace p2
