#include "demos/FrictionRamp.h"

#include "Renderer.h"

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/BodyDef.h"
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

constexpr Color kRampColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kHatchColor{0.38f, 0.42f, 0.48f, 1.0f};
constexpr Color kBlockColor{0.98f, 0.62f, 0.25f, 1.0f};
constexpr Color kBlockCentroidColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr Color kVelocityColor{0.35f, 0.90f, 0.40f, 1.0f};

constexpr float kVelocityArrowScale = 0.25f; // sliding speeds are ~1-3 m/s, so a
                                             // longer scale than the stack demos
constexpr float kVelocityArrowThresholdSquared = 0.01f * 0.01f;
constexpr float kMillimetersPerMeter = 1000.0f;

// Surface hatching: short strokes on the solid side of the drawn slope so the
// incline reads as ground rather than as a free-floating line.
constexpr float kHatchSpacing = 2.0f;
constexpr float kHatchLength = 0.35f;

// Unit vector up the slope (+x rotated by theta) and the surface normal (out of the
// solid half-space). Both follow the engine convention: the plane's local normal is
// (0,1) and the body rotation carries it into world space.
[[nodiscard]] Vec2 SlopeDirection(float angleRadians) noexcept {
    return Vec2{std::cos(angleRadians), std::sin(angleRadians)};
}

[[nodiscard]] Vec2 RampNormal(float angleRadians) noexcept {
    return Vec2{-std::sin(angleRadians), std::cos(angleRadians)};
}

} // namespace

const char* FrictionRampDemo::Name() const noexcept {
    return "Friction Ramp";
}

void FrictionRampDemo::Setup(World& world) {
    // Fresh-scene entry point: the sliders start from their defaults. Reset() below
    // keeps the current theta/mu, mirroring FallingBall's restitution behaviour.
    m_inclineDegrees = kDefaultInclineDegrees;
    m_friction = kDefaultFriction;
    m_resetCount = 0;
    m_ground = BodyHandle{};
    m_block = BodyHandle{};
    CreateRamp(world);
    CreateBlock(world);

    const float tanTheta = std::tan(m_inclineDegrees * kDegToRad);
    const bool predictedStatic = m_friction > tanTheta;
    std::printf("[FrictionRamp] static inclined plane theta=%.1fdeg (tan=%.3f) + dynamic box 1x1 m "
                "hx=hy=0.5, friction mu=%.3f; predicted %s (mu %s tan(theta)); infinite half-space, "
                "Reset re-seats the box\n",
                m_inclineDegrees, tanTheta, m_friction, predictedStatic ? "STATIC" : "SLIDING",
                predictedStatic ? ">" : "<=");
}

void FrictionRampDemo::Update(World& world, float /*dt*/) {
    // Intentionally empty apart from the safety net: the scene has no scripted motion
    // and no auto-reset while the box is on screen (a sliding box is the measurement).
    if (!world.IsValid(m_block)) {
        return;
    }
    if (world.GetPosition(m_block).y < kFallThroughResetY) {
        // Only reachable after a very long slide down the infinite half-space (or a
        // physics regression that teleports the box): report it instead of silently
        // losing the body below the visible world.
        std::fprintf(stderr, "[FrictionRamp] box left the visible world (y < %.1f); resetting scene\n",
                     kFallThroughResetY);
        Reset(world);
    }
}

void FrictionRampDemo::Draw(Renderer& renderer, const World& world) {
    const float angleRadians =
        world.IsValid(m_ground) ? world.GetAngle(m_ground) : m_inclineDegrees * kDegToRad;
    const Vec2 slopeDirection = SlopeDirection(angleRadians);
    const Vec2 normal = RampNormal(angleRadians);

    renderer.DrawSegment(slopeDirection * -kRampHalfExtent, slopeDirection * kRampHalfExtent, kRampColor);
    for (float distance = -kRampHalfExtent; distance <= kRampHalfExtent + 0.001f; distance += kHatchSpacing) {
        const Vec2 surfacePoint = slopeDirection * distance;
        renderer.DrawSegment(surfacePoint, surfacePoint - normal * kHatchLength, kHatchColor);
    }

    if (!world.IsValid(m_block)) {
        return;
    }
    const Body* body = world.GetBody(m_block);
    if (body == nullptr || !world.IsValidShape(body->shape)) {
        return;
    }
    const Shape* shape = world.GetShape(body->shape);
    if (shape == nullptr || shape->type != ShapeType::Polygon) {
        return;
    }

    // Draw what the collision code stores, not what the demo thinks it spawned: the
    // same rationale as BoxStack, a construction bug must be visible on screen.
    std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
    const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
    const Transform transform = world.GetTransform(m_block);
    for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        worldVertices[static_cast<std::size_t>(vertexIndex)] =
            Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
    }
    renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount), kBlockColor);
    renderer.DrawPoint(Mul(transform, shape->polygon.centroid), kBlockCentroidColor, 0.05f);

    // Velocity arrow: the direction is down the slope when friction loses, so the
    // arrow is the quickest check that the friction cone was actually applied.
    const Vec2 velocity = world.GetLinearVelocity(m_block);
    if (LengthSquared(velocity) > kVelocityArrowThresholdSquared) {
        const Vec2 position = world.GetPosition(m_block);
        renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
    }
}

void FrictionRampDemo::Ui(World& world) {
    ImGui::TextUnformatted("Box on an adjustable incline: static iff mu > tan(theta).");

    if (!world.IsValid(m_ground) || !world.IsValid(m_block)) {
        ImGui::TextDisabled("Ramp/box body is missing");
        return;
    }

    float inclineDegrees = m_inclineDegrees;
    if (ImGui::SliderFloat("Incline theta (deg)", &inclineDegrees, kMinInclineDegrees, kMaxInclineDegrees,
                           "%.1f", ImGuiSliderFlags_AlwaysClamp)) {
        m_inclineDegrees = inclineDegrees;
        // Live edit, no body rebuild: rotate the static ramp and re-seat the box at
        // the same angle with zero velocity so every theta starts from the same
        // controlled state (otherwise a slide from the previous theta leaks in).
        ApplyIncline(world);
        PlaceBlockAtStart(world);
    }

    // The world is the source of truth for mu: main.cpp may have applied the DebugUI
    // material defaults when the scene was built, and the panel must show the value the
    // contacts actually use. The combined friction of two equal bodies is sqrt(mu*mu).
    m_friction = std::sqrt(world.GetFriction(m_ground) * world.GetFriction(m_block));
    float frictionSlider = m_friction;
    if (ImGui::SliderFloat("Friction mu", &frictionSlider, 0.0f, 1.0f, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        m_friction = frictionSlider;
        ApplyFriction(world);
    }

    const float tanTheta = std::tan(m_inclineDegrees * kDegToRad);
    const bool predictedStatic = m_friction > tanTheta;
    ImGui::Text("mu = %.3f   tan(theta) = %.3f", m_friction, tanTheta);
    if (predictedStatic) {
        ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.40f, 1.0f), "Prediction: STATIC (mu > tan(theta))");
    } else {
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.30f, 1.0f), "Prediction: SLIDING (mu <= tan(theta))");
    }

    // Measured state is reported separately from the prediction: a disagreement is
    // exactly the interesting friction bug this scene exists to expose.
    const Vec2 position = world.GetPosition(m_block);
    const Vec2 velocity = world.GetLinearVelocity(m_block);
    const float drift = Length(position - m_spawnPosition);
    const float speed = Length(velocity);
    const float tiltDegrees = (world.GetAngle(m_block) - m_spawnAngle) * kRadToDeg;
    const bool measuredMoving = speed > kMovingSpeedThreshold || drift > kMovingDriftThreshold;
    ImGui::Text("Measured: drift=%.3f m   |v|=%.3f m/s   tilt=%+.2f deg", drift, speed, tiltDegrees);
    if (measuredMoving) {
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.30f, 1.0f), "Measured: SLIDING");
    } else {
        ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.40f, 1.0f), "Measured: STATIC");
    }

    int32_t touchingContacts = 0;
    float maxPenetration = 0.0f;
    float blockContactFriction = 0.0f;
    bool blockTouching = false;
    for (const Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingContacts;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
        if (contact.bodyA == m_block || contact.bodyB == m_block) {
            blockTouching = true;
            blockContactFriction = contact.friction;
        }
    }
    ImGui::Text("Contacts: %d (box touching: %s)   max pen: %.2f mm", touchingContacts,
                blockTouching ? "yes" : "no", maxPenetration * kMillimetersPerMeter);
    ImGui::Text("Contact mu (sqrt rule): %.3f", blockContactFriction);

    if (ImGui::Button("Reset block")) {
        Reset(world);
    }
    ImGui::Text("Resets: %d", m_resetCount);
    ImGui::TextDisabled("Editing theta/mu re-seats the box through the World API (no rebuild).");
}

void FrictionRampDemo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() invalidates every
    // handle, so both bodies are recreated below with the current theta/mu.
    world.Clear();
    m_ground = BodyHandle{};
    m_block = BodyHandle{};
    CreateRamp(world);
    CreateBlock(world);
    ++m_resetCount;
}

void FrictionRampDemo::CreateRamp(World& world) {
    const float angleRadians = m_inclineDegrees * kDegToRad;
    BodyDef bodyDef;
    bodyDef.type = BodyType::Static;
    bodyDef.position = Vec2{0.0f, 0.0f};
    // The body rotation carries the plane normal: local (0,1) becomes the world
    // normal of a slope inclined by theta. PlaneShape only allows static bodies.
    bodyDef.angle = angleRadians;
    bodyDef.friction = m_friction;
    bodyDef.debugName = "ramp";
    m_ground = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ground)) {
        std::fprintf(stderr, "[FrictionRamp] World::CreateBody rejected the ramp body\n");
        return;
    }

    const ShapeHandle planeShape = world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f);
    if (!world.IsValidShape(planeShape) || !world.AttachShape(m_ground, planeShape)) {
        std::fprintf(stderr, "[FrictionRamp] failed to create/attach the ramp plane shape\n");
    }
}

void FrictionRampDemo::CreateBlock(World& world) {
    const float angleRadians = m_inclineDegrees * kDegToRad;
    const Vec2 normal = RampNormal(angleRadians);
    BodyDef bodyDef;
    bodyDef.type = BodyType::Dynamic;
    // Exactly on the surface along the normal: the initial state has zero penetration,
    // so the first Step exercises the friction path instead of a deep-overlap recovery.
    bodyDef.position = normal * kHalfExtent;
    bodyDef.angle = angleRadians; // face parallel to the slope
    bodyDef.density = kDensity;
    bodyDef.friction = m_friction;
    bodyDef.restitution = 0.0f;
    bodyDef.debugName = "block";
    m_block = world.CreateBody(bodyDef);
    if (!world.IsValid(m_block)) {
        std::fprintf(stderr, "[FrictionRamp] World::CreateBody rejected the block body\n");
        return;
    }

    const ShapeHandle boxShape = world.CreateBoxShape(kHalfExtent, kHalfExtent);
    if (!world.IsValidShape(boxShape) || !world.AttachShape(m_block, boxShape)) {
        std::fprintf(stderr, "[FrictionRamp] failed to create/attach the block box shape\n");
        return;
    }
    m_spawnPosition = bodyDef.position;
    m_spawnAngle = angleRadians;
}

void FrictionRampDemo::ApplyIncline(World& world) {
    if (!world.IsValid(m_ground)) {
        return;
    }
    world.SetTransform(m_ground, Vec2{0.0f, 0.0f}, m_inclineDegrees * kDegToRad);
}

void FrictionRampDemo::ApplyFriction(World& world) {
    // Both bodies get the same value: the contact combines friction as the geometric
    // mean, so sqrt(mu * mu) == mu and the panel value is what the solver uses.
    if (world.IsValid(m_ground)) {
        world.SetFriction(m_ground, m_friction);
    }
    if (world.IsValid(m_block)) {
        world.SetFriction(m_block, m_friction);
    }
}

void FrictionRampDemo::PlaceBlockAtStart(World& world) {
    if (!world.IsValid(m_block)) {
        return;
    }
    const float angleRadians = m_inclineDegrees * kDegToRad;
    const Vec2 position = RampNormal(angleRadians) * kHalfExtent;
    world.SetTransform(m_block, position, angleRadians);
    world.SetLinearVelocity(m_block, Vec2{0.0f, 0.0f});
    world.SetAngularVelocity(m_block, 0.0f);
    m_spawnPosition = position;
    m_spawnAngle = angleRadians;
}

} // namespace p2
