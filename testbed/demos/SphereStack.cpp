#include "demos/SphereStack.h"

#include "Renderer.h"

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace p2 {
namespace {

// One distinct colour per sphere so a collapsing stack makes it obvious which body
// moved where (the bottom sphere must stay on the ground in a healthy solve).
constexpr std::array<Color, 3> kSphereColors{{
    {0.95f, 0.55f, 0.20f, 1.0f},
    {0.35f, 0.72f, 0.95f, 1.0f},
    {0.72f, 0.55f, 0.95f, 1.0f},
}};
constexpr Color kCenterColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kVelocityColor{0.35f, 0.90f, 0.40f, 1.0f};

// Half-width of the drawn ground line. The visible world at 1x zoom is 32 m wide on
// the default 1280 px window, so 16 m reaches both screen edges.
constexpr float kGroundHalfWidth = 16.0f;
constexpr float kMillimetersPerMeter = 1000.0f;
// Velocity arrows are skipped below this squared speed (m^2/s^2): a resting stack
// always has sub-mm/s correction noise, and drawing it would fill the screen with
// arrows that mean nothing.
constexpr float kVelocityArrowThresholdSquared = 0.01f * 0.01f;

} // namespace

const char* SphereStackDemo::Name() const noexcept {
    return "Sphere Stack";
}

void SphereStackDemo::Setup(World& world) {
    m_resetCount = 0;
    m_spheres.fill(BodyHandle{});
    CreateGround(world);
    CreateSpheres(world);
    std::printf("[SphereStack] static ground plane y=0 + %d dynamic circles r=0.5 at "
                "y=0.5/1.5/2.5, density 1, restitution 0; Week 3 Sequential Impulse - "
                "a standing stack with small jitter is the expected result\n",
                kSphereCount);
}

void SphereStackDemo::Update(World& /*world*/, float /*dt*/) {
    // Intentionally empty: the stack has no scripted motion, and the card explicitly
    // forbids auto-resetting a collapsed stack (a falling tower is the measurement).
}

void SphereStackDemo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, kGroundY}, {kGroundHalfWidth, kGroundY}, kGroundColor);

    for (int32_t sphereIndex = 0; sphereIndex < kSphereCount; ++sphereIndex) {
        const BodyHandle handle = m_spheres[static_cast<std::size_t>(sphereIndex)];
        if (!world.IsValid(handle)) {
            continue;
        }
        const Vec2 position = world.GetPosition(handle);
        renderer.DrawCircle(position, kSphereRadius, kSphereColors[static_cast<std::size_t>(sphereIndex)],
                            kCircleSegments);
        renderer.DrawPoint(position, kCenterColor, 0.04f);

        const Vec2 velocity = world.GetLinearVelocity(handle);
        if (LengthSquared(velocity) > kVelocityArrowThresholdSquared) {
            renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
        }
    }
}

void SphereStackDemo::Ui(World& world) {
    ImGui::TextUnformatted("Three stacked circles (r=0.5) on a static ground plane.");
    ImGui::TextUnformatted(
        "A standing stack is the Week 3 Sequential Impulse check; small jitter is expected.");

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

    for (int32_t sphereIndex = 0; sphereIndex < kSphereCount; ++sphereIndex) {
        const BodyHandle handle = m_spheres[static_cast<std::size_t>(sphereIndex)];
        if (!world.IsValid(handle)) {
            ImGui::TextDisabled("Sphere %d missing", sphereIndex);
            continue;
        }
        const Vec2 position = world.GetPosition(handle);
        const Vec2 velocity = world.GetLinearVelocity(handle);
        ImGui::Text("Sphere %d: y=%.3f m   vy=%.3f m/s", sphereIndex, position.y, velocity.y);
    }

    if (ImGui::Button("Reset stack")) {
        Reset(world);
    }
    ImGui::Text("Resets: %d", m_resetCount);
}

void SphereStackDemo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() invalidates every
    // handle, so the ground and all spheres are recreated here.
    world.Clear();
    m_spheres.fill(BodyHandle{});
    CreateGround(world);
    CreateSpheres(world);
    ++m_resetCount;
}

void SphereStackDemo::CreateGround(World& world) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Static;
    bodyDef.position = {0.0f, kGroundY};
    bodyDef.debugName = "ground";
    m_ground = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ground)) {
        std::fprintf(stderr, "[SphereStack] World::CreateBody rejected the ground body\n");
        return;
    }

    // PlaneShape::normal points out of the solid toward free space: (0,1) + offset 0
    // is the half-space y < 0, so the ground surface is exactly y = 0. This block is
    // duplicated from FallingBall on purpose for now; the shared SceneBuilders.h the
    // design document schedules for Week 6 will absorb it.
    const ShapeHandle planeShape = world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f);
    if (!world.IsValidShape(planeShape) || !world.AttachShape(m_ground, planeShape)) {
        std::fprintf(stderr, "[SphereStack] failed to create/attach the ground plane shape\n");
    }
}

void SphereStackDemo::CreateSpheres(World& world) {
    for (int32_t sphereIndex = 0; sphereIndex < kSphereCount; ++sphereIndex) {
        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        // Exactly tangent spacing (2r) keeps the initial state contact-free by the
        // analytic circle test (penetration 0 counts as touching) instead of starting
        // the scene with an artificial overlap the solver would have to explode.
        bodyDef.position = {0.0f, kFirstSphereCenterY + kSphereSpacing * static_cast<float>(sphereIndex)};
        bodyDef.density = kSphereDensity;
        // Restitution 0: a stack must absorb its own fall instead of bouncing apart.
        bodyDef.restitution = 0.0f;
        bodyDef.debugName = "sphere_" + std::to_string(sphereIndex);
        const BodyHandle handle = world.CreateBody(bodyDef);
        if (!world.IsValid(handle)) {
            std::fprintf(stderr, "[SphereStack] World::CreateBody rejected sphere %d\n", sphereIndex);
            continue;
        }

        const ShapeHandle circleShape = world.CreateCircleShape(kSphereRadius);
        if (!world.IsValidShape(circleShape) || !world.AttachShape(handle, circleShape)) {
            std::fprintf(stderr, "[SphereStack] failed to create/attach sphere %d shape\n", sphereIndex);
            continue;
        }
        m_spheres[static_cast<std::size_t>(sphereIndex)] = handle;
    }
}

} // namespace p2
