#include "demos/FallingBall.h"

#include "Renderer.h"

#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"

#include <imgui.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace p2 {
namespace {

constexpr Color kBallColor{0.95f, 0.55f, 0.20f, 1.0f};
constexpr Color kBallCenterColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
constexpr Color kVelocityColor{0.35f, 0.90f, 0.40f, 1.0f};

// Half-width of the drawn ground line. The visible world at 1x zoom is 32 m wide on
// the default 1280 px window, so 16 m reaches both screen edges.
constexpr float kGroundHalfWidth = 16.0f;
constexpr float kNormalVectorLength = 0.5f; // m, fixed so penetration changes do not
                                            // make the arrows grow into clutter
constexpr float kMillimetersPerMeter = 1000.0f;

} // namespace

const char* FallingBallDemo::Name() const noexcept {
    return "Falling Ball";
}

void FallingBallDemo::Setup(World& world) {
    // Setup is the "fresh scene" entry point: defaults reset here, while Reset()
    // (the UI button) keeps the user's restitution choice.
    m_restitution = 0.0f;
    m_resetCount = 0;
    CreateGround(world);
    CreateBall(world);
    std::printf("[FallingBall] static ground plane y=0 + dynamic circle r=0.5 at (0, %.1f), "
                "density 1 kg/m^2, restitution %.2f; restitution is solved by the Week 3 "
                "Sequential Impulse solver (threshold %.1f m/s), e=0 lands and stops, e>0 bounces\n",
                kDropHeight, m_restitution, kRestitutionVelocityThreshold);
}

void FallingBallDemo::Update(World& world, float /*dt*/) {
    // The scene is quasi-static: the ball only falls and (with e > 0) bounces, so the
    // demo has no per-frame animation state of its own. The dt parameter is part of
    // the Demo contract and stays unused here.
    if (!world.IsValid(m_ball)) {
        return;
    }
    if (world.GetPosition(m_ball).y < kFallThroughResetY) {
        // Only reachable if the narrow phase or the solver regresses: make it loud
        // rather than letting the ball disappear off screen.
        std::fprintf(stderr,
                     "[FallingBall] ball fell through the plane (y < %.1f) - physics regression, "
                     "restarting scene\n",
                     kFallThroughResetY);
        Reset(world);
    }
}

void FallingBallDemo::Draw(Renderer& renderer, const World& world) {
    // The ground line is visual only: the real collision is the infinite half-space
    // behind PlaneShape. Drawing it at y = 0 makes the contact point easy to judge
    // against the surface.
    renderer.DrawSegment({-kGroundHalfWidth, kGroundY}, {kGroundHalfWidth, kGroundY}, kGroundColor);

    if (!world.IsValid(m_ball)) {
        return;
    }
    // Draw from the stable accessors, not from Body internals: the Week 4 hot/cold
    // split will move the storage without changing this API.
    const Vec2 position = world.GetPosition(m_ball);
    renderer.DrawCircle(position, kBallRadius, kBallColor, kCircleSegments);
    renderer.DrawPoint(position, kBallCenterColor, 0.05f);

    // Velocity arrow: shows the free fall accelerating, then the impulse solver
    // reversing it on impact (e > 0) or zeroing it (e = 0); the arrow is the quickest
    // visual check that the solver ran at all.
    const Vec2 velocity = world.GetLinearVelocity(m_ball);
    renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityColor);
}

void FallingBallDemo::Ui(World& world) {
    ImGui::TextUnformatted("Dynamic circle (r=0.5 m, density 1) vs static ground plane y=0.");

    if (world.IsValid(m_ball)) {
        const Vec2 position = world.GetPosition(m_ball);
        const Vec2 velocity = world.GetLinearVelocity(m_ball);
        ImGui::Text("Ball y: %.3f m   vy: %.3f m/s", position.y, velocity.y);
    } else {
        ImGui::TextDisabled("Ball body is missing");
    }

    // Mirror the DebugUI contact statistics here: the scene parameters panel is where
    // the user tunes restitution, so the resulting contact state belongs next to it.
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

    // e = 0 must land and stop; higher values bounce with the impact speed scaled by e.
    // Since Week 3 the Sequential Impulse solver applies restitution only above the
    // threshold speed, so a resting contact does not buzz even with e > 0.
    float restitution = m_restitution;
    if (ImGui::SliderFloat("Restitution (e)", &restitution, 0.0f, 1.0f, "%.2f")) {
        m_restitution = restitution;
        if (world.IsValid(m_ball)) {
            world.SetRestitution(m_ball, m_restitution);
        }
    }
    if (!world.GetContacts().empty()) {
        // Combined material rule (max of both bodies) as seen by the response; useful
        // to confirm the slider actually reached the contact.
        ImGui::Text("Contact e (max rule): %.2f", world.GetContacts().front().restitution);
    }

    if (ImGui::Button("Drop again")) {
        Reset(world);
    }
    ImGui::Text("Drops / respawns: %d", m_resetCount);
}

void FallingBallDemo::Reset(World& world) {
    // Demo::Reset contract: rebuild the scene in place. Clear() also invalidates the
    // old handles, so both bodies are recreated below with the current restitution.
    world.Clear();
    CreateGround(world);
    CreateBall(world);
    ++m_resetCount;
}

void FallingBallDemo::CreateGround(World& world) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Static;
    bodyDef.position = {0.0f, kGroundY};
    bodyDef.debugName = "ground";
    m_ground = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ground)) {
        std::fprintf(stderr, "[FallingBall] World::CreateBody rejected the ground body\n");
        return;
    }

    // PlaneShape::normal points out of the solid toward free space: (0,1) + offset 0
    // is the half-space y < 0, i.e. the ground surface is exactly y = 0. The shape is
    // attached at the body origin; planes are only valid on static bodies.
    const ShapeHandle planeShape = world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f);
    if (!world.IsValidShape(planeShape) || !world.AttachShape(m_ground, planeShape)) {
        std::fprintf(stderr, "[FallingBall] failed to create/attach the ground plane shape\n");
    }
}

void FallingBallDemo::CreateBall(World& world) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Dynamic;
    bodyDef.position = {0.0f, kDropHeight};
    // Density is the source of truth: AttachShape recomputes mass = density * pi * r^2
    // and the disc inertia. BodyDef.mass keeps its default value and is
    // overwritten by the attach, so it must not be treated as the real mass.
    bodyDef.density = kBallDensity;
    bodyDef.debugName = "falling_ball";
    m_ball = world.CreateBody(bodyDef);
    if (!world.IsValid(m_ball)) {
        std::fprintf(stderr, "[FallingBall] World::CreateBody rejected the ball body\n");
        return;
    }

    const ShapeHandle circleShape = world.CreateCircleShape(kBallRadius);
    if (!world.IsValidShape(circleShape) || !world.AttachShape(m_ball, circleShape)) {
        std::fprintf(stderr, "[FallingBall] failed to create/attach the ball circle shape\n");
        return;
    }

    // Materials live on the Body (combined by the contact: restitution = max(eA,eB)),
    // so re-apply the slider value after every respawn.
    world.SetRestitution(m_ball, m_restitution);
}

} // namespace p2
