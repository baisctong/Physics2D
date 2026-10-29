#pragma once

// p2::FrictionRampDemo - Week 5 scene (P15): a box on an adjustable incline whose
// friction coefficient is adjustable too.
//
// Why this scene: it is the visual counterpart of the friction-cone unit tests. The
// box stays put exactly when mu > tan(theta); a box that slides at mu <= tan(theta)
// (or creeps at the balance point) is a visible friction bug. The scene panel prints
// mu vs tan(theta), the predicted state and the measured drift/speed, so the demo
// shows both the theory and what the solver actually did with it.
//
// Construction: one static body carrying PlaneShape(normal (0,1), offset 0) rotated
// by theta, and one dynamic 1x1 m box rotated by the same theta, spawned exactly on
// that surface (zero initial penetration). Both bodies are given the same friction,
// so the pairwise combination sqrt(muBox * muGround) equals the panel value - that
// combination rule is the reason the ground is set too, not just the box.
//
// The plane is infinite by construction (it is a half-space shape), so a sliding box
// keeps sliding down rather than falling off a visible edge; Reset re-seats it and
// the safety net in Update() resets the scene if it leaves the visible world.
//
// Changing theta or mu in the panel does NOT rebuild bodies: the plane/box transforms
// and the material values are written through the public World API, which is the same
// live-tweak path the solver settings panel uses.

#include "Demo.h"

#include "physics2d/Core/Handle.h"
#include "physics2d/Math/Vec2.h"

#include <cstdint>

namespace p2 {

class FrictionRampDemo final : public Demo {
  public:
    FrictionRampDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

  private:
    void CreateRamp(World& world);
    void CreateBlock(World& world);
    void ApplyIncline(World& world);
    void ApplyFriction(World& world);
    void PlaceBlockAtStart(World& world);

    static constexpr float kHalfExtent = 0.5f; // 1 m x 1 m box, same as BoxStack
    static constexpr float kDensity = 1.0f;    // kg/m^2; box mass = 1 kg
    // 15 deg = tan 0.268, just below the BodyDef default friction 0.3: the shipped
    // scene is the "barely static" case, one slider step away from sliding.
    static constexpr float kDefaultInclineDegrees = 15.0f;
    static constexpr float kDefaultFriction = 0.3f;
    static constexpr float kMinInclineDegrees = 0.0f;
    static constexpr float kMaxInclineDegrees = 45.0f;
    // Drawn slope length in each direction; anything further is still collidable
    // (infinite plane) but off screen on the default camera.
    static constexpr float kRampHalfExtent = 10.0f;
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices
    static constexpr float kFallThroughResetY = -30.0f;    // safety net, not scene logic
    // "Moving" thresholds for the panel verdict: a resting contact legitimately has
    // sub-cm/s solver residual, so anything below these is reported as static.
    static constexpr float kMovingSpeedThreshold = 0.02f; // m/s
    static constexpr float kMovingDriftThreshold = 0.01f; // m

    BodyHandle m_ground; // static ramp body (plane attached, rotated by theta)
    BodyHandle m_block;  // dynamic box
    float m_inclineDegrees = kDefaultInclineDegrees;
    float m_friction = kDefaultFriction;
    Vec2 m_spawnPosition;      // where the block was last re-seated (drift reference)
    float m_spawnAngle = 0.0f; // block angle at that moment (tilt reference)
    int32_t m_resetCount = 0;
};

} // namespace p2
