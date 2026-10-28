#pragma once

// p2::FallingBallDemo - Week 2 scene: a dynamic circle falling onto an infinite
// static ground plane.
//
// Week 2 adds shapes and the analytic circle-plane narrow phase, so this scene now
// demonstrates the whole pipeline end to end:
//  - static ground body + PlaneShape(normal (0,1), offset 0), i.e. the surface
//    y = 0, attached at the body origin (0,0);
//  - dynamic circle body (radius 0.5 m, density 1 kg/m^2, so mass = pi * r^2)
//    spawned at (0, 4) with restitution 0;
//  - World::Step generates the contact and the temporary Week 2 position-level
//    response pushes the ball out of the ground and removes its normal velocity.
//    That response is temporary: momentum is not conserved and the restitution
//    behaviour is only approximate until the Week 3 impulse solver / Week 5 split
//    impulse. The demo's UI says so, so nobody mistakes the bounce for final
//    behaviour.
//  - the restitution slider drives World::SetRestitution live: e = 0 lands and
//    stops, e = 0.5/0.8 bounce approximately.
//
// Contact points and contact normals are visualised by the generic World-level
// overlay in main.cpp rather than here: the Demo interface (frozen by the brief)
// has no display-option parameter, and contacts are engine data shared by every
// future scene. The DebugUI toggles gate that overlay.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <cstdint>

namespace p2 {

class FallingBallDemo final : public Demo {
  public:
    FallingBallDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

  private:
    void CreateGround(World& world);
    void CreateBall(World& world);

    static constexpr float kBallRadius = 0.5f;
    static constexpr float kBallDensity = 1.0f; // kg/m^2, mass = density * pi * r^2
    static constexpr float kDropHeight = 4.0f;  // m, 3.5 m of free fall above the surface
    static constexpr float kGroundY = 0.0f;
    // Safety net, not scene logic: if a regression ever lets the ball tunnel through
    // the plane the demo reports it loudly and restarts instead of silently losing
    // the body off screen.
    static constexpr float kFallThroughResetY = -5.0f;
    static constexpr float kVelocityArrowScale = 0.15f; // seconds of travel shown
    static constexpr int32_t kCircleSegments = 48;

    BodyHandle m_ground;
    BodyHandle m_ball;
    float m_restitution = 0.0f; // up to Week 5 this is the body material used by the temporary response
    int32_t m_resetCount = 0;
};

} // namespace p2
