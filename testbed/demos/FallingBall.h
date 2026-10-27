#pragma once

// p2::FallingBallDemo - Week 1 D6 scene: one dynamic circle under gravity.
//
// Scope of the scene matches what the Week 1 integrator can actually do:
//  - one Dynamic body (mass 1 kg, allowSleep default) starting at (0, 5);
//  - World::Step integrates forces then positions, so the circle accelerates
//    downwards under World::GetGravity();
//  - there are no shapes/contacts yet, so the "ground" is only a drawn line and
//    the ball falls through it; the demo auto-resets below y = -8 so the scene
//    never runs off screen.
//
// A second, purely visual body - a rotating square - is drawn from demo state
// (not from the World). That is the D5 renderer check: it proves the MVP matrix,
// the polygon path and the rotation math all work before physics shapes exist.

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
    void CreateBall(World& world);

    static constexpr float kBallRadius = 0.5f;
    static constexpr float kDropHeight = 5.0f;
    static constexpr float kGroundY = -8.0f; // drawn ground line and reset threshold
    static constexpr float kSquareHalfExtent = 0.5f;
    static constexpr float kSquareCenterX = 4.0f;
    static constexpr float kSquareCenterY = 3.0f;
    static constexpr float kDefaultSpinSpeed = 1.5f;    // rad/s
    static constexpr float kVelocityArrowScale = 0.15f; // seconds of travel shown

    BodyHandle m_ball;
    float m_squareAngle = 0.0f;
    float m_spinSpeed = kDefaultSpinSpeed;
    int32_t m_resetCount = 0;
};

} // namespace p2
