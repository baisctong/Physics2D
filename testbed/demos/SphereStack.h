#pragma once

// p2::SphereStackDemo - Week 3 scene: three circles stacked vertically on the same
// infinite static ground plane as FallingBall.
//
// Why this scene: it is the first test of the Sequential Impulse solver. Three circle
// bodies stacked on top of each other form a contact chain (ground->sphere0,
// sphere0->sphere1, sphere1->sphere2) in which every normal impulse has to carry the
// weight of everything above it. If the solver misses iterations or clamps the
// accumulated impulses incorrectly the tower sinks, bounces apart or jitters; the
// Week 3 acceptance criterion is deliberately weak - "it stays standing, small jitter
// is allowed" - because warm starting and split impulse only land in Week 5.
//
// The spheres start exactly tangent (centre spacing == 2 * radius), so any visible
// sinking is the solver working through the position slop, not an initial overlap.
// There is no auto-reset: a collapsed stack is a real physics result the user should
// see, not a state the demo hides.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <array>
#include <cstdint>

namespace p2 {

class SphereStackDemo final : public Demo {
  public:
    SphereStackDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

  private:
    void CreateGround(World& world);
    void CreateSpheres(World& world);

    static constexpr int32_t kSphereCount = 3;
    static constexpr float kSphereRadius = 0.5f;
    static constexpr float kSphereDensity = 1.0f; // kg/m^2, mass = density * pi * r^2
    static constexpr float kGroundY = 0.0f;
    static constexpr float kFirstSphereCenterY = 0.5f;            // sphere sits on the ground surface
    static constexpr float kSphereSpacing = 2.0f * kSphereRadius; // exactly tangent stack
    static constexpr float kVelocityArrowScale = 0.10f;           // shorter than FallingBall: stack
                                                                  // jitter is small but meaningful
    static constexpr int32_t kCircleSegments = 48;

    BodyHandle m_ground;
    std::array<BodyHandle, kSphereCount> m_spheres{};
    int32_t m_resetCount = 0;
};

} // namespace p2
