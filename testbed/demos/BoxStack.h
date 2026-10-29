#pragma once

// p2::BoxStackDemo - Week 4/5 scene: a tower of ten 1 m x 1 m boxes on the ground plane.
//
// Why this scene: Week 4 landed the polygon narrow phase (SAT + clipping), and the
// tower is the first polygon stress test: every box has to be supported by a two-point
// box-box manifold plus the ground contact, and any error in the clipped contact points
// shows up as rocking. In Week 4 the solver had normal impulses only, so the tower was
// expected to jitter, sink and slowly topple - that collapse was recorded as the W4
// baseline.
//
// Week 5 acceptance: with the friction cone + warm starting + split-impulse position
// layer the same ten boxes must stand. The launch defaults (velocity iterations 8 from
// SolverSettings, default friction 0.3 from BodyDef, restitution 0) are the shipped
// configuration - the demo deliberately does not tune anything to make it pass. The
// smoke check is: `--demo BoxStack --smoke 300 --hidden` -> 11 bodies, 10 contacts
// (20 contact points), max penetration < 0.01 m and centres at y ~= 0.5..9.5.
// The draw code reads the polygon vertices back from the shape, so the wireframe always
// shows what the collision code actually sees - not what the demo thinks it spawned.
//
// Optional interaction: "Drop impact ball" adds a dynamic circle above the tower; that
// exercises circle-vs-polygon contacts (the last analytic path completed in Week 4).
// Reset rebuilds the tower in place and keeps the surrounding World settings (the
// DebugUI solver/material panels stay in effect).

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <array>
#include <cstdint>
#include <vector>

namespace p2 {

class BoxStackDemo final : public Demo {
  public:
    BoxStackDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

  private:
    void CreateGround(World& world);
    void CreateTower(World& world);
    void CreateImpactBall(World& world);

    static constexpr int32_t kLayerCount = 10;
    static constexpr float kHalfExtent = 0.5f; // 1 m x 1 m boxes, matching the design doc
    static constexpr float kDensity = 1.0f;    // kg/m^2; box mass = density * area = 1 kg
    static constexpr float kGroundY = 0.0f;
    static constexpr float kFirstLayerCenterY = 0.5f;
    // Exactly stacked: box height == layer pitch, so the initial state has zero
    // penetration and the solver starts from a legal configuration.
    static constexpr float kLayerSpacing = 2.0f * kHalfExtent;
    static constexpr float kImpactBallRadius = 0.5f;
    static constexpr float kImpactBallSpawnY = 12.0f; // above the 9.5 m top layer
    static constexpr int32_t kImpactBallSegments = 48;
    static constexpr int32_t kMaxImpactBalls = 8;          // keeps the body count bounded during play
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices

    BodyHandle m_ground;
    std::array<BodyHandle, kLayerCount> m_boxes{};
    std::vector<BodyHandle> m_impactBalls;
    int32_t m_resetCount = 0;
};

} // namespace p2
