#pragma once

// p2::Pyramid200Demo - Week 6 scene: a perfect 2D box pyramid on the ground plane.
//
// Name note: the design document calls the scene "Pyramid200" (~200 OBBs). A perfect
// pyramid has a triangular number of boxes, so the shipped default is 16 layers =
// 136 boxes of 1x1 m (layer k holds baseCount - k boxes, centred). The name is kept for
// continuity; the actual count/layer configuration is printed in the log and shown in
// the panel, and the layer count is configurable there (slider + Rebuild). The size was
// chosen from measured stability runs, not tuned: with the engine's default 8 velocity
// iterations the 210/171/136/105-box versions settle at ~100/60/30/21 mm penetration
// respectively, and 136 boxes is the largest that settles without collapse in 10 s
// (table in testbed/README.md). The solver settings are never adjusted per size.
//
// Framing: even the default 16-layer 1 m pyramid is 16 m tall (the configurable maximum
// is 24 m), i.e. taller than the comfortable default 32x20 m view once the ground line
// is included; the host loop calls Camera::FocusOnWorld once after Setup/Reset (never
// per frame), so the whole scene is visible while pan/zoom remain user-owned afterwards.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <cstdint>
#include <vector>

namespace p2 {

class Pyramid200Demo final : public Demo {
  public:
    Pyramid200Demo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

  private:
    void BuildScene(World& world);

    // 16 layers = 136 boxes. The design-document target is "~200 OBBs", but a perfect
    // pyramid at the engine's default 8 velocity iterations does not hold that many
    // boxes: the measured "stability vs size" table (default settings, 600 steps, debug)
    // lives in testbed/README.md - 20 layers = 210 boxes sinks to 100 mm penetration at
    // the bottom rows, 18 = 171 boxes to 60 mm, 16 = 136 boxes settles at 30 mm with
    // <=1.4 deg tilt, and 14 = 105 boxes at 21 mm. 16 is the largest tested size that
    // settles without collapse/sliding in 10 s, so it is the default; the layer-count
    // slider can reproduce any larger/smaller size interactively.
    static constexpr int32_t kDefaultBaseCount = 16;
    static constexpr int32_t kMinBaseCount = 4;
    static constexpr int32_t kMaxBaseCount = 24;           // 300 boxes: still interactive in debug
    static constexpr float kBoxHalfExtent = 0.5f;          // 1 m x 1 m boxes, matching the design doc
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices
    static constexpr float kGroundHalfWidth = 16.0f;
    // Safety net, not scene logic: a box below this height means a physics regression
    // (the ground is an infinite half-space), so report and rebuild instead of letting
    // the scene silently degrade.
    static constexpr float kFallThroughResetY = -5.0f;

    // Standing verdict thresholds. A standing pyramid keeps its tilt small and its top
    // near the spawn height. Penetration is much larger than the Week 5 10 mm single-column
    // acceptance: a 136-box pyramid loads its bottom rows with ~16 boxes of weight and the
    // default 8 velocity iterations settle at a few tens of mm there (measured 30 mm at
    // 600 steps). The threshold below is a "did not collapse/sink" bound, not the W5
    // milestone - BoxStack still owns the <10 mm acceptance check.
    static constexpr float kStandingMaxTiltDegrees = 10.0f;
    static constexpr float kStandingMinTopHeightRatio = 0.9f;
    static constexpr float kStandingMaxPenetration = 0.05f; // m, stress-scene bound only

    BodyHandle m_ground;
    std::vector<BodyHandle> m_boxes;
    int32_t m_baseCount = kDefaultBaseCount;
    int32_t m_resetCount = 0;
};

} // namespace p2
