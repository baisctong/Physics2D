#pragma once

// p2::StressBenchmarkDemo - Week 6 scene: a deterministic random field of OBBs used for
// broad-phase comparison and solver stress.
//
// What it measures: the DebugUI broad-phase dropdown can switch
// None(O(N^2)) / BruteForce / SAP without rebuilding this scene, and the panel shows the
// live proxies / candidate pairs / contacts / Step time / FPS. Because the bodies are
// not rebuilt, the same run can be compared between implementations (the World
// canonicalises candidate pairs, so the physics must stay identical - core-dev's
// equivalence tests own that contract; this scene makes it observable).
//
// Construction: CreateRandomBoxField with a fixed seed (42) and an overlap-rejecting
// spawn so the first frames measure broad phase/solver work rather than a position
// correction explosion. The count is configurable (slider + Rebuild, default 400);
// Rebuild is an explicit action, never a per-frame body rebuild.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <cstdint>
#include <vector>

namespace p2 {

class StressBenchmarkDemo final : public Demo {
  public:
    StressBenchmarkDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

    // Week 6: the host loop pushes its smoothed timings in once per frame (Demo.h hook)
    // so this scene can quote Step ms / FPS without widening the frozen Demo methods.
    void SetFrameStats(const DemoFrameStats& stats) override;

  private:
    void BuildScene(World& world);

    static constexpr int32_t kDefaultBoxCount = 400; // "300-500 random boxes" per task card
    static constexpr int32_t kMinBoxCount = 50;
    static constexpr int32_t kMaxBoxCount = 800;
    static constexpr uint32_t kFieldSeed = 42u; // fixed: smoke numbers are reproducible
    // Spawn volume: x in [-18, 18], y in [one box, 18]. The settled pile is a few metres
    // tall; Camera::FocusOnWorld frames the initial field once on setup.
    static constexpr float kFieldHalfExtent = 18.0f;
    static constexpr float kMaxBoxHalfExtent = 0.5f;       // 1 m boxes
    static constexpr float kMaxInitialSpeed = 4.0f;        // m/s in a random direction
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices
    static constexpr float kGroundHalfWidth = 22.0f;
    static constexpr float kFallThroughResetY = -10.0f;

    BodyHandle m_ground;
    std::vector<BodyHandle> m_boxes;
    DemoFrameStats m_frameStats;
    int32_t m_boxCount = kDefaultBoxCount;
    int32_t m_resetCount = 0;
};

} // namespace p2
