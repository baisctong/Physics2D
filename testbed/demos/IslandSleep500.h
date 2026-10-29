#pragma once

// p2::IslandSleep500Demo - Week 7 scene: ~500 resting boxes whose islands fall asleep,
// with the awake-vs-asleep Step-time comparison measured from REAL timings.
//
// Construction: 100 columns x 5 stacked boxes (500 boxes, 1 m each) on one ground plane.
// Five-box towers settle below the sleep tolerances quickly (a 10-box tower creeps at
// ~0.03 m/s and needs ~24 s - measured), and the columns are separated enough that the
// broad-phase proximity wake cannot keep the grid awake, so the whole scene (100 islands)
// falls asleep in a couple of seconds. The demo deliberately does not tune anything.
//
// Measurement: main.cpp hands the host loop's per-frame Step timing through
// Demo::SetFrameStats (raw + smoothed). The demo accumulates three windows:
//   - early window (frames 30..90): the scene is awake and settling,
//   - awake window (after warm-up, not all asleep): generic awake cost,
//   - asleep window (all boxes asleep): the sleeping win.
// Demo::PrintSmokeSummary prints the averages at the end of a smoke run, and the panel
// shows them live; the numbers are the source for README/devlog's sleep comparison.
//
// The "Wake all" button calls World::WakeBody on every box (whole-island wake), which
// lets the user re-run the asleep -> awake transition without rebuilding the scene.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <cstdint>
#include <vector>

namespace p2 {

class IslandSleep500Demo final : public Demo {
  public:
    IslandSleep500Demo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

    void SetFrameStats(const DemoFrameStats& stats) override;
    void PrintSmokeSummary(const World& world) const override;

  private:
    void BuildScene(World& world);
    void WakeAll(World& world);

    // 100 * 5 = 500 boxes: 100 five-box towers on one ground plane. Five stacked boxes
    // is small enough to settle below the sleep tolerances quickly (a 10-box tower keeps
    // a ~0.03 m/s solver residual and needs ~24 s to sleep - measured), so the whole
    // scene is asleep by ~frame 100 in a default build. Horizontal pitch 1.25 m leaves a
    // 0.25 m gap between towers: each tower is its own island and the fat AABBs
    // (0.6 m half width) do NOT touch across islands, so the broad-phase proximity wake
    // cannot cascade and keep the grid awake. The vertical pitch is the box height
    // (exactly stacked, zero initial penetration).
    static constexpr int32_t kColumnCount = 100; // towers along x
    static constexpr int32_t kStackHeight = 5;   // boxes stacked per tower
    static constexpr int32_t kTotalBoxes = kColumnCount * kStackHeight;
    static constexpr float kBoxHalfExtent = 0.5f;
    static constexpr float kHorizontalPitch = 1.25f;
    static constexpr float kGroundHalfWidth = 66.0f;       // covers the 62.5 m wide field
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices

    // Measurement windows (frames of the rendered loop; in smoke mode frames == steps).
    static constexpr int32_t kWarmupFrames = 10;     // skip the first landing/contact frame
    static constexpr int32_t kEarlyWindowStart = 30; // "awake" reference window (task card)
    static constexpr int32_t kEarlyWindowEnd = 90;

    BodyHandle m_ground;
    std::vector<BodyHandle> m_boxes;
    DemoFrameStats m_frameStats;
    int32_t m_frameIndex = 0;
    int32_t m_firstAllAsleepFrame = -1;
    double m_earlyStepSum = 0.0;
    int64_t m_earlyFrameCount = 0;
    double m_awakeStepSum = 0.0;
    int64_t m_awakeFrameCount = 0;
    double m_asleepStepSum = 0.0;
    int64_t m_asleepFrameCount = 0;
    int32_t m_resetCount = 0;
};

} // namespace p2
