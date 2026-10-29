#pragma once

// p2::DominoesDemo - Week 7 scene: a row of thin upright dominoes on the ground that is
// pushed to validate whole-island wake propagation.
//
// Why this scene: Week 7's sleeping design wakes a WHOLE island atomically. A domino
// chain is the cleanest visual proof that the wake condition and the contact graph
// propagation agree: after the chain has gone to sleep, an impulse on the first domino
// must wake it, then the next contact wakes the second island, and so on - no domino in
// the middle may be skipped (a skipped one would stay frozen upright while the wave
// passes through it). The panel therefore reports, live:
//   - islands / awake / sleeping (World statistics),
//   - "woken after sleeping" (transitions slept -> awake, i.e. chain-woken voices),
//   - fallen count and `chain reach` (consecutive fallen from index 0: any skipped
//     domino shows up as a gap and lowers the reach),
//   - max speed and contact count.
//
// Smoke: `--demo Dominoes --smoke N --hidden` is deterministic. The scene auto-pushes the
// first domino at t = 1.0 s (frame 60 in smoke, where frames == physics steps) if the
// user has not pressed "Push first" already, so an unattended run still exercises the
// chain and prints the final state through Demo::PrintSmokeSummary.
//
// The initial state has no box-box contacts: the dominoes stand `spacing` apart on the
// ground, so any contact after the push is genuinely created by the falling chain.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

#include <cstdint>
#include <vector>

namespace p2 {

class DominoesDemo final : public Demo {
  public:
    DominoesDemo() = default;

    [[nodiscard]] const char* Name() const noexcept override;
    void Setup(World& world) override;
    void Update(World& world, float dt) override;
    void Draw(Renderer& renderer, const World& world) override;
    void Ui(World& world) override;
    void Reset(World& world) override;

    void PrintSmokeSummary(const World& world) const override;

  private:
    void BuildScene(World& world);
    void PushFirst(World& world);

    // 20 dominoes: 20 x 1.0 m = 19 m of chain. The task allows 20-40; 20 keeps the
    // chain wave inside a 600-step smoke run (the wave travels at roughly
    // spacing / fall-time, about 2 m/s here).
    static constexpr int32_t kDefaultDominoCount = 20;
    static constexpr int32_t kMinDominoCount = 10;
    static constexpr int32_t kMaxDominoCount = 40;
    // 0.30 m x 1.0 m dominoes (width:height 1:3.3). Measured constraint from core-dev:
    // 0.12 m thin dominoes keep a ~0.015 m/s residual jitter spike (just above
    // kSleepLinearTolerance = 0.01 m/s), which re-arms sleepTime every 4-6 frames and
    // means the row NEVER sleeps; the 0.30 x 1.0 m shape settles and sleeps in ~32
    // static steps. Wider/thicker here is not physics tuning: it makes the scene able
    // to exercise the sleep -> chain-wake path at all.
    static constexpr float kDominoHalfWidth = 0.15f; // 0.30 m thick
    static constexpr float kDominoHalfHeight = 0.5f; // 1.0 m tall
    static constexpr float kDominoSpacing = 1.0f;    // > thickness, < height
    // Auto-push time: 1.0 s = step 60 of a smoke run (frames == steps), matching the
    // task card's suggested step 60. By then the settled row is asleep, so the push
    // proves the whole-island wake path.
    static constexpr float kAutoPushSeconds = 1.0f;
    // Impulse (N*s) near the top of the first domino. core-dev measured 0.25 N*s on the
    // 0.30x1.0 m shape in a 12-domino scene (all 12 woken/tumbled, no skips); the same
    // shape is used here. Mass = density * area = 0.3 kg.
    static constexpr float kPushImpulse = 0.25f;
    static constexpr float kGroundHalfWidth = 20.0f;
    static constexpr int32_t kMaxPolygonVerticesLocal = 8; // == p2::kMaxPolygonVertices

    BodyHandle m_ground;
    std::vector<BodyHandle> m_dominoes;
    // Per-domino history flags: bytes, not vector<bool>, so &flags[0] is a real pointer
    // and the state stays cache-friendly for the 40-domino loop.
    std::vector<uint8_t> m_wasAsleep;    // has slept at least once since the last push
    std::vector<uint8_t> m_wokenInChain; // slept -> awake transition after a push
    float m_elapsedSeconds = 0.0f;
    bool m_pushApplied = false;
    int32_t m_dominoCount = kDefaultDominoCount;
    int32_t m_resetCount = 0;
};

} // namespace p2
