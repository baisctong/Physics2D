#pragma once

// p2::DebugUI - the testbed's control and status panels (Week 1 D5/D6).
//
// Week 1 deliberately keeps the UI plain (Section 5, Week 1 trap warning: do not
// polish the ImGui panels before the simulation is trustworthy). It exposes only
// what the current simulation supports: playback control, the gravity slider
// wired to World::SetGravity, the demo selector, and a few display toggles.
//
// The panel owns its UI state (paused / speed / demo selection). The host loop
// owns the simulation state; the two are connected through the FrameActions
// result so the render loop stays readable: UI -> actions -> world.

#include "Demo.h"

#include <cstdint>
#include <vector>

namespace p2 {

class World;

class DebugUI {
  public:
    // Per-frame numbers shown in the status panel. Filled by main.cpp from its
    // own timers because ImGui only knows about frame time, not physics time.
    struct FrameStats {
        float framesPerSecond = 0.0f;
        float frameMilliseconds = 0.0f;
        float physicsStepMilliseconds = 0.0f;
        int32_t physicsSteps = 0;
    };

    // Display switches consumed by the render loop after the demo has drawn.
    struct DisplayOptions {
        bool showAxes = true;
        bool showAabb = false;
    };

    // One-frame commands. They are edges, not states: main.cpp applies them once
    // and the next frame starts from false again.
    struct FrameActions {
        bool demoChanged = false;
        bool resetRequested = false;
        bool singleStepRequested = false;
        bool quitRequested = false;
    };

    DebugUI() = default;

    // Draws both panels and the optional Dear ImGui demo window. `selectedDemo`
    // is edited in place by the scene combo box; the caller rebuilds the World
    // when the returned action says so.
    [[nodiscard]] FrameActions Draw(World& world, Demo& currentDemo,
                                    const std::vector<const char*>& demoNames, int& selectedDemo,
                                    const FrameStats& stats, DisplayOptions& display);

    [[nodiscard]] bool IsPaused() const noexcept { return m_paused; }

    // 1x / 0.5x / 0.25x, matching the design document's playback speeds.
    [[nodiscard]] float GetSpeedMultiplier() const noexcept;

  private:
    void DrawControlPanel(World& world, Demo& currentDemo, const std::vector<const char*>& demoNames,
                          int& selectedDemo, DisplayOptions& display, FrameActions& actions);
    void DrawStatusPanel(const World& world, const Demo& currentDemo, const FrameStats& stats);

    bool m_paused = false;
    int m_speedIndex = 0; // index into kSpeedMultipliers
    bool m_showImGuiDemo = false;
};

} // namespace p2
