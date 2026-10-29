#pragma once

// p2::DebugUI - the testbed's control, solver, contact and status panels
// (Week 1 D5/D6; contact statistics in Week 2; in Week 5 the Solver Settings
// panel, the live contact inspector and the Step-timing/island counters from P14).
//
// The panels stay deliberately plain (design document Section 5 warns against
// polishing the UI before the simulation is trustworthy). They expose only state
// the engine already owns:
//   - Testbed Controls: scene selection, playback, speed and the per-scene Ui()
//     block. Global controls live in the other panels now that Week 5 grew the
//     parameter set;
//   - Solver Settings: edits a copy of World::GetSolverSettings() and writes it
//     back with World::SetSolverSettings(). Every slider therefore takes effect
//     on the next fixed step - no scene rebuild, no restart. Since Week 6 the same
//     panel also hosts the broad-phase dropdown (Auto / None / BruteForce / SAP)
//     with the live installed implementation, proxy and candidate-pair counts;
//     switching implementations only swaps World's broad-phase object, the scene is
//     never rebuilt. Auto is the default and dispatches on the bench-measured body
//     count crossover (see BroadPhaseMode below);
//   - Contacts: reads World::GetContacts() (rebuilt every Step) and lets the
//     user click a pair to highlight it. main.cpp draws the highlighted contact
//     point and normal in a distinct colour so a clicked row is visible in the
//     scene;
//   - Status: FPS / frame + Step timings, body/shape counts, islands / awake /
//     sleeping (Week 7), contact and contact-point counts, max penetration,
//     broad-phase name / proxy / candidate-pair counts and the active solver
//     settings.
//
// Layout ownership: the panel owns UI state (paused / speed / scene selection /
// material defaults); the host loop owns the simulation state. The two are
// connected through FrameActions and DisplayOptions so the render loop stays
// readable: UI -> actions -> world.

#include "Demo.h"

#include "physics2d/Core/Handle.h"

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
        bool showContacts = true;
        bool showNormals = false;
        // Engine-level velocity overlay for every non-static body. Off by default:
        // most demos already draw their own velocity arrows, and this switch exists
        // for scenes/tests that do not (Week 5 FrictionRamp uses it in the docs).
        bool showVelocities = false;

        // Contact highlighted in the Contacts panel, identified by the body pair
        // instead of a list index: World rebuilds its contact vector every Step, so
        // indices are not stable across frames. main.cpp validates both handles
        // before drawing (Reset/scene switch invalidate them); an invalid A handle
        // means "no highlight".
        BodyHandle highlightedContactBodyA;
        BodyHandle highlightedContactBodyB;
    };

    // One-frame commands. They are edges, not states: main.cpp applies them once
    // and the next frame starts from false again.
    struct FrameActions {
        bool demoChanged = false;
        bool resetRequested = false;
        bool singleStepRequested = false;
        bool quitRequested = false;
        // Pushes MaterialDefaults onto every body of the current world. Used by the
        // Solver panel's "Apply to current scene" button (the same values are applied
        // automatically when main.cpp builds a scene).
        bool applyMaterialDefaultsRequested = false;
    };

    // Material values for newly built scenes (P14 "friction / restitution"
    // sliders). BodyDef carries the low-level defaults; these are the testbed's
    // experiment knobs: main.cpp writes them onto every body after a scene switch,
    // and "Apply to current scene" does the same on demand. Friction is combined
    // pairwise as sqrt(fA*fB) and restitution as max(eA,eB), so equal values on
    // both bodies give the value shown here (that is why static bodies are included).
    struct MaterialDefaults {
        float friction = 0.3f;    // mirrors BodyDef::friction so a fresh session is unchanged
        float restitution = 0.0f; // mirrors BodyDef::restitution
    };

    // Week 6: broad-phase selection policy. World owns only the object; the testbed owns
    // the choice:
    //  - Auto (default): main.cpp calls ApplyBroadPhase() after every scene build and it
    //    installs BruteForce for small scenes and SAP above kBroadPhaseAutoThreshold;
    //  - None / BruteForce / SAP: manual override from the Solver Settings dropdown or
    //    --broadphase. An override survives scene switches until Auto is chosen again
    //    (main re-applies the current mode on every build; the dropdown re-installs
    //    immediately when it changes).
    // The physics must not depend on the choice: World canonicalises the candidate pairs
    // before the narrow phase. "None" makes World fall back to its built-in O(N^2)
    // enumeration (the Week 2-5 path).
    enum class BroadPhaseMode : int { Auto = 0, None = 1, BruteForce = 2, SAP = 3 };

    // Bench-derived dispatch crossover (Release build, seed42, 1000 frames, dev machine;
    // the full table is in testbed/README.md and the root README performance table):
    // SAP is slower than BruteForce below ~25 bodies (N=20: 0.80x) and clearly faster
    // above (N=50: 2.12x, N=200: 4.99x, N=500: 5.91x), so 32 sits on the conservative
    // side of the measured 25-32 crossover.
    static constexpr int32_t kBroadPhaseAutoThreshold = 32;

    [[nodiscard]] static const char* BroadPhaseModeName(BroadPhaseMode mode) noexcept;

    [[nodiscard]] BroadPhaseMode GetBroadPhaseMode() const noexcept { return m_broadPhaseMode; }
    void SetBroadPhaseMode(BroadPhaseMode mode) noexcept { m_broadPhaseMode = mode; }

    // Installs the broad phase selected by the current mode (Auto re-derives from the
    // world's live body count). Called by main.cpp after a scene is built or reset -
    // never every frame - and by the dropdown on a mode change (no body rebuild).
    void ApplyBroadPhase(World& world);

    DebugUI() = default;

    // Draws the panels (and the optional Dear ImGui demo window). `selectedDemo`
    // is edited in place by the scene combo box; the caller rebuilds the World
    // when the returned action says so.
    [[nodiscard]] FrameActions Draw(World& world, Demo& currentDemo,
                                    const std::vector<const char*>& demoNames, int& selectedDemo,
                                    const FrameStats& stats, DisplayOptions& display);

    [[nodiscard]] bool IsPaused() const noexcept { return m_paused; }

    // Week 7 keyboard shortcuts (main.cpp edge-detects the keys): Space calls
    // TogglePaused, '.' single-steps while paused (main reads IsPaused), 1/2/3 select the
    // playback speed. The UI buttons remain the discoverable path.
    void TogglePaused() noexcept { m_paused = !m_paused; }
    static constexpr int kPlaybackSpeedCount = 3; // 1x / 0.5x / 0.25x
    void SetSpeedIndex(int index) noexcept {
        if (index >= 0 && index < kPlaybackSpeedCount) {
            m_speedIndex = index;
        }
    }

    // 1x / 0.5x / 0.25x, matching the design document's playback speeds.
    [[nodiscard]] float GetSpeedMultiplier() const noexcept;

    // Values main.cpp applies when a scene is (re)built.
    [[nodiscard]] const MaterialDefaults& GetMaterialDefaults() const noexcept { return m_materialDefaults; }

  private:
    void DrawControlPanel(World& world, Demo& currentDemo, const std::vector<const char*>& demoNames,
                          int& selectedDemo, FrameActions& actions);
    void DrawSolverPanel(World& world, DisplayOptions& display, FrameActions& actions);
    void DrawContactsPanel(World& world, DisplayOptions& display);
    void DrawStatusPanel(const World& world, const Demo& currentDemo, const FrameStats& stats);

    bool m_paused = false;
    int m_speedIndex = 0; // index into kSpeedMultipliers
    bool m_showImGuiDemo = false;
    MaterialDefaults m_materialDefaults;
    BroadPhaseMode m_broadPhaseMode = BroadPhaseMode::Auto;
};

} // namespace p2
