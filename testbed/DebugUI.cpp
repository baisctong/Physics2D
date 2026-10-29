#include "DebugUI.h"

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Core/World.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <memory>

namespace p2 {
namespace {

// ---- panel metrics -----------------------------------------------------------
// Default floating layout: two columns that together fit the 1280x800 window the
// testbed opens (no imgui.ini is written, so these defaults are what a reviewer
// sees on the first run). ImGuiCond_FirstUseEver keeps the user's drag/dock.
constexpr float kPanelGap = 12.0f;
constexpr float kControlPanelWidth = 340.0f;
constexpr float kControlPanelHeight = 420.0f;
constexpr float kStatusPanelWidth = 300.0f;
constexpr float kStatusPanelHeight = 310.0f;
constexpr float kSolverPanelWidth = 340.0f;
constexpr float kSolverPanelHeight = 440.0f;
constexpr float kContactsPanelWidth = 440.0f;
constexpr float kContactsPanelHeight = 330.0f;
// Height of the scrollable contact table inside its panel (leaves room for the
// summary line above and the highlight hint below).
constexpr float kContactsTableHeight = 226.0f;

// Playback speeds from the design document's panel spec. The multiplier scales how
// much simulated time one rendered frame contributes, so physics stays on the same
// fixed dt and only the accumulator feed changes (Section 4.3).
constexpr int kSpeedCount = 3;
constexpr float kSpeedMultipliers[kSpeedCount] = {1.0f, 0.5f, 0.25f};
constexpr const char* kSpeedLabels[kSpeedCount] = {"1x", "0.5x", "0.25x"};

// Solver slider ranges. Iterations start at 0 on purpose: "skip this phase" is a
// legal experiment, and the upper bound of 40 is the point where more iterations
// stop changing a 10-box tower. World::Step asserts on negative counts, so the
// slider must never be able to produce one.
constexpr int kMinIterations = 0;
constexpr int kMaxIterations = 40;

// Split-impulse / contact slop are exposed in metres with a step below
// kLinearSlop (5 mm), so the user can reproduce the "slop smaller than the
// penetration band" experiments without leaving the sensible range.
constexpr float kMinSlop = 0.0f;
constexpr float kMaxSlop = 0.02f;
constexpr float kMinCorrection = 0.0f;
constexpr float kMaxCorrection = 0.5f;

// Gravity range: -30 m/s^2 is "very heavy", 0 is weightlessness. Negative-only
// keeps the "which way is down" convention unambiguous in the panel.
constexpr float kMinGravityY = -30.0f;
constexpr float kMaxGravityY = 0.0f;

constexpr float kMinMaterial = 0.0f;
constexpr float kMaxMaterial = 1.0f;

// Penetration is displayed in millimetres: the interesting values are the
// tolerated slop (5 mm) and the transient overlap during a landing, which are
// unreadable as metres in a status panel.
constexpr float kMillimetersPerMeter = 1000.0f;

// Right column x offset: both right-hand panels end `kPanelGap` from the edge.
constexpr float kRightColumnOffset = kStatusPanelWidth + kPanelGap;

// ---- broad-phase dropdown (Week 6) ------------------------------------------
// Combo order matches DebugUI::BroadPhaseMode. Auto is the shipped default: main.cpp
// calls ApplyBroadPhase after every scene build, and it dispatches on the live body
// count using the bench-measured crossover. The names are also the labels shown in the
// panel, so the user always sees the real implementation plus the choice that produced
// it (see the Status panel line).
constexpr const char* kBroadPhaseModeLabels[] = {"Auto (by body count)", "None (O(N^2) fallback)",
                                                 "BruteForce", "SAP"};
constexpr int kBroadPhaseModeCount = 4;

} // namespace

const char* DebugUI::BroadPhaseModeName(BroadPhaseMode mode) noexcept {
    switch (mode) {
    case BroadPhaseMode::Auto:
        return "Auto";
    case BroadPhaseMode::None:
        return "None";
    case BroadPhaseMode::BruteForce:
        return "BruteForce";
    case BroadPhaseMode::SAP:
        return "SAP";
    }
    return "Auto";
}

void DebugUI::ApplyBroadPhase(World& world) {
    // Every path ends with exactly one implementation installed (or none for None), so
    // the caller never has to reason about leftover proxies: World::SetBroadPhase
    // releases the old implementation's proxies and the next Step re-inserts them.
    switch (m_broadPhaseMode) {
    case BroadPhaseMode::None:
        world.SetBroadPhase(nullptr);
        return;
    case BroadPhaseMode::BruteForce:
        world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
        return;
    case BroadPhaseMode::SAP:
        world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
        return;
    case BroadPhaseMode::Auto:
    default:
        // Dispatch by scene size (the Week 6 bench crossover): small scenes are cheaper
        // with the O(N^2) baseline, large scenes need the sweep.
        if (world.GetBodyCount() >= kBroadPhaseAutoThreshold) {
            world.SetBroadPhase(std::make_unique<BroadPhaseSAP>());
        } else {
            world.SetBroadPhase(std::make_unique<BroadPhaseBruteForce>());
        }
        return;
    }
}

float DebugUI::GetSpeedMultiplier() const noexcept {
    if (m_speedIndex < 0 || m_speedIndex >= kSpeedCount) {
        return 1.0f;
    }
    return kSpeedMultipliers[m_speedIndex];
}

DebugUI::FrameActions DebugUI::Draw(World& world, Demo& currentDemo,
                                    const std::vector<const char*>& demoNames, int& selectedDemo,
                                    const FrameStats& stats, DisplayOptions& display) {
    FrameActions actions;
    DrawControlPanel(world, currentDemo, demoNames, selectedDemo, actions);
    DrawSolverPanel(world, display, actions);
    DrawContactsPanel(world, display);
    DrawStatusPanel(world, currentDemo, stats);
    if (m_showImGuiDemo) {
        // Built-in demo window: the fastest way to verify docking, tables and
        // input plumbing work on this machine without writing widgets ourselves.
        ImGui::ShowDemoWindow(&m_showImGuiDemo);
    }
    return actions;
}

void DebugUI::DrawControlPanel(World& world, Demo& currentDemo, const std::vector<const char*>& demoNames,
                               int& selectedDemo, FrameActions& actions) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // ImGuiCond_FirstUseEver: the placement below is only a default. Once layout
    // persistence is enabled, a user's docked layout wins on later runs.
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + kPanelGap, viewport->WorkPos.y + kPanelGap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(kControlPanelWidth, kControlPanelHeight), ImGuiCond_FirstUseEver);
    ImGui::Begin("Testbed Controls");

    ImGui::TextUnformatted("Physics2D testbed - Week 5");
    ImGui::Separator();

    if (!demoNames.empty()) {
        if (selectedDemo < 0 || selectedDemo >= static_cast<int>(demoNames.size())) {
            selectedDemo = 0;
        }
        if (ImGui::Combo("Scene", &selectedDemo, demoNames.data(), static_cast<int>(demoNames.size()))) {
            actions.demoChanged = true;
        }
    } else {
        ImGui::TextDisabled("No scenes registered");
    }

    ImGui::Separator();
    if (ImGui::Button(m_paused ? "Play" : "Pause")) {
        m_paused = !m_paused;
    }
    ImGui::SameLine();
    // Single-step only makes sense while paused; disabling the button makes that
    // state obvious instead of silently ignoring clicks.
    ImGui::BeginDisabled(!m_paused);
    if (ImGui::Button("Step")) {
        actions.singleStepRequested = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        actions.resetRequested = true;
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Speed");
    for (int speedIndex = 0; speedIndex < kSpeedCount; ++speedIndex) {
        if (speedIndex > 0) {
            ImGui::SameLine();
        }
        if (ImGui::RadioButton(kSpeedLabels[speedIndex], m_speedIndex == speedIndex)) {
            m_speedIndex = speedIndex;
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Quit")) {
        actions.quitRequested = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Camera: middle-drag / wheel / R");

    // Solver, material, gravity and display controls moved to the Solver Settings
    // panel in Week 5; this panel stays the scene/playback panel so the floating
    // windows fit a 1280x800 screen without overlapping.
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Scene parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        currentDemo.Ui(world);
    }
    ImGui::End();
}

void DebugUI::DrawSolverPanel(World& world, DisplayOptions& display, FrameActions& actions) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - kRightColumnOffset,
                                   viewport->WorkPos.y + kPanelGap + kStatusPanelHeight + kPanelGap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(kSolverPanelWidth, kSolverPanelHeight), ImGuiCond_FirstUseEver);
    ImGui::Begin("Solver Settings");

    // Edit a copy and write it back in one place: dragging a slider therefore
    // reaches World::SetSolverSettings exactly once per changed frame, and the next
    // fixed step (which reads m_solverSettings) uses the new value. Body storage is
    // never touched, so no contact/manifold cache is rebuilt by the UI.
    SolverSettings settings = world.GetSolverSettings();
    bool settingsChanged = false;

    if (ImGui::Button("Reset solver defaults")) {
        settings = SolverSettings{};
        settingsChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("live");

    ImGui::Separator();
    ImGui::TextUnformatted("Broad phase (Week 6)");
    // The mode is the testbed's policy (Auto dispatches by scene size); the installed
    // implementation is World's. The combo shows the mode, the lines below show what is
    // actually running, so an Auto scene never hides its real implementation.
    int broadPhaseModeIndex = static_cast<int>(m_broadPhaseMode);
    if (ImGui::Combo("Implementation", &broadPhaseModeIndex, kBroadPhaseModeLabels, kBroadPhaseModeCount)) {
        // Runtime switch only: World releases the old implementation's proxies and
        // re-inserts them on the next step. Bodies/contacts are NOT rebuilt, and the
        // candidate pairs are canonicalised before the narrow phase, so the physics
        // result must stay identical between implementations (core-dev's equivalence
        // contract; this panel is the interactive way to observe it).
        if (broadPhaseModeIndex < 0) {
            broadPhaseModeIndex = 0;
        }
        m_broadPhaseMode = static_cast<BroadPhaseMode>(broadPhaseModeIndex);
        ApplyBroadPhase(world);
    }
    const IBroadPhase* broadPhase = world.GetBroadPhase();
    ImGui::Text("Installed: %s   mode: %s", broadPhase != nullptr ? broadPhase->Name() : "None",
                BroadPhaseModeName(m_broadPhaseMode));
    // Counts describe the last completed Step (the callback runs inside World::Step).
    ImGui::Text("Proxies: %d   Candidate pairs: %d", broadPhase != nullptr ? broadPhase->GetProxyCount() : 0,
                world.GetBroadPhasePairCount());
    if (m_broadPhaseMode == BroadPhaseMode::Auto) {
        ImGui::TextDisabled("Auto: BruteForce < %d bodies, SAP >= %d (bench crossover)",
                            kBroadPhaseAutoThreshold, kBroadPhaseAutoThreshold);
    } else {
        // A manual pick sticks across scene switches until Auto is chosen again; this line
        // makes that explicit instead of letting a scene switch silently undo it.
        ImGui::TextDisabled("Manual override active (%d bodies); pick Auto to re-enable dispatch.",
                            world.GetBodyCount());
    }
    ImGui::TextDisabled("Fat AABB margin %.2f m (kAabbMargin)", static_cast<double>(kAabbMargin));

    ImGui::Separator();
    ImGui::TextUnformatted("Iterations (default 8 / 3)");
    int velocityIterations = settings.velocityIterations;
    if (ImGui::SliderInt("Velocity iterations", &velocityIterations, kMinIterations, kMaxIterations, "%d",
                         ImGuiSliderFlags_AlwaysClamp)) {
        settings.velocityIterations = velocityIterations;
        settingsChanged = true;
    }
    int positionIterations = settings.positionIterations;
    if (ImGui::SliderInt("Position iterations", &positionIterations, kMinIterations, kMaxIterations, "%d",
                         ImGuiSliderFlags_AlwaysClamp)) {
        settings.positionIterations = positionIterations;
        settingsChanged = true;
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Warm starting");
    if (ImGui::Checkbox("Use warm starting", &settings.useWarmStarting)) {
        settingsChanged = true;
    }
    // 0 = no inherited impulse (Week 4 behaviour), 1 = full inheritance. The shipped
    // default is 1.0: the Week 5 10-box measurement (recorded in Settings.h) collapses
    // with partial seeds (0.85 -> step 259, 0.90 -> 283, 0.95 -> 332) because PGS has
    // to re-resolve the discarded fraction every frame. Still exposed as the P09 knob.
    if (ImGui::SliderFloat("Warm start factor", &settings.warmStartFactor, 0.0f, 1.0f, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Split impulse (position layer)");
    if (ImGui::Checkbox("Use split impulse", &settings.useSplitImpulse)) {
        settingsChanged = true;
    }
    // The beta/slop sliders only exist for the split-impulse layer; the Baumgarte
    // factor is only used when it is off (the two must never stack, see HANDOFF).
    ImGui::BeginDisabled(!settings.useSplitImpulse);
    if (ImGui::SliderFloat("Split impulse beta", &settings.splitImpulseBeta, 0.0f, 1.0f, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }
    if (ImGui::SliderFloat("Split impulse slop (m)", &settings.splitImpulseSlop, kMinSlop, kMaxSlop, "%.4f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(settings.useSplitImpulse);
    if (ImGui::SliderFloat("Baumgarte (split off)", &settings.baumgarte, 0.0f, 1.0f, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::TextUnformatted("Contact tolerances");
    if (ImGui::SliderFloat("Linear slop (m)", &settings.linearSlop, kMinSlop, kMaxSlop, "%.4f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }
    if (ImGui::SliderFloat("Max linear correction (m)", &settings.maxLinearCorrection, kMinCorrection,
                           kMaxCorrection, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }
    if (ImGui::Checkbox("Use SAT cache", &settings.useSatCache)) {
        settingsChanged = true;
    }
    if (ImGui::SliderFloat("Restitution threshold (m/s)", &settings.restitutionThreshold, 0.0f, 10.0f, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        settingsChanged = true;
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Gravity");
    const Vec2 gravity = world.GetGravity();
    float gravityY = gravity.y;
    // Wired straight to World::SetGravity: the next fixed step uses the new value.
    if (ImGui::SliderFloat("Gravity Y (m/s^2)", &gravityY, kMinGravityY, kMaxGravityY, "%.1f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        world.SetGravity(Vec2{gravity.x, gravityY});
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Material defaults (for built scenes)");
    // BodyDef already gives every body friction 0.3 / restitution 0. A scene that
    // wants a different comparison sets the materials after building (FrictionRamp
    // does), so these sliders are the experiment defaults main.cpp applies when a
    // scene is switched to, plus the explicit "apply now" button below.
    ImGui::SliderFloat("Default friction", &m_materialDefaults.friction, kMinMaterial, kMaxMaterial, "%.2f",
                       ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Default restitution", &m_materialDefaults.restitution, kMinMaterial, kMaxMaterial,
                       "%.2f", ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::Button("Apply to current scene")) {
        actions.applyMaterialDefaultsRequested = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Write the two values above onto friction/restitution of every body in the current world.");
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("World axes", &display.showAxes);
        ImGui::Checkbox("Body AABBs", &display.showAabb);
        // Contact visualisation: points are the manifold anchors; normals show the
        // A->B direction along which the Sequential Impulse solver pushes the pair apart.
        ImGui::Checkbox("Contact points", &display.showContacts);
        ImGui::Checkbox("Contact normals", &display.showNormals);
        ImGui::Checkbox("Velocity arrows (overlay)", &display.showVelocities);
        ImGui::Checkbox("Dear ImGui demo window", &m_showImGuiDemo);
    }

    if (settingsChanged) {
        world.SetSolverSettings(settings);
    }
    ImGui::End();
}

void DebugUI::DrawContactsPanel(World& world, DisplayOptions& display) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + kPanelGap,
                                   viewport->WorkPos.y + kPanelGap + kControlPanelHeight + kPanelGap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(kContactsPanelWidth, kContactsPanelHeight), ImGuiCond_FirstUseEver);
    ImGui::Begin("Contacts");

    // The contact vector is rebuilt every Step, so it is called a list snapshot, not
    // a database: read it, never cache a pointer/index across frames.
    const std::vector<Contact>& contacts = world.GetContacts();
    int32_t touchingCount = 0;
    int32_t pointCount = 0;
    float maxPenetration = 0.0f;
    for (const Contact& contact : contacts) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingCount;
        pointCount += contact.manifold.pointCount;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    ImGui::Text("Touching: %d   points: %d   max pen: %.2f mm", touchingCount, pointCount,
                maxPenetration * kMillimetersPerMeter);

    // Drop a stale highlight: Reset/scene switch invalidate the handles, and a pair
    // that separated this frame no longer has anything to draw.
    bool highlightExists = false;
    if (display.highlightedContactBodyA.IsValid()) {
        for (const Contact& contact : contacts) {
            if (contact.manifold.IsTouching() && contact.bodyA == display.highlightedContactBodyA &&
                contact.bodyB == display.highlightedContactBodyB) {
                highlightExists = true;
                break;
            }
        }
        if (!highlightExists) {
            display.highlightedContactBodyA = BodyHandle{};
            display.highlightedContactBodyB = BodyHandle{};
        }
    }

    const ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                       ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("##contacts", 5, tableFlags, ImVec2(0.0f, kContactsTableHeight))) {
        ImGui::TableSetupColumn("Pair", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Pts", ImGuiTableColumnFlags_WidthFixed, 26.0f);
        ImGui::TableSetupColumn("Pen mm", ImGuiTableColumnFlags_WidthFixed, 56.0f);
        ImGui::TableSetupColumn("Normal", ImGuiTableColumnFlags_WidthFixed, 84.0f);
        ImGui::TableSetupColumn("Imp n/t", ImGuiTableColumnFlags_WidthFixed, 86.0f);
        // Keep the header visible while the per-contact rows scroll.
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();

        for (const Contact& contact : contacts) {
            if (!contact.manifold.IsTouching()) {
                continue;
            }
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);

            const Body* bodyA = world.GetBody(contact.bodyA);
            const Body* bodyB = world.GetBody(contact.bodyB);
            const char* nameA = bodyA != nullptr ? bodyA->debugName.c_str() : "?";
            const char* nameB = bodyB != nullptr ? bodyB->debugName.c_str() : "?";

            const bool selected = contact.bodyA == display.highlightedContactBodyA &&
                                  contact.bodyB == display.highlightedContactBodyB;
            // The ### suffix keeps the widget id stable and unique while the visible
            // label stays "ground | box_3".
            char rowLabel[160];
            std::snprintf(rowLabel, sizeof(rowLabel), "%s | %s###contact_%d_%d", nameA, nameB,
                          contact.bodyA.index, contact.bodyB.index);
            if (ImGui::Selectable(rowLabel, selected, ImGuiSelectableFlags_SpanAllColumns)) {
                if (selected) {
                    // Clicking the highlighted row again clears the highlight.
                    display.highlightedContactBodyA = BodyHandle{};
                    display.highlightedContactBodyB = BodyHandle{};
                } else {
                    display.highlightedContactBodyA = contact.bodyA;
                    display.highlightedContactBodyB = contact.bodyB;
                }
            }

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d", contact.manifold.pointCount);

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.2f", contact.manifold.penetration * kMillimetersPerMeter);

            ImGui::TableSetColumnIndex(3);
            ImGui::Text("(%.2f, %.2f)", contact.manifold.normal.x, contact.manifold.normal.y);

            ImGui::TableSetColumnIndex(4);
            float normalImpulse = 0.0f;
            float tangentImpulse = 0.0f;
            for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
                normalImpulse += contact.manifold.points[pointIndex].normalImpulse;
                tangentImpulse += contact.manifold.points[pointIndex].tangentImpulse;
            }
            ImGui::Text("%.2f/%.2f", normalImpulse, tangentImpulse);
        }
        ImGui::EndTable();
    }

    if (display.highlightedContactBodyA.IsValid()) {
        const Body* bodyA = world.GetBody(display.highlightedContactBodyA);
        const Body* bodyB = world.GetBody(display.highlightedContactBodyB);
        if (bodyA != nullptr && bodyB != nullptr) {
            ImGui::Text("Highlighted: %s | %s", bodyA->debugName.c_str(), bodyB->debugName.c_str());
        }
        if (ImGui::Button("Clear highlight")) {
            display.highlightedContactBodyA = BodyHandle{};
            display.highlightedContactBodyB = BodyHandle{};
        }
    } else {
        ImGui::TextDisabled("Click a row to highlight its point + normal.");
    }
    ImGui::End();
}

void DebugUI::DrawStatusPanel(const World& world, const Demo& currentDemo, const FrameStats& stats) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - kRightColumnOffset,
                                   viewport->WorkPos.y + kPanelGap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(kStatusPanelWidth, kStatusPanelHeight), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status");

    ImGui::Text("FPS: %.1f   Frame: %.2f ms", stats.framesPerSecond, stats.frameMilliseconds);
    // Smoothed in main.cpp over the last frames: a single Step can legitimately sit
    // in cache-miss noise, and the smoothed number is what the devlog quotes.
    ImGui::Text("Step: %.3f ms   Steps/frame: %d", stats.physicsStepMilliseconds, stats.physicsSteps);

    ImGui::Separator();
    ImGui::Text("Scene: %s", currentDemo.Name());
    ImGui::Text("Bodies: %d   Shapes: %d", world.GetBodyCount(), world.GetShapeCount());
    // Islands are a Week 7 feature: BuildIslands() is still an empty reserved phase,
    // so the honest value is 0 and the label says so instead of pretending.
    ImGui::Text("Awake: %d   Islands: %d (Week 7)", world.GetAwakeBodyCount(), world.GetIslandCount());

    // Contact statistics. Only touching manifolds count; a pair with pointCount == 0
    // is separated and is not part of the contact list anyway, but guarding keeps the
    // panel correct if that changes later.
    int32_t touchingContacts = 0;
    int32_t contactPoints = 0;
    float maxPenetration = 0.0f;
    for (const Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingContacts;
        contactPoints += contact.manifold.pointCount;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    ImGui::Separator();
    ImGui::Text("Contacts: %d   Points: %d", touchingContacts, contactPoints);
    ImGui::Text("Max penetration: %.2f mm", maxPenetration * kMillimetersPerMeter);

    // Week 6: which broad phase produced the candidate pairs for the last Step, the
    // testbed mode that selected it, how many proxies it tracks and how many pairs it
    // handed to the narrow phase. The fat AABB margin (Settings.h kAabbMargin) is what
    // makes these counts slightly above the number of actually touching pairs.
    const IBroadPhase* broadPhase = world.GetBroadPhase();
    ImGui::Text("Broad phase: %s   mode: %s", broadPhase != nullptr ? broadPhase->Name() : "None",
                BroadPhaseModeName(m_broadPhaseMode));
    ImGui::Text("Proxies: %d   Candidate pairs: %d", broadPhase != nullptr ? broadPhase->GetProxyCount() : 0,
                world.GetBroadPhasePairCount());

    ImGui::Separator();
    const Vec2 gravity = world.GetGravity();
    ImGui::Text("Gravity: (%.1f, %.1f) m/s^2", gravity.x, gravity.y);
    // Mirror of the Solver panel: confirms the settings actually reached the World
    // (a slider that silently edits a disconnected copy would show up here).
    const SolverSettings& settings = world.GetSolverSettings();
    ImGui::Text("Solver: %d vel / %d pos it", settings.velocityIterations, settings.positionIterations);
    ImGui::Text("  warm %s %.2f   split %s", settings.useWarmStarting ? "on" : "off",
                settings.warmStartFactor, settings.useSplitImpulse ? "on" : "off");
    ImGui::Text("Fixed dt: %.5f s", kFixedTimeStep);
    ImGui::Text("Paused: %s   Speed: %.2fx", m_paused ? "yes" : "no", GetSpeedMultiplier());

    ImGui::End();
}

} // namespace p2
