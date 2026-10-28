#include "DebugUI.h"

#include "physics2d/Core/World.h"

#include <imgui.h>

#include <algorithm>

namespace p2 {
namespace {

// Playback speeds from the design document's panel spec. The multiplier scales how
// much simulated time one rendered frame contributes, so physics stays on the same
// fixed dt and only the accumulator feed changes (Section 4.3).
constexpr int kSpeedCount = 3;
constexpr float kSpeedMultipliers[kSpeedCount] = {1.0f, 0.5f, 0.25f};
constexpr const char* kSpeedLabels[kSpeedCount] = {"1x", "0.5x", "0.25x"};

// Gravity range: -50 m/s^2 covers "very heavy" experiments while +10 m/s^2 lets
// the user flip gravity upwards to sanity-check the integrator sign convention.
constexpr float kMinGravityY = -50.0f;
constexpr float kMaxGravityY = 10.0f;

// Penetration is displayed in millimetres: the interesting Week 2 values are the
// tolerated slop (5 mm) and the transient overlap during a landing, which are
// unreadable as metres in a status panel.
constexpr float kMillimetersPerMeter = 1000.0f;

} // namespace

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
    DrawControlPanel(world, currentDemo, demoNames, selectedDemo, display, actions);
    DrawStatusPanel(world, currentDemo, stats);
    if (m_showImGuiDemo) {
        // Built-in demo window: the fastest way to verify docking, tables and
        // input plumbing work on this machine without writing widgets ourselves.
        ImGui::ShowDemoWindow(&m_showImGuiDemo);
    }
    return actions;
}

void DebugUI::DrawControlPanel(World& world, Demo& currentDemo, const std::vector<const char*>& demoNames,
                               int& selectedDemo, DisplayOptions& display, FrameActions& actions) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // ImGuiCond_FirstUseEver: the placement below is only a default. Once layout
    // persistence is enabled, a user's docked layout wins on later runs.
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 12.0f, viewport->WorkPos.y + 12.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340.0f, 470.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Testbed Controls");

    ImGui::TextUnformatted("Physics2D testbed - Week 2");
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
    const Vec2 gravity = world.GetGravity();
    float gravityY = gravity.y;
    // Directly wired to World::SetGravity so the D6 "slider changes gravity"
    // acceptance check needs no rebuild: the next fixed step uses the new value.
    if (ImGui::SliderFloat("Gravity Y (m/s^2)", &gravityY, kMinGravityY, kMaxGravityY, "%.1f")) {
        world.SetGravity(Vec2{gravity.x, gravityY});
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Display");
    ImGui::Checkbox("World axes", &display.showAxes);
    ImGui::Checkbox("Body AABBs", &display.showAabb);
    // Week 2 contact visualisation: points are the manifold anchors, normals show the
    // A->B direction the temporary response acts along.
    ImGui::Checkbox("Contact points", &display.showContacts);
    ImGui::Checkbox("Contact normals", &display.showNormals);
    ImGui::Checkbox("Dear ImGui demo window", &m_showImGuiDemo);

    ImGui::Separator();
    if (ImGui::Button("Quit")) {
        actions.quitRequested = true;
    }
    ImGui::TextDisabled("Camera: middle-drag pan / wheel zoom / R reset");

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Scene parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        currentDemo.Ui(world);
    }
    ImGui::End();
}

void DebugUI::DrawStatusPanel(const World& world, const Demo& currentDemo, const FrameStats& stats) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    constexpr float kPanelWidth = 300.0f;
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - kPanelWidth - 12.0f, viewport->WorkPos.y + 12.0f),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(kPanelWidth, 260.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status");

    ImGui::Text("FPS: %.1f", stats.framesPerSecond);
    ImGui::Text("Frame: %.2f ms", stats.frameMilliseconds);
    ImGui::Text("Physics step: %.3f ms", stats.physicsStepMilliseconds);
    ImGui::Text("Steps this frame: %d", stats.physicsSteps);

    ImGui::Separator();
    ImGui::Text("Scene: %s", currentDemo.Name());
    ImGui::Text("Bodies: %d   Shapes: %d   Awake: %d", world.GetBodyCount(), world.GetShapeCount(),
                world.GetAwakeBodyCount());

    // Week 2 contact statistics. Only touching manifolds count; a pair with
    // pointCount == 0 is separated and is not part of the contact list anyway, but
    // guarding keeps the panel correct if that changes later.
    int32_t touchingContacts = 0;
    float maxPenetration = 0.0f;
    for (const Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        ++touchingContacts;
        maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
    }
    ImGui::Text("Contacts: %d   Max penetration: %.2f mm", touchingContacts,
                maxPenetration * kMillimetersPerMeter);

    const Vec2 gravity = world.GetGravity();
    ImGui::Text("Gravity: (%.1f, %.1f) m/s^2", gravity.x, gravity.y);
    ImGui::Text("Fixed dt: %.5f s", kFixedTimeStep);
    ImGui::Text("Paused: %s   Speed: %.2fx", m_paused ? "yes" : "no", GetSpeedMultiplier());

    ImGui::End();
}

} // namespace p2
