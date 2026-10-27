#pragma once

// p2::Camera - world <-> screen mapping for the testbed (Week 1 D5).
//
// Rendering happens in world space (metres); this is the single place where
// physical units are converted to pixels. `kPixelsPerMeter` lives here and only
// here, which is the project rule from Section 4.1: physics code never mentions
// pixels.
//
// Screen convention: GLFW/ImGui report the cursor with the origin at the TOP-LEFT
// and +y pointing down, while the world is +y up (Section 4.2). Camera hides that
// asymmetry so demo code can keep working in metres.
//
// Column-major 4x4 MVP: the renderer feeds it straight into
// glUniformMatrix4fv(..., GL_FALSE, ...), matching the GLSL mat4 memory layout.

#include "physics2d/Math/Vec2.h"

#include <array>
#include <cstdint>

struct GLFWwindow; // forward-declared: Camera.h does not force a GLFW include

namespace p2 {

class Camera {
  public:
    // 40 px per metre (Section 4.1). A 1280x800 window therefore shows 32x20 m,
    // which comfortably contains the Week 1 demo (ball falls from y=5 to y=-8).
    static constexpr float kPixelsPerMeter = 40.0f;

    // Zoom limits keep the ortho projection far from zero-size (NaN/Inf) and stop
    // the user from scrolling into numerical noise or into empty space forever.
    static constexpr float kMinZoom = 0.1f;
    static constexpr float kMaxZoom = 20.0f;
    static constexpr float kZoomStep = 1.1f;

    void SetViewportSize(int32_t widthPixels, int32_t heightPixels) noexcept;

    // Center (0,0) at 1x zoom. Bound to the R key, and also the state the
    // "reset view" path restores when a demo is switched.
    void Reset() noexcept;

    // Installs the scroll/key GLFW callbacks that feed this camera. Must be called
    // BEFORE ImGui_ImplGlfw_InitForOpenGL: the ImGui backend chains whatever
    // callbacks already exist, so installing ours first lets ImGui keep its own
    // mouse-wheel handling while still forwarding events to the camera.
    void InstallInputCallbacks(GLFWwindow* window);

    // Middle-mouse drag pans, wheel zooms around the cursor, R resets. `allowInput`
    // is false while ImGui owns mouse/keyboard input, so dragging a slider never
    // also moves the camera.
    void UpdateInput(GLFWwindow* window, bool allowInput);

    [[nodiscard]] Vec2 ScreenToWorld(Vec2 screenPixels) const noexcept;
    [[nodiscard]] Vec2 WorldToScreen(Vec2 world) const noexcept;

    [[nodiscard]] float GetPixelsPerMeter() const noexcept { return kPixelsPerMeter * m_zoom; }
    [[nodiscard]] float GetZoom() const noexcept { return m_zoom; }
    [[nodiscard]] Vec2 GetCenter() const noexcept { return m_center; }

    // Column-major mat4 (16 floats) mapping world space to OpenGL NDC.
    [[nodiscard]] std::array<float, 16> GetViewProjectionMatrix() const noexcept;

  private:
    static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void PanByPixels(Vec2 deltaPixels) noexcept;
    void ZoomAtPixels(float scrollSteps, Vec2 anchorPixels) noexcept;

    Vec2 m_center{0.0f, 0.0f};
    float m_zoom = 1.0f;
    int32_t m_widthPixels = 1280;
    int32_t m_heightPixels = 800;

    Vec2 m_lastCursorPixels{0.0f, 0.0f};
    float m_pendingScrollSteps = 0.0f;
    bool m_panning = false;
    bool m_resetRequested = false;
};

} // namespace p2
