#include "Camera.h"

#include "physics2d/Math/MathUtils.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

namespace p2 {
namespace {

[[nodiscard]] float ClampZoom(float zoom) noexcept {
    return Clamp(zoom, Camera::kMinZoom, Camera::kMaxZoom);
}

} // namespace

void Camera::SetViewportSize(int32_t widthPixels, int32_t heightPixels) noexcept {
    // A minimized window reports 0x0. Keeping at least one pixel avoids a
    // division by zero in the projection matrix; no visible frame is produced
    // while the window is minimized anyway.
    m_widthPixels = widthPixels > 1 ? widthPixels : 1;
    m_heightPixels = heightPixels > 1 ? heightPixels : 1;
}

void Camera::Reset() noexcept {
    m_center = {0.0f, 0.0f};
    m_zoom = 1.0f;
    m_panning = false;
    m_pendingScrollSteps = 0.0f;
    m_resetRequested = false;
}

void Camera::InstallInputCallbacks(GLFWwindow* window) {
    if (window == nullptr) {
        return;
    }
    // The window user pointer is used to reach this camera from the C callbacks.
    // The testbed owns the window, so no other subsystem competes for it.
    glfwSetWindowUserPointer(window, this);
    glfwSetScrollCallback(window, &Camera::ScrollCallback);
    glfwSetKeyCallback(window, &Camera::KeyCallback);
}

void Camera::ScrollCallback(GLFWwindow* window, double /*xOffset*/, double yOffset) {
    auto* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (camera != nullptr) {
        // Store the wheel movement and apply it in UpdateInput: only then do we
        // know whether ImGui owns the mouse this frame and where the cursor is.
        camera->m_pendingScrollSteps += static_cast<float>(yOffset);
    }
}

void Camera::KeyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (key != GLFW_KEY_R || action != GLFW_PRESS) {
        return;
    }
    auto* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (camera != nullptr) {
        camera->m_resetRequested = true;
    }
}

void Camera::UpdateInput(GLFWwindow* window, bool allowInput) {
    if (window == nullptr) {
        return;
    }

    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window, &cursorX, &cursorY);
    const Vec2 cursorPixels{static_cast<float>(cursorX), static_cast<float>(cursorY)};

    if (!allowInput) {
        // ImGui owns mouse/keyboard input this frame. Dropping the pending events
        // (instead of queueing them) is deliberate: a wheel scroll over a slider
        // must not zoom the view once the cursor leaves the panel.
        m_panning = false;
        m_pendingScrollSteps = 0.0f;
        m_resetRequested = false;
        m_lastCursorPixels = cursorPixels;
        return;
    }

    const bool middleDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (middleDown) {
        if (!m_panning) {
            // First frame of the drag only records the anchor; applying the very
            // first delta would make the view jump by the distance travelled
            // before the button was registered as pressed.
            m_panning = true;
            m_lastCursorPixels = cursorPixels;
        } else {
            PanByPixels(cursorPixels - m_lastCursorPixels);
            m_lastCursorPixels = cursorPixels;
        }
    } else {
        m_panning = false;
        m_lastCursorPixels = cursorPixels;
    }

    if (m_pendingScrollSteps != 0.0f) {
        ZoomAtPixels(m_pendingScrollSteps, cursorPixels);
        m_pendingScrollSteps = 0.0f;
    }

    if (m_resetRequested) {
        Reset();
    }
}

void Camera::PanByPixels(Vec2 deltaPixels) noexcept {
    const float pixelsPerMeter = GetPixelsPerMeter();
    if (pixelsPerMeter <= 0.0f) {
        return;
    }
    // Drag direction: the scene follows the cursor, so the camera center moves the
    // opposite way. Screen +y is down while world +y is up (Section 4.2).
    m_center.x -= deltaPixels.x / pixelsPerMeter;
    m_center.y += deltaPixels.y / pixelsPerMeter;
}

void Camera::ZoomAtPixels(float scrollSteps, Vec2 anchorPixels) noexcept {
    if (scrollSteps == 0.0f) {
        return;
    }
    // Cursor-anchored zoom: remember which world point is under the cursor, change
    // the scale, then translate the center so that same world point stays under
    // the cursor. This is what makes wheel zoom feel like a map, not a spring.
    const Vec2 worldBefore = ScreenToWorld(anchorPixels);
    m_zoom = ClampZoom(m_zoom * std::pow(kZoomStep, scrollSteps));
    const Vec2 worldAfter = ScreenToWorld(anchorPixels);
    m_center += worldBefore - worldAfter;
}

Vec2 Camera::ScreenToWorld(Vec2 screenPixels) const noexcept {
    const float pixelsPerMeter = GetPixelsPerMeter();
    const float halfWidthPixels = static_cast<float>(m_widthPixels) * 0.5f;
    const float halfHeightPixels = static_cast<float>(m_heightPixels) * 0.5f;
    return {(screenPixels.x - halfWidthPixels) / pixelsPerMeter + m_center.x,
            (halfHeightPixels - screenPixels.y) / pixelsPerMeter + m_center.y};
}

Vec2 Camera::WorldToScreen(Vec2 world) const noexcept {
    const float pixelsPerMeter = GetPixelsPerMeter();
    const float halfWidthPixels = static_cast<float>(m_widthPixels) * 0.5f;
    const float halfHeightPixels = static_cast<float>(m_heightPixels) * 0.5f;
    return {(world.x - m_center.x) * pixelsPerMeter + halfWidthPixels,
            -(world.y - m_center.y) * pixelsPerMeter + halfHeightPixels};
}

std::array<float, 16> Camera::GetViewProjectionMatrix() const noexcept {
    const float pixelsPerMeter = GetPixelsPerMeter();
    const float halfWidthMeters = static_cast<float>(m_widthPixels) * 0.5f / pixelsPerMeter;
    const float halfHeightMeters = static_cast<float>(m_heightPixels) * 0.5f / pixelsPerMeter;
    const float scaleX = 1.0f / halfWidthMeters;
    const float scaleY = 1.0f / halfHeightMeters;

    // Column-major layout (GLSL default): element [column * 4 + row]. The ortho
    // matrix maps the visible world rectangle to NDC [-1, 1] on both axes:
    //   ndc.x = (world.x - center.x) * scaleX
    //   ndc.y = (world.y - center.y) * scaleY
    std::array<float, 16> matrix{};
    matrix[0] = scaleX;                // m[0][0] = 2 / (right - left)
    matrix[5] = scaleY;                // m[1][1] = 2 / (top - bottom)
    matrix[10] = 1.0f;                 // z passes through; the testbed draws in 2D
    matrix[12] = -m_center.x * scaleX; // m[0][3] translation
    matrix[13] = -m_center.y * scaleY; // m[1][3] translation
    matrix[15] = 1.0f;                 // homogeneous w stays 1
    return matrix;
}

} // namespace p2
