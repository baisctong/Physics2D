#pragma once

// p2::Renderer - batched immediate-mode line renderer (Week 1 D5).
//
// Why one dynamic VBO + GL_LINES for everything:
//  - Every debug primitive a physics testbed needs (circles, polygons, AABBs,
//    arrows, contact points) is a line loop in disguise. Collapsing them into a
//    single triangle-free batch means one buffer upload and one draw call per
//    frame, independent of body count - which matters later when 200-body scenes
//    are on screen.
//  - The design document's Layer-2 scope explicitly targets "wireframe is
//    enough" (Section 1.1); a full retained renderer would be wasted effort.
//
// Lifecycle: Initialize() once after the GL context is current, Shutdown() while
// it is still current. Every frame: Begin(camera) -> Draw* calls -> End() (one
// Flush). Vertices are world-space metres; the camera supplies the MVP matrix.

#include "Camera.h"
#include "GLLoader.h"

#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Vec2.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace p2 {

// Straight RGBA float color. Kept a plain aggregate-like value type so demos can
// write `Color{0.9f, 0.5f, 0.2f}` without pulling in a color library.
struct Color {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;

    constexpr Color() = default;
    constexpr Color(float red, float green, float blue, float alpha = 1.0f)
        : r(red), g(green), b(blue), a(alpha) {}
};

class Renderer {
  public:
    Renderer() = default;
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    // Compiles the shaders and allocates the VAO/VBO. On failure `error` contains
    // the shader compiler/linker log; the caller must treat that as fatal because
    // nothing can be drawn afterwards.
    [[nodiscard]] bool Initialize(std::string& error);
    void Shutdown();

    // Begin() starts a batch and records the camera MVP; End() flushes it. Calling
    // Begin/End exactly once per frame is the intended use: it keeps the design
    // invariant "one draw call per frame" visible at the call site.
    void Begin(const Camera& camera);
    void End();

    // ---- primitives (world space, metres) ------------------------------------
    void DrawSegment(Vec2 from, Vec2 to, Color color);
    void DrawCircle(Vec2 center, float radius, Color color, int32_t segments = 48);
    void DrawPolygon(const Vec2* worldVertices, std::size_t vertexCount, Color color);
    void DrawAABB(const AABB& box, Color color);
    // Arrow used for normals / velocities / forces. The head is clamped so short
    // arrows still look like arrows instead of a triangle.
    void DrawArrow(Vec2 from, Vec2 to, Color color, float headLength = 0.2f);
    // Contact-point style marker. GL_LINES cannot draw a round point, so the
    // marker is a small cross - visually distinct from a segment endpoint.
    void DrawPoint(Vec2 point, Color color, float radius = 0.06f);
    // X axis in red, Y axis in green; useful as a world-space reference when the
    // demo scene is mostly empty (early weeks).
    void DrawAxes(float length = 5.0f);

  private:
    // 24-byte vertex: position vec2 + color vec4. A plain float array keeps the
    // struct standard-layout so offsets are guaranteed and offsetof is valid.
    struct Vertex {
        float position[2];
        float color[4];
    };

    static_assert(sizeof(Vertex) == 6 * sizeof(float), "VAO attribute layout depends on this stride");

    void PushVertex(Vec2 position, Color color);
    void PushSegment(Vec2 from, Vec2 to, Color color);
    void Flush();

    GLuint m_program = 0;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    GLint m_viewProjectionLocation = -1;

    std::vector<Vertex> m_vertices;
    std::size_t m_vboCapacityBytes = 0;
    std::array<float, 16> m_viewProjection{};
    bool m_begun = false;
    bool m_initialized = false;
};

} // namespace p2
