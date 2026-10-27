#include "Renderer.h"

#include "physics2d/Math/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace p2 {
namespace {

// GLSL 330 core. The testbed is 2D, so position stays vec2 and z is emitted as 0:
// the depth test is disabled, drawing order decides visibility.
constexpr const char* kVertexShaderSource = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec4 aColor;
uniform mat4 uViewProjection;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uViewProjection * vec4(aPosition, 0.0, 1.0);
}
)GLSL";

constexpr const char* kFragmentShaderSource = R"GLSL(
#version 330 core
in vec4 vColor;
out vec4 oColor;
void main() {
    oColor = vColor;
}
)GLSL";

// Info-log buffers include the terminating NUL in their reported length, so one
// extra byte keeps the std::string construction bounded and correct.
std::string FetchShaderLog(GLuint shader) {
    GLint logLength = 0;
    gl::GetShaderiv(shader, gl::kInfoLogLength, &logLength);
    if (logLength <= 1) {
        return {};
    }
    std::vector<char> log(static_cast<std::size_t>(logLength) + 1, '\0');
    GLsizei written = 0;
    gl::GetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &written, log.data());
    return std::string(log.data(), static_cast<std::size_t>(written));
}

std::string FetchProgramLog(GLuint program) {
    GLint logLength = 0;
    gl::GetProgramiv(program, gl::kInfoLogLength, &logLength);
    if (logLength <= 1) {
        return {};
    }
    std::vector<char> log(static_cast<std::size_t>(logLength) + 1, '\0');
    GLsizei written = 0;
    gl::GetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &written, log.data());
    return std::string(log.data(), static_cast<std::size_t>(written));
}

// Returns 0 on failure. The compiler log is printed immediately (not only on
// failure) because driver warnings explain most "shader silently does nothing"
// bugs, and --smoke runs never show the UI.
GLuint CompileShader(GLenum shaderType, const char* source, const char* stageName, std::string& error) {
    const GLuint shader = gl::CreateShader(shaderType);
    if (shader == 0) {
        error = std::string("glCreateShader failed for the ") + stageName + " shader";
        return 0;
    }
    const char* sources[1] = {source};
    gl::ShaderSource(shader, 1, sources, nullptr);
    gl::CompileShader(shader);

    GLint compiled = GL_FALSE;
    gl::GetShaderiv(shader, gl::kCompileStatus, &compiled);
    const std::string log = FetchShaderLog(shader);
    if (!log.empty()) {
        std::fprintf(stderr, "[renderer] %s shader log:\n%s\n", stageName, log.c_str());
    }
    if (compiled == GL_FALSE) {
        error = std::string("failed to compile the ") + stageName + " shader:\n" + log;
        gl::DeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

Renderer::~Renderer() {
    // Shutdown performs GL calls, therefore the caller must keep the GL context
    // current until this object is destroyed. main.cpp guarantees that by
    // declaring the renderer after the window/ImGui guards.
    Shutdown();
}

bool Renderer::Initialize(std::string& error) {
    error.clear();
    if (m_initialized) {
        error = "renderer is already initialized";
        return false;
    }

    std::string loadError;
    if (!gl::LoadFunctions(loadError)) {
        error = loadError;
        return false;
    }

    const GLuint vertexShader = CompileShader(gl::kVertexShader, kVertexShaderSource, "vertex", error);
    if (vertexShader == 0) {
        return false;
    }
    const GLuint fragmentShader =
        CompileShader(gl::kFragmentShader, kFragmentShaderSource, "fragment", error);
    if (fragmentShader == 0) {
        gl::DeleteShader(vertexShader);
        return false;
    }

    m_program = gl::CreateProgram();
    if (m_program == 0) {
        error = "glCreateProgram failed";
        gl::DeleteShader(vertexShader);
        gl::DeleteShader(fragmentShader);
        return false;
    }
    gl::AttachShader(m_program, vertexShader);
    gl::AttachShader(m_program, fragmentShader);
    gl::LinkProgram(m_program);
    // The shader objects are reference-counted by the program; deleting them here
    // is the standard way to avoid leaking them if linking later fails.
    gl::DeleteShader(vertexShader);
    gl::DeleteShader(fragmentShader);

    GLint linked = GL_FALSE;
    gl::GetProgramiv(m_program, gl::kLinkStatus, &linked);
    const std::string linkLog = FetchProgramLog(m_program);
    if (!linkLog.empty()) {
        std::fprintf(stderr, "[renderer] program link log:\n%s\n", linkLog.c_str());
    }
    if (linked == GL_FALSE) {
        error = "failed to link the line shader program:\n" + linkLog;
        gl::DeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    m_viewProjectionLocation = gl::GetUniformLocation(m_program, "uViewProjection");
    if (m_viewProjectionLocation < 0) {
        error = "shader program does not expose the uViewProjection uniform";
        gl::DeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    // Core profile requires a bound VAO for every draw call, even with a single
    // interleaved attribute stream.
    gl::GenVertexArrays(1, &m_vao);
    gl::BindVertexArray(m_vao);
    gl::GenBuffers(1, &m_vbo);
    gl::BindBuffer(gl::kArrayBuffer, m_vbo);

    const GLsizei stride = static_cast<GLsizei>(sizeof(Vertex));
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(0));
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(
        1, 4, GL_FLOAT, GL_FALSE, stride,
        reinterpret_cast<const void*>(static_cast<std::uintptr_t>(offsetof(Vertex, color))));

    gl::BindVertexArray(0);
    gl::BindBuffer(gl::kArrayBuffer, 0);
    gl::UseProgram(0);

    // 1-pixel lines: the core profile only guarantees a line width of 1.0 and
    // drivers may clamp larger values silently, so the renderer does not depend on
    // glLineWidth for its look (P14 flags this explicitly).
    glLineWidth(1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // A stale error here would otherwise surface as a confusing failure on the
    // first frame; report it but keep running (the driver may have logged a
    // harmless state warning from the ImGui backend's setup).
    const GLenum glErrorCode = glGetError();
    if (glErrorCode != GL_NO_ERROR) {
        std::fprintf(stderr, "[renderer] warning: OpenGL error 0x%04X after initialization\n",
                     static_cast<unsigned int>(glErrorCode));
    }

    m_initialized = true;
    return true;
}

void Renderer::Shutdown() {
    m_vertices.clear();
    if (m_vbo != 0) {
        gl::DeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }
    if (m_vao != 0) {
        gl::DeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
    if (m_program != 0) {
        gl::DeleteProgram(m_program);
        m_program = 0;
    }
    m_viewProjectionLocation = -1;
    m_vboCapacityBytes = 0;
    m_initialized = false;
    m_begun = false;
}

void Renderer::Begin(const Camera& camera) {
    if (!m_initialized) {
        return;
    }
    m_viewProjection = camera.GetViewProjectionMatrix();
    m_begun = true;
    // Typical debug scenes are a few thousand segments; reserving once avoids
    // reallocation in the middle of filling the batch every frame.
    if (m_vertices.capacity() < 8192) {
        m_vertices.reserve(8192);
    }
}

void Renderer::End() {
    Flush();
    m_begun = false;
}

void Renderer::PushVertex(Vec2 position, Color color) {
    if (!m_initialized || !m_begun) {
        return;
    }
    m_vertices.push_back(Vertex{{position.x, position.y}, {color.r, color.g, color.b, color.a}});
}

void Renderer::PushSegment(Vec2 from, Vec2 to, Color color) {
    PushVertex(from, color);
    PushVertex(to, color);
}

void Renderer::Flush() {
    if (!m_initialized || m_vertices.empty()) {
        return;
    }
    const std::size_t vertexCount = m_vertices.size();
    const std::size_t byteCount = vertexCount * sizeof(Vertex);

    gl::UseProgram(m_program);
    gl::BindVertexArray(m_vao);
    gl::BindBuffer(gl::kArrayBuffer, m_vbo);

    // Grow the VBO geometrically: BufferData reallocates storage, BufferSubData
    // streams the new frame into it. Orphaning the buffer lets the driver avoid a
    // synchronisation stall in the common case where the previous draw is still
    // in flight.
    if (byteCount > m_vboCapacityBytes) {
        std::size_t newCapacity = m_vboCapacityBytes == 0 ? 64 * 1024 : m_vboCapacityBytes;
        while (newCapacity < byteCount) {
            newCapacity *= 2;
        }
        gl::BufferData(gl::kArrayBuffer, static_cast<std::ptrdiff_t>(newCapacity), nullptr, gl::kDynamicDraw);
        m_vboCapacityBytes = newCapacity;
    }
    gl::BufferSubData(gl::kArrayBuffer, 0, static_cast<std::ptrdiff_t>(byteCount), m_vertices.data());

    gl::UniformMatrix4fv(m_viewProjectionLocation, 1, GL_FALSE, m_viewProjection.data());
    gl::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertexCount));

    // Clear instead of shrink: the capacity stays for the next frame.
    m_vertices.clear();
}

void Renderer::DrawSegment(Vec2 from, Vec2 to, Color color) {
    PushSegment(from, to, color);
}

void Renderer::DrawCircle(Vec2 center, float radius, Color color, int32_t segments) {
    if (radius <= 0.0f) {
        return;
    }
    // A line loop is not available inside one GL_LINES batch, so the circle is
    // approximated by `segments` short segments; 48 is visually smooth at the
    // largest radius the testbed uses (about 2 m * 40 px/m).
    const int32_t segmentCount = segments < 3 ? 3 : segments;
    const float step = kTwoPi / static_cast<float>(segmentCount);
    Vec2 previous{center.x + radius, center.y};
    for (int32_t i = 1; i <= segmentCount; ++i) {
        const float angle = step * static_cast<float>(i);
        const Vec2 current{center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)};
        PushSegment(previous, current, color);
        previous = current;
    }
}

void Renderer::DrawPolygon(const Vec2* worldVertices, std::size_t vertexCount, Color color) {
    if (worldVertices == nullptr || vertexCount < 2) {
        return;
    }
    // Closed outline: the last vertex connects back to the first.
    for (std::size_t i = 0; i < vertexCount; ++i) {
        PushSegment(worldVertices[i], worldVertices[(i + 1) % vertexCount], color);
    }
}

void Renderer::DrawAABB(const AABB& box, Color color) {
    const Vec2 lowerLeft = box.lowerBound;
    const Vec2 upperRight = box.upperBound;
    const Vec2 lowerRight{upperRight.x, lowerLeft.y};
    const Vec2 upperLeft{lowerLeft.x, upperRight.y};
    PushSegment(lowerLeft, lowerRight, color);
    PushSegment(lowerRight, upperRight, color);
    PushSegment(upperRight, upperLeft, color);
    PushSegment(upperLeft, lowerLeft, color);
}

void Renderer::DrawArrow(Vec2 from, Vec2 to, Color color, float headLength) {
    const Vec2 delta = to - from;
    const float length = Length(delta);
    if (length < kEpsilon) {
        return;
    }
    const Vec2 direction = delta / length;
    // Clamp the head so a very short arrow does not render as a blob wider than
    // the shaft it is attached to.
    const float head = std::min(headLength, length * 0.5f);
    const Vec2 base = to - direction * head;
    const Vec2 side = Skew(direction) * (head * 0.5f);
    PushSegment(from, to, color);
    PushSegment(to, base + side, color);
    PushSegment(to, base - side, color);
}

void Renderer::DrawPoint(Vec2 point, Color color, float radius) {
    if (radius <= 0.0f) {
        return;
    }
    // GL_LINES cannot draw a round point; a small cross reads as a marker and
    // stays visible on top of other lines (contact points, body origins).
    PushSegment({point.x - radius, point.y}, {point.x + radius, point.y}, color);
    PushSegment({point.x, point.y - radius}, {point.x, point.y + radius}, color);
}

void Renderer::DrawAxes(float length) {
    if (length <= 0.0f) {
        return;
    }
    constexpr Color xAxisColor{0.85f, 0.25f, 0.25f, 1.0f};
    constexpr Color yAxisColor{0.25f, 0.85f, 0.35f, 1.0f};
    PushSegment({0.0f, 0.0f}, {length, 0.0f}, xAxisColor);
    PushSegment({0.0f, 0.0f}, {0.0f, length}, yAxisColor);
}

} // namespace p2
