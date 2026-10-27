#pragma once

// Minimal OpenGL 3.3 core-profile loader for the testbed (Layer 2 only).
//
// Why a hand-written loader instead of glad/glew:
//  - The testbed needs ~25 entry points; a generator adds a dependency and a
//    code-generation step for no benefit (Section 1.2 keeps the dependency list
//    short and explicit).
//  - <GL/gl.h> from the Windows SDK only declares OpenGL 1.1, so everything that
//    the 3.3 core profile introduced must be resolved at runtime via
//    glfwGetProcAddress.
//
// Design: header-only with `inline` variables, so every translation unit that
// includes this file sees the same function pointers without an extra .cpp. The
// symbols live in p2::gl to avoid colliding with the loader Dear ImGui's OpenGL3
// backend declares in its own translation unit.
//
// Usage (only after the GLFW window exists and glfwMakeContextCurrent ran):
//     std::string error;
//     if (!p2::gl::LoadFunctions(error)) { /* report error and stop */ }

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <GL/gl.h>

// GLFW supplies glfwGetProcAddress (and pulls in the system GL header on every
// platform). Including it here keeps the loader self-sufficient for its single
// external call.
#include <GLFW/glfw3.h>

#include <cstddef>
#include <string>

namespace p2 {
namespace gl {

// OpenGL entry points use the __stdcall convention on Windows. On other
// platforms the C default convention matches the GL C ABI, so the macro is empty.
#if defined(_WIN32)
#define P2_GL_CALLING_CONVENTION APIENTRY
#else
#define P2_GL_CALLING_CONVENTION
#endif

// ---- function pointer types -------------------------------------------------
// Only the subset the testbed renderer uses is declared. Signatures follow the
// OpenGL 3.3 core specification; `std::ptrdiff_t` is used for GLsizeiptr /
// GLintptr because that is their definition in glcorearb.h and on Windows
// <GL/gl.h> does not define those two typedefs at all.

using PFN_GenVertexArrays = void(P2_GL_CALLING_CONVENTION*)(GLsizei n, GLuint* arrays);
using PFN_BindVertexArray = void(P2_GL_CALLING_CONVENTION*)(GLuint array);
using PFN_DeleteVertexArrays = void(P2_GL_CALLING_CONVENTION*)(GLsizei n, const GLuint* arrays);
using PFN_GenBuffers = void(P2_GL_CALLING_CONVENTION*)(GLsizei n, GLuint* buffers);
using PFN_BindBuffer = void(P2_GL_CALLING_CONVENTION*)(GLenum target, GLuint buffer);
using PFN_BufferData = void(P2_GL_CALLING_CONVENTION*)(GLenum target, std::ptrdiff_t size, const void* data,
                                                       GLenum usage);
using PFN_BufferSubData = void(P2_GL_CALLING_CONVENTION*)(GLenum target, std::ptrdiff_t offset,
                                                          std::ptrdiff_t size, const void* data);
using PFN_DeleteBuffers = void(P2_GL_CALLING_CONVENTION*)(GLsizei n, const GLuint* buffers);
using PFN_EnableVertexAttribArray = void(P2_GL_CALLING_CONVENTION*)(GLuint index);
using PFN_VertexAttribPointer = void(P2_GL_CALLING_CONVENTION*)(GLuint index, GLint size, GLenum type,
                                                                GLboolean normalized, GLsizei stride,
                                                                const void* pointer);
using PFN_CreateShader = GLuint(P2_GL_CALLING_CONVENTION*)(GLenum type);
using PFN_ShaderSource = void(P2_GL_CALLING_CONVENTION*)(GLuint shader, GLsizei count,
                                                         const char* const* string, const GLint* length);
using PFN_CompileShader = void(P2_GL_CALLING_CONVENTION*)(GLuint shader);
using PFN_GetShaderiv = void(P2_GL_CALLING_CONVENTION*)(GLuint shader, GLenum pname, GLint* params);
using PFN_GetShaderInfoLog = void(P2_GL_CALLING_CONVENTION*)(GLuint shader, GLsizei bufSize, GLsizei* length,
                                                             char* infoLog);
using PFN_DeleteShader = void(P2_GL_CALLING_CONVENTION*)(GLuint shader);
using PFN_CreateProgram = GLuint(P2_GL_CALLING_CONVENTION*)(void);
using PFN_AttachShader = void(P2_GL_CALLING_CONVENTION*)(GLuint program, GLuint shader);
using PFN_LinkProgram = void(P2_GL_CALLING_CONVENTION*)(GLuint program);
using PFN_GetProgramiv = void(P2_GL_CALLING_CONVENTION*)(GLuint program, GLenum pname, GLint* params);
using PFN_GetProgramInfoLog = void(P2_GL_CALLING_CONVENTION*)(GLuint program, GLsizei bufSize,
                                                              GLsizei* length, char* infoLog);
using PFN_DeleteProgram = void(P2_GL_CALLING_CONVENTION*)(GLuint program);
using PFN_UseProgram = void(P2_GL_CALLING_CONVENTION*)(GLuint program);
using PFN_GetUniformLocation = GLint(P2_GL_CALLING_CONVENTION*)(GLuint program, const char* name);
using PFN_UniformMatrix4fv = void(P2_GL_CALLING_CONVENTION*)(GLint location, GLsizei count,
                                                             GLboolean transpose, const GLfloat* value);
using PFN_DrawArrays = void(P2_GL_CALLING_CONVENTION*)(GLenum mode, GLint first, GLsizei count);

#undef P2_GL_CALLING_CONVENTION

// ---- entry point table --------------------------------------------------------
// X(Name, glToken): the first field names the inline variable, the second is the
// string handed to glfwGetProcAddress. Keeping the table in one place means
// declaration and loading can never drift apart.
#define P2_GL_FUNCTION_LIST(X)                                                                               \
    X(GenVertexArrays, glGenVertexArrays)                                                                    \
    X(BindVertexArray, glBindVertexArray)                                                                    \
    X(DeleteVertexArrays, glDeleteVertexArrays)                                                              \
    X(GenBuffers, glGenBuffers)                                                                              \
    X(BindBuffer, glBindBuffer)                                                                              \
    X(BufferData, glBufferData)                                                                              \
    X(BufferSubData, glBufferSubData)                                                                        \
    X(DeleteBuffers, glDeleteBuffers)                                                                        \
    X(EnableVertexAttribArray, glEnableVertexAttribArray)                                                    \
    X(VertexAttribPointer, glVertexAttribPointer)                                                            \
    X(CreateShader, glCreateShader)                                                                          \
    X(ShaderSource, glShaderSource)                                                                          \
    X(CompileShader, glCompileShader)                                                                        \
    X(GetShaderiv, glGetShaderiv)                                                                            \
    X(GetShaderInfoLog, glGetShaderInfoLog)                                                                  \
    X(DeleteShader, glDeleteShader)                                                                          \
    X(CreateProgram, glCreateProgram)                                                                        \
    X(AttachShader, glAttachShader)                                                                          \
    X(LinkProgram, glLinkProgram)                                                                            \
    X(GetProgramiv, glGetProgramiv)                                                                          \
    X(GetProgramInfoLog, glGetProgramInfoLog)                                                                \
    X(DeleteProgram, glDeleteProgram)                                                                        \
    X(UseProgram, glUseProgram)                                                                              \
    X(GetUniformLocation, glGetUniformLocation)                                                              \
    X(UniformMatrix4fv, glUniformMatrix4fv)                                                                  \
    X(DrawArrays, glDrawArrays)

#define P2_GL_DECLARE_FUNCTION(name, token) inline PFN_##name name = nullptr;
P2_GL_FUNCTION_LIST(P2_GL_DECLARE_FUNCTION)
#undef P2_GL_DECLARE_FUNCTION

// Resolves every entry point. Returns false and fills `error` with the first
// missing function name; a missing core function means the driver did not really
// give us a 3.3 core context, which is a hard failure for the renderer.
[[nodiscard]] inline bool LoadFunctions(std::string& error) {
    error.clear();
    bool allLoaded = true;
#define P2_GL_LOAD_FUNCTION(name, token)                                                                     \
    name = reinterpret_cast<PFN_##name>(glfwGetProcAddress(#token));                                         \
    if (name == nullptr && allLoaded) {                                                                      \
        error = "required OpenGL 3.3 entry point is missing: " #token;                                       \
        allLoaded = false;                                                                                   \
    }
    P2_GL_FUNCTION_LIST(P2_GL_LOAD_FUNCTION)
#undef P2_GL_LOAD_FUNCTION
    return allLoaded;
}

// ---- enums introduced after GL 1.1 -------------------------------------------
// <GL/gl.h> only declares the OpenGL 1.1 enums, so the 3.3 values the renderer
// needs are spelled out here (values from the OpenGL 3.3 core specification).
inline constexpr GLenum kArrayBuffer = 0x8892u;    // GL_ARRAY_BUFFER
inline constexpr GLenum kDynamicDraw = 0x88E8u;    // GL_DYNAMIC_DRAW
inline constexpr GLenum kVertexShader = 0x8B31u;   // GL_VERTEX_SHADER
inline constexpr GLenum kFragmentShader = 0x8B30u; // GL_FRAGMENT_SHADER
inline constexpr GLenum kCompileStatus = 0x8B81u;  // GL_COMPILE_STATUS
inline constexpr GLenum kLinkStatus = 0x8B82u;     // GL_LINK_STATUS
inline constexpr GLenum kInfoLogLength = 0x8B84u;  // GL_INFO_LOG_LENGTH

} // namespace gl
} // namespace p2
