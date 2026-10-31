# Third-party dependencies (Section 2.2).
#
# The design document allows two dependency strategies and insists on using only
# one across the project. This repository uses **FetchContent**, not vcpkg,
# because the project must be buildable from a clean `git clone` on Windows and
# Linux without bootstrapping a vcpkg tree:
#   - GoogleTest : test framework (v1.15.2)
#   - Eigen      : cross-validation only, tests target only (3.4.0)
#   - GLFW       : window/input for the testbed (3.4)
#   - Dear ImGui : debug UI for the testbed (docking branch, pinned commit)
#
# `physics2d_core` links NONE of these. Everything below is consumed by
# testbed/tests/bench targets only, which is the "Layer 1 has zero third-party
# dependencies" rule of Section 1.2.
#
# Offline / air-gapped builds: configure once online, then pass
#   -DPHYSICS2D_OFFLINE=ON
# to reuse sources already downloaded into build/<preset>/_deps.

include(FetchContent)

option(PHYSICS2D_OFFLINE "Do not download dependencies; reuse already populated sources" OFF)
if(PHYSICS2D_OFFLINE)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif()

# ---------------------------------------------------------------------------
# GoogleTest (tests only)
# ---------------------------------------------------------------------------
if(PHYSICS2D_BUILD_TESTS)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# ---------------------------------------------------------------------------
# Eigen (tests only: independent implementation used to cross-check the
# hand-written Vec2/Mat2 math, tolerance 1e-6)
# ---------------------------------------------------------------------------
if(PHYSICS2D_BUILD_TESTS AND PHYSICS2D_USE_EIGEN)
    # SOURCE_SUBDIR points at a non-existent directory on purpose: Eigen ships
    # its own CMake project, but we only need the headers. This makes
    # FetchContent populate the source tree without add_subdirectory(Eigen).
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR _p2_no_cmake_subdir)
    FetchContent_MakeAvailable(eigen)

    add_library(p2_eigen INTERFACE)
    add_library(p2::eigen ALIAS p2_eigen)
    target_include_directories(p2_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
    target_compile_features(p2_eigen INTERFACE cxx_std_17)
elseif(PHYSICS2D_BUILD_TESTS)
    message(STATUS "Physics2D: Eigen cross-validation disabled (PHYSICS2D_USE_EIGEN=OFF)")
endif()

# ---------------------------------------------------------------------------
# GLFW + Dear ImGui (testbed only)
# ---------------------------------------------------------------------------
if(PHYSICS2D_BUILD_TESTBED)
    find_package(OpenGL REQUIRED)

    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    # GLFW 3.4 defaults to also building its Wayland backend on Linux, which needs
    # wayland-scanner (libwayland-dev/wayland-protocols). The testbed targets the
    # X11 backend on Linux (xorg-dev in CI/dev-setup), so force the choice to keep
    # the dependency list minimal and the configure step deterministic across
    # machines that do not have the Wayland tooling installed.
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
        URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(glfw)

    # Dear ImGui has no CMake project of its own: populate sources, then build a
    # small static target here. The dockng branch commit is pinned for
    # reproducibility (a moving branch would silently change UI behaviour).
    FetchContent_Declare(imgui
        URL https://github.com/ocornut/imgui/archive/d88f238d1d363ef387382b2e29ad12cf96f826ac.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR _p2_no_cmake_subdir)
    FetchContent_MakeAvailable(imgui)

    add_library(imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/imgui_demo.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp")
    target_include_directories(imgui
        PUBLIC "${imgui_SOURCE_DIR}" "${imgui_SOURCE_DIR}/backends")
    target_compile_features(imgui PUBLIC cxx_std_17)
    target_link_libraries(imgui PUBLIC glfw PRIVATE OpenGL::GL)

    # Third-party sources: keep MSVC quiet so /W4 everywhere else stays useful.
    if(MSVC)
        target_compile_options(imgui PRIVATE /W0)
    else()
        target_compile_options(imgui PRIVATE -w)
    endif()
endif()

# ---------------------------------------------------------------------------
# Tracy (reserved, Week 6+): not auto-fetched. Enabling it requires a local
# Tracy checkout so that the dependency decision stays explicit.
# ---------------------------------------------------------------------------
if(PHYSICS2D_ENABLE_TRACY)
    if(NOT PHYSICS2D_TRACY_ROOT)
        message(FATAL_ERROR
            "PHYSICS2D_ENABLE_TRACY=ON requires -DPHYSICS2D_TRACY_ROOT=<tracy source dir>. "
            "Tracy is scheduled for the profiling milestone (Week 6+); see docs/dev-interfaces.md.")
    endif()
    add_library(p2_tracy INTERFACE)
    target_include_directories(p2_tracy SYSTEM INTERFACE "${PHYSICS2D_TRACY_ROOT}/public")
    target_compile_definitions(p2_tracy INTERFACE TRACY_ENABLE=1)
    message(STATUS "Physics2D: Tracy hooks enabled from ${PHYSICS2D_TRACY_ROOT}")
endif()
