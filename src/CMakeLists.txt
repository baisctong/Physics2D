# physics2d_core: the simulated world, ZERO third-party dependencies.
#
# Layer 1 of Section 1.2: only the C++ standard library. If a rendering / UI /
# profiler include ever appears below this directory, the CI layering job fails.
# New source files are listed explicitly (not globbed) on purpose: the core is
# the part of the project that has to stay auditable.

add_library(physics2d_core STATIC
    Math/Mat2.cpp
    Math/Rot.cpp
    Math/AABB.cpp
    Math/MathUtils.cpp
    Core/IDPool.cpp
    Core/Body.cpp
    Core/World.cpp
    Core/TimeStep.cpp
    Core/Shape.cpp
    Collision/CircleShape.cpp
    Collision/PlaneShape.cpp
    Collision/PolygonShape.cpp
    Collision/Clipping.cpp
    Collision/SAT.cpp
    Collision/Analytic.cpp
    Solver/ContactSolver.cpp
    Solver/ContactSolverPosition.cpp
)

add_library(p2::core ALIAS physics2d_core)

target_include_directories(physics2d_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(physics2d_core PUBLIC cxx_std_20)

p2_apply_project_warnings(physics2d_core)
p2_apply_sanitizers(physics2d_core)

# Debug-only compile definition: enables the NaN/Inf validator and assertions.
target_compile_definitions(physics2d_core PRIVATE
    $<$<CONFIG:Debug>:PHYSICS2D_DEBUG=1>)

# Reserved (Week 6+): Tracy hooks are compiled in only when explicitly enabled.
if(PHYSICS2D_ENABLE_TRACY)
    target_link_libraries(physics2d_core PRIVATE p2_tracy)
endif()
