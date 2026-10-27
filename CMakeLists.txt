cmake_minimum_required(VERSION 3.24)

project(Physics2D
    VERSION 0.1.0
    DESCRIPTION "A hand-written 2D rigid body physics engine (learning/resume project)"
    LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------------------
# Language and defaults
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(PHYSICS2D_BUILD_TESTS      "Build unit tests (GoogleTest)"           ON)
option(PHYSICS2D_BUILD_TESTBED    "Build OpenGL + Dear ImGui testbed"        ON)
option(PHYSICS2D_BUILD_BENCH      "Build the headless benchmark executable"  ON)
option(PHYSICS2D_ENABLE_TRACY     "Enable Tracy profiler hooks"              OFF)
option(PHYSICS2D_ENABLE_ASAN      "Enable AddressSanitizer"                  OFF)
option(PHYSICS2D_ENABLE_UBSAN     "Enable UndefinedBehaviorSanitizer"        OFF)
option(PHYSICS2D_USE_EIGEN        "Use Eigen in tests for cross-validation"  ON)
option(PHYSICS2D_WARNINGS_AS_ERRORS "Treat project warnings as errors"       OFF)
option(PHYSICS2D_HEADLESS_ONLY    "Configure without any rendering target"   OFF)

if(PHYSICS2D_HEADLESS_ONLY)
    set(PHYSICS2D_BUILD_TESTBED OFF)
endif()

# Reserved (Week 6+): path to a Tracy source checkout. Kept as an explicit
# cache variable because the profiler is not needed for Week 1 and the design
# document schedules it for the profiling milestone.
set(PHYSICS2D_TRACY_ROOT "" CACHE PATH "Path to Tracy source tree (reserved, Week 6)")

# ---------------------------------------------------------------------------
# Sanitizer + debug flags
# ---------------------------------------------------------------------------
# MSVC's default Debug flags contain /RTC1, which is incompatible with
# /fsanitize=address. Strip it only when ASan is actually requested, so normal
# Debug builds keep runtime checks.
if(MSVC AND PHYSICS2D_ENABLE_ASAN)
    string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    # /fsanitize=address makes MSVC define _DISABLE_VECTOR_ANNOTATION /
    # _DISABLE_STRING_ANNOTATION for the TUs it compiles. Third-party targets
    # (GoogleTest) are compiled without ASan, so they would keep the STL
    # annotations enabled and the linker reports LNK2038 annotate_vector /
    # annotate_string mismatches. Defining both macros globally makes every TU
    # agree regardless of instrumentation.
    add_compile_definitions(_DISABLE_VECTOR_ANNOTATION _DISABLE_STRING_ANNOTATION)
endif()
if(PHYSICS2D_ENABLE_ASAN OR PHYSICS2D_ENABLE_UBSAN)
    add_compile_definitions(PHYSICS2D_SANITIZERS=1)
endif()

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Dependencies.cmake)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
add_subdirectory(src)

if(PHYSICS2D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(PHYSICS2D_BUILD_BENCH)
    add_subdirectory(bench)
endif()

if(PHYSICS2D_BUILD_TESTBED)
    add_subdirectory(testbed)
endif()

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "Physics2D ${PROJECT_VERSION} configuration")
message(STATUS "  build type      : ${CMAKE_BUILD_TYPE}${CMAKE_CONFIGURATION_TYPES}")
message(STATUS "  compiler        : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  tests           : ${PHYSICS2D_BUILD_TESTS}")
message(STATUS "  testbed         : ${PHYSICS2D_BUILD_TESTBED}")
message(STATUS "  bench           : ${PHYSICS2D_BUILD_BENCH}")
message(STATUS "  eigen (tests)   : ${PHYSICS2D_USE_EIGEN}")
message(STATUS "  ASan / UBSan    : ${PHYSICS2D_ENABLE_ASAN} / ${PHYSICS2D_ENABLE_UBSAN}")
message(STATUS "  warnings-as-err : ${PHYSICS2D_WARNINGS_AS_ERRORS}")
message(STATUS "")
