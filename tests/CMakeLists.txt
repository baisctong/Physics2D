# Unit test executable (Section 7.1 - L1 + L2 tests).
#
# Sources are globbed with CONFIGURE_DEPENDS so that adding a test file never
# requires touching this file (the module owners add tests independently).
# Every test file becomes an individual ctest entry via gtest_discover_tests.

file(GLOB P2_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp")

add_executable(physics2d_tests ${P2_TEST_SOURCES})
add_executable(p2::tests ALIAS physics2d_tests)

target_link_libraries(physics2d_tests PRIVATE
    physics2d_core
    GTest::gtest
    GTest::gtest_main)

target_include_directories(physics2d_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

p2_apply_project_warnings(physics2d_tests)
p2_apply_sanitizers(physics2d_tests)

if(PHYSICS2D_USE_EIGEN)
    target_link_libraries(physics2d_tests PRIVATE p2::eigen)
    target_compile_definitions(physics2d_tests PRIVATE PHYSICS2D_USE_EIGEN=1)
endif()

include(GoogleTest)
gtest_discover_tests(physics2d_tests
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "unit")

# ---------------------------------------------------------------------------
# Determinism regression tool (Week 6, P16). It replays a fixed scene headlessly
# and compares a QUANTISED state hash against a committed golden file; only core
# is linked, so the CI headless job can run it without a renderer.
#
# The weak hash is stable WITHIN one compiler/standard-library toolchain but not
# ACROSS toolchains: the first remote Linux CI run measured MSVC
# 588d30e70f9b4ddd vs GCC 001d846118f05125 (same intents, different libm/FMA last
# bits, amplified by the chaotic 30-box collapse). Keep one golden per toolchain
# family and let the shared regression_default entry gate the matching file; see
# tests/regression/README.md for the policy.
# ---------------------------------------------------------------------------
if(MSVC)
    set(P2_REGRESSION_GOLDEN "default_600.txt")
else()
    set(P2_REGRESSION_GOLDEN "default_600_gcc.txt")
endif()

add_executable(physics2d_regression regression/regression_main.cpp)
add_executable(p2::regression ALIAS physics2d_regression)
target_link_libraries(physics2d_regression PRIVATE physics2d_core)
p2_apply_project_warnings(physics2d_regression)
p2_apply_sanitizers(physics2d_regression)

# Plain CTest entry (not a gtest case): the tool has its own CLI and exit codes.
add_test(NAME regression_default
         COMMAND physics2d_regression --check
                 ${CMAKE_CURRENT_SOURCE_DIR}/regression/golden/${P2_REGRESSION_GOLDEN})
set_tests_properties(regression_default PROPERTIES LABELS "regression")
