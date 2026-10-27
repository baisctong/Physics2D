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
# Reserved: headless determinism regression tool (Week 6-7, P16). The target
# will be added here together with tests/regression/regression_main.cpp; the
# directory already exists so the future layout is documented.
# ---------------------------------------------------------------------------
