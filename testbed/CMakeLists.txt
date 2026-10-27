# physics2d_testbed - Layer 2 executable (Section 1.2).
#
# May include GLFW / OpenGL / Dear ImGui. Must never be a dependency of
# physics2d_core (the arrow only points outwards).
#
# Sources are globbed with CONFIGURE_DEPENDS because demos are added week after
# week by different contributors.

find_package(OpenGL REQUIRED)

file(GLOB P2_TESTBED_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/demos/*.cpp")

add_executable(physics2d_testbed ${P2_TESTBED_SOURCES})
add_executable(p2::testbed ALIAS physics2d_testbed)

target_link_libraries(physics2d_testbed PRIVATE
    physics2d_core
    glfw
    imgui
    OpenGL::GL)

target_include_directories(physics2d_testbed PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

p2_apply_project_warnings(physics2d_testbed)
p2_apply_sanitizers(physics2d_testbed)

# Keep a console window on Windows: the testbed prints the smoke-test summary
# and GL errors to stdout, which is far more useful during development than a
# silent GUI subsystem binary (see docs/dev-setup.md).
if(WIN32)
    target_compile_definitions(physics2d_testbed PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
