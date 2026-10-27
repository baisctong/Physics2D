# Headless benchmark executable (Week 6, P12). Week 1 ships a placeholder with
# the final command-line shape so that the target name and wiring are already in
# place; bench_broadphase.cpp is replaced when the broad phase lands.
#
# Rule: the benchmark links physics2d_core only (no GLFW/ImGui), so it can run
# in CI without a display.

add_executable(physics2d_bench bench_broadphase.cpp)
add_executable(p2::bench ALIAS physics2d_bench)

target_link_libraries(physics2d_bench PRIVATE physics2d_core)
p2_apply_project_warnings(physics2d_bench)
p2_apply_sanitizers(physics2d_bench)
