# third_party

This directory is intentionally almost empty.

Dependencies (GoogleTest, Eigen, GLFW, Dear ImGui) are pulled by CMake
`FetchContent` at configure time into the build directory, as allowed by the
design document (Section 2.2: "GLFW / ImGui / Tracy 全部用 FetchContent 拉源码").
That keeps `git clone` + `cmake --preset` self-contained without requiring a
pre-installed vcpkg tree.

If you need to work offline, configure once while online (sources are cached in
`build/<preset>/_deps/`), or pre-populate this directory and pass
`-DFETCHCONTENT_SOURCE_DIR_<NAME>=<path>` on the configure command line.

Only `physics2d_core` is dependency-free: it links **no** third-party target.
