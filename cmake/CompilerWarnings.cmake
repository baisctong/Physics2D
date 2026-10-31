# Project-wide warning contract (Section 2.4 / P01).
#
# Applied per target instead of globally so third-party sources (ImGui, GLFW,
# GoogleTest, Eigen) keep their own warning policy. "Zero warnings" in the
# design document's checklist means zero warnings from *our* targets.

function(p2_apply_project_warnings target)
    if(MSVC)
        # /utf-8        : source and execution charset are UTF-8 (comments/docs)
        # /permissive-  : strict standard conformance; catches MSVC-isms early
        # /Zc:__cplusplus: make __cplusplus report C++20 instead of 199711L
        # /W4           : the highest sane warning level for hand-written code
        target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8)
        if(PHYSICS2D_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wconversion
            -Wsign-conversion
            -Wnon-virtual-dtor
            -Wold-style-cast)
        # No FP contraction (fused multiply-add): the determinism golden file is a
        # quantized weak hash and must not depend on the compiler deciding to fuse
        # a*b+c into FMA (MSVC does not contract by default either). This keeps the
        # Linux CI regression comparable to the Windows one; it costs a rounding
        # step worth of precision at most and is required for reproducible physics.
        target_compile_options(${target} PRIVATE -ffp-contract=off)
        if(PHYSICS2D_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()
