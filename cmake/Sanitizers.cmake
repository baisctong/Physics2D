# Sanitizer wiring (Section 2.3 / P01).
#
# ASan/UBSan are applied per target so that the test and testbed executables can
# be built with the same instrumentation while third-party static libraries stay
# untouched. Usage:
#   cmake --preset sanitize && cmake --build --preset sanitize && ctest --preset sanitize
#
# MSVC supports ASan (/fsanitize=address) but not UBSan; the UBSan option is
# ignored with a warning there, which is why CI runs the sanitizer job on Linux.

function(p2_apply_sanitizers target)
    if(PHYSICS2D_ENABLE_ASAN)
        if(MSVC)
            target_compile_options(${target} PRIVATE /fsanitize=address)
            target_link_options(${target} PRIVATE /INCREMENTAL:NO)
        else()
            target_compile_options(${target} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE -fsanitize=address)
        endif()
    endif()

    if(PHYSICS2D_ENABLE_UBSAN)
        if(MSVC)
            message(WARNING "UBSan is not supported by MSVC; PHYSICS2D_ENABLE_UBSAN ignored for '${target}'")
        else()
            target_compile_options(${target} PRIVATE -fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE -fsanitize=undefined)
        endif()
    endif()
endfunction()
