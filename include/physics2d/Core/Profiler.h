#pragma once

// p2::Profiler — zero-cost profiling hooks (Section 8.1 of the design document).
//
// The design doc asks for Tracy zones in exactly 8 places (one per Step phase)
// plus a few plots. Week 1 cannot enable Tracy yet, so this header defines the
// call sites with no-op implementations while TRACY_ENABLE is off. Wiring the
// real profiler later is then a CMake-only change: define TRACY_ENABLE, add the
// Tracy include directory, and link the client target. No engine source changes.
//
// The macros expand to statements (not expressions) so that they can be used
// inside an `if` without braces without triggering -Wdangling-else.

#if defined(TRACY_ENABLE)
#include <tracy/Tracy.hpp>

#define P2_PROFILE_ZONE() ZoneScoped
#define P2_PROFILE_ZONE_N(name) ZoneScopedN(name)
#define P2_PROFILE_FRAME_MARK() FrameMark
#define P2_PROFILE_PLOT(name, value) TracyPlot(name, static_cast<double>(value))
#define P2_PROFILE_MESSAGE(text) TracyMessage(text, std::strlen(text))
#else
#define P2_PROFILE_ZONE() ((void)0)
#define P2_PROFILE_ZONE_N(name) ((void)0)
#define P2_PROFILE_FRAME_MARK() ((void)0)
#define P2_PROFILE_PLOT(name, value) ((void)0)
#define P2_PROFILE_MESSAGE(text) ((void)0)
#endif
