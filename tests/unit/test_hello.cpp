// Week 1 D1 smoke test: the build/CTest pipeline is alive.
// Replaced/extended by the module test suites; kept as the "hello world" test
// that proves a fresh clone can configure, build and run something.
#include <gtest/gtest.h>

#include "physics2d/Core/Settings.h"
#include "physics2d/Math/Vec2.h"

TEST(BuildPipeline, CoreHeaderIsUsable) {
    const p2::Vec2 v{3.0f, 4.0f};
    EXPECT_FLOAT_EQ(p2::Length(v), 5.0f);
    EXPECT_FLOAT_EQ(p2::kFixedTimeStep, 1.0f / 60.0f);
}
