// Debug-contract tests: invalid body construction must trip a readable assert
// (Section 3.4 of the design document: "Debug 用 assert（带可读消息）").
//
// Scope note: the general assert contracts (IDPool double release, accumulator
// configuration, World::Step dt / CheckSane) are covered by the module owners'
// death tests in test_idpool.cpp / test_accumulator.cpp / test_world_step.cpp.
// This file covers the one contract that has no other owner: Body::Initialize
// rejects non-finite state at the boundary, so that NaN never reaches the
// integrator.
//
// These tests use GoogleTest's death-test machinery, which re-executes the test
// binary and communicates through a temporary file. `p2test::DeathTestFixture`
// turns the test into a visible SKIP when the contract does not apply (release
// build with NDEBUG) or when the host sandbox forbids that write (this
// environment), so the suite stays green everywhere while CI runs the assertion
// for real in a Debug build.

#include <limits>

#include <gtest/gtest.h>

#include "TestSupport.h"

#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"

namespace {

class BodyInitializeDeathTest : public p2test::DeathTestFixture {};

} // namespace

TEST_F(BodyInitializeDeathTest, NonFinitePositionAsserts) {
    p2::Body body;
    p2::BodyDef def;
    def.type = p2::BodyType::Dynamic;
    def.position = p2::Vec2(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    EXPECT_DEATH({ body.Initialize(def); }, "non-finite position");
}

TEST_F(BodyInitializeDeathTest, NonFiniteVelocityAsserts) {
    p2::Body body;
    p2::BodyDef def;
    def.type = p2::BodyType::Dynamic;
    def.linearVelocity = p2::Vec2(0.0f, std::numeric_limits<float>::infinity());
    EXPECT_DEATH({ body.Initialize(def); }, "non-finite velocity");
}
