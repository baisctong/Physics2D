#pragma once

// Shared test support for the unit-test suite (Week 1).
//
// Death tests are valuable: they pin down the debug-assert contract for invalid
// input (Section 3.4 of the design document). GoogleTest implements them by
// re-executing the test binary, which requires the process to create a
// temporary file. Some sandboxed environments deny file writes to processes
// whose executable lives inside the workspace - in that case the death tests
// can never run in place.
//
// Rather than failing the whole suite in such an environment, death tests call
// RequireDeathTestSupport() first, which turns the test into a visible SKIP with
// an explicit reason. On a normal machine (and in CI) the probe succeeds and
// every death test runs normally.
//
// Usage:
//   class MyDeathTest : public p2test::DeathTestFixture {};
//   TEST_F(MyDeathTest, RejectsInvalidInput) { EXPECT_DEATH(...); }

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace p2test {

// True when this process can create (and delete) a temporary file. Probes the
// same operation GoogleTest's death-test machinery performs. Uses <fstream>
// instead of <cstdio> so that MSVC /W4 does not flag the C4996 "unsafe" CRT
// functions in every translation unit that includes this header.
[[nodiscard]] inline bool CanCreateTemporaryFile() {
    std::error_code errorCode;
    const std::filesystem::path tempDirectory = std::filesystem::temp_directory_path(errorCode);
    if (errorCode) {
        return false;
    }
    const std::filesystem::path probePath =
        tempDirectory /
        ("p2_death_test_probe_" + std::to_string(reinterpret_cast<std::uintptr_t>(&errorCode)));
    {
        std::ofstream probe(probePath, std::ios::binary);
        if (!probe) {
            return false;
        }
        probe << "probe";
        if (!probe) {
            return false;
        }
    }
    std::filesystem::remove(probePath, errorCode);
    return true;
}

// Base fixture for death tests: skips with a readable reason when the contract
// cannot be exercised in the current build/environment.
class DeathTestFixture : public ::testing::Test {
  protected:
    void SetUp() override {
#if defined(NDEBUG)
        // Release/RelWithDebInfo compiles assert() out, so "invalid input must
        // die" is not a property of that build. Running EXPECT_DEATH there would
        // fail by design; skip instead (CI runs the death tests in Debug).
        GTEST_SKIP() << "debug asserts are compiled out in this build (NDEBUG); death tests only apply to "
                        "Debug builds";
#else
        if (!CanCreateTemporaryFile()) {
            GTEST_SKIP() << "death tests need to create a temporary file; this sandboxed host denies file "
                            "writes to processes whose executable lives inside the workspace. Run the test "
                            "binary from outside the workspace (or in CI) to execute this test.";
        }
#endif
    }
};

} // namespace p2test
