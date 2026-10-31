// p2::physics2d_regression — headless determinism harness (Week 6, P16 / Section 7.3).
//
// WHAT IT DOES
//   Builds one fixed scene with the public core API only (no testbed, no GLFW),
//   advances 600 fixed timesteps and folds the final body state into two FNV-1a
//   hashes:
//
//     hash      quantised state (kHashQuantum = 1e-5 m/rad). This is the "weak"
//               hash: values are rounded to the same bucket before hashing, so a
//               cross-platform difference below the quantum does not flip it.
//               The golden file pins this hash and --check compares it.
//     exact     raw IEEE-754 bit patterns. Informational only: FMA contraction
//               and libm differences between compilers can change the last bit,
//               so CI must not gate on it (Section 7.4).
//
//   Why only the final state is hashed: the simulation is chaotic, so hashing
//   every frame would let the quantised hash diverge on the first bucket-boundary
//   flip even when the physics is "the same". The final-state weak hash is the
//   cross-platform contract; the exact hash catches local bit-level drift.
//
// SCENE (deterministic, fixed seed; no <random> - its sequence is not portable)
//   static ground plane at y = 0
//   + 30 x 1 m boxes stacked at x = 0, y = 0.5 + i
//   + 20 circles with radius in [0.2, 0.45] at LCG-seeded positions in
//     x in [-6, 6], y in [2, 8].
//   Bodies = 51. No initial velocity, gravity (0, -10), dt = 1/60 s.
//
//   Every body is created with `allowSleep = false`: the regression scene
//   deliberately keeps exercising continuous dynamics. Island sleeping is covered
//   by tests/unit/test_world_sleep.cpp, and island grouping / the flat solve are
//   bit-identical (Solver/Island.h), so the weak hash must stay stable WITHIN a
//   toolchain: MSVC `588d30e70f9b4ddd`, GCC/libstdc++ `001d846118f05125` (the
//   first remote Linux CI run proved the hash is not portable across compilers:
//   the chaotic 30-box collapse amplifies libm/FMA last-bit differences). Each
//   toolchain has its own golden file; a same-toolchain change means the physics
//   changed - investigate, never silently update the golden.
//
// USAGE / EXIT CODES (test contract, task-5)
//   physics2d_regression                        print the hash line, exit 0
//   physics2d_regression --write-golden <path>  write the golden file, exit 0
//   physics2d_regression --check <path>         match -> 0, hash mismatch -> 1
//   bad/unknown arguments or unreadable file    exit 2
//
//   --help prints the usage text and exits 0.
//
// This tool links physics2d_core only, so CI can run it headless on every OS.
// The CMake target is registered by the Lead in tests/CMakeLists.txt.

#include "physics2d/Core/Body.h"
#include "physics2d/Core/BodyDef.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"
#include "physics2d/Math/Vec2.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

using p2::Body;
using p2::BodyDef;
using p2::BodyHandle;
using p2::BodyType;
using p2::ShapeHandle;
using p2::Vec2;
using p2::World;

constexpr int32_t kFrames = 600;
constexpr int32_t kTowerBoxes = 30;
constexpr int32_t kCircleCount = 20;
constexpr float kHashQuantum = 1.0e-5f; // 0.01 mm / 0.00057 deg
constexpr uint32_t kLcgSeed = 20261029u;
constexpr uint32_t kLcgMultiplier = 1664525u;
constexpr uint32_t kLcgIncrement = 1013904223u;
constexpr float kBoxHalfExtent = 0.5f;
constexpr const char* kSceneName = "tower30+circles20";

// Deterministic [0, 1) generator. std::mt19937/ distributions are not required to
// produce the same sequence across standard libraries, and the regression harness
// must not depend on that, so it ships its own 3-line LCG.
uint32_t NextRandom(uint32_t& state) {
    state = state * kLcgMultiplier + kLcgIncrement;
    return state;
}

float RandomUnit(uint32_t& state) {
    // Use the high 24 bits: the low bits of an LCG have short periods.
    return static_cast<float>(NextRandom(state) >> 8) * (1.0f / 16777216.0f);
}

// Abort on any rejected scene-setup call. A golden generated from a scene that
// silently lost a body or a shape would be a false baseline, so the harness fails
// hard in every preset: assert() is compiled out in release, this check is not.
[[noreturn]] void FailSceneSetup(const char* what) {
    std::fprintf(stderr, "[regression] scene setup failed: %s\n", what);
    std::abort();
}

// Consumes World::AttachShape's [[nodiscard]] result with a real validity check
// (never a cast-to-void): a rejected attach is a hard scene-setup failure.
void AttachShapeChecked(World& world, BodyHandle body, ShapeHandle shape) {
    if (!world.AttachShape(body, shape)) {
        FailSceneSetup("World::AttachShape rejected a scene shape");
    }
}

void BuildScene(World& world) {
    BodyDef groundDef;
    groundDef.type = BodyType::Static;
    groundDef.allowSleep = false;
    const BodyHandle ground = world.CreateBody(groundDef);
    AttachShapeChecked(world, ground, world.CreatePlaneShape(Vec2(0.0f, 1.0f), 0.0f));

    for (int32_t index = 0; index < kTowerBoxes; ++index) {
        BodyDef boxDef;
        boxDef.type = BodyType::Dynamic;
        boxDef.position = Vec2(0.0f, kBoxHalfExtent + static_cast<float>(index) * 2.0f * kBoxHalfExtent);
        boxDef.density = 1.0f;
        boxDef.allowSleep = false; // continuous dynamics: see the file header
        const BodyHandle box = world.CreateBody(boxDef);
        AttachShapeChecked(world, box, world.CreateBoxShape(kBoxHalfExtent, kBoxHalfExtent));
    }

    uint32_t randomState = kLcgSeed;
    for (int32_t index = 0; index < kCircleCount; ++index) {
        const float x = -6.0f + 12.0f * RandomUnit(randomState);
        const float y = 2.0f + 6.0f * RandomUnit(randomState);
        const float radius = 0.2f + 0.25f * RandomUnit(randomState);
        BodyDef circleDef;
        circleDef.type = BodyType::Dynamic;
        circleDef.position = Vec2(x, y);
        circleDef.density = 1.0f;
        circleDef.friction = 0.4f;
        circleDef.restitution = 0.1f;
        circleDef.allowSleep = false; // continuous dynamics: see the file header
        const BodyHandle circle = world.CreateBody(circleDef);
        AttachShapeChecked(world, circle, world.CreateCircleShape(radius));
    }
}

struct RegressionResult {
    uint64_t weakHash = 0;
    uint64_t exactHash = 0;
    int32_t bodyCount = 0;
    int32_t contactCount = 0;
    int32_t frames = 0;
};

RegressionResult RunScene() {
    World world;
    world.SetGravity(Vec2(0.0f, -10.0f));
    BuildScene(world);
    for (int32_t frame = 0; frame < kFrames; ++frame) {
        world.Step(p2::kFixedTimeStep);
    }

    RegressionResult result;
    result.frames = kFrames;
    uint64_t weakHash = p2::kFnv1aOffsetBasis;
    uint64_t exactHash = p2::kFnv1aOffsetBasis;
    const std::vector<Body>& bodies = world.GetBodies();
    for (const Body& body : bodies) {
        if (!body.handle.IsValid()) {
            continue;
        }
        ++result.bodyCount;
        // Hot state only: transform (position + rotation, stored as cos/sin so the
        // hash is independent of atan2 implementation), velocities, pseudo state.
        // Force accumulators are cleared every step, so they are always zero here.
        const float values[] = {body.xf.p.x,
                                body.xf.p.y,
                                body.xf.q.c,
                                body.xf.q.s,
                                body.linearVelocity.x,
                                body.linearVelocity.y,
                                body.angularVelocity,
                                body.pseudoLinearVelocity.x,
                                body.pseudoLinearVelocity.y,
                                body.pseudoAngularVelocity};
        for (const float value : values) {
            weakHash = p2::HashQuantizedFloat(weakHash, value, kHashQuantum);
            exactHash = p2::HashFloat(exactHash, value);
        }
    }
    // Fold the counts too: a lost body or contact pair is a regression even if the
    // remaining float sequence happened to hash to the same value.
    result.contactCount = world.GetContactCount();
    weakHash = p2::Fnv1aStep(weakHash, &result.bodyCount, sizeof(result.bodyCount));
    weakHash = p2::Fnv1aStep(weakHash, &result.contactCount, sizeof(result.contactCount));
    exactHash = p2::Fnv1aStep(exactHash, &result.bodyCount, sizeof(result.bodyCount));
    exactHash = p2::Fnv1aStep(exactHash, &result.contactCount, sizeof(result.contactCount));
    result.weakHash = weakHash;
    result.exactHash = exactHash;
    return result;
}

void PrintResult(const RegressionResult& result) {
    std::printf("hash=%016llx frames=%d bodies=%d contacts=%d exact=%016llx scene=%s\n",
                static_cast<unsigned long long>(result.weakHash), static_cast<int>(result.frames),
                static_cast<int>(result.bodyCount), static_cast<int>(result.contactCount),
                static_cast<unsigned long long>(result.exactHash), kSceneName);
}

void PrintUsage() {
    std::printf("usage:\n"
                "  physics2d_regression                        print the determinism hash\n"
                "  physics2d_regression --write-golden <path>  write a golden baseline file\n"
                "  physics2d_regression --check <path>         compare against a golden baseline\n"
                "exit codes: 0 ok/match, 1 hash mismatch, 2 bad arguments or unreadable file\n");
}

std::string FormatHash(uint64_t hash) {
    char buffer[17] = {};
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(hash));
    return std::string(buffer);
}

// Golden file format (plain text, one key=value per line, '#' comments):
//   # physics2d_regression golden v1
//   scene=tower30+circles20
//   frames=600
//   bodies=51
//   contacts=<n>
//   hash=<16 hex digits>       <- the quantised weak hash, the compared value
//   exact=<16 hex digits>      <- informational bit-exact hash, never compared
bool WriteGolden(const std::string& path, const RegressionResult& result) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        std::fprintf(stderr, "[regression] cannot open golden file for writing: %s\n", path.c_str());
        return false;
    }
    file << "# physics2d_regression golden v1\n";
    file << "scene=" << kSceneName << "\n";
    file << "frames=" << result.frames << "\n";
    file << "bodies=" << result.bodyCount << "\n";
    file << "contacts=" << result.contactCount << "\n";
    file << "hash=" << FormatHash(result.weakHash) << "\n";
    file << "exact=" << FormatHash(result.exactHash) << "\n";
    if (!file) {
        std::fprintf(stderr, "[regression] failed writing golden file: %s\n", path.c_str());
        return false;
    }
    return true;
}

// Returns false when the file cannot be read or contains no parsable hash= line;
// *hash receives the weak hash on success.
bool ReadGoldenHash(const std::string& path, uint64_t& hash) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::fprintf(stderr, "[regression] cannot open golden file for reading: %s\n", path.c_str());
        return false;
    }
    std::string line;
    bool found = false;
    while (std::getline(file, line)) {
        if (line.rfind("hash=", 0) != 0) {
            continue;
        }
        std::string hexText = line.substr(std::strlen("hash="));
        // Tolerate a trailing CR / whitespace: the checked-in golden may be
        // re-checked-out with CRLF by git, and an editor may add trailing spaces.
        // Rejecting those would turn a matching baseline into "bad file" (exit 2).
        while (!hexText.empty()) {
            const char last = hexText.back();
            if (last == '\r' || last == '\n' || last == ' ' || last == '\t') {
                hexText.pop_back();
            } else {
                break;
            }
        }
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(hexText.c_str(), &end, 16);
        if (hexText.empty() || end == nullptr || *end != '\0') {
            std::fprintf(stderr, "[regression] invalid hash line in golden file: %s\n", line.c_str());
            return false;
        }
        hash = static_cast<uint64_t>(parsed);
        found = true;
        break;
    }
    if (!found) {
        std::fprintf(stderr, "[regression] golden file has no hash= line: %s\n", path.c_str());
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    // Parse the mode BEFORE running the scene: invalid arguments must fail fast
    // with exit code 2 and without pretending to have a hash.
    enum class Mode { Print, WriteGolden, Check };
    Mode mode = Mode::Print;
    std::string path;
    if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
        PrintUsage();
        return 0;
    }
    if (argc == 3 && std::strcmp(argv[1], "--write-golden") == 0) {
        mode = Mode::WriteGolden;
        path = argv[2];
    } else if (argc == 3 && std::strcmp(argv[1], "--check") == 0) {
        mode = Mode::Check;
        path = argv[2];
    } else if (argc != 1) {
        std::fprintf(stderr, "[regression] bad arguments (argc = %d)\n", argc);
        PrintUsage();
        return 2;
    }

    const RegressionResult result = RunScene();
    PrintResult(result);

    switch (mode) {
    case Mode::Print:
        return 0;
    case Mode::WriteGolden:
        return WriteGolden(path, result) ? 0 : 2;
    case Mode::Check: {
        uint64_t expectedHash = 0;
        if (!ReadGoldenHash(path, expectedHash)) {
            return 2;
        }
        if (expectedHash == result.weakHash) {
            std::printf("[regression] golden OK: hash=%s (file %s)\n", FormatHash(result.weakHash).c_str(),
                        path.c_str());
            return 0;
        }
        std::fprintf(stderr, "[regression] golden MISMATCH: expected=%s actual=%s (file %s)\n",
                     FormatHash(expectedHash).c_str(), FormatHash(result.weakHash).c_str(), path.c_str());
        return 1;
    }
    }
    return 2; // unreachable; keeps the compiler happy about every path returning
}
