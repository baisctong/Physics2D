// Week 6 P12 — headless broad-phase benchmark (physics2d_bench).
//
// What is measured: the two IBroadPhase implementations on an identical,
// deterministic scene of N boxes moving through a 30x30 m world. Physics is NOT
// stepped - each frame only moves the boxes, refreshes the proxies with the moved
// (fat) AABBs and enumerates the candidate pairs. This isolates the broad phase,
// which is what the README/devlog tables quote.
//
// Timing discipline (Section 8 / D04):
//  - Release build required for quotable numbers (the build type is printed);
//  - std::chrono::steady_clock, per-frame samples; median AND mean reported;
//  - the first 100 frames are warm-up and discarded (SAP's initial std::sort and
//    the CPU cache are cold there);
//  - per frame, body motion and the AABB arithmetic happen OUTSIDE the timed
//    region, so only UpdateAABB + QueryPairs are measured;
//  - the callback just counts candidate pairs and exact tight-AABB overlaps. That
//    counting is part of the measured loop for both implementations (an
//    application does work per pair in the real callback too), so the comparison
//    stays apples-to-apples.
//
// Determinism: both implementations are driven from the same RNG seed, so they see
// the same bodies and the same motion path frame by frame.
//
// Usage: ./physics2d_bench [--broadphase brute|sap|all] [--bodies 50,200,500,2000]
//                          [--frames 1000] [--seed 42] [--json]

#include "physics2d/Collision/BroadPhaseBruteForce.h"
#include "physics2d/Collision/BroadPhaseSAP.h"
#include "physics2d/Core/BroadPhaseInterface.h"
#include "physics2d/Core/Settings.h"
#include "physics2d/Math/AABB.h"
#include "physics2d/Math/Vec2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <utility>
#include <vector>

namespace {

constexpr int32_t kWarmupFrames = 100;
constexpr int32_t kMaxBodyCount = 100000;
constexpr float kAreaSize = 30.0f;
constexpr float kMinHalfExtent = 0.1f;
constexpr float kMaxHalfExtent = 0.5f;
constexpr float kMaxSpeed = 2.0f;
constexpr float kTimeStep = 1.0f / 60.0f;
constexpr uint32_t kDefaultSeed = 42u;

enum class Target { All, BruteForce, Sap };

struct Options {
    Target target = Target::All;
    std::vector<int32_t> bodyCounts{50, 200, 500, 2000};
    int32_t frames = 1000;
    uint32_t seed = kDefaultSeed;
    bool json = false;
};

struct BenchmarkBody {
    p2::Vec2 center{0.0f, 0.0f};
    p2::Vec2 velocity{0.0f, 0.0f};
    p2::Vec2 halfExtent{0.1f, 0.1f};
};

struct RunResult {
    double medianMs = 0.0;
    double meanMs = 0.0;
    double candidatesPerFrame = 0.0;
    double trueOverlapsPerFrame = 0.0;
    int32_t proxyCount = 0;
    int32_t measuredFrames = 0;
};

void PrintUsage() {
    std::printf("usage: physics2d_bench [--broadphase brute|sap|all] [--bodies 50,200,500,2000]\n");
    std::printf("                         [--frames 1000] [--seed 42] [--json]\n");
    std::printf("  default: --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed %u\n",
                static_cast<unsigned>(kDefaultSeed));
}

[[nodiscard]] bool ParseInt32(const char* text, int32_t minimum, int32_t maximum, int32_t& value) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
        return false;
    }
    value = static_cast<int32_t>(parsed);
    return true;
}

[[nodiscard]] bool ParseUint32(const char* text, uint32_t& value) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed > 0xFFFFFFFFul) {
        return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
}

// Comma-separated body counts, e.g. "50,200,500,2000".
[[nodiscard]] bool ParseBodyList(const char* text, std::vector<int32_t>& bodyCounts) {
    bodyCounts.clear();
    const char* cursor = text;
    while (cursor != nullptr && *cursor != '\0') {
        char* end = nullptr;
        const long parsed = std::strtol(cursor, &end, 10);
        if (end == cursor || parsed < 1 || parsed > kMaxBodyCount) {
            return false;
        }
        bodyCounts.push_back(static_cast<int32_t>(parsed));
        cursor = end;
        if (*cursor == ',') {
            ++cursor;
            if (*cursor == '\0') {
                return false; // trailing comma
            }
        } else if (*cursor != '\0') {
            return false; // garbage between numbers
        }
    }
    return !bodyCounts.empty();
}

[[nodiscard]] bool ParseArguments(int argc, char** argv, Options& options) {
    for (int argument = 1; argument < argc; ++argument) {
        const char* name = argv[argument];
        const auto nextValue = [&]() -> const char* {
            return (argument + 1 < argc) ? argv[argument + 1] : nullptr;
        };
        if (std::strcmp(name, "--help") == 0 || std::strcmp(name, "-h") == 0) {
            return false;
        }
        if (std::strcmp(name, "--json") == 0) {
            options.json = true;
            continue;
        }
        if (std::strcmp(name, "--broadphase") == 0) {
            const char* value = nextValue();
            if (value == nullptr) {
                return false;
            }
            if (std::strcmp(value, "all") == 0) {
                options.target = Target::All;
            } else if (std::strcmp(value, "brute") == 0) {
                options.target = Target::BruteForce;
            } else if (std::strcmp(value, "sap") == 0) {
                options.target = Target::Sap;
            } else {
                return false;
            }
            ++argument;
            continue;
        }
        if (std::strcmp(name, "--bodies") == 0) {
            const char* value = nextValue();
            if (value == nullptr || !ParseBodyList(value, options.bodyCounts)) {
                return false;
            }
            ++argument;
            continue;
        }
        if (std::strcmp(name, "--frames") == 0) {
            const char* value = nextValue();
            if (value == nullptr || !ParseInt32(value, 2, 1000000, options.frames)) {
                return false;
            }
            ++argument;
            continue;
        }
        if (std::strcmp(name, "--seed") == 0) {
            const char* value = nextValue();
            if (value == nullptr || !ParseUint32(value, options.seed)) {
                return false;
            }
            ++argument;
            continue;
        }
        return false; // unknown flag
    }
    return true;
}

[[nodiscard]] std::vector<BenchmarkBody> MakeScene(int32_t bodyCount, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> position(0.0f, kAreaSize);
    std::uniform_real_distribution<float> halfExtent(kMinHalfExtent, kMaxHalfExtent);
    std::uniform_real_distribution<float> speed(-kMaxSpeed, kMaxSpeed);
    std::vector<BenchmarkBody> bodies;
    bodies.reserve(static_cast<size_t>(bodyCount));
    for (int32_t index = 0; index < bodyCount; ++index) {
        BenchmarkBody body;
        body.center = p2::Vec2{position(rng), position(rng)};
        body.velocity = p2::Vec2{speed(rng), speed(rng)};
        body.halfExtent = p2::Vec2{halfExtent(rng), halfExtent(rng)};
        bodies.push_back(body);
    }
    return bodies;
}

// Simple elastic bounce inside the world box: keeps the boxes distributed over the
// whole area for the entire run, so all frames have the same expected density.
void AdvanceScene(std::vector<BenchmarkBody>& bodies) {
    for (BenchmarkBody& body : bodies) {
        body.center += body.velocity * kTimeStep;
        if (body.center.x - body.halfExtent.x < 0.0f) {
            body.center.x = body.halfExtent.x;
            body.velocity.x = std::fabs(body.velocity.x);
        } else if (body.center.x + body.halfExtent.x > kAreaSize) {
            body.center.x = kAreaSize - body.halfExtent.x;
            body.velocity.x = -std::fabs(body.velocity.x);
        }
        if (body.center.y - body.halfExtent.y < 0.0f) {
            body.center.y = body.halfExtent.y;
            body.velocity.y = std::fabs(body.velocity.y);
        } else if (body.center.y + body.halfExtent.y > kAreaSize) {
            body.center.y = kAreaSize - body.halfExtent.y;
            body.velocity.y = -std::fabs(body.velocity.y);
        }
    }
}

[[nodiscard]] RunResult RunBenchmark(p2::IBroadPhase& broadPhase, int32_t bodyCount, uint32_t seed,
                                     int32_t frameCount) {
    std::vector<BenchmarkBody> bodies = MakeScene(bodyCount, seed);
    std::vector<int32_t> proxyIds;
    proxyIds.reserve(static_cast<size_t>(bodyCount));
    std::vector<p2::AABB> tightAabbs(static_cast<size_t>(bodyCount));
    std::vector<p2::AABB> fatAabbs(static_cast<size_t>(bodyCount));

    for (int32_t index = 0; index < bodyCount; ++index) {
        const BenchmarkBody& body = bodies[static_cast<size_t>(index)];
        const p2::AABB tight{body.center - body.halfExtent, body.center + body.halfExtent};
        p2::AABB fat = tight;
        fat.Extend(p2::kAabbMargin);
        tightAabbs[static_cast<size_t>(index)] = tight;
        fatAabbs[static_cast<size_t>(index)] = fat;
        proxyIds.push_back(broadPhase.AddProxy(fat));
    }

    const int32_t warmupFrames = (frameCount > kWarmupFrames) ? kWarmupFrames : (frameCount - 1);
    const int32_t measuredFrames = frameCount - warmupFrames;
    std::vector<double> samplesPerFrame;
    samplesPerFrame.reserve(static_cast<size_t>(measuredFrames));
    int64_t totalCandidates = 0;
    int64_t totalTrueOverlaps = 0;

    for (int32_t frame = 0; frame < frameCount; ++frame) {
        AdvanceScene(bodies);
        for (int32_t index = 0; index < bodyCount; ++index) {
            const BenchmarkBody& body = bodies[static_cast<size_t>(index)];
            const p2::AABB tight{body.center - body.halfExtent, body.center + body.halfExtent};
            p2::AABB fat = tight;
            fat.Extend(p2::kAabbMargin);
            tightAabbs[static_cast<size_t>(index)] = tight;
            fatAabbs[static_cast<size_t>(index)] = fat;
        }

        int64_t candidates = 0;
        int64_t trueOverlaps = 0;
        const auto start = std::chrono::steady_clock::now();
        for (int32_t index = 0; index < bodyCount; ++index) {
            broadPhase.UpdateAABB(proxyIds[static_cast<size_t>(index)], fatAabbs[static_cast<size_t>(index)]);
        }
        broadPhase.QueryPairs([&candidates, &trueOverlaps, &tightAabbs](int32_t proxyIdA, int32_t proxyIdB) {
            ++candidates;
            if (p2::Overlaps(tightAabbs[static_cast<size_t>(proxyIdA)],
                             tightAabbs[static_cast<size_t>(proxyIdB)])) {
                ++trueOverlaps;
            }
        });
        const auto end = std::chrono::steady_clock::now();

        if (frame >= warmupFrames) {
            samplesPerFrame.push_back(std::chrono::duration<double, std::milli>(end - start).count());
            totalCandidates += candidates;
            totalTrueOverlaps += trueOverlaps;
        }
    }

    RunResult result;
    result.proxyCount = broadPhase.GetProxyCount();
    result.measuredFrames = measuredFrames;
    if (!samplesPerFrame.empty()) {
        std::sort(samplesPerFrame.begin(), samplesPerFrame.end());
        result.medianMs = samplesPerFrame[samplesPerFrame.size() / 2];
        double sum = 0.0;
        for (const double sample : samplesPerFrame) {
            sum += sample;
        }
        result.meanMs = sum / static_cast<double>(samplesPerFrame.size());
        result.candidatesPerFrame =
            static_cast<double>(totalCandidates) / static_cast<double>(measuredFrames);
        result.trueOverlapsPerFrame =
            static_cast<double>(totalTrueOverlaps) / static_cast<double>(measuredFrames);
    }
    return result;
}

const char* BuildTypeName() {
#ifdef NDEBUG
    return "Release (NDEBUG)";
#else
    return "Debug (asserts enabled; not quotable)";
#endif
}

void FormatCompiler(char* buffer, size_t bufferSize) {
#if defined(_MSC_VER)
    std::snprintf(buffer, bufferSize, "MSVC %d.%d", static_cast<int>(_MSC_VER / 100),
                  static_cast<int>(_MSC_VER % 100));
#elif defined(__clang__)
    std::snprintf(buffer, bufferSize, "clang %d.%d.%d", __clang_major__, __clang_minor__,
                  __clang_patchlevel__);
#elif defined(__GNUC__)
    std::snprintf(buffer, bufferSize, "gcc %d.%d.%d", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
    std::snprintf(buffer, bufferSize, "unknown");
#endif
}

const char* CpuName() {
#ifdef _WIN32
#if defined(_MSC_VER)
    // _dupenv_s instead of getenv: MSVC /W4 flags getenv as unsafe (C4996) and the
    // project builds warning-free. The value is copied into a static buffer so the
    // returned pointer stays valid for both the text and the JSON report.
    char* buffer = nullptr;
    size_t bufferSize = 0;
    if (_dupenv_s(&buffer, &bufferSize, "PROCESSOR_IDENTIFIER") == 0 && buffer != nullptr) {
        static char cachedCpuName[256];
        std::snprintf(cachedCpuName, sizeof(cachedCpuName), "%s", buffer);
        std::free(buffer);
        return cachedCpuName;
    }
    return "unknown";
#else
    const char* identifier = std::getenv("PROCESSOR_IDENTIFIER");
    return (identifier != nullptr) ? identifier : "unknown";
#endif
#else
    return "unknown";
#endif
}

void PrintJsonEscaped(const char* text) {
    std::putchar('"');
    for (const char* cursor = text; cursor != nullptr && *cursor != '\0'; ++cursor) {
        const unsigned char character = static_cast<unsigned char>(*cursor);
        if (character == '"' || character == '\\') {
            std::putchar('\\');
            std::putchar(static_cast<char>(character));
        } else if (character < 0x20u) {
            std::printf("\\u%04x", static_cast<unsigned>(character));
        } else {
            std::putchar(static_cast<char>(character));
        }
    }
    std::putchar('"');
}

struct Row {
    int32_t bodyCount = 0;
    bool hasBruteForce = false;
    bool hasSap = false;
    RunResult bruteForce;
    RunResult sap;
};

void PrintMarkdownTable(const Options& options, const std::vector<Row>& rows) {
    char compiler[64];
    FormatCompiler(compiler, sizeof(compiler));
    std::printf("## Bench environment\n\n");
    std::printf("| item | value |\n| --- | --- |\n");
    std::printf("| CPU | %s |\n", CpuName());
    std::printf("| Compiler | %s |\n", compiler);
    std::printf("| Build type | %s |\n", BuildTypeName());
    std::printf(
        "| World | %.0f x %.0f m, random boxes with half extents %.1f-%.1f m, bouncing at %.1f m/s |\n",
        static_cast<double>(kAreaSize), static_cast<double>(kAreaSize), static_cast<double>(kMinHalfExtent),
        static_cast<double>(kMaxHalfExtent), static_cast<double>(kMaxSpeed));
    std::printf("| Proxies | fat AABBs: tight + %.3f m margin (kAabbMargin), same as World::Synchronize |\n",
                static_cast<double>(p2::kAabbMargin));
    std::printf("| Frames | %d measured per body count, first %d discarded as warm-up |\n", options.frames,
                kWarmupFrames);
    std::printf("| Seed | %u |\n", static_cast<unsigned>(options.seed));
    std::printf("| Clock | std::chrono::steady_clock, per-frame samples, median and mean |\n\n");

    std::printf("## Broad-phase timing\n\n");
    std::printf(
        "| Bodies | BruteForce ms/frame (median) | BruteForce mean | SAP ms/frame (median) | SAP mean | "
        "Speedup (Brute/SAP) | Candidates/frame | True overlaps/frame | Proxies |\n");
    std::printf("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
    for (const Row& row : rows) {
        std::printf("| %d ", row.bodyCount);
        if (row.hasBruteForce) {
            std::printf("| %.4f | %.4f ", row.bruteForce.medianMs, row.bruteForce.meanMs);
        } else {
            std::printf("| - | - ");
        }
        if (row.hasSap) {
            std::printf("| %.4f | %.4f ", row.sap.medianMs, row.sap.meanMs);
        } else {
            std::printf("| - | - ");
        }
        if (row.hasBruteForce && row.hasSap && row.sap.medianMs > 0.0) {
            std::printf("| %.2fx ", row.bruteForce.medianMs / row.sap.medianMs);
        } else {
            std::printf("| - ");
        }
        const RunResult& reference = row.hasSap ? row.sap : row.bruteForce;
        std::printf("| %.1f | %.1f | %d |\n", reference.candidatesPerFrame, reference.trueOverlapsPerFrame,
                    reference.proxyCount);
    }
    std::printf(
        "\nNotes: \"Candidates/frame\" is what QueryPairs emitted; \"True overlaps/frame\" counts the pairs "
        "whose TIGHT AABBs (without the fat margin) also overlap, so the gap is the false-positive cost the "
        "fat margin creates for the narrow phase. Physics is not simulated - the timed region is only "
        "UpdateAABB + QueryPairs.\n");
}

void PrintJson(const Options& options, const std::vector<Row>& rows) {
    char compiler[64];
    FormatCompiler(compiler, sizeof(compiler));
    std::printf("{\n");
    std::printf("  \"environment\": {\n");
    std::printf("    \"cpu\": ");
    PrintJsonEscaped(CpuName());
    std::printf(",\n    \"compiler\": ");
    PrintJsonEscaped(compiler);
    std::printf(",\n    \"buildType\": ");
    PrintJsonEscaped(BuildTypeName());
    std::printf(",\n    \"frames\": %d,\n    \"warmupFrames\": %d,\n    \"seed\": %u,\n", options.frames,
                kWarmupFrames, static_cast<unsigned>(options.seed));
    std::printf("    \"worldSize\": %.3f,\n    \"fatMargin\": %.3f,\n", static_cast<double>(kAreaSize),
                static_cast<double>(p2::kAabbMargin));
    std::printf("    \"padding\": \"candidatesPerFrame counts QueryPairs output; trueOverlapsPerFrame counts "
                "the subset whose tight AABBs overlap\"\n");
    std::printf("  },\n  \"rows\": [\n");
    for (size_t index = 0; index < rows.size(); ++index) {
        const Row& row = rows[index];
        std::printf("    {\"bodies\": %d", row.bodyCount);
        if (row.hasBruteForce) {
            std::printf(", \"bruteForce\": {\"medianMsPerFrame\": %.6f, \"meanMsPerFrame\": %.6f, "
                        "\"candidatesPerFrame\": %.3f, \"trueOverlapsPerFrame\": %.3f, \"proxies\": %d}",
                        row.bruteForce.medianMs, row.bruteForce.meanMs, row.bruteForce.candidatesPerFrame,
                        row.bruteForce.trueOverlapsPerFrame, row.bruteForce.proxyCount);
        }
        if (row.hasSap) {
            std::printf(", \"sap\": {\"medianMsPerFrame\": %.6f, \"meanMsPerFrame\": %.6f, "
                        "\"candidatesPerFrame\": %.3f, \"trueOverlapsPerFrame\": %.3f, \"proxies\": %d}",
                        row.sap.medianMs, row.sap.meanMs, row.sap.candidatesPerFrame,
                        row.sap.trueOverlapsPerFrame, row.sap.proxyCount);
        }
        std::printf("}%s\n", (index + 1 < rows.size()) ? "," : "");
    }
    std::printf("  ]\n}\n");
}

} // namespace

int main(int argc, char** argv) {
    for (int argument = 1; argument < argc; ++argument) {
        if (std::strcmp(argv[argument], "--help") == 0 || std::strcmp(argv[argument], "-h") == 0) {
            PrintUsage();
            return 0;
        }
    }

    Options options;
    if (!ParseArguments(argc, argv, options)) {
        PrintUsage();
        return 2;
    }

    std::vector<Row> rows;
    rows.reserve(options.bodyCounts.size());
    for (const int32_t bodyCount : options.bodyCounts) {
        Row row;
        row.bodyCount = bodyCount;
        if (options.target != Target::Sap) {
            p2::BroadPhaseBruteForce bruteForce;
            row.bruteForce = RunBenchmark(bruteForce, bodyCount, options.seed, options.frames);
            row.hasBruteForce = true;
        }
        if (options.target != Target::BruteForce) {
            p2::BroadPhaseSAP sap;
            row.sap = RunBenchmark(sap, bodyCount, options.seed, options.frames);
            row.hasSap = true;
        }
        rows.push_back(row);
    }

    if (options.json) {
        PrintJson(options, rows);
    } else {
        PrintMarkdownTable(options, rows);
    }
    return 0;
}
