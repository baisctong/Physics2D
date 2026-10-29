#pragma once

// testbed/SceneBuilders.h - shared scene-construction helpers for the testbed demos
// (Week 6, R14 "Demo 扩展").
//
// Header-only and deliberately testbed-local:
//  - it only uses the public physics2d_core API (World::CreateBody / Create*Shape /
//    AttachShape and the material/velocity setters), so every demo builds the same
//    scene with the same code;
//  - tests/regression is headless and links physics2d_core ONLY: it must not include
//    this Layer-2 header. It gets its own scene code / a core-side scene module when
//    the Week 6/7 regression task needs it - this header is a testbed convenience,
//    not a layering exception.
//
// Determinism: the random field uses std::mt19937 seeded by the caller. The same seed
// produces the same field on the same standard-library implementation (that is what
// makes the smoke numbers reproducible); cross-platform bit-identical scenes are the
// headless regression tool's job, not this header's.
//
// All builders use the engine defaults for friction (0.3) and restitution (0), and
// density 1 kg/m^2, so a 1x1 m box weighs 1 kg. The DebugUI "material defaults" are
// applied by main.cpp after the scene is built (Week 5 experiment hook).

#include "physics2d/Core/World.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace p2 {

// Testbed-owned scene constants. Physics constants (slop, iterations, margin) stay in
// Core/Settings.h; these are scene-definition values that only the testbed uses.
inline constexpr float kSceneBoxHalfExtent = 0.5f;       // 1 m x 1 m box
inline constexpr float kSceneBoxDensity = 1.0f;          // kg/m^2 -> a 1x1 m box is 1 kg
inline constexpr float kSceneFriction = 0.3f;            // mirrors BodyDef::friction
inline constexpr float kSceneMinRandomHalfExtent = 0.1f; // 0.2 m: smallest random box
inline constexpr float kSceneMaxRandomSpin = 3.0f;       // rad/s, random initial spin

// One dynamic box body with independent half extents (Week 7: dominoes are thin and
// tall, so they cannot use the square overload). Returns an invalid handle if the World
// rejects the body/shape (the builders then skip it and keep going; a partial scene is
// better than a silent crash for a debugging tool).
[[nodiscard]] inline BodyHandle CreateBoxBody(World& world, Vec2 position, float angleRadians,
                                              Vec2 halfExtents, const std::string& debugName) {
    if (!(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f)) {
        std::fprintf(stderr, "[SceneBuilders] invalid box half extents (%.4f, %.4f) for '%s'\n",
                     static_cast<double>(halfExtents.x), static_cast<double>(halfExtents.y),
                     debugName.c_str());
        return BodyHandle{};
    }

    BodyDef bodyDef;
    bodyDef.type = BodyType::Dynamic;
    bodyDef.position = position;
    bodyDef.angle = angleRadians;
    bodyDef.density = kSceneBoxDensity;
    bodyDef.friction = kSceneFriction;
    bodyDef.restitution = 0.0f;
    bodyDef.debugName = debugName;

    const BodyHandle handle = world.CreateBody(bodyDef);
    if (!world.IsValid(handle)) {
        std::fprintf(stderr, "[SceneBuilders] World::CreateBody rejected '%s'\n", debugName.c_str());
        return BodyHandle{};
    }
    const ShapeHandle boxShape = world.CreateBoxShape(halfExtents.x, halfExtents.y);
    if (!world.IsValidShape(boxShape) || !world.AttachShape(handle, boxShape)) {
        std::fprintf(stderr, "[SceneBuilders] failed to create/attach the box shape for '%s'\n",
                     debugName.c_str());
        world.DestroyBody(handle);
        return BodyHandle{};
    }
    return handle;
}

// Square-box convenience overload.
[[nodiscard]] inline BodyHandle CreateBoxBody(World& world, Vec2 position, float angleRadians,
                                              float halfExtent, const std::string& debugName) {
    return CreateBoxBody(world, position, angleRadians, Vec2{halfExtent, halfExtent}, debugName);
}

// Infinite static ground plane through y = `groundY` (the half-space below it is
// solid). Returns the static body; the plane itself is an infinite shape, so callers
// that frame a camera must skip it when computing scene bounds.
[[nodiscard]] inline BodyHandle CreateGroundPlane(World& world, float groundY = 0.0f) {
    BodyDef bodyDef;
    bodyDef.type = BodyType::Static;
    bodyDef.position = Vec2{0.0f, groundY};
    bodyDef.debugName = "ground";

    const BodyHandle ground = world.CreateBody(bodyDef);
    if (!world.IsValid(ground)) {
        std::fprintf(stderr, "[SceneBuilders] World::CreateBody rejected the ground body\n");
        return BodyHandle{};
    }
    const ShapeHandle planeShape = world.CreatePlaneShape(Vec2{0.0f, 1.0f}, 0.0f);
    if (!world.IsValidShape(planeShape) || !world.AttachShape(ground, planeShape)) {
        std::fprintf(stderr, "[SceneBuilders] failed to create/attach the ground plane shape\n");
    }
    return ground;
}

// A vertical stack of exactly stacked 1x1 m boxes at x = 0, y = 0.5, 1.5, ... The
// layer pitch equals the box height, so the initial state has zero penetration.
[[nodiscard]] inline std::vector<BodyHandle> CreateBoxStack(World& world, int32_t layers,
                                                            float boxHalfExtent = kSceneBoxHalfExtent) {
    std::vector<BodyHandle> bodies;
    if (layers <= 0 || !(boxHalfExtent > 0.0f)) {
        std::fprintf(stderr, "[SceneBuilders] CreateBoxStack: invalid layers/half extent (%d, %.3f)\n",
                     layers, static_cast<double>(boxHalfExtent));
        return bodies;
    }
    bodies.reserve(static_cast<std::size_t>(layers));
    const float pitch = 2.0f * boxHalfExtent;
    for (int32_t layerIndex = 0; layerIndex < layers; ++layerIndex) {
        const Vec2 position{0.0f, boxHalfExtent + pitch * static_cast<float>(layerIndex)};
        bodies.push_back(
            CreateBoxBody(world, position, 0.0f, boxHalfExtent, "stack_box_" + std::to_string(layerIndex)));
    }
    return bodies;
}

// A perfect 2D box pyramid: layer k (k = 0 is the ground layer) holds baseCount - k
// boxes, centred horizontally with one-box pitch, so total = baseCount*(baseCount+1)/2.
// E.g. baseCount 20 -> 210 boxes, 19 layers tall (1 m boxes -> 20 m tall).
[[nodiscard]] inline std::vector<BodyHandle> CreateBoxPyramid(World& world, int32_t baseCount,
                                                              float boxHalfExtent) {
    std::vector<BodyHandle> bodies;
    if (baseCount <= 0 || !(boxHalfExtent > 0.0f)) {
        std::fprintf(stderr, "[SceneBuilders] CreateBoxPyramid: invalid base/half extent (%d, %.3f)\n",
                     baseCount, static_cast<double>(boxHalfExtent));
        return bodies;
    }
    const int32_t totalCount = baseCount * (baseCount + 1) / 2;
    bodies.reserve(static_cast<std::size_t>(totalCount));
    const float pitch = 2.0f * boxHalfExtent;
    int32_t boxIndex = 0;
    for (int32_t layerIndex = 0; layerIndex < baseCount; ++layerIndex) {
        const int32_t rowCount = baseCount - layerIndex;
        const float centerY = boxHalfExtent + pitch * static_cast<float>(layerIndex);
        // Centred row: box i sits at x = (i - (rowCount-1)/2) * pitch.
        const float rowOffset = static_cast<float>(rowCount - 1) * 0.5f;
        for (int32_t boxInRow = 0; boxInRow < rowCount; ++boxInRow) {
            const Vec2 position{(static_cast<float>(boxInRow) - rowOffset) * pitch, centerY};
            bodies.push_back(CreateBoxBody(world, position, 0.0f, boxHalfExtent,
                                           "pyramid_box_" + std::to_string(boxIndex)));
            ++boxIndex;
        }
    }
    return bodies;
}

// A deterministic random field of boxes for the broad-phase/solver stress scene.
//
// Spawn volume: x in [-areaHalfExtent, +areaHalfExtent], y in [halfExtent,
// max(areaHalfExtent, one box)] - i.e. the field is dropped from above the ground and
// settles into view. Sizes are uniform in [0.2 m, 2*maxHalfExtent] full width, angles
// uniform over the full circle, optional random linear velocity (direction uniform,
// speed in [0, maxInitialSpeed]) and random spin.
//
// Overlap avoidance: placements are rejected when the rotated AABB overlaps an already
// accepted spawn AABB (a few attempts each). Without it a dense field starts deeply
// interpenetrated and the first frames are a position-correction explosion instead of a
// broad-phase measurement; if the area is too small for `count`, the last attempt is
// kept anyway so the scene still builds.
[[nodiscard]] inline std::vector<BodyHandle> CreateRandomBoxField(World& world, int32_t count, uint32_t seed,
                                                                  float areaHalfExtent, float maxHalfExtent,
                                                                  float maxInitialSpeed = 0.0f) {
    std::vector<BodyHandle> bodies;
    if (count <= 0) {
        return bodies;
    }
    if (!(areaHalfExtent > 0.0f) || !(maxHalfExtent > 0.0f)) {
        std::fprintf(stderr, "[SceneBuilders] CreateRandomBoxField: invalid area/half extent (%.3f, %.3f)\n",
                     static_cast<double>(areaHalfExtent), static_cast<double>(maxHalfExtent));
        return bodies;
    }

    const float largestHalfExtent = std::max(maxHalfExtent, kSceneMinRandomHalfExtent);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> halfExtentDistribution(kSceneMinRandomHalfExtent,
                                                                 largestHalfExtent);
    std::uniform_real_distribution<float> horizontalDistribution(-areaHalfExtent, areaHalfExtent);
    std::uniform_real_distribution<float> angleDistribution(0.0f, 2.0f * kPi);
    std::uniform_real_distribution<float> speedDistribution(0.0f, std::max(maxInitialSpeed, 0.0f));
    std::uniform_real_distribution<float> directionDistribution(0.0f, 2.0f * kPi);
    std::uniform_real_distribution<float> spinDistribution(-kSceneMaxRandomSpin, kSceneMaxRandomSpin);

    // Top of the spawn band. The lower bound of y is one box half extent above the
    // ground, so nothing starts inside the half-space.
    const float spawnTopY = std::max(areaHalfExtent, largestHalfExtent * 2.0f);

    constexpr int32_t kMaxPlacementAttempts = 24;
    constexpr float kPlacementClearance = 0.02f; // m, keeps spawn AABBs from touching

    std::vector<AABB> occupied;
    occupied.reserve(static_cast<std::size_t>(count));
    bodies.reserve(static_cast<std::size_t>(count));

    for (int32_t bodyIndex = 0; bodyIndex < count; ++bodyIndex) {
        const float halfExtent = halfExtentDistribution(rng);
        const float angle = angleDistribution(rng);
        // World-aligned half extent of the rotated square: |cos|+|sin| times the side.
        const float alignedHalfExtent =
            halfExtent * (std::fabs(std::cos(angle)) + std::fabs(std::sin(angle)));
        const float minCenterY = alignedHalfExtent;
        const float maxCenterY = std::max(spawnTopY - alignedHalfExtent, minCenterY);
        std::uniform_real_distribution<float> verticalDistribution(minCenterY, maxCenterY);

        Vec2 position{0.0f, minCenterY};
        bool placed = false;
        for (int32_t attempt = 0; attempt < kMaxPlacementAttempts; ++attempt) {
            position = Vec2{horizontalDistribution(rng), verticalDistribution(rng)};
            const AABB candidate{position - Vec2{alignedHalfExtent + kPlacementClearance,
                                                 alignedHalfExtent + kPlacementClearance},
                                 position + Vec2{alignedHalfExtent + kPlacementClearance,
                                                 alignedHalfExtent + kPlacementClearance}};
            bool overlapsExisting = false;
            for (const AABB& existing : occupied) {
                if (Overlaps(candidate, existing)) {
                    overlapsExisting = true;
                    break;
                }
            }
            if (!overlapsExisting) {
                placed = true;
                occupied.push_back(AABB{position - Vec2{alignedHalfExtent, alignedHalfExtent},
                                        position + Vec2{alignedHalfExtent, alignedHalfExtent}});
                break;
            }
        }
        if (!placed) {
            // Dense field: keep the last candidate. The solver's position layer absorbs
            // the residual overlap; the scene is still a valid stress test.
            occupied.push_back(AABB{position - Vec2{alignedHalfExtent, alignedHalfExtent},
                                    position + Vec2{alignedHalfExtent, alignedHalfExtent}});
        }

        const BodyHandle handle =
            CreateBoxBody(world, position, angle, halfExtent, "random_box_" + std::to_string(bodyIndex));
        if (!world.IsValid(handle)) {
            continue;
        }
        if (maxInitialSpeed > 0.0f) {
            const float speed = speedDistribution(rng);
            const float direction = directionDistribution(rng);
            world.SetLinearVelocity(handle, Vec2{std::cos(direction), std::sin(direction)} * speed);
        }
        world.SetAngularVelocity(handle, spinDistribution(rng));
        bodies.push_back(handle);
    }
    return bodies;
}

// A row of `count` thin upright dominoes standing on the ground (Week 7 Dominoes
// scene). `halfExtents` = (thickness/2, height/2); the boxes are centred at
// y = groundY + halfExtents.y and spaced `spacing` apart along x, so the initial state
// has no box-box contacts (the chain only couples once the first one falls). Index 0 is
// the domino the scene pushes - the wave propagates towards increasing x.
[[nodiscard]] inline std::vector<BodyHandle> CreateDominoChain(World& world, int32_t count, float spacing,
                                                               Vec2 halfExtents, float groundY = 0.0f) {
    std::vector<BodyHandle> bodies;
    const float minimumSpacing = 2.0f * halfExtents.x; // never start interpenetrated
    if (count <= 0 || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) || !(spacing > minimumSpacing)) {
        std::fprintf(stderr,
                     "[SceneBuilders] CreateDominoChain: invalid count/spacing/half extents (%d, %.3f, "
                     "%.3f, %.3f)\n",
                     count, static_cast<double>(spacing), static_cast<double>(halfExtents.x),
                     static_cast<double>(halfExtents.y));
        return bodies;
    }
    bodies.reserve(static_cast<std::size_t>(count));
    const float centerY = groundY + halfExtents.y;
    const float rowOffset = static_cast<float>(count - 1) * 0.5f;
    for (int32_t index = 0; index < count; ++index) {
        const Vec2 position{(static_cast<float>(index) - rowOffset) * spacing, centerY};
        bodies.push_back(
            CreateBoxBody(world, position, 0.0f, halfExtents, "domino_" + std::to_string(index)));
    }
    return bodies;
}

// A rectangular grid of upright boxes (Week 7 IslandSleep500 scene): `rows` columns
// along x spaced `horizontalPitch`, each holding `columns` boxes stacked with
// `verticalPitch` (0 = default = exactly 2*boxHalfExtent, i.e. an exactly stacked
// tower). Total = rows*columns. Distinct columns are separated horizontally, so with
// horizontalPitch > 2*boxHalfExtent each column is its own island once the island
// builder runs - a scene that shows both the island partition and whole-island sleep.
[[nodiscard]] inline std::vector<BodyHandle> CreateBoxGrid(World& world, int32_t rows, int32_t columns,
                                                           float boxHalfExtent, float horizontalPitch,
                                                           float verticalPitch = 0.0f, float groundY = 0.0f) {
    std::vector<BodyHandle> bodies;
    const float resolvedVerticalPitch = verticalPitch > 0.0f ? verticalPitch : 2.0f * boxHalfExtent;
    if (rows <= 0 || columns <= 0 || !(boxHalfExtent > 0.0f) || !(horizontalPitch > 2.0f * boxHalfExtent) ||
        !(resolvedVerticalPitch >= 2.0f * boxHalfExtent)) {
        std::fprintf(
            stderr, "[SceneBuilders] CreateBoxGrid: invalid rows/columns/pitches (%d, %d, %.3f, %.3f)\n",
            rows, columns, static_cast<double>(horizontalPitch), static_cast<double>(resolvedVerticalPitch));
        return bodies;
    }
    bodies.reserve(static_cast<std::size_t>(rows) * static_cast<std::size_t>(columns));
    const float rowOffset = static_cast<float>(rows - 1) * 0.5f;
    for (int32_t rowIndex = 0; rowIndex < rows; ++rowIndex) {
        const float centerX = (static_cast<float>(rowIndex) - rowOffset) * horizontalPitch;
        for (int32_t columnIndex = 0; columnIndex < columns; ++columnIndex) {
            // +2 mm stagger-free gap is not needed: verticalPitch >= box height means the
            // initial state is exactly touching (zero penetration) like the W5 tower.
            const Vec2 position{centerX, groundY + boxHalfExtent +
                                             resolvedVerticalPitch * static_cast<float>(columnIndex)};
            bodies.push_back(
                CreateBoxBody(world, position, 0.0f, boxHalfExtent,
                              "grid_box_" + std::to_string(rowIndex) + "_" + std::to_string(columnIndex)));
        }
    }
    return bodies;
}

} // namespace p2
