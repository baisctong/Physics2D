#include "demos/Dominoes.h"

#include "Renderer.h"
#include "SceneBuilders.h"

#include "physics2d/Collision/ShapeTypes.h"
#include "physics2d/Core/Shape.h"
#include "physics2d/Core/World.h"
#include "physics2d/Math/MathUtils.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace p2 {
namespace {

constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
// Sleeping dominoes are drawn cold/dim, awake ones warm: in a 20-domino chain the
// colour is the fastest way to see the wake wave travel.
constexpr Color kSleepingColor{0.35f, 0.45f, 0.70f, 1.0f};
constexpr Color kAwakeColor{0.98f, 0.62f, 0.25f, 1.0f};
constexpr Color kFallenColor{0.35f, 0.85f, 0.45f, 1.0f};
constexpr Color kCentroidColor{1.00f, 0.95f, 0.85f, 1.0f};
constexpr float kVelocityArrowScale = 0.25f;
constexpr float kVelocityArrowThresholdSquared = 0.01f * 0.01f;
// A domino counts as fallen once it has rotated more than 60 degrees from upright.
constexpr float kFallenAngleRadians = 1.0471976f; // 60 deg

struct DominoCounts {
    int32_t awake = 0;
    int32_t sleeping = 0;
    int32_t wokenAfterSleep = 0;
    int32_t fallen = 0;
    int32_t chainReach = 0; // consecutive fallen dominoes starting at index 0
    int32_t firstSkipped = -1;
    float maxSpeed = 0.0f;
};

[[nodiscard]] DominoCounts CountDominoes(const World& world, const std::vector<BodyHandle>& dominoes,
                                         const std::vector<uint8_t>& wokenInChain) {
    DominoCounts counts;
    for (std::size_t index = 0; index < dominoes.size(); ++index) {
        const BodyHandle handle = dominoes[index];
        if (!world.IsValid(handle)) {
            continue;
        }
        if (world.IsAwake(handle)) {
            ++counts.awake;
        } else {
            ++counts.sleeping;
        }
        if (index < wokenInChain.size() && wokenInChain[index] != 0) {
            ++counts.wokenAfterSleep;
        }
        if (std::fabs(world.GetAngle(handle)) > kFallenAngleRadians) {
            ++counts.fallen;
        }
        counts.maxSpeed = std::max(counts.maxSpeed, Length(world.GetLinearVelocity(handle)));
    }
    // Chain reach: keep walking while the next domino has fallen. The first upright one
    // after the fallen prefix is the "skipped" domino (should stay -1 in a healthy chain).
    for (std::size_t index = 0; index < dominoes.size(); ++index) {
        if (!world.IsValid(dominoes[index])) {
            break;
        }
        if (std::fabs(world.GetAngle(dominoes[index])) > kFallenAngleRadians) {
            ++counts.chainReach;
        } else {
            if (counts.firstSkipped < 0 && counts.fallen > 0) {
                counts.firstSkipped = static_cast<int32_t>(index);
            }
            break;
        }
    }
    return counts;
}

} // namespace

const char* DominoesDemo::Name() const noexcept {
    return "Dominoes";
}

void DominoesDemo::Setup(World& world) {
    m_dominoCount = kDefaultDominoCount;
    m_elapsedSeconds = 0.0f;
    m_pushApplied = false;
    m_resetCount = 0;
    m_ground = BodyHandle{};
    m_dominoes.clear();
    m_wasAsleep.clear();
    m_wokenInChain.clear();
    BuildScene(world);
}

void DominoesDemo::BuildScene(World& world) {
    m_ground = CreateGroundPlane(world, 0.0f);
    m_dominoes = CreateDominoChain(world, m_dominoCount, kDominoSpacing,
                                   Vec2{kDominoHalfWidth, kDominoHalfHeight}, 0.0f);
    m_wasAsleep.assign(m_dominoes.size(), 0);
    m_wokenInChain.assign(m_dominoes.size(), 0);
    std::printf("[Dominoes] static ground + %d dominoes (%.2f x %.2f m, hx=%.2f hy=%.2f) spaced "
                "%.2f m along x; auto-push first at t=%.1f s (step 60 in smoke); goal: chain wake, "
                "no domino skipped\n",
                static_cast<int32_t>(m_dominoes.size()), static_cast<double>(2.0f * kDominoHalfWidth),
                static_cast<double>(2.0f * kDominoHalfHeight), static_cast<double>(kDominoHalfWidth),
                static_cast<double>(kDominoHalfHeight), static_cast<double>(kDominoSpacing),
                static_cast<double>(kAutoPushSeconds));
}

void DominoesDemo::PushFirst(World& world) {
    if (m_dominoes.empty() || !world.IsValid(m_dominoes.front())) {
        return;
    }
    const BodyHandle first = m_dominoes.front();
    const Vec2 position = world.GetPosition(first);
    // Hit the first domino near its top: the lever arm produces the tipping torque and,
    // because World::ApplyLinearImpulse wakes the whole sleeping island first, this is
    // also the external wake trigger the Week 7 sleep contract must handle.
    world.ApplyLinearImpulse(first, Vec2{kPushImpulse, 0.0f},
                             position + Vec2{0.0f, kDominoHalfHeight * 0.9f});
    m_pushApplied = true;
    std::printf("[Dominoes] pushed domino 0 with impulse %.3f N*s at t=%.3f s\n",
                static_cast<double>(kPushImpulse), static_cast<double>(m_elapsedSeconds));
}

void DominoesDemo::Update(World& world, float dt) {
    m_elapsedSeconds += dt;

    // Per-domino history: record "has slept" so a later wake can be attributed to the
    // chain rather than to the initial settle (the row starts awake).
    for (std::size_t index = 0; index < m_dominoes.size(); ++index) {
        const BodyHandle handle = m_dominoes[index];
        if (!world.IsValid(handle)) {
            continue;
        }
        const bool awake = world.IsAwake(handle);
        if (!awake && index < m_wasAsleep.size()) {
            m_wasAsleep[index] = 1;
        }
        if (m_pushApplied && awake && index < m_wasAsleep.size() && m_wasAsleep[index] != 0 &&
            index < m_wokenInChain.size()) {
            m_wokenInChain[index] = 1;
        }
    }

    // Unattended smoke run: push the first domino once after the row has settled (and, in
    // a sleeping-enabled build, slept). A manual "Push first" click before then wins.
    if (!m_pushApplied && m_elapsedSeconds >= kAutoPushSeconds) {
        PushFirst(world);
    }

    // Safety net: a domino below the infinite ground plane means a physics regression.
    for (const BodyHandle handle : m_dominoes) {
        if (world.IsValid(handle) && world.GetPosition(handle).y < -5.0f) {
            std::fprintf(stderr, "[Dominoes] a domino fell through the ground plane - physics regression, "
                                 "rebuilding the scene\n");
            Reset(world);
            return;
        }
    }
}

void DominoesDemo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, 0.0f}, {kGroundHalfWidth, 0.0f}, kGroundColor);

    for (const BodyHandle handle : m_dominoes) {
        if (!world.IsValid(handle)) {
            continue;
        }
        const Body* body = world.GetBody(handle);
        if (body == nullptr || !world.IsValidShape(body->shape)) {
            continue;
        }
        const Shape* shape = world.GetShape(body->shape);
        if (shape == nullptr || shape->type != ShapeType::Polygon) {
            continue;
        }
        const bool fallen = std::fabs(world.GetAngle(handle)) > kFallenAngleRadians;
        const bool asleep = !world.IsAwake(handle);
        const Color color = fallen ? kFallenColor : (asleep ? kSleepingColor : kAwakeColor);

        std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
        const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
        const Transform transform = world.GetTransform(handle);
        for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            worldVertices[static_cast<std::size_t>(vertexIndex)] =
                Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
        }
        renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount), color);
        renderer.DrawPoint(Mul(transform, shape->polygon.centroid), kCentroidColor, 0.02f);

        const Vec2 velocity = world.GetLinearVelocity(handle);
        if (LengthSquared(velocity) > kVelocityArrowThresholdSquared) {
            const Vec2 position = world.GetPosition(handle);
            renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kCentroidColor);
        }
    }
}

void DominoesDemo::Ui(World& world) {
    ImGui::TextUnformatted("Thin dominoes on the ground: push domino 0, watch the wake wave.");
    ImGui::TextUnformatted("Chain wake must be whole-island atomic: no domino may be skipped.");

    const DominoCounts counts = CountDominoes(world, m_dominoes, m_wokenInChain);
    int32_t touchingContacts = 0;
    for (const Contact& contact : world.GetContacts()) {
        if (contact.manifold.IsTouching()) {
            ++touchingContacts;
        }
    }
    ImGui::Text("Total: %d   Awake: %d   Sleeping: %d", static_cast<int32_t>(m_dominoes.size()), counts.awake,
                counts.sleeping);
    ImGui::Text("Islands: %d   Contacts: %d   max speed: %.2f m/s", world.GetIslandCount(), touchingContacts,
                static_cast<double>(counts.maxSpeed));

    ImGui::Separator();
    ImGui::Text("Woken after sleep: %d / %d", counts.wokenAfterSleep,
                static_cast<int32_t>(m_dominoes.size()));
    ImGui::Text("Fallen: %d   Chain reach: %d / %d", counts.fallen, counts.chainReach,
                static_cast<int32_t>(m_dominoes.size()));
    if (counts.firstSkipped >= 0) {
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.30f, 1.0f), "First skipped (still upright): domino %d",
                           counts.firstSkipped);
    } else if (counts.chainReach == static_cast<int32_t>(m_dominoes.size())) {
        ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.40f, 1.0f), "Full chain fallen, no domino skipped");
    } else {
        ImGui::TextDisabled("Chain in progress (reach %d, no gap so far)", counts.chainReach);
    }
    ImGui::TextDisabled("Auto-push at t=%.1f s; button pushes now. Push state: %s",
                        static_cast<double>(kAutoPushSeconds), m_pushApplied ? "pushed" : "waiting");

    ImGui::Separator();
    if (ImGui::Button("Push first")) {
        PushFirst(world);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        Reset(world);
    }

    // Rebuilding is an explicit action (never per frame): the count slider only takes
    // effect through the Rebuild button.
    ImGui::SliderInt("Domino count", &m_dominoCount, kMinDominoCount, kMaxDominoCount, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) {
        Reset(world);
    }
    ImGui::Text("Resets: %d", m_resetCount);
}

void DominoesDemo::Reset(World& world) {
    world.Clear();
    m_ground = BodyHandle{};
    m_dominoes.clear();
    m_wasAsleep.clear();
    m_wokenInChain.clear();
    m_elapsedSeconds = 0.0f;
    m_pushApplied = false;
    ++m_resetCount;
    BuildScene(world);
}

void DominoesDemo::PrintSmokeSummary(const World& world) const {
    const DominoCounts counts = CountDominoes(world, m_dominoes, m_wokenInChain);
    std::printf("[smoke] dominoes total=%d woken_after_sleep=%d fallen=%d chain_reach=%d first_skipped=%d "
                "awake=%d sleeping=%d islands=%d pushed=%d\n",
                static_cast<int32_t>(m_dominoes.size()), counts.wokenAfterSleep, counts.fallen,
                counts.chainReach, counts.firstSkipped, counts.awake, counts.sleeping, world.GetIslandCount(),
                m_pushApplied ? 1 : 0);
}

} // namespace p2
