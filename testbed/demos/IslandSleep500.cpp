#include "demos/IslandSleep500.h"

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
#include <cstdio>

namespace p2 {
namespace {

constexpr Color kGroundColor{0.55f, 0.60f, 0.66f, 1.0f};
// Sleeping boxes are drawn dim blue; awake ones warm. A 500-box scene is a sea of
// wireframes, so the colour is what makes "the whole scene just fell asleep" visible.
constexpr Color kSleepingColor{0.32f, 0.42f, 0.65f, 1.0f};
constexpr Color kAwakeColor{0.98f, 0.62f, 0.25f, 1.0f};

[[nodiscard]] const char* FormatAverage(char* buffer, std::size_t bufferSize, double sum, int64_t count,
                                        const char* label) {
    if (count <= 0) {
        std::snprintf(buffer, bufferSize, "%s n/a", label);
    } else {
        std::snprintf(buffer, bufferSize, "%s %.4f ms (%lld frames)", label, sum / static_cast<double>(count),
                      static_cast<long long>(count));
    }
    return buffer;
}

} // namespace

const char* IslandSleep500Demo::Name() const noexcept {
    return "Island Sleep 500";
}

void IslandSleep500Demo::Setup(World& world) {
    m_frameIndex = 0;
    m_firstAllAsleepFrame = -1;
    m_earlyStepSum = 0.0;
    m_earlyFrameCount = 0;
    m_awakeStepSum = 0.0;
    m_awakeFrameCount = 0;
    m_asleepStepSum = 0.0;
    m_asleepFrameCount = 0;
    m_resetCount = 0;
    m_ground = BodyHandle{};
    m_boxes.clear();
    BuildScene(world);
}

void IslandSleep500Demo::BuildScene(World& world) {
    m_ground = CreateGroundPlane(world, 0.0f);
    m_boxes = CreateBoxGrid(world, kColumnCount, kStackHeight, kBoxHalfExtent, kHorizontalPitch);

    std::printf("[IslandSleep500] static ground + %d boxes in %d towers x %d boxes/tower (1x1 m, pitch "
                "%.2f m); %d islands expected; sleeping enabled by default -> settle then all asleep\n",
                static_cast<int32_t>(m_boxes.size()), kColumnCount, kStackHeight,
                static_cast<double>(kHorizontalPitch), kColumnCount);
}

void IslandSleep500Demo::SetFrameStats(const DemoFrameStats& stats) {
    m_frameStats = stats;
}

void IslandSleep500Demo::Update(World& world, float /*dt*/) {
    ++m_frameIndex;

    // "All asleep" is measured on this scene's boxes, not on World::GetSleepingBodyCount()
    // alone: the demo must know when THIS scene is quiet (the world has no other dynamics,
    // but being explicit keeps the metric meaningful if the demo is ever embedded).
    int32_t asleepBoxCount = 0;
    for (const BodyHandle handle : m_boxes) {
        if (world.IsValid(handle) && !world.IsAwake(handle)) {
            ++asleepBoxCount;
        }
    }
    const bool allAsleep = !m_boxes.empty() && asleepBoxCount == static_cast<int32_t>(m_boxes.size());
    if (allAsleep && m_firstAllAsleepFrame < 0) {
        m_firstAllAsleepFrame = m_frameIndex;
        std::printf("[IslandSleep500] all %d boxes asleep at frame %d (t=%.3f s)\n",
                    static_cast<int32_t>(m_boxes.size()), m_firstAllAsleepFrame,
                    static_cast<double>(m_firstAllAsleepFrame) * static_cast<double>(kFixedTimeStep));
    }

    // Raw Step timing from the host loop (0 when paused / no step this frame). The
    // windows are frame-based, which is exactly what the smoke run's frames==steps
    // guarantee makes comparable to "steps".
    const float stepMilliseconds = m_frameStats.rawStepMilliseconds;
    if (stepMilliseconds > 0.0f && m_frameIndex > kWarmupFrames) {
        if (m_frameIndex >= kEarlyWindowStart && m_frameIndex < kEarlyWindowEnd) {
            m_earlyStepSum += static_cast<double>(stepMilliseconds);
            ++m_earlyFrameCount;
        }
        if (allAsleep) {
            m_asleepStepSum += static_cast<double>(stepMilliseconds);
            ++m_asleepFrameCount;
        } else {
            m_awakeStepSum += static_cast<double>(stepMilliseconds);
            ++m_awakeFrameCount;
        }
    }
}

void IslandSleep500Demo::Draw(Renderer& renderer, const World& world) {
    renderer.DrawSegment({-kGroundHalfWidth, 0.0f}, {kGroundHalfWidth, 0.0f}, kGroundColor);

    for (const BodyHandle handle : m_boxes) {
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
        const bool asleep = !world.IsAwake(handle);

        std::array<Vec2, kMaxPolygonVerticesLocal> worldVertices{};
        const int32_t vertexCount = std::min(shape->polygon.count, kMaxPolygonVerticesLocal);
        const Transform transform = world.GetTransform(handle);
        for (int32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            worldVertices[static_cast<std::size_t>(vertexIndex)] =
                Mul(transform, shape->polygon.vertices[static_cast<std::size_t>(vertexIndex)]);
        }
        renderer.DrawPolygon(worldVertices.data(), static_cast<std::size_t>(vertexCount),
                             asleep ? kSleepingColor : kAwakeColor);
    }
}

void IslandSleep500Demo::WakeAll(World& world) {
    // WakeBody is the documented external wake path; it wakes the WHOLE island of each
    // body (waking only one box would leave its sleeping tower frozen around it).
    for (const BodyHandle handle : m_boxes) {
        if (world.IsValid(handle)) {
            world.WakeBody(handle);
        }
    }
    std::printf("[IslandSleep500] wake-all requested; the totals will climb again as the towers "
                "are re-slept\n");
}

void IslandSleep500Demo::Ui(World& world) {
    // Two short lines instead of one long sentence: the panel is 340 px wide and ImGui
    // clips (does not wrap) a Text line, so the previous one-liners lost their tail -
    // including the measured asleep Step time, which is the number this scene exists for.
    ImGui::TextUnformatted("500 boxes in 100 five-box towers.");
    ImGui::TextUnformatted("Measured: awake vs asleep Step ms.");

    int32_t asleepBoxCount = 0;
    for (const BodyHandle handle : m_boxes) {
        if (world.IsValid(handle) && !world.IsAwake(handle)) {
            ++asleepBoxCount;
        }
    }
    int32_t touchingContacts = 0;
    for (const Contact& contact : world.GetContacts()) {
        if (contact.manifold.IsTouching()) {
            ++touchingContacts;
        }
    }

    ImGui::Text("Boxes: %d   Awake: %d   Sleeping: %d", static_cast<int32_t>(m_boxes.size()),
                static_cast<int32_t>(m_boxes.size()) - asleepBoxCount, asleepBoxCount);
    ImGui::Text("Islands: %d   Contacts: %d", world.GetIslandCount(), touchingContacts);

    const SolverSettings& settings = world.GetSolverSettings();
    ImGui::Text("enableSleeping: %s   useIslandSolving: %s", settings.enableSleeping ? "on" : "off",
                settings.useIslandSolving ? "on" : "off");
    ImGui::Text("Step: %.3f ms (smoothed)   last: %.3f ms",
                static_cast<double>(m_frameStats.stepMilliseconds),
                static_cast<double>(m_frameStats.rawStepMilliseconds));
    ImGui::Text("First all-asleep frame: %d%s", m_firstAllAsleepFrame,
                m_firstAllAsleepFrame >= 0 ? "" : " (not reached yet)");

    ImGui::Separator();
    char buffer[128];
    ImGui::TextUnformatted(FormatAverage(buffer, sizeof(buffer), m_earlyStepSum, m_earlyFrameCount, "early"));
    ImGui::TextUnformatted(FormatAverage(buffer, sizeof(buffer), m_awakeStepSum, m_awakeFrameCount, "awake"));
    ImGui::TextUnformatted(
        FormatAverage(buffer, sizeof(buffer), m_asleepStepSum, m_asleepFrameCount, "asleep"));
    if (m_asleepFrameCount > 0 && m_awakeFrameCount > 0) {
        const double awakeAverage = m_awakeStepSum / static_cast<double>(m_awakeFrameCount);
        const double asleepAverage = m_asleepStepSum / static_cast<double>(m_asleepFrameCount);
        // One compact line: the 340 px panel clips (does not wrap) long Text lines, and the
        // asleep Step time is the number this scene exists for. "awake -> asleep ms" keeps
        // both windows readable in the width that is left after the label.
        ImGui::Text("Speedup: %.2fx (%.4f -> %.4f ms)", awakeAverage / asleepAverage, awakeAverage,
                    asleepAverage);
    } else {
        ImGui::TextDisabled("Asleep window not reached yet (sleeping may be disabled or not settled).");
    }

    ImGui::Separator();
    if (ImGui::Button("Wake all")) {
        WakeAll(world);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        Reset(world);
    }
    ImGui::SameLine();
    ImGui::Text("Resets: %d", m_resetCount);
}

void IslandSleep500Demo::Reset(World& world) {
    world.Clear();
    m_ground = BodyHandle{};
    m_boxes.clear();
    ++m_resetCount;
    m_frameIndex = 0;
    m_firstAllAsleepFrame = -1;
    m_earlyStepSum = 0.0;
    m_earlyFrameCount = 0;
    m_awakeStepSum = 0.0;
    m_awakeFrameCount = 0;
    m_asleepStepSum = 0.0;
    m_asleepFrameCount = 0;
    BuildScene(world);
}

void IslandSleep500Demo::PrintSmokeSummary(const World& world) const {
    const float earlyAverage =
        m_earlyFrameCount > 0 ? static_cast<float>(m_earlyStepSum / static_cast<double>(m_earlyFrameCount))
                              : 0.0f;
    const float awakeAverage =
        m_awakeFrameCount > 0 ? static_cast<float>(m_awakeStepSum / static_cast<double>(m_awakeFrameCount))
                              : 0.0f;
    const float asleepAverage =
        m_asleepFrameCount > 0 ? static_cast<float>(m_asleepStepSum / static_cast<double>(m_asleepFrameCount))
                               : 0.0f;
    const float speedup = (awakeAverage > 0.0f && asleepAverage > 0.0f) ? awakeAverage / asleepAverage : 0.0f;

    std::printf("[smoke] islandsleep500 boxes=%d islands=%d awake=%d sleeping=%d first_all_asleep_frame=%d "
                "early_avg_step_ms=%.4f early_frames=%lld awake_avg_step_ms=%.4f awake_frames=%lld "
                "asleep_avg_step_ms=%.4f asleep_frames=%lld speedup=%.2fx\n",
                static_cast<int32_t>(m_boxes.size()), world.GetIslandCount(), world.GetAwakeBodyCount(),
                world.GetSleepingBodyCount(), m_firstAllAsleepFrame, static_cast<double>(earlyAverage),
                static_cast<long long>(m_earlyFrameCount), static_cast<double>(awakeAverage),
                static_cast<long long>(m_awakeFrameCount), static_cast<double>(asleepAverage),
                static_cast<long long>(m_asleepFrameCount), static_cast<double>(speedup));
}

} // namespace p2
