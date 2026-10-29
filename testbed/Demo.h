#pragma once

// p2::Demo - base class for interactive testbed scenes (Section 3.1 / P14).
//
// Why an interface for a single scene: Week 1 only ships FallingBall, but the
// design document schedules eight more scenes (SphereStack, Dominoes, Pyramid200,
// ...). The testbed owns a vector<unique_ptr<Demo>> and rebuilds the World when
// the selection changes, so adding a scene never touches the window/render loop.
//
// Contract:
//  - Setup() receives a cleared World and must (re)create the scene contents.
//  - Update() runs once per rendered frame (before physics is advanced for that
//    frame in the current loop layout) and may auto-reset the scene.
//  - Draw() is read-only with respect to the World: it visualises state, it never
//    mutates the simulation.
//  - Ui() adds scene-specific controls; the global controls (pause, gravity,
//    display toggles) live in DebugUI.
//  - Reset() rebuilds the scene inside the World (Clear + Setup is the canonical
//    implementation) and is bound to the UI reset button.
//
// Week 6: a scene that displays performance numbers (StressBenchmark) needs the host
// loop's timings. Rather than widening Ui() for every demo, the host pushes one small
// stats struct per frame into an optional hook; demos that do not display timings keep
// the do-nothing default. This is a testbed-internal addition (no core API change).

namespace p2 {

class Renderer;
class World;

// Host-loop timings handed to Demo::SetFrameStats. Deliberately tiny: only what a
// demo panel would quote. FPS is the smoothed frame rate, stepMilliseconds is the
// smoothed cost of one World::Step (or of the whole stepped batch this frame), and
// rawStepMilliseconds is this frame's un-smoothed Step cost - Week 7's IslandSleep500
// averages the two sleep states from RAW values so the EMA does not blur the transition.
struct DemoFrameStats {
    float framesPerSecond = 0.0f;
    float stepMilliseconds = 0.0f;
    float rawStepMilliseconds = 0.0f;
};

class Demo {
  public:
    virtual ~Demo() = default;

    Demo(const Demo&) = delete;
    Demo& operator=(const Demo&) = delete;
    Demo(Demo&&) = delete;
    Demo& operator=(Demo&&) = delete;

    [[nodiscard]] virtual const char* Name() const noexcept = 0;
    virtual void Setup(World& world) = 0;
    virtual void Update(World& world, float dt) = 0;
    virtual void Draw(Renderer& renderer, const World& world) = 0;
    virtual void Ui(World& world) = 0;
    virtual void Reset(World& world) = 0;

    // Non-pure on purpose: the timing readout is a Week 6 convenience for one scene.
    virtual void SetFrameStats(const DemoFrameStats& /*stats*/) {}

    // Week 7: scene-specific smoke summary lines printed after main.cpp's generic
    // `[smoke] demo=...` block (Dominoes chain state, IslandSleep500 awake-vs-asleep
    // Step-time comparison). Non-pure: most scenes have nothing scene-specific to add.
    virtual void PrintSmokeSummary(const World& /*world*/) const {}

  protected:
    Demo() = default;
};

} // namespace p2
