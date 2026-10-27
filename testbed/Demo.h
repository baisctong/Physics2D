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

namespace p2 {

class Renderer;
class World;

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

  protected:
    Demo() = default;
};

} // namespace p2
