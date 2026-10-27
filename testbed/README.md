# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 (D5/D6): GLFW window +
OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer, camera and
the `FallingBall` demo.

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable)
build/debug/physics2d_testbed.exe

# automated smoke test: render 120 frames with a hidden window, print a summary
build/debug/physics2d_testbed.exe --smoke 120 --hidden
```

Output of a successful smoke run:

```text
[testbed] GLFW 3.4.0 ...
[testbed] OpenGL 3.3.0 ... (GPU / driver name)
[FallingBall] one dynamic circle at (0, 5.0), mass 1 kg, gravity (0, -10.0); ...
[smoke] frames=120 physics_steps=120 bodies=1 avg_frame_ms=0.123
[smoke] first_body='falling_ball' position=(0.000, ...)
```

Exit codes: `0` success, `1` failure, `2` smoke test could not run because the
environment has no usable desktop session / OpenGL 3.3 context.

In smoke mode every frame feeds the fixed-step accumulator exactly `1/60 s`, so
`frames == physics_steps` is deterministic and independent of machine speed.

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera |
| Left panel | Scene selector, play/pause, single step, speed (1x / 0.5x / 0.25x), gravity slider, display toggles, reset |
| Right panel | FPS, frame time, physics step time, body/awake counts, gravity |

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI, fixed-timestep loop, demo registry |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP |
| `DebugUI.*` | Left control panel + right status panel |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) |
| `demos/FallingBall.*` | Week 1 scene: falling circle, rotating square, visual ground line |

Adding a scene: implement `p2::Demo` in `testbed/demos/<Name>.cpp` (the CMake glob
picks it up) and register it in the `demos` vector in `main.cpp`.
