# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 built the shell (GLFW
window + OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer,
camera). Week 2 upgrades the `FallingBall` scene to the first real collision demo:
a dynamic circle falling onto an infinite static ground plane, with contact
visualisation and a live restitution slider.

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable)
build/testbed2/testbed/physics2d_testbed.exe

# automated smoke test: 120 deterministic frames with a hidden window
build/testbed2/testbed/physics2d_testbed.exe --smoke 120 --hidden
```

Output of a successful Week 2 smoke run:

```text
[testbed] GLFW 3.4.0 ...
[testbed] OpenGL 3.3.0 ... (GPU / driver name)
[FallingBall] static ground plane y=0 + dynamic circle r=0.5 at (0, 4.0), density 1 kg/m^2, restitution 0.00; ...
[smoke] frames=120 physics_steps=120 bodies=2 shapes=2 contacts=1 avg_frame_ms=0.123
[smoke] body='ground' position=(0.000, 0.000) velocity=(0.000, 0.000)
[smoke] body='falling_ball' position=(0.000, 0.500) velocity=(0.000, 0.000)
```

Exit codes: `0` success, `1` failure, `2` smoke test could not run because the
environment has no usable desktop session / OpenGL 3.3 context.

In smoke mode every frame feeds the fixed-step accumulator exactly `1/60 s`, so
`frames == physics_steps` is deterministic and independent of machine speed. The
ball needs ~50 steps to free-fall from y = 4 to the surface, so a 120-frame run
always ends with the ball resting in contact with the ground.

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera |
| Left panel | Scene selector, play/pause, single step, speed (1x / 0.5x / 0.25x), gravity slider, display toggles (axes, AABBs, contact points, contact normals), reset, quit |
| Scene parameters | Restitution slider (0..1, default 0), "Drop again" button, ball y/vy, contact count and max penetration (mm) |
| Right panel (Status) | FPS, frame time, physics step time, body/shape/awake counts, contact count, max penetration (mm), gravity |

The restitution slider calls `World::SetRestitution` every frame it changes:
`e = 0` lands and stops, `e = 0.5`/`0.8` bounce. **The bounce is approximate on
purpose**: Week 2 implements a temporary position-level response (push out +
normal-velocity projection). Momentum conservation and correct restitution arrive
with the Week 3 Sequential Impulse solver; split impulse in Week 5 replaces the
position-level part.

The contact overlay is drawn by `main.cpp` (it is engine-level data owned by
`World`, shared by every future scene), gated by the DebugUI display toggles.
When `Contact points` is on, each manifold anchor is a red ring + cross; when
`Contact normals` is on, a yellow arrow shows the manifold normal (A -> B, i.e.
out of the ground for this scene).

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI, fixed-timestep loop, demo registry, engine-level debug overlays (axes/AABBs/contacts/normals), smoke summary |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP |
| `DebugUI.*` | Left control panel (incl. contact toggles) + right status panel (incl. contact/penetration stats) |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) |
| `demos/FallingBall.*` | Week 2 scene: static ground plane + dynamic circle, restitution slider, drop-again respawn, safety auto-reset if a regression lets the ball tunnel |

Adding a scene: implement `p2::Demo` in `testbed/demos/<Name>.cpp` (the CMake glob
picks it up) and register it in the `demos` vector in `main.cpp`.
