# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 built the shell (GLFW
window + OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer,
camera). Week 2 added shapes, the analytic circle/plane narrow phase, contact
visualisation and the ground-plane scene. Week 3 added the Sequential Impulse
velocity solver, the stacked-sphere scene and `--demo` scene selection for
automated smoke runs. Week 4 adds polygon SAT + clipping and the ten-box tower
scene (`BoxStack`), whose visible jitter is the baseline that Week 5's warm
starting/friction/split impulse will be compared against.

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable), FallingBall is the default scene
build/debug/testbed/physics2d_testbed.exe
build/debug/testbed/physics2d_testbed.exe --demo SphereStack
build/debug/testbed/physics2d_testbed.exe --demo BoxStack

# automated smoke runs (deterministic frames, hidden window).
# With scripts/build.ps1 -BuildDir build/testbed4 the binary lands in
# build/testbed4/testbed/ instead.
build/debug/testbed/physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden
build/debug/testbed/physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
```

`--demo NAME` accepts `FallingBall`, `SphereStack` or `BoxStack` (case-insensitive;
spaces, underscores and hyphens are ignored, so `sphere stack` also works). An
unknown name prints the available scene list and exits with code **2** - the smoke
harness uses that to distinguish a typo from a physics/render failure.

Output of a successful smoke run (observed on the dev machine; values vary slightly
per machine):

```text
[testbed] GLFW 3.4.0 ...
[testbed] OpenGL 3.3.0 ... (GPU / driver name)
[SphereStack] static ground plane y=0 + 3 dynamic circles r=0.5 at y=0.5/1.5/2.5, ...
[smoke] demo=SphereStack frames=300 physics_steps=300 bodies=4 shapes=4 contacts=3 avg_frame_ms=0.936
[smoke] body='ground' position=(0.000, 0.000) velocity=(0.000, 0.000) angle=0.00deg
[smoke] body='sphere_0' position=(0.000, 0.494) velocity=(0.000, -0.012) angle=0.00deg
[smoke] body='sphere_1' position=(0.000, 1.488) velocity=(0.000, -0.019) angle=0.00deg
[smoke] body='sphere_2' position=(0.000, 2.482) velocity=(0.000, -0.019) angle=0.00deg
[smoke] contact 'ground'|'sphere_0' points=1 penetration=5.824mm normal=(0.000, 1.000)
[smoke] contact 'sphere_0'|'sphere_1' points=1 penetration=6.236mm normal=(0.000, 1.000)
[smoke] contact 'sphere_1'|'sphere_2' points=1 penetration=5.618mm normal=(0.000, 1.000)
```

The stack stands: all three centres stay within ~2 cm of the spawn heights with
residual velocities <= 0.02 m/s and three persistent contacts (the small negative
velocity is the allowed Week 3 jitter). `FallingBall --smoke 150 --hidden` ends with
`contacts=1`, ball `y=0.495`, velocity zero and 5 mm penetration (the linear slop).

### BoxStack (Week 4): the expected "before" result

`--demo BoxStack --smoke 300 --hidden` exits 0 and prints the honest Week 4 outcome:
`bodies=11 shapes=11 contacts=10`. **The tower collapses** - with no friction and no
warm starting the solver residual tilts the boxes and nothing damps the lateral slide,
so after 300 steps the ten boxes lie scattered in a line on the ground
(`x` from about -13 m to +15 m, `y ~= 0.50 m`, lateral speeds up to ~5.3 m/s, several
boxes tilted to +/-90 deg). Every remaining contact is `ground|box_N` (2 points,
normal (0,1), ~5 mm slop); no box-box contact survives. This is not tuned away on
purpose: it is the W4 baseline that Week 5's friction + warm starting is measured
against. The per-body smoke lines include `angle=...deg` precisely so the tilt is
visible in the log.

Exit codes: `0` success, `1` failure/usage error, `2` no usable GL environment or
unknown `--demo` name. In smoke mode every frame feeds the fixed-step accumulator
exactly `1/60 s`, so `frames == physics_steps` is deterministic and independent of
machine speed.

## Scenes

| Scene | Description |
| --- | --- |
| `FallingBall` (default) | Dynamic circle (r=0.5, density 1) dropped from y=4 onto a static ground plane at y=0. Restitution slider (0..1) is solved by the Week 3 Sequential Impulse solver: e=0 lands and stops, e>0 bounces with the expected energy loss. Restitution is applied only above `kRestitutionVelocityThreshold` (1 m/s), so resting contacts stay quiet. |
| `SphereStack` | Three dynamic circles (r=0.5, density 1, restitution 0) stacked exactly tangent at y=0.5/1.5/2.5 on the same ground plane. This is the Week 3 solver check: a standing stack with small jitter is a pass; a collapsed stack is a visible physics failure. No auto-reset - the result is the measurement. |
| `BoxStack` | Ten dynamic 1x1 m boxes (half extents 0.5, density 1, restitution 0) stacked exactly at y=0.5..9.5, drawn as wireframes from the polygon vertices stored in the shape. This is the Week 4 polygon SAT/clipping scene: with normal impulses only (no friction, no warm start, no split impulse) the tower is expected to jitter, sink and slowly tilt. It is deliberately not tuned - the W4-vs-W5 comparison is the point. Optional "Drop impact ball" adds a dynamic circle above the tower (circle-vs-polygon path). No auto-reset. |

All scenes use the shared engine-level contact overlay: red ring+cross at every
manifold anchor, yellow fixed-0.5 m normal arrow (manifold normal, A->B). Toggle
them in the left panel (`Contact points` default on, `Contact normals` default off).

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera |
| Left panel | Scene selector, play/pause, single step, speed (1x / 0.5x / 0.25x), gravity slider, display toggles, reset, quit |
| Scene parameters | Scene-specific controls: FallingBall -> restitution slider / "Drop again"; SphereStack -> per-sphere y/vy / "Reset stack"; BoxStack -> per-layer y/tilt, contacts/max penetration, "Reset tower", "Drop impact ball" |
| Right panel (Status) | FPS, frame time, physics step time, body/shape/awake counts, contact count, max penetration (mm), gravity |

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI (`--demo`, `--smoke`, `--hidden`), demo registry, fixed-timestep loop, engine-level overlays, smoke summary |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP |
| `DebugUI.*` | Left control panel + right status panel (incl. contact/penetration stats) |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) |
| `demos/FallingBall.*` | Ground-plane drop scene with live restitution |
| `demos/SphereStack.*` | Three-sphere stack, the Week 3 Sequential Impulse acceptance scene |
| `demos/BoxStack.*` | Ten-box tower, the Week 4 polygon SAT/clipping acceptance scene |

Adding a scene: implement `p2::Demo` in `testbed/demos/<Name>.cpp` (the CMake glob
picks it up) and register it in the `demos` vector in `main.cpp`; `--demo <name>`
then selects it by its `Name()` (spaces/case ignored).
