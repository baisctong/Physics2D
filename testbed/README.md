# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 built the shell (GLFW
window + OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer,
camera). Week 2 added shapes, the analytic circle/plane narrow phase, contact
visualisation and the ground-plane scene. Week 3 added the Sequential Impulse
velocity solver, the stacked-sphere scene and `--demo` scene selection for
automated smoke runs. Week 4 added polygon SAT + clipping and the ten-box tower.
Week 5 adds the live **Solver Settings** panel (iterations, warm starting,
split impulse, gravity, material defaults), the **Contacts** inspector (per-pair
normal, penetration and accumulated impulses, click to highlight), Step timing /
contact-point / island counters in the status panel, the ten-box tower's
"friction + warm starting + split impulse: it must stand now" acceptance readout,
and the **FrictionRamp** scene (mu vs tan(theta)).

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable), FallingBall is the default scene
build/debug/testbed/physics2d_testbed.exe
build/debug/testbed/physics2d_testbed.exe --demo SphereStack
build/debug/testbed/physics2d_testbed.exe --demo BoxStack
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp

# automated smoke runs (deterministic frames, hidden window).
# With scripts/build.ps1 -BuildDir build/testbed5 the binary lands in
# build/testbed5/testbed/ instead.
build/debug/testbed/physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden
build/debug/testbed/physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
```

`--demo NAME` accepts `FallingBall`, `SphereStack`, `BoxStack` or `FrictionRamp`
(case-insensitive; spaces, underscores and hyphens are ignored, so `sphere stack`
also works). An unknown name prints the available scene list and exits with code
**2** - the smoke harness uses that to distinguish a typo from a physics/render
failure. `--hidden` keeps the window invisible but still creates the GL context
(exit code 2 if no usable OpenGL 3.3 environment exists); `--smoke N` feeds the
fixed-step accumulator exactly `1/60 s` per frame, so `frames == physics_steps` is
deterministic and independent of machine speed.

Output of a successful smoke run (observed on the dev machine; timings vary per
machine):

```text
[testbed] GLFW 3.4.0 ...
[testbed] OpenGL 3.3.0 ... (GPU / driver name)
[SphereStack] static ground plane y=0 + 3 dynamic circles r=0.5 at y=0.5/1.5/2.5, ...
[smoke] demo=SphereStack frames=300 physics_steps=300 bodies=4 shapes=4 contacts=3 contact_points=3 max_penetration_mm=3.063 avg_frame_ms=1.164
[smoke] solver velocity_iterations=8 position_iterations=3 warm_starting=1 warm_start_factor=1.00 split_impulse=1 split_impulse_beta=0.20 linear_slop_m=0.0050 sat_cache=1 gravity_y=-10.0
[smoke] body='ground' position=(0.000, 0.000) velocity=(0.000, 0.000) angle=0.00deg
[smoke] body='sphere_0' position=(0.000, 0.500) velocity=(0.000, -0.000) angle=0.00deg
[smoke] body='sphere_1' position=(0.000, 1.500) velocity=(0.000, -0.000) angle=0.00deg
[smoke] body='sphere_2' position=(0.000, 2.496) velocity=(0.000, 0.000) angle=0.00deg
[smoke] contact 'ground'|'sphere_0' points=1 penetration=0.308mm normal=(0.000, 1.000) normal_impulse=0.393 tangent_impulse=0.000
[smoke] contact 'sphere_0'|'sphere_1' points=1 penetration=0.158mm normal=(0.000, 1.000) normal_impulse=0.262 tangent_impulse=0.000
[smoke] contact 'sphere_1'|'sphere_2' points=1 penetration=3.063mm normal=(0.000, 1.000) normal_impulse=0.131 tangent_impulse=0.000
```

The one-line `[smoke] demo=...` summary is the machine-checkable verdict:
`bodies / shapes` (alive counts), `contacts` (touching manifolds), `contact_points`
(individual manifold anchors), `max_penetration_mm` (milestone metric) and
`avg_frame_ms`. The next `[smoke] solver ...` line records the configuration the run
actually used, and the per-body / per-contact lines carry final positions,
velocities, tilt angles and the accumulated normal/tangent impulses. The stack
stands: all three centres stay at their spawn heights (0.500 / 1.500 / 2.496 m) with
residual velocities at machine precision and three persistent contacts whose
penetration stays inside the 5 mm slop band.

### BoxStack (Week 5): the ten-box stability milestone

`--demo BoxStack --smoke 300 --hidden` is the Week 5 acceptance scene. With the
friction cone, warm starting and the split-impulse position layer the ten 1 m boxes
must stand using the shipped defaults (8 velocity / 3 position iterations, friction
0.3 from `BodyDef`, restitution 0) - nothing in the scene is tuned:

| Check | Expected after 300 steps |
| --- | --- |
| bodies / shapes | 11 / 11 |
| contacts / contact points | 10 / 20 (box-box and box-ground face contacts) |
| max penetration | **< 0.01 m** (`max_penetration_mm < 10`) |
| box centres | `y ≈ 0.5 .. 9.5`, tilt near 0 deg |

Measured on the dev machine (debug build, `--demo BoxStack --smoke 600 --hidden`,
i.e. the 10 s milestone): `bodies=11 shapes=11 contacts=10 contact_points=20
max_penetration_mm=3.893 avg_frame_ms=1.321`; the box centres stayed at
`y = 0.497 .. 9.478` m with `|tilt| <= 0.21 deg`. The 300-step check shows the same
contact picture with max penetration 3.763 mm. Absolute numbers depend on the
build/machine; the `[smoke]` line is the source of truth.

The in-scene panel prints the same aggregate numbers plus a
`Verdict: STANDING (Week 5 acceptance met)` / `NOT STABLE yet` line and the per-layer
`y` / `tilt`, so a run can be judged without parsing the log. The old Week 4 outcome
(the tower collapsing with no friction / no warm starting) can be reproduced live by
opening the Solver Settings panel and dragging **Default friction** to 0, then
"Apply to current scene" - a controlled comparison instead of a tuned-away baseline.

Exit codes: `0` success, `1` failure/usage error, `2` no usable GL environment or
unknown `--demo` name.

## Scenes

| Scene | Description |
| --- | --- |
| `FallingBall` (default) | Dynamic circle (r=0.5, density 1) dropped from y=4 onto a static ground plane at y=0. Restitution slider (0..1) is solved by the Sequential Impulse solver: e=0 lands and stops, e>0 bounces with the expected energy loss. Restitution is applied only above `kRestitutionVelocityThreshold` (1 m/s), so resting contacts stay quiet. |
| `SphereStack` | Three dynamic circles (r=0.5, density 1, restitution 0) stacked exactly tangent at y=0.5/1.5/2.5 on the same ground plane. A standing stack with small jitter is a pass; a collapsed stack is a visible physics failure. No auto-reset - the result is the measurement. |
| `BoxStack` | Ten dynamic 1x1 m boxes (half extents 0.5, density 1, friction 0.3, restitution 0) stacked exactly at y=0.5..9.5, drawn as wireframes from the polygon vertices stored in the shape. Week 5 acceptance: the tower stands (see the milestone table above). Optional "Drop impact ball" adds a dynamic circle above the tower (circle-vs-polygon path). Reset rebuilds the tower and keeps the World's solver/material settings. |
| `FrictionRamp` | A 1x1 m box on an inclined static plane; panel sliders for the incline angle theta (0..45 deg) and the friction mu (0..1), both applied live through `World::SetTransform` / `World::SetFriction` (no body rebuild). The panel shows mu vs tan(theta), the theoretical prediction (STATIC iff mu > tan(theta)) and the measured drift/speed, so theory and solver behaviour are visible side by side. Both the ramp and the box get the same friction value, because contacts combine them as sqrt(fA*fB). The plane is an infinite half-space, so a sliding box keeps sliding down; "Reset block" re-seats it. |

All scenes use the shared engine-level contact overlay: red ring+cross at every
manifold anchor, yellow fixed-0.5 m normal arrow (manifold normal, A->B). Toggle
them in the Solver Settings panel (`Contact points` default on, `Contact normals`
default off). The contact highlighted from the Contacts panel is drawn cyan with a
longer normal regardless of those two switches.

## Panels

| Panel | Contents |
| --- | --- |
| Testbed Controls (top left) | Scene selector, play/pause, single step, reset, speed (1x / 0.5x / 0.25x), quit, and the selected scene's own parameters (`Demo::Ui`). |
| Contacts (bottom left) | Live table of touching contacts: pair names, point count, penetration (mm), normal (A->B) and the accumulated normal/tangent impulses. Click a row to highlight that contact in the scene (cyan point + 1 m normal); click again or use "Clear highlight" to drop it. Rows are identified by body pair, not by list index, because the contact vector is rebuilt every step. |
| Status (top right) | FPS, frame time, smoothed Step time and steps/frame, scene, body/shape counts, awake count, island count (0 until Week 7), contact and contact-point counts, max penetration, gravity, the active solver settings, fixed dt, pause/speed state. |
| Solver Settings (bottom right) | Live `World::SetSolverSettings` / `World::SetGravity` controls: velocity and position iterations (0..40), warm starting on/off + factor (0..1), split impulse on/off + beta (0..1) + slop (0..20 mm), Baumgarte (used when split impulse is off), linear slop, max linear correction, restitution threshold, SAT cache on/off, gravity Y (-30..0), material defaults (friction / restitution) with an "Apply to current scene" button, and the display toggles (axes, AABBs, contact points, contact normals, velocity arrows, ImGui demo window). Every slider takes effect on the next fixed step; no restart and no body rebuild. |

Material defaults are applied automatically to every body when a scene is built
(initial scene and scene switch), which is how "set friction to 0 and watch the
tower collapse" works without touching code. A demo's own `Reset()` deliberately
keeps the material it has, so e.g. the FallingBall restitution choice survives
"Drop again".

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera |
| Left/right panels | Playback, solver/material settings, contact inspector, status |
| Scene parameters | Scene-specific controls: FallingBall -> restitution slider / "Drop again"; SphereStack -> per-sphere y/vy / "Reset stack"; BoxStack -> contacts/max penetration/verdict, per-layer y/tilt, "Reset tower", "Drop impact ball"; FrictionRamp -> theta/mu sliders, prediction/measured readout, "Reset block" |

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI (`--demo`, `--smoke`, `--hidden`), demo registry, fixed-timestep loop, engine-level overlays (contacts incl. highlight, axes, AABBs, velocity arrows), material-default application, smoke summary |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP |
| `DebugUI.*` | Testbed Controls, Solver Settings, Contacts inspector and Status panels (live solver settings, contact table + highlight, Step timing, contact/point/island counters) |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) |
| `demos/FallingBall.*` | Ground-plane drop scene with live restitution |
| `demos/SphereStack.*` | Three-sphere stack, the Week 3 Sequential Impulse acceptance scene |
| `demos/BoxStack.*` | Ten-box tower, the Week 5 friction/warm-start/split-impulse acceptance scene |
| `demos/FrictionRamp.*` | Adjustable incline + friction scene: mu vs tan(theta), static-vs-sliding prediction and measurement |

Adding a scene: implement `p2::Demo` in `testbed/demos/<Name>.cpp` (the CMake glob
picks it up) and register it in the `demos` vector in `main.cpp`; `--demo <name>`
then selects it by its `Name()` (spaces/case ignored).
