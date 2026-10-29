# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 built the shell (GLFW
window + OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer,
camera). Week 2 added shapes, the analytic circle/plane narrow phase, contact
visualisation and the ground-plane scene. Week 3 added the Sequential Impulse
velocity solver, the stacked-sphere scene and `--demo` scene selection for
automated smoke runs. Week 4 added polygon SAT + clipping and the ten-box tower.
Week 5 added the live **Solver Settings** panel (iterations, warm starting,
split impulse, gravity, material defaults), the **Contacts** inspector (per-pair
normal, penetration and accumulated impulses, click to highlight), Step timing /
contact-point / island counters in the status panel, the ten-box tower's
acceptance readout and the **FrictionRamp** scene (mu vs tan(theta)).
**Week 6** adds the pluggable broad phase: the **Auto / None / BruteForce / SAP**
dropdown with live proxy + candidate-pair counts, the bench-derived Auto dispatch
(BruteForce for small scenes, SAP at/above 32 bodies), `testbed/SceneBuilders.h`
for shared scene construction, auto camera framing for scenes larger than the
default view, and two new scenes - **Pyramid200** (a perfect box pyramid) and
**StressBenchmark** (400 random OBBs).

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable), FallingBall is the default scene
build/debug/testbed/physics2d_testbed.exe
build/debug/testbed/physics2d_testbed.exe --demo SphereStack
build/debug/testbed/physics2d_testbed.exe --demo BoxStack
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp
build/debug/testbed/physics2d_testbed.exe --demo Pyramid200
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark

# automated smoke runs (deterministic frames, hidden window).
# With scripts/build.ps1 -BuildDir build/testbed6 the binary lands in
# build/testbed6/testbed/ instead.
build/debug/testbed/physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden
build/debug/testbed/physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo Pyramid200 --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden

# broad-phase A/B runs (same physics, different candidate-pair counts)
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase sap --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase bf --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase none --smoke 300 --hidden
```

`--demo NAME` accepts `FallingBall`, `SphereStack`, `BoxStack`, `FrictionRamp`,
`Pyramid200` or `StressBenchmark` (case-insensitive; spaces, underscores and
hyphens are ignored, so `pyramid 200` also works). An unknown name prints the
available scene list and exits with code **2** - the smoke harness uses that to
distinguish a typo from a physics/render failure.

`--broadphase NAME` accepts `Auto` (default), `None`, `BruteForce` (`bf`/`brute`)
or `SAP` (same loose matching). An unknown value is a usage error (exit **1**).
`--hidden` keeps the window invisible but still creates the GL context (exit code
2 if no usable OpenGL 3.3 environment exists); `--smoke N` feeds the fixed-step
accumulator exactly `1/60 s` per frame, so `frames == physics_steps` is
deterministic and independent of machine speed and of the camera (auto-framing
never touches the simulation).

Output of a successful smoke run (observed on the dev machine; timings vary per
machine):

```text
[testbed] broad phase: Auto (BruteForce < 32 bodies, SAP >= 32; bench crossover)
[testbed] GLFW 3.4.0 ...
[testbed] OpenGL 3.3.0 ... (GPU / driver name)
[BoxStack] static ground plane y=0 + 10 dynamic boxes 1x1 m ...
[smoke] demo=BoxStack frames=300 physics_steps=300 bodies=11 shapes=11 contacts=10 contact_points=20 max_penetration_mm=3.763 max_speed_mps=0.031 max_tilt_deg=0.13 max_center_y=9.478 broadphase=BruteForce candidate_pairs=19 proxies=11 avg_frame_ms=1.219
[smoke] solver velocity_iterations=8 position_iterations=3 warm_starting=1 warm_start_factor=1.00 split_impulse=1 split_impulse_beta=0.20 linear_slop_m=0.0050 sat_cache=1 gravity_y=-10.0
[smoke] body='ground' position=(0.000, 0.000) velocity=(0.000, 0.000) angle=0.00deg
[smoke] body='box_0' position=(-0.001, 0.497) velocity=(0.000, 0.000) angle=-0.02deg
...
[smoke] contact 'ground'|'box_0' points=2 penetration=3.310mm normal=(0.000, 1.000) normal_impulse=1.667 tangent_impulse=0.001
...
```

The one-line `[smoke] demo=...` summary is the machine-checkable verdict:
`bodies / shapes` (alive counts), `contacts` (touching manifolds), `contact_points`
(individual anchors), `max_penetration_mm`, the body aggregates
`max_speed_mps` / `max_tilt_deg` / `max_center_y` (a stress-scene verdict cannot
be read off hundreds of per-body lines), the broad-phase fields
`broadphase=<installed Name()>` `candidate_pairs=<N>` `proxies=<N>`, and
`avg_frame_ms`. The next `[smoke] solver ...` line records the configuration the
run actually used; the per-body / per-contact lines carry final positions,
velocities, tilt angles and the accumulated normal/tangent impulses.

## Broad phase (Week 6)

The Solver Settings panel has a **Broad phase** section:

| Control | Meaning |
| --- | --- |
| `Auto (by body count)` | Default. After every scene build/reset the testbed installs `BruteForce` below **32** bodies and `SAP` at/above 32. |
| `None (O(N^2) fallback)` | `World::SetBroadPhase(nullptr)`: the built-in O(N^2) enumeration (the Week 2-5 path). Shows `proxies=0, candidate_pairs=0`. |
| `BruteForce` | The O(N^2) reference implementation, with proxy bookkeeping. |
| `SAP` | Sweep-and-prune endpoint array (the main Week 6 implementation). |

The dropdown also shows the installed implementation (`Name()`), proxy count and
candidate-pair count, plus the fat-AABB margin (`kAabbMargin = 0.1 m` from
`Core/Settings.h`). Switching is a runtime swap of World's broad-phase object -
bodies/contacts are never rebuilt, and the physics result must not change: World
canonicalises the candidate pairs before the narrow phase, so only the
`proxies`/`candidate_pairs` numbers differ between implementations.

The Auto threshold comes from the Release bench (seed42, 1000 frames, dev
machine; full table in the root README performance section): SAP beats
BruteForce from ~25-32 bodies on (N=20: 0.80x, N=50: 2.12x, N=200: 4.99x,
N=500: 5.91x), so 32 is the conservative side of the measured crossover. A
manual choice is an override: it survives scene switches until `Auto` is picked
again (the panel says `Manual override active`), so a scene switch never
silently undoes the user's choice.

Verified in this testbed (300 steps, `body`/`contact` smoke lines hashed): the
physics output is **bit-identical** for `sap`, `bf` and `none` on BoxStack
(11 bodies), Pyramid200 (137 bodies) and StressBenchmark (401 bodies), while
`candidate_pairs` changes (e.g. StressBenchmark: SAP 1687 pairs / 401 proxies,
None 0 / 0). This is the UI-side counterpart of core-dev's equivalence tests.

## Camera framing (Week 6)

`Camera::FocusOnWorld(bounds)` (and `Camera::GetDefaultHalfExtents()`) frame a
world-space AABB. `main.cpp` calls it **once per scene build/switch/reset**, never
per frame, so it cannot fight the user's pan/zoom; `R` restores the default
origin-centred view. Scene bounds are the union of finite bodies (the infinite
ground plane is skipped) with the fat AABB margin plus 15% padding, and the zoom
is clamped to `[0.2, 3.0]`. Scenes that already fit the conventional 32x20 m view
(`FallingBall`, `SphereStack`, `FrictionRamp`) keep/reset to the default camera;
taller/wider scenes (`BoxStack` when it nearly fills the height, `Pyramid200`,
`StressBenchmark`) are auto-framed.

## Scenes

| Scene | Description |
| --- | --- |
| `FallingBall` (default) | Dynamic circle (r=0.5, density 1) dropped from y=4 onto a static ground plane at y=0. Restitution slider (0..1) is solved by the Sequential Impulse solver: e=0 lands and stops, e>0 bounces with the expected energy loss. Restitution is applied only above `kRestitutionVelocityThreshold` (1 m/s), so resting contacts stay quiet. |
| `SphereStack` | Three dynamic circles (r=0.5, density 1, restitution 0) stacked exactly tangent at y=0.5/1.5/2.5 on the same ground plane. A standing stack with small jitter is a pass; a collapsed stack is a visible physics failure. No auto-reset - the result is the measurement. |
| `BoxStack` | Ten dynamic 1x1 m boxes (half extents 0.5, density 1, friction 0.3, restitution 0) stacked exactly at y=0.5..9.5, drawn as wireframes from the polygon vertices stored in the shape. Week 5 acceptance: the tower stands (11 bodies / 10 contacts / max penetration 3.8 mm). Optional "Drop impact ball" adds a dynamic circle above the tower (circle-vs-polygon path). Reset rebuilds the tower and keeps the World's solver/material/broad-phase settings. |
| `FrictionRamp` | A 1x1 m box on an inclined static plane; panel sliders for the incline angle theta (0..45 deg) and the friction mu (0..1), both applied live through `World::SetTransform` / `World::SetFriction` (no body rebuild). The panel shows mu vs tan(theta), the theoretical prediction (STATIC iff mu > tan(theta)) and the measured drift/speed, so theory and solver behaviour are visible side by side. The plane is an infinite half-space, so a sliding box keeps sliding down; "Reset block" re-seats it. |
| `Pyramid200` | A perfect 2D box pyramid: layer k holds `baseCount - k` 1x1 m boxes centred horizontally. The name keeps the design-document "~200 OBB" target, but the shipped default is **16 layers = 136 boxes** because at the engine's default 8 velocity iterations the larger pyramids sink (measured table below); the panel slider (4..24 layers) + Rebuild reproduce any size. Panel shows bodies/contacts/max penetration/max tilt/top height plus a standing verdict. Ground plane at y=0; no randomness. |
| `StressBenchmark` | 400 random-size OBBs (seed 42, sizes 0.2-1.0 m, random angles and initial velocity <= 4 m/s) dropped into a 36 m wide field, for broad-phase comparison and solver stress. Panel shows bodies/contacts/candidate pairs/proxies/Step ms/FPS and the installed broad phase; box colour encodes speed. The count slider (50..800) + Rebuild regenerates the deterministic field. Switching the broad phase does not rebuild the scene. |

All scenes use the shared engine-level contact overlay: red ring+cross at every
manifold anchor, yellow fixed-0.5 m normal arrow (manifold normal, A->B). Toggle
them in the Solver Settings panel (`Contact points` default on, `Contact normals`
default off). The contact highlighted from the Contacts panel is drawn cyan with a
longer normal regardless of those two switches. `Body AABBs` now draws the Week 6
**fat** AABB (tight bounds + 0.1 m margin), i.e. exactly what the broad phase sees.

### Pyramid200: measured stability vs size

Default engine settings (8 velocity / 3 position iterations), debug build,
`--demo Pyramid200 --smoke N --hidden`, no tuning. The max penetration of a
loaded pyramid is not the Week 5 10 mm single-column acceptance: the bottom rows
support up to 16 boxes of weight, so the position layer settles at a few tens of
mm. What matters for the scene is whether it **settles without collapsing or
sliding**.

| Base count | Boxes | Bodies | max pen mm @300 / @600 | max tilt deg @600 | top centre y (spawn) @600 | max speed m/s @600 | avg_frame_ms @600 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 20 | 210 | 211 | 85.7 / 100.5 | 6.02 | 19.057 (19.5) | 0.337 | 10.79 |
| 18 | 171 | 172 | 61.8 / 60.3 | 2.76 | 17.229 (17.5) | 0.253 | 8.78 |
| **16 (default)** | **136** | **137** | **41.9 / 29.7** | **1.36** | **15.341 (15.5)** | **0.060** | **6.98** |
| 14 | 105 | 106 | 28.5 / 21.4 | 0.96 | 13.381 (13.5) | 0.066 | 5.54 |

20 layers sinks ~0.44 m at the bottom rows and keeps creeping; 18 layers still
sinks ~0.27 m; 16 layers settles (speed 0.06 m/s) with ~0.16 m total sink;
14 layers is slightly stiffer but further from the intended ~200-box scale. 16 is
therefore the shipped default; the solver settings are never adjusted per size.
`--smoke 600` measured data (default): bodies=137 shapes=137 contacts=338
contact_points=642 max_penetration_mm=29.666 max_speed_mps=0.060 max_tilt_deg=1.36
max_center_y=15.341 broadphase=SAP candidate_pairs=496 proxies=137
avg_frame_ms=6.982.

### StressBenchmark: measured data (default 400 boxes, seed 42)

`--smoke 300 --hidden`: bodies=401 shapes=401 contacts=819 contact_points=1227
max_penetration_mm=178.151 max_speed_mps=0.500 max_tilt_deg=179.95
max_center_y=5.437 broadphase=SAP candidate_pairs=1687 proxies=401
avg_frame_ms=13.971. The 178 mm "penetration" is the dense pile at the bottom of
a 400-body random field under the default 8 iterations (a few boxes are stacked
on deep contacts), not a broad-phase error: the same run under BruteForce/None
produces identical physics (see the broad-phase section). The scene is a stress
benchmark, not a stability demo.

## Panels

| Panel | Contents |
| --- | --- |
| Testbed Controls (top left) | Scene selector, play/pause, single step, reset, speed (1x / 0.5x / 0.25x), quit, and the selected scene's own parameters (`Demo::Ui`). |
| Contacts (bottom left) | Live table of touching contacts: pair names, point count, penetration (mm), normal (A->B) and the accumulated normal/tangent impulses. Click a row to highlight that contact in the scene (cyan point + 1 m normal); click again or use "Clear highlight" to drop it. Rows are identified by body pair, not by list index, because the contact vector is rebuilt every step. |
| Status (top right) | FPS, frame time, smoothed Step time and steps/frame, scene, body/shape counts, awake count, island count (0 until Week 7), contact and contact-point counts, max penetration, broad-phase installed name + mode + proxy/candidate-pair counts, gravity, the active solver settings, fixed dt, pause/speed state. |
| Solver Settings (bottom right) | Live `World::SetSolverSettings` / `World::SetGravity` controls: velocity and position iterations (0..40), warm starting on/off + factor (0..1), split impulse on/off + beta (0..1) + slop (0..20 mm), Baumgarte (used when split impulse is off), linear slop, max linear correction, restitution threshold, SAT cache on/off, gravity Y (-30..0), material defaults (friction / restitution) with an "Apply to current scene" button, the broad-phase dropdown (Auto / None / BruteForce / SAP) with live counts, and the display toggles (axes, AABBs, contact points, contact normals, velocity arrows, ImGui demo window). Every slider takes effect on the next fixed step; no restart and no body rebuild. |

Material defaults are applied automatically to every body when a scene is built
(initial scene and scene switch), which is how "set friction to 0 and watch the
tower collapse" works without touching code. A demo's own `Reset()` deliberately
keeps the material it has, so e.g. the FallingBall restitution choice survives
"Drop again". The broad-phase mode behaves the same way: Auto re-dispatches after
every scene build, an explicit choice is kept until Auto is selected again.

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera to the default origin-centred 1x view (does not reframe the scene) |
| Left/right panels | Playback, solver/broad-phase/material settings, contact inspector, status |
| Scene parameters | Scene-specific controls: FallingBall -> restitution slider / "Drop again"; SphereStack -> per-sphere y/vy / "Reset stack"; BoxStack -> contacts/max penetration/verdict, per-layer y/tilt, "Reset tower", "Drop impact ball"; FrictionRamp -> theta/mu sliders, prediction/measured readout, "Reset block"; Pyramid200 -> layer slider + Rebuild, standing verdict, "Reset pyramid"; StressBenchmark -> box-count slider + Rebuild, broad-phase readout, "Reset field" |

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI (`--demo`, `--broadphase`, `--smoke`, `--hidden`), demo registry, fixed-timestep loop, engine-level overlays (contacts incl. highlight, axes, fat AABBs, velocity arrows), material-default application, broad-phase policy application after scene builds, auto camera framing, smoke summary |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP, `FocusOnWorld` auto-framing |
| `DebugUI.*` | Testbed Controls, Solver Settings (incl. broad-phase mode/dropdown), Contacts inspector and Status panels |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) plus the optional `SetFrameStats` host-timing hook (Week 6) |
| `SceneBuilders.h` | Header-only shared scene construction (ground plane, box stack, perfect pyramid, deterministic random box field); testbed-only - `tests/regression` stays core-only and does not include it |
| `demos/FallingBall.*` | Ground-plane drop scene with live restitution |
| `demos/SphereStack.*` | Three-sphere stack, the Week 3 Sequential Impulse acceptance scene |
| `demos/BoxStack.*` | Ten-box tower, the Week 5 friction/warm-start/split-impulse acceptance scene |
| `demos/FrictionRamp.*` | Adjustable incline + friction scene: mu vs tan(theta), static-vs-sliding prediction and measurement |
| `demos/Pyramid200.*` | Perfect box pyramid (default 136 boxes), configurable layer count, stability readout |
| `demos/StressBenchmark.*` | 400-box deterministic random field for broad-phase comparison / solver stress |

Adding a scene: implement `p2::Demo` in `testbed/demos/<Name>.cpp` (the CMake glob
picks it up) and register it in the `demos` vector in `main.cpp`; `--demo <name>`
then selects it by its `Name()` (spaces/case ignored). Build scenes with
`SceneBuilders.h` where possible so the demo code stays about physics, not
boilerplate.

## Build note (Windows, this machine)

MSVC here is localized, so its `/showIncludes` lines are printed as
`注意: 包含文件:` and Ninja does not parse them as dependencies. Consequence:
**changing a header alone may not trigger a rebuild** - touch the dependent `.cpp`
(or delete the affected object/`build/<dir>`) before trusting an incremental
build. The canonical clean build in the Lead's verification run is unaffected.
