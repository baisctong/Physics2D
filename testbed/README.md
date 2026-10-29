# Physics2D Testbed (Layer 2)

Interactive visual debugger for `physics2d_core`. Week 1 built the shell (GLFW
window + OpenGL 3.3 core context + Dear ImGui docking UI, batched line renderer,
camera). Week 2 added shapes, the analytic circle/plane narrow phase, contact
visualisation and the ground-plane scene. Week 3 added the Sequential Impulse
velocity solver, the stacked-sphere scene and `--demo` scene selection for
automated smoke runs. Week 4 added polygon SAT + clipping and the ten-box tower.
Week 5 added the live **Solver Settings** panel (iterations, warm starting,
split impulse, gravity, material defaults), the **Contacts** inspector, Step
timing / contact-point counters, the ten-box tower's acceptance readout and the
**FrictionRamp** scene. Week 6 added the pluggable broad phase (**Auto / None /
BruteForce / SAP** dropdown with live proxy + candidate-pair counts and the
bench-derived Auto dispatch), `testbed/SceneBuilders.h`, auto camera framing,
**Pyramid200** and **StressBenchmark**.
**Week 7** adds islands and whole-island sleeping: the **Dominoes** scene (chain
wake from sleep, no domino skipped) and the **IslandSleep500** scene (500 boxes
that fall asleep, with a real awake-vs-asleep Step-time comparison), islands /
awake / sleeping in the Status panel, `enableSleeping` / `useIslandSolving` /
"Wake all bodies" in Solver Settings, keyboard playback shortcuts (Space, `.`,
1/2/3) and the full **Contacts** inspector detail (normal arrow, contact points,
per-point accumulated impulses with bars and Coulomb-cone usage).

> Layer rule: this directory is the only place allowed to include GLFW / OpenGL /
> Dear ImGui. `physics2d_core` stays headless (see `docs/architecture.md`).

## Run

```powershell
# interactive window (1280x800, resizable), FallingBall is the default scene
build/debug/testbed/physics2d_testbed.exe
build/debug/testbed/physics2d_testbed.exe --demo SphereStack
build/debug/testbed/physics2d_testbed.exe --demo BoxStack
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp
build/debug/testbed/physics2d_testbed.exe --demo Dominoes
build/debug/testbed/physics2d_testbed.exe --demo IslandSleep500
build/debug/testbed/physics2d_testbed.exe --demo Pyramid200
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark

# automated smoke runs (deterministic frames, hidden window).
# With scripts/build.ps1 -BuildDir build/testbed7 the binary lands in
# build/testbed7/testbed/ instead.
build/debug/testbed/physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden
build/debug/testbed/physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo Dominoes --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo Dominoes --smoke 600 --hidden
build/debug/testbed/physics2d_testbed.exe --demo IslandSleep500 --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo Pyramid200 --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden

# broad-phase A/B runs (same physics, different candidate-pair counts)
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase sap --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase bf --smoke 300 --hidden
build/debug/testbed/physics2d_testbed.exe --demo StressBenchmark --broadphase none --smoke 300 --hidden
```

`--demo NAME` accepts `FallingBall`, `SphereStack`, `BoxStack`, `FrictionRamp`,
`Dominoes`, `IslandSleep500`, `Pyramid200` or `StressBenchmark` (case-insensitive;
spaces, underscores and hyphens are ignored, so `island sleep 500` also works). An
unknown name prints the available scene list and exits with code **2** - the smoke
harness uses that to distinguish a typo from a physics/render failure.

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
[smoke] demo=BoxStack frames=300 physics_steps=300 bodies=11 shapes=11 islands=1 awake=10 sleeping=0 contacts=10 contact_points=20 max_penetration_mm=3.763 max_speed_mps=0.031 max_tilt_deg=0.13 max_center_y=9.478 broadphase=BruteForce candidate_pairs=19 proxies=11 avg_frame_ms=1.338
[smoke] solver velocity_iterations=8 position_iterations=3 warm_starting=1 warm_start_factor=1.00 split_impulse=1 split_impulse_beta=0.20 linear_slop_m=0.0050 sat_cache=1 gravity_y=-10.0 enable_sleeping=1 use_island_solving=1
[smoke] body='ground' ...
...
[smoke] contact 'ground'|'box_0' points=2 penetration=3.310mm normal=(0.000, 1.000) normal_impulse=1.667 tangent_impulse=0.001
```

Week 7 scenes add their own summary line through `Demo::PrintSmokeSummary`:

```text
[smoke] dominoes total=20 woken_after_sleep=20 fallen=20 chain_reach=20 first_skipped=-1 awake=20 sleeping=0 islands=5 pushed=1
[smoke] islandsleep500 boxes=500 islands=100 awake=0 sleeping=500 first_all_asleep_frame=105 early_avg_step_ms=4.5533 early_frames=60 awake_avg_step_ms=4.5587 awake_frames=94 asleep_avg_step_ms=2.9522 asleep_frames=196 speedup=1.54x
```

The one-line `[smoke] demo=...` summary is the machine-checkable verdict:
`bodies / shapes`, `islands / awake / sleeping` (Week 7 World statistics),
`contacts` (touching manifolds), `contact_points`, `max_penetration_mm`, the body
aggregates `max_speed_mps` / `max_tilt_deg` / `max_center_y`, the broad-phase
fields `broadphase=<installed Name()>` `candidate_pairs=<N>` `proxies=<N>`, and
`avg_frame_ms`. The `[smoke] solver ...` line records the configuration the run
actually used (including the Week 7 `enable_sleeping` / `use_island_solving`
flags); the per-body / per-contact lines carry final positions, velocities, tilt
angles and the accumulated normal/tangent impulses.

## Islands & sleeping (Week 7)

The engine solves per island and sleeps whole islands atomically. The testbed
exposes the state in two places:

- **Status panel**: `Islands: N  Awake: N  Sleeping: N` (island count from the
  partition built by the last Step; sleeping counts dynamic bodies only).
- **Solver Settings -> Islands & sleeping**: `Use island solving` and
  `Enable sleeping` checkboxes (written back through `World::SetSolverSettings`,
  effective on the next Step), the same three counters, and a **Wake all bodies**
  button that calls `World::WakeBody` on every dynamic body (whole-island wake).

Measured behaviour (this machine, **debug** build, `--hidden`, one fixed step per
rendered frame; `Step ms` is the host loop's raw `std::chrono` average around
`World::Step`, not wall-clock frame time):

| Scene / window | What is measured | Value |
| --- | --- | --- |
| `IslandSleep500`, early window (frames 30-90) | Step ms while the scene is awake/settling | **~4.55-4.90 ms** |
| `IslandSleep500`, all awake frames after warm-up | Step ms while awake | **~4.56-4.91 ms** (94 frames) |
| `IslandSleep500`, all-asleep window | Step ms after every box sleeps | **~2.93-3.23 ms** (196-496 frames) |
| `IslandSleep500` | sleeping speedup (awake-average / asleep-average) | **1.51-1.59x** |
| `IslandSleep500` | first all-asleep frame | **105** (t = 1.75 s) |
| `IslandSleep500` | final state | islands=100, awake=0, sleeping=500, contacts=500 |

(The 300-step runs give early ~4.55-4.90 ms / awake ~4.56-4.91 ms / asleep
~2.95-3.23 ms / 1.51-1.56x depending on machine noise; the 600-step run gives
4.65 / 4.65 / 2.93 ms / 1.59x - the longer run has more asleep samples so its
asleep window includes more of the true steady state.)

Why the win is ~1.6x and not 5x: a sleeping island still keeps its broad-phase
proxies and its contacts are reused (that is what makes whole-island wake cheap),
so the broad-phase sweep, island build and contact list maintenance still run;
only the narrow phase + solver + integration for those islands are skipped. The
numbers are debug-build numbers for reproducibility - the point is the relative
drop, not an absolute production figure.

`Dominoes` is the wake-propagation proof. The 20-dominio row goes to sleep
(singleton islands), then the scene pushes domino 0 at t = 1.0 s
(frame 60 in smoke):

| Run | woken after sleep | fallen | chain reach | first skipped | awake / sleeping | islands |
| --- | --- | --- | --- | --- | --- | --- |
| `--smoke 300` (chain still travelling) | 10 | 9 | 9 | 9 (in progress) | 10 / 10 | 12 |
| `--smoke 600` | **20 / 20** | **20** | **20** | **-1 (none)** | 20 / 0 | 5 |

`chain_reach` walks the fallen prefix from index 0: if one domino stayed upright
while the wave passed, it would show up as `first_skipped >= 0`. The 600-step run
shows 20/20 woken after sleep, 20/20 fallen, no skipped domino. (The 300-step run
is mid-wave and the 600-step run's collapsed pile is still settling - `awake=20`
there is the pile, not a failed wake; the chain metrics are the verdict.)

Domino shape note (measured constraint from core-dev, reproduced here): 0.12 m
thin dominoes keep a ~0.015 m/s residual jitter spike, just above
`kSleepLinearTolerance = 0.01 m/s`, which re-arms `sleepTime` every 4-6 frames
and means a resting row **never** sleeps. The scene therefore uses the measured
sleepable shape **0.30 x 1.0 m** (width:height 1:3.3), spacing 1.0 m and the
core-dev-measured 0.25 N*s push impulse; the row sleeps before the step-60 push
and the chain then validates wake-from-sleep. This is a scene-geometry choice,
not a physics-settings tweak.

Honest note: not every old scene is asleep by the 300-step smoke snapshot.
`FallingBall` (1 body), `SphereStack` (3 bodies) and `FrictionRamp` (1 body)
sleep; the 10-box `BoxStack` tower's solver residual decays slowly (max speed
0.031 m/s at 300 steps, 0.007 m/s at 600 steps, still awake at 600 because the
0.5 s qualification window re-armed, `awake=0 sleeping=10` at 900 steps);
`Pyramid200` and `StressBenchmark` stay awake because their loaded piles keep
creeping above the 0.01 m/s tolerance. This is the solver residual at the default
iterations, reported rather than tuned away.

### Playback: pause, single step, slow motion

| Control | How |
| --- | --- |
| Pause / play | `Pause` button or `Space` |
| Single step | `Step` button or `.` (only while paused; advances exactly one fixed 1/60 s step so manifolds/contact points/impulses can be inspected frame by frame) |
| Slow motion | `1x` / `0.5x` / `0.25x` radio buttons or keys `1` / `2` / `3` |
| Camera | middle-drag pan, wheel zoom, `R` reset view |

Keyboard shortcuts are edge-triggered and suppressed while ImGui owns the
keyboard (e.g. typing in the ImGui demo window), so they never fight a panel.
Single-step uses the same fixed dt as normal play, so the sampled physics state is
identical to a normally-stepped run at that step count.

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
taller/wider scenes (`BoxStack` when it nearly fills the height, `Dominoes` when
long, `IslandSleep500`, `Pyramid200`, `StressBenchmark`) are auto-framed.

## Scenes

| Scene | Description |
| --- | --- |
| `FallingBall` (default) | Dynamic circle (r=0.5, density 1) dropped from y=4 onto a static ground plane at y=0. Restitution slider (0..1) is solved by the Sequential Impulse solver: e=0 lands and stops, e>0 bounces with the expected energy loss. Restitution is applied only above `kRestitutionVelocityThreshold` (1 m/s), so resting contacts stay quiet. |
| `SphereStack` | Three dynamic circles (r=0.5, density 1, restitution 0) stacked exactly tangent at y=0.5/1.5/2.5 on the same ground plane. A standing stack with small jitter is a pass; a collapsed stack is a visible physics failure. No auto-reset - the result is the measurement. |
| `BoxStack` | Ten dynamic 1x1 m boxes (half extents 0.5, density 1, friction 0.3, restitution 0) stacked exactly at y=0.5..9.5, drawn as wireframes from the polygon vertices stored in the shape. Week 5 acceptance: the tower stands (11 bodies / 10 contacts / max penetration 3.8 mm). Optional "Drop impact ball" adds a dynamic circle above the tower (circle-vs-polygon path). Reset rebuilds the tower and keeps the World's solver/material/broad-phase settings. |
| `FrictionRamp` | A 1x1 m box on an inclined static plane; panel sliders for the incline angle theta (0..45 deg) and the friction mu (0..1), both applied live through `World::SetTransform` / `World::SetFriction` (no body rebuild). The panel shows mu vs tan(theta), the theoretical prediction (STATIC iff mu > tan(theta)) and the measured drift/speed. The plane is an infinite half-space, so a sliding box keeps sliding down; "Reset block" re-seats it. |
| `Dominoes` | 20 upright dominoes (0.30 x 1.0 m - the measured sleepable shape, see below - spacing 1.0 m) on the ground plane, with no initial box-box contacts. The row settles and falls asleep. At t=1.0 s the scene auto-pushes domino 0 (also a "Push first" button) with a 0.25 N*s impulse near its top; the panel reports islands / awake / sleeping / contacts / max speed, "woken after sleep", fallen count and **chain reach** (a skipped upright domino would show as a gap). The 600-step smoke run ends with 20/20 woken-after-sleep and 20/20 fallen, chain reach 20, first skipped none. Count slider (10..40) + Rebuild; Reset. |
| `IslandSleep500` | 500 1x1 m boxes in 100 five-box towers on one ground plane (100 islands, 0.25 m gaps between towers so the broad-phase proximity wake cannot cascade). Five-box towers settle below the sleep tolerances quickly (measured: whole scene asleep at frame 105 = 1.75 s; the 10-box tower needs ~step 859 per core-dev's measurement, which is why the scene uses 5-box towers), and the panel/smoke summary print the **measured** awake-window vs asleep-window Step ms and their ratio. "Wake all" re-runs the asleep -> awake transition without rebuilding; Reset. |
| `Pyramid200` | A perfect 2D box pyramid: layer k holds `baseCount - k` 1x1 m boxes centred horizontally. The name keeps the design-document "~200 OBB" target, but the shipped default is **16 layers = 136 boxes** because at the engine's default 8 velocity iterations the larger pyramids sink (measured table below); the panel slider (4..24 layers) + Rebuild reproduce any size. Panel shows bodies/contacts/max penetration/max tilt/top height plus a standing verdict. Ground plane at y=0; no randomness. |
| `StressBenchmark` | 400 random-size OBBs (seed 42, sizes 0.2-1.0 m, random angles and initial velocity <= 4 m/s) dropped into a 36 m wide field, for broad-phase comparison and solver stress. Panel shows bodies/contacts/candidate pairs/proxies/Step ms/FPS and the installed broad phase; box colour encodes speed. The count slider (50..800) + Rebuild regenerates the deterministic field. Switching the broad phase does not rebuild the scene. |

All scenes use the shared engine-level contact overlay: red ring+cross at every
manifold anchor, yellow fixed-0.5 m normal arrow (manifold normal, A->B). Toggle
them in the Solver Settings panel (`Contact points` default on, `Contact normals`
default off). The contact highlighted from the Contacts panel is drawn cyan with a
longer normal regardless of those two switches. `Body AABBs` draws the Week 6
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
(These are Week 6 measurements; the Week 7 island/sleep build reproduces the same
default state - the pyramid is a single awake island whose residual is above the
sleep tolerance.)

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
| Testbed Controls (top left) | Scene selector, play/pause, single step, reset, speed (1x / 0.5x / 0.25x), quit, and the selected scene's own parameters (`Demo::Ui`). Keyboard shortcut hint line; Week 7 added the Dominoes / IslandSleep500 scene controls here. |
| Contacts (bottom left) | Live table of touching contacts: pair names, point count, penetration (mm), normal (A->B) and the accumulated normal/tangent impulses. Click a row to highlight it; the scene then draws its point (cyan ring) and normal (1 m arrow), and the panel shows the Week 7 detail: per-manifold-point **accumulated normal impulse bar**, **tangent impulse bar with Coulomb-cone usage** (|accT| / (mu*accN), 0% = sticking, 100% = sliding) and the contact normal/mu/depth. Click again or "Clear highlight" to drop it. Rows are identified by body pair, not list index, because the contact vector is rebuilt every step. |
| Status (top right) | FPS, frame time, smoothed Step time and steps/frame, scene, body/shape counts, **islands / awake / sleeping** (Week 7), contact and contact-point counts, max penetration, broad-phase installed name + mode + proxy/candidate-pair counts, gravity, solver iterations/warm/split and sleeping/island-solve flags, fixed dt, pause/speed state. |
| Solver Settings (bottom right) | Live `World::SetSolverSettings` / `World::SetGravity` controls: velocity and position iterations (0..40), warm starting on/off + factor (0..1), split impulse on/off + beta (0..1) + slop (0..20 mm), Baumgarte (used when split impulse is off), linear slop, max linear correction, restitution threshold, SAT cache on/off, **islands & sleeping (enable sleeping / use island solving / cycles + "Wake all bodies")**, gravity Y (-30..0), material defaults (friction / restitution) with "Apply to current scene", the broad-phase dropdown (Auto / None / BruteForce / SAP) with live counts, and the display toggles (axes, AABBs, contact points, contact normals, velocity arrows, ImGui demo window). Every change takes effect on the next fixed step; no restart and no body rebuild. |

Material defaults are applied automatically to every body when a scene is built
(initial scene and scene switch), which is how "set friction to 0 and watch the
tower collapse" works without touching code. A demo's own `Reset()` deliberately
keeps the material it has, so e.g. the FallingBall restitution choice survives
"Drop again". The broad-phase and solver-settings state behave the same way.

## Controls

| Input | Action |
| --- | --- |
| Middle mouse drag | Pan the camera |
| Mouse wheel | Zoom around the cursor |
| `R` | Reset the camera to the default origin-centred 1x view (does not reframe the scene) |
| `Space` | Pause / play |
| `.` | Single step (only while paused; one fixed 1/60 s step per press) |
| `1` / `2` / `3` | Playback speed 1x / 0.5x / 0.25x |
| Left/right panels | Playback, solver/broad-phase/material/island-sleep settings, contact inspector with impulse bars, status |
| Scene parameters | Scene-specific controls: FallingBall -> restitution slider / "Drop again"; SphereStack -> per-sphere y/vy / "Reset stack"; BoxStack -> contacts/max penetration/verdict, per-layer y/tilt, "Reset tower", "Drop impact ball"; FrictionRamp -> theta/mu sliders, prediction/measured readout, "Reset block"; Dominoes -> push/fallen/chain-reach readout, count slider + Rebuild, "Push first", "Reset"; IslandSleep500 -> islands/awake/sleeping + awake-vs-asleep Step ms, "Wake all", "Reset"; Pyramid200 -> layer slider + Rebuild, standing verdict, "Reset pyramid"; StressBenchmark -> box-count slider + Rebuild, broad-phase readout, "Reset field" |

Keyboard shortcuts are edge-triggered and ignored while ImGui owns the keyboard
(text fields, ImGui demo window), exactly like the camera keys. The UI buttons
remain the discoverable path.

## Files

| File | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL/ImGui lifecycle, CLI (`--demo`, `--broadphase`, `--smoke`, `--hidden`), demo registry, fixed-timestep loop, keyboard shortcuts, engine-level overlays (contacts incl. highlight, axes, fat AABBs, velocity arrows), material-default application, broad-phase policy after scene builds, auto camera framing, smoke summary (incl. island/sleep fields and per-demo summary hook) |
| `GLLoader.h` | Minimal OpenGL 3.3 core loader (`glfwGetProcAddress`, no glad/glew) |
| `Renderer.*` | One dynamic VBO + GL_LINES batch: circles, polygons, AABBs, arrows, points, axes |
| `Camera.*` | World <-> screen mapping, `kPixelsPerMeter = 40`, pan/zoom/reset, column-major MVP, `FocusOnWorld` auto-framing |
| `DebugUI.*` | Testbed Controls, Solver Settings (incl. broad-phase and islands/sleeping sections), Contacts inspector (incl. impulse bars) and Status panels |
| `Demo.h` | Scene interface (`Name/Setup/Update/Draw/Ui/Reset`) plus the optional `SetFrameStats` host-timing hook and `PrintSmokeSummary` scene-specific smoke hook (Weeks 6/7) |
| `SceneBuilders.h` | Header-only shared scene construction (ground plane, box stack, perfect pyramid, deterministic random box field, domino chain, box grid); testbed-only - `tests/regression` stays core-only and does not include it |
| `demos/FallingBall.*` | Ground-plane drop scene with live restitution |
| `demos/SphereStack.*` | Three-sphere stack, the Week 3 Sequential Impulse acceptance scene |
| `demos/BoxStack.*` | Ten-box tower, the Week 5 friction/warm-start/split-impulse acceptance scene |
| `demos/FrictionRamp.*` | Adjustable incline + friction scene: mu vs tan(theta), static-vs-sliding prediction and measurement |
| `demos/Dominoes.*` | Domino chain: whole-island wake propagation, no skipped domino, auto-push at t=1 s in smoke runs |
| `demos/IslandSleep500.*` | 500 boxes in 100 five-box towers; island count and the measured awake-vs-asleep Step-time comparison |
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
**changing a header alone may not trigger a rebuild** - use
`scripts/build.ps1 -Clean` (or delete the affected build directory) after editing a
header, or touch the dependent `.cpp`, before trusting an incremental build. The
canonical clean build in the Lead's verification run is unaffected.
