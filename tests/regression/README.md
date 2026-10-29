# tests/regression — headless determinism regression (Week 6, P16)

`physics2d_regression` is a **core-only** executable (links `physics2d_core`, never
GLFW/ImGui/OpenGL) that builds one fixed scene, advances it 600 fixed timesteps and
folds the final state into FNV-1a hashes. It is the Week 6 determinism harness
(Section 7.3 / P16) and the seed for the Week 6-7 regression tooling.

## Files

| File | Purpose |
| --- | --- |
| `regression_main.cpp` | scene builder + 600 frames + hashing + golden file I/O + CLI |
| `golden/default_600.txt` | checked-in baseline: the compared **weak** hash for the default scene |
| `cases/` | reserved (Week 7): parameterised scene description files |
| `README.md` | this contract |

CMake target `physics2d_regression` is registered in `tests/CMakeLists.txt`; the
CTest entry `regression_default` (LABEL `regression`) runs
`physics2d_regression --check .../golden/default_600.txt`, so a hash drift fails CI.

## Fixed scene (must stay in sync with `regression_main.cpp`)

* static ground plane at `y = 0`, gravity `(0, -10)`, fixed step `1/60 s`;
* 30 x 1 m boxes stacked at `x = 0`, `y = 0.5 + i`;
* 20 circles, radius in `[0.2, 0.45]`, positions from the harness's own 3-line LCG
  (seed `20261029`, high 24 bits) — deliberately **no `<random>`**, whose sequence
  is not portable across standard libraries;
* 51 bodies total, 600 frames.
* Measured on the current solver: 83 contacts at the final frame (informational —
  the counts are folded into the hash, so a lost body/contact is a regression).

## Hash contract

Two 64-bit FNV-1a hashes are computed over the **final** state only:

| Value | Meaning | Gated? |
| --- | --- | --- |
| `hash` | **weak hash**: each value is rounded to a `1e-5` quantum (0.01 mm / 0.00057 deg) before hashing, so sub-quantum cross-compiler float noise does not flip it | **yes** — the golden file pins it, `--check` compares it |
| `exact` | raw IEEE-754 bit-pattern hash of the same values | no — printed for local bit-exact debugging only; FMA contraction / libm differences between compilers can change the last bits |

The hashed state is the hot body state (position, rotation as `cos/sin` so the hash
does not depend on the `atan2` implementation, linear/angular velocity, the
split-impulse pseudo velocities) plus the live body count and contact count.
Hashing every frame would make the weak hash diverge on the first quantum-boundary
flip even when the physics is equivalent (the simulation is chaotic), which is why
the final state is the cross-platform contract.

## Usage and exit codes

Run from the repository root after building the target (paths are examples):

```powershell
# print the hash line, exit 0
.\build\core6\tests\physics2d_regression.exe

# write a golden baseline, exit 0
.\build\core6\tests\physics2d_regression.exe --write-golden tests\regression\golden\default_600.txt

# verify against a golden baseline: exit 0 match / 1 hash mismatch / 2 unreadable or invalid file
.\build\core6\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# usage text, exit 0
.\build\core6\tests\physics2d_regression.exe --help
```

Exit codes: `0` OK/match, `1` weak-hash mismatch, `2` bad arguments or unreadable
file. The exact hash is never part of the pass/fail decision.

## Golden policy (when to update)

The golden is a **physical-behaviour baseline**, not a formatting artefact:

1. Regenerate only when the physics or the scene **intentionally** changes
   (solver change, broad-phase default change that is supposed to be
   physics-equivalent, scene/seed/frame count edit).
2. A pure refactor (e.g. the `BodySim`/`BodyState` hot/cold split, switching the
   default broad phase while the canonical pair set is unchanged) **must not**
   change the weak hash; if it does, that is a bug to investigate, not a golden
   update.
3. Every golden change must be explained in the commit message / `docs/devlog.md`
   with the old and new hash. Never hand-edit the hash.
4. Regeneration command (canonical debug build, from the repository root):

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/core6 -NoTestbed -Target physics2d_regression
   .\build\core6\tests\physics2d_regression.exe --write-golden tests\regression\golden\default_600.txt
   .\build\core6\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
   ```

   The hash is computed from the quantised final state, so it only depends on the
   physics (not on the build directory or timing).

## Reserved (Week 7+)

* `cases/*.yaml`: parameterised scenes (seed, rows, shape mix) instead of the one
  hard-coded scene;
* CSV/JSON state export for external analysis;
* the full exit-code table of the design document (NaN / penetration-over-threshold
  gates). The Week 6 skeleton deliberately implements only
  `0` match / `1` mismatch / `2` bad input; the deeper checks belong to W7 when the
  scene becomes parameterised.
