# tests/regression — headless determinism regression (Week 6, P16)

`physics2d_regression` is a **core-only** executable (links `physics2d_core`, never
GLFW/ImGui/OpenGL) that builds one fixed scene, advances it 600 fixed timesteps and
folds the final state into FNV-1a hashes. It is the Week 6 determinism harness
(Section 7.3 / P16) and the seed for the Week 6-7 regression tooling.

## Files

| File | Purpose |
| --- | --- |
| `regression_main.cpp` | scene builder + 600 frames + hashing + golden file I/O + CLI |
| `golden/default_600.txt` | checked-in baseline for the **MSVC** toolchain (the compared weak hash) |
| `golden/default_600_gcc.txt` | checked-in baseline for the **GCC/libstdc++ / Linux** toolchain |
| `cases/` | reserved (Week 7): parameterised scene description files |
| `README.md` | this contract |

CMake target `physics2d_regression` is registered in `tests/CMakeLists.txt`; the
CTest entry `regression_default` (LABEL `regression`) runs
`physics2d_regression --check .../golden/<toolchain golden>`, so a hash drift fails
CI. `tests/CMakeLists.txt` picks the golden: `MSVC` → `default_600.txt`,
otherwise → `default_600_gcc.txt`.

## Fixed scene (must stay in sync with `regression_main.cpp`)

* static ground plane at `y = 0`, gravity `(0, -10)`, fixed step `1/60 s`;
* 30 x 1 m boxes stacked at `x = 0`, `y = 0.5 + i`;
* 20 circles, radius in `[0.2, 0.45]`, positions from the harness's own 3-line LCG
  (seed `20261029`, high 24 bits) — deliberately **no `<random>`**, whose sequence
  is not portable across standard libraries;
* 51 bodies total, 600 frames.
* every body is created with `allowSleep = false` on purpose: the regression scene
  keeps exercising continuous dynamics. Island sleeping has its own tests
  (`tests/unit/test_world_sleep.cpp`).
* Measured on the current solver: 83 contacts at the final frame on MSVC, 82 on
  GCC/Linux (informational — the counts are folded into the hash, so a lost
  body/contact is a regression along with the hash).

## Hash contract (and why there are two goldens)

Two 64-bit FNV-1a hashes are computed over the **final** state only:

| Value | Meaning | Gated? |
| --- | --- | --- |
| `hash` | **weak hash**: each value is rounded to a `1e-5` quantum (0.01 mm / 0.00057 deg) before hashing; sub-quantum noise does not flip it | **yes** — the toolchain golden pins it, `--check` compares it |
| `exact` | raw IEEE-754 bit-pattern hash of the same values | no — printed for local bit-exact debugging only; FMA contraction / libm differences between compilers can change the last bit |

The hashed state is the hot body state (position, rotation as `cos/sin` so the hash
does not depend on the `atan2` implementation, linear/angular velocity, the
split-impulse pseudo velocities) plus the live body count and contact count.

**The weak hash is stable within one toolchain but NOT across toolchains.** The Week 6
design assumed the `1e-5` quantisation would make it compiler portable; the first
remote Linux CI run disproved that: the same scene produced `588d30e70f9b4ddd` on
MSVC and `001d846118f05125` on GCC 13/libstdc++ (stable across Debug/Release/ASan on
Linux, so it is not random noise). Root cause: the 30-box tower collapses under the
default 8 iterations and the simulation is chaotic, so last-bit differences in libm
(`sqrt`/`cos`/`sin`/`atan2`) and FP contraction decisions get amplified over 600
steps. Both `-ffp-contract=off` (non-MSVC, see `cmake/CompilerWarnings.cmake`) and a
custom LCG scene generator are already in place; they reduce but cannot remove the
cross-toolchain difference.

Consequences:

* **one golden per toolchain family** (`default_600.txt` MSVC,
  `default_600_gcc.txt` GCC/libstdc++); both are regenerated deliberately when the
  physics/scene intentionally changes;
* cross-platform "same physics" is covered by the 386 unit tests, which assert
  bit-identical results *within the running toolchain* (three broad phases, three
  island solve modes, sleep/wake bookkeeping) and tolerance-based physics facts
  (penetration, restitution, momentum) — not by the cross-compiler hash;
* a **toolchain upgrade** (new GCC, new libstdc++, new MSVC) may legitimately change
  the weak hash; regenerate that toolchain's golden, write old/new + reason in the
  commit/devlog, and never hand-edit the number.

## Usage and exit codes

Run from the repository root after building the target (paths are examples):

```powershell
# print the hash line, exit 0
.\build\core6\tests\physics2d_regression.exe

# write a golden baseline for the current toolchain, exit 0
.\build\core6\tests\physics2d_regression.exe --write-golden tests\regression\golden\default_600.txt

# verify against a golden baseline: exit 0 match / 1 hash mismatch / 2 unreadable or invalid file
.\build\core6\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# usage text, exit 0
.\build\core6\tests\physics2d_regression.exe --help
```

Exit codes: `0` OK/match, `1` weak-hash mismatch, `2` bad arguments or unreadable
file. The exact hash is never part of the pass/fail decision.

In CI: `build-test` (Ubuntu/Windows debug+release), `headless`, `sanitizers` and the
`regression` job all run this CTest entry, so each platform gates its own golden.
The explicit `regression` job on `ubuntu-latest` calls the GCC golden directly.

## Golden policy (when to update)

The golden is a **physical-behaviour baseline**, not a formatting artefact:

1. Regenerate only when the physics or the scene **intentionally** changes
   (solver change, broad-phase default change that is supposed to be
   physics-equivalent, scene/seed/frame count edit) **or when the toolchain
   itself changes** (new compiler/libm; see above).
2. A pure refactor (e.g. the `BodySim`/`BodyState` hot/cold split, switching the
   default broad phase while the canonical pair set is unchanged) **must not**
   change the weak hash within the same toolchain; if it does, that is a bug to
   investigate, not a golden update.
3. Every golden change must be explained in the commit message / `docs/devlog.md`
   with the old and new hash **and the toolchain**. Never hand-edit the hash.
4. Regeneration command (canonical debug build, from the repository root; on Linux
   write the `_gcc` file instead):

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
  `0` match / `1` mismatch / `2` bad input.
