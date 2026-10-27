# tests/regression — RESERVED (Week 6-7, P16)

This directory will hold the headless determinism regression tool:

| File | Purpose | Week |
| --- | --- | --- |
| `regression_main.cpp` | `physics2d_regression` executable: fixed scene + N steps + state hash + CSV export | 6-7 |
| `cases/*.yaml` | parameterised scene descriptions (seed, rows, shape mix) | 7 |
| `golden/*.csv` | golden baselines; regenerated only when physical behaviour intentionally changes | 6-7 |

Design contract (Section 7.3 / P16):

```bash
# write a baseline
physics2d_regression --scene pyramid200 --steps 10000 --seed 42 --out golden/pyramid200.csv

# verify
physics2d_regression --scene pyramid200 --steps 10000 --seed 42 --hash --compare golden/pyramid200.csv
```

* Exit codes: `0` pass, `1` hash mismatch, `2` NaN detected, `3` penetration over threshold, `4` bad arguments.
* Two hashes are printed: a bit-exact FNV-1a of float state, and a "weak" hash over values
  quantised to `1e-5` for cross-platform comparison (Section 7.4).
* The tool links `physics2d_core` only, never GLFW/ImGui, so CI can run it headless.

The CMake target is intentionally not created yet: it would show up red in CI
before the physics behaviour exists. `tests/CMakeLists.txt` contains the reserved
comment block where the target will be registered.
