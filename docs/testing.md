# 手动测试指南（Manual Testing Guide）

> 面向两类读者：**人类用户**想亲手验收这个引擎；**换会话的新 AI** 需要知道怎么复现全部验证。
> 所有命令在仓库根目录执行（`cd "<repo-root>"`）。
> 本机**已装 PowerShell 7（`pwsh` 7.6.9），但构建一律走 `powershell`（PS 5.1）**：`scripts/build.ps1`
> 在 PS7 下会因 `$isWindows` 只读自动变量报错（尚未适配）；本文命令均按 PS 5.1 验证。
>
> 当前基线（tag `v1.0-release`）：**debug/sanitize 387 / release 390 个 ctest 全绿、
> 0 项目警告**（31 个 sandbox death test 自动 SKIP）；8 个 demo smoke 全 exit 0；回归弱 hash 按 toolchain（MSVC `588d30e70f9b4ddd` / GCC·Linux `001d846118f05125`，同一 toolchain 内稳定）。数字随机器/噪声略有浮动，
> 断言以"通过/exit 0/哈希相等"为准，具体毫秒数只作参考。

---

## 0. 30 秒版（只想确认"项目是活的"）

```powershell
cd "<repo-root>"
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
```

期望输出结尾：

```text
100% tests passed, 0 tests failed out of 387
```

（31 个 `Skipped` 是 sandbox 里无法运行的 death tests，属正常，见 §8。）

想看画面：

```powershell
.\build\debug\testbed\physics2d_testbed.exe
```

---

## 1. 测试层级总览

| 层 | 测什么 | 怎么跑 | 耗时（本机参考） |
| --- | --- | --- | --- |
| L0 全量构建+单测 | 编译、0 警告、387 个 ctest | `scripts/build.ps1 -Preset debug -RunTests` | clean ≈60 s / 增量 ≈10 s |
| L1 单个/一组 gtest | 某模块的精确行为 | `build\debug\tests\physics2d_tests.exe --gtest_filter=...` | 单测 <1 s |
| L2 ctest 过滤 | 按名字跑一组（含 regression） | `ctest --test-dir build/debug -R "WorldSleepTest"` | 1-15 s |
| L3 smoke（自动跑 demo） | 8 个场景的物理/渲染回归 | `physics2d_testbed.exe --demo X --smoke N --hidden` | 每 demo 1-10 s |
| L4 交互测试台（人眼） | 稳定堆叠、摩擦、休眠、宽相切换、面板 | `physics2d_testbed.exe`（见 §4） | 10 分钟 |
| L5 bench（性能） | SAP vs BruteForce 真实耗时 | Release 的 `physics2d_bench.exe`（见 §6） | ≈10 s |
| L6 确定性回归 | 600 步弱 hash 与 golden 一致 | `physics2d_regression.exe --check ...`（见 §7） | <1 s |
| L7 death tests | 非法输入必须 assert（契约） | 把测试 exe 复制到工作区外跑（见 §8） | ≈15 s |

---

## 2. 全量验收（每次改动后推荐）

```powershell
# debug：配置 + 编译 + ctest（Windows 上若检测到 header 比 obj 新，会自动 clean-first）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
```

期望：`100% tests passed, 0 tests failed out of 387`，且构建输出里 **`warning C` 数量为 0**。

三 preset 完整验收（注意全是**干净构建**才有可比的耗时数字）：

```powershell
# Release（390 个用例，含 3 个 release-only 契约测试）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -RunTests

# Sanitize（MSVC ASan；387 个用例；ASan 下跑测试需要脚本导入的 vcvars PATH）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -RunTests
```

干净构建（想复现周验收耗时）先删 canonical 目录再跑：

```powershell
Remove-Item -Recurse -Force build/debug, build/release, build/sanitize -ErrorAction SilentlyContinue
# 然后按上面三条命令依次跑；本机 v1.0 canonical 参考（首次含 FetchContent 下载）：debug ≈50 s / release ≈54 s / sanitize ≈85 s
```

> 想开一个不影响 canonical 的隔离目录：给命令加 `-BuildDir build/mycheck`（可再加 `-NoTestbed`
> 只跑 core+测试，更快）。

---

## 3. 跑单个 / 一组测试

**ctest 过滤（推荐，名字 = `Suite.Test`，支持正则）：**

```powershell
# 整个 WorldSleepTest 套件（10 个）
ctest --test-dir build/debug -R "WorldSleepTest" --output-on-failure

# 整个 WorldIslandsTest 套件（4 个）
ctest --test-dir build/debug -R "WorldIslandsTest" --output-on-failure

# 精确一个用例
ctest --test-dir build/debug -R "WorldStabilityTest.TenBoxTowerStandsForTenSecondsAtDefaultIterations" --output-on-failure

# 只看失败时打印日志（排查必加 --output-on-failure）
ctest --test-dir build/debug -R "SolverIsland" --output-on-failure
```

**直接跑 gtest 可执行文件（更快、能看 printf 数据）：**

```powershell
# 先列出所有用例名
.\build\debug\tests\physics2d_tests.exe --gtest_list_tests

# 精确一个用例（输出里有手算期望值/实测数据）
.\build\debug\tests\physics2d_tests.exe --gtest_filter="SolverIslandTest.GroupedVelocitySolveIsBitIdenticalToFlatSolve"

# 按套件跑
.\build\debug\tests\physics2d_tests.exe --gtest_filter="WorldSleepTest.*"

# 跳过 death tests（工作区内它们本来也会 SKIP）→ 338 个
.\build\debug\tests\physics2d_tests.exe --gtest_filter="-*DeathTest*"

# 只跑失败过的
.\build\debug\tests\physics2d_tests.exe --gtest_filter="*" --gtest_break_on_failure
```

可用的套件（写测试/排查时最常用）：`MathTest` 系列、`SATTest`、`ClippingTest`、`ContactSolver`、
`SolverFriction`、`SolverWarmStart`、`SolverSplitImpulse`、`SolverIslandTest`、`IslandTest`（岛屿 DFS）、
`BroadPhaseBruteForceTest`、`BroadPhaseSapTest`、`WorldStabilityTest`、`WorldBroadPhaseTest`、
`WorldIslandsTest`、`WorldSleepTest`。（回归工具不是 gtest，而是普通 CTest 条目 `regression_default`，
见 §7。）

---

## 4. 交互式测试台（人眼验收，最重要的"手动测试"）

```powershell
# 默认从 FallingBall 开始
.\build\debug\testbed\physics2d_testbed.exe

# 直接进某个 demo
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes
.\build\debug\testbed\physics2d_testbed.exe --demo IslandSleep500
```

**操作键**（左下 Testbed Controls 也有按钮；快捷键在 ImGui 输入框聚焦时自动失效）：

| 输入 | 作用 |
| --- | --- |
| 鼠标中键拖拽 / 滚轮 | 平移 / 缩放相机 |
| `R` | 相机回默认视野（不重建场景） |
| `Space` | 暂停 / 继续 |
| `.` | **单步**（暂停时生效，每按一次前进一个固定 1/60 s 步） |
| `1` / `2` / `3` | 1x / 0.5x / 0.25x 慢放 |

**10 分钟人工验收脚本（按顺序点一遍即可覆盖 W1-W7 全部特性）：**

1. **FallingBall（默认）**：球落地停住（穿透 5 mm，肉眼不可见）；把 Solver 面板的
   **`Restitution (e)`** 拉到 0.8，点 demo 面板的 **`Drop again`** → 弹起约 3.4 m；速度降到阈值以下后不再弹。
2. **BoxStack**：10 层塔站住不抖（穿透约 3.8 mm）；点 **`Drop impact ball`** 撞塔；**`Reset tower`** 重来；
   把 Solver 面板 `Velocity iterations` 从 8 调到 3，塔会明显变软/下陷——这就是迭代预算的直观演示
   （恢复到 8）。
3. **FrictionRamp**：θ=15°、μ=0.30 > tanθ=0.268 → 面板判定 STATIC、方块不动；把 **`Friction mu`**
   拉到 0.1 → 方块下滑；用 **`Incline theta (deg)`** 调到 45° 再调 μ 观察阈值翻转；`Reset block` 复位。
4. **Dominoes**：开场约 1 秒后排全部入睡（Status `Awake=0 Sleeping=20`）；第 60 帧自动推第一块
   （或点 **`Push first`**）→ 20 块逐个倒下，面板 `Woken after sleep 20/20`、`Chain reach 20`、
   `Fallen 20`；`Reset`/`Rebuild` 重来。
5. **IslandSleep500**：500 盒、Status `Islands=100`；约 1.75 s 后 `Awake=0 Sleeping=500`；demo 面板显示
   **`Sleeping speedup: ≈1.7x (awake … ms -> asleep … ms)`**；点 demo 的 **`Wake all`**（或 Solver 面板的
   **`Wake all bodies`**）全部醒、再逐渐睡回去。
6. **宽相切换**：Solver Settings 面板的 **`Implementation`** 下拉依次选 `None / BruteForce / SAP`，
   Status 与 demo 面板的 `proxies / candidates` 数字变化，但**物理结果不变**（BoxStack 塔仍然一样稳）。
7. **Contacts 面板**：点任一接触行 → 场景里该接触变青色、画出 1 m 法线；面板显示每个接触点的
   **累计法向/切向冲量柱状条**与摩擦锥占用（0%=静摩擦，100%=滑动）；暂停后按 `.` 单步，能看到
   接触点/冲量逐帧变化。
8. **显示开关**（DebugUI 底部）：勾选 **`Body AABBs`** → 看到比形状大 0.1 m 的 fat AABB（宽相实际看的
   盒子）；**`Contact points`** / **`Contact normals`** / **`Velocity arrows (overlay)`** 各自可视化。
9. **Pause / 单步 / 慢放**：`Space` 暂停 → `.` 逐帧观察 BoxStack 的接触点与冲量；`3`（0.25x）慢放看
   多米诺倒塌过程；Testbed Controls 面板也有 `Step` / `Reset` 按钮与 1x/0.5x/0.25x 单选。

> 交互模式需要可用的 OpenGL 3.3 上下文。远程桌面/无 GPU 环境可能失败；`--smoke` 会用退出码 2
> 报告"无 GL 环境"（物理与单测不受影响）。

---

## 5. Smoke（无人工的视觉层回归）

```powershell
$tb = ".\build\debug\testbed\physics2d_testbed.exe"
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall  --smoke 150 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack  --smoke 300 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack     --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes     --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo Pyramid200   --smoke 300 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden
```

每个命令 = 隐藏窗口 + 固定 1/60 s 步进 N 帧 + 打印一行机器可读摘要，**exit 0 表示通过**。
关键期望值（v1.0 canonical debug，允许噪声；精确值见 `profiling.md` §6）：

| Demo | 关键字段期望 |
| --- | --- |
| FallingBall 150 | `contacts=1`、`max_penetration_mm=5.057`（≈kLinearSlop）、球心 y=0.495、已入睡 |
| SphereStack 300 | `bodies=4 contacts=3`、`max_pen=3.063mm`、`sleeping=3` |
| BoxStack 600 | `bodies=11 contacts=10 contact_points=20 max_pen=3.893mm`、塔站立（`max_center_y=9.478`、`max_speed=0.007`） |
| FrictionRamp 300 | `contacts=1 max_pen=2.686mm`、方块静止（θ=15°, μ=0.30） |
| Dominoes 600 | `woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1` |
| IslandSleep500 600 | `islands=100 awake=0 sleeping=500 contacts=500 max_pen=3.983mm`、`first_all_asleep_frame=105`、`speedup≈1.7x` |
| Pyramid200 300 | `bodies=137 contacts=324 max_pen=41.863mm broadphase=SAP`、不塌 |
| StressBenchmark 300 | `bodies=401 contacts=820 max_pen=170.739mm broadphase=SAP`、物理与宽相选择无关 |

**宽相 A/B（验证"换宽相不改物理"）**——同一 demo 三种宽相跑，body/contact 输出行必须逐字节相同：

```powershell
$tb = ".\build\debug\testbed\physics2d_testbed.exe"
# 人工对比：只有 candidate_pairs 应该不同
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase none
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase bf
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase sap

# 或自动比对 SHA-256（三行应完全相同）
function Get-BodyHash($bp) {
  $lines = & .\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase $bp 2>&1 |
           Select-String -Pattern '^\[smoke\] body=|^\[smoke\] contact=' | ForEach-Object { $_.Line }
  $sha = [System.Security.Cryptography.SHA256]::Create().ComputeHash(
             [System.Text.Encoding]::UTF8.GetBytes(($lines -join "`n")))
  [System.BitConverter]::ToString($sha).Replace('-','').Substring(0,16)
}
"none=$(Get-BodyHash none) bf=$(Get-BodyHash bf) sap=$(Get-BodyHash sap)"
```

退出码约定：`0` = 通过；`2` = 无 GL 环境 **或** 未知 demo 名；`1` = 非法参数（如错误的 `--broadphase`）。

---

## 6. 性能测试（bench，必须 Release）

```powershell
# 构建 Release 的 bench（若已构建过可跳过）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -NoTestbed -Target physics2d_bench

# 完整表格：N=50/200/500/2000，每档 1000 帧（前 100 帧预热丢弃），只计宽相
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42

# 小 N 反转（SAP 在 <~30 体时更慢，测 Auto 阈值来源）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 5,10,20,50 --frames 500 --seed 42

# 机器可读
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200 --frames 200 --seed 42 --json
```

期望（v1.0 canonical Release，3 次中位；本机 i7-12650H / MSVC 19.44；不同机器会浮动，看趋势与加速比）：

| Bodies | BruteForce ms/帧 | SAP ms/帧 | 加速比 |
| ---: | ---: | ---: | ---: |
| 50 | ≈0.0034 | ≈0.0016 | ≈2.12x |
| 200 | ≈0.0839 | ≈0.0173 | ≈4.88x |
| 500 | ≈0.5466 | ≈0.0931 | ≈5.92x |
| 2000 | ≈9.3061 | ≈1.2084 | ≈7.70x |

> 性能数字必须用 Release（Debug 数字无意义）。本机噪声约 ±2-5%，只比较量级/加速比。

---

## 7. 确定性回归（hash）

```powershell
# MSVC 本机（Windows）：默认 golden 名 default_600.txt
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# GCC/Clang（Linux）：使用 _gcc golden（CMake 的 regression_default 会自动选择；手工跑时显式指定）
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600_gcc.txt

# 只打印当前 hash（调试用）
.\build\debug\tests\physics2d_regression.exe

# 有意改变物理/场景后重新生成该 toolchain 的 golden（谨慎！必须写 devlog 说明 old/new 与原因）
.\build\debug\tests\physics2d_regression.exe --write-golden tests\regression\golden\default_600.txt
```

- 判定值是**量化弱 hash**（quantum 1e-5），`exact=` 只是原始位 hash 的信息打印。
- **同一 toolchain 内**：纯重构（宽相切换、岛屿分组、排序优化）hash 必须不变；只有"有意的物理/场景
  变更"才更新 golden，并在 `docs/devlog.md` 记录。
- **跨 toolchain**：hash 不可移植（首次远端 Linux CI 实测：MSVC `588d30e70f9b4ddd` vs GCC
  `001d846118f05125`，Linux debug/release/ASan 三次都稳定在后者）。因此仓库有**两份 golden**，
  `tests/CMakeLists.txt` 按编译器选择；详见 [`tests/regression/README.md`](../tests/regression/README.md)。
- 退出码：`0` 匹配 / `1` 不匹配 / `2` 文件或参数错误。

---

## 8. Death tests（契约测试）——为什么工作区里 SKIP、怎么真跑

个别测试验证"非法输入必须 assert（debug）"，它们需要子进程写临时文件；本机 sandbox 不允许工作区
内可执行文件这样做，所以 `ctest` 里 31 个 death test 会显示 `Skipped`，**这是正常的**。

想真正执行它们（验证契约没退化）：

```powershell
$dir = Join-Path $env:LOCALAPPDATA 'Physics2D\testrun'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
Copy-Item "build\debug\tests\physics2d_tests.exe" $dir -Force
Push-Location $dir
.\physics2d_tests.exe        # 期望：386 tests from 60+ suites，PASSED 386
Pop-Location
```

> Week 7 就是靠这一步抓到一个"sleeping skip 把 static-static 非法输入也跳过、导致 assert 不再触发"
> 的契约回归；**改动求解器/World 之后建议都跑一次**。期望 386/386（含 31 个 death test）。

---

## 9. 常见问题（FAQ）

| 现象 | 原因 / 处理 |
| --- | --- |
| `pwsh : 无法将"pwsh"项识别为...` | 旧环境才会出现（本机现已装 PowerShell 7）；遇到时把 `pwsh` 换成 `powershell` 即可——本仓库命令统一用 `powershell`（`build.ps1` 在 PS7 下未适配） |
| `ctest --preset debug` 报 no such test preset | ctest 的 preset 名不是 `debug`；用 `ctest --test-dir build/debug` 或直接 `build.ps1 -RunTests` |
| ctest 显示 31 个 Skipped | 正常，sandbox death tests；见 §8 |
| ctest 用例数 debug 387 / release 390 | 正常：release 多 3 个 release-only 契约测试 |
| 构建输出里 `warning C...` | 不应该有；任何项目代码警告都算回归（第三方依赖警告不算） |
| 改了 header 但增量构建"没反应" / 出现 `HEAP CORRUPTION` | 本机中文 MSVC 的 `/showIncludes` 让 Ninja depfile 为空。`build.ps1` 已自动检测并强制 clean-first；也可手动 `-Clean` 或删 `build/<preset>` |
| 增量构建很慢 / 每次都全量 | 说明 header 比 obj 新，触发了自动 clean-first（正确行为）；稳定后增量恢复 |
| 测试数字（毫秒/穿透）和文档差一点 | 允许的机器噪声；以"通过/exit 0/hash 相等"为验收标准，绝对值参考 devlog 的测量条件 |
| testbed 窗口打不开 / `--smoke` exit 2 | 需要 OpenGL 3.3 上下文（远程桌面/无 GPU 环境不可用）；物理核心与单测不受影响 |
| BoxStack smoke 300 步 `awake=10`，不是睡着吗？ | 正常：塔在 step 859 才满足 0.5 s 连续安静窗（残余抖动，见 README 已知限制）；600 步塔仍站立，900 步全睡 |
| Pyramid200/StressBenchmark 穿透几十~一百多 mm | 载荷/迭代预算现象（不是 bug）：底面承受十几层重量、400 盒密堆；切宽相物理不变。金字塔默认 16 层就是按实测选的 |
| 为什么 `Dominoes` 不用更薄的骨牌？ | 0.12 m 薄牌静止抖动 > 睡眠阈值 0.01 m/s，永远睡不着（已被实测记录）；0.30×1.0 m 是"可入睡"形状，正解是 v1.1 的 2×2 block solver（v1.0 未做，见 README 已知限制） |

---

## 10. 验收清单（手动跑完后逐项打勾）

- [ ] `build.ps1 -Preset debug -RunTests` → **387/387 passed**，0 项目警告
- [ ] `-Preset release -RunTests` → **390/390 passed**，0 警告
- [ ] `-Preset sanitize -RunTests` → **387/387 passed**（ASan 无诊断）
- [ ] 8 个 demo `--smoke --hidden` 全部 **exit 0**，关键字段符合 §5 表
- [ ] `--broadphase none|bf|sap` 的 body/contact 输出 **SHA-256 相同**（§5 脚本）
- [ ] `physics2d_regression --check` → **MSVC `588d30e70f9b4ddd` / GCC·Linux `001d846118f05125`**、exit 0
- [ ] 工作区外跑测试 exe → **386/386 gtest passed**（含 31 个 death test，§8）
- [ ] 交互验收 §4 的 9 项（塔稳定/摩擦翻坡/多米诺连锁/休眠 1.7x/宽相切换/Contact 柱状条/单步/慢放/AABB）
- [ ] `scripts/format.ps1 -Check` → OK（117 个 C++ 文件）
- [ ] `scripts/record_gifs.ps1` → 5 个 GIF，单张 ≤600KB、总计 ≤2.5MB（§11）
- [ ] （可选）Release bench 加速比趋势符合 §6 表

---

## 11. 录制 / 更新 README GIF（W8 新增）

GIF 由 testbed 的**确定性帧捕获**生成（不是录屏）：`--smoke` 的固定 1/60 s 步进保证每次录制
回放同一段物理；`--capture-dir DIR` 在 world + ImGui 都画完后、`glfwSwapBuffers` 前用
`glReadPixels(GL_BACK)` 读回像素，写成 PPM，再由 `scripts/record_gifs.ps1` 内嵌的 Python+Pillow
编码成 GIF（本机无 ffmpeg）。一条命令重录全部 5 张：

```powershell
# 先有 release（或 debug）testbed；Python 3 带 Pillow（DSH 环境可用捆绑 Python，-PythonExe 指定）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
# 只重录其中几张 / 指定构建目录 / 指定 Python
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 -Only dominoes,island_sleep
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 -Testbed build/w8tbrel/testbed/physics2d_testbed.exe
```

捕获 CLI（`--capture-*` 只在 `--smoke` 下有效；默认关闭零开销）：

```powershell
.\build\release\testbed\physics2d_testbed.exe --demo BoxStack --smoke 900 --hidden `
    --capture-dir build/capture/box_stack --capture-every 8 --capture-start 1
# 另有 --capture-hide-ui（只录场景）与 --capture-display contacts/normals/aabb/axes/velocities=on|off
```

输出与预算：`docs/images/{dominoes,box_stack,sap_stress,island_sleep,contact_debug}.gif`，
720px 宽、12.5 fps、每张 ≤600KB、总计 ≤2.5MB（脚本自动校验）；PPM 中间文件写在 gitignored 的
`build/capture/` 并在编码后删除——**不要把 PPM/PNG/大文件提交进仓库**。改 testbed 渲染后重录，
并用 `read_image` 或 Pillow 抽 1-2 帧确认画面/UI 可读。详见 [`docs/demos.md`](demos.md) §4。

---

## 附：命令速查（复制即用）

```powershell
cd "<repo-root>"

# 构建 + 全部测试
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 只跑一组 / 一个
ctest --test-dir build/debug -R "WorldSleepTest" --output-on-failure
.\build\debug\tests\physics2d_tests.exe --gtest_filter="SolverIslandTest.*"

# 看画面 / 冒烟
.\build\debug\testbed\physics2d_testbed.exe
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes --smoke 600 --hidden

# 性能 / 回归
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -NoTestbed -Target physics2d_bench
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
```
