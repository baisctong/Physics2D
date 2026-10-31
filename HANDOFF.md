# Physics2D 交接报告（Session Handoff）· v1.0-release

> **一句话现状**：**Week 1-8 全部完成，tag `v1.0-release`**。完整物理链路（宽相 → 窄相 → SAT/裁剪
> Manifold → Sequential Impulse（摩擦锥/warm starting/split impulse）→ 岛屿/整岛休眠）、可交互
> testbed（8 demo + 帧捕获 GIF）、完整文档（架构/数学/性能/边界契约/演示）、确定性回归与
> 边界契约测试全部交付。**canonical clean 验收：debug 387 / release 390 / sanitize 387 个 ctest
> 全绿、0 项目警告、回归弱 hash（MSVC `588d30e70f9b4ddd` / GCC·Linux `001d846118f05125`，同一 toolchain 内）不变、8 demo smoke 全 exit 0、宽相 A/B
> SHA-256 三路相同、工作区外 386/386 gtest（31 death test 真跑）**。
> **下一步**：v1.1（首位：2 点接触 2×2 block solver；其余 Tracy/插值/冷热拆分/多场景回归/岛屿并行）。
>
> 新会话请按顺序读：
> 1. 本文件（10 分钟：现状、基线、红线、坑、v1.1 入口）
> 2. [`AGENTS.md`](AGENTS.md)（项目宪法 + 工作协议，**必须遵守**）
> 3. [`docs/ROADMAP.md`](docs/ROADMAP.md)（8 周计划与每周验收记录）、
>    [`docs/dev-interfaces.md`](docs/dev-interfaces.md)（接口状态 + **偏差登记** + 不变量）
> 4. 需要细节时再查 [`docs/architecture.md`](docs/architecture.md)、
>    [`docs/math.md`](docs/math.md)、[`docs/profiling.md`](docs/profiling.md)、[`docs/devlog.md`](docs/devlog.md)
>
> 原始设计文档（仓库外）：`../2D物理引擎项目完整工作流与DeepSeek提示词库.md`
> 仓库路径：`<repo-root>`
> 当前 HEAD：`v1.0-release` 指向的提交（`git log -1` / `git describe --tags` 为准）。

---

## 0. 新会话开场 3 步（别跳过）

1. **读**本文件 §1-§4 + `AGENTS.md`，确认你理解固定 Step 顺序、符号约定、分层铁律、
   W5/W6/W7/W8 不变量（§4.3）。
2. **跑验证基线**（§2）。不先确认基线的会话，后面所有回归都不可信。具体命令、期望输出、
   单测过滤、交互验收脚本见 [`docs/testing.md`](docs/testing.md)。
3. **按 §5 决定方向**：v1.0 已发布，"物理行为"进入维护模式——任何物理/求解器改动先建任务卡、
   先写测量方案、改后必须过 §2 全量验收并把回归 hash 的 old/new 写进 devlog。

---

## 1. 仓库与状态

| 项 | 值 |
| --- | --- |
| 仓库路径 | `<repo-root>` |
| Git | `main` 分支，工作区干净，tags `v0.1-falling-ball` → … → `v0.7-island-sleep` → **`v1.0-release`** |
| 构建目录 | `build/`（**可随时整个删除**，已被 gitignore；canonical `debug/release/sanitize` + 各 agent 并行目录） |
| 团队状态 | 所有 teammate 为 inactive；无后台任务；W8 任务卡（task-1..6）均 completed |
| 环境 | Windows 11 22631 / i7-12650H / 16GB / VS2022 (MSVC 19.44) / CMake 3.31.12 / Ninja 1.12.1（VS 自带）/ RTX 4060 |
| 版本状态 | v1.0：**物理与公共 API 冻结**；v1.1 只允许在任务卡与测量方案下改动 |

文档地图：

| 文件 | 作用 |
| --- | --- |
| `AGENTS.md` | 项目宪法 + 工作协议 + 技术债 + 下一轮入口（**每次会话必读**） |
| `docs/ROADMAP.md` | 8 周计划、每周 DoD、W1-W8 验收记录 |
| `docs/dev-interfaces.md` | 18 条预留接口状态、**偏差登记（重要）**、W5/W6/W7 不变量 |
| `docs/architecture.md` | W8 升级版：一张图总览 + 模块依赖 + 数据布局/所有权 + Step 11 阶段时序 + 岛屿/休眠状态机 + 宽相/窄相/求解数据流（6 张 Mermaid） |
| `docs/math.md` | **新**：SAT/冲量/摩擦锥/warm starting/split impulse/岛屿休眠逐步推导 + §10 边界契约与已知限制 |
| `docs/profiling.md` | **新**：全部实测性能表（宽相/休眠/三求解模式/构建耗时/8 demo smoke）+ 测量环境与纪律 + §1.5 Phase C canonical 复测；**性能数字的唯一事实来源** |
| `docs/boundary-contracts.md` | **新**：边界场景契约表（Debug assert / 返回值 / 文档化限制）+ 13 条已知限制 + 新增测试索引 |
| `docs/demos.md` | **新**：8 个 demo 一览、操作键、4 个 DebugUI 面板、GIF 清单与录制命令 |
| `docs/release-notes-v1.0.md` | **新**：v1.0-release 发布说明（GitHub Release 草稿） |
| `docs/github-publish.md` | **新**：推送到 GitHub 的检查清单（仓库体检、Git 身份重写、remote/tags、Release/About/badges、CI 首跑注意） |
| `docs/testing.md` | **手动测试指南**：30 秒版 / 全量验收 / 单测过滤 / 10 分钟人工验收脚本 / smoke / bench / 回归 / death tests / GIF 录制 / FAQ / 验收清单 |
| `docs/dev-setup.md` | 环境、构建、测试台操作、疑难杂症（含本机增量构建坑与自动防护） |
| `docs/devlog.md` | W1-W8 周报：完成/卡住/数据/下周/提醒（含所有卡点与修复复盘；W8 章含"多 agent 文档数字漂移"复盘） |
| `docs/images/` | 5 张 README GIF（每张 ≤600KB，总计 1,692.6 KB；由 `scripts/record_gifs.ps1` 生成） |
| `README.md` / `README.en.md` | 对外展示（v1.0 特性/GIF 墙/快速开始/架构/核心算法/真实性能表/Demos/已知限制/Roadmap） |
| `HANDOFF.md` | 本文件：换会话交接 |

---

## 2. 验证基线（开工前先跑，确认环境没变）

> **想自己手动验收/玩一玩？** 完整分层指南在 [`docs/testing.md`](docs/testing.md)
> （30 秒版、单测过滤、交互测试台 10 分钟人工验收脚本、smoke/bench/回归/death tests、GIF 录制、
> FAQ、验收清单）。下面是最短的官方基线。

> 本机**已装 PowerShell 7（`pwsh` 7.6.9），但仓库命令统一用 Windows PowerShell 5.1**：`build.ps1`
> 在 PS7 下会因 `$isWindows` 只读自动变量报错（尚未适配，见 `docs/dev-setup.md` §6）；
> 首次 configure 会联网 FetchContent 下载依赖（1-2 分钟）。命令必须在仓库根目录执行。

```powershell
cd "<repo-root>"

# 全量：配置 + 编译 + ctest（幂等；Windows 上若检测到 header 比 obj 新会自动 clean-first）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
```

**期望基线（v1.0 release 实测，2026-10-30 canonical clean）**：

| 检查项 | 期望值 |
| --- | --- |
| 构建 | exit 0，MSVC `/W4 /permissive-` **0 个项目警告** |
| ctest | **debug 387 / release 390 / sanitize 387 passed, 0 failed, 31 skipped**（skipped 全是 sandbox death tests） |
| 测试 exe | debug = **386 个 gtest** + 1 个 `regression_default`；release 多 3 个 release-only 契约测试 |
| clean 构建耗时 | debug **50.1 s** / release **53.6 s** / sanitize(ASan) **86.0 s**（含依赖下载/编译） |
| 测试 exe 工作区外运行 | **386/386 gtest 全过**（31 个 death test 真正执行；这是发现契约回归的唯一可靠尺子） |
| 格式 | `scripts/format.ps1 -Check` → **117 个 C++ 文件 OK**（CI 有 clang-format job） |
| 远端 CI | **GitHub Actions 9/9 全绿**（run `41174523124`，2026-10-31；Ubuntu/Windows debug+release、ASan+UBSan、headless、regression、format、layering） |

八个 demo 冒烟（**v1.0 canonical debug 实测**；`islands/awake/sleeping` 为 World 统计）：

```powershell
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall  --smoke 150 --hidden
# bodies=2  islands=1 awake=0 sleeping=1 contacts=1 max_pen=5.057mm
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack  --smoke 300 --hidden
# bodies=4  islands=1 awake=0 sleeping=3 contacts=3 max_pen=3.063mm
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack     --smoke 600 --hidden
# bodies=11 islands=1 awake=10 sleeping=0 contacts=10 max_pen=3.893mm（900 步后入睡）
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
# bodies=2  islands=1 awake=0 sleeping=1 contacts=1 max_pen=2.686mm
.\build\debug\testbed\physics2d_testbed.exe --demo Pyramid200   --smoke 300 --hidden
# bodies=137 islands=1 awake=136 contacts=324 max_pen=41.863mm broadphase=SAP
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden
# bodies=401 islands=4 awake=400 contacts=820 max_pen=170.739mm broadphase=SAP
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes     --smoke 600 --hidden
# bodies=21 islands=5；dominoes woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1
.\build\debug\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden
# bodies=501 islands=100 awake=0 sleeping=500 contacts=500；frame 105 全睡；
# debug: early 4.96ms -> asleep 2.96ms (1.68x)；release: 0.4634 -> 0.2656 ms (1.74x)
```

宽相 A/B（**切宽相物理结果必须完全一致**）：`--broadphase none|bf|sap`，同一 demo 的 body/contact
输出行应逐字节相同（只有 candidate_pairs 不同）。v1.0 实测 BoxStack/StressBenchmark/Dominoes 三场景
三路 SHA-256 相同。

回归工具（golden 必须绿，hash 自 W6 起未变）：

```powershell
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
# 期望（MSVC）：hash=588d30e70f9b4ddd（quantum 1e-5）、exact=f4390ef86c075a04、exit 0；
# mismatch exit 1；文件/参数错误 exit 2
```

bench（**Release**，数字必须可复现，完整条件表见 `docs/profiling.md` §2）：

```powershell
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42
# 期望（Phase C canonical，3 次中位）：SAP 加速比 2.12x / 4.88x / 5.92x / 7.70x；
# 小 N 交叉 N≈24–26（Auto 阈值保守取 32）
```

GIF 重录（可选，5 张，一条命令；需要 Python 3 + Pillow，本机无 ffmpeg）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
# 输出 docs/images/{dominoes,box_stack,sap_stress,island_sleep,contact_debug}.gif
# 每张 ≤600KB、总 ≤2.5MB；PPM 中间文件自动删除
```

> `testbed` 可执行文件路径是**嵌套的**：`build/<dir>/testbed/physics2d_testbed.exe`。
> 无 OpenGL 上下文时 smoke 会返回退出码 2 并打印原因（当前机器可用，NVIDIA 3.3）。

**怀疑环境坏了时**（先看 `docs/dev-setup.md` §6 / `docs/testing.md` §9）：

- 本机已装 `pwsh`，但统一用 `powershell`（`build.ps1` 在 PS7 下会因 `$isWindows` 报错）；
- Ninja 必须在仓库外（`scripts/build.ps1` 自动从 VS 安装目录找）；把 Ninja 放进 `tools/` 会触发
  `cl D8050` / `Permission denied`（沙箱限制）；
- **本机增量构建坑（重要）**：中文 locale 的 cl.exe 把 `/showIncludes` 打成 `注意: 包含文件:`，
  CMake 写入 `rules.ninja` 的是 UTF-8 前缀，Ninja 按字节匹配失败 → **depfile 为空，改 header 不重编**
  → 类布局混链会 **HEAP CORRUPTION**。`scripts/build.ps1` 已自动检测"header 比 obj 新"并强制
  `--clean-first`；canonical 验收仍须删 `build/<preset>` 干净重编；
- `ctest --preset` 必须在仓库根目录跑（或用 `--test-dir`）；
- death tests 在本沙箱自动 SKIP；把 `build/debug/tests/physics2d_tests.exe` 复制到
  `%LOCALAPPDATA%\Physics2D\testrun\` 再运行可得到 **386/386 gtest 全过**（证明 CI 会执行它们）；
- **不要用 PowerShell 5.1 的 `Get-Content`/`Set-Content` 改 UTF-8 中文文档**（会乱码）；
  用编辑工具（本 harness 的 write/edit，或 `-Encoding UTF8` 且显式处理 BOM）。

---

## 3. 已完成内容（Week 1-8）

| 周 | 主题 | 关键交付 | 数据亮点 |
| --- | --- | --- | --- |
| W1 | 数学 + 构建 + 渲染循环 + 积分骨架 | `p2::Math`、`Handle`+`IDPool`、`Body`/`World`、11 阶段 `Step`、`StepAccumulator`、GLFW+ImGui testbed、CMake/CI | 120 测试；clean build 23.6s |
| W2 | 形状 + 解析碰撞 + Contact 持久化 | `Shape` 池（Circle/Plane）、解析窄相、`Contact`/`ContactID` 帧间匹配 | 185 测试；静止穿透 7.8mm |
| W3 | 法向 Sequential Impulse | `ContactSolver`（K_n、累计钳制、迭代、restitution bias）、SphereStack | 210 测试；8 迭代残余 0.00065 m/s |
| W4 | 凸多边形 SAT + 裁剪 + SATCache | `PolygonShape`、`SAT`/`Clipping`/`CollidePolygons`、`ContactID.Key()` 去 flip | 278 测试；10 盒塔会塌（W4 基线） |
| W5 | 摩擦锥 + warm starting + split impulse | `ContactSolver` 摩擦/warm start/Baumgarte 基线；`ContactSolverPosition.cpp` 伪速度层；DebugUI 4 面板 + FrictionRamp | **10 盒 10 s 稳定（穿透 5.223mm）** |
| W6 | BruteForce + SAP 宽相 + 量化 + 回归 | 双宽相实现、World 接线 + fat AABB + pair 规范化、`bench_broadphase`、`physics2d_regression` + golden + CI job、Pyramid200/StressBenchmark | **2000 物体 SAP 7.70x**；切宽相物理**逐位不变** |
| W7 | 岛屿 + 按岛求解 + 整岛休眠 | `Solver/Island.{h,cpp}`（显式栈 DFS/规范序）、`ContactSolver` 三模式按岛求解、`World::BuildIslands/UpdateSleep/WakeIslandOfBody`、Dominoes/IslandSleep500、单步/慢放、Contact 冲量柱状条 | 354 测试；三求解模式 21,760 float 0 差异；休眠 **1.74x**；多米诺 20/20 无跳过；回归 hash 不变 |
| W8 | 文档与交付（v1.0-release） | `docs/{math,profiling,boundary-contracts,demos}.md` + `architecture` 升级 + README v1.0/英文版 + release notes；testbed 确定性帧捕获 + `scripts/record_gifs.ps1` + 5 GIF；**35 个边界测试 + 4 处契约硬化**；CI clang-format job | **387/390/387 测试全绿、0 警告**；boundary/death 386/386；hash 不变；宽相 **7.70x**；5 GIF 共 1.69MB |

**模块 → 文件**（只列关键入口，完整用 `git ls-files`）：

```text
include/physics2d/Math/*.h            Vec2 / Mat2 / Rot / Transform / AABB / MathUtils
include/physics2d/Core/               World / Body / BodyDef / Shape / Settings / Handle / IDPool /
                                      TimeStep / BroadPhaseInterface / Profiler
include/physics2d/Collision/          CircleShape / PlaneShape / PolygonShape / Analytic / Manifold /
                                      Contact / ContactID / SAT / Clipping / SATCache / ShapeTypes /
                                      BroadPhaseBruteForce / BroadPhaseSAP
include/physics2d/Solver/             ContactSolver.h / Island.h
src/{Math,Core,Collision,Solver}/*.cpp (Solver = ContactSolver.cpp + ContactSolverPosition.cpp + Island.cpp)
tests/unit/*.cpp                      GLOB 自动收集（加文件不用改 CMake；W8 新增 test_boundary_solver.cpp / test_boundary_world.cpp）
tests/regression/                     regression_main.cpp + golden/default_600.txt + README
bench/bench_broadphase.cpp            只依赖 core 的宽相基准
testbed/                              main / Renderer / Camera / DebugUI / Demo.h / SceneBuilders.h /
                                      demos/{FallingBall,SphereStack,BoxStack,FrictionRamp,
                                             Pyramid200,StressBenchmark,Dominoes,IslandSleep500}
docs/  scripts/                       W8 新增 docs/images/*.gif、scripts/record_gifs.ps1
```

---

## 4. 架构速览（v1.0 已冻结）

### 4.1 World::Step 固定流水线（顺序不可变！）

```text
1 IntegrateForces            [W1；W7 跳过休眠体]
2 UpdateBroadPhase(dt)       [W6；W7 休眠 body 跳过 AABB 更新、proxy 保留]
3 UpdateContacts             [W2/W4/W6；W7 共享 proximity wake pass + 休眠-休眠 pair 复用上一帧 contact]
4 BuildIslands               [W7：DFS 显式栈 + 规范序 + islandId]
4b PrepareConstraints(dt)    [W5；W7 休眠岛不建约束]
5 WarmStart                  [W5；W7 按岛]
6 SolveVelocityConstraints   [W5/W7：岛分组三模式，休眠岛整岛跳过]
7 IntegratePositions         [W1；跳过休眠体]
8 SolvePositionConstraints   [W5；W7 按岛 + 休眠岛跳过]
9 Synchronize                [W2/W6：tight AABB + kAabbMargin 的 fat AABB]
10 UpdateSleep(dt)           [W7：整岛原子计时/入睡]
11 Validate                  [W1：NaN/Inf 扫描，含伪速度；W8 追加 islandId/sleepTime/休眠零速度]
```

- 第一阶段（`IntegrateForces`）先给动态体加重力速度，然后才窄相/求解——**不要改变顺序**。
- 接触锚点是**生成时刻**的几何（`IntegratePositions` 之前）；测试不要用 step 之后的 transform 反投影。
- **v1.0 之后物理冻结**：任何求解器/World 改动都要先有任务卡 + 测量方案 + 回归 hash 处置说明。

### 4.2 数据流与所有权

- `World::m_bodies`（槽位数组，index == BodyHandle::index，永不压实）+ `IDPool` generation。
- `World::m_shapes`（Shape tagged union）+ `Body::shape`；`AttachShape` 从 density 推质量/惯性。
- `World::m_contacts`（每帧重建）+ `m_previousContacts`（上一帧；warm start 数据源 + 休眠 contact 复用源）。
- **宽相（W6）**：`Body::aabb` = fat AABB（tight + `kAabbMargin=0.1`，`Synchronize` 写）；
  `Body::proxyId`（-1 = 无 proxy）+ `World::m_proxyToBody` 反向映射；
  `World::m_broadPhasePairs` = 规范化候选对。
- **岛屿/休眠（W7）**：`Solver/Island.{h,cpp}` 的 `BuildIslandPartition` 产出
  `World::m_islandBodyIndices` / `m_islandContactIndices`（规范序，指针交给 solver）；
  `Body::islandId`（平铺模式为 -1）、`Body::sleepTime`、`Body::isAwake`；
  `World::WakeIslandOfBody` 是整岛唤醒的唯一入口；`SolverSettings::useIslandSolving/
  enableSleeping/packIslandBodies` 是运行时开关。
- `ContactSolver`（World 以 `unique_ptr` 前向声明持有）：`Prepare`（含休眠岛跳过）、`WarmStart`、
  `SolveVelocity`（flat/grouped-direct/grouped-packed 三模式）、`SolvePosition`、`SetIslandPartition`。
- `ContactID::Key()` **不含 flip**；warm starting 依赖 Key 帧间稳定，不要动这些语义。
- `Settings.h` 是唯一常量来源；休眠阈值 `kSleepLinearTolerance=0.01` 等保持设计值。

### 4.3 不变量（v1.1 改前必读；完整清单见 `docs/dev-interfaces.md` §6）

- **宽相只决定"谁进窄相"**：候选对规范化为 `(i<j, sorted, deduped)`；切宽相物理逐位一致；
  proxy 生命周期由 World 管；fat AABB 只加候选。
- **摩擦锥钳制累计值**、法向先解切向后解、`μ = contact.friction`。
- **warm start 只施加一次**（factor 默认 1.0）；**split impulse 只碰伪速度**。
- **岛屿分组求解必须逐位等价**：三种求解模式在同一初始状态下 body 状态与 contact 累计冲量逐位一致；
  规范序（岛按最小成员、岛内 bodies 升序、contacts 全局序）是前提。
- **休眠整岛原子**：不允许单 body 冻结/唤醒；休眠岛保留 proxy 与上一帧 contact；休眠岛跳过 AABB
  更新/窄相/warm start/速度与位置求解；`WakeIslandOfBody` 整岛唤醒；proximity wake 必须在
  `UpdateContacts` 共享 pass（不要放 `UpdateBroadPhase`——那里无宽相会提前 return）。
- **阈值不调**：入睡慢是收敛质量（残余抖动）问题，正解是 2 点接触 block solver（v1.1 首位）。
- **回归 hash 纪律**：golden 按 toolchain 维护（MSVC `588d30e70f9b4ddd`、GCC/Linux `001d846118f05125`）；只在有意的物理/场景变更或 toolchain 升级时更新，devlog 写 old/new + toolchain 与原因；
  纯重构/优化（宽相切换、Tracy 埋点、渲染插值、冷热拆分）hash 必须不变。
- **文档数字纪律（W8 起）**：`docs/profiling.md` §1.5 是性能数字的单一事实来源（canonical 复测）；
  所有"当前"数字必须来自那里并标注口径；历史周数据只留在 devlog/ROADMAP 的周记录里。

---

## 5. v1.1 任务卡（下一轮入口）

> v1.0 已发布；v1.1 的主题是"求解器质量 + profiling + 性能深化"。**按优先级**，每项必须先有测量方案。

### 5.1 2 点接触 2×2 block solver（最高优先级技术债）

- **动机（真实数据）**：逐点 PGS 对 2 点接触的顺序偏差 → 薄片/2 点静止残余抖动尖峰
  （W5 实测 maxV 0.015–0.025 m/s > `kSleepLinearTolerance=0.01`）→ **10 盒塔 859 步才入睡**、
  更薄形状（<0.2 m）几乎无法入睡（Dominoes 因此用 0.30×1.0 m 宽牌）。
- **做法**：在 `ContactSolver::SolveVelocity` 里对 `pointCount==2` 的 manifold 装配 2×2 法向块
  （K 矩阵 + 累计钳制 + 投影到可行域），1 点路径保持不变；保留三模式/宽相等价性测试。
- **验收**：薄片/薄塔入睡步数、jitter、10 盒塔与 Dominoes 数据 before/after；三模式逐位一致；
  **回归 hash 会变化 → 必须更新 golden 并在 devlog 写 old/new 与物理原因**（这是唯一允许的 hash 变化）。
- **风险**：这是发布后第一次动物理；先备份对照矩阵（10 盒塔/薄片/多米诺/金字塔），修复循环上限 3 轮。

### 5.2 Tracy 正式接入（R11）

- `Core/Profiler.h` 宏与 CMake 选项已就位；按设计文档 8.1 只埋 8 个 Step 区段 + `SAP::Sort/Scan`
  + `Plot("AwakeBodies"/"ContactPairs"/"ManifoldPoints")`；截图放 `docs/profiling.md`（需新截图目录）。
- 纯观测：**回归 hash 必须不变**。Tracy 客户端默认 OFF，CI 不构建。

### 5.3 渲染插值（R17）

- `StepAccumulator::GetAlpha()` 已预留；需要 Body 上一帧 transform 缓存（World 追加，不改公共语义），
  testbed 渲染时插值。纯渲染层：物理 hash 不变；加"物理不受影响"的测试。

### 5.4 BodySim/BodyState 冷热拆分（R15）

- **前置条件：Tracy 数据证明 body 布局是热点**。W7 的 packed 测量（0.87x）与 W8 profiling 都没有
  支持这个假设；没有数据不要做（213 个热字段访问点的纯重构，回归风险大、收益未证明）。

### 5.5 其他（按需）

- 多场景回归 golden（R12 扩展：Pyramid200 / 休眠场景）；`scripts/compare_hash.py` 不需要（C++ 单文件够）；
- 岛屿级并行（岛屿天然独立，per-island 并行是起点）；DynamicAABBTree（可选宽相第三实现）；
- CI：clang-tidy 仍未纳入（v1.0 已加 clang-format job）；多配置矩阵按需扩展。

---

## 6. 不该做的（红线 / 已知大坑，务必遵守）

### 6.1 项目范围红线（设计文档 0.5，违反会被拒绝）

- ❌ 不做 3D / 软体 / 流体 / 布料 / GPU 加速 / 完整关节系统 / CCD-TOI / 兼容 Box2D API / 引入游戏引擎。
- ❌ 不为了"更强的 demo"扩张范围；新想法一律进 `docs/ROADMAP.md` 的 Roadmap，不进当前迭代。

### 6.2 发布后（v1.0 已交付）的坑

- ❌ **不要为了"更快入睡"调大 `kSleepLinearTolerance`**：859 步是残余抖动的诚实结果，正解是 block
  solver（v1.1 首位）；阈值保持设计值 0.01。
- ❌ 不要在没有测量方案的情况下做"看起来更快"的重构（W7 的 packed 实测反而慢：默认值只按 Release 定）。
- ❌ 不要在发布后随意改物理：任何物理改动 = golden old/new + devlog + 全量复测；纯重构 hash 必须不变。
- ❌ 文档数字不要复制历史值当"当前值"；性能数字以 `docs/profiling.md` §1.5 为准，保留口径
  （testbed 端到端 1.74x vs 纯全醒/全睡 ≈2.00x；packed 0.87x；split impulse 深穿透 0.10 m 对应
  1.139999 m/s，2 cm 浅重叠对应 0.18 m/s）。
- ❌ 不要把 GIF/PPM/PNG/大文件塞进 git：只提交 `docs/images/*.gif`（每张 ≤600KB、总 ≤2.5MB，
  由脚本校验）；PPM 中间文件只放 `build/capture/` 并在编码后删除。
- ❌ 改 header 后不要相信增量构建（本机 depfile 为空）——`build.ps1` 会自动 clean-first，但 canonical
  验收必须删 build dir 重编。
- ❌ 不要在别人构建时删除/清理 `build/`；并行开发一律独立 `-BuildDir build/<name>`。
- ❌ 共享工作区并行开发时 build dir 可能链接旧 obj；最终结论一律以 canonical clean build 为准。
- ❌ 多 agent 并行写文档前先冻结"数字来源 + 当前/历史口径"（W8 复盘见 devlog W8 卡点 1），
  否则收尾会对账很久。

### 6.3 协作红线

- 每个 teammate 只改**任务写作用域**内的文件；Header/CMake/根文档（README/AGENTS/HANDOFF/ROADMAP/
  dev-interfaces/testing）由 Lead 独占。
- 跨模块接口问题先消息 Lead，不要直接改对方文件；Lead 用"手算解析解"裁决测试期望，
  **不通过弱化断言来消除失败**。
- 不要运行 `git commit/push/tag`（只有 Lead 做）；teammate 不创建分支。
- 修复循环上限 3 轮：超过就拆任务卡或人工读代码，禁止无限 AI 自循环。
- 发布后新增测试仍要覆盖边界；新 death test 在沙箱会 SKIP → 用工作区外跑一次完整 gtest 验契约。

---

## 7. 已知技术债 & 待办（v1.1）

| # | 技术债 | 位置 | 计划 |
| --- | --- | --- | --- |
| 1 | **2 点接触 2×2 block solver**：逐点 PGS 顺序偏差 → 薄片/2 点残余抖动、入睡慢（10 盒塔 859 步） | `ContactSolver::SolveVelocity` | **v1.1 最高优先级**；改后回归 hash 需 old/new + 解释 |
| 2 | Tracy 只有零成本宏（R11） | `Core/Profiler.h`、CMake | v1.1（§5.2） |
| 3 | 渲染插值未实现（R17） | `StepAccumulator`/testbed | v1.1（§5.3） |
| 4 | `BodySim`/`BodyState` 冷热拆分（R15） | `Body.h`/`World`/`ContactSolver` | v1.1 候选，**先有 Tracy 数据证明热点** |
| 5 | 按岛 body 打包默认关闭（0.87x） | `SolverSettings::packIslandBodies` | 保留 opt-in；>cache 规模场景可再测 |
| 6 | 回归工具单场景 golden（可扩展到多场景） | `tests/regression/` | v1.1 可选（R12 扩展） |
| 7 | `ContactSolver::SolvePosition` 每帧分配局部 vector（未证明热点） | `ContactSolverPosition.cpp` | 先测量再做 |
| 8 | 30:1 单列/金字塔底部在默认 8 迭代下的穿透（载荷/迭代预算） | `ContactSolver` | 记录在 README，不修 |
| 9 | 薄形状几乎无法入睡（同 #1） | solver | 同 #1 |
| 10 | CI 无 clang-tidy / 无多配置矩阵（已有 clang-format job） | `.github/workflows/ci.yml` | v1.1 按需 |

---

## 8. 已登记的偏差（避免重复讨论）

完整列表见 `docs/dev-interfaces.md` §5；高频的几条（W1-W8 全部）：

- **依赖**：vcpkg → FetchContent；**Ninja 不放仓库**；sandbox death tests 自动 SKIP（CI/普通机器执行）。
- **Contact 文件位置**：`Solver/Contact.h` → `Collision/Contact.h`；**ContactID**：`Key()` 不含 flip、
  incidentVertex 绝对索引、SAT tie-break 对称。
- **W5**：`kWarmStartFactor` 0.85→1.0（实测 <1.0 都倒）、`kSplitImpulseSlop`→`kLinearSlop`、
  `useSplitImpulse=false` = Baumgarte、`Prepare` 追加 dt。
- **W6**：`Body::aabb`=fat AABB、`UpdateContacts` 保留 O(N²) fallback、候选对规范化、SAP 惰性排序 +
  mutable + erase、回归用 txt golden + 量化弱 hash、W6 无 `DetachShape`、bench 真实重叠口径 = tight AABB。
- **W7**：static/kinematic **不传递岛屿连通性**；proximity wake 在 `UpdateContacts` 共享 pass（不是
  `UpdateBroadPhase`）；`packIslandBodies=false`；休眠阈值保持 0.01（10 盒塔 859 步入睡如实记录）；
  Dominoes 用 0.30×1.0 m 宽牌；`ContactSolver` 休眠跳过谓词收窄为"双方 `!isAwake` 且非 static-static"；
  **冷热拆分第 4 次顺延**（W8 决定不做，v1.1 候选）。
- **W8**：可选优化（block solver / Tracy / 渲染插值 / 冷热拆分）全部未做 → v1.1；
  testbed 新增确定性帧捕获（`--capture-*`，默认零开销，物理未改）；
  边界硬化（`PolygonShape` 溢出守卫、`ContactSolver`/`SolvePosition` assert、`World` Release 负迭代钳 0、
  `CheckSane` 覆盖 W7 状态）——只影响非法输入/Debug 校验，回归 hash 不变；
  文档新增 6 个文件、README 中英双语、CI 加 clang-format job。
- **testbed**：控制台子系统（为了 stdout smoke）；Pyramid200 名字沿用设计文档（默认 16 层=136 盒）。

---

## 9. 团队协作流程（本仓库已验证有效，直接复用）

### 9.1 角色

- **Lead（你）**：写/冻结公共头 + CMake + 根文档；分任务；裁决跨模块测试期望；最终 clean 验证 +
  git 提交/tag；唯一动 git 的人。
- **Teammates（durable、可复用）**：`solver-dev`（数学/几何/求解器：W1 Math、W3 Solver、W4 SAT/解析、
  W5 摩擦/warm start/split impulse、W6 SAP+bench、W7 岛屿 DFS+三模式位级一致、W8 math.md+边界测试）、
  `core-dev`（World/集成测试/回归/量化：W5 稳定性证据、W6 三路等价、W7 休眠/唤醒/岛屿集成、
  W8 profiling.md + Core 边界测试）、`testbed-dev`（UI/demo/录制：W5 面板、W6 场景、W7 Dominoes/
  IslandSleep500/单步慢放、W8 帧捕获 + 5 GIF + demos.md）、`docs-dev`（W8 新增：
  README.en.md、boundary-contracts.md）。
  换会话时它们仍存在但 inactive；用 `send_message` 唤醒（`list_agents` 查看目标名）。
  注意：teammate 的上下文**不带**本会话的记忆，任务卡必须自包含；它们已完成的交付物在仓库里。

### 9.2 一轮标准流程

1. `create_goal`（可选，长任务）→ 读设计文档对应章节 + 现有头文件；
2. **Lead Phase A**：冻结接口/写任务卡（交付文件清单、实现要求、测试要求、构建命令、write scope、
   完成标准）；一次写全公共头 + CMake + 占位 .cpp，先 `-Target physics2d_core` 验证头文件可编译；
3. `team_task_create` 每模块一张卡；`send_message` 让每个 teammate claim 并开工；
4. 并行开发：每人独立 `-BuildDir build/<name>`；跨模块 transient link 失败用 `-Target physics2d_core`
   先验证编译；接口问题消息 Lead，不互相改文件；
5. **Lead 整合验证**：`format` → 删 canonical build → 三 preset 全绿 → 8 个 demo smoke → `--broadphase`
   A/B → 回归 `--check` → 工作区外 death test 全量 → bench/数字 canonical 复测 → 核对文档；
6. **Lead 收尾**：更新 docs → Conventional Commits（按模块拆）→ annotated tag → 汇报。

### 9.3 任务卡模板（复制即可）

```text
仓库：<repo-root>
目标：<主题>，对应设计文档第 X 周 / PXX。
交付文件（只允许改/建这些）：<清单>
实现要求：<公式/语义/边界/错误处理>
单测要求：<手算期望值、边界、DoD>
构建命令：powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/<name> -NoTestbed -RunTests
约束：不改 include/CMake/其它 docs；不 git；零 /W4 警告；完成后标记 task complete 并给 Lead 短报告。
```

### 9.4 单人开发也行

不派 agent 时流程同 Phase A/C：接口先行、测试手算、最后统一验证这三个习惯不要丢；
v1.1 的 block solver 建议 single-agent 串行做（跨求解器/测试/文档，且会改回归 hash）。

---

## 10. 常用命令速查

> 每个命令的期望输出、如何只跑一组测试、如何做交互式人工验收、bench/回归/death tests/GIF 的完整
> 说明见 [`docs/testing.md`](docs/testing.md)。下面是最高频的几条。

```powershell
cd "<repo-root>"

# 构建 + 测试（日常；Windows 上 header 比 obj 新会自动 clean-first）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests -Clean

# 并行开发隔离目录（多人/多 agent）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/mywork -NoTestbed -RunTests

# Release / Sanitize（MSVC 是 ASan；UBSan 走 Linux CI）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -RunTests

# 格式化
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check

# 测试台（交互）与冒烟（8 个 demo）
.\build\debug\testbed\physics2d_testbed.exe
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden --broadphase sap

# bench（必须 Release）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42

# 确定性回归
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# GIF 重录（5 张，Python+Pillow）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
```

关键常量/入口：`include/physics2d/Core/Settings.h`（所有魔法数字）、
`include/physics2d/Core/World.h`（Step 注释 + API + 岛屿/休眠入口）、
`include/physics2d/Solver/ContactSolver.h`（求解器契约 + 三模式）、
`include/physics2d/Solver/Island.h`（岛屿分区契约）、
`src/Solver/ContactSolverPosition.cpp`（split impulse 位置层）、
`include/physics2d/Core/BroadPhaseInterface.h` + `Collision/BroadPhaseSAP.h`（宽相契约）、
`include/physics2d/Collision/ContactID.h`（Key 语义）。
测试可执行文件：`build/debug/tests/physics2d_tests.exe`（`--gtest_filter=` 可单独跑）。

---

## 11. 新会话开场提示词（直接复制粘贴）

> 你是 Physics2D 项目的接手开发者。请先读
> `<repo-root>\HANDOFF.md`、
> 同目录 `AGENTS.md`、`docs/ROADMAP.md`、`docs/dev-interfaces.md`、`docs/testing.md`、
> `docs/profiling.md`、`docs/boundary-contracts.md`，
> 再按 `HANDOFF.md` §2 跑验证基线（预期 debug 387 / release 390 / sanitize 387 个 ctest 全绿、
> 0 警告、8 个 smoke 全 exit 0、回归 hash 按 toolchain（MSVC `588d30e70f9b4ddd` / GCC `001d846118f05125`）、工作区外 386/386 gtest）。
> 当前 v1.0-release 已完成，**物理与公共 API 冻结**；下一轮 v1.1 按 `HANDOFF.md` §5 的优先级执行，
> 首位是 **2 点接触 2×2 block solver**（会改回归 hash，需 golden old/new + 全量复测）。
> 严格遵守 `HANDOFF.md` §6 的"不该做的"、以 `docs/profiling.md` §1.5 为性能数字唯一来源，
> 用 §9 的团队流程；结束时更新 HANDOFF 状态并汇报真实数据。

---

## 12. 本次交接完成检查清单（写此报告时已确认）

- [x] 工作区干净，HEAD = tag `v1.0-release`
- [x] canonical clean：debug **387** / release **390** / sanitize **387** 个 ctest 全绿、0 项目警告
      （31 个 sandbox death-test SKIP）；clean 构建 50.1 / 53.6 / 86.0 s
- [x] 工作区外 gtest **386/386 全过**（31 个 death test 真正执行）
- [x] 八个 demo smoke 全 exit 0，数据已记入 §2（canonical debug）
- [x] 回归弱 hash **`588d30e70f9b4ddd`（MSVC）/ `001d846118f05125`（GCC·Linux）** 不变、exit 0；宽相 A/B 三场景 SHA-256 三路相同
- [x] `scripts/format.ps1 -Check` 117 文件 OK；CI 新增 clang-format job
- [x] 文档：`docs/math.md`、`docs/profiling.md`、`docs/boundary-contracts.md`、`docs/demos.md`、
      `docs/release-notes-v1.0.md`、README v1.0 + `README.en.md` 全部落盘，
      architecture/ROADMAP/dev-interfaces/devlog/testing/AGENTS/HANDOFF 更新到 v1.0
- [x] GIF：5 张（总计 1,692.6 KB，单张 ≤600KB），`scripts/record_gifs.ps1` 一条命令可复现
- [x] 边界契约：35 个新测试 + 4 处契约硬化（只影响非法输入/Debug 校验，hash 不变）
- [x] 可选优化未做（block solver/Tracy/渲染插值/冷热拆分 → v1.1，理由已登记）
- [x] 技术债、偏差、红线、团队流程、v1.1 任务卡、验收数据全部写入本文件
- [x] 无遗留后台任务/未完成 team task；所有 teammate inactive，可唤醒复用

> 下一会话开始后，第一件事：`cd` 到仓库、跑 §2 基线、确认 HEAD/tag 与工作区状态，再按 §5 决定
> 是否开 v1.1（block solver 建议先建 goal + 任务卡 + 对照矩阵，再动物理）。
