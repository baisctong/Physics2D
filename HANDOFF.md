# Physics2D 交接报告（Session Handoff）

> **一句话现状**：Week 1-7 已完成并验证通过（tag `v0.7-island-sleep`，**354 个测试全绿、0 警告、
> 工作区干净**）：**岛屿分区 + 三种按岛求解模式（逐位一致）+ 整岛原子休眠/唤醒**已落地，
> 500 物体休眠后 Step **1.73x**、多米诺 20/20 连锁唤醒无跳过、回归弱 hash 保持不变。
> **下一步**：Week 8 —— 文档与交付周（profiling/math/architecture 升级、README + GIF、已知限制与
> 边界测试、简历与面试演练、tag `v1.0-release`；可选 block solver / Tracy / 渲染插值）。
>
> 新会话请按顺序读：
> 1. 本文件（10 分钟，含"该做/不该做/踩过的坑"）
> 2. [`AGENTS.md`](AGENTS.md)（项目宪法 + 工作协议，**必须遵守**）
> 3. [`docs/ROADMAP.md`](docs/ROADMAP.md)（8 周计划）、[`docs/dev-interfaces.md`](docs/dev-interfaces.md)
>    （预留接口 + **偏差登记** + W5/W6/W7 不变量）
> 4. 需要细节时再查 [`docs/architecture.md`](docs/architecture.md) 与 [`docs/devlog.md`](docs/devlog.md)
>
> 原始设计文档（仓库外）：`E:\Big Fat Fish's Workstation\2D物理引擎项目\2D物理引擎项目完整工作流与DeepSeek提示词库.md`
> 仓库路径：`E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D`
> 当前 HEAD：tag `v0.7-island-sleep` **之后**的一个 docs 提交（新增 `docs/testing.md` 手动测试指南；
> `git log -1` / `git describe --tags` 为准）。代码与测试状态对应 tag `v0.7-island-sleep`。

---

## 0. 新会话开场 3 步（别跳过）

1. **读**本文件 §1-§4 + `AGENTS.md`，确认你理解固定 Step 顺序、符号约定、分层铁律、W5/W6/W7 不变量。
2. **跑验证基线**（§2）。不先确认基线的会话，后面所有回归都不可信。具体命令与期望输出、单测过滤、
   交互验收脚本见 [`docs/testing.md`](docs/testing.md)。
3. **按 §5 Week 8 任务卡开工**，用 §9 的团队流程；结束时按 §10 清单收尾（commit + tag + 更新本文件状态）。

---

## 1. 仓库与状态

| 项 | 值 |
| --- | --- |
| 仓库路径 | `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D` |
| Git | `main` 分支，工作区干净，tags `v0.1-falling-ball` → … → `v0.6-sap` → **`v0.7-island-sleep`** |
| 构建目录 | `build/`（**可随时整个删除**，已被 gitignore；canonical `debug/release/sanitize` + 各 agent 并行目录） |
| 团队状态 | 所有 teammate 为 inactive；无后台任务；W7 任务（task-7/8/9）均 completed |
| 环境 | Windows 11 / i7-12650H / 16GB / VS2022 (MSVC 19.44) / CMake 3.31.12 / Ninja 1.12.1（VS 自带）/ RTX 4060 |

文档地图：

| 文件 | 作用 |
| --- | --- |
| `AGENTS.md` | 项目宪法 + 工作协议 + 技术债 + 下周入口（**每次会话必读**） |
| `docs/ROADMAP.md` | 8 周计划、每周 DoD、已完成周的验收记录 |
| `docs/dev-interfaces.md` | 18 条预留接口状态、**偏差登记（重要）**、W5/W6/W7 不变量 |
| `docs/architecture.md` | 模块职责、数据布局、Step 流水线、宽相/岛屿/休眠数据流与算法 |
| `docs/devlog.md` | 每周周报：完成/卡住/数据/下周/提醒（面试素材来源） |
| `docs/dev-setup.md` | 环境、构建、测试台操作、**疑难杂症（含本机增量构建坑与自动防护）** |
| `docs/testing.md` | **手动测试指南**：30 秒版 / 全量验收 / 单测过滤 / 交互测试台 10 分钟人工验收脚本 / smoke / bench / 回归 / death tests / FAQ / 验收清单（**人类用户与新手 AI 都先看这个**） |
| `README.md` | 对外展示（W1-7 特性、真实性能表、Roadmap、已知限制） |
| `HANDOFF.md` | 本文件：换会话交接 |

---

## 2. 验证基线（开工前先跑，确认环境没变）

> **想自己手动验收/玩一玩？** 完整的分层指南（30 秒版、单测过滤、交互测试台 10 分钟人工验收脚本、
> smoke/bench/回归/death tests、FAQ、验收清单）在 [`docs/testing.md`](docs/testing.md)。
> 下面是最短的官方基线，必须首先通过。

> 本机只有 **Windows PowerShell 5.1（没有 `pwsh`）**，且首次 configure 会联网 FetchContent
> 下载依赖（1-2 分钟）。命令必须在仓库根目录执行。

```powershell
cd "E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D"

# 全量：配置 + 编译 + ctest（幂等；Windows 上若检测到 header 比 obj 新会自动 clean-first）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
```

**期望基线（Week 7 结束时实测）**：

| 检查项 | 期望值 |
| --- | --- |
| 构建 | exit 0，MSVC `/W4 /permissive-` **0 个项目警告** |
| ctest | **354/354 passed, 0 failed, 15 skipped**（skipped 全是 sandbox death tests；含 1 个 `regression_default`） |
| Release | **355/355 passed**（多 1 个 release-only 契约测试） |
| Sanitize | `-Preset sanitize -RunTests` 同样 354/354 |
| clean 构建耗时 | debug **55.4 s** / release **53.5 s** / sanitize **82.9 s**（含依赖编译） |
| 测试 exe 工作区外运行 | **353/353 gtest 全过**（15 个 death test 真正执行；这是发现 W5/W7 契约回归的唯一可靠尺子） |

八个 demo 冒烟（**真实输出基线**；`islands/awake/sleeping` 为 World 统计）：

```powershell
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall  --smoke 150 --hidden
# bodies=2  islands=1 awake=0 sleeping=1 contacts=1 max_pen=5.057mm
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack  --smoke 300 --hidden
# bodies=4  islands=1 awake=0 sleeping=3 contacts=3 max_pen=3.063mm
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack     --smoke 600 --hidden
# bodies=11 islands=1 awake=10 sleeping=0 contacts=10 max_pen=3.893mm（900 步后入睡）
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
# bodies=2 islands=1 awake=0 sleeping=1 contacts=1 max_pen=2.686mm
.\build\debug\testbed\physics2d_testbed.exe --demo Pyramid200   --smoke 300 --hidden
# bodies=137 islands=1 awake=136 contacts=324 max_pen=41.863mm broadphase=SAP
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden
# bodies=401 islands=4 awake=400 contacts=820 broadphase=SAP
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes     --smoke 600 --hidden
# bodies=21 islands=5；dominoes woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1
.\build\debug\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden
# bodies=501 islands=100 awake=0 sleeping=500 contacts=500；frame 105 全睡；
# debug: early 4.95ms -> asleep 2.94ms (1.69x)；release: 0.4554 -> 0.2627 ms (1.73x)
```

宽相 A/B（**切宽相物理结果必须完全一致**）：`--broadphase none|bf|sap`，同一 demo 的 body/contact
输出行应逐字节相同（只有 candidate_pairs 不同）。

回归工具（golden 必须绿，W7 后弱 hash **不变**）：

```powershell
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
# 期望：hash=588d30e70f9b4ddd（quantum 1e-5）、exit 0；mismatch exit 1；文件/参数错误 exit 2
```

bench（**Release**，数字必须可复现，表格见 README）：

```powershell
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42
# 期望：N=50/200/500/2000 SAP 加速比 2.20x / 4.96x / 5.89x / 7.81x；小 N 交叉 N≈25-32
```

> `testbed` 可执行文件路径是**嵌套的**：`build/<dir>/testbed/physics2d_testbed.exe`。
> 无 OpenGL 上下文时 smoke 会返回退出码 2 并打印原因（当前机器可用，NVIDIA 3.3）。

**怀疑环境坏了时**（先看 `docs/dev-setup.md` §6）：

- `pwsh` 不存在是正常的，用 `powershell`；
- Ninja 必须在仓库外（`scripts/build.ps1` 自动从 VS 安装目录找）；把 Ninja 放进 `tools/` 会触发
  `cl D8050` / `Permission denied`（沙箱限制）；
- **本机增量构建坑（重要）**：中文 locale 的 cl.exe 把 `/showIncludes` 打成 `注意: 包含文件:`，
  CMake 写入 `rules.ninja` 的是 UTF-8 前缀，Ninja 按字节匹配失败 → **depfile 为空，改 header 不重编**
  → 类布局混链会 **HEAP CORRUPTION**（W7 真实踩到）。`scripts/build.ps1` 已自动检测"header 比 obj 新"
  并强制 `--clean-first`；canonical 周验收仍须删 `build/<preset>` 干净重编；
- `ctest --preset` 必须在仓库根目录跑（或用 `--test-dir`）；
- death tests 在本沙箱自动 SKIP；把 `build/debug/tests/physics2d_tests.exe` 复制到
  `%LOCALAPPDATA%\Physics2D\testrun\` 再运行可得到 **353/353 gtest 全过**（证明 CI 会执行它们）；
- **不要用 PowerShell 5.1 的 `Get-Content`/`Set-Content` 改 UTF-8 中文文档**（会乱码）；
  用编辑工具（本 harness 的 write/edit，或 `-Encoding UTF8` 且显式处理 BOM）。

---

## 3. 已完成内容（Week 1-7）

| 周 | 主题 | 关键交付 | 数据亮点 |
| --- | --- | --- | --- |
| W1 | 数学 + 构建 + 渲染循环 + 积分骨架 | `p2::Math`、`Handle`+`IDPool`、`Body`/`World`、11 阶段 `Step`、`StepAccumulator`、GLFW+ImGui testbed、CMake/CI | 120 测试；clean build 23.6s |
| W2 | 形状 + 解析碰撞 + Contact 持久化 | `Shape` 池（Circle/Plane）、解析窄相、`Contact`/`ContactID` 帧间匹配 | 185 测试；静止穿透 7.8mm |
| W3 | 法向 Sequential Impulse | `ContactSolver`（K_n、累计钳制、迭代、restitution bias）、SphereStack | 210 测试；8 迭代残余 0.00065 m/s |
| W4 | 凸多边形 SAT + 裁剪 + SATCache | `PolygonShape`、`SAT`/`Clipping`/`CollidePolygons`、`ContactID.Key()` 去 flip | 278 测试；10 盒塔会塌（W4 基线） |
| W5 | 摩擦锥 + warm starting + split impulse | `ContactSolver` 摩擦/warm start/Baumgarte 基线；`ContactSolverPosition.cpp` 伪速度层；DebugUI 4 面板 + FrictionRamp | **10 盒 10 s 稳定（穿透 5.223mm）** |
| W6 | BruteForce + SAP 宽相 + 量化 + 回归 | 双宽相实现、World 接线 + fat AABB + pair 规范化、`bench_broadphase`、`physics2d_regression` + golden + CI job、Pyramid200/StressBenchmark | **2000 物体 SAP 7.81x**；切宽相物理**逐位不变** |
| W7 | 岛屿 + 按岛求解 + 整岛休眠 | `Solver/Island.{h,cpp}`（显式栈 DFS/规范序）、`ContactSolver` 三模式按岛求解、`World::BuildIslands/UpdateSleep/WakeIslandOfBody`（整岛原子休眠/唤醒、休眠岛跳过宽相 AABB 更新/窄相/求解、宽相邻近唤醒）、Dominoes/IslandSleep500、单步/慢放、Contact 冲量柱状条 | **354 测试**；三求解模式 21,760 float 比较 0 差异；休眠 **1.73x**；多米诺 20/20 无跳过；回归 hash 不变 |

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
tests/unit/*.cpp                      GLOB 自动收集（加文件不用改 CMake）
tests/regression/                     regression_main.cpp + golden/default_600.txt + README
bench/bench_broadphase.cpp            只依赖 core 的宽相基准
testbed/                              main / Renderer / Camera / DebugUI / Demo.h / SceneBuilders.h /
                                      demos/{FallingBall,SphereStack,BoxStack,FrictionRamp,
                                             Pyramid200,StressBenchmark,Dominoes,IslandSleep500}
docs/  scripts/
```

---

## 4. 架构速览（W8 要改的地方）

### 4.1 World::Step 固定流水线（顺序不可变！）

```text
1 IntegrateForces            [done W1；W7 跳过休眠体]
2 UpdateBroadPhase(dt)       [done W6；W7 休眠 body 跳过 AABB 更新、proxy 保留]
3 UpdateContacts             [done W2/W4/W6；W7 共享 proximity wake pass + 休眠-休眠 pair 复用上一帧 contact]
4 BuildIslands               [done W7：DFS 显式栈 + 规范序 + islandId]
4b PrepareConstraints(dt)    [done W5；W7 休眠岛不建约束]
5 WarmStart                  [done W5；W7 按岛]
6 SolveVelocityConstraints   [done W5/W7：岛分组三模式，休眠岛整岛跳过]
7 IntegratePositions         [done W1；跳过休眠体]
8 SolvePositionConstraints(dt) [done W5；W7 按岛 + 休眠岛跳过]
9 Synchronize                [done W2/W6：tight AABB + kAabbMargin 的 fat AABB]
10 UpdateSleep(dt)           [done W7：整岛原子计时/入睡]
11 Validate                  [done W1：NaN/Inf 扫描，含伪速度]
```

- 第一阶段（`IntegrateForces`）先给动态体加重力速度，然后才窄相/求解——**不要改变顺序**。
- 接触锚点是**生成时刻**的几何（`IntegratePositions` 之前）；测试不要用 step 之后的 transform 反投影。
- W8 是文档周：**默认不动物理**；若做可选优化（block solver / 插值），先读 §4.3 不变量与
  `docs/dev-interfaces.md` 第 6 节，回归弱 hash 必须保持或解释变化。

### 4.2 数据流与所有权（W8 直接依赖）

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
- `Settings.h` 是唯一常量来源；休眠阈值 `kSleepLinearTolerance=0.01`/`kSleepAngularTolerance`/
  `kSleepTimeThreshold=0.5` 保持设计值。

### 4.3 W5/W6/W7 不变量（W8 不要破坏；改前必读）

- **宽相只决定"谁进窄相"**：候选对规范化为 `(i<j, sorted, deduped)`；切宽相物理逐位一致；
  proxy 生命周期由 World 管；fat AABB 只加候选。
- **摩擦锥钳制累计值**、法向先解切向后解、`μ = contact.friction`。
- **warm start 只施加一次**（factor 默认 1.0）；**split impulse 只碰伪速度**。
- **岛屿分组求解必须逐位等价**：三种求解模式在同一初始状态下 body 状态与 contact 累计冲量逐位一致；
  规范序（岛按最小成员、岛内 bodies 升序、contacts 全局序）是前提。
- **休眠整岛原子**：不允许单 body 冻结/唤醒；休眠岛保留 proxy 与上一帧 contact；休眠岛跳过 AABB
  更新/窄相/warm start/速度与位置求解；`WakeIslandOfBody` 整岛唤醒；proximity wake 必须在
  `UpdateContacts` 共享 pass（不要放 `UpdateBroadPhase`——那里无宽相会提前 return）。
- **阈值不调**：入睡慢是收敛质量（残余抖动）问题，正解是 2 点接触 block solver。
- **回归 hash 纪律**：`588d30e70f9b4ddd` 只在有意的物理/场景变更时更新，devlog 写 old/new 与原因；
  纯重构/优化（block solver 之外的顺序变换、Tracy 埋点、渲染插值）hash 必须不变。

---

## 5. 该做的：Week 8 任务卡（文档与交付周）

> 设计文档 Week 8：架构图、`docs/math.md`、`docs/profiling.md`、README 完整版 + GIF、已知限制与边界
> 测试、简历 bullet 与面试演练、tag `v1.0-release`。**默认不改物理**；可选优化必须测量驱动。

### 5.1 目标与验收（DoD）

| 项 | 验收标准 |
| --- | --- |
| 架构图 | 一张图讲清：模块依赖 + Step 11 阶段时序 + 岛屿/休眠状态机 + 宽相/窄相/求解数据流（Mermaid 进 `docs/architecture.md`，README 摘要） |
| `docs/math.md` | SAT 轴/分离推导、法向/切向冲量方程、摩擦锥、restitution 阈值、split impulse bias、warm start 收敛解释；每个公式能手推/口述 |
| `docs/profiling.md` | 汇总全部真实性能表（SAP vs BruteForce、休眠前后 Step、三求解模式、构建耗时）+ **测量环境**；Tracy 若接入附时间线（R11 可选） |
| README 完整版 | GIF（10 盒稳定 / 2000 物体 SAP / 500 物体休眠 / 多米诺唤醒）+ 快速开始（陌生人 5 分钟可编译运行）+ 架构 + 性能表 + 已知限制 + Roadmap |
| 已知限制/边界测试 | 退化多边形、零密度、极大速度、超长薄条、极端 mass ratio、NaN 输入——所有已知崩溃点有 assert 或明确文档；`CheckSane`/`Validate` 覆盖伪速度/islandId/sleepTime |
| 简历/面试 | 按设计文档 10.3 写 bullet（含真实数字）、10.4 的 25 问能闭卷答 20 分钟 |
| 质量 | 三 preset clean 全绿、0 警告；8 demo smoke + `--broadphase` A/B + 回归 `--check` + 工作区外 death test 全量 |
| tag | **`v1.0-release`**（annotated）+ GitHub Release 说明（可选发社区） |
| 可选优化 | 仅在文档完成后、且测量证明：**2 点接触 2×2 block solver**（最高优先，解决薄片抖动/更快入睡，改后回归 hash 可能变化需解释）、Tracy 实接、R17 渲染插值、R15 冷热拆分（先 profiling 证明 body 布局是热点） |

### 5.2 Phase A/B/C（沿用已验证流程）

- **Phase A（Lead）**：定 W8 文档结构（`docs/math.md`/`docs/profiling.md` 新建、README/architecture
  模板）；若做 Tracy 实接，Lead 改 CMake 选项与 `Core/Profiler.h` 埋点清单；边界测试清单拆卡。
- **Phase B（3 个 teammate 并行，按文档/测试拆分，注意写作用域不重叠）**：
  - `solver-dev`：`docs/math.md`（推导 + 与代码对照）+ 边界测试（退化多边形/零密度/极大速度/超长薄条/
    极端 mass ratio/NaN 的 assert 与文档）+（可选）block solver 原型 + 测量；
  - `core-dev`：`docs/profiling.md`（汇总所有真实数字 + 环境）+ 边界测试（World 生命周期/休眠/
    宽相/回归工具）+ 回归多场景（可选）；
  - `testbed-dev`：GIF 录制（8 个 demo 的关键画面）+ README 的 demo/操作/面板说明 + 交互验证；
  - Lead：读写架构总图、README 主体、AGENTS/HANDOFF/ROADMAP/dev-interfaces 收尾、简历素材与
    面试问答整理（可让 teammate 起草再由 Lead 合并）。
- **Phase C（Lead）**：格式 → 三 preset clean 全绿 → 8 demo smoke + A/B + 回归 + 工作区外 death test →
  文档一致性检查（数字/链接/命令都可复现）→ 提交（Conventional Commits 3-5 个）→ tag `v1.0-release`
  → 汇报（含真实数据与已知限制）。

### 5.3 边界测试清单（设计文档 10 章 DoD，W8 必须覆盖）

- 退化多边形（面积≈0/凹/顺时针/重复顶点/共线）、零/负密度、非有限输入；
- 极大速度（> kMaxTranslation 钳制、无 NaN）、超长薄条（1×100 m 盒）、极端 mass ratio（1:1e6）；
- `World::CreateBody`/`AttachShape`/`DestroyBody` 的非法/重复调用；stale handle；
- `Step(dt)` 的非法 dt/迭代数（assert + release 安全返回）；
- 休眠边界：刚入睡即唤醒、邻居宽相重叠、`allowSleep=false`、`enableSleeping=false`；
- 宽相切换/无宽相 fallback 等价（已有）；回归 golden 的 exit code 语义（已有）。

---

## 6. 不该做的（红线 / 已知大坑，务必遵守）

### 6.1 项目范围红线（设计文档 0.5，违反会被拒绝）

- ❌ 不做 3D / 软体 / 流体 / 布料 / GPU 加速 / 完整关节系统 / CCD-TOI / 兼容 Box2D API / 引入游戏引擎。
- ❌ 不为了"更强的 demo"扩张范围；新想法一律进 `docs/ROADMAP.md` 的 Roadmap，不进当前周。

### 6.2 W8 专属的坑

- ❌ **文档数字必须来自实测且标注条件**；不要复制旧数字/手算猜测；表格里每个数字要能一条命令复现。
- ❌ 不要为了 GIF 好看而调物理参数（休眠阈值、迭代数、阻尼）；demo 用默认设置录。
- ❌ 可选优化（block solver / 冷热拆分 / Tracy）**只有在文档与最终验收完成后**才做，且必须用测量
  证明收益；做之前先读 `docs/dev-interfaces.md` 第 6 节不变量，回归弱 hash 必须保持或把 old/new
  写进 devlog。
- ❌ 不要 weaken/删除任何既有测试来让优化变绿；block solver 若改变薄片入睡步数，更新的是期望值 +
  注释物理原因，不是删除断言。
- ❌ 不要把 GIF/大文件塞进 git（用外部链接或 Git LFS；README 里放链接）。
- ❌ 改 header 后不要相信增量构建（本机 depfile 为空）——`build.ps1` 会自动 clean-first，但 canonical
  周验收必须删 build dir 重编。
- ❌ 不要在别人构建时删除/清理 `build/`；并行开发一律独立 `-BuildDir build/<name>`。
- ❌ 共享工作区并行开发时 build dir 可能链接旧 obj；最终结论一律以 canonical clean build 为准。

### 6.3 协作红线

- 每个 teammate 只改**任务写作用域**内的文件；Header/CMake/docs/提交由 Lead 独占。
- 跨模块接口问题先消息 Lead，不要直接改对方文件；Lead 用"手算解析解"裁决测试期望，
  **不通过弱化断言来消除失败**。
- 不要运行 `git commit/push/tag`（只有 Lead 做）；teammate 不创建分支。
- 修复循环上限 3 轮：超过就拆任务卡或人工读代码，禁止无限 AI 自循环。

---

## 7. 已知技术债 & 待办（W8）

| # | 技术债 | 位置 | 计划 |
| --- | --- | --- | --- |
| 1 | **2 点接触 2×2 block solver**：逐点 PGS 的顺序偏差 → 薄片/2 点静止残余抖动（maxV 0.015–0.025 m/s > 睡眠阈值 0.01）→ 入睡慢（10 盒塔 859 步）、更薄形状几乎无法入睡 | `ContactSolver::SolveVelocity` | **W8 可选（最高优先技术债）**；改后回归 hash 需解释/更新 |
| 2 | `BodySim`/`BodyState` 冷热拆分（W4→…→W8 顺延） | `Body.h`/`World`/`ContactSolver` | W8 **可选**；先 profiling 证明 body 布局是热点（W7 实测打包慢 18% 说明不是） |
| 3 | 按岛 body 打包默认关闭（0.82x） | `SolverSettings::packIslandBodies` | 保留 opt-in；>cache 规模场景可再测 |
| 4 | Tracy 只有零成本宏（R11） | `Core/Profiler.h`、CMake | W8 可选（profiling 文档需要数字时优先） |
| 5 | 渲染插值（R17） | `StepAccumulator`/testbed | W8 可选 |
| 6 | CI format 检查/clang-tidy 未纳入 | `.github/workflows/ci.yml`、scripts | W8（本机无 LLVM clang-tidy；clang-format 可 bootstrap） |
| 7 | 回归工具单场景 golden（可扩展多场景） | `tests/regression/` | W8 可选 |
| 8 | `ContactSolver::SolvePosition` 每帧分配局部 vector（未证明热点） | `ContactSolverPosition.cpp` | 先测量 |
| 9 | 30:1 单列/金字塔底部在默认 8 迭代下的穿透（载荷/迭代预算） | `ContactSolver` | 记录在 README，不修 |
| 10 | 薄形状几乎无法入睡（同 #1） | solver | 同 #1 |

---

## 8. 已登记的偏差（避免重复讨论）

完整列表见 `docs/dev-interfaces.md` §5；高频的几条（W1-W7 全部）：

- **依赖**：vcpkg → FetchContent；**Ninja 不放仓库**；sandbox death tests 自动 SKIP（CI/普通机器执行）。
- **Contact 文件位置**：`Solver/Contact.h` → `Collision/Contact.h`；**ContactID**：`Key()` 不含 flip、
  incidentVertex 绝对索引、SAT tie-break 对称。
- **W5**：`kWarmStartFactor` 0.85→1.0（实测 <1.0 都倒）、`kSplitImpulseSlop`→`kLinearSlop`、
  `useSplitImpulse=false` = Baumgarte、`Prepare` 追加 dt。
- **W6**：`Body::aabb`=fat AABB、`UpdateContacts` 保留 O(N²) fallback、候选对规范化、SAP 惰性排序 +
  mutable + erase、回归用 txt golden + 量化弱 hash、W6 无 `DetachShape`、bench 真实重叠口径 = tight AABB。
- **W7**：static/kinematic **不传递岛屿连通性**（设计措辞歧义，按"3 独立堆=3 岛"解释）；
  proximity wake 在 `UpdateContacts` 共享 pass（不是 `UpdateBroadPhase`）；`packIslandBodies=false`
  （Release 0.82x）；休眠阈值保持 0.01（10 盒塔 859 步入睡如实记录）；Dominoes 用 0.30×1.0 m 宽牌；
  `ContactSolver` 休眠跳过谓词收窄为"双方 `!isAwake` 且非 static-static"（保住 W5 death-test 契约）；
  **冷热拆分第 4 次顺延到 W8（可选）**——W7 测量表明该规模下 body 布局不是热点。
- **testbed**：控制台子系统（为了 stdout smoke）；Pyramid200 名字沿用设计文档（默认 16 层=136 盒）。

---

## 9. 团队协作流程（本仓库已验证有效，直接复用）

### 9.1 角色

- **Lead（你）**：写/冻结公共头 + CMake + 文档；分任务；裁决跨模块测试期望；最终 clean 验证 +
  git 提交/tag；唯一动 git 的人。
- **Teammates（durable、可复用）**：`solver-dev`（数学/几何/求解器，W1 Math、W3 Solver、W4 SAT/解析、
  W5 摩擦/warm start/split impulse、W6 SAP+bench、W7 岛屿 DFS+三模式位级一致）、
  `core-dev`（World/集成测试/回归，W5 稳定性证据、W6 三路等价性、W7 休眠/唤醒/岛屿集成）、
  `testbed-dev`（UI/demo，W5 面板、W6 场景+demo、W7 Dominoes/IslandSleep500/单步慢放）。
  换会话时它们仍存在但 inactive；用 `send_message` 唤醒（`list_agents` 查看目标名）。

### 9.2 一轮标准流程

1. `create_goal`（可选，长任务）→ 读设计文档对应周 + 现有头文件；
2. **Lead Phase A**：一次写全该周公共头（新类型/API/字段，只追加）+ `src/CMakeLists.txt` + 占位 .cpp，
   跑 `-Target physics2d_core` 确认头文件可编译；
3. `team_task_create` 每模块一张卡（描述里写清：交付文件清单、实现要求、测试要求、构建命令、
   write scope、完成标准）；`send_message` 让每个 teammate claim 并开工；
4. 并行开发：每人独立 `-BuildDir build/<name>`；跨模块 transient link 失败用 `-Target physics2d_core`
   先验证编译；接口问题消息 Lead，不互相改文件；
5. **Lead 整合验证**：`format` → 删 canonical build → 三 preset 全绿 → 8 个 demo smoke → `--broadphase`
   A/B → 回归 `--check` → 工作区外 death test 全量 → 核对数据；
6. **Lead 收尾**：更新 docs → Conventional Commits（按模块拆）→ annotated tag → 汇报。

### 9.3 任务卡模板（复制即可）

```text
仓库：E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D
目标：<本周主题>，对应设计文档第 X 周 / P0X。
交付文件（只允许改/建这些）：<清单>
实现要求：<公式/语义/边界/错误处理>
单测要求：<手算期望值、边界、DoD>
构建命令：powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/<name> -NoTestbed -RunTests
约束：不改 include/CMake/docs/其它模块；不 git；零 /W4 警告；完成后标记 task complete 并给 Lead 短报告。
```

### 9.4 单人开发也行

不派 agent 时流程同 Phase A/C：Lead 自己按模块写 + 测试 + 三 preset 验证 + 提交。难度不变，
只是周期长一些；**接口先行、测试手算、最后统一验证**这三个习惯不要丢。

---

## 10. 常用命令速查

> 每个命令的期望输出、如何只跑一组测试、如何做交互式人工验收、bench/回归/death tests 的完整说明见
> [`docs/testing.md`](docs/testing.md)。下面是最高频的几条。

```powershell
cd "E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D"

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
> `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D\HANDOFF.md`、
> 同目录 `AGENTS.md`、`docs/ROADMAP.md`、`docs/dev-interfaces.md`、`docs/testing.md`，
> 再按 `HANDOFF.md` §2 跑验证基线（预期 354/354、15 skip、0 warning、八个 smoke、bench/回归数据可复现；
> 具体命令与期望值在 `docs/testing.md` §2/§5/§6/§7）。
> 确认基线后进入 Week 8（文档与交付周）：架构总图、`docs/math.md`、`docs/profiling.md`、README 完整版 + GIF、
> 已知限制与边界测试、简历 bullet 与面试演练，最后打 tag `v1.0-release`。
> 默认不改物理；可选优化（2×2 block solver、Tracy、渲染插值、BodySim 拆分）必须在文档完成后、以测量为前提。
> 严格遵守 HANDOFF §6 的"不该做的"，用 §9 的团队流程；结束时更新 HANDOFF 状态并汇报真实数据。

---

## 12. 本次交接完成检查清单（写此报告时已确认）

- [x] 工作区干净，HEAD = tag `v0.7-island-sleep`
- [x] 354 个测试全绿（15 sandbox skip；含 regression_default）、三 preset 零警告（Week 7 收尾实测）
- [x] 八个 demo smoke 可复现，数据已记入本文件 §2
- [x] 岛屿：3 独立堆→3 岛、桥接→2 岛、500-body 链 1 岛；三求解模式逐位一致（21,760 float 0 差异）
- [x] 休眠：500 盒 frame 105 全睡、Step 1.73x（release）；10 盒塔 100 步位不变；4 唤醒入口 + 邻近唤醒
- [x] 多米诺 20/20 连锁唤醒无跳过；回归弱 hash `588d30e70f9b4ddd` 保持不变
- [x] `docs/`（ROADMAP/dev-interfaces/architecture/devlog/dev-setup）+ README + AGENTS 与代码一致
- [x] 技术债、偏差、红线、团队流程、W8 任务卡、验收数据全部写入本文件
- [x] 无遗留后台任务/未完成 team task；所有 teammate inactive，可唤醒复用

> 下一会话开始后，第一件事：`cd` 到仓库、跑 §2 基线、`team_task_create` 建 W8 文档/测试任务卡。
> 结束后把本文件 §0/§1/§2/§5 更新到 Week 8（v1.0）状态，再提交。
