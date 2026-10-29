# Physics2D 交接报告（Session Handoff）

> **一句话现状**：Week 1-6 已完成并验证通过（tag `v0.6-sap`，**321 个测试全绿、0 警告、工作区干净**），
> **可插拔宽相（BruteForce + SAP）已落地**：2000 物体宽相 **7.81x 加速**、切换宽相物理结果**逐位不变**、
> headless 确定性回归 + bench 真实数字 + 6 个 demo 全部就绪。
> **下一步**：Week 7 —— 岛屿构建/按岛求解/整岛休眠 + 冷热拆分 + Dominoes/IslandSleep500 demo（tag `v0.7-island-sleep`）。
>
> 新会话请按顺序读：
> 1. 本文件（10 分钟，含"该做/不该做/踩过的坑"）
> 2. [`AGENTS.md`](AGENTS.md)（项目宪法 + 工作协议，**必须遵守**）
> 3. [`docs/ROADMAP.md`](docs/ROADMAP.md)（8 周计划）、[`docs/dev-interfaces.md`](docs/dev-interfaces.md)
>    （预留接口 + **偏差登记** + W5/W6 不变量）
> 4. 需要细节时再查 [`docs/architecture.md`](docs/architecture.md) 与 [`docs/devlog.md`](docs/devlog.md)
>
> 原始设计文档（仓库外）：`E:\Big Fat Fish's Workstation\2D物理引擎项目\2D物理引擎项目完整工作流与DeepSeek提示词库.md`
> 仓库路径：`E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D`
> 当前 HEAD：tag `v0.6-sap`（`git log -1` 查看具体 hash；本报告随该 tag 一起提交）

---

## 0. 新会话开场 3 步（别跳过）

1. **读**本文件 §1-§4 + `AGENTS.md`，确认你理解固定 Step 顺序、符号约定、分层铁律、W5/W6 不变量。
2. **跑验证基线**（§2）。不先确认基线的会话，后面所有回归都不可信。
3. **按 §5 Week 7 任务卡开工**，用 §9 的团队流程；结束时按 §10 清单收尾（commit + tag + 更新本文件状态）。

---

## 1. 仓库与状态

| 项 | 值 |
| --- | --- |
| 仓库路径 | `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D` |
| Git | `main` 分支，工作区干净，tags `v0.1-falling-ball` → … → `v0.5-stable-stack` → **`v0.6-sap`** |
| 构建目录 | `build/`（**可随时整个删除**，已被 gitignore；canonical `debug/release/sanitize` + 各 agent 并行目录） |
| 团队状态 | 所有 teammate 为 inactive；无后台任务；W6 任务（task-4/5/6）均 completed |
| 环境 | Windows 11 / i7-12650H / 16GB / VS2022 (MSVC 19.44) / CMake 3.31.12 / Ninja 1.12.1（VS 自带）/ RTX 4060 |

文档地图：

| 文件 | 作用 |
| --- | --- |
| `AGENTS.md` | 项目宪法 + 工作协议 + 技术债 + 下周入口（**每次会话必读**） |
| `docs/ROADMAP.md` | 8 周计划、每周 DoD、已完成周的验收记录 |
| `docs/dev-interfaces.md` | 18 条预留接口状态、接入步骤、**偏差登记（重要）**、W5/W6 不变量 |
| `docs/architecture.md` | 模块职责、数据布局、Step 流水线、宽相数据流、求解器公式 |
| `docs/devlog.md` | 每周周报：完成/卡住/数据/下周/提醒（面试素材来源） |
| `docs/dev-setup.md` | 环境、构建、测试台操作、**疑难杂症（含本机增量构建坑）** |
| `README.md` | 对外展示（W1-6 特性、真实性能表、Roadmap） |
| `HANDOFF.md` | 本文件：换会话交接 |

---

## 2. 验证基线（开工前先跑，确认环境没变）

> 本机只有 **Windows PowerShell 5.1（没有 `pwsh`）**，且首次 configure 会联网 FetchContent
> 下载依赖（1-2 分钟）。命令必须在仓库根目录执行。

```powershell
cd "E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D"

# 全量：配置 + 编译 + ctest（幂等，增量很快）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
```

**期望基线（Week 6 结束时实测）**：

| 检查项 | 期望值 |
| --- | --- |
| 构建 | exit 0，MSVC `/W4 /permissive-` **0 个项目警告** |
| ctest | **321/321 passed, 0 failed, 15 skipped**（skipped 全是 sandbox death tests；含 1 个 `regression_default`） |
| Release | **322/322 passed**（多 1 个 release-only 契约测试） |
| Sanitize | `-Preset sanitize -RunTests` 同样 321/321 |
| clean 构建耗时 | debug **42.5 s** / release **47.1 s** / sanitize **66.1 s**（含依赖编译） |
| 测试 exe 工作区外运行 | 320/320 gtest 全过（15 个 death test 真正执行） |

六个 demo 冒烟（**真实输出基线**；`broadphase=` 是 Auto 实际安装的实现）：

```powershell
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall  --smoke 150 --hidden
# bodies=2  contacts=1 points=1 max_pen=5.000mm  broadphase=BruteForce candidate_pairs=1  avg≈0.98ms
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack  --smoke 300 --hidden
# bodies=4  contacts=3 points=3 max_pen=3.063mm  broadphase=BruteForce candidate_pairs=5  avg≈1.07ms
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack     --smoke 300 --hidden
# bodies=11 contacts=10 points=20 max_pen=3.763mm broadphase=BruteForce candidate_pairs=19 avg≈1.20ms
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
# bodies=2  contacts=1 points=2 max_pen=2.686mm  broadphase=BruteForce candidate_pairs=1  avg≈1.03ms（θ=15°, μ=0.30 静止）
.\build\debug\testbed\physics2d_testbed.exe --demo Pyramid200   --smoke 300 --hidden
# bodies=137 contacts=324 points=595 max_pen=41.863mm broadphase=SAP candidate_pairs=496 avg≈6.9ms
#   （600 步：max_pen=29.7mm、基本静止；默认 16 层=136 盒，4–24 层可调）
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark --smoke 300 --hidden
# bodies=401 contacts=819 candidate_pairs=1687 broadphase=SAP avg≈13.7ms
#   （400 盒密堆的 178mm 穿透是载荷/迭代预算现象，切宽相物理不变）
```

宽相 A/B（**切宽相物理结果必须完全一致**）：

```powershell
# 同一 demo 用三种宽相跑，body/contact 输出行应逐字节相同（只有 candidate_pairs 不同）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase none
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase bf
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase sap
# --broadphase 非法值 exit 1；未知 demo exit 2
```

bench（**Release**，数字必须可复现）：

```powershell
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42
# 期望（i7-12650H / MSVC 19.44 / seed 42 / median）：
#   50: BF 0.0033 / SAP 0.0015 ms (2.20x)    200: 0.0838 / 0.0169 (4.96x)
#  500: BF 0.5399 / SAP 0.0916 ms (5.89x)   2000: 9.0985 / 1.1656 (7.81x)
# 小 N 反转：--bodies 5,10,20,50 → SAP 0.00x/0.50x/0.80x/2.12x（交叉点 N≈25-32）
```

回归工具（golden 必须绿）：

```powershell
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
# 期望：hash=588d30e70f9b4ddd（quantum 1e-5）、exit 0；mismatch exit 1；文件/参数错误 exit 2
```

> `testbed` 可执行文件路径是**嵌套的**：`build/<dir>/testbed/physics2d_testbed.exe`。
> 无 OpenGL 上下文时 smoke 会返回退出码 2 并打印原因（当前机器可用，NVIDIA 3.3）。

**怀疑环境坏了时**（先看 `docs/dev-setup.md` §6）：

- `pwsh` 不存在是正常的，用 `powershell`；
- Ninja 必须在仓库外（`scripts/build.ps1` 自动从 VS 安装目录找）；把 Ninja 放进 `tools/` 会触发
  `cl D8050` / `Permission denied`（沙箱限制）；
- **本机增量构建坑（重要）**：中文 locale 的 cl.exe 把 `/showIncludes` 打成 `注意: 包含文件:`，
  CMake 写入 `rules.ninja` 的是 UTF-8 前缀，Ninja 按字节匹配失败 → **depfile 为空，改 header 不触发重编**
  （`VSLANG=1033` 本机无效，已实测）。改过 header 后必须 `build.ps1 -Clean` 或 clean build；
  canonical 周验收一律删 `build/<preset>` 重编，所以历史数据未受影响；
- `ctest --preset` 必须在仓库根目录跑（或用 `--test-dir`）；
- death tests 在本沙箱自动 SKIP；把 `build/debug/tests/physics2d_tests.exe` 复制到
  `%LOCALAPPDATA%\Physics2D\testrun\` 再运行可得到 **320/320 gtest 全过**（证明 CI 会执行它们）；
- **不要用 PowerShell 5.1 的 `Get-Content`/`Set-Content` 改 UTF-8 中文文档**（会乱码）；
  用编辑工具（本 harness 的 write/edit，或 `-Encoding UTF8` 且显式处理 BOM）。

---

## 3. 已完成内容（Week 1-6）

| 周 | 主题 | 关键交付 | 数据亮点 |
| --- | --- | --- | --- |
| W1 | 数学 + 构建 + 渲染循环 + 积分骨架 | `p2::Math`、`Handle`+`IDPool`、`Body`/`World`、11 阶段 `Step`、`StepAccumulator`、GLFW+ImGui testbed、CMake/CI | 120 测试；clean build 23.6s |
| W2 | 形状 + 解析碰撞 + Contact 持久化 | `Shape` 池（Circle/Plane）、解析窄相、`Contact`/`ContactID` 帧间匹配、临时位置响应 | 185 测试；静止穿透 7.8mm |
| W3 | 法向 Sequential Impulse | `ContactSolver`（K_n、累计钳制、迭代、restitution bias）、World 接线、SphereStack | 210 测试；8 迭代残余 0.00065 m/s |
| W4 | 凸多边形 SAT + 裁剪 + SATCache | `PolygonShape`、`SAT`/`Clipping`/`CollidePolygons`、`ContactID.Key()` 去 flip、BoxStack | 278 测试；10 盒塔会塌（W4 基线） |
| W5 | 摩擦锥 + warm starting + split impulse | `ContactSolver` 摩擦/warm start/Baumgarte 基线；`ContactSolverPosition.cpp` 伪速度层；DebugUI 4 面板 + FrictionRamp | **301 测试**；**10 盒 10 s 稳定（穿透 5.223mm）** |
| W6 | BruteForce + SAP 宽相 + 量化 + 回归 | `BroadPhaseBruteForce`/`BroadPhaseSAP`、World 宽相接线 + fat AABB + pair 规范化、`bench_broadphase`、`physics2d_regression` + golden + CI job、Pyramid200/StressBenchmark + 宽相下拉/自适应取景 | **321 测试**；2000 物体宽相 **7.81x**；切宽相物理**逐位不变**；SAP vs BF 10,000 帧 0 mismatch |

**模块 → 文件**（只列关键入口，完整用 `git ls-files`）：

```text
include/physics2d/Math/*.h            Vec2 / Mat2 / Rot / Transform / AABB / MathUtils
include/physics2d/Core/               World / Body / BodyDef / Shape / Settings / Handle / IDPool /
                                      TimeStep / BroadPhaseInterface / Profiler
include/physics2d/Collision/          CircleShape / PlaneShape / PolygonShape / Analytic / Manifold /
                                      Contact / ContactID / SAT / Clipping / SATCache / ShapeTypes /
                                      BroadPhaseBruteForce / BroadPhaseSAP
include/physics2d/Solver/             ContactSolver.h
src/{Math,Core,Collision,Solver}/*.cpp (Solver = ContactSolver.cpp + ContactSolverPosition.cpp)
tests/unit/*.cpp                      GLOB 自动收集（加文件不用改 CMake）
tests/regression/                     regression_main.cpp + golden/default_600.txt + README
bench/bench_broadphase.cpp            只依赖 core 的宽相基准
testbed/                              main / Renderer / Camera / DebugUI / Demo.h / SceneBuilders.h /
                                      demos/{FallingBall,SphereStack,BoxStack,FrictionRamp,Pyramid200,StressBenchmark}
docs/  scripts/
```

---

## 4. 架构速览（W7 要改的地方）

### 4.1 World::Step 固定流水线（顺序不可变！）

```text
1 IntegrateForces            [done W1]
2 UpdateBroadPhase(dt)       [done W6：proxy 同步 + QueryPairs + pair 规范化 (i<j,sorted,deduped)]
3 UpdateContacts             [done W2/W4/W6：宽相 pair 列表 or O(N²) fallback → CreateContactForPair]
4 BuildIslands               [空→W7 实现：DFS 分岛、islandId、按岛打包]
4b PrepareConstraints(dt)    [done W5：K_n/K_t、restitution+Baumgarte bias、warm-start 播种]
5 WarmStart                  [done W5]
6 SolveVelocityConstraints   [done W5：法向→切向（摩擦锥）迭代]
7 IntegratePositions         [done W1]
8 SolvePositionConstraints(dt) [done W5：split impulse 伪速度层]
9 Synchronize                [done W2/W6：tight AABB + kAabbMargin 的 fat AABB]
10 UpdateSleep(dt)           [空→W7 实现：sleepTime、整岛原子睡眠/唤醒]
11 Validate                  [done W1：NaN/Inf 扫描，含伪速度]
```

- 第一阶段（`IntegrateForces`）先给动态体加重力速度，然后才窄相/求解——**不要改变顺序**。
- 接触锚点是**生成时刻**的几何（`IntegratePositions` 之前）；测试不要用 step 之后的 transform 反投影。
- W7 的 `BuildIslands` 在 phase 4、`UpdateSleep` 在 phase 10：**空方法已经占位，填函数体即可，不要插入新阶段**。
  按岛求解意味着 phase 6 的遍历顺序从"全局接触列表"变为"岛内打包顺序"——这**会改变浮点结果**，
  需要新增"按岛 vs 不按岛"的开关做对照，并接受默认行为变化（解释进 devlog），或保证整数级等价（更推荐）。

### 4.2 数据流与所有权（W7 直接依赖）

- `World::m_bodies`（槽位数组，index == BodyHandle::index，永不压实）+ `IDPool` generation。
- `World::m_shapes`（Shape tagged union）+ `Body::shape`；`AttachShape` 从 density 推质量/惯性。
- `World::m_contacts`（每帧重建）+ `m_previousContacts`（上一帧；warm start 数据源）。
- **宽相（W6）**：`Body::aabb` = fat AABB（tight + `kAabbMargin=0.1`，`Synchronize` 写）；
  `Body::proxyId`（-1 = 无 proxy）+ `World::m_proxyToBody` 反向映射；
  `World::m_broadPhasePairs` = 规范化候选对；`GetBroadPhasePairCount()` 只 expose count。
- `ContactSolver`（World 以 `unique_ptr` 前向声明持有）：`Prepare` 缓存 rA/rB、normalMass、tangentMass、
  bias、warm-start 播种；`WarmStart` 施加；`SolveVelocity` 迭代；`SolvePosition` 伪速度层。
- `ContactID::Key()` **不含 flip**；incidentVertex 是绝对顶点索引；SAT tie-break 对称。
  **warm starting 依赖 Key 的帧间稳定，不要动这些语义。**
- `Settings.h` 是唯一常量来源；`SolverSettings` 可运行时改（`World::SetSolverSettings`）。
  休眠常量 `kSleepLinearTolerance`/`kSleepAngularTolerance`/`kSleepTimeThreshold` 已存在（W1 预留）。
- `Body::islandId`、`sleepTime`、`allowSleep`、`World::GetIslandCount()`/`GetAwakeBodyCount()` 已预留
  （`GetIslandCount()` 目前恒 0）。

### 4.3 W5/W6 不变量（W7 不要破坏；改前必读）

- **宽相只决定"谁进窄相"**：候选对必须规范化为 `(i<j, sorted, deduped)` 的 body-index 对再进窄相；
  切宽相物理结果必须逐位一致；`IBroadPhase` 只知道 AABB；proxy 生命周期由 World 管；
  fat AABB 只加候选（漏检先查顺序/时序，不要调大 margin）。
- **摩擦锥钳制累计值**、法向先解切向后解、`μ = contact.friction`（World 已组合）。
- **warm start 只施加一次**（`Prepare` 播种 × factor，`WarmStart` 施加 cache 原值，默认 factor 1.0）。
- **split impulse 只碰伪速度**：真实速度逐位不变、伪速度每步清零、位置层用位置积分后的 transform。
- **回归 hash 纪律**：`tests/regression/golden/default_600.txt` 只在有意的物理/场景变更时更新，
  devlog 写 old/new 与原因；**纯重构（BodySim 拆分、岛屿打包、排序优化）hash 必须不变**。

---

## 5. 该做的：Week 7 任务卡（可直接照做）

> 设计文档 Week 7：岛屿划分（DFS）、按岛求解、整岛原子休眠/唤醒、多米诺连锁唤醒、
> IslandSleep500/Dominoes demo、暂停/单步/慢放、Contact 面板完整版；另补做冷热拆分（三次顺延）。
> tag `v0.7-island-sleep`。

### 5.1 目标与验收（DoD）

| 项 | 验收标准 |
| --- | --- |
| 岛屿构建 | `World::BuildIslands`：awake dynamic body 为起点 DFS（**显式栈**）；边 = Contact；static/kinematic 作边界（约束跨越、不入岛、状态不变）；同岛 body 共享 `islandId`；岛 body/contact 打包进连续数组；**3 岛/合并岛测试**（两个堆叠 + 小圆连接后合并为 1 岛） |
| 按岛求解 | phase 6 按岛打包顺序求解；提供开关/对照，保证「按岛 vs 不按岛」在可接受定义下一致（若顺序变化导致浮点差异，需在 devlog 解释并给出确定性证据）；**不能把物理改坏**（10 盒里程碑、摩擦角扫描、三路宽相等价性继续绿） |
| 休眠 | `UpdateSleep`：`sleepTime` 累计；整岛原子 `isAwake=false` 且速度清零；sleep 岛跳过宽相/窄相/求解**但保留 proxy**；唤醒：外力/冲量/SetTransform → 整岛唤醒；awake 物体 AABB 与 sleep 岛 proxy 重叠 → 整岛唤醒；**sleep 后连续 100 步状态逐位不变**；**多米诺连锁唤醒**测试 |
| 统计 | `GetIslandCount()`/`GetAwakeBodyCount()` 接真实值；testbed Status 面板显示 islands/awake 曲线；IslandSleep500 休眠前后 Step 耗时对比进 README |
| 冷热拆分 | `Body`→`BodySim`/`BodyState` 并行数组（设计文档 3.3）；**321 个既有测试全绿 + 回归弱 hash 不变**；`GetBody()`/`GetBodies()` 契约与 testbed 不变（必要时提供 debug 视图） |
| demo | `Dominoes`（多米诺连锁唤醒）、`IslandSleep500`（休眠前后耗时）；`testbed/SceneBuilders.h` 复用 |
| 回归 | **既有 321 个测试不得弱化/删除**；新增测试覆盖岛屿/休眠/冷热拆分 |
| 质量 | 三 preset 零警告；debug/release/sanitize 全绿；sanitize 干净；format OK |
| 文档/tag | devlog W7、README 性能表（休眠对比）、ROADMAP ✅、architecture（岛屿/休眠/数据布局）、dev-interfaces（R9/R10/R14/R15/R17）、AGENTS 收尾；`v0.7-island-sleep` |

### 5.2 Phase A（Lead 先做：接口/常量）

1. 读设计文档第 6.5 节（岛屿与休眠伪代码）+ P13；确认 `Body::islandId/sleepTime/allowSleep`、
   `Settings.h` 的休眠常量、`World::GetIslandCount/GetAwakeBodyCount` 已够用；如缺常量按"只追加"加入
   （例如 `kSleepIslandBudget` 不需要，先不加）。
2. 若决定新增求解器入口（如 `ContactSolver::Prepare/...` 接受"岛内连续数组"），先在头文件加**重载/新方法**
   （不改已有签名），`src/CMakeLists.txt` 登记新 .cpp（如 `src/Solver/Island.cpp`），
   `-Target physics2d_core` 编译检查。
3. 定**按岛求解的确定性方案**：岛排序键（最小 body index）→ 打包顺序稳定；明确"按岛 vs 不按岛"的
   对照方式（开关或测试）——先定这个再派任务，否则物理 hash 会漂。
4. Lead 独占：`include/`、`src/CMakeLists.txt`、docs、git。

### 5.3 Phase B（三个实现者并行；沿用已验证的分工）

> 复用 durable teammates：`solver-dev`（求解器/算法）、`core-dev`（World/集成测试/回归）、
> `testbed-dev`（UI/demo）。每人独立 build dir，互不碰文件。

- **solver-dev（岛/休眠算法核心）** — 文件：`src/Solver/Island.cpp`（新）、`include/physics2d/Solver/Island.h`（新）、
  `tests/unit/test_island.cpp`（新）、`tests/unit/test_sleep.cpp`（新）、`src/Core/World.cpp`?（不——World 归 core-dev）。
  要点：DFS 显式栈、岛内 body/contact 连续数组、按岛求解入口、休眠判定/整岛原子睡眠/唤醒传播；
  与 core-dev 的接口（`World::BuildIslands/UpdateSleep` 调 solver）先与 Lead 对齐签名。
- **core-dev（World 接线 + 集成测试 + 回归）** — 文件：`src/Core/World.cpp`（BuildIslands/UpdateSleep/
  按岛求解接线/唤醒 API）、`tests/unit/test_world_islands.cpp`（新）、`tests/unit/test_world_sleep.cpp`
  （或合并）、既有 World 测试的必要更新（注释解释物理原因，不弱化）、**冷热拆分**（若排期到 W7 的一个
  独立 commit：`Body.h`+`World.cpp`+`ContactSolver`+测试；行为 hash 不变）。
- **testbed-dev（demo + 面板）** — 文件：`testbed/demos/Dominoes.*`、`testbed/demos/IslandSleep500.*`、
  `testbed/SceneBuilders.h`（扩展 domino/island 场景）、`testbed/DebugUI.*`（islands/awake、暂停/单步/慢放）、
  `testbed/main.cpp`、`testbed/README.md`；休眠前后 Step 耗时对比数据交给 Lead。
- **bench/回归**：`bench/` 可加休眠场景（可选）；回归工具扩展多场景 golden（可选，先保证 default 不变）。

### 5.4 Phase C（Lead 收尾；顺序固定）

1. `scripts/format.ps1`（就地）→ `-Check` 必须 OK；
2. **删掉 canonical `build/debug`、`build/release`、`build/sanitize`**，用 `build.ps1 -Preset X -RunTests`
   从干净状态跑三套（记录耗时、grep `warning C` 必须 0、ctest 全绿）；**改过 header 的增量构建不可信**；
3. 六个 demo smoke + 新增 demo smoke + `--broadphase` A/B + 回归 `--check` 必须绿；
4. 更新文档：`docs/devlog.md`（W7 周报）、`docs/ROADMAP.md`（W7 ✅）、`docs/architecture.md`
   （岛屿/休眠/数据布局/Step 状态）、`docs/dev-interfaces.md`（R9/R10/R14/R15/R17 + 偏差）、
   `README.md`（休眠性能表、demo 列表）、`AGENTS.md`（状态、技术债、W8 入口）、**本文件 `HANDOFF.md`**；
5. git 提交（Conventional Commits，按模块拆 3-5 个）+ `git tag -a v0.7-island-sleep`；
6. 确认 `git status` 干净、`git log/tag` 正确、给用户汇报（含真实数据与偏差）。

---

## 6. 不该做的（红线 / 已知大坑，务必遵守）

### 6.1 项目范围红线（设计文档 0.5，违反会被拒绝）

- ❌ 不做 3D / 软体 / 流体 / 布料 / GPU 加速 / 完整关节系统 / CCD-TOI / 兼容 Box2D API / 引入游戏引擎。
- ❌ 不为了"更强的 demo"扩张范围；新想法一律进 `docs/ROADMAP.md` 的 Roadmap，不进当前周。

### 6.2 W7 专属的坑（W5/W6 踩过/预见的）

- ❌ **顺序就是物理**：岛屿打包顺序、按岛求解顺序、宽相候选对顺序都会改变浮点结果。任何顺序变更
  必须证明"行为可接受"（要么整数级等价，要么在 devlog 记录差异并给出确定性证据），不能让宽相三路
  等价性/10 盒里程碑变绿得不明不白。
- ❌ **休眠必须整岛原子**：不能单个 body 睡眠（岛边界会穿模）；sleep 岛的 proxy 必须保留在宽相里，
  否则不会与 awake 物体重新接触 → 无法唤醒。
- ❌ 不要在 sleep 岛内"跳过"状态但留下非零速度/伪速度/力；睡眠时速度清零、伪速度清零，且
  **连续 100 步状态逐位不变**（测试钉死）。
- ❌ 冷热拆分是**行为不变**的重构：`BodySim`/`BodyState` 拆分后回归弱 hash 必须不变；不要在同一
  commit 里改求解器公式。
- ❌ 不要弱化/删除既有 321 个测试来让新代码变绿；行为变化必须有机制解释并写进 devlog。
- ❌ 不要用 `unordered_map` 遍历顺序影响岛屿/求解顺序。
- ❌ 改 header 后不要相信增量构建（本机 depfile 为空）——`-Clean` 或 clean build。
- ❌ 不要在别人构建时删除/清理 `build/`；并行开发一律独立 `-BuildDir build/<name>`。
- ❌ 不要写没有测量条件的性能数字（休眠对比必须 Release + 说明场景/帧数）；不要伪造 smoke/测试结果。
- ❌ 不要在物理核心引入 GLFW/ImGui/OpenGL；宽相/窄相的分层继续守住。
- ❌ 共享工作区并行开发时 build dir 可能链接旧 obj；最终结论一律以 canonical clean build 为准。

### 6.3 协作红线

- 每个 teammate 只改**任务写作用域**内的文件；Header/CMake/docs/提交由 Lead 独占。
- 跨模块接口问题先消息 Lead，不要直接改对方文件；Lead 用"手算解析解"裁决测试期望，
  **不通过弱化断言来消除失败**。
- 不要运行 `git commit/push/tag`（只有 Lead 做）；teammate 不创建分支。
- 修复循环上限 3 轮：超过就拆任务卡或人工读代码，禁止无限自循环。

---

## 7. 已知技术债 & 待办（W7-W8）

| # | 技术债 | 位置 | 计划 |
| --- | --- | --- | --- |
| 1 | **岛屿/休眠未实现**（`BuildIslands`/`UpdateSleep` 空；`GetIslandCount()`=0；proxy 不区分睡眠） | `World.cpp`、`Body` 预留字段 | **W7** |
| 2 | **`BodySim`/`BodyState` 冷热拆分**（W4→W5→W6→W7 三次顺延） | `Body.h` + `World.cpp` + `ContactSolver` | **W7**（行为 hash 不变） |
| 3 | 按岛求解顺序对浮点结果的影响未量化 | `World::SolveVelocityConstraints` | **W7**（对照 + devlog） |
| 4 | 多 demo 集缺 Dominoes/IslandSleep500 | `testbed/demos/` | **W7** |
| 5 | Tracy 只有零成本宏，未接入（R11） | `Core/Profiler.h`、CMake 选项 | W7-W8 |
| 6 | CI format 检查/clang-tidy 未纳入 | `.github/workflows/ci.yml`、scripts | W8 |
| 7 | 回归工具只有单场景 golden（可选多场景模板） | `tests/regression/` | W7 可选 |
| 8 | `ContactSolver::SolvePosition` 每帧分配局部 vector（未测量热点） | `ContactSolverPosition.cpp` | 若 W7 bench 显示热点再优化 |
| 9 | **30:1 单列/金字塔底部在默认 8 迭代下超 10mm 穿透**（载荷/迭代预算，非 bug） | `ContactSolver` | 记录在 README/devlog，不修 |
| 10 | 渲染插值（R17） | `StepAccumulator`/testbed | W7 可选（暂停/单步/慢放优先） |

另：`tests/regression/`、`bench/` 已是真实工具（不是占位）；`docs/dev-interfaces.md` 第 5 节是**偏差登记**。

---

## 8. 已登记的偏差（避免重复讨论）

完整列表见 `docs/dev-interfaces.md` §5；高频的几条：

- **依赖**：vcpkg → FetchContent（本机无 vcpkg；设计文档允许二选一）。
- **Ninja 位置**：不放仓库（本机沙箱会拒绝工作区内可执行文件的子进程写文件）。
- **验收方式**：sandbox 内 death tests 自动 SKIP；CI/普通机器执行（可复制 exe 到工作区外验证）。
- **Contact 文件位置**：`Solver/Contact.h` → `Collision/Contact.h`（World 拥有接触列表，Core 不依赖 Solver）。
- **ContactID**：`Key()` 不含 flip；incidentVertex 用绝对顶点索引；SAT tie-break 对称（warm start 依赖）。
- **`kWarmStartFactor` = 1.0**（W1 计划 0.85；实测所有 <1.0 的 factor 都让 10 盒塔在 10s 内倒）。
- **`kSplitImpulseSlop` = `kLinearSlop`**；`useSplitImpulse=false` = Baumgarte 速度层 bias（非 W2 位置推出）。
- **`Body::aabb` = fat AABB（tight + `kAabbMargin`=0.1）**；`AttachShape` 后到下一次 `Synchronize` 前仍是 tight。
- **`UpdateContacts` 保留 O(N²) fallback**（未装宽相的 World 与 W5 行为完全一致）。
- **候选对顺序规范化** `(i<j, sorted, deduped)` 是物理正确性要求，不是优化。
- **SAP 惰性排序**（AddProxy 追加 + 观察点 `std::sort`，稳态冒泡；`ValidateInternalState` 用 mutable）；
  **RemoveProxy 用 erase** 而非 swap-pop。
- **回归 golden 用 txt + 量化弱 hash**（不是 P16 设想的 CSV + Python 脚本）。
- **W6 无 `DetachShape` API**（proxy 生命周期测试口径已按真实契约写）。
- **冷热拆分 W7 做**（三次顺延的理由见偏差表）。
- **testbed**：控制台子系统（不是 `WIN32_EXECUTABLE`，为了 stdout smoke）；Tracy 延后到 W7-W8。
- **Pyramid200 名字**：沿用设计文档名，默认 16 层=136 盒（20 层=210 盒在 8 迭代下底部下沉 0.44m，
  可调出复现；稳定性-层数表在 README/devlog）。

---

## 9. 团队协作流程（本仓库已验证有效，直接复用）

### 9.1 角色

- **Lead（你）**：写/冻结公共头 + CMake + 文档；分任务；裁决跨模块测试期望；最终 clean 验证 +
  git 提交/tag；唯一动 git 的人。
- **Teammates（durable、可复用）**：`solver-dev`（数学/几何/求解器，历史完成 W1 Math、W3 Solver、
  W4 Polygon/Analytic、W5 摩擦/warm start/split impulse、W6 SAP+bench）、
  `core-dev`（World/集成测试，W5 稳定性证据、W6 三路等价性 + 回归工具）、
  `testbed-dev`（UI/demo，W5 面板+FrictionRamp、W6 场景+demo+宽相面板）。
  换会话时它们仍存在但 inactive；用 `send_message` 唤醒（`list_agents` 查看目标名）。

### 9.2 一轮标准流程

1. `create_goal`（可选，长任务）→ 读设计文档对应周 + 现有头文件；
2. **Lead Phase A**：一次写全该周公共头（新类型/API/字段，只追加）+ `src/CMakeLists.txt` + 占位 .cpp，
   跑 `-Target physics2d_core` 确认头文件可编译；
3. `team_task_create` 每模块一张卡（描述里写清：交付文件清单、实现要求、测试要求、构建命令、
   write scope、完成标准）；`send_message` 让每个 teammate claim 并开工；
4. 并行开发：每人独立 `-BuildDir build/<name>`；跨模块 transient link 失败用 `-Target physics2d_core`
   先验证编译；接口问题消息 Lead，不互相改文件；
5. **Lead 整合验证**：`format` → 删 canonical build → 三 preset 全绿 → 六个+新增 demo smoke →
   宽相 A/B → 回归 `--check` → 核对数据；
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

```powershell
cd "E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D"

# 构建 + 测试（日常；改过 header 加 -Clean！）
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

# 测试台（交互）与冒烟
.\build\debug\testbed\physics2d_testbed.exe
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo Pyramid200 --smoke 300 --hidden --broadphase sap

# bench（必须 Release）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42

# 确定性回归
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
```

关键常量/入口：`include/physics2d/Core/Settings.h`（所有魔法数字）、
`include/physics2d/Core/World.h`（Step 注释 + API）、
`include/physics2d/Solver/ContactSolver.h`（求解器契约）、
`src/Solver/ContactSolverPosition.cpp`（split impulse 位置层）、
`include/physics2d/Core/BroadPhaseInterface.h` + `Collision/BroadPhaseSAP.h`（宽相契约）、
`include/physics2d/Collision/ContactID.h`（Key 语义）。
测试可执行文件：`build/debug/tests/physics2d_tests.exe`（`--gtest_filter=` 可单独跑）。

---

## 11. 新会话开场提示词（直接复制粘贴）

> 你是 Physics2D 项目的接手开发者。请先读
> `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D\HANDOFF.md`、
> 同目录 `AGENTS.md`、`docs/ROADMAP.md`、`docs/dev-interfaces.md`，
> 再按 `HANDOFF.md` §2 跑验证基线（预期 321/321、15 skip、0 warning、六个 smoke、bench/回归数据可复现）。
> 确认基线后从 Week 7 开始：岛屿构建（DFS 显式栈）+ 按岛求解 + 整岛原子休眠/唤醒 + 多米诺连锁唤醒、
> Dominoes/IslandSleep500 demo，并补做冷热拆分（Body→BodySim/BodyState，行为 hash 不变），
> 目标是 tag `v0.7-island-sleep`。
> 严格遵守 HANDOFF §6 的"不该做的"，用 §9 的团队流程；结束时更新 HANDOFF 状态并汇报真实数据。

---

## 12. 本次交接完成检查清单（写此报告时已确认）

- [x] 工作区干净，HEAD = tag `v0.6-sap`
- [x] 321 个测试全绿（15 sandbox skip；含 regression_default）、三 preset 零警告（Week 6 收尾实测）
- [x] 六个 demo smoke 可复现，数据已记入本文件 §2
- [x] 宽相 2000 物体 **7.81x**；切宽相物理结果逐位不变（World 测试 + testbed A/B SHA 一致）
- [x] bench 真实数字写进 README；回归弱 hash `588d30e70f9b4ddd` 与 golden 一致
- [x] `docs/`（ROADMAP/dev-interfaces/architecture/devlog/dev-setup）+ README + AGENTS 与代码一致
- [x] 技术债、偏差、红线、团队流程、W7 任务卡、验收数据全部写入本文件
- [x] 无遗留后台任务/未完成 team task；所有 teammate inactive，可唤醒复用

> 下一会话开始后，第一件事：`cd` 到仓库、跑 §2 基线、`team_task_create` 建 W7 任务卡。
> 结束后把本文件 §0/§1/§2/§5 更新到 Week 7 状态，再提交。
