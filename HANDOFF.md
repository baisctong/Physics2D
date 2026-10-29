# Physics2D 交接报告（Session Handoff）

> **一句话现状**：Week 1-5 已完成并验证通过（tag `v0.5-stable-stack`，**301 个测试全绿、0 警告、工作区干净**），
> **10 方块堆叠 10 秒稳定里程碑已达成**（最大穿透 5.223 mm < 10 mm）。
> **下一步**：Week 6 —— Broad Phase（IBroadPhase + BruteForce + SAP）+ 量化 + headless 回归 +
> **补做冷热拆分**（tag `v0.6-sap`）。
>
> 新会话请按顺序读：
> 1. 本文件（10 分钟，含"该做/不该做/踩过的坑"）
> 2. [`AGENTS.md`](AGENTS.md)（项目宪法 + 工作协议，**必须遵守**）
> 3. [`docs/ROADMAP.md`](docs/ROADMAP.md)（8 周计划）、[`docs/dev-interfaces.md`](docs/dev-interfaces.md)
>    （预留接口 + **偏差登记** + W5 求解器不变量）
> 4. 需要细节时再查 [`docs/architecture.md`](docs/architecture.md) 与 [`docs/devlog.md`](docs/devlog.md)
>
> 原始设计文档（仓库外）：`E:\Big Fat Fish's Workstation\2D物理引擎项目\2D物理引擎项目完整工作流与DeepSeek提示词库.md`
> 仓库路径：`E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D`
> 当前 HEAD：tag `v0.5-stable-stack`（`git log -1` 查看具体 hash；本报告随该 tag 一起提交）

---

## 0. 新会话开场 3 步（别跳过）

1. **读**本文件 §1-§4 + `AGENTS.md`，确认你理解固定 Step 顺序、符号约定、分层铁律、**W5 求解器不变量**。
2. **跑验证基线**（§2）。不先确认基线的会话，后面所有回归都不可信。
3. **按 §5 Week 6 任务卡开工**，用 §9 的团队流程；结束时按 §10 清单收尾（commit + tag + 更新本文件状态）。

---

## 1. 仓库与状态

| 项 | 值 |
| --- | --- |
| 仓库路径 | `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D` |
| Git | `main` 分支，工作区干净，tags `v0.1-falling-ball` → `v0.2-plane-contact` → `v0.3-impulse` → `v0.4-sat` → **`v0.5-stable-stack`** |
| 构建目录 | `build/`（**可随时整个删除**，已被 gitignore；里面是各周并行开发目录 + canonical `debug/release/sanitize`） |
| 团队状态 | 所有 teammate 为 inactive；无后台任务；无未完成任务（task-1..task-3 均 completed） |
| 环境 | Windows 11 / i7-12650H / 16GB / VS2022 (MSVC 19.44) / CMake 3.31.12 / Ninja 1.12.1（VS 自带）/ RTX 4060 |

文档地图：

| 文件 | 作用 |
| --- | --- |
| `AGENTS.md` | 项目宪法 + 工作协议 + 技术债 + 下周入口（**每次会话必读**） |
| `docs/ROADMAP.md` | 8 周计划、每周 DoD、已完成周的验收记录 |
| `docs/dev-interfaces.md` | 18 条预留接口状态、接入步骤、**偏差登记（重要）**、W5 求解器不变量 |
| `docs/architecture.md` | 模块职责、数据布局、Step 流水线状态、求解器公式（摩擦/warm start/split impulse）、技术债 |
| `docs/devlog.md` | 每周周报：完成/卡住/数据/下周/提醒（面试素材来源） |
| `docs/dev-setup.md` | 环境、构建、测试台操作、疑难杂症 |
| `README.md` | 对外展示（已写 Week 1-5 特性、数据、Roadmap） |
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

**期望基线（Week 5 结束时实测）**：

| 检查项 | 期望值 |
| --- | --- |
| 构建 | exit 0，MSVC `/W4 /permissive-` **0 个项目警告** |
| ctest | **301/301 passed, 0 failed, 15 skipped**（skipped 全是 sandbox death tests） |
| Release | **302/302 passed**（多 1 个 release-only 契约测试） |
| Sanitize | `-Preset sanitize -RunTests` 同样 301/301 |
| clean 构建耗时 | debug **31.8 s** / release **41.0 s** / sanitize **42.9 s**（含依赖编译） |

四个 demo 冒烟（**真实输出基线**）：

```powershell
# 1) 单球落地：bodies=2 contacts=1，球心 y=0.495，穿透 5.000mm = kLinearSlop
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden

# 2) 3 球叠放：bodies=4 shapes=4 contacts=3，y=0.500/1.500/2.496，穿透 3.063mm
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden

# 3) 10 层方块塔（Week 5 里程碑：站稳！）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
#    期望：bodies=11 shapes=11 contacts=10 contact_points=20 max_penetration_mm=3.763，
#    avg_frame_ms≈1.2；box y=0.497..9.478，|tilt|≤0.13°
#    (600 步 = 10 s 里程碑：max_penetration_mm=3.893)

# 4) 斜面摩擦：θ=15°、μ=0.30 > tanθ=0.268 → 方块静止，穿透 2.686mm
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
```

> `testbed` 的可执行文件路径是**嵌套的**：`build/<dir>/testbed/physics2d_testbed.exe`。
> 无 OpenGL 上下文时 smoke 会返回退出码 2 并打印原因（当前机器可用，NVIDIA 3.3）。
> 未知 demo 名 exit 2（约定）。

**怀疑环境坏了时**（先看 `docs/dev-setup.md` §6）：

- `pwsh` 不存在是正常的，用 `powershell`；
- Ninja 必须在仓库外（`scripts/build.ps1` 自动从 VS 安装目录找）；把 Ninja 放进 `tools/` 会触发
  `cl D8050` / `Permission denied`（沙箱限制）；
- `ctest --preset` 必须在仓库根目录跑（或用 `--test-dir`）；
- death tests 在本沙箱自动 SKIP；把 `build/debug/tests/physics2d_tests.exe` 复制到
  `%LOCALAPPDATA%\Physics2D\testrun\` 再运行可得到 **301/301 全过**（证明 CI 会执行它们）；
- **不要用 PowerShell 5.1 的 `Get-Content`/`Set-Content` 改 UTF-8 中文文档**（会乱码）；
  用编辑工具（本 harness 的 write/edit，或 `-Encoding UTF8` 且显式处理 BOM）。

---

## 3. 已完成内容（Week 1-5）

| 周 | 主题 | 关键交付 | 数据亮点 |
| --- | --- | --- | --- |
| W1 | 数学 + 构建 + 渲染循环 + 积分骨架 | `p2::Math`、`Handle`+`IDPool`、`Body`/`World`、11 阶段 `Step` 骨架、`StepAccumulator`、GLFW+ImGui testbed、CMake/CI | 120 测试；clean build 23.6s |
| W2 | 形状 + 解析碰撞 + Contact 持久化 | `Shape` 池（Circle/Plane）、`CollideCircles`/`CircleAndPlane`、O(N²)+AABB 窄相、`Contact`/`ContactID` 帧间匹配、临时位置响应、Testbed 落地 demo | 185 测试；静止穿透 7.8mm（临时响应） |
| W3 | 法向 Sequential Impulse | `Solver/ContactSolver`（Prepare/WarmStart(no-op)/SolveVelocity）、K_n、累计钳制、迭代、restitution bias、World 接线、SphereStack demo | 210 测试；静止穿透 5.000mm；1→8 迭代残余 0.0833→0.00065 m/s |
| W4 | 凸多边形 SAT + 裁剪 + SATCache | `PolygonShape`、`SAT`/`Clipping`/`CollidePolygons`、`ContactID.Key()` 去 flip、圆-多边形/多边形-平面解析、BoxStack demo | 278 测试；Key 181 帧只变 2 次；10 盒塔会塌（W4 基线） |
| W5 | **摩擦锥 + warm starting + split impulse + 稳定堆叠里程碑** | `ContactSolver` 摩擦锥/warm start/Baumgarte 对比基线；新增 `Solver/ContactSolverPosition.cpp` 伪速度位置层；`Body` 伪速度字段；删除 W2 `TEMPORARY` 位置推出；World 接线；DebugUI 4 面板 + FrictionRamp demo；W4 崩塌基线改稳定期望 | **301 测试**；**10 盒 10 s：穿透 5.223mm、顶层 y 9.478、倾斜 0.0046 rad、无 NaN**；摩擦 20 格扫描全过；split impulse 不注入能量（0 vs Baumgarte 1.14 m/s）；确定性 343 float 逐位一致 |

**模块 → 文件**（只列关键入口，完整用 `git ls-files`）：

```text
include/physics2d/Math/*.h            Vec2 / Mat2 / Rot / Transform / AABB / MathUtils
include/physics2d/Core/               World.h / Body.h / BodyDef.h / Shape.h / Settings.h /
                                      Handle.h / IDPool.h / TimeStep.h / BroadPhaseInterface.h / Profiler.h
include/physics2d/Collision/          CircleShape / PlaneShape / PolygonShape / Analytic / Manifold /
                                      Contact / ContactID / SAT / Clipping / SATCache / ShapeTypes
include/physics2d/Solver/             ContactSolver.h
src/{Math,Core,Collision,Solver}/*.cpp   （Solver 有 ContactSolver.cpp + ContactSolverPosition.cpp）
tests/unit/*.cpp                      GLOB 自动收集（加文件不用改 CMake）
testbed/                              main.cpp / Renderer / Camera / DebugUI / Demo.h / demos/*
                                      demos: FallingBall / SphereStack / BoxStack / FrictionRamp
docs/  scripts/  bench/(占位)  tests/regression/(W6-7 占位)
```

---

## 4. 架构速览（W6 要改的地方）

### 4.1 World::Step 固定流水线（顺序不可变！）

```text
1 IntegrateForces            [done W1]
2 UpdateBroadPhase           [空→W6 实现：同步 AABB/fat margin、查询候选对]
3 UpdateContacts             [done W2/W4：O(N²)+AABB 过滤→W6 换宽相输出；解析/SAT 窄相 + 持久化]
4 BuildIslands               [空，W7]
4b PrepareConstraints(dt)    [done W5：K_n/K_t、restitution+Baumgarte bias、warm-start 播种]
5 WarmStart                  [done W5：施加继承冲量一次]
6 SolveVelocityConstraints   [done W5：法向→切向（摩擦锥）迭代]
7 IntegratePositions         [done W1]
8 SolvePositionConstraints(dt) [done W5：split impulse 伪速度层]
9 Synchronize                [done W2：由形状算 AABB]
10 UpdateSleep               [空，W7]
11 Validate                  [done W1：NaN/Inf 扫描，含伪速度]
```

- 第一阶段（`IntegrateForces`）会先给动态体加重力速度，然后才窄相/求解——**不要改变顺序**。
- 接触锚点（`ContactPoint.localPointA/B`）是**生成时刻**的几何，在 `IntegratePositions` 之前生成；
  测试里不要用 step 之后的 transform 反投影锚点（W2 踩过这个坑）。
- **W6 注意**：换宽相后配对枚举顺序可能变，导致 Contact 列表顺序变 → 物理结果抖动。
  World 必须把宽相回调的 pair 规范化（A<B + 按 body index 排序）后再窄相，才能满足
  "切宽相物理结果 hash 不变"（ROADMAP W6 验收）。

### 4.2 数据流与所有权（W6 直接依赖）

- `World::m_bodies`（槽位数组，index == BodyHandle::index，永不压实）；`IDPool` 管 generation。
- `World::m_shapes`（Shape tagged union）+ `Body::shape`；`AttachShape` 从 density 推质量/惯性。
- `World::m_contacts`（每帧重建）+ `World::m_previousContacts`（上一帧；warm start 数据源）。
- `Body::aabb` 在 `Synchronize()` 里由形状 + transform 算出（W6 要加 fat margin 外推）。
- `ContactSolver`（World 以 `unique_ptr` 前向声明持有）：`Prepare` 缓存 rA/rB、normalMass、tangentMass、
  bias、warm-start 播种；`WarmStart` 施加；`SolveVelocity` 迭代；`SolvePosition` 伪速度层。
- `ContactID::Key()` **不含 flip**；`incidentVertex` 是绝对顶点索引；SAT 平局 tie-break 对称。
  **warm starting 依赖 Key 的帧间稳定，W6 不要动这些语义。**
- `Settings.h` 是唯一常量来源；`SolverSettings` 可运行时改（`World::SetSolverSettings`）；
  testbed 的 Solver 面板实时写回它。

### 4.3 W5 求解器不变量（W6 不要破坏；改求解器前必读）

- 摩擦锥钳制**累计值**：`accT = Clamp(accT + λt, −μ·accN_new, +μ·accN_new)`；法向先解、切向后解；
  `μ = contact.friction`（World 已组合，不要二次组合）。
- warm start **只施加一次**：`Prepare` 播种 `inherited × warmStartFactor`，`WarmStart` 施加 cache 原值；
  默认 `warmStartFactor = 1.0`（<1 会每帧丢弃已收敛解、重新注入 PGS 顺序偏差；实测 10 盒塔
  所有 <1.0 的 factor 都倒）。
- split impulse **只碰伪速度**：真实速度逐位不变、伪速度每步清零、位置层用位置积分后的 transform
  重算杠杆臂（速度层 cache 已过期一步）；位移/转角分别按 `maxLinearCorrection`/`kMaxAngularCorrection` 钳制。
- restitution bias 在 `Prepare`（warm start 之前）由初始接近速度捕获，低于阈值时 bias=0。

---

## 5. 该做的：Week 6 任务卡（可直接照做）

> 设计文档 Week 6：IBroadPhase + BruteForce 基线、SAP（端点数组 + 增量排序）、
> World 宽相接线、bench 量化、Pyramid200/StressBenchmark demo、headless 回归；
> 另补做冷热拆分（W4→W5→W6 二次顺延）。tag `v0.6-sap`。

### 5.1 目标与验收（DoD）

| 项 | 验收标准 |
| --- | --- |
| IBroadPhase | 接口已存在（`Core/BroadPhaseInterface.h`，纯虚 + `PairCallback`，约定 A<B 不重复）；BruteForce 与 SAP 都实现它 |
| BruteForce | 配对集合与"直接双重循环 vector<AABB>"参考实现**完全一致**；增删 proxy 500 次 id 复用正确；回调严格 A<B 无重复 |
| SAP | 端点数组 `{float value; int32_t proxyId : 31; uint32_t isMin : 1}` 单一 vector；首帧 `std::sort`、之后冒泡式局部移动（插入排序/帧间相干性）；与 BruteForce 在 10 组随机场景（50–500 物体）**配对集合完全一致** |
| World 接线 | `UpdateBroadPhase` 同步 proxy（fat AABB margin via `AABB::Extend`，平面特判）；`SetBroadPhase` 运行时切换；**切宽相物理结果 hash 不变**（pair 规范化排序） |
| bench | `bench/bench_broadphase`：N∈{50,200,500,2000} × 1000 帧、两种实现各一遍；**真实数字表**（写清测量条件）进 README/`docs/profiling.md` |
| demo | `SceneBuilders.h` 共享场景构造；`Pyramid200`/`StressBenchmark`；ImGui 下拉框切宽相 + 实时耗时显示 |
| 回归 | `tests/regression/` 的 regression_main + golden hash + 比较脚本（可与 W7 合并，但尽量本周出骨架） |
| 冷热拆分 | `Body`→`BodySim`/`BodyState` 并行数组；**301 个测试全绿、行为 hash 不变** |
| 回归 | **既有 301 个测试保持通过**（除宽相相关的顺序规范化外，物理行为不应变化；失败的测试先怀疑新代码） |
| 质量 | 三 preset 零警告；debug/release/sanitize 全绿；sanitize 干净 |
| 文档/tag | devlog W6、README 性能表、ROADMAP ✅、architecture（宽相/数据布局）、dev-interfaces（R1/R2/R3/R13/R15）、AGENTS 收尾；`v0.6-sap` |

### 5.2 Phase A（Lead 先做：接口/接线骨架）

1. 读设计文档第 6.2 节 + P11/P12；确认 `IBroadPhase`/`PairCallback` 签名已够用
   （W1 预留：`AddProxy(AABB)->int32_t`、`RemoveProxy`、`UpdateAABB`、`QueryPairs(const PairCallback&)`、
   `Name()`、`GetProxyCount()`；若需追加方法按"只追加"原则改接口并登记）。
2. `World::UpdateBroadPhase` 骨架：由 `Synchronize` 算好的 `Body::aabb` 推 proxy；**fat margin 常量**进
   `Settings.h`（如 `kAabbMargin`）；`SetBroadPhase` 已存在；`UpdateContacts` 的候选对来源换成
   `m_broadPhase->QueryPairs(...)`（保留 O(N²) 作为无宽相时的退路——决定后登记）。
3. `src/CMakeLists.txt` 登记新 .cpp（`Collision/BroadPhaseBruteForce.cpp`、`Collision/BroadPhaseSAP.cpp`、
   `bench/bench_broadphase.cpp` 进 bench target）；先 `-Target physics2d_core` 编译检查。
4. **先定"配对顺序规范化"方案**（宽相回调顺序 → 窄相遍历顺序），再派任务，否则物理 hash 会变。

### 5.3 Phase B（三个实现者并行；沿用已验证的分工）

> 复用 durable teammates：`solver-dev`（数学/求解器；W6 可承担 SAP 端点数组/排序）、
> `core-dev`（World/集成/回归）、`testbed-dev`（UI/demo）。每人独立 build dir，互不碰文件。
> W6 也可以让 `solver-dev` 专做 BruteForce+SAP（纯算法 + 单测），`core-dev` 专做 World 接线 + 回归，
> `testbed-dev` 专做 SceneBuilders/Pyramid200/宽相下拉框。

- **solver-dev（SAP/BruteForce 算法）** — 文件：`src/Collision/BroadPhaseBruteForce.cpp`、
  `src/Collision/BroadPhaseSAP.cpp`、`include/physics2d/Collision/BroadPhase*.h`、
  `tests/unit/test_broadphase_bruteforce.cpp`、`test_broadphase_sap.cpp`。
  要点：proxy 空闲列表 + generation；端点单一 vector；首帧 std::sort、后续冒泡；扫描活跃集合；
  y 轴二次过滤；`QueryPairs` 严格 A<B 且不重复；随机场景对照测试（10 组×50/200/500）。
- **core-dev（World 接线 + 回归）** — 文件：`src/Core/World.cpp`、`tests/unit/test_world_contacts.cpp`
  （宽相等价测试新增到新文件 `tests/unit/test_world_broadphase.cpp`）、
  `tests/regression/*`（可选）、fat AABB margin 生效验证、宽相切换 hash 等价测试；
  冷热拆分若做：`Body.h`/`World.cpp`/`ContactSolver` 全链路（**放最后，单独任务卡**）。
- **testbed-dev（场景与可视化）** — 文件：`testbed/SceneBuilders.h`、`testbed/demos/Pyramid200.*`、
  `testbed/demos/StressBenchmark.*`、`testbed/DebugUI.*`（宽相下拉）、`testbed/main.cpp`、README。
  要点：headless 与 testbed 共用场景生成；切宽相物理结果一致（同种子轨迹 hash）；ImGui 显示
  宽相耗时/配对数。
- **bench**：`bench/bench_broadphase.cpp`（只依赖 core），N∈{50,200,500,2000}×1000 帧，
  输出 markdown 表格；数据必须用 Release 测（Debug 数字不能进 README）。

### 5.4 Phase C（Lead 收尾；顺序固定）

1. `scripts/format.ps1`（就地）→ `-Check` 必须 OK；
2. **删掉 canonical `build/debug`、`build/release`、`build/sanitize`**，用 `build.ps1 -Preset X -RunTests`
   从干净状态跑三套（记录耗时、grep `warning C` 必须 0、ctest 全绿）；
3. 四个 demo smoke + 宽相切换后物理结果对比（同场景两次 run 的 body 状态逐位比较）；
4. 更新文档：`docs/devlog.md`（W6 周报：完成/卡住/数据/下周/提醒）、`docs/ROADMAP.md`（W6 ✅）、
   `docs/architecture.md`（宽相数据流/Step 状态/冷热拆分）、`docs/dev-interfaces.md`（R1/R2/R3/R13/R15
   状态、偏差登记）、`README.md`（W6 特性与真实性能表）、`AGENTS.md`（状态、技术债、W7 入口）、
   **本文件 `HANDOFF.md`**（更新现状/下一周）；
5. git 提交（Conventional Commits，按模块拆 3-5 个）+ `git tag -a v0.6-sap`；
6. 确认 `git status` 干净、`git log/tag` 正确、给用户汇报（含真实数据与偏差）。

---

## 6. 不该做的（红线 / 已知大坑，务必遵守）

### 6.1 项目范围红线（设计文档 0.5，违反会被拒绝）

- ❌ 不做 3D / 软体 / 流体 / 布料 / GPU 加速 / 完整关节系统 / CCD-TOI / 兼容 Box2D API / 引入游戏引擎。
- ❌ 不为了"更强的 demo"扩张范围；新想法进 `docs/ROADMAP.md` 的 Roadmap，不进当前周。

### 6.2 W6 专属的坑（W5 踩过/预见的）

- ❌ **不要改变物理结果**：宽相只决定"哪些对进入窄相"，切换实现后配对集合一致 + 顺序规范化 →
  `World::Step` 结果应逐位一致。测法：同场景两实现跑 600 步，body 状态 memcmp。
  出现差异先查 pair 顺序/去重，不要改窄相或求解器。
- ❌ 不要用 `unordered_set`/`unordered_map` 存配对或 proxy 顺序（非确定性）。
- ❌ 不要把 Ninja 放进仓库、不要用 `pwsh`、不要用 PS 5.1 的 Get/Set-Content 改中文文档。
- ❌ 不要在别人构建时删除/清理 `build/`；并行开发一律独立 `-BuildDir build/<name>`。
- ❌ 不要写没有测量条件的性能数字（bench 必须 Release + 说明 N/帧数/机器）；不要伪造 smoke/测试结果。
- ❌ 不要弱化/删除既有 301 个测试来让新代码变绿；行为变化必须有物理/机制解释并写进 devlog。
- ❌ fat AABB 的 margin 不是越大越好：太大引入假阳性（窄相过滤兜底）并拖慢 SAP 扫描；
  取值进 `Settings.h` 并记录理由。平面必须特判（无限半空间）。
- ❌ 冷热拆分（`BodySim`/`BodyState`）是**行为不变**的重构：跑完必须 301 全绿 + 回归 hash 一致；
  不要在拆分同时改求解器公式。
- ❌ 共享工作区并行开发时别人的 build dir 可能链接旧 obj（stale object）；最终结论一律以
  canonical clean build 为准。

### 6.3 协作红线

- 每个 teammate 只改**任务写作用域**内的文件；Header/CMake/docs/提交由 Lead 独占。
- 跨模块接口问题先消息 Lead，不要直接改对方文件；Lead 用"手算解析解"裁决测试期望，
  **不通过弱化断言来消除失败**。
- 不要运行 `git commit/push/tag`（只有 Lead 做）；teammate 不创建分支。
- 修复循环上限 3 轮：超过就拆任务卡或人工读代码，禁止无限自循环。

---

## 7. 已知技术债 & 待办（W6-W8）

| # | 技术债 | 位置 | 计划 |
| --- | --- | --- | --- |
| 1 | **O(N²) 配对枚举**（在 `UpdateContacts` 里） | `World::UpdateContacts` / `UpdateBroadPhase` | **W6** 换 `IBroadPhase` + BruteForce/SAP |
| 2 | **`Body` 单一结构（冷/热已分区注释，未拆 `BodySim`/`BodyState`）** | `Body.h` + `World.cpp` + `ContactSolver` | **W6**（W4→W5→W6 二次顺延，见偏差登记） |
| 3 | fat AABB margin 未做；宽相未接线 | `Body::aabb`、`World::UpdateBroadPhase` | **W6** |
| 4 | 岛屿/休眠未实现（`BuildIslands`/`UpdateSleep` 空；`GetIslandCount()`=0） | `World.cpp` | W7 |
| 5 | 回归/bench 仍是占位 | `bench/`、`tests/regression/` | **W6**（可与 W7 合并） |
| 6 | Tracy 只有零成本宏，未接入 | `Core/Profiler.h`、CMake 选项 | W6-W8 |
| 7 | CI format 检查/clang-tidy 未纳入（本机无 LLVM clang-tidy；clang-format 经 bootstrap 安装） | `.github/workflows/ci.yml`、scripts | W8 |
| 8 | 多 demo 集未做（Pyramid200/StressBenchmark/Dominoes/IslandSleep500…） | `testbed/demos/` | W6-W7 |
| 9 | README/GIF/性能表未补（SAP 对比、休眠对比、Tracy 数字） | `README.md`、`docs/profiling.md`（未建） | W6-W8 |
| 10 | **30:1 单列高塔默认 8 迭代会塌**（迭代预算限制，非 bug；10 盒是验收场景） | `ContactSolver` | 记录在 README/devlog，不修 |
| 11 | `ContactSolver::SolvePosition` 每帧分配局部 vector（约束缓存） | `ContactSolverPosition.cpp` | 若 W6-7 bench 显示热点再复用缓存（先测量） |

另：`bench/bench_broadphase.cpp`、`tests/regression/README.md` 是占位约定，别误删；
`docs/dev-interfaces.md` 第 5 节是**偏差登记**，有疑问先看它。

---

## 8. 已登记的偏差（避免重复讨论）

完整列表见 `docs/dev-interfaces.md` §5；高频的几条：

- **依赖**：vcpkg → FetchContent（本机无 vcpkg；设计文档允许二选一）。
- **Ninja 位置**：不放仓库（本机沙箱会拒绝工作区内可执行文件的子进程写文件）。
- **验收方式**：sandbox 内 death tests 自动 SKIP；CI/普通机器执行（本机可复制 exe 到工作区外验证）。
- **Contact 文件位置**：`Solver/Contact.h` → `Collision/Contact.h`（World 拥有接触列表，Core 不依赖 Solver）。
- **ContactID**：`Key()` 不含 flip；incidentVertex 用绝对顶点索引；SAT tie-break 对称（warm start 依赖）。
- **`kWarmStartFactor` = 1.0**（W1 计划 0.85；实测所有 <1.0 的 factor 都会让 10 盒塔在 10s 内倒，
  机制是丢弃已收敛解后 PGS 顺序偏差重新注入）。0/0.5/0.85/1.0 仍是实验旋钮。
- **`kSplitImpulseSlop` = `kLinearSlop`**（W1 占位 0.01 = 里程碑验收线，位置层死区等于验收线时永远达不到）。
- **`useSplitImpulse=false` = Baumgarte 速度层 bias**（不是回退 W2 位置推出；W2 推出已整体删除）。
- **`ContactSolver::Prepare` 追加默认参数 `dt`**（Baumgarte bias 需要步长；不在冻结 API 清单内）。
- **冷热拆分**：W4 → W5 末 → **W6** 二次顺延（同周跨模块重构会让里程碑回归无法定位）。
- **testbed**：控制台子系统（不是 `WIN32_EXECUTABLE`，为了 stdout smoke）；Tracy 延后到 W6-8。
- **30 层位置层验收用 32 速度迭代**（8 迭代是速度预算不足，不是位置层缺陷；测试保留 8 迭代塌陷数据）。

---

## 9. 团队协作流程（本仓库已验证有效，直接复用）

### 9.1 角色

- **Lead（你）**：写/冻结公共头 + CMake + 文档；分任务；裁决跨模块测试期望；最终 clean 验证 +
  git 提交/tag；唯一动 git 的人。
- **Teammates（durable、可复用）**：`solver-dev`（数学/几何/求解器，历史完成 W1 Math、W3 Solver、
  W4 Polygon/Analytic、W5 摩擦/warm start/split impulse）、`core-dev`（World/集成测试/W5 稳定性证据）、
  `testbed-dev`（UI/demo/W5 面板 + FrictionRamp）。换会话时它们仍存在但 inactive；
  用 `spawn_teammate` 新建或 `send_message` 唤醒（`list_agents` 查看）。

### 9.2 一轮标准流程

1. `create_goal`（可选，长任务）→ 读设计文档对应周 + 现有头文件；
2. **Lead Phase A**：一次写全该周公共头（新类型/API/字段，只追加）+ `src/CMakeLists.txt` + 占位 .cpp，
   跑 `-Target physics2d_core` 确认头文件可编译；
3. `team_task_create` 每模块一张卡（描述里写清：交付文件清单、实现要求、测试要求、构建命令、
   write scope、完成标准）；`spawn_teammate`/`send_message` 让每个 teammate claim 并开工；
4. 并行开发：每人独立 `-BuildDir build/<name>`；跨模块 transient link 失败用 `-Target physics2d_core`
   先验证编译；接口问题消息 Lead，不互相改文件；
5. **Lead 整合验证**：`format` → 删 canonical build → 三 preset 全绿 → 四个 smoke → 核对数据；
6. **Lead 收尾**：更新 docs（devlog/ROADMAP/architecture/dev-interfaces/README/AGENTS/HANDOFF）→
   Conventional Commits（按模块拆）→ annotated tag → 汇报。

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

# 构建 + 测试（日常）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 并行开发隔离目录（多人/多 agent）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/mywork -NoTestbed -RunTests

# Release / Sanitize（MSVC 是 ASan；UBSan 走 Linux CI）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -RunTests

# 格式化
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check

# 测试台（交互）
.\build\debug\testbed\physics2d_testbed.exe
# 冒烟（无人工干预；四个 demo）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 600 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden
.\build\debug\testbed\physics2d_testbed.exe --demo FrictionRamp --smoke 300 --hidden
```

关键常量/入口：`include/physics2d/Core/Settings.h`（所有魔法数字）、
`include/physics2d/Core/World.h`（Step 注释 + API）、
`include/physics2d/Solver/ContactSolver.h`（求解器契约）、
`src/Solver/ContactSolverPosition.cpp`（split impulse 位置层）、
`include/physics2d/Core/BroadPhaseInterface.h`（W6 主战场）、
`include/physics2d/Collision/ContactID.h`（Key 语义）。
测试可执行文件：`build/debug/tests/physics2d_tests.exe`（`--gtest_filter=` 可单独跑）。

---

## 11. 新会话开场提示词（直接复制粘贴）

> 你是 Physics2D 项目的接手开发者。请先读
> `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D\HANDOFF.md`、
> 同目录 `AGENTS.md`、`docs/ROADMAP.md`、`docs/dev-interfaces.md`，
> 再按 `HANDOFF.md` §2 跑验证基线（预期 301/301、15 skip、0 warning、四个 smoke 通过）。
> 确认基线后从 Week 6 开始：IBroadPhase + BruteForce + SAP、World 宽相接线（pair 顺序规范化）、
> bench 真实耗时表、Pyramid200/StressBenchmark demo、headless 回归，并补做冷热拆分（W4→W5→W6 顺延），
> 目标是 tag `v0.6-sap`。
> 严格遵守 HANDOFF §6 的"不该做的"，用 §9 的团队流程；结束时更新 HANDOFF 状态并汇报真实数据。

---

## 12. 本次交接完成检查清单（写此报告时已确认）

- [x] 工作区干净，HEAD = tag `v0.5-stable-stack`
- [x] 301 个测试全绿（15 sandbox skip）、三 preset 零警告（Week 5 收尾实测）
- [x] 四个 demo（FallingBall/SphereStack/BoxStack/FrictionRamp）smoke 可复现，数据已记入本文件 §2
- [x] **10 方块堆叠 10 s 里程碑达成**：最大穿透 5.223 mm < 10 mm、顶层 y 9.478、倾斜 0.0046 rad、无 NaN
- [x] `docs/`（ROADMAP/dev-interfaces/architecture/devlog/dev-setup）+ README + AGENTS 与代码一致
- [x] 技术债、偏差、红线、团队流程、W6 任务卡、验收数据全部写入本文件
- [x] 无遗留后台任务/未完成 team task；所有 teammate inactive，可唤醒复用

> 下一会话开始后，第一件事：`cd` 到仓库、跑 §2 基线、`team_task_create` 建 W6 任务卡。
> 结束后把本文件 §0/§1/§2/§5 更新到 Week 6 状态，再提交。
