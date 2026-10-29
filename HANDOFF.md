# Physics2D 交接报告（Session Handoff）

> **一句话现状**：Week 1-4 已完成并验证通过（tag `v0.4-sat`，278 个测试全绿、0 警告、工作区干净）。
> **下一步**：Week 5 —— 摩擦 + warm starting + split impulse + **10 方块堆叠稳定里程碑**
> （tag `v0.5-stable-stack`）。
>
> 新会话请按顺序读：
> 1. 本文件（10 分钟，含"该做/不该做/踩过的坑"）
> 2. [`AGENTS.md`](AGENTS.md)（项目宪法 + 工作协议，**必须遵守**）
> 3. [`docs/ROADMAP.md`](docs/ROADMAP.md)（8 周计划）、[`docs/dev-interfaces.md`](docs/dev-interfaces.md)（预留接口 + 偏差登记）
> 4. 需要细节时再查 [`docs/architecture.md`](docs/architecture.md) 与 [`docs/devlog.md`](docs/devlog.md)
>
> 原始设计文档（仓库外）：`E:\Big Fat Fish's Workstation\2D物理引擎项目\2D物理引擎项目完整工作流与DeepSeek提示词库.md`
> 仓库路径：`E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D`
> 当前 HEAD：`8255c06 docs: Week 4 SAT architecture, interface statuses, devlog and Week 5 handoff`

---

## 0. 新会话开场 3 步（别跳过）

1. **读**本文件 §1-§4 + `AGENTS.md`，确认你理解固定 Step 顺序、符号约定、分层铁律。
2. **跑验证基线**（§2）。不先确认基线的会话，后面所有回归都不可信。
3. **按 §5 Week 5 任务卡开工**，用 §9 的团队流程；结束时按 §10 清单收尾（commit + tag + 更新本文件状态）。

---

## 1. 仓库与状态

| 项 | 值 |
| --- | --- |
| 仓库路径 | `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D` |
| Git | `main` 分支，工作区干净（0 changed），120 个 tracked 文件（含本文件） |
| Tags | `v0.1-falling-ball` → `v0.2-plane-contact` → `v0.3-impulse` → `v0.4-sat` |
| 构建目录 | `build/`（**可随时整个删除**，已被 gitignore；里面是各周并行开发目录 + canonical `debug/release/sanitize`） |
| 团队状态 | 所有 teammate 为 inactive；无后台任务；无未完成任务（task-1..task-13 均 completed） |
| 环境 | Windows 11 / i7-12650H / 16GB / VS2022 (MSVC 19.44) / CMake 3.31.12 / Ninja 1.12.1（VS 自带）/ RTX 4060 |

文档地图：

| 文件 | 作用 |
| --- | --- |
| `AGENTS.md` | 项目宪法 + 工作协议 + 技术债 + 下周入口（**每次会话必读**） |
| `docs/ROADMAP.md` | 8 周计划、每周 DoD、已完成周的验收记录 |
| `docs/dev-interfaces.md` | 18 条预留接口状态、接入步骤、**偏差登记（重要）** |
| `docs/architecture.md` | 模块职责、数据布局、Step 流水线状态、公式与技术债说明 |
| `docs/devlog.md` | 每周周报：完成/卡住/数据/下周/提醒（面试素材来源） |
| `docs/dev-setup.md` | 环境、构建、测试台操作、疑难杂症 |
| `README.md` | 对外展示（已写 Week 1-4 特性、数据、Roadmap） |
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

**期望基线（Week 4 结束时实测）**：

| 检查项 | 期望值 |
| --- | --- |
| 构建 | exit 0，MSVC `/W4 /permissive-` **0 个项目警告** |
| ctest | **278/278 passed, 0 failed, 15 skipped**（skipped 全是 sandbox death tests） |
| Release | **279/279 passed**（多 1 个 release-only 契约测试） |
| Sanitize | `-Preset sanitize` 同样 278/278（MSVC 只有 ASan） |
| clean 构建耗时 | debug **52.0s** / release **41.5s** / sanitize **38.5s**（含依赖编译） |

三个 demo 冒烟（**真实输出基线**；BoxStack 会在 Week 5 后变化）：

```powershell
# 1) 单球落地：contacts=1，球心 y≈0.495（半径 0.5），穿透 5.000mm = kLinearSlop
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden

# 2) 3 球叠放：bodies=4 shapes=4 contacts=3，y≈0.494/1.488/2.482，残余速度 ≤0.019 m/s
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden

# 3) 10 层方块塔（Week 4 真实基线：无摩擦/无 warm start → 会塌）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
#   期望：bodies=11 shapes=11 contacts=10；箱子散在 x≈-13..+15、y≈0.495，全部是 ground 接触
```

> `testbed` 的可执行文件路径是**嵌套的**：`build/<dir>/testbed/physics2d_testbed.exe`。
> 无 OpenGL 上下文时 smoke 会返回退出码 2 并打印原因（当前机器可用，NVIDIA 3.3）。

**怀疑环境坏了时**（先看 `docs/dev-setup.md` §6）：

- `pwsh` 不存在是正常的，用 `powershell`；
- Ninja 必须在仓库外（`scripts/build.ps1` 自动从 VS 安装目录找）；把 Ninja 放进 `tools/` 会触发
  `cl D8050` / `Permission denied`（沙箱限制）；
- `ctest --preset` 必须在仓库根目录跑（或用 `--test-dir`）；
- death tests 在本沙箱自动 SKIP；把 `build/debug/tests/physics2d_tests.exe` 复制到
  `%LOCALAPPDATA%\Physics2D\testrun\` 再运行可得到 **278/278 全过**（证明 CI 会执行它们）；
- **不要用 PowerShell 5.1 的 `Get-Content`/`Set-Content` 改 UTF-8 中文文档**（会乱码）；
  用编辑工具（本 harness 的 write/edit，或 `-Encoding UTF8` 且显式处理 BOM）。

---

## 3. 已完成内容（Week 1-4）

| 周 | 主题 | 关键交付 | 数据亮点 |
| --- | --- | --- | --- |
| W1 | 数学 + 构建 + 渲染循环 + 积分骨架 | `p2::Math`（Vec2/Mat2/Rot/Transform/AABB/MathUtils）、`Handle`+`IDPool`、`Body`/`World`、11 阶段 `Step` 骨架、`StepAccumulator`、GLFW+ImGui testbed、CMake/CI | 120 测试；clean build 23.6s |
| W2 | 形状 + 解析碰撞 + Contact 持久化 | `Shape` 池（Circle/Plane）、`CollideCircles`/`CircleAndPlane`、O(N²)+AABB 窄相、`Contact`/`ContactID` 帧间匹配（按 Key 继承冲量）、临时位置响应、Testbed 落地 demo | 185 测试；静止穿透 7.8mm（临时响应） |
| W3 | 法向 Sequential Impulse | `Solver/ContactSolver`（Prepare/WarmStart(no-op)/SolveVelocity）、K_n、累计钳制、迭代、restitution bias、World 接线、SphereStack demo | 210 测试；静止穿透 5.000mm；1→8 迭代残余 0.0833→0.00065 m/s |
| W4 | 凸多边形 SAT + 裁剪 + SATCache | `PolygonShape`（严格校验/质量积分）、`SAT`/`Clipping`/`CollidePolygons`、`SATCache`、`ContactID.Key` 去 flip、圆-多边形/多边形-平面解析、World 分派/cache 持久化、BoxStack demo | 278 测试；方块旋转 181 帧 Key 只变 2 次；cache churn 4 vs 264；**10 盒塔会塌（W4 基线）** |

**模块 → 文件**（只列关键入口，完整用 `git ls-files`）：

```text
include/physics2d/Math/*.h            Vec2 / Mat2 / Rot / Transform / AABB / MathUtils
include/physics2d/Core/               World.h / Body.h / BodyDef.h / Shape.h / Settings.h /
                                      Handle.h / IDPool.h / TimeStep.h / BroadPhaseInterface.h / Profiler.h
include/physics2d/Collision/          CircleShape / PlaneShape / PolygonShape / Analytic / Manifold /
                                      Contact / ContactID / SAT / Clipping / SATCache / ShapeTypes
include/physics2d/Solver/             ContactSolver.h
src/{Math,Core,Collision,Solver}/*.cpp
tests/unit/*.cpp                      GLOB 自动收集（加文件不用改 CMake）
testbed/                              main.cpp / Renderer / Camera / DebugUI / Demo.h / demos/*
docs/  scripts/  bench/(占位)  tests/regression/(W6-7 占位)
```

---

## 4. 架构速览（W5 要改的地方）

### 4.1 World::Step 固定流水线（顺序不可变！）

```text
1 IntegrateForces            [done W1]
2 UpdateBroadPhase           [空，W6]
3 UpdateContacts             [done W2/W4：O(N²)+AABB 过滤 + 解析/SAT 窄相 + 持久化]
4 BuildIslands               [空，W7]
4b PrepareConstraints        [done W3：ContactSolver::Prepare]
5 WarmStart                  [空！W5 实现]
6 SolveVelocityConstraints   [done W3 法向；W5 加摩擦+warm start 生效]
7 IntegratePositions         [done W1]
8 SolvePositionConstraints   [⚠ TEMPORARY W2 位置推出；W5 换 split impulse]
9 Synchronize                [done W2]
10 UpdateSleep               [空，W7]
11 Validate                  [done W1]
```

- 第一阶段（`IntegrateForces`）会先给动态体加重力速度，然后才窄相/求解——**不要在 W5 改变顺序**。
- 接触锚点（`ContactPoint.localPointA/B`）是**生成时刻**的几何，在 `IntegratePositions` 之前生成；
  测试里不要用 step 之后的 transform 反投影锚点（W2 踩过这个坑）。

### 4.2 数据流与所有权（W5 直接依赖）

- `World::m_bodies`（槽位数组，index == BodyHandle::index，永不压实）；`IDPool` 管 generation。
- `World::m_shapes`（Shape tagged union）+ `Body::shape`；`AttachShape` 从 density 推质量/惯性。
- `World::m_contacts`（每帧重建）+ `World::m_previousContacts`（上一帧，用于按
  `ContactID::Key()` 继承 `ContactPoint.normalImpulse/tangentImpulse`）。
- `ContactSolver`（World 以 `unique_ptr` 前向声明持有）：`Prepare` 缓存 `rA/rB、normalMass、bias`；
  `SolveVelocity` 迭代；**每帧把累计冲量写回 `ContactPoint`**（W5 warm start 的数据已经在）。
- `ContactID::Key()` **不含 flip**（朝向元数据），同一物理接触 A/B 交换 Key 不变；
  `incidentVertex` 是入射多边形绝对顶点索引；SAT 平局 tie-break 对称。W5 不要改这些语义。
- `Settings.h` 是唯一常量来源；`SolverSettings` 可运行时改（`World::SetSolverSettings`）。

### 4.3 关键公式约定（W5 会用到）

- `normal` 从 A 指向 B；`vRel = (vB + ωB×rB) − (vA + ωA×rA)`（`Cross(float, Vec2)` 重载）。
- 法向：`K_n = invMassA+invMassB+invIA·Cross(rA,n)²+invIB·Cross(rB,n)²`；
  `λ = -(vn + bias)·normalMass`；累计钳制 `accN = max(accN+λ, 0)`，只施加增量。
- 计算接触点冲量时**positions 在速度阶段冻结**，所以 `rA/rB` 只需 Prepare 一次。
- split impulse 的位移修正上限：`kMaxLinearCorrection`/`kMaxAngularCorrection`（Settings.h 已有）。

---

## 5. 该做的：Week 5 任务卡（可直接照做）

> 设计文档 Week 5：摩擦、冲量累计、warm starting、split impulse、ImGui 参数面板、
> **里程碑 10 方块堆叠 10 秒稳定（穿透 < 0.01m）**、tag `v0.5-stable-stack`。

### 5.1 目标与验收（DoD）

| 项 | 验收标准 |
| --- | --- |
| 摩擦 | 斜面 μ > tanθ 静止、μ < tanθ 下滑（参数化扫描测试）；μ=0 时切向冲量恒 0 |
| 摩擦锥 | 累计切向冲量钳制在 `[-μ·accN, +μ·accN]`；1000 帧后 `accN >= 0` 且 `|accT| <= μ·accN + 1e-6` |
| Warm starting | `ContactPoint` 跨帧继承冲量 + `WarmStart` 施加 `acc·warmFactor`；warmFactor 0/0.5/0.85/1.0 对比表（迭代收敛/残余速度/穿透）；`useWarmStarting=false` 退化为 W4 行为 |
| Split impulse | 伪速度层替换 `TEMPORARY` 位置推出；30 层堆叠穿透 < slop；开启时不向真实速度注入能量 |
| **里程碑** | **10 个 1m 方块塔 10 秒稳定：最大穿透 < 0.01m，顶点位移/倾斜有界，默认 8–10 速度迭代即可**（不再需要 40+） |
| 回归 | **既有 278 个测试保持通过**（W4 那个"10 盒崩塌基线"测试需改为"稳定"期望，并说明原因） |
| 质量 | 三 preset 零警告；debug/release/sanitize 全绿；sanitize 干净 |
| 文档/tag | devlog W5（含 warmFactor/split impulse 对比、摩擦角扫描、叠放数据）、README 性能表、ROADMAP、AGENTS 收尾；`v0.5-stable-stack` |

### 5.2 Phase A（Lead 先做：只改接口，不动实现）

1. **`include/physics2d/Solver/ContactSolver.h`**（追加，不改已有签名/语义）：
   - `ContactConstraintPoint` 追加：`Vec2 tangent; float tangentMass; float accumulatedTangentImpulse;`
     以及 split impulse 需要的 `float pseudoNormalMass`（可选：与 normalMass 相同）——注释说明为 W5 追加。
   - `Prepare` 语义改为：**从 `ContactPoint::normalImpulse/tangentImpulse` 继承**（World 已按 Key 匹配），
     不再每帧清零；`WarmStart` 实装；`SolveVelocity` 里法向→切向。
   - 新增位置层方法（推荐）：`void SolvePosition(std::vector<Body>& bodies, std::vector<Contact>& contacts, float dt, const SolverSettings& settings);`
   - 新增 `void ClearPseudoVelocities(std::vector<Body>& bodies) const;`（或放在 World 里）。
2. **`include/physics2d/Core/Body.h`**（hot 数据区追加）：
   `Vec2 pseudoLinearVelocity{0,0}; float pseudoAngularVelocity = 0;`（注释：split impulse 专用，
   每帧结束清零，**绝不写回真实速度**）。
3. **`include/physics2d/Core/World.h`**：
   - `void SolvePositionConstraints(float dt);`（把 dt 传下去；Step 中调用处同步改）；
   - 在 `Step` 里 `prepare → warmStart → solveVelocity → integrate → solvePosition` 的现有顺序不变；
   - 若打算做冷热拆分（见 §7），**放到 W5 最后**再动，先别碰。
4. `Settings.h`：如需要新常量（例如 `kMaxPseudoVelocity`）集中加在这里；`SolverSettings` 已有
   `useWarmStarting/warmStartFactor/useSplitImpulse/splitImpulseBeta/splitImpulseSlop/linearSlop/
   maxLinearCorrection/restitutionThreshold/useSatCache`，够用。
5. 若新增 .cpp（如 `src/Solver/SolverContactPosition.cpp`），记得在 `src/CMakeLists.txt` 显式登记。
6. 改完先跑 `-Target physics2d_core` 编译检查，再派任务。

### 5.3 Phase B（三个实现者并行；沿用已验证的分工）

> 复用 durable teammates：`math-dev`（求解器/几何）、`core-dev`（World/集成测试）、
> `testbed-dev`（UI/demo）。每人独立 build dir，互不碰文件。

- **solver-dev（math-dev）** — 摩擦 + warm start + split impulse 内层：
  - 文件：`src/Solver/ContactSolver.cpp`、`tests/unit/test_contact_solver.cpp`（+ 可新建 `test_solver_friction.cpp`/`test_solver_warmstart.cpp`）
  - 公式：`t = Skew(n)`（即 `Perp(n)`）、`K_t`、`λt`、**钳累计** `accT ∈ [-μ·accN, +μ·accN]`；
    μ 直接用 `contact.friction`（World 已算 `sqrt(μA·μB)`，不要重复组合）；法向先解、切向后解、同一迭代；
  - warm start：`Prepare` 读继承值，`WarmStart` 施加 `acc*factor`（法向+切向）；
  - split impulse：伪速度求解 `bias = -(β/dt)·max(0, penetration - slop)`，位置迭代用伪速度积分
    （每步位移受 `kMaxLinearCorrection` 限制），最后清零伪速度，真实速度不变；
  - 测试：斜面摩擦角扫描、μ=0、摩擦锥钳制、1000 帧 accN>=0、warmFactor 对比、伪速度不污染真实速度、
    30 层穿透 < slop、迭代 8 次下收敛。
  - 本目录 build：`build/solver5`。
- **core-dev** — World 接线 + 集成测试：
  - 文件：`src/Core/World.cpp`、`tests/unit/test_world_stability.cpp`（新建）、
    `tests/unit/test_world_sat.cpp`（把 W4 "10 盒崩塌基线"改成稳定期望并说明）、
    必要时 `tests/unit/test_world_response.cpp`（split impulse 后稳态穿透应更小）
  - 内容：`WarmStart()` 调 solver；`SolvePositionConstraints(dt)` 调 solver 位置层并替换 TEMPORARY
    位置推出；`Step` 的 dt 传递；`useWarmStarting=false`/`useSplitImpulse=false` 的退化路径；
    **10 盒 10s 稳定测试**（默认 8 迭代、max penetration<0.01、无 NaN、确定性 memcmp）；
    warmFactor/split impulse/cache 的对比数据打印。
  - 本目录 build：`build/core5`。
- **testbed-dev** — 参数面板 + demo 更新：
  - 文件：`testbed/DebugUI.h/.cpp`（新增求解器参数面板与 Contact 面板）、`testbed/main.cpp`、
    `testbed/demos/BoxStack.cpp/.h`（W5 起塔应站稳：更新说明/重置逻辑）、`testbed/README.md`；
    可选新增 `testbed/demos/FrictionRamp.cpp`（P15 的斜面场景：μ vs tanθ 目视验证）。
  - 面板项：velocity/position iterations、warmFactor、gravity、friction、restitution、linearSlop、
    splitImpulseBeta、useWarmStarting、useSplitImpulse、useSatCache、显示开关；状态面板加
    contacts/awake/island(0)/Step 耗时。
  - smoke：`--demo BoxStack --smoke 300 --hidden` 期望 **11 个 body 仍在塔位（y≈0.5..9.5）、
    contacts≥18、最大穿透 <0.01m**；FallingBall/SphereStack 保持。
  - 本目录 build：`build/testbed5`。

### 5.4 Phase C（Lead 收尾；顺序固定）

1. `scripts/format.ps1`（就地）→ `-Check` 必须 OK；
2. **删掉 canonical `build/debug`、`build/release`、`build/sanitize`**，用 `build.ps1 -Preset X -RunTests`
   从干净状态跑三套（记录耗时、grep `warning C` 必须 0、ctest 全绿）；
3. 三个 demo smoke（BoxStack 期望已稳定）+ 需要时把测试 exe 复制到工作区外验证 death tests 全过；
4. 更新文档：`docs/devlog.md`（W5 周报：完成/卡住/数据/下周/提醒）、`docs/ROADMAP.md`（W5 ✅）、
   `docs/architecture.md`（位置层/split impulse 说明、Step 状态）、`docs/dev-interfaces.md`
   （R8 状态、R15 冷热拆分实际决定、偏差登记）、`README.md`（W5 特性与性能数据）、`AGENTS.md`
   （状态、技术债、Week 6 入口）、**本文件 `HANDOFF.md`**（更新现状/下一周）；
5. git 提交（Conventional Commits，按模块拆 3-5 个）+ `git tag -a v0.5-stable-stack`；
6. 确认 `git status` 干净、`git log/tag` 正确、给用户汇报（含真实数据与偏差）。

---

## 6. 不该做的（红线 / 已知大坑，务必遵守）

### 6.1 项目范围红线（设计文档 0.5，违反会被拒绝）

- ❌ 不做 3D / 软体 / 流体 / 布料 / GPU 加速 / 完整关节系统 / CCD-TOI / 兼容 Box2D API / 引入游戏引擎。
- ❌ 不为了"更强的 demo"扩张范围；新想法进 `docs/ROADMAP.md` 的 Roadmap，不进当前周。

### 6.2 W5 专属的坑（都是 W1-W4 真实踩过的教训）

- ❌ **不要用调大迭代数/降低摩擦系数/给 demo 加阻尼去"修" BoxStack 崩塌**。W4 已证明根因是
  "无摩擦 + 无 warm starting"；W5 用真实的摩擦锥 + warm start 解决。任何"稳定"必须是物理机制的
  结果，并在 devlog 记录实测数据。
- ❌ 不要裁剪**切向冲量增量**——必须钳**累计**切向冲量（`accT`），否则摩擦锥不成立。
- ❌ 不要在 split impulse 开启时把伪速度写回真实速度，也不要把 Baumgarte bias 和 split impulse
  同时叠加在速度层（会注入能量）。伪速度每帧结束必须清零。
- ❌ 不要在 warm start 之后才计算 restitution bias：bias 必须在 `Prepare`（warm start 之前）
  由初始相对速度捕获，否则静止接触会被反复"补弹"。
- ❌ 不要改变 `World::Step` 的阶段顺序；不要删 `Synchronize/Validate`。
- ❌ 不要删除/弱化既有 278 个测试。W4 的"10 盒崩塌"是**预期基线**，W5 修复后把它改成"稳定"
  期望并在注释/报告解释；其它测试失败先怀疑新代码。
- ❌ 不要改冻结公共 API 签名（World/Body/形状/ContactID 语义/Math/Solver 已有签名）。
  只允许**追加**（ContactSolver 字段、Body 伪速度字段属于追加），并在 `docs/dev-interfaces.md` 登记。
- ❌ 不要把 `ContactID::Key()` 重新包含 flip，不要改 incidentVertex 为相对索引，不要破坏 SAT
  对称 tie-break（W5 warm starting 依赖 Key 的帧间稳定）。
- ❌ 不要在物理核心（`include/`、`src/`）引入 GLFW/OpenGL/ImGui；宽相仍是 O(N²)（W6 才换）。
- ❌ 不要在物理迭代里用 `unordered_map` 遍历顺序；不要用可变步长；不要把场景随机数带进物理步进。
- ❌ 不要新增第三方依赖到 core；依赖只能经 `cmake/Dependencies.cmake` 给 testbed/tests/bench。
- ❌ 不要用魔法数字——常量只放 `Settings.h`（或 MathUtils.h 的数学常数）。
- ❌ 不要把 Ninja 放进仓库（沙箱限制）；不要用 `pwsh`；不要用 PS 5.1 的 Get/Set-Content 改中文文档。
- ❌ 不要在别的 agent 构建时删除/清理 `build/`；并行开发一律用独立 `-BuildDir build/<name>`。
- ❌ 不要写没有测量条件的性能数字；不要伪造 smoke/测试结果（失败的测试如实报告）。

### 6.3 协作红线

- 每个 teammate 只改**任务写作用域**内的文件；Header/CMake/docs/提交由 Lead 独占。
- 跨模块接口问题先消息 Lead，不要直接改对方文件；Lead 用"手算解析解"裁决测试期望，
  **不通过弱化断言来消除失败**。
- 不要运行 `git commit/push/tag`（只有 Lead 做）；teammate 不创建分支。
- 修复循环上限 3 轮：超过就拆任务卡或人工读代码，禁止无限自循环。

---

## 7. 已知技术债 & 待办（W5-W8）

| # | 技术债 | 位置 | 计划 |
| --- | --- | --- | --- |
| 1 | **`TEMPORARY (Week 2)` 位置推出**（slop+单步钳制的 mass 加权分离，会向低速接触注入势能） | `World::SolvePositionConstraints`（`grep TEMPORARY`） | **W5** 换 split impulse |
| 2 | **无摩擦 / 无 warm starting**（`WarmStart` no-op、`Prepare` 每帧从 0 累计） | `ContactSolver` | **W5** |
| 3 | **10 盒 BoxStack 崩塌基线测试**（W4 记录，断言不站立） | `tests/unit/test_world_sat.cpp` | **W5** 改成稳定期望 + 保留历史数据注释 |
| 4 | O(N²) 配对枚举（在 `UpdateContacts` 里） | `World::UpdateContacts` | **W6** 换 `IBroadPhase` + BruteForce/SAP |
| 5 | `Body` 单一结构（冷/热已分区注释，但未拆 `BodySim`/`BodyState`） | `Body.h` + `World.cpp` + `ContactSolver` | **W5 末**做（原 W4，已登记偏差；时间不够则 W6 并更新登记） |
| 6 | 宽相/回归/bench 仍是占位 | `bench/`、`tests/regression/`、`World::UpdateBroadPhase` | W6-W7 |
| 7 | 岛屿/休眠未实现（`BuildIslands`/`UpdateSleep` 空；`GetIslandCount()`=0） | `World.cpp` | W7 |
| 8 | Tracy 只有零成本宏，未接入 | `Core/Profiler.h`、CMake 选项 | W6-W8 |
| 9 | CI format 检查/clang-tidy 未纳入（本机无 LLVM clang-tidy；clang-format 经 bootstrap 安装） | `.github/workflows/ci.yml`、scripts | W8 |
| 10 | DebugUI 参数面板未做（W5 计划）；contact 调试面板未做 | `testbed/DebugUI.*` | **W5** |
| 11 | 多 demo 集未做（FrictionRamp/Dominoes/Pyramid200/IslandSleep500…） | `testbed/demos/` | W5-W7 |
| 12 | README/GIF/性能表未补（SAP 对比、休眠对比、Tracy 数字） | `README.md`、`docs/profiling.md`（未建） | W6-W8 |

另：`bench/bench_broadphase.cpp`、`tests/regression/README.md` 是占位约定，别误删；
`docs/dev-interfaces.md` 第 5 节是**偏差登记**，有疑问先看它。

---

## 8. 已登记的偏差（避免重复讨论）

完整列表见 `docs/dev-interfaces.md` §5；高频的几条：

- **依赖**：vcpkg → FetchContent（本机无 vcpkg；设计文档允许二选一）。
- **Ninja 位置**：不放仓库（本机沙箱会拒绝工作区内可执行文件的子进程写文件）。
- **验收方式**：sandbox 内 death tests 自动 SKIP；CI/普通机器执行（本机可复制 exe 到工作区外验证）。
- **Contact 文件位置**：`Solver/Contact.h` → `Collision/Contact.h`（World 拥有接触列表，Core 不依赖 Solver；
  `World.h` 只前向声明 `ContactSolver`）。
- **ContactID**：`Key()` 不含 flip；incidentVertex 用绝对顶点索引；SAT tie-break 对称（W5 warm start 依赖）。
- **多边形校验**：P04 的"合并近重复顶点"改为**拒绝**（设计文档 3.4 禁止静默修正；DoD 也要求拒绝）。
- **BoxStack W4 会塌**：无摩擦+无 warm start 的物理结果，已作为 W5 的 before 基线；不是 bug。
- **冷热拆分**：W4 移到 W5 末（避免同周跨模块重构；见 dev-interfaces 偏差表）。
- **testbed**：控制台子系统（不是 `WIN32_EXECUTABLE`，为了 stdout smoke）；Tracy 延后到 W6-8。

---

## 9. 团队协作流程（本仓库已验证有效，直接复用）

### 9.1 角色

- **Lead（你）**：写/冻结公共头 + CMake + 文档；分任务；裁决跨模块测试期望；最终 clean 验证 +
  git 提交/tag；唯一动 git 的人。
- **Teammates（durable、可复用）**：`math-dev`（数学/几何/求解器，历史完成 W1 Math、W4 Polygon/Analytic、
  W3 Solver）、`core-dev`（World/集成测试）、`testbed-dev`（UI/demo）、`sat-dev`（W4 新增，SAT/裁剪）。
  换会话时它们仍存在但 inactive；用 `spawn_teammate` 新建或 `send_message` 唤醒（`list_agents` 查看）。

### 9.2 一轮标准流程

1. `create_goal`（可选，长任务）→ 读设计文档对应周 + 现有头文件；
2. **Lead Phase A**：一次写全该周公共头（新类型/API/字段，只追加）+ `src/CMakeLists.txt` + 占位 .cpp，
   跑 `-Target physics2d_core` 确认头文件可编译；
3. `team_task_create` 每模块一张卡（描述里写清：交付文件清单、实现要求、测试要求、构建命令、
   write scope、完成标准）；`spawn_teammate`/`send_message` 让每个 teammate claim 并开工；
4. 并行开发：每人独立 `-BuildDir build/<name>`；跨模块 transient link 失败用 `-Target physics2d_core`
   先验证编译；接口问题消息 Lead，不互相改文件；
5. **Lead 整合验证**：`format` → 删 canonical build → 三 preset 全绿 → 三个 smoke → 核对数据；
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
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -RunTests

# 格式化
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check

# 测试台（交互）
.\build\debug\testbed\physics2d_testbed.exe
# 冒烟（无人工干预）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
```

关键常量/入口：`include/physics2d/Core/Settings.h`（所有魔法数字）、
`include/physics2d/Core/World.h`（Step 注释 + API）、`include/physics2d/Solver/ContactSolver.h`（W5 主战场）、
`include/physics2d/Collision/ContactID.h`（Key 语义）。
测试可执行文件：`build/debug/tests/physics2d_tests.exe`（`--gtest_filter=` 可单独跑）。

---

## 11. 新会话开场提示词（直接复制粘贴）

> 你是 Physics2D 项目的接手开发者。请先读
> `E:\Big Fat Fish's Workstation\2D物理引擎项目\Physics2D\HANDOFF.md`、
> 同目录 `AGENTS.md`、`docs/ROADMAP.md`、`docs/dev-interfaces.md`，
> 再按 `HANDOFF.md` §2 跑验证基线（预期 278/278、15 skip、0 warning、三个 smoke 通过）。
> 确认基线后从 Week 5 开始：摩擦 + warm starting + split impulse + ImGui 参数面板，
> 目标是 10 方块堆叠 10 秒稳定（穿透<0.01m）并打 tag `v0.5-stable-stack`。
> 严格遵守 HANDOFF §6 的"不该做的"，用 §9 的团队流程；结束时更新 HANDOFF 状态并汇报真实数据。

---

## 12. 本次交接完成检查清单（写此报告时已确认）

- [x] 工作区干净（`git status` 0 changed），HEAD `8255c06`，tag 到 `v0.4-sat`
- [x] 278 个测试全绿（15 sandbox skip）、三 preset 零警告（Week 4 收尾实测）
- [x] 三个 demo（FallingBall/SphereStack/BoxStack）smoke 可复现，数据已记入本文件 §2
- [x] `docs/`（ROADMAP/dev-interfaces/architecture/devlog/dev-setup）+ README + AGENTS 与代码一致
- [x] 技术债、偏差、红线、团队流程、W5 任务卡、验收数据全部写入本文件
- [x] 无遗留后台任务/未完成 team task；所有 teammate inactive，可唤醒复用

> 下一会话开始后，第一件事：`cd` 到仓库、跑 §2 基线、`team_task_create` 建 W5 三张卡。
> 结束后把本文件 §0/§1/§2/§5 更新到 Week 5 状态，再提交。
