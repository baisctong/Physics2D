# AGENTS.md — Physics2D 仓库工作手册（给后续 AI/开发者）

> 任何新的工作轮次（人或 AI）**先读本文件**，再读 `docs/dev-interfaces.md` 与 `docs/ROADMAP.md`。
> 本文件是"项目宪法 + 交接协议"，避免每个会话重新发明约定、把符号约定搞混。
>
> **换会话/换人接手时，先读 [`HANDOFF.md`](HANDOFF.md)**：一页纸交接（现状、验证基线、
> 该做/不该做、Week 5 任务卡、环境坑、团队流程）。本文件负责长期规则，HANDOFF 负责当前状态。

---

## 0. 项目是什么

`Physics2D`：从零手写的 C++20 2D 刚体物理引擎，用于校招简历与面试深挖。
评价标准是「**链路完整 + 原理正确 + 有量化数据 + 代码可读可讲**」，不是功能多。
设计文档原文在仓库外：`../2D物理引擎项目完整工作流与DeepSeek提示词库.md`（第 5 章按周执行，第 12 章提示词库）。

**当前状态：Week 1-6 已完成**（数学库、Body/World 积分骨架、形状系统 + 圆族解析碰撞、
**凸多边形 + SAT + 裁剪式 Manifold + SATCache**、**摩擦锥 + warm starting + split impulse 求解器**、
**10 方块堆叠 10 秒稳定里程碑**、**BruteForce + SAP 可插拔宽相（2000 物体 7.8x 加速，切宽相物理逐位
不变）**、**headless 确定性回归（量化 hash + golden + CI job）**、Testbed 6 个 demo 与 Solver/Contact/
Broad phase/Status 面板、测试/CI/构建体系；Week 7-8 的接口已预留）。
详见 `docs/ROADMAP.md` 与 `docs/devlog.md`。

---

## 1. 项目宪法（不可协商）

### 1.1 技术栈（不要建议更换）

- C++20；CMake ≥ 3.25 + Ninja（Windows 用 VS2022 / MSVC 19.3x；CI 双平台）。
- `physics2d_core` **零第三方依赖**（只用 C++ 标准库；Tracy 是唯一例外且默认关闭）。
- testbed：GLFW 3.4 + OpenGL 3.3 Core + Dear ImGui（docking 分支，commit 固定）。
- 测试：GoogleTest；数学交叉验证用 Eigen（仅测试 target）。
- 依赖统一走 CMake `FetchContent`（`cmake/Dependencies.cmake`），**不使用 vcpkg**。

### 1.2 分层铁律

- `include/physics2d/`、`src/` 内**绝对禁止** `#include` GLFW / OpenGL / ImGui / SDL（Tracy 例外，走 `Core/Profiler.h`）。
- `physics2d_tests` / `physics2d_bench` 只依赖 core；`physics2d_testbed` 才允许渲染依赖。
- 物理代码不出现"像素"：`kPixelsPerMeter` 只在 `testbed/Camera` 里。

### 1.3 单位与符号约定（违反会导致数值失稳）

- MKS：米/千克/秒/弧度；刚体尺寸 0.1–10 m；默认重力 `(0,-10)`；渲染 `kPixelsPerMeter = 40`。
- 右手系，+x 右，+y 上，逆时针角度为正。
- 接触法线 **从形状 A 指向形状 B**；相对速度 `vRel = (vB + ωB×rB) − (vA + ωA×rA)`。
- 三个 Cross 重载严禁混用：`Cross(Vec2,Vec2)->float`、`Cross(Vec2,float)->Vec2`、`Cross(float,Vec2)->Vec2`。
- 积分：semi-implicit Euler（先速度后位置）；固定步长 `dt=1/60`，可变步长禁止。

### 1.4 编码规范

- `namespace p2`；类型 PascalCase；函数 PascalCase；成员 `m_` 前缀；常量 `kPascalCase`；
  文件名 = 主类型名；公共头 `include/physics2d/<模块>/`，实现 `src/<模块>/`。
- 有返回值的函数一律 `[[nodiscard]]`；浮点统一 `float`；除法/归一化必须有零值保护。
- 核心库不抛异常：Debug 用带消息的 `assert`，Release 用返回值/`std::optional`；**不静默修正非法输入**。
- 所有魔法数字集中在 `include/physics2d/Core/Settings.h`；本文件没有的常量名不允许散落。
- 注释用英文解释**为什么**（设计取舍），不是复述代码。

### 1.5 既定设计（不要另起炉灶）

- Broad Phase：`IBroadPhase` 接口 + BruteForce 基线 + SAP 主线（DynamicAABBTree 选做）。
- Narrow Phase：凸多边形 SAT + 面-面裁剪（≤2 接触点）；圆相关走解析路径。
- ContactID 基于特征编码（referenceEdge/incidentVertex/type/flip），帧间稳定是 warm starting 前提。
- Solver：Sequential Impulse（PGS），必须含 warm starting、累计冲量钳制、库仑摩擦锥、restitution 速度阈值、split impulse。
- 岛屿：DFS 分岛，整岛原子休眠/唤醒。
- `World::Step` 流水线顺序固定：积分力→宽相→窄相→建岛→warm start→速度求解→积分位置→位置求解→同步→休眠→Debug 校验。

### 1.6 范围红线（不做，也不建议做）

3D、软体/流体/布料、GPU 加速、完整关节系统、CCD/TOI（除非明确要求）、兼容 Box2D API、引入游戏引擎。
新想法一律先进 `docs/ROADMAP.md` 的 Roadmap，不进当前迭代。

### 1.7 输出要求（AI 协作时）

- 一次只处理一张任务卡/一个模块；不要顺手改别的文件。
- 先给 3-6 行计划（要改哪些文件、风险），再写代码。
- 代码必须完整可编译（含所有 include），不写 TODO/伪代码/"此处省略"。
- 每个非显然决定用一行注释说明理由；需求与宪法冲突时先指出再给替代方案。

---

## 2. 环境与构建（本机实测）

**Windows 注意事项（很重要）**

1. 本机只有 **Windows PowerShell 5.1，没有 `pwsh`**。命令一律用：
   `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 ...`
2. **Ninja 不能放在仓库内**：本机 DSH agent 沙箱会拒绝"可执行文件位于工作区内部"的子进程写文件，
   表现为 `cl ... error D8050` / `.ninja_lock Permission denied`。
   `scripts/build.ps1` 会自动使用 VS 自带 Ninja（或 `%LOCALAPPDATA%\Physics2D\tools`），不要复制到 `tools/`。
3. 首次 configure 会 FetchContent 下载 gtest/Eigen/GLFW/ImGui（约 1-2 分钟）。
4. **不要用 PowerShell 的 `Get-Content/Set-Content` 改写 UTF-8 中文文档**：PS 5.1 会把无 BOM 的
   UTF-8 当 ANSI 读，导致中文变成乱码。改文档用编辑器工具（或确认编码后处理）。

常用命令：

```powershell
# 全量：配置 + 编译 + 测试（推荐给日常开发/KPI 验证）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 并行工作隔离构建目录（多人/多 agent 同时开发）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/mywork -NoTestbed -RunTests

# Release（性能测量必须用）/ Sanitizer（MSVC 只有 ASan，UBSan 走 Linux）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -BuildDir build/sanitize -NoTestbed -RunTests

# 没有 Ninja 的环境：VS 生成器兜底
cmake --preset vs2022-debug
cmake --build --preset vs2022-debug
ctest --preset vs2022-debug
```

---

## 3. 文件地图（新增代码放哪里）

| 你要做的事 | 文件位置 | 是否要改 CMake |
| --- | --- | --- |
| 新增 core 模块实现 | `src/<模块>/*.cpp` | 要：`src/CMakeLists.txt` 显式源文件列表追加 |
| 新增公共头 | `include/physics2d/<模块>/*.h` | 不要（PUBLIC include 目录已配置） |
| 新增单测 | `tests/unit/test_*.cpp` | 不要（GLOB CONFIGURE_DEPENDS 自动收集） |
| 新增 demo | `testbed/demos/*.cpp` | 不要（GLOB 自动收集；demo 需在 `testbed` 注册表登记） |
| 新增依赖 | `cmake/Dependencies.cmake` | 只允许 testbed/tests/bench 消费；core 不允许 |
| 新增常量 | `include/physics2d/Core/Settings.h` | 不要 |
| 文档 | `docs/*.md`、根 README / AGENTS | 不要 |

**公共 API 冻结**：`World` 全部 public 方法、`Body/BodyDef/Handle/IDPool/TimeStep/Settings`、
`Math` 全部签名、`Shape/ShapeType/MassData/{Circle,Plane,Polygon}Shape`、
`Collide*`/`FlipManifold`/`SAT`/`CollidePolygons`/`FindMaxSeparation`/`ClipSegmentToLine`/`SATCache` 签名、
`IBroadPhase/ContactID/Manifold/Contact` 字段语义。
只允许**追加**；破坏性修改必须在 `docs/dev-interfaces.md` 第 5 节登记理由。

**当前技术债（明确标记，按周清理）**：

- ~~`TEMPORARY (Week 2)` 位置推出~~ → **W5 已删除**，由 `Solver/ContactSolverPosition.cpp` 的
  split impulse 伪速度层取代；`grep TEMPORARY` 现在只应命中文档/历史注释。
- ~~无摩擦 / 无 warm start / 无 split impulse~~ → **W5 已完成**：摩擦锥（钳累计值）、warm starting
  （默认 factor 1.0，完整继承）、split impulse 位置层、Baumgarte 对比基线。
- ~~O(N²) 配对枚举~~ → **W6 已完成**：`IBroadPhase` + `BroadPhaseBruteForce` + `BroadPhaseSAP`，
  fat AABB、proxy 生命周期、候选对规范化（切宽相物理逐位一致），bench 真实数字进 README。
- ~~headless 回归占位~~ → **W6 已完成**：`physics2d_regression` + golden + CTest + CI job。
- **30:1 单列高塔/金字塔底部在默认 8 速度迭代下会塌或超 10mm 穿透**（载荷/迭代预算限制，
  10 盒才是 8 迭代验收场景）；验收场景必须写清迭代数，不要用位置层去硬顶速度迭代不足。
- **`BodySim`/`BodyState` 冷热拆分从 W4→W5→W6→W7 三次顺延**（见 dev-interfaces 偏差）：
  W7 岛屿周做，用 W6 的回归弱 hash 证明行为不变。
- 岛屿/休眠未实现（`BuildIslands`/`UpdateSleep` 空、`GetIslandCount()`=0）→ **W7**。
- Tracy 正式接入（R11）在 W7-W8；CI format/clang-tidy 在 W8。
- **本机增量构建坑**：中文 locale MSVC 的 `/showIncludes` 不被 Ninja 识别 → 改 header 后必须
  `build.ps1 -Clean` 或 clean build（`docs/dev-setup.md` §6、`scripts/build.ps1` 注释）。
- `ContactSolver::SolvePosition` 每帧分配局部约束 vector（若 bench 显示热点再复用缓存，先测量）。

---

## 4. 每周工作的标准动作（Definition of Done）

完成一个任务前必须全部通过：

- [ ] `clang-format`（或保持与 `.clang-format` 一致的手工风格）
- [ ] `scripts/build.ps1 -Preset debug` **零项目警告**（MSVC /W4）
- [ ] `scripts/build.ps1 -Preset debug -RunTests` 全绿（新增测试覆盖新代码 + 边界）
- [ ] `-Preset sanitize` + ctest 干净（至少每周一次；MSVC 为 ASan）
- [ ] 新增公共 API 有 `[[nodiscard]]`（若有返回值）
- [ ] 没有魔法数字散落（全在 Settings.h）
- [ ] 没有渲染头文件进入 core（CI 分层 job 会抓）
- [ ] `docs/devlog.md` 写 3 行说明（用自己的话）
- [ ] commit message 用 Conventional Commits（`feat(solver): ...` / `fix(sat): ...` / `perf(broadphase): ...` / `test(...)`）
- [ ] 周结束：打 tag（`v0.2-plane-contact`、`v0.3-impulse` …）、写周报、更新 README 性能表（如有新数字）

**修复循环上限 3 轮**：编译/测试错误改 3 轮还不绿 → 拆任务卡或人工读代码，禁止无限 AI 自循环。

---

## 5. 下一周（Week 7）从哪开始

1. 读 `docs/dev-interfaces.md` 第 2 节 R9/R10/R14/R15/R17 行与第 4 节 W7 行；读设计文档第 5 章
   Week 7、第 6.5 节（岛屿与休眠）、提示词 P13（岛屿/休眠）、P15（demo 集）、D05（调试手册）。
2. W7 的产出（详细验收见 `docs/ROADMAP.md` W7 表）：
   - **岛屿构建**（`World::BuildIslands`）：以 awake dynamic body 为起点做 DFS（**显式栈**，不用递归），
     边是 Contact；static/kinematic 作遍历边界（约束跨越、但不入岛、状态不变）；同一岛 body 共享
     `islandId`；岛的 body/contact 打包进连续临时数组；
   - **按岛求解**：整岛拷贝进连续数组 → 求解 → 写回（注释说明 cache 收益）；与"不按岛"结果必须一致；
   - **休眠**（`World::UpdateSleep`）：`sleepTime` 累计、`kSleepLinearTolerance`/`kSleepAngularTolerance`/
     `kSleepTimeThreshold`；**整岛原子睡眠/唤醒**；sleep 岛跳过宽相/窄相/求解但在宽相保留 proxy；
     唤醒条件：外力/冲量/SetTransform 作用于岛内 body，或 awake 物体 AABB 与 sleep 岛 proxy 重叠；
     **多米诺连锁唤醒**测试；
   - **补做冷热拆分 `Body`→`BodySim`/`BodyState`**（W4→W5→W6→W7 三次顺延，见偏差）：行为 hash 不变，
     用 `physics2d_regression` 的量化弱 hash 兜底；
   - demo：`Dominoes`、`IslandSleep500`（休眠前后 Step 耗时对比，进 README 性能表）；
     `GetIslandCount()`/`GetAwakeBodyCount()` 接真实值；testbed 状态面板显示 islands/awake 曲线；
   - 可选：R11 Tracy 接入（8 个 Step 阶段埋点）；Contact 调试面板完整版。
3. 验收：3 岛/合并岛测试；static body 不入岛但约束跨越；sleep 后连续 100 步状态**逐位不变**；
   对 sleep 岛施加冲量 → 整岛唤醒；**既有 321 个测试不得弱化/删除**；回归弱 hash 在纯重构（冷热拆分）
   后必须不变（变了要解释）。
4. 完成后更新 `docs/ROADMAP.md`、`docs/dev-interfaces.md`（R9/R10/R14/R15/R17）、`docs/devlog.md`、
   `README.md` 性能表，跑三 preset，打 tag `v0.7-island-sleep`。

### W6 的宽相约定（改宽相/World 接线前必读）

- **换宽相不得改物理**：候选对必须规范化为 `(i<j, sorted, deduped)` 的 body-index 对再进窄相；
  验收 = 同一场景 无宽相/BruteForce/SAP 跑 N 步 body 状态与接触列表**逐位一致**。
- **宽相只知道 AABB**：`IBroadPhase` 不出现 Body/World 类型；proxy↔body 映射由 World 维护
  （`Body::proxyId` + `World::m_proxyToBody`）。改接口属破坏性变更，先登记偏差。
- **proxy 生命周期**：`UpdateBroadPhase` 是唯一建/更 proxy 的地方；`DestroyBody`/`SetBroadPhase`/`Clear`
  必须同步释放；无 shape 的 body 没有 proxy。
- **fat AABB 只加候选**：`Synchronize` 写 tight + `kAabbMargin`；漏检先查 pair 规范化/时序，不要调大 margin。
- **`ValidateInternalState` 是契约**：SAP 任意时刻（增删/移动后）都必须 true。
- **回归 hash 纪律**：`tests/regression/golden/default_600.txt` 只在**有意的物理/场景变更**时更新，
  devlog 写 old/new 与原因；纯重构（宽相切换、BodySim 拆分、排序优化）hash 不变。
- 不变量清单见 `docs/dev-interfaces.md` 第 6 节。

### W5 的求解器约定（改求解器前必读）

- **摩擦锥钳制累计值**：`accT = Clamp(accT + λt, −μ·accN_new, +μ·accN_new)`；法向先解、切向后解；
  `μ` 用 `contact.friction`（World 已做 `sqrt(μA·μB)`），不要二次组合。
- **warm start 只施加一次**：`Prepare` 播种 `inherited × warmStartFactor`，`WarmStart` 施加 cache
  原值；默认 `warmStartFactor = 1.0`（小于 1 会每帧丢已收敛解、重新注入 PGS 顺序偏差，实测 10 盒塔
  所有 <1.0 的 factor 都倒）。
- **split impulse 只碰伪速度**：位置层不得读写真实速度；每步结束清零伪速度；位置层要用
  **位置积分后**的 transform 重算杠杆臂（速度层 cache 已过期一步）。
- 不变量清单见 `docs/dev-interfaces.md` 第 6 节「W5 起生效的求解器不变量」。

---

## 6. 参考

- 设计文档：第 5 章（周计划）、第 6 章（公式/伪代码）、第 7 章（测试）、第 12 章（提示词）。
- Box2D v2.4.1：`b2ContactSolver` / `b2CollidePolygon` / `b2DynamicTree`（入门优先读 v2.4.1）。
- Erin Catto, *Iterative Dynamics with Temporal Coherence*（GDC 2005）。
