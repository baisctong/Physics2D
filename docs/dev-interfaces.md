# 预留工作接口清单（Week 1 已交付 → Week 2-8 如何接着做）

> 这是本仓库最重要的"交接文档"。第 1 周不只交付了能跑的代码，还把后续 7 周的
> **插入点、数据结构、回调/接口签名**全部固定下来，保证之后每周都是"填一个方法 + 加一组测试"，
> 而不是"重构一遍再往上写"。
>
> 对应设计文档：《2D物理引擎项目完整工作流与DeepSeek提示词库》第 3.3 / 4.5 / 4.6 / 5 / 6 章与 P04-P16。

---

## 0. 三条总原则

1. **插入，不重构**：`World::Step` 的 11 个阶段已经是最终顺序，每个阶段对应一个具名私有方法。
   后续工作 = 填充空方法的函数体，不改调用顺序、不改公共签名。
2. **先数据后算法**：Week 1 已经定义了后面所有跨模块交换的数据类型
   （`ContactID` / `Manifold` / `ContactPoint` / `Contact` / `IBroadPhase`），
   算法（SAT、裁剪、SAP、求解器）直接生产/消费这些类型，不需要再改接口。
3. **追加优先**：需要新字段时追加到结构体尾部；需要新能力时追加 `World` 方法或填预留方法。
   任何破坏性修改必须在本文件第 5 节"偏差登记"里写理由。

---

## 1. 已冻结（不要改签名/语义）的公共接口

| 文件 | 冻结内容 |
| --- | --- |
| `include/physics2d/Math/*.h` | 全部类型与自由函数签名（`Vec2`/`Mat2`/`Rot`/`Transform`/`AABB`/`MathUtils`） |
| `include/physics2d/Core/World.h` | 全部 public 方法签名 |
| `include/physics2d/Core/Body.h`、`BodyDef.h`、`Handle.h`、`IDPool.h`、`TimeStep.h`、`Settings.h` | 字段语义、常量名（只允许追加） |
| `include/physics2d/Core/BroadPhaseInterface.h` | `IBroadPhase` + `PairCallback` |
| `include/physics2d/Collision/ContactID.h`、`Manifold.h` | 字段布局与符号约定 |
| `include/physics2d/Solver/Contact.h` | 字段语义 |
| `World::Step` 的 11 阶段顺序 | 不可变（设计文档 4.6） |

---

## 2. 预留接口总表

| # | 预留点 | 位置（当前状态） | 计划周次 | 需要做的事 | 验收标准 |
| --- | --- | --- | --- | --- | --- |
| R1 | 宽相抽象 | `Core/BroadPhaseInterface.h`：纯虚类 + `PairCallback` 已定义，无实现 | W6 | 新增 `Collision/BroadPhaseBruteForce.{h,cpp}`、`BroadPhaseSAP.{h,cpp}`；testbed 下拉框切换 | 两实现的配对集合与暴力参考完全一致（10 组随机场景 × 50/200/500 物体）；bench 出数字 |
| R2 | 宽相接线 | `World::SetBroadPhase` / `GetBroadPhase` / `m_broadPhase` 已存在（默认 null）；`World::UpdateBroadPhase()` 为空 | W6 | 在 `UpdateBroadPhase` 里同步 `Body::aabb` → proxy，`QueryPairs` 结果交 R4 | 切宽相不改变物理结果（回归 hash 一致） |
| R3 | AABB 同步 | `Body::aabb` 字段已留；`World::Synchronize()` 为空 | W6 | 形状 AABB 计算 + fat AABB（`AABB::Extend`） | fat margin 可在 ImGui 调；高速物体不漏检 |
| R4 | 形状系统 | `Body::shape`（`ShapeHandle`）字段已留；无 Shape 类 | W2/W4 | 新增 `Core/Shape.h`、`Collision/CircleShape.h`、`PolygonShape.h`；`World` 形状存储池；`CreateShape`/`AttachShape` | P04/P06 全部单测 |
| R5 | 窄相数据 | `Collision/ContactID.h`、`Manifold.h` 已定义（含 `Key()` 打包） | W2/W4 | `CollideCircles`、`CollideCircleAndPolygon`、`CollidePolygonAndPlane`（W2）；`SAT`+`Clipping`（W4） | 9 种位姿、面-面 2 点/顶点-面 1 点、交换 A/B 法线反向 |
| R6 | Contact 幂等匹配 | `Solver/Contact.h` 已定义（bodyA/B、manifold、friction、restitution、isPersistent） | W2/W5 | `ContactManager`/`UpdateContacts` 持久化；按 `ContactID::Key()` 继承累计冲量（`MatchAndInheritImpulses`） | 缓慢旋转 100 帧同一接触 Key 不变（W4 D5） |
| R7 | 窄相阶段 | `World::UpdateContacts()` 为空 | W2-W5 | 遍历候选对 → `Contact` 列表 | 与宽相配对数量一致；无 NaN |
| R8 | 求解器阶段 | `World::WarmStart()`、`SolveVelocityConstraints()`、`SolvePositionConstraints()` 为空；`SolverSettings` 已定义 | W3/W5 | `Solver/ContactSolver.{h,cpp}`：法向 SI（W3）、摩擦+累计钳制+warm start（W5）、restitution+split impulse（W5） | 10 方块堆叠稳定、动量/能量守恒、`accN>=0` 恒成立 |
| R9 | 岛屿/休眠 | `World::BuildIslands()`、`UpdateSleep()` 为空；`Body::islandId`、`sleepTime`、`allowSleep`、`World::GetIslandCount()`、`GetAwakeBodyCount()` 已留 | W7 | `Solver/Island.{h,cpp}`、`Sleep`；整岛原子睡眠/唤醒 | 3 岛/合并岛测试；sleep 后状态逐位不变；多米诺连锁唤醒 |
| R10 | 接触/岛统计 | `World::GetContactCount()` 返回 0；`GetIslandCount()` 返回 0；`GetAwakeBodyCount()` 当前按类型粗算 | W2/W7 | 接真实计数，供 DebugUI/回归输出 | ImGui 面板数字与实际一致 |
| R11 | 性能剖析 | `Core/Profiler.h` 宏已定义（默认空）；CMake 选项 `PHYSICS2D_ENABLE_TRACY`/`PHYSICS2D_TRACY_ROOT` 已留 | W6/W8 | 打开 Tracy、在 8 个 Step 阶段与 SAP 排序/扫描埋点；`Plot` awake/contact 数 | Tracy 时间线与 README 数字对应 |
| R12 | headless 回归 | `tests/regression/` 目录 + README 约定；`tests/CMakeLists.txt` 预留注释块；`MathUtils` FNV-1a/量化 hash 已实现 | W6-W7 | `regression_main.cpp` + `physics2d_regression` target + `golden/*.csv` + `scripts/compare_hash.py` | CI headless job 跑 10000 步 hash 对比；退出码 0/1/2/3/4 语义 |
| R13 | 基准工具 | `bench/bench_broadphase.cpp` 占位 + `physics2d_bench` target 已存在（只依赖 core） | W6 | 参数化场景，输出 markdown/JSON 表格 | N=50/200/500/2000 实测数字进 README |
| R14 | Demo 扩展 | `testbed/Demo.h` 基类 + demo 注册表；`testbed/demos/` 已接入 CMake glob | W4-W7 | 按 P15 加 BoxStack/Pyramid200/FrictionRamp/Dominoes/IslandSleep500/StressBenchmark 等 | ImGui 下拉可切换；headless 与 testbed 共用场景生成（`testbed/SceneBuilders.h`，W6 再抽） |
| R15 | 冷热拆分 | `Body` 内已分「冷/热」注释区；`World` 公共 API 只暴露基本类型访问器 | W4 | 拆 `BodySim`/`BodyState` 并行数组（设计文档 3.3 的"重构亮点"） | 全部测试通过、行为 hash 不变 |
| R16 | ContactID 对称性 | `ContactID::Key()` 只做位域打包，`flip` 字段已留 | W4 D5 | SAT 生成时归一化 reference/incident 归属，使 A/B 交换后 Key 不变 | 交换 A/B 单测 Key 集合相同 |
| R17 | 时间步/插值 | `StepAccumulator` 已实现 + `GetAlpha()` | W7 | 渲染插值（可选） | 视觉顺滑，不影响物理确定性 |
| R18 | 求解器参数暴露 | `SolverSettings` 全部字段 + `World::SetSolverSettings` 已实现 | W5 | DebugUI 滑条实时生效（迭代数、warmFactor、μ/e 组合、slop、β） | 拖滑条不崩、不需重启 |

---

## 3. "加一个阶段/模块"的标准步骤

以 Week 3 的求解器为例（其余周同构）：

1. **只动预留方法**：在 `src/Core/World.cpp` 的 `WarmStart()` / `SolveVelocityConstraints()` 里加逻辑，
   不要改 `Step()` 的调用顺序。
2. **新增文件**：头文件放 `include/physics2d/<模块>/`，实现放 `src/<模块>/`，并在
   `src/CMakeLists.txt` 的显式源文件列表里登记（core 的源文件列表是显式的，便于审计）。
3. **新增测试**：`tests/unit/test_xxx.cpp`（GLOB 自动收集，无需改 CMake）。
4. **不改变公共签名**；如必须追加字段，追加到结构体尾部并在本文件第 5 节登记。
5. **跑四件套**：`scripts/build.ps1 -Preset debug -RunTests`、`-Preset sanitize`、
   更新 `docs/devlog.md`、打 git tag。
6. 若新增了预留接口的消费方，更新第 2 节表格的状态。

---

## 4. 后续每周的"第一步"清单（对应设计文档 12.7 的提示词顺序）

| 周 | 提示词 | 要交付的文件（新增/填充） | 需要填的预留方法 |
| --- | --- | --- | --- |
| W2 | P05(完整)、P06 | `Core/Shape.h`、`Collision/{CircleShape,Manifold}.{h,cpp}`（Manifold 已有头）、`Collision/Plane` 解析路径 | `World::UpdateContacts`（先 O(N²) 遍历）、`Body::shape` 使用 |
| W3 | P08 | `Solver/{ContactSolver}.{h,cpp}`、`Body::ApplyLinearImpulse`（已实现，直接用） | `World::SolveVelocityConstraints`（仅法向） |
| W4 | P04、P07 | `Collision/{PolygonShape,SAT,Clipping}.{h,cpp}`、`ContactID` 对称映射 | `World::UpdateContacts` 走 SAT；W4 末做 R15 冷热拆分 |
| W5 | P09、P10、P14 | `Solver/ContactSolver` 扩展：摩擦锥、累计钳制、warm start、restitution、split impulse；DebugUI 参数面板 | `World::WarmStart`、`World::SolvePositionConstraints` |
| W6 | P11、P12、D04 | `Collision/{BroadPhaseBruteForce,BroadPhaseSAP}`、`bench/bench_broadphase` 实现、`testbed/SceneBuilders.h` | `World::UpdateBroadPhase`、`World::Synchronize`；`m_broadPhase` 接线 |
| W7 | P13、P15、P16 | `Solver/{Island,Sleep}`、8 个 demo、`tests/regression/regression_main.cpp` | `World::BuildIslands`、`World::UpdateSleep`；回归 target 加进 `tests/CMakeLists.txt` |
| W8 | W01-W04、D03 | `docs/{architecture,math,profiling}.md` 升级、README 数字表、GIF | 无（文档与验收周） |

---

## 5. 与设计文档的偏差登记（Week 1 实际选择）

| 设计文档 | 实际做法 | 理由 |
| --- | --- | --- |
| vcpkg manifest（`vcpkg.json`） | **FetchContent**（`cmake/Dependencies.cmake`），不引入 `vcpkg.json` | 本机无 vcpkg；设计文档 2.2 明确允许二选一，且要求"全项目只用一种"。FetchContent 让 `git clone` 后可直接配置 |
| Ninja 在 `tools/`（bootstrap 下载到仓库内） | Ninja **不放进仓库**：优先用 VS 自带 Ninja，否则下载到 `%LOCALAPPDATA%\Physics2D\tools` | 本机 agent 沙箱会限制"可执行文件位于工作区内部"的子进程写文件；放仓库内会导致 cl/D8050 与 `.ninja_lock` Permission denied。仓库外则完全正常 |
| Tracy 在 Week 1 可用 | 只保留零成本 `P2_PROFILE_*` 宏与 CMake 选项；真正接入推迟到 W6-W8 | Week 1 没有需要剖析的热点；Tracy 客户端会拖慢首次配置。钩子已就位，接入是纯 CMake 改动 |
| `Body` 直接三数组 BodySim/BodyState | Week 1-3 单 `Body` 结构（内部分冷/热注释分区），W4 按计划重构 | 遵循设计文档 3.3 的"分阶段实施"：这次拆分本身就是简历亮点；`World` 的公共访问器保证重构不外溢 |
| `-fno-exceptions` | 未启用；核心库约定"不抛异常"（assert + 返回值） | 设计文档 2.4 自己提示与 GoogleTest 冲突，推荐全项目保留异常 |
| testbed 用 `WIN32_EXECUTABLE` | 保持控制台子系统 | Week 1 需要 stdout 打印 smoke 结果/GL 错误；GUI 子系统化留到发布周 |
| CMakePresets 只有 Ninja | 增加 `vs2022-*` 预设（VS 生成器） | 本机沙箱下 Ninja 二进制必须在仓库外；VS 生成器作为零依赖兜底（`cmake --preset vs2022-debug`） |
| CI 用 vcpkg action | CI 直接 FetchContent + 系统 Ninja | 同上，依赖策略统一为 FetchContent |
| clang-tidy 纳入 CI | 暂不纳入（本机无 LLVM/clang-tidy） | clang-format 可经 bootstrap 安装；clang-tidy 留到 W8 工具链收尾 |
| `tests/regression` 在 W1 就有 target | 只建目录与约定文档，target 留到 W6-W7 | 物理行为还不存在时建空回归目标会让 CI 变成假绿 |

---

## 6. 绝对不可破坏的不变量（每周自检）

- **Step 顺序**：积分力 → 宽相 → 窄相 → 建岛 → warm start → 速度求解 → 积分位置 → 位置求解 → 同步 → 休眠 → 校验。
- **符号约定**：右手系、+y 向上、逆时针为正；接触法线 **A → B**；`vRel = (vB + ωB×rB) − (vA + ωA×rA)`；
  `Cross(a,b)->float`、`Cross(a,s)->Vec2`、`Cross(s,a)->Vec2` 三个重载不混用。
- **单位**：MKS；物理代码不出现像素；`kPixelsPerMeter` 只存在于 testbed。
- **固定步长**：`World::Step` 只接受调用方传入的固定 dt；accumulator 是唯一的时间转换点。
- **分层**：`physics2d_core` 不 include GLFW/ImGui/OpenGL（Tracy 例外），不 link 第三方目标。
- **句柄而非指针**：公共 API 只交换 `BodyHandle`/`ShapeHandle`；stale 句柄必须安全失效。
- **确定性**：迭代顺序由 vector 下标决定；禁止 unordered_map 遍历顺序影响物理；场景随机数只用于建场景。
- **魔法数字**：所有容差/系数进 `Settings.h`。
