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
| R3 | AABB 同步 | `World::Synchronize()` 已实现（W2：由形状算 `Body::aabb`）；fat AABB margin 未做 | W6 | 加 margin 外推（`AABB::Extend`）；平面形状由宽相特判 | fat margin 可在 ImGui 调；高速物体不漏检 |
| R4 | 形状系统 | **W2/W4 已实现**：`Core/Shape.h` + `Collision/{ShapeTypes,CircleShape,PlaneShape,PolygonShape}.h`、World 形状池、`CreateCircleShape`/`CreatePlaneShape`/`CreatePolygonShape`/`CreateBoxShape`/`AttachShape` | — | 无（多边形已接入质量/AABB/窄相） | P04 全部单测；既有圆/平面测试不回归 |
| R5 | 窄相数据 | **W2/W4 已实现**：圆族解析 + `CollideCircleAndPolygon`/`CollidePolygonAndCircle`/`CollidePolygonAndPlane`/`CollidePlaneAndPolygon` + `SAT`/`Clipping`/`CollidePolygons`/`FindMaxSeparation`/`ClipSegmentToLine`/`SATCache` | — | 无（SAT 路线完成；以后只有 DynamicAABBTree / 关节等扩展） | 9 种位姿、面-面 2 点/顶点-面 1 点、交换 A/B 法线反向 + Key 集合同、旋转 100 帧 Key 稳定 |
| R6 | Contact 持久化 | **W2 已实现 + W5 接入求解器**：`World::UpdateContacts` 按 body 对 + `ContactID::Key()` 继承每点累计冲量（`m_previousContacts`），`isPersistent` 就位；W5 的 `Prepare` 用它播种 warm start（`isPersistent=false` 的点从精确 0 开始） | — | 无。匹配-继承逻辑保持 World 内部私有函数（`MatchAndInheritImpulses`），契约通过 `GetContacts()` 公开视图测试；不新增公共 API | 缓慢旋转 100 帧同一接触 Key 不变（W4 D5）；`isPersistent` 语义 = 全部新点都匹配到前帧点 |
| R7 | 窄相阶段 | **W2 已实现**：`World::UpdateContacts()` = O(N²) 对枚举 + AABB 过滤 + 按形状类型分派 | W6 | 把对枚举换成 `IBroadPhase::QueryPairs`；窄相函数表（3×3 分派）随 Polygon 一起抽 | 与宽相配对数量一致；无 NaN；切宽相行为不变（回归 hash） |
| R8 | 求解器阶段 | **W3 法向 + W5 完整**：`Solver/ContactSolver.{h,cpp}`（Prepare/WarmStart/SolveVelocity）+ `ContactSolverPosition.cpp`（split impulse 位置层）；摩擦锥累计钳制、warm starting（完整继承，factor 只作实验旋钮）、restitution bias、Baumgarte 对比基线；`World::PrepareConstraints(dt)`/`WarmStart`/`SolvePositionConstraints(dt)` 已接线 | — | 无（无关节/无 block solver；W6 只动宽相，不碰求解器） | 10 方块堆叠 10 s 稳定（穿透 4.22 mm）、摩擦角 20 格扫描、`accN>=0` 1000 帧、1000 帧摩擦锥 0 violation、split impulse 真实速度逐位不变 |
| R9 | 岛屿/休眠 | `World::BuildIslands()`、`UpdateSleep()` 为空；`Body::islandId`、`sleepTime`、`allowSleep`、`World::GetIslandCount()`、`GetAwakeBodyCount()` 已留 | W7 | `Solver/Island.{h,cpp}`、`Sleep`；整岛原子睡眠/唤醒 | 3 岛/合并岛测试；sleep 后状态逐位不变；多米诺连锁唤醒 |
| R10 | 接触/岛统计 | `World::GetContactCount()` **W2 已接真实计数**；`GetShapeCount()` 已加；`GetIslandCount()` 仍返回 0（W7） | W7 | 岛屿统计接真实值；回归输出 awake 曲线 | ImGui 面板数字与实际一致 |
| R11 | 性能剖析 | `Core/Profiler.h` 宏已定义（默认空）；CMake 选项 `PHYSICS2D_ENABLE_TRACY`/`PHYSICS2D_TRACY_ROOT` 已留 | W6/W8 | 打开 Tracy、在 8 个 Step 阶段与 SAP 排序/扫描埋点；`Plot` awake/contact 数 | Tracy 时间线与 README 数字对应 |
| R12 | headless 回归 | `tests/regression/` 目录 + README 约定；`tests/CMakeLists.txt` 预留注释块；`MathUtils` FNV-1a/量化 hash 已实现 | W6-W7 | `regression_main.cpp` + `physics2d_regression` target + `golden/*.csv` + `scripts/compare_hash.py` | CI headless job 跑 10000 步 hash 对比；退出码 0/1/2/3/4 语义 |
| R13 | 基准工具 | `bench/bench_broadphase.cpp` 占位 + `physics2d_bench` target 已存在（只依赖 core） | W6 | 参数化场景，输出 markdown/JSON 表格 | N=50/200/500/2000 实测数字进 README |
| R14 | Demo 扩展 | `testbed/Demo.h` 基类 + demo 注册表；`testbed/demos/` 已接入 CMake glob | W4-W7 | 按 P15 加 BoxStack/Pyramid200/FrictionRamp/Dominoes/IslandSleep500/StressBenchmark 等 | ImGui 下拉可切换；headless 与 testbed 共用场景生成（`testbed/SceneBuilders.h`，W6 再抽） |
| R15 | 冷热拆分 | `Body` 内已分「冷/热」注释区；W5 只**追加**了伪速度字段（split impulse 需要），未拆并行数组 | **W6**（W4→W5→W6 二次顺延，见第 5 节偏差登记） | 拆 `BodySim`/`BodyState` 并行数组（设计文档 3.3 的"重构亮点"）；行为 hash 不变 | 全部测试通过、行为 hash 不变 |
| R16 | ContactID 对称性 | **W4 已实现**：`Key()` 不再包含 `flip` 位（flip 是朝向元数据），同一物理接触 A/B 交换后 Key 不变；SAT 的 reference/incident 归属与 flip 位按此约定填写；World 的持久化匹配按 Key 工作 | — | 无 | 交换 A/B 单测 Key 集合相同；旋转 100 帧 Key 只在特征切换帧变化 |
| R17 | 时间步/插值 | `StepAccumulator` 已实现 + `GetAlpha()` | W7 | 渲染插值（可选） | 视觉顺滑，不影响物理确定性 |
| R18 | 求解器参数暴露 | `SolverSettings` 全部字段 + `World::SetSolverSettings` 已实现；**W5 完成 DebugUI 实时面板**（迭代数/warmFactor/useWarmStarting/useSplitImpulse/β/slop/gravity/linearSlop/maxLinearCorrection/useSatCache）+ Contacts 调试面板 + 状态面板（Step 耗时） | — | 无 | 拖滑条不崩、无需重启；10 盒掉到 3 迭代会明显塌、8 迭代稳定（肉眼可见） |

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
| W2 | P05(完整)、P06 | ✅ **已完成**：`Core/Shape.h`、`Collision/{ShapeTypes,CircleShape,PlaneShape,Analytic,Contact}.{h,cpp}`、`World` 形状池/接触列表/临时响应、Testbed 地面演示 | `World::UpdateContacts`（O(N²)）、`Synchronize`、`SolvePositionConstraints`（临时） |
| W3 | P08 | ✅ **已完成**：`Solver/ContactSolver.{h,cpp}`（法向 SI + restitution bias）、World 的 `PrepareConstraints`/`SolveVelocityConstraints` 接线、删除 W2 临时速度投影；Testbed SphereStack demo | `World::SolveVelocityConstraints`（法向部分） |
| W4 | P04、P07 | ✅ **已完成**：`Collision/{PolygonShape,SAT,Clipping}.{h,cpp}`、`SATCache`、`CollideCircleAndPolygon`/`CollidePolygonAndPlane`（P06 剩余）、`ContactID` flip 不计入 Key、World 多边形 API/分派/cache 持久化、BoxStack demo | `World::UpdateContacts` 走 SAT；冷热拆分移到 W5（见偏差） |
| W5 | P09、P10、P14 | ✅ **已完成**：`Solver/ContactSolver.cpp` 扩展（摩擦锥、warm start、Baumgarte 对比基线）、新增 `Solver/ContactSolverPosition.cpp`（split impulse 伪速度位置层）、`Body` 追加伪速度字段、`World` 接线与 `TEMPORARY` 位置推出删除、DebugUI Solver/Contacts/状态面板、FrictionRamp demo、W4 崩塌基线测试改稳定期望；`Body`→`BodySim`/`BodyState` 冷热拆分二次顺延到 W6（见偏差） | `World::WarmStart`、`World::SolvePositionConstraints(dt)`、`World::PrepareConstraints(dt)` |
| W6 | P11、P12、D04 | `Collision/{BroadPhaseBruteForce,BroadPhaseSAP}`、`bench/bench_broadphase` 实现、`testbed/SceneBuilders.h` | `World::UpdateBroadPhase`、`World::Synchronize`；`m_broadPhase` 接线 |
| W7 | P13、P15、P16 | `Solver/{Island,Sleep}`、8 个 demo、`tests/regression/regression_main.cpp` | `World::BuildIslands`、`World::UpdateSleep`；回归 target 加进 `tests/CMakeLists.txt` |
| W8 | W01-W04、D03 | `docs/{architecture,math,profiling}.md` 升级、README 数字表、GIF | 无（文档与验收周） |

---

## 5. 与设计文档的偏差登记（Week 1-5 实际选择）

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
| `Contact` 定义在 `Solver/Contact.h`（P08） | Week 2 移到 `Collision/Contact.h`，字段语义不变 | World 拥有接触列表（窄相输出），而 Core 不能依赖 Solver；移动头文件比让 Core→Solver 反向依赖更干净。W3 的 `ContactSolver` 直接 include 新路径 |
| P06 包含 `CollideCircleAndPolygon` | Week 2 只做圆-圆、圆-平面；圆-多边形随 Week 4 的 `PolygonShape`（P04/P07）一起做 | Week 2 任务表（设计文档 5 章）只要求平面与圆-圆；多边形形状在 W4 才存在，提前做会先写一套临时的多边形表示 |
| Week 2 的碰撞响应 | `SolvePositionConstraints` 为**临时**位置推出 + 法向速度投影（带 restitution），代码标 `TEMPORARY (Week 2)` | 设计文档 D3/D6 本身要求的临时方案：先用最粗暴的办法让球停下/弹起，把"为什么位置修正无法正确表达弹性碰撞"写进 devlog，作为 W3 Sequential Impulse 的动机 |
| 平面形状的 AABB | 用一个 `kPlaneAabbExtent = 1e6 m` 的超大盒子参与 AABB 过滤 | 宽相在 W6 才实现（届时特判平面）；W2 的 O(N²)+AABB 过滤若给平面零尺寸会漏掉地面。超大盒子只影响 1 个静态 body，代价可忽略 |
| 圆形的 `localCenter ≠ 0` | `ComputeCircleMass` 返回绕质心的惯性矩；`World::AttachShape` 用平行轴折算到 body 原点，但 body 原点即旋转中心 | W2 没有角动量响应（临时响应不处理 r×J），W3 求解器落地时再决定是否引入质心偏移的完整处理；现有 API 已预留 center 字段 |
| P04 "内部合并距离 < kLinearSlop 的相邻顶点" | **拒绝**而不是静默合并（`SetPolygon` 返回 false） | 设计文档 3.4 明确要求"绝不静默修正用户输入"；DoD 测试也要求"重复顶点被拒绝"。调用方拿到确定性的 false 自行修正数据 |
| `ContactID::Key()` 在 W1 设计里含 flip 位 | Week 4 改为 `referenceEdge | incidentVertex<<4 | type<<8`，**不含 flip**；`operator==` 仍比较全部字段（含 flip） | P07 D3/D5 要求交换 A/B 后"Key 集合相同"，而 A/B 交换必然翻转 flip 位；身份必须与朝向无关。W5 warm starting 的跨帧匹配也因此更稳 |
| W4 计划里的 `Body`→`BodySim`/`BodyState` 冷热拆分 | W4 → **W5 末** → **W6**（二次顺延）。W5 只做了拆分的前置：`Body` 追加 `pseudoLinearVelocity/pseudoAngularVelocity`（split impulse 热数据） | 设计文档 3.3 原计划"进入求解器时拆分"，但求解器 W3 已落地、W4 是碰撞最重的一周、W5 是本项目最难的求解器周（摩擦+warm start+split impulse+里程碑）。同周再做跨模块热冷重构会让里程碑回归无法定位；W6 宽相周正好只需要 AABB/宽相热数据，拆分收益在那周验证 |
| Collision 依赖 Core（分层方向） | `PlaneShape.cpp` 为 `kPlaneAabbExtent` include 了 `Core/Settings.h`（纯常量头，只依赖 Math，无循环） | 设计文档要求魔法数字集中在 Settings.h；Settings 是叶子头文件，实际耦合为零。W6 拆宽相模块时若需要再引入 Collision 私有常量头 |
| death tests 在 Release 下 | `TestSupport.h::DeathTestFixture` 在 `NDEBUG` 或沙箱禁止写临时文件时 `GTEST_SKIP()`（带原因） | assert 在 Release 被编译掉，"非法输入必须崩"不成立；CI 的 Debug job 真正执行这些断言测试 |
| **W5**：`kSplitImpulseSlop` 计划值 0.01 m | 默认值改为 `kLinearSlop`（0.005 m） | W1 的 0.01 是占位符，恰好等于里程碑验收上限（"最大穿透 <0.01 m"）——位置层的死区等于验收线时永远达不到验收。0.01 保留为**验收阈值**而不是求解器目标；稳态穿透实测 4.22 mm（10 盒） |
| **W5**：`kWarmStartFactor` 计划值 0.85（注释称"1.0 会过冲振荡"） | 默认值改为 **1.0**（完整继承）；0/0.5/0.85/1.0 仍是实验旋钮 | 该注释是 W1 未跑堆叠时的假设。实测 10 盒 600 步、默认 8 迭代、摩擦 0.3、split impulse 开：factor 0 → step 258 塌、0.5 → 314、0.85 → 259、0.90 → 283、0.95 → 332，**1.0 → 不塌**（max pen 5.22 mm、top y 9.478、末秒抖动 0.0248 m/s）。即**所有 <1.0 的 factor 在 10 s 内都失去塔**（具体倒塌步数依赖瞬态，不作契约）。机制：factor<1 每帧丢掉 (1-f) 的已收敛冲量，PGS 必须每帧重解，把约束顺序偏差重新注入——warm start 本意正是压制它。Box2D 2.4 也是完整继承 |
| **W5**：`useSplitImpulse=false` 时"退化为 Baumgarte" | 实装为 `Prepare` 的 Baumgarte bias（`-(β/dt)·max(0, pen-linearSlop)`），**不是**回退到 W2 位置推出 | W2 位置推出已随本次改动整体删除：它不守恒且无角响应，作为对照实验不具可比性。Baumgarte 是设计文档 P10 指定的对照基线（速度层注入能量），实测静置 2 cm 重叠箱：split impulse 真实速度 3e-5 m/s vs Baumgarte 0.18 m/s（=bias 目标） |
| **W5**：`ContactSolver::ContactConstraintPoint` 预留的 `pseudoNormalMass` 字段 | 未添加；位置层直接复用 `normalMass`（K_n） | 伪速度迭代期间位置冻结（伪速度只在迭代结束后积分一次），杠杆臂与 K_n 与速度层同形，冗余缓存只会带来不一致风险 |
| **W5**：`ContactSolver::Prepare` 签名 | 追加 `float dt = kFixedTimeStep`（默认参数，旧 3 参调用仍有效） | Baumgarte bias 是速度目标 `(β/dt)·pen`，需要步长；把 dt 放到 SolverSettings 或成员状态会让"步长唯一来源"变味。World 显式传本步 dt。`Prepare` 不在冻结公共 API 清单内（World/Body/Math/形状/Contact 才是） |
| **W5**：`World::SetUseSplitImpulse(bool)`（P10 要求） | 未新增该方法；用已有的 `World::SetSolverSettings` + `SolverSettings::useSplitImpulse` | 冻结 API 优先：World 已有通用求解器设置入口（R18），再加一个单项 setter 会让配置入口分叉。testbed 面板与测试都走 `SetSolverSettings` |
| **W5**：P09 要求 `MatchAndInheritImpulses` 抽成独立可测函数 | 保持 `World.cpp` 内部私有函数，不新增公共 API；继承契约通过 `World::GetContacts()` 公开视图做端到端测试 | 该函数是 World 持久化策略的实现细节，公开它会冻结一个本可随 W6 宽相等调整的内部接口。测试覆盖等价（`isPersistent`、跨帧继承、Key 匹配、部分匹配） |
| **W5**：P10 的"30 层堆叠 10 s 穿透 < slop" | 位置层压力测试用 **32 速度迭代**（站立窗口 5 s、最大穿透 4.12 mm < slop），并在测试注释/printf 中记录 8 迭代实测塌陷数据（maxPen 0.146 m、max|angle|=π） | 30:1 单列每帧要传递 30 层支撑，8 次 PGS 迭代的传播预算不足（W4 已验证 10 盒 8 迭代都站不住）。这是**速度迭代预算**限制，不是位置层缺陷：位置层验收看站立窗口内穿透 < slop。W5 里程碑明确是 10 盒 @8 迭代 |
| **W5**：P09 的 `World::SetSolverParameters` | 未新增；`SolverSettings` + `SetSolverSettings` 已覆盖全部参数 | 同 R18：避免分叉配置入口 |
| **W5**：`Body` 的 `pseudoLinearVelocity/pseudoAngularVelocity` | 追加在 `Body` 热数据区，未做热冷拆分 | 见 R15/W6 顺延；伪速度是纯热数据，拆分时随 `BodySim` 一起走 |

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

### W5 起生效的求解器不变量（改求解器前先核对）

- **warm start 只施加一次**：`Prepare` 把继承冲量 × `warmStartFactor` 播种进 `ContactConstraintPoint`，
  `WarmStart` 施加 cache 原值；禁止在 `WarmStart` 里二次缩放，也禁止在 `Prepare` 之后重算 restitution bias。
- **钳制累计值，不钳增量**：法向 `accN = max(accN+λ, 0)`、切向 `accT = Clamp(accT+λt, −μ·accN_new, +μ·accN_new)`；
  法向先解、切向后解（摩擦锥半径用更新后的 accN）；`μ` 直接用 `contact.friction`（World 已算 `sqrt(μA·μB)`）。
- **伪速度层只碰伪速度**：`SolvePosition` 不得读写 `linearVelocity/angularVelocity`；每步结束必须清零伪速度；
  穿透修正只能推不能拉（累计伪冲量 ≥ 0）。
- **restitution 阈值**：仅当初始接近速度 `−vn0 > restitutionThreshold` 时 `bias = e·vn0`，否则 0
  （静止堆叠不能被微小接近速度反复补弹）。
- **`isPersistent=false` 的接触点从精确 0 开始**（warm start 只能继承同一物理特征上一帧的解）。
