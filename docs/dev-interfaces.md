# 预留工作接口清单（Week 1 已交付 → Week 2-8 如何接着做 → v1.1 入口）

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
| R1 | 宽相抽象 | **W6 已完成**：`Core/BroadPhaseInterface.h`（接口未改一行）+ `Collision/BroadPhaseBruteForce.{h,cpp}`（O(N²) 参考基线）+ `Collision/BroadPhaseSAP.{h,cpp}`（单 vector 端点数组 + 首帧 `std::sort` / 稳态冒泡增量排序 + 按 proxy id 有序活跃集合 + 完整 `AABB::Overlaps` 二次过滤；`ValidateInternalState()` 可验内部不变量） | — | 无（DynamicAABBTree 仍是可选扩展，不进当前 roadmap 周） | 10 组随机场景×1000 帧（N=50/200/500）：10,000 帧逐帧 SAP == BruteForce，677,584 对比较 0 mismatch；惰性场景端点零移动；Release bench N=50/200/500/2000 加速比 2.12x/4.88x/5.92x/7.70x（Phase C canonical 复测） |
| R2 | 宽相接线 | **W6 已完成**：`World::UpdateBroadPhase` 由 `Body::aabb` 建/更/释放 proxy（`Body::proxyId` + World 的 `m_proxyToBody` 反向映射），`QueryPairs` 结果转成 `(i<j, sorted, deduped)` 的 body-index 候选对，`UpdateContacts` 与 O(N²) fallback 共用 `CreateContactForPair`；`DeliverBody/AttachShape/Clear/SetBroadPhase` 都正确释放 proxy；无宽相时保留 O(N²) fallback | — | 无 | 同一场景 无宽相/BruteForce/SAP 三路 300 步**逐帧 545 floats 逐位一致**（含接触列表与累计冲量）；运行中切换 none→BF→SAP→BF→none 与从未装宽相的参照 World 150 步逐位一致 |
| R3 | AABB 同步 | **W6 已完成**：`World::Synchronize` 在 tight `ComputeShapeAABB` 基础上 `Extend(kAabbMargin=0.1m)`（fat AABB）；`AttachShape` 立即刷新 tight AABB；平面仍是 `kPlaneAabbExtent` 超大盒（margin 对 1e6 无影响）；proxy 每步刷新（不需要速度外推） | — | 无（massive scale / sleeping proxy 留 W7） | margin 只加候选不加接触（0.15m 间隙圆对：候选 1 / 接触 0）；box/circle 的 aabb 与 tight+margin 四边逐位相等 |
| R4 | 形状系统 | **W2/W4 已实现**：`Core/Shape.h` + `Collision/{ShapeTypes,CircleShape,PlaneShape,PolygonShape}.h`、World 形状池、`CreateCircleShape`/`CreatePlaneShape`/`CreatePolygonShape`/`CreateBoxShape`/`AttachShape` | — | 无（多边形已接入质量/AABB/窄相） | P04 全部单测；既有圆/平面测试不回归 |
| R5 | 窄相数据 | **W2/W4 已实现**：圆族解析 + `CollideCircleAndPolygon`/`CollidePolygonAndCircle`/`CollidePolygonAndPlane`/`CollidePlaneAndPolygon` + `SAT`/`Clipping`/`CollidePolygons`/`FindMaxSeparation`/`ClipSegmentToLine`/`SATCache` | — | 无（SAT 路线完成；以后只有 DynamicAABBTree / 关节等扩展） | 9 种位姿、面-面 2 点/顶点-面 1 点、交换 A/B 法线反向 + Key 集合同、旋转 100 帧 Key 稳定 |
| R6 | Contact 持久化 | **W2 已实现 + W5 接入求解器**：`World::UpdateContacts` 按 body 对 + `ContactID::Key()` 继承每点累计冲量（`m_previousContacts`），`isPersistent` 就位；W5 的 `Prepare` 用它播种 warm start（`isPersistent=false` 的点从精确 0 开始） | — | 无。匹配-继承逻辑保持 World 内部私有函数（`MatchAndInheritImpulses`），契约通过 `GetContacts()` 公开视图测试；不新增公共 API | 缓慢旋转 100 帧同一接触 Key 不变（W4 D5）；`isPersistent` 语义 = 全部新点都匹配到前帧点 |
| R7 | 窄相阶段 | **W2 已实现 + W6 换源**：`World::UpdateContacts()` 的候选对来源 = 已安装宽相的规范化 pair 列表；无宽相时退回 W2 的 O(N²) 枚举。两条路径共用 `World::CreateContactForPair`（同一窄相分派，不会漂移）；窄相函数表（3×3 分派）仍显式 switch（未抽函数指针表） | — | 无（函数指针表是可选优化） | 三路（无/BF/SAP）逐位一致；候选对与 fallback 接触列表一致 |
| R8 | 求解器阶段 | **W3 法向 + W5 完整 + W7 按岛**：`Solver/ContactSolver.{h,cpp}`（Prepare/WarmStart/SolveVelocity）+ `ContactSolverPosition.cpp`（split impulse 位置层）+ `Island.cpp`（分区）；摩擦锥累计钳制、warm starting（完整继承，factor 只作实验旋钮）、restitution bias、Baumgarte 对比基线；三求解模式（flat / grouped-direct 默认 / grouped-packed opt-in，逐位一致）；`World::PrepareConstraints(dt)`/`WarmStart`/`SolvePositionConstraints(dt)` 已接线 | **v1.1（W8 评估后未做，列为最高优先技术债）** | 2 点接触 2×2 block solver（消除薄片残余抖动 → 更快入睡；W7 已记录为技术债）；关节未做 | 10 方块堆叠 10 s 稳定；摩擦角 20 格扫描；三求解模式 21,760 次 float 比较 0 差异；回归 hash 不变 |
| R9 | 岛屿/休眠 | **W7 已完成**：`Solver/Island.{h,cpp}`（显式栈 DFS、forward-star、规范序、static/kinematic 边界）+ `World::BuildIslands`（islandId/规范分区）+ `World::UpdateSleep`（整岛原子计时/入睡）+ `WakeIslandOfBody`（整岛唤醒：外力/冲量/transform + 宽相邻近）；休眠岛跳过 AABB 更新/窄相/求解但保留 proxy，休眠-休眠 pair 复用上一帧 contact | — | 无（关节未做；整岛求解顺序变更已由位级测试锁定） | 3 岛/合并岛；500-body 链 1 岛；sleep 后 100 步状态与接触冲量逐位不变；4 个唤醒入口 + 邻近唤醒；多米诺 20/20 无跳过 |
| R10 | 接触/岛统计 | **W7 已完成**：`GetIslandCount()`（真实岛数）、`GetSleepingBodyCount()`（dynamic 休眠数）、`GetAwakeBodyCount()`；testbed Status 面板显示 islands/awake/sleeping | — | 无 | ImGui 面板数字与 World 一致（IslandSleep500：100 岛 / sleeping 500） |
| R11 | 性能剖析 | `Core/Profiler.h` 宏已定义（默认空）；CMake 选项 `PHYSICS2D_ENABLE_TRACY`/`PHYSICS2D_TRACY_ROOT` 已留 | **v1.1（W8 评估后推迟）** | 打开 Tracy、在 8 个 Step 阶段与 SAP 排序/扫描埋点；`Plot` awake/contact 数 | Tracy 时间线与 README 数字对应。**W8 决策**：v1.0 的量化需求由 bench + testbed Status + `docs/profiling.md` 覆盖，不为此引入第三方依赖与 1–3% 采集开销；钩子保持可用 |
| R12 | headless 回归 | **W6 骨架已完成**：`tests/regression/regression_main.cpp` + `physics2d_regression` target（只 link core）+ `tests/regression/golden/default_600.txt` + CTest `regression_default` + CI `regression` job；hash = FNV-1a **量化弱 hash**（quantum 1e-5，跨编译器/平台可移植）作为判定值，exact float-bit hash 仅信息打印；退出码 0 匹配 / 1 不匹配 / 2 用法或 IO 错误 | W7（可选增强） | `--frames` 模板化多场景 golden（W7 的 Pyramid200/休眠场景）；`scripts/compare_hash.py`（当前不需要，C++ 单文件更简单） | `regression_default` 在 debug/release/sanitize 三 preset 的 ctest 全绿；CI headless job 跑同一命令 |
| R13 | 基准工具 | **W6 已完成**：`bench/bench_broadphase.cpp` 完整 CLI（`--broadphase brute|sap|all --bodies 50,200,500,2000 --frames --seed --json --help`）；30×30m 随机盒场景、只计 UpdateAABB+QueryPairs、前 100 帧预热、median/mean、markdown + JSON、打印环境 | — | 无（Tracy 接入在 W6-8 剩余项） | Release 实测表格进 README（Phase C canonical：N=50/200/500/2000 = 2.12x/4.88x/5.92x/7.70x）；小 N 交叉 N≈24–26（W8 测量会话 22–26、W6 记录 25–32，量级一致） |
| R14 | Demo 扩展 | **W4-W8 已完成**：`testbed/Demo.h` 基类 + demo 注册表；**8 个 demo**（FallingBall/SphereStack/BoxStack/FrictionRamp/Pyramid200/StressBenchmark/**Dominoes**/**IslandSleep500**）；`SceneBuilders.h` 共享构造（含 DominoChain/BoxGrid）；Camera 自适应取景；暂停/单步/慢放快捷键；**W8 新增确定性帧捕获（`--capture-dir/--capture-every/--capture-start` → PPM→GIF），5 个 GIF 进 `docs/images/`** | — | 无（GIF 录制与说明已交付，见 `docs/demos.md` + `scripts/record_gifs.ps1`） | 8 个 demo smoke 全 exit 0；宽相切换物理结果不变；GIF 一条命令可复现 |
| R15 | 冷热拆分 | **v1.1 候选（W8 决定不做）**（W4→W5→W6→W7→W8 连续顺延，见第 5 节偏差登记）：W7 的"岛内打包"实测在 400 体规模反而更慢（W8 复测 0.87x，比 flat 慢约 15%）；W8 profiling（`docs/profiling.md`）也没有证据表明 body 布局是热点 | v1.1 | `BodySim`/`BodyState` 并行数组（设计文档 3.3）；行为 hash 不变 | 全部测试通过、回归 hash 不变；**先有 Tracy/profiling 数据显示收益** |
| R16 | ContactID 对称性 | **W4 已实现**：`Key()` 不再包含 `flip` 位（flip 是朝向元数据），同一物理接触 A/B 交换后 Key 不变；SAT 的 reference/incident 归属与 flip 位按此约定填写；World 的持久化匹配按 Key 工作 | — | 无 | 交换 A/B 单测 Key 集合相同；旋转 100 帧 Key 只在特征切换帧变化 |
| R17 | 时间步/插值 | `StepAccumulator` 已实现 + `GetAlpha()`；testbed 已有暂停/单步/慢放（Space / `.` / 1-2-3） | **v1.1（W8 未做）** | 渲染插值（用 alpha 在两步之间插值渲染；需要 Body 上一帧 transform 缓存） | 视觉顺滑，不影响物理确定性 |
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
| W6 | P11、P12、D04 | ✅ **已完成**：`Collision/BroadPhaseBruteForce.{h,cpp}`（参考基线）、`Collision/BroadPhaseSAP.{h,cpp}`（端点数组 + 增量冒泡 + `ValidateInternalState`）、`bench/bench_broadphase.cpp`（完整 CLI + 真实数字）、`tests/regression/`（工具 + golden + CTest + CI job）、`testbed/SceneBuilders.h` + `Pyramid200`/`StressBenchmark` demo + 宽相下拉/候选对显示；`Body::proxyId`、`Settings::kAabbMargin`；`World` 宽相接线与 pair 规范化 | `World::UpdateBroadPhase`、`World::Synchronize`（fat AABB）、`World::CreateContactForPair`、`World::GetBroadPhasePairCount` |
| W7 | P13、P15、P16 | ✅ **已完成**：`Solver/Island.{h,cpp}`（显式栈 DFS + 规范序）、`ContactSolver` 三模式按岛求解、`World::BuildIslands`/`UpdateSleep`/`WakeIslandOfBody`（整岛原子休眠/唤醒、休眠岛跳过宽相 AABB 更新/窄相/求解、宽相邻近唤醒）、`GetIslandCount`/`GetSleepingBodyCount`、`Dominoes`/`IslandSleep500` demo、暂停/单步/慢放、Contact 冲量柱状条；回归 hash 不变 | `World::BuildIslands`、`World::UpdateSleep`、`World::WakeBody`、`World::GetSleepingBodyCount` |
| W8 | W01-W04、D03 | ✅ **已完成（v1.0-release）**：`docs/{architecture,math,profiling,boundary-contracts,demos}.md` 新建/升级（architecture 加总图/时序图/状态机/数据所有权图）、README v1.0（GIF 墙/核心算法/性能表/已知限制/Roadmap）、`docs/resume-interview.md`（简历 bullet + 25 问 + 20 分钟脚本）、边界契约测试、testbed 帧捕获 + 5 个 GIF + `scripts/record_gifs.ps1` | 无（文档与验收周；未做可选优化，理由见第 5 节 W8 偏差） |

---

## 5. 与设计文档的偏差登记（Week 1-8 实际选择）

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
| **W5**：`Body` 的 `pseudoLinearVelocity/pseudoAngularVelocity` | 追加在 `Body` 热数据区，未做热冷拆分 | 见 R15/W7 顺延；伪速度是纯热数据，拆分时随 `BodySim` 一起走 |
| **W6**：`Body::aabb` 的语义 | 从 "tight AABB（形状 AABB）" 改为 **fat AABB = tight + `kAabbMargin`(0.1m)**；`AttachShape` 之后仍是 tight，直到下一次 `Synchronize` | 宽相需要 fat AABB 才能容忍一步滞后与亚毫米移动；O(N²) fallback 也用同一字段，保证两条路径候选对一致。所有测试改为"包含 tight + 外扩 margin"契约，没有弱化：margin 只加候选、不加接触（有专项测试） |
| **W6**：`Body::proxyId` 与 `World` 的 proxy↔body 映射 | `Body` 追加 `int32_t proxyId`（-1 = 无 proxy）；World 额外持有 `m_proxyToBody` 反向映射表 | 宽相接口只交换 proxy id，World 需要反查 body。没有把反向映射塞进 `IBroadPhase`，因为宽相接口是"只知道 AABB、不知道 body"的冻结契约（Section 4.5） |
| **W6**：`UpdateContacts` 保留 O(N²) fallback | 安装宽相时用规范化 pair 列表，未安装宽相（`World` 默认）时仍走 W2 的 O(N²) 枚举；两条路径共用 `CreateContactForPair` | 默认 World 的行为与 W5 完全一致（301 个既有测试直接绿），切换宽相是**选择加入**；也让"宽相 bug"与"窄相 bug"可用三路对比隔离 |
| **W6**：候选对顺序 | 宽相输出必须规范化为 `(i<j, sorted, deduped)` 的 body-index 对（sort + unique），**即使实现已经升序发射** | 接触列表顺序 → PGS 求解顺序 → 物理结果。没有这个规范化，"切宽相物理 hash 不变"就不成立。测试用一个"反序 + 重复 + 逆序发射"的 noisy decorator 证明规范化生效（缺 dedupe 对数翻倍、缺 sort 物理发散） |
| **W6**：SAP 惰性排序 | `AddProxy` 尾部追加端点 + 脏标记，在下一个 `UpdateAABB`/`QueryPairs`/`ValidateInternalState` 观察点做一次 `std::sort`；稳态 `UpdateAABB` 才对两个端点做冒泡。为让 `const` 的 `ValidateInternalState` 任意时刻可验，排序用 `mutable` 成员 | 批量插入（场景加载）比逐个插入逐次排序快得多；稳态仍是增量冒泡（惰性场景端点零移动有测试）。`mutable` 是契约需要（任意时刻内部状态可验），已注释说明 |
| **W6**：`RemoveProxy` 用 `erase` + 索引刷新而非 swap-pop | 端点数组保持有序不变量，删除频率低（body 生命周期级）；注释说明大 N 下的替代方案 | swap-pop 会破坏"端点有序 + 缓存索引"不变量，需要额外重建；删除不是热路径 |
| **W6**：`World` 不暴露规范化 pair 列表（只有 count） | 只加 `GetBroadPhasePairCount()`；"严格 i<j/升序/无重复"由 count 容差 0 + 三路物理逐位一致 + noisy decorator 用例共同证明 | 公开 pair 列表等于把内部排序策略冻结成 API；测试用这三条证据已足够强 |
| **W6**：bench 的"真实接触对"口径 | 用 **tight AABB 精确重叠**计数（不是窄相真实接触），两个实现同一口径；文档注明 | bench 只依赖 core 且刻意不跑物理；AABB 口径与窄相口径的差是固定的 fat-margin 假阳性成本，可解释、可复现。真实接触数由 testbed smoke/World 测试给出 |
| **W6**：回归工具 golden 形态 | `golden/default_600.txt`（文本：弱 hash + frames + bodies + contacts），不是 P16 设想的 `golden/*.csv` + `scripts/compare_hash.py` | 单场景单 hash 用 C++ 单文件工具即可，跨语言脚本只增加维护面；量化弱 hash（1e-5）保证跨平台可移植。W7 若多场景再扩展模板 |
| **W6**：回归工具 hash | 判定值用 FNV-1a **量化弱 hash**（quantum 1e-5）；exact float-bit hash 只打印 | 浮点逐位 hash 在跨编译器/平台（FMA 收缩、libm 差异）下不可靠；量化 1e-5 仍能捕获真实物理回归（穿透/位置漂移远大于 1e-5） |
| **W6**：W6 无 `DetachShape` API | proxy 生命周期测试按"无 shape → 无 proxy；AttachShape 下一步 → 1；替换 shape → 1；DestroyBody → 释放；Clear → 0"口径；`DestroyShape` 未附着 shape 与 proxy 正交 | 冻结 API 优先：W6 不新增 detach；运行中 body 本就无法失去 shape（`DestroyShape` 对已附着 shape assert 拒绝），测试口径反映真实契约 |
| **W6**：冷热拆分（R15）第 **三** 次顺延到 W7 | 只在 `Body` 追加了 `proxyId`（hot block），未拆并行数组 | W6 本身已经改动 hot block（`proxyId` + fat `aabb` 语义）；同一批 213 个热字段访问点如果一周内改两次，回归定位成本翻倍。W6 用"三路逐位一致 + 回归弱 hash"把行为不变钉死，W7（岛屿周，本来就要碰 body 存储）再做拆分并复用这套 hash 兜底 |
| **W7**：岛屿连通性——static/kinematic 是否"传递约束" | **不传递连通性**：只有 dynamic-dynamic contact 是边；static/kinematic 是边界（其 contact 随岛求解但两个岛不会因为共享地面而合并） | 设计文档 §6.5/P13 措辞"能穿过它们传递约束"有歧义，但同一处 DoD 要求"3 个互不接触的堆叠 → 3 个岛"——若地面传递连通性，同一地面上所有堆叠都会合并成 1 个岛，岛屿/休眠直接失效。设计文档的"两盒叠地上同岛"例子由 box-box 接触满足。已用测试钉死（同一地面上两个独立堆叠 = 2 岛） |
| **W7**：proximity wake 的位置 | 从任务卡措辞的 `UpdateBroadPhase` 挪到 `UpdateContacts` 开头的**共享 wake pass**（宽相路径遍历 `m_broadPhasePairs`，fallback 遍历 fat-AABB 的 i<j；同一唤醒时机） | `UpdateBroadPhase` 在 `m_broadPhase == nullptr` 时提前 return，把 wake 放那里会让"无宽相 fallback 世界"永远不被邻近唤醒，破坏 W6 的三路逐位一致（实测 step 87 分歧）。挪到共享路径后三个 W6 等价性测试**未改断言**即恢复 |
| **W7**：设计文档 3.3/6.5 的"整岛 BodyState 打包到连续数组" | **实现但默认关闭**（`SolverSettings::packIslandBodies=false`）；默认 `grouped-direct`（按岛序迭代、直接读写 Body）；`grouped-packed` 保留为 opt-in 并被位级测试覆盖 | Release 实测（W7：400 bodies/760 contacts 单岛、40 reps、8 迭代）：flat 1.741 ms / grouped-direct 1.863 ms（0.93x）/ **packed 2.122 ms（0.82x）**；W8 复测（同场景 8 次中位）：flat 1.860 / direct 1.908（0.97x）/ packed 2.137（**0.87x，比 flat 慢约 15%**），方向一致。400 个 Body ≈ 80 KB 本就装得进 L2，每帧 O(bodies) 拷贝成本 > 局部性收益；Debug 下 packed 反而快（访问放大），说明默认值必须按 Release 定。岛屿的真实收益是休眠跳过与多岛剔除（IslandSleep500 1.74x） |
| **W7**：休眠阈值 0.01 m/s 与"10 盒塔 600 步入睡" | 阈值保持 `kSleepLinearTolerance=0.01`（设计文档/Box2D 同值）；10 盒塔实测 **step 859** 才整岛入睡，测试放宽到 900 步并在注释/README 记录原因 | 静止残余抖动尖峰（逐点 PGS 对 2 点接触的顺序偏差，W5 已实测 jitter max 0.0248 m/s）会重置 sleepTime；正确修法是 2 点接触 2×2 block solver（W8 技术债），不是调大阈值把数字做漂亮 |
| **W7**：Dominoes 骨牌尺寸 | 用 **0.30×1.0 m（hw=0.15）**，不是任务卡示例的 hx=0.05 薄牌 | 0.12 m 宽薄牌静止抖动 maxV≈0.015 m/s > 0.01 阈值 → 永不入睡，无法演示"睡眠 → 推倒 → 连锁唤醒"；0.30×1.0 实测 32 步入睡且 12/12/20/20 连锁唤醒无跳过 |
| **W7**：`ContactSolver` 的休眠跳过谓词 | "双方都 `!isAwake`" 收窄为"双方都 `!isAwake` 且非 static-static" | static body 自 W1 起就是 `isAwake=false`，宽谓词会把 **static-static 非法输入**也跳过 → W5 契约 death test `StaticStaticContactAssertsOnNonPositiveMass` 不再 assert（工作区外跑完整 gtest 才暴露，sandbox 会 SKIP death test）。收窄后契约保持 |
| **W7**：冷热拆分（R15）第 **四** 次顺延到 W8（可选） | 不做 `BodySim`/`BodyState` 并行数组 | W7 的 packed 测量（0.82x）直接否定了"body 布局是当前热点"的假设：该规模下 body 数组本就 cache 友好，每帧拷贝反而慢。正确顺序是先有 profiling 数据（Tracy，W8）证明 body 布局是热点，再做这项纯重构；否则只是把 213 个访问点改一遍、无收益、回归风险大 |
| **W8**：可选优化（block solver / Tracy / 渲染插值 / 冷热拆分） | **全部未做**，登记为 v1.1 候选（顺序：block solver → Tracy → 插值 → 冷热拆分） | W8 是文档与交付周，HANDOFF §6.2 规定"可选优化只有在文档与最终验收完成后才做，且必须用测量证明收益"。W8 的实际产能用于：math/profiling/边界契约/README+GIF/简历面试 + 最终三 preset/8 smoke/A-B/回归/death test 全量验收。把"10 盒塔 859 步入睡、薄片难入睡"如实写进 README 并给出正解（2×2 block solver），比在发布周改物理更负责——block solver 会改变回归 hash，需要 golden old/new 与完整复测窗口 |
| **W8**：GIF 录制工具 | 给 testbed 增加**确定性帧捕获模式**（`--capture-dir DIR --capture-every K [--capture-start N]`，smoke 帧步进；渲染完成后 `glReadPixels` → PPM → Pillow 编码 GIF），并提交 `scripts/record_gifs.ps1` + 4 个优化后的 GIF 到 `docs/images/`（每个 ≤600KB，总计 ≤2.5MB；`.gitignore` 已白名单 `docs/**/*.gif`） | 本机无 ffmpeg、GIF 必须可复现且不能靠"手忙脚乱地录屏"；帧捕获走 smoke 的固定 1/60 步进 → 每次录制视觉完全一致。PPM→Pillow 链路避免引入 C++ 图片编码依赖；默认渲染路径零开销、物理未改。属于 testbed 工具（Layer 2），不违反"核心零第三方依赖" |
| **W8**：文档结构 | 新增 `docs/{math,profiling,boundary-contracts,demos,resume-interview}.md`；`architecture.md` 增加 §0 总图、§3.7 数据所有权图、§4.6 Step 时序图、§4.7 岛屿/休眠状态机；README 重写为 v1.0 结构（GIF 墙/核心算法/性能/已知限制/Roadmap） | 设计文档 10.1/12.5 的 README/架构/数学提示词要求"一张图讲清 + 每个数字有测量条件 + 已知限制坦诚"；文档拆分成"单一事实来源"避免 README 与 docs 数字漂移（`profiling.md` 是性能数字的唯一来源，README 摘录关键表） |
| **W8**：回归 hash | 保持 `588d30e70f9b4ddd` **不变** | W8 只新增文档/边界测试/testbed 捕获工具，没有改任何物理代码或场景；回归 golden 未更新（边界测试也不进回归场景） |

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

### W6 起生效的宽相不变量（改宽相/World 接线前先核对）

- **换宽相不得改物理**：候选对必须规范化为 `(i<j, sorted, deduped)` 的 body-index 对再进窄相；
  验收 = 同一场景 无宽相/BruteForce/SAP 跑 N 步后 body 状态与接触列表**逐位一致**。
- **宽相只知道 AABB**：`IBroadPhase` 接口不出现 Body/World 类型；proxy↔body 映射由 World 维护
  （`Body::proxyId` + `World::m_proxyToBody`）。改接口属破坏性变更，必须走偏差登记。
- **proxy 生命周期**：`UpdateBroadPhase` 是唯一建/更 proxy 的地方；`DestroyBody`、`SetBroadPhase`、
  `Clear` 必须同步释放（否则 proxy 泄漏或 stale id 复用）；无 shape 的 body 没有 proxy。
- **fat AABB 只加候选**：`Synchronize` 写 `tight + kAabbMargin`；`margin` 不得被用来"修"漏检
  （漏检应先查 pair 规范化/时序，不是调大 margin）。
- **`ValidateInternalState` 是契约**：SAP 测试任意时刻（增删/移动后）都必须为 true；
  不允许出现"只在 QueryPairs 之后才一致"的实现。
- **回归 hash 纪律**：`tests/regression/golden/default_600.txt` 的弱 hash 只在**有意的物理/场景变更**
  时更新，且必须在 devlog 写 old/new 与原因；纯重构（宽相切换、BodySim 拆分、排序优化）hash 不变。

### W7 起生效的岛屿/休眠不变量（改 World/求解器前先核对）

- **分组求解必须逐位等价**：岛屿分区是规范序（岛按最小成员 body slot、岛内 bodies 升序、contacts 按
  全局 contact 序）；`flat` / `grouped-direct` / `grouped-packed` 三种模式在同一初始状态下必须产生
  逐位相同的 body 状态与 contact 累计冲量。改岛序/岛内顺序前先想清楚这一点。
- **休眠是整岛原子状态**：不允许单 body 冻结/唤醒（岛边界穿模）；休眠岛必须保留 broad-phase proxy
  与上一帧 contact（否则岛图丢失、无法被邻近唤醒）；休眠 body 的真实速度、伪速度必须为 0。
- **休眠岛零工作**：AABB 更新、窄相、warm start、速度/位置求解全部跳过；其 `ContactPoint` 累计冲量
  冻结不动，唤醒后继续 warm start（这会把 `-0.0` 之类的位级问题暴露出来，见 W7 devlog）。
- **唤醒路径**：外部力/冲量/SetTransform/SetVelocity 必须 `WakeIslandOfBody`（整岛）；宽相邻近
  （awake body 的 fat AABB 与休眠岛 proxy 重叠）也必须整岛唤醒，且必须对**宽相与无宽相 fallback
  两条路径同样生效**（wake pass 必须放在两者共享的 `UpdateContacts` 开头）。
- **阈值不动**：`kSleepLinearTolerance`/`kSleepAngularTolerance`/`kSleepTimeThreshold` 保持设计值；
  入睡慢是收敛质量（残余抖动）问题，正确修法是 block solver，不是调阈值。
