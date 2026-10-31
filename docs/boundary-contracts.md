# 边界契约与已知限制（v1.0-release · Week 8）

> **这是 Week 8「已知限制 + 边界契约」的单一事实来源**，供 README、`docs/math.md` §10、
> `docs/profiling.md`、面试材料引用。它回答三个问题：
> ① 每个**非法/极端输入**的契约行为是什么（assert？返回值？只是文档化限制？）；
> ② 每条已知限制的**数字、测量条件、为什么不修、正解是什么、如果要做怎么开始**；
> ③ 每个契约**由哪个测试钉住**（精确到 `文件::测试名`，不写编造的测试名）。
>
> **与其它文档的分工**（避免数字漂移）：
> - 本文 = 契约行为与限制清单（输入 → 行为 → 实现位置 → 测试）；
> - [`math.md`](math.md) §10 = 数学/几何/求解器侧的"输入→行为→测试"表（推导视角，与本文互为索引）；
> - [`profiling.md`](profiling.md) = **所有性能/耗时数字的唯一来源**（测量环境与纪律也在这里）；
> - [`testing.md`](testing.md) = **测试规模/三 preset/怎么跑**的唯一来源（本文不重复总数）；
> - [`README.md`](../README.md) §已知限制 = 对外摘要；[`dev-interfaces.md`](dev-interfaces.md)
>   §5/§6 = 偏差登记与"不可破坏的不变量"；[`devlog.md`](devlog.md) = 每周实测历史。
>
> **v1.0 状态声明**：W8 只新增**边界测试、断言校验与文档**，没有改物理行为、Step 顺序、
> 物理常数或公共 API（逐位证据：同一 toolchain 内回归弱 hash 未变（MSVC `588d30e70f9b4ddd`），exact
> `f4390ef86c075a04` 仅作信息打印；见 `profiling.md` §7.1）。W8 的 4 处代码改动清单见 §5。

---

## 0. 三类契约、验证方式与计数口径

### 0.1 三类契约（全文用标签标注）

| 标签 | 含义 | Release（`NDEBUG`）下的行为 | 例子 |
| --- | --- | --- | --- |
| **【断】** Debug assert | 非法输入是**调用方 bug**：Debug 用带消息的 `assert` 立即失败（便于定位）；核心库不抛异常 | assert 被编译掉后走**兜底路径**：安全返回 / 钳制为定义明确的值 / 拒绝写入。**不要依赖 Release 的 assert** | `World::Step` 的非法 dt / 负迭代数；`ContactSolver::Prepare` 的越界 body index |
| **【返】** 返回值契约 | 失败通过**返回值/句柄/指针**表达：`false`、invalid `Handle`、`nullptr`、中性默认值；不 assert、不抛异常 | **所有 build 一致**（Debug 也返回，不崩） | `SetPolygon` 拒绝退化多边形；`CreateBody` 对非法 `BodyDef` 返回 invalid handle；stale 句柄的 getter 返回 0/nullptr |
| **【限】** 文档化已知限制 | 输入**合法**（不是 bug），但物理/精度上不承诺理想结果；**不修**，写清"正解"与"怎么开始" | 同左（行为一致，只是不保证"更好"的结果） | 无 CCD 时高速物体可能隧穿；30:1 塔在 8 迭代下会塌；10 盒塔 859 步才入睡 |

补充约定：
- 核心库**没有 `std::optional` 公共 API、不抛异常**（项目宪法 §1.4）。"返回失败"一律是
  invalid handle / `false` / `nullptr` / 安全默认值。
- **【断】的 Release 兜底必须是"定义明确的"**：`World::Step` 负迭代数在 Release 被**钳到 0**
  （而不是靠 for 循环恰好不执行）；`ContactSolver` 的非法接触点在 Release 被**跳过**（不除零）。
- 契约只保护**公开 API 的输入**；引擎内部（如 `Body::Initialize`、`ContactSolver` 的私有路径）
  的 assert 是"内部误用"防线，不是用户接口。

### 0.2 death test 怎么执行（【断】契约的验证方式）

断言只在 **Debug** 生效；GoogleTest 的 death test 需要子进程写临时文件。本机 agent 沙箱禁止
"工作区内可执行文件"写临时文件，因此 `ctest` 里 death test 会显示 **Skipped**（正常现象，
`tests/unit/TestSupport.h::DeathTestFixture` 会给可见原因），**不是失败**。真正执行方式：

```powershell
# 1) 工作区外运行（本机可行；期望全部 gtest 通过，含所有 death test 真正执行）
$dir = Join-Path $env:LOCALAPPDATA 'Physics2D\testrun'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
Copy-Item "build\debug\tests\physics2d_tests.exe" $dir -Force
Push-Location $dir; .\physics2d_tests.exe; Pop-Location

# 2) CI：Debug job 正常执行（无沙箱限制）
```

- Debug 构建：death test 真正跑；Release/RelWithDebInfo：fixture 直接 `GTEST_SKIP()`（`assert`
  被编译掉，"非法输入必须死"不是该 build 的属性）。
- 跑测试总数 / 三 preset 结果请以 [`testing.md`](testing.md) 为准（Phase C 会写入最终
  canonical 数字；**本文只记录本批新增的 23 + 12 个用例**，见 §4）。

### 0.3 本批（W8）测试与代码改动的边界

| 项 | 内容 | 位置/数量 |
| --- | --- | --- |
| solver/几何侧边界测试 | **23 个用例**（10 常规 + 13 death；套件 `BoundaryPolygon`/`BoundaryDensity`/`BoundaryVelocity`/`BoundaryThinBar`/`BoundaryMassRatio`/`BoundaryDensityDeathTest`/`BoundarySolverDeathTest`/`BoundaryWorldDeathTest`） | [`tests/unit/test_boundary_solver.cpp`](../tests/unit/test_boundary_solver.cpp)，索引见 §4.1 |
| World/宽相/休眠侧边界测试 | **12 个用例**（debug 10 + release-only 2；含 3 death；套件 `WorldLifecycleBoundaryTest`/`WorldStepBoundaryDeathTest`/`WorldStepBoundaryTest`/`WorldSleepBoundaryTest`/`WorldBroadPhaseBoundaryTest`/`WorldStateInvariantTest`/`WorldBoundaryProfilingTest`） | [`tests/unit/test_boundary_world.cpp`](../tests/unit/test_boundary_world.cpp)，索引见 §4.2 |
| 代码改动 | 只加"拒绝/校验/assert"：3 处溢出拒绝（`PolygonShape.cpp`）、非有限 manifold assert（`ContactSolver.cpp`）、body index/pointCount assert（`ContactSolverPosition.cpp`）、Release 负迭代钳 0 + `CheckSane` 覆盖 W7 状态（`World.cpp`） | §5；**不改物理、Step 顺序、常数、公共 API** |

---

## 1. 边界场景总表（输入 → 契约行为 → 实现位置 → 测试）

> 阅读方式：先看标签（【断】/【返】/【限】），再看"测试位置"列——每个行为都有可执行证据。
> 同一契约在旧测试里已钉住的，本文引用旧测试，不重复实现。

### 1.1 几何 / 形状边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| 多边形顶点数 ∉ [3, 8]、`vertices == nullptr` | **【返】** `SetPolygon` 返回 `false`，形状置 invalid（不崩）；**World 层【断】** `CreatePolygonShape` assert + 返回 invalid handle，且不消耗 shape 池槽位 | `src/Collision/PolygonShape.cpp::SetPolygon`；`src/Core/World.cpp::CreatePolygonShape` | `test_boundary_solver.cpp::BoundaryPolygon.DegeneratePolygonsAreRejectedWithoutCrash`；`test_polygon_shape.cpp::PolygonShape.ValidationRejectsDegenerateInput` |
| 非有限坐标（NaN/Inf） | **【返】** `SetPolygon` 返回 `false`、形状 invalid；`CreatePolygonShape`/`CreateBoxShape` 同上的 assert + invalid handle | 同上 | `test_boundary_solver.cpp::BoundaryPolygon.DegeneratePolygonsAreRejectedWithoutCrash`（NaN）；`test_world_sat.cpp::PolygonValidationDeathTest.InvalidPolygonArraysAreRejected` |
| 相邻重复/过近顶点（< `kLinearSlop`）、共线冗余顶点、凹顶点、顺时针绕序、面积≈0 | **【返】** `SetPolygon` 拒绝（`false` + invalid），**不静默合并/修复**（设计文档允许"合并"，本项目按"绝不静默修正输入"选择拒绝，登记在 `dev-interfaces.md` §5） | `src/Collision/PolygonShape.cpp::SetPolygon` 的 5 组守卫 | `test_boundary_solver.cpp::BoundaryPolygon.DegeneratePolygonsAreRejectedWithoutCrash`；`test_polygon_shape.cpp::PolygonShape.ValidationRejectsDegenerateInput`、`RejectedInputInvalidatesAPreviouslyValidShape` |
| **有限但算术溢出**的坐标（边长平方/鞋带和/质心加权和溢出 float，如半宽 1e19） | **【返】** `SetPolygon` 返回 `false`（否则会产生 (0,0) 边法线或 inf/NaN 质心，污染 SAT 与质量属性）；World 层 **【断】** assert + invalid handle——**W8 新增守卫** | `src/Collision/PolygonShape.cpp`（3 处 "Week 8 boundary hardening"）；`src/Core/World.cpp::CreateBoxShape`/`CreatePolygonShape` | `test_boundary_solver.cpp::BoundaryPolygon.FiniteButOverflowingCoordinatesAreRejected`；`BoundaryDensityDeathTest.WorldCreateShapeRejectsOverflowingGeometry` |
| box 半宽/半高非有限或 ≤ 0 | **【返】** `SetPolygonAsBox` 返回 `false`；`CreateBoxShape` **【断】** assert + invalid handle | `src/Collision/PolygonShape.cpp::SetPolygonAsBox`；`World::CreateBoxShape` | `test_polygon_shape.cpp::PolygonShape.SetPolygonAsBoxRejectsInvalidParameters`；`test_world_sat.cpp::PolygonValidationDeathTest.InvalidBoxExtentsAreRejected` |
| 圆半径 ≤ 0 / 非有限、`localCenter` 非有限 | **【断】** `CreateCircleShape` assert + 返回 invalid handle（Release 也返回 invalid handle） | `src/Core/World.cpp::CreateCircleShape`；`Shape::IsValid` | `test_world_shapes.cpp::ShapeValidationDeathTest.CircleRejectsInvalidGeometry`；`test_shapes.cpp::CircleShape.IsValidRejectsZeroNegativeAndNonFiniteRadius`、`IsValidRejectsNonFiniteLocalCenter` |
| 平面 normal 非单位/零/非有限、offset 非有限 | **【断】** `CreatePlaneShape` assert + invalid handle | `src/Core/World.cpp::CreatePlaneShape` | `test_world_shapes.cpp::ShapeValidationDeathTest.PlaneRejectsInvalidGeometry`；`test_shapes.cpp::PlaneShape.IsValidRejectsNonUnitZeroAndNonFiniteNormals` |
| `PlaneShape` 挂到 dynamic/kinematic body | **【断】** `AttachShape` assert + 返回 `false`（所有 build 返回 false） | `src/Core/World.cpp::AttachShape` | `test_world_shapes.cpp::ShapeValidationDeathTest.PlaneCannotAttachToDynamicBody` |
| 销毁仍被 body 附着的 shape | **【断】** `DestroyShape` assert + 拒绝（不释放）；先销毁 body 或替换 shape | `src/Core/World.cpp::DestroyShape` | `test_world_shapes.cpp::ShapeValidationDeathTest.DestroyAttachedShapeAsserts` |
| 场景尺度远超 0.1–10 m（AGENTS.md §1.3 支持范围） | **【限】** 不支持/不保证精度；W8 溢出守卫只保证"**拒绝而不崩**"（不会静默产生坏几何）；超长薄条 100 m 可用但物理不承诺"完全平躺" | 见本文 §2.13 | `test_boundary_solver.cpp::BoundaryThinBar.OneHundredMetreBarOnGroundStaysFinite`；`CollidePolygons.ThinStripAndTinyBoxProduceFiniteResults`（`test_sat.cpp`）；`ClipSegmentToLine.DoesNotProduceNaNForExtremeCoordinates`（`test_clipping.cpp`） |

### 1.2 密度 / 质量属性边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| **负 / 非有限**密度（dynamic 与 static 都拒绝） | **【返】** `CreateBody` 返回 **invalid handle**（不 assert，所有 build 一致）；被拒绝的调用不消耗 body 槽位，之后合法 body 仍可创建 | `src/Core/World.cpp::IsValidBodyDef`（匿名命名空间）+ `CreateBody` | `test_boundary_solver.cpp::BoundaryDensity.NonFiniteAndNegativeDensityAreRejectedAtCreateBody` |
| **零密度** dynamic body（尚未附着 shape） | **【返】** 合法：`CreateBody` 接受，`BodyDef.mass/inertia` 生效（fallback；mass=2.5 测试）；无 shape 时无 broad-phase proxy、AABB=point | `src/Core/Body.cpp::Initialize`；`World::CreateBody` | `test_boundary_solver.cpp::BoundaryDensity.ZeroDensityDynamicBodyUsesTheBodyDefMassFallback` |
| 零密度 dynamic + `AttachShape` | **【断】** assert `"requires density > 0"`；Release 返回 `false`（不改变原 shape/质量） | `src/Core/World.cpp::AttachShape` | `test_boundary_solver.cpp::BoundaryDensityDeathTest.ZeroDensityDynamicAttachAsserts` |
| 密度大到质量溢出（如 3.4e38 → m≈1.36e39） | **【断】** `ComputePolygonMass`/`AttachShape` assert `"non-finite mass"`；Release 由 `AttachShape` 的非有限/非物理质量检查**返回 false**（不写入） | `src/Collision/PolygonShape.cpp::ComputePolygonMass`；`World::AttachShape` | `test_boundary_solver.cpp::BoundaryDensityDeathTest.HugeDensityOverflowingMassAsserts` |
| static / kinematic body + 任意密度 + shape | **【返】** 合法：不推导质量；质量/惯量保持 0（无穷大质量的表示，`invMass=0`），不参与密度检查 | `src/Core/Body.cpp::Initialize`；`World::AttachShape` | `test_boundary_solver.cpp::BoundaryDensity.StaticAndKinematicBodiesIgnoreDensityWhenAttaching`；`test_world_shapes.cpp::WorldShapeTest.*` |
| `BodyDef` 的 position/angle/velocity 非有限；mass/inertia/friction/restitution 非有限或为负；dynamic 的 mass/inertia ≤ 0 | **【返】** `CreateBody` 返回 invalid handle（`IsValidBodyDef` 统一校验，不 assert）；被拒绝的调用不消耗槽位，之后合法 def 仍可创建 | `src/Core/World.cpp::IsValidBodyDef` | `test_world_step.cpp::WorldTest.CreateBodyRejectsInvalidDefinitions`（逐项：position/angle/velocity/mass/inertia/density/friction/restitution）；直接构造 `Body` 的 Debug 断言见 `test_debug_asserts.cpp::BodyInitializeDeathTest.NonFinitePositionAsserts` / `NonFiniteVelocityAsserts` |

### 1.3 变换 / 速度 / 外力 / 材质 setter

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| `World` 构造非有限重力 | **【断】** Debug assert `"non-finite gravity"`；**Release 无二次校验**（构造仍完成并保存该值，随后物理产生 NaN，Debug 的 `CheckSane` 会抓；Release 不扫描）——调用方契约 | `src/Core/World.cpp::World` | `test_boundary_solver.cpp::BoundaryWorldDeathTest.ConstructorRejectsNonFiniteGravity` |
| `SetTransform` / `SetLinearVelocity` / `SetAngularVelocity` 非有限输入 | **【断】** Debug assert；**Release 无二次校验**（assert 编译掉后会把值写入；调用方必须保证有限）。stale 句柄是**安全的 no-op**（先做有效性检查，见 `WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal`） | `src/Core/World.cpp::SetTransform`/`SetLinearVelocity`/`SetAngularVelocity` | `test_boundary_solver.cpp::BoundaryWorldDeathTest.SetTransformRejectsNonFinite`、`SetLinearVelocityRejectsNonFinite`；stale no-op：`test_boundary_world.cpp::WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal` |
| `SetFriction` / `SetRestitution` 负值或非有限 | **【断】** Debug assert；**Release 拒绝写入**（`if (!IsValid(handle) || !std::isfinite(x) || x < 0) return;`——比上面三个 setter 多了 Release 守卫） | `src/Core/World.cpp::SetFriction`/`SetRestitution` | `test_boundary_solver.cpp::BoundaryWorldDeathTest.SetFrictionRejectsNegativeOrNonFinite`；`test_world_step.cpp::WorldTest.TransformAndVelocitySettersRoundTrip` |
| 极大速度（> `kMaxTranslation/dt` = 120 m/s） | **【限】+ 行为契约**：单步**位移**被钳制到 `kMaxTranslation`（2 m）/ `kMaxRotation`（π/2 rad），方向保留；**速度本身不变**（下一步继续钳制）；无 NaN/Inf、fat AABB 有效、SAP 内部不变量有效。**没有 CCD，可能隧穿**（见 §2.6） | `src/Core/World.cpp::IntegratePositions` | `test_boundary_solver.cpp::BoundaryVelocity.HugeVelocityIsClampedPerStepWithoutNonFiniteState`、`HugeDownwardVelocityAgainstGroundStaysFinite`；`test_world_step.cpp::WorldTest.LinearDisplacementIsClampedPerStep`、`AngularDisplacementIsClampedPerStep` |
| `ApplyForce` / `ApplyLinearImpulse` 非有限输入 | **【断】无**：当前没有运行时校验（调用方契约；与 setter 的 assert 不同）。非有限值会进入 `force`/速度，Debug 的 `CheckSane` 在下一步抓，Release 不扫描 | `src/Core/World.cpp::ApplyForce`/`ApplyLinearImpulse` | 无专门断言测试（诚实记录：这是**未覆盖的调用方契约**，不是"已验证安全"）；生命周期测试覆盖了 stale handle 的 no-op（`WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal`） |

### 1.4 求解器单元入口（手搭 contact；`ContactSolver` 是公开子步骤）

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| contact 的 body index 越界（`bodyA/bodyB.index` 超出 bodies 向量） | **【断】** `Prepare` 与 `SolvePosition` 都 assert `"bodyA/bodyB index out of range"`（W8 前位置层是 UB） | `src/Solver/ContactSolver.cpp::BuildConstraintsForContact`；`src/Solver/ContactSolverPosition.cpp::buildContact` | `test_boundary_solver.cpp::BoundarySolverDeathTest.SolvePositionRejectsOutOfRangeBodyIndex` |
| `manifold.pointCount` 越界（<0 或 >2） | **【断】** `Prepare` 与 `SolvePosition` 都 assert `"pointCount out of range"`。**诚实说明**：当前 death test 通过"越界 body index"路径触发同一断言块；`pointCount` 分支本身没有单独的 death test（实现有断言，但测试覆盖不完整） | 同上 | 断言代码：`ContactSolver.cpp::Prepare` / `ContactSolverPosition.cpp::buildContact`；无独立测试（同上） |
| manifold 的 normal/penetration 非有限 | **【断】** `Prepare` 与 `SolvePosition` 都 assert（`"non-finite manifold normal/penetration"` / `"non-finite manifold state"`）——W8 新增，避免 NaN 静默污染 | 同上 | `test_boundary_solver.cpp::BoundarySolverDeathTest.PrepareRejectsNonFiniteManifoldNormal` |
| `Prepare` / `SolvePosition` 的 `dt` ≤ 0 或非有限 | **【断】** assert `"dt must be finite and > 0"`（Baumgarte bias 是 `(β/dt)·pen`，dt=0 会除零） | `ContactSolver.cpp::Prepare`；`ContactSolverPosition.cpp::SolvePosition` | `test_boundary_solver.cpp::BoundarySolverDeathTest.PrepareRejectsNonPositiveDt`、`SolvePositionRejectsNonFiniteDt` |
| `SolvePosition` 的 `iterations` < 0 | **【断】** assert `"iterations must be non-negative"`；`iterations == 0` 是合法配置（显式"位置层关闭"实验） | `src/Solver/ContactSolverPosition.cpp::SolvePosition` | 合法零迭代：`test_solver_split_impulse.cpp::SolverSplitImpulse.ZeroPositionIterationsPerformsNoCorrection` |
| static-static（双方无逆质量）contact | **【断】** `Prepare` assert `"non-positive effective normal mass"` 并**跳过该点**（不除零）；Release 跳过不崩（W7 曾收窄休眠跳过谓词以保住此契约） | `ContactSolver.cpp::BuildConstraintsForContact` | `test_contact_solver.cpp::SolverDeathTest.StaticStaticContactAssertsOnNonPositiveMass`；release-only：`test_contact_solver.cpp::ContactSolver.StaticStaticContactIsSkippedInReleaseBuilds` |

### 1.5 World 生命周期 / 句柄边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| `DestroyBody` 重复调用 / stale handle | **【返】** 幂等：stale/未知句柄直接 no-op；不会销毁复用该槽位的新 body（generation 检查，防 ABA） | `src/Core/World.cpp::DestroyBody`/`IsValid` | `test_boundary_world.cpp::WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal`；`test_world_step.cpp::WorldTest.DestroyBodyInvalidatesHandleAndReusesSlot`；`test_idpool.cpp::IDPoolTest.ReleaseBumpsGenerationAndRejectsStaleHandle` |
| stale 句柄的**读**访问器 | **【返】** 全部返回中性默认值（position=(0,0)、velocity=(0,0)、mass=0、type=Static、userData=nullptr、body=nullptr），不触碰已释放槽位（UI 会 poll 一帧） | `src/Core/World.cpp` 的 `Get*` 访问器 | 同上（`DoubleDestroyAndStaleHandlesAreTotal` 逐项断言） |
| stale 句柄的**写**操作（Set/Apply/Wake） | **【返】** 安全 no-op（先做 `IsValid` 检查）；`GetBody(body)` 返回 `nullptr` | 同上 | 同上 |
| 只剩死槽的 World 继续 `Step` | **【返】** 不崩、不产生状态 | `World::Step` 各阶段跳过失效槽位 | `WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal` |
| 休眠场景 `Clear()` | **【返】** 释放全部 body/shape/proxy/接触/岛分区（`GetIslandCount()==0`、休眠计数 0）；旧 handle/shape 全部失效；`Clear` 幂等；World 可复用且槽位从 0/1 重新开始 | `src/Core/World.cpp::Clear` + `ReleaseBroadPhaseProxies` | `test_boundary_world.cpp::WorldLifecycleBoundaryTest.ClearDropsBodiesShapesProxiesAndIslandState` |
| `DestroyBody` 后残留的岛分区（stale partition） | **【返】** 不崩：`WakeIslandOfBody`/`UpdateSleep` 跳过死槽；下一帧重建分区 | `World.cpp::WakeIslandOfBody`/`UpdateSleep`/`BuildIslands` | `test_world_sleep.cpp::WorldSleepTest.DestroyBodyBetweenStepsLeavesAStalePartitionThatMustNotCrash` |
| `AttachShape` 替换已有 shape | **【返】** 成功返回 `true` 并释放旧 shape 槽位、重算质量；非法 shape/body 句柄返回 `false`（不改变原状态） | `World::AttachShape` | `test_world_shapes.cpp::WorldShapeTest.DestroyBodyReleasesAttachedShape` 等 |
| 销毁 body 时的缓存接触 | **【返】** 从 `m_contacts`/`m_previousContacts` 移除引用该 body 的接触（读不到 stale 接触） | `World::DestroyBody` | `test_world_contacts.cpp::WorldContactTest.DestroyBodyDropsCachedContacts` |

### 1.6 `World::Step` 输入边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| `dt` ≤ 0 / NaN / Inf | **【断】** Debug assert `"dt must be finite and > 0"`；**Release 提前 return**（完全不改状态） | `src/Core/World.cpp::Step(dt, velIters, posIters)` | Debug：`test_boundary_solver.cpp::BoundaryWorldDeathTest.StepRejectsInvalidDt`、`test_boundary_world.cpp::WorldStepBoundaryDeathTest.ZeroDeltaTimeAsserts`、`NegativeAndNonFiniteDeltaTimeAsserts`；Release：`WorldStepBoundaryTest.InvalidStepInputIsRefusedInReleaseBuilds`（release-only） |
| 负迭代数（velocity/position） | **【断】** Debug assert `"iteration counts must be non-negative"`；**Release 钳到 0**（定义明确的"零迭代求解"，而不是负预算） | 同上 | Debug：`BoundaryWorldDeathTest.StepRejectsNegativeIterationCounts`、`WorldStepBoundaryDeathTest.NegativeIterationCountsAssert`；Release：`WorldStepBoundaryTest.InvalidStepInputIsRefusedInReleaseBuilds` |
| 每帧调用 `Step` 的传入 dt 与固定步长不一致 | **【限】** 引擎按调用方给的 dt 物理（固定步长是**调用方契约**：testbed 的 `StepAccumulator` 是唯一时间转换点）。给非固定 dt 会破坏 warm start/split impulse 的假设，不保证行为 | `Step` + `Core/TimeStep.h` | `test_accumulator.cpp::StepAccumulatorTest.*`（accumulator 契约）；文档：`docs/architecture.md` §3.3、`dev-interfaces.md` §6 |

### 1.7 休眠与宽相边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| 刚入睡就唤醒（`WakeBody` / 外力 / 冲量 / `SetTransform` / `SetVelocity`） | **行为契约**：必须**整岛**唤醒（不允许半睡半醒）；`sleepTime` 清零、重新计安静窗；5 盒塔实测入睡后立刻 `WakeBody` → 5/5 全醒 | `src/Core/World.cpp::WakeIslandOfBody` + 各入口 | `test_boundary_world.cpp::WorldSleepBoundaryTest.WakeBodyIsWholeIslandAndProxiesSurviveSleep`；`test_world_sleep.cpp` 的 4 个唤醒入口测试 |
| 休眠体保留 broad-phase proxy | **不变量**：休眠岛保留 proxy（邻近唤醒与冻结接触复用都依赖它）；5 盒塔 + 地面 proxy 数为 6/6（唤醒/再入睡后不泄漏） | `World::UpdateBroadPhase`（休眠体跳过 AABB 刷新但保留 proxy） | 同上；`test_world_sleep.cpp::WorldSleepTest.TenBoxTowerSleepsAtomicallyAndStaysFrozen`（10 盒塔 proxy=11） |
| 邻近唤醒（awake body 的 fat AABB 与休眠岛 proxy 重叠） | **行为契约**：整岛唤醒；**宽相路径与无宽相 fallback 必须同序**（放在两者共享的 `UpdateContacts` wake pass，不能放 `UpdateBroadPhase`） | `World::UpdateContacts` 的共享 wake pass | `test_world_sleep.cpp::WorldSleepTest.AwakeBodyFatAabbProximityWakesSleepingIsland`（宽相 on/off 两条路径）；W8 生命周期 churn 逐位一致见 1.8 |
| `allowSleep = false` 的成员 | **行为契约**：整个岛保持清醒（不允许"一个不睡、其他睡"的半冻状态） | `World::UpdateSleep` 的 `allAllowedToSleep` 分类 | `test_world_sleep.cpp::WorldSleepTest.NonSleepableBodyKeepsItsIslandAwake` |
| 运行中把 `enableSleeping` 关掉 | **行为契约**：已睡的岛被**重新唤醒**（是真"休眠开关"，不是永久冻结）；重新打开后能再次入睡 | `World::UpdateSleep` 开关分支 | `test_boundary_world.cpp::WorldSleepBoundaryTest.DisablingSleepingAfterSettleWakesTheIslandAgain`；`test_world_sleep.cpp::WorldSleepTest.SleepingDisabledKeepsEveryBodyAwake` |
| 睡着时 `SetBroadPhase` 切换实现 | **行为契约**：旧 proxy 立即释放、下一步按 body 数重建（含休眠体）；**不唤醒**休眠者；冻结接触列表保持不变；切换后仍可整岛唤醒（5 盒塔实测：proxy 恢复 6、contacts 保持、sleeping=5） | `World::SetBroadPhase` + `ReleaseBroadPhaseProxies` + `UpdateBroadPhase` | `test_boundary_world.cpp::WorldBroadPhaseBoundaryTest.SwitchingBroadPhaseWhileAsleepPreservesSleepAndProxies`；`test_world_broadphase.cpp::WorldBroadPhaseTest.RuntimeBroadPhaseSwitchKeepsPhysicsIdentical` |
| 无宽相 fallback / BruteForce / SAP 等价 | **行为契约**：换宽相/无宽相**不改物理**——候选对规范化为 `(i<j, sorted, deduped)`；W8 进一步验证**生命周期 churn**（中途销毁接触中的 body、创建复用槽位的 body）下三路仍逐帧逐位一致 | `World::UpdateBroadPhase`/`UpdateContacts`/`CreateContactForPair` | `test_boundary_world.cpp::WorldBroadPhaseBoundaryTest.PairSourcesStayBitIdenticalAcrossLifecycleChurn`（200 步 ×3 路）；`WorldBroadPhaseTest.FallbackBruteForceAndSapAreBitIdenticalEveryStep`；`BroadPhaseSap.MatchesBruteForceOnTenRandomScenariosForThousandFrames` |
| SAP 内部不变量（任意时刻） | **不变量契约**：`ValidateInternalState()` 在任意时刻（增删/move 后）必须为 true；极端速度/超长薄条测试逐步验证 | `src/Collision/BroadPhaseSAP.cpp` | `test_boundary_solver.cpp::BoundaryVelocity.*`、`BoundaryThinBar.*`（每步断言）；`test_broadphase_sap.cpp::BroadPhaseSap.EndpointInvariantsHoldUnderArbitraryChurn` |
| fat AABB margin | **不变量**：`kAabbMargin=0.1 m` 只加候选、不加接触（0.15 m 间隙圆对：候选 1 / 接触 0）；休眠体 AABB 刷新被跳过但 proxy 保留 | `World::Synchronize`；`Settings.h` | `test_world_broadphase.cpp::WorldBroadPhaseTest.MarginWidensCandidatesButNotContacts`、`FatAabbIsTightAabbPlusMargin` |
| 休眠/岛记账的 Debug 校验 | **【断】** `CheckSane`/`Validate`（Debug only）扫描 `islandId` 范围、`sleepTime` 有限非负、休眠动态体四路速度精确为 0；健康场景不误报（W8 边界测试在每步或关键状态后调用 `CheckSane`，见 §4.2 各用例） | `src/Core/World.cpp::CheckSane`（phase 11 调用 `Validate`） | `test_boundary_world.cpp::WorldStateInvariantTest.IslandSleepBookkeepingIsConsistentAfterEveryStep`（逐步 `CheckSane` + 手工不变量） |

### 1.8 确定性 / 回归工具边界

| 输入 / 场景 | 契约行为 | 实现位置 | 测试位置 |
| --- | --- | --- | --- |
| 确定性回归 golden | **行为契约**：弱 hash（quantum 1e-5）必须与该 toolchain 的 golden 一致（MSVC `default_600.txt` `588d30e70f9b4ddd`；GCC `default_600_gcc.txt` `001d846118f05125`）；**同一 toolchain 内**纯重构 hash 不变；只有"有意的物理/场景变更或 toolchain 升级"才能更新 golden，且 devlog 写 old/new + toolchain 与原因 | `tests/regression/regression_main.cpp`、`tests/regression/golden/default_600{,_gcc}.txt` | CTest `regression_default` + CI job；`tests/regression/README.md`；`profiling.md` §7.1；命令行见 §6 |
| 回归工具 exit code | **【返】** `0` = 匹配/正常打印/写 golden；`1` = hash mismatch；`2` = 参数错误 / 文件不存在或不可读 / 未知参数 | `tests/regression/regression_main.cpp` | `profiling.md` §7.1 的逐条复核表（7 种调用） |
| 同平台确定性 | **行为契约**：固定 dt、vector 下标顺序、候选对规范化、岛分区规范序 → 同平台同 build 逐位一致（两 World 343 floats、三求解模式 469 floats/步、三宽相 545 floats/帧） | 见 `dev-interfaces.md` §6 | `test_world_stability.cpp::WorldStabilityTest.TwoIdenticalTowersStayBitIdentical` 等 |
| 跨平台 | **【限】** 不保证 bit-exact（FMA 收缩/libm 差异）；用量化弱 hash（1e-5）判定 | `MathUtils::HashQuantizedFloat` | `test_mathutils.cpp::MathUtils.HashQuantizedFloatBuckets`；见 §2.10 |
| `StepAccumulator` 输入 | **【返】** 负/N 帧时间按 0 处理；每帧步数上限 5（多余时间丢弃）；`GetAlpha()` 在 [0,1) | `src/Core/TimeStep.cpp` | `test_accumulator.cpp::StepAccumulatorTest.*`；death：`StepAccumulatorAssertDeathTest.ZeroFixedDeltaAsserts` |

### 1.9 支持范围与"不承诺"清单（非 bug，见 §2）

| 场景 | 现状 | 出处 |
| --- | --- | --- |
| 刚体尺寸/场景尺度 | 设计支持 0.1–10 m（MKS）；超出不保证精度，W8 只保证"溢出几何被拒绝" | `AGENTS.md` §1.3；§1.1 溢出行 |
| 30:1 单列 / 金字塔底部载荷 | 默认 8 速度迭代下会下沉/穿透（载荷/迭代预算属性） | §2.1 |
| 2 点接触残余抖动 / 入睡慢 / 薄片 | 10 盒塔 859 步；<0.2 m 薄片几乎无法入睡 | §2.2 |
| 凹多边形 / Capsule / CCD / 关节 / 3D / 软体 / 流体 / GPU / 多线程 / DAT | 均**未实现**（范围红线或 v1.1 候选） | §2.4、§2.6、§2.8、§2.9 |
| Tracy 剖析 / 渲染插值 / Body 冷热拆分（v1.1 候选） | 均未做（R11 / R17 / R15） | §2.4、§2.5、§2.7 |
| 跨平台 bit-exact | 不承诺（量化 1e-5 容差） | §2.10 |

---

## 2. 已知限制清单（现象/数字 → 为什么不修 → 正解 → 如果要做怎么开始）

> 每条都尽量落到**真实数字 + 测量条件**；数字的完整表格在 [`profiling.md`](profiling.md)，
> 本文只引用与"限制/契约"直接相关的量。**不要把这里的数字当成性能指标引用**。

### 2.1 8 速度迭代的传播预算：30:1 塔会塌、金字塔底部会下沉

- **现象/数字**：默认 8 速度迭代 / 3 位置迭代下，10×1 m 塔 600 步稳定（最大穿透 5.223 mm，
  里程碑）；但 30:1 单列 8 迭代 collapse（W5 记录 maxPen 0.146 m、max|angle|≈π），加到
  **32 次速度迭代**才站立 5 s（穿透 4.12 mm < slop）。金字塔底部承载 16+ 盒时：16 层 136 盒
  600 步 29.7 mm / 1.36°；20 层 210 盒 100.5 mm / 6.02°（载荷导致底部持续下沉）。
- **为什么不修**：这是 PGS 的**传播预算属性**（每帧要沿堆叠层数传递支撑），不是求解器 bug；
  `Pyramid200` 的 "200" 是设计文档的字面目标，已按实测把默认改成 16 层并记录层数-穿透表。
  靠调大迭代数/加阻尼"修"数字会掩盖真实属性，并让 CPU 成本与验收场景脱钩。
- **正解**：明确"验收场景 = 10 盒 @8 迭代"；更高塔/更大载荷需要更多迭代或更强求解器
  （2 点 block solver 改善 2 点接触收敛；多线程/更快的求解器只解决吞吐，不解决传播预算）。
- **如果要做怎么开始**：① 选定目标场景（30 盒/20 层金字塔）；② 扫速度迭代数 8/16/32/64 记录
  穿透-耗时曲线（`--gtest_filter=WorldStabilityTest.*` 或写临时探针）；③ 若要低迭代站住，
  先实现 block solver（§2.2），再复测；④ 把结论写回 `profiling.md` 与 README 的已知限制。

### 2.2 10 盒塔 859 步才入睡 / 薄片几乎无法入睡（2 点接触残余抖动）

- **现象/数字**：10 盒塔在 600 步（10 s）时已稳定（最大穿透 5.223 mm、末帧最大速度
  0.007216 m/s），但按 `kSleepLinearTolerance=0.01 m/s` + 连续 0.5 s 安静窗的规则，实测
  **step 859** 才整岛入睡（W7 debug 历史实测，同机；`profiling.md` §3.4）。W5 已记录静止残余
  抖动尖峰 **max 0.0248 m/s > 0.01**。0.12 m 宽薄牌更糟（maxV≈0.015 m/s）→ 几乎永不入睡，
  所以 Dominoes 用 0.30×1.0 m 骨牌（W8 边界测试顺带实测：单座 5 盒塔 Release 首次全睡 step 84、
  两座 3 盒塔 step 37，见 `profiling.md` §3.3——小场景入睡快，问题集中在 2 点接触的薄/高场景）。
- **为什么不修**：根因是**逐点 PGS 对 2 点接触的顺序偏差**（先解第 1 点、再解第 2 点，两点耦合
  产生周期性残余力矩），不是休眠规则 bug；**没有调大休眠阈值**（保持设计/Box2D 值 0.01），
  也没有加阻尼掩盖——W8 是文档/交付周，改求解器会让回归 hash 变化、需要完整复测窗口。
- **正解**：**2 点接触 2×2 block solver**（把同一 manifold 的两个法向约束作为一个 2×2 块同时
  求解，消除顺序偏差），列为 v1.1 **最高优先级技术债**。
- **如果要做怎么开始**：① 先写一个薄片/薄塔入睡探针，记录 jitter 与 sleep step 基线；
  ② 在 `ContactSolver::SolveVelocity` 里对 `pointCount==2` 的 manifold 装配 2×2 法向块
  （K 矩阵 + 累计钳制 + 投影到可行域），保留 1 点路径不变；③ 断言三模式/宽相等价性仍成立、
  回归 hash 的 old/new 写进 devlog；④ 复测入睡步数、jitter、10 盒塔与 block solver 前后数据。

### 2.3 按岛求解的 body 打包默认关闭（packed 0.87x）

- **现象/数字**：W8 最终源码轮（`profiling.md` §4，400 bodies / 760 contacts 单岛、40 reps、
  8 迭代、8 次运行中位）：flat 1.860 ms / grouped-direct 1.908 ms（0.97x）/
  **grouped-packed 2.137 ms（flat/packed = 0.87x，即 packed 比 flat 慢约 15%）**。
  32 迭代：7.056 / 7.570（0.93x）/ 7.928（0.89x）。W7 历史同场景为 0.82x / 慢 ~18%，
  量级与方向一致（历史值只作对照，**现行口径用 0.87x**）。
- **为什么不修**：400 个 `Body` ≈ 80 KB，整个数组本来就装得进 L2；每帧把热数据拷进连续 scratch
  的 O(bodies) 拷贝成本 > 局部性收益。Debug 下 packed 反而快是"访问放大"效应，**默认值只按
  Release 测量定**（这是本项目"默认值必须 Release 实测"纪律的典型例子）。
- **正解**：`packIslandBodies=false`（opt-in 保留、被三模式位级测试持续覆盖）；岛屿的真实收益
  是**休眠跳过 + 多岛剔除**（不是"打包更快"）。
- **如果要做怎么开始**：① 用 Tracy 或计时探针确认目标场景的瓶颈在 SolveVelocity 且
  working set 超过 L2；② 只有 >cache 规模（数千~万体）才值得重开 packed；③ 改完必须保持
  三模式逐位一致（21,760 floats）且回归 hash 不变（纯重构）。

### 2.4 `Body` 仍是单一结构，未做 `BodySim`/`BodyState` 冷热拆分

- **现象/数字**：设计文档 3.3 原计划 W4 拆分，实际 W4→W8 连续顺延后**决定不做**（R15 列为
  v1.1 候选）。W7 的"岛内打包"实测在 400 体规模反而慢（§2.3）；W8 `profiling.md` 也没有证据
  表明 body 布局是热点（当前所有优化决策都能由现有计时解释，见 `profiling.md` §9）。
- **为什么不修**：该规模下 body 数组本就 cache 友好（≈80 KB），每帧 O(bodies) 拷贝 > 收益；
  且拆分是**纯重构**（213 个热字段访问点），没有 profiling 数据支撑就做，只会增加回归定位成本。
- **正解**：保持单 `Body` + 世界只暴露**基本类型访问器**（`GetPosition`/`GetLinearVelocity`/
  `GetMass`…），`GetBody()/GetBodies()` 明确标注 debug/testbed 用途——将来真拆分不外溢。
- **如果要做怎么开始**：① 先实接 Tracy（§2.5）在 8 个 Step 阶段 + SAP 排序/扫描埋点；
  ② 用时间线与 cache 指标证明 `SolveVelocity`/积分阶段受 body 布局限制；③ 再做
  `BodySim`/`BodyState` 并行数组，验收 = 全部测试 + 回归 hash **不变**（纯重构）。

### 2.5 性能剖析（Tracy / R11）未正式接入

- **现象/数字**：`Core/Profiler.h` 只有零成本宏（`TRACY_ENABLE` 未定义时展开为 `((void)0)`）；
  CMake 选项 `PHYSICS2D_ENABLE_TRACY`/`PHYSICS2D_TRACY_ROOT` 已预留但默认 OFF。v1.0 的所有性能
  数字来自 `physics2d_bench`（steady_clock 逐帧 median/mean + JSON）、testbed Status/Demo 面板与
  gtest 打印（`[solver7]`/`[boundary8]`），**不是 Tracy 时间线**（`profiling.md` §9）。
- **为什么不修**：W8 优先保证文档/边界测试/三 preset 全绿；Tracy 需要外部源码与截图流程，
  开启后有 1–3% 采集开销；当前所有优化决策（packed 0.87x、休眠 1.74x/≈2.0x、三模式）已可由
  现有计时证明，没有"必须看时间线才能定位"的热点（`profiling.md` §9 的 W8 决策）。
- **正解**：v1.1 正式接入（R11）：按设计文档 8.1 只埋 8 个 Step 阶段 `ZoneScopedN` + SAP
  排序/扫描 + `AwakeBodies`/`ContactPairs`/`ManifoldPoints` 曲线；截图与 `profiling.md` 并列。
- **如果要做怎么开始**：① 把 Tracy 源码放到仓库外，用
  `-DPHYSICS2D_ENABLE_TRACY=ON -DPHYSICS2D_TRACY_ROOT=<路径>` 验证宏可用（钩子已就位，纯 CMake 改动）；
  ② 在 8 个阶段与 SAP `EnsureSorted`/`QueryPairs` 加 zone，跑 IslandSleep500/StressBenchmark 出
  时间线；③ 物理/回归 hash 不受影响（埋点零成本或仅采样开销），验收 = 截图 + `profiling.md` 新增一节。

### 2.6 没有 CCD / TOI：高速物体可能隧穿

- **现象/数字**：单步位移被 `kMaxTranslation=2 m`（+ `kMaxRotation=π/2`）钳制，方向保留；
  但**速度本身不截断**，超过 `kMaxTranslation/dt = 120 m/s` 的物体仍可能一帧跨过薄障碍。
  W8 边界测试只承诺"状态有限、不崩、宽相不变量有效"，**不承诺碰撞结果**。
- **为什么不修**：CCD/TOI 在项目宪法 §1.6 的**范围红线**里（校招项目不做完整 CCD）；
  钳制 + fat AABB 是廉价的隧穿防线，覆盖绝大多数常规速度。
- **正解**：真正需要时用 swept shapes / conservative advancement / speculative contacts；
  也可以先做"速度上限 + 加厚薄墙"的内容侧规避。
- **如果要做怎么开始**：① 先做 speculative contact（把 AABB 沿速度外推一个步长，窄相里对
  "下一步会接触"的 pair 用带负分离的 manifold 生成约束）——改动小、不改变现有 API；
  ② 再用 `BoundaryVelocity` 场景做隧穿对照；③ 验收需要新的高速回归场景（当前 golden 不含）。

### 2.7 渲染插值未实现（R17）

- **现象/数字**：`StepAccumulator::GetAlpha()` 已预留（[0,1)），testbed 的暂停/单步/1-2-3 慢放可用；
  但高刷新率下运动仍是"每物理步更新一次"（1/60 s 一跳）。
- **为什么不修**：纯视觉打磨，不影响物理/测试/确定性；W8 产能用于文档与交付验收。
- **正解**：渲染层缓存上一帧 transform，按 alpha 插值（物理仍固定 dt，确定性不受影响）。
- **如果要做怎么开始**：① 在 `Body`/testbed 侧保存上一物理步的 transform（README 渲染插值方案）；
  ② testbed `Renderer` 用 alpha 插值；③ 不影响核心测试，不需要改 hash。

### 2.8 未实现的功能（范围红线或 v1.1 候选）

| 未实现 | 为什么不做 | 正解 / 如果要做 |
| --- | --- | --- |
| 凹多边形（需拆分） | 范围：只支持凸多边形（≤8 顶点）/圆/无限平面 | 内容侧拆成凸集；引擎侧可加 `CompoundShape`（多凸形），但会扩公共 API，需走偏差登记 |
| Capsule | 同上（设计文档可选，未做） | 解析 capsule 路径或退化到两圆+线段，需要新的 ShapeType |
| 关节系统（revolute/prismatic/…） | 范围红线（不做完整关节系统） | 从 1-DOF revolute 关节开始：约束 + 有效质量 + PGS（复用现有 solver 结构），每类关节一个测试场景 |
| 3D / 软体 / 流体 / 布料 / GPU | 范围红线 | 不在一期目标内 |
| 多线程（岛屿并行） | v1.1 起点；单线程 500 体 Release 休眠前后 Step ≈0.4540 → 0.2671 ms（`profiling.md` §3.1），当前已足够 | 按岛并行（岛间不共享 body，更新可交换）；宽相可按 x 条带分块再合并排序；确定性需要固定归约顺序 |
| DynamicAABBTree | 可选第三实现；SAP 已 7.70x（2000 体，Phase C canonical），DAT 的复杂度/调试成本不划算 | 先 profiling 证明 SAP 在目标场景是瓶颈再加；接口 `IBroadPhase` 已冻结，实现可插拔 |

### 2.9 SAP 规模上限（>10k proxy 的升级路径）

- **现象/数字**：`BroadPhaseSAP` 的 active 集合是有序 `std::vector<int32_t>`；本项目里程碑最大
  2000 体（宽相 7.70x），该规模下 vector 比 hash set 更快且顺序确定。>10k proxy 时 active 集合
  的插入/删除开销会显现。
- **为什么不修**：当前规模不受影响；换数据结构会改变遍历/发射顺序，需要重新证明与 BF/fallback
  的逐位一致（工程量大、收益在目标规模外）。
- **正解**：>10k proxy 换固定容量位集 + 位扫描（`BroadPhaseSAP.h` 注释已写升级路径）。
- **如果要做怎么开始**：① 造 10k/50k 体 bench 场景确认拐点；② 实现位集 active 集合并保持
  `ValidateInternalState` 契约；③ 用 10,000 帧配对一致 + 三路逐位一致验证。

### 2.10 确定性范围：同平台逐位、跨平台量化

- **现象/数字**：同平台同 build 逐位一致（两 World 600 步 343 floats、三求解模式 469 floats/步、
  三宽相 545 floats/帧）；跨平台**不保证 bit-exact**（FMA 收缩、libm、x87 差异），用
  **量化弱 hash（quantum 1e-5）** + 按 toolchain 的 golden 判定（MSVC `588d30e70f9b4ddd` / GCC `001d846118f05125`）。
- **为什么不修**：这是浮点/编译器的客观差异，不是引擎能"修"的；强行追求跨平台逐位会限制编译器
  优化（如禁 FMA）并让 CI 变得不可靠。
- **正解**：对外承诺"同平台可复现 + 跨平台量化容差一致"；`exact` 位 hash 只打印作信息。
- **如果要做怎么开始**：若要更强的跨平台一致性，固定编译器/优化选项并禁用 FMA 收缩，
  或把回归判定改为阈值断言（设计文档 R10 的方案）；代价是性能与可移植性。

### 2.11 `ContactSolver::SolvePosition` 每帧分配局部 vector（未证明热点）

- **现象/数字**：位置层每次调用构造局部 `std::vector<PositionConstraintPoint>` 并 reserve（按
  manifold 点数）；目前没有 profiling 证据表明它是热点（W8 的性能数据都在 bench/Step 级别）。
- **为什么不修**：技术债纪律"先测量再优化"；未证明热点时改成成员缓存只会增加状态/回归风险。
- **正解**：先 Tracy/计时确认分配开销，再改成可复用成员缓冲（`Clear()` 保留容量）。
- **如果要做怎么开始**：① 在 `SolvePosition` 前后加计时/分配计数探针；② 若占比 >5%，
  把局部 vector 提升为 solver 成员并 `clear()` 复用；③ 纯重构，三模式/宽相等价与回归 hash 必须不变。

### 2.12 本机增量构建不可信（环境问题，不是引擎问题）

- **现象/数字**：中文 locale MSVC 的 `/showIncludes` 输出"注意: 包含文件:"（GBK），CMake 以
  UTF-8 写入 `rules.ninja` 前缀 → Ninja depfile 为空 → **改 header 不重编** → 类布局混链会
  HEAP CORRUPTION（W7 真实踩到；看起来像越界，其实不是）。
- **为什么不修**：是工具链/本机环境问题，不是仓库代码；对最终用户（普通英文 locale/CI）无影响。
- **正解**：`scripts/build.ps1` 自动检测"header 比 obj 新"并强制 `--clean-first`；
  **canonical 周验收一律删 `build/<preset>` 后 clean build**（CI 无此问题）。
- **如果要做怎么开始**：若在 CI 上遇到同类问题，设置 `VSLANG=1033` / 英文 UI 或改用
  `CMAKE_DEPENDS_USE_COMPILER=OFF`（本机 VS 安装对此均无效，已记录）；回归到 clean build 即可。

### 2.13 场景尺度超出 0.1–10 m 不保证精度

- **现象/数字**：MKS、刚体尺寸设计范围 0.1–10 m；W8 的溢出守卫只保证"**拒绝而不崩**"
  （半宽 1e19 的盒子/FLT_MAX 三角形被 `SetPolygon` 拒绝，不会产生 (0,0) 法线或 NaN 质心）；
  1×100 m 超长薄条可以跑（AABB 100 m、SAP 代理、接触杠杆臂正常，无 NaN），但**不承诺平躺**
  （长杠杆 + 2 点 PGS 有残余转动）。
- **为什么不修**：3D 场景/超大尺度不在校招项目范围；给数值精度做通用保证需要 double 或
  区间分析，成本和收益不匹配。
- **正解**：内容侧把场景保持在 0.1–10 m；需要大尺度时检查溢出守卫的拒绝路径，并接受精度降级。
- **如果要做怎么开始**：① 明确支持的尺度范围（已有）；② 若要扩展，先测 float 精度误差
  （位置/惯性矩/接触法线）再决定是否局部用 double。

---

## 3. 数字口径（引用前必读）

> W8 交付期统一了若干历史口径。**本文按最新口径**；如果 README/其它文档的摘要与本文不一致，
> 以本文与 [`profiling.md`](profiling.md) 为准（Phase C 由 Lead 做最终同步）。

| 主题 | 现行口径（本文使用） | 历史/其它写法（不要直接引用） |
| --- | --- | --- |
| 测试规模 / 三 preset 结果 | 以 [`testing.md`](testing.md) 为准；本批新增 **23 + 12** 个边界用例（§4） | 旧文档的 354/355/353 是 W7 数字 |
| 休眠加速 | **对外端到端 1.74x**（canonical Release；testbed 的 awake 窗口是"还没全睡"的混合窗口，含已入睡的岛；W8 测量会话值 1.70x）；**纯全醒/全睡 ≈ 2.0x**（enableSleeping=false 对照：0.5557 ms / 0.2772 ms，canonical） | 不写"100 岛全醒 1.73x"；1.73x 是 W7 历史同机值（量级一致，仅作历史） |
| packed 模式 | **0.87x（比 flat 慢约 15%）**（W8 最终源码轮：flat 1.860 / packed 2.137 ms，8 迭代） | W7 历史 0.82x/~18% 仅作量级对照 |
| split impulse 深穿透 | **1.139999 m/s 对应 0.10 m 深穿透**场景（手算 `(0.2/dt)·(0.10−0.005)=1.14`）；**0.02 m 浅重叠**对应 Baumgarte 0.18 m/s（另一个单测场景） | 不要把 1.139999 标成"2 cm 重叠" |
| 10 盒塔入睡 | **step 859**（W7 debug 历史实测，同机）；W8 Release 小场景：5 盒塔首次全睡 **step 84**、两座 3 盒塔 **step 37**（`profiling.md` §3.3） | 不要写"10 盒塔 600 步入睡" |
| 回归 hash | 同一 toolchain 内弱 hash 未变（MSVC `588d30e70f9b4ddd`；GCC `001d846118f05125`）；exact `f4390ef86c075a04` 仅信息 | — |
| W8 是否改物理 | **没有**：Step 顺序/物理常数/公共 API 未变；4 处改动只加拒绝/校验/assert（§5） | — |
| 性能数字 | 唯一来源 [`profiling.md`](profiling.md)（含测量环境与噪声纪律）；本文只引用与限制相关的数字 | 不要把 Debug/ASan/`avg_frame_ms`（含 GPU）当性能指标 |

---

## 4. 本批新增测试索引（与 §1 逐条对应）

### 4.1 `tests/unit/test_boundary_solver.cpp`（23 个：10 常规 + 13 death）

| # | 测试（`套件.用例`） | 钉住的契约 | §1 对应 |
| --- | --- | --- | --- |
| 1 | `BoundaryPolygon.DegeneratePolygonsAreRejectedWithoutCrash` | 面积≈0/近零/顺时针/凹/重复/共线/NaN/nullptr/顶点数<3 → `SetPolygon` false 且形状 invalid，不崩 | 1.1 行 1–3 |
| 2 | `BoundaryPolygon.FiniteButOverflowingCoordinatesAreRejected` | 有限但溢出（1e19 盒、FLT_MAX 三角形、1e19 菱形/正方形）→ 拒绝；普通 0.5 盒仍接受 | 1.1 行 4 |
| 3 | `BoundaryDensity.NonFiniteAndNegativeDensityAreRejectedAtCreateBody` | 负/NaN/±Inf 密度（dynamic+static）→ `CreateBody` invalid handle、不占槽位、之后合法 body 仍可建 | 1.2 行 1 |
| 4 | `BoundaryDensity.ZeroDensityDynamicBodyUsesTheBodyDefMassFallback` | density=0 dynamic 合法 → 用 BodyDef.mass/inertia；无 shape → `proxyId=-1`、AABB=point | 1.2 行 2 |
| 5 | `BoundaryDensity.StaticAndKinematicBodiesIgnoreDensityWhenAttaching` | static/kinematic + density=0 + shape 合法；mass/invMass=0（无穷大表示） | 1.2 行 5 |
| 6 | `BoundaryVelocity.HugeVelocityIsClampedPerStepWithoutNonFiniteState` | 1e6 m/s：每步位移恰为 `kMaxTranslation`、方向保留、速度不变、AABB 包含位置、SAP 不变量有效 | 1.3 行 4 |
| 7 | `BoundaryVelocity.HugeDownwardVelocityAgainstGroundStaysFinite` | 5e7 m/s 向下撞地面：只承诺 30 步状态有限 + SAP 有效（**不承诺不隧穿**） | 1.3 行 4、§2.6 |
| 8 | `BoundaryThinBar.OneHundredMetreBarOnGroundStaysFinite` | 1×100 m 条：AABB≈100 m、200 步无 NaN、SAP 有效、仍与地面接触；不承诺完全平躺 | 1.1 行 9、1.9 |
| 9 | `BoundaryMassRatio.SolverEffectiveMassStaysFiniteAtOneToMillion` | 1:1e6：`K_n=1.000001`、`normalMass≈0.999999`、单步动量守恒（误差 ≤1e-4）、无除零/NaN | 1.4（质量比） |
| 10 | `BoundaryMassRatio.WorldStackWithOneToMillionMassRatioStaysFinite` | World 级 1:1e6 堆叠 120 步：状态有限、轻盒没有被弹飞/穿透 | 1.4 |
| 11 | `BoundaryDensityDeathTest.ZeroDensityDynamicAttachAsserts` | 零密度 dynamic + AttachShape → assert `requires density > 0` | 1.2 行 3 |
| 12 | `BoundaryDensityDeathTest.HugeDensityOverflowingMassAsserts` | 3.4e38 密度 → 质量溢出 → assert `non-finite mass` | 1.2 行 4 |
| 13 | `BoundaryDensityDeathTest.WorldCreateShapeRejectsOverflowingGeometry` | World API 对溢出几何 assert（`CreateBoxShape`/`CreatePolygonShape`）且不占 shape 槽位 | 1.1 行 4 |
| 14 | `BoundarySolverDeathTest.PrepareRejectsNonFiniteManifoldNormal` | 手搭 contact 的 NaN normal → `Prepare` assert | 1.4 行 3 |
| 15 | `BoundarySolverDeathTest.PrepareRejectsNonPositiveDt` | `Prepare(dt=0)` → assert `dt must be finite and > 0` | 1.4 行 4 |
| 16 | `BoundarySolverDeathTest.SolvePositionRejectsNonFiniteDt` | `SolvePosition(dt=NaN)` → assert | 1.4 行 4 |
| 17 | `BoundarySolverDeathTest.SolvePositionRejectsOutOfRangeBodyIndex` | 手搭 contact 指向不存在的 body slot → 位置层 assert `bodyB index out of range`（W8 前是 UB） | 1.4 行 1 |
| 18 | `BoundaryWorldDeathTest.StepRejectsInvalidDt` | `Step(NaN/0/-dt)` → assert | 1.6 行 1 |
| 19 | `BoundaryWorldDeathTest.StepRejectsNegativeIterationCounts` | 负速度/位置迭代数 → assert | 1.6 行 2 |
| 20 | `BoundaryWorldDeathTest.ConstructorRejectsNonFiniteGravity` | 非有限重力构造 → assert | 1.3 行 1 |
| 21 | `BoundaryWorldDeathTest.SetLinearVelocityRejectsNonFinite` | 非有限速度 setter → assert | 1.3 行 2 |
| 22 | `BoundaryWorldDeathTest.SetTransformRejectsNonFinite` | 非有限 transform setter → assert | 1.3 行 2 |
| 23 | `BoundaryWorldDeathTest.SetFrictionRejectsNegativeOrNonFinite` | 负/Inf 摩擦 → assert | 1.3 行 3 |

> 注：第 11–23 是 death tests（13 个），在沙箱内 SKIP、工作区外/CI 真跑（§0.2）。

### 4.2 `tests/unit/test_boundary_world.cpp`（12 个：debug 10 + release-only 2，含 3 death）

| # | 测试（`套件.用例`） | 钉住的契约 | §1 对应 |
| --- | --- | --- | --- |
| 1 | `WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal` | DestroyBody 幂等；stale 读/写/唤醒全部安全失效（中性默认值/no-op）；槽位复用 generation+1，旧句柄不 alias 新 body；只剩死槽也能 Step | 1.5 行 1–3 |
| 2 | `WorldLifecycleBoundaryTest.ClearDropsBodiesShapesProxiesAndIslandState` | 休眠场景 Clear：body/shape/proxy/contact/island 全清、旧句柄失效、Clear 幂等、World 可复用（槽位从 0/1） | 1.5 行 4 |
| 3 | `WorldStepBoundaryDeathTest.ZeroDeltaTimeAsserts` | `Step(0)` assert | 1.6 行 1 |
| 4 | `WorldStepBoundaryDeathTest.NegativeAndNonFiniteDeltaTimeAsserts` | `Step(-dt/NaN/Inf)` assert | 1.6 行 1 |
| 5 | `WorldStepBoundaryDeathTest.NegativeIterationCountsAssert` | 负迭代数 assert | 1.6 行 2 |
| 6 | `WorldStepBoundaryTest.InvalidStepInputIsRefusedInReleaseBuilds`（**release-only**） | NDEBUG：非法 dt 安全返回且状态不变；负迭代数钳 0（定义明确）；`CheckSane` 无 NaN/Inf | 1.6 行 1–2 |
| 7 | `WorldSleepBoundaryTest.WakeBodyIsWholeIslandAndProxiesSurviveSleep` | 5 盒塔全睡后：proxy 保留 6/6；休眠体四路速度=0、`islandId`/`sleepTime` 合法；`WakeBody` 整岛唤醒（5/5）；再入睡无 proxy 泄漏。（Release 实测首次全睡 step 84，见 `profiling.md` §3.3；测试本身不硬编码该步数） | 1.7 行 1–2 |
| 8 | `WorldSleepBoundaryTest.DisablingSleepingAfterSettleWakesTheIslandAgain` | 运行中关 `enableSleeping` → 已睡岛被唤醒且不再入睡；重新打开能再睡 | 1.7 行 5 |
| 9 | `WorldBroadPhaseBoundaryTest.SwitchingBroadPhaseWhileAsleepPreservesSleepAndProxies` | 睡着时切宽相：旧 proxy 立即释放、下一步重建 6 个（含休眠体）、不唤醒、接触列表不变、仍可唤醒 | 1.7 行 6 |
| 10 | `WorldBroadPhaseBoundaryTest.PairSourcesStayBitIdenticalAcrossLifecycleChurn` | 200 步 × {无宽相/BF/SAP}：中途销毁接触中的 body、新建复用槽位，逐帧 body+contact 浮点逐位一致 | 1.7 行 7 |
| 11 | `WorldStateInvariantTest.IslandSleepBookkeepingIsConsistentAfterEveryStep` | 每步后：`islandId` 在 `[0,islandCount)`、`sleepTime` 有限非负、休眠动态体四路速度=0、伪速度清零；两岛独立唤醒（唤一座另一座仍睡） | 1.7 行 8 |
| 12 | `WorldBoundaryProfilingTest.SleepingOnVsOffStepTimingIsPrinted`（**release-only**） | 休眠 on/off 直接计时（`[boundary8] islandsleep500` 行），供 `profiling.md` §3.2 复现；不在 Debug/ASan 引用数字 | §3 口径 |

### 4.3 既有测试的交叉引用（不重复实现）

| 契约 | 既有测试（文件::测试） |
| --- | --- |
| 多边形校验 / box 非法 extents | `test_polygon_shape.cpp::PolygonShape.ValidationRejectsDegenerateInput` 等；`test_world_sat.cpp::PolygonValidationDeathTest.*` |
| 形状校验 / 平面挂 dynamic / 销毁附着 shape | `test_world_shapes.cpp::ShapeValidationDeathTest.*` |
| Body 直接初始化非有限输入 | `test_debug_asserts.cpp::BodyInitializeDeathTest.*`；`test_world_step.cpp::WorldAssertDeathTest.BodyInitializeRejectsNonFiniteInput` |
| 单步位移/转角钳制 | `test_world_step.cpp::WorldTest.LinearDisplacementIsClampedPerStep`、`AngularDisplacementIsClampedPerStep` |
| stale 句柄 / 槽位复用 | `test_world_step.cpp::WorldTest.DestroyBodyInvalidatesHandleAndReusesSlot`、`WorldTest.CreateBodyRejectsInvalidDefinitions`；`test_idpool.cpp::IDPoolTest.*` |
| static-static 接触、release-only 跳过 | `test_contact_solver.cpp::SolverDeathTest.StaticStaticContactAssertsOnNonPositiveMass`、`ContactSolver.StaticStaticContactIsSkippedInReleaseBuilds` |
| 宽相等价 / 运行时切换 / proxy 生命周期 / fat AABB | `test_world_broadphase.cpp::WorldBroadPhaseTest.*`；`test_broadphase_sap.cpp::BroadPhaseSap.*`；`test_broadphase_bruteforce.cpp::BroadPhaseBruteForce.*` |
| 休眠/唤醒（W7） | `test_world_sleep.cpp::WorldSleepTest.*`（10 盒塔 859 步、4 唤醒入口、邻近唤醒、`allowSleep=false`、stale partition） |
| 三求解模式逐位一致 | `test_solver_island.cpp::SolverIslandTest.*`（21,760 floats）；`test_world_islands.cpp::WorldIslandsTest.IslandSolvingToggleIsBitIdenticalPhysics` |
| 确定性 | `test_world_stability.cpp::WorldStabilityTest.TwoIdenticalTowersStayBitIdentical`；`test_world_broadphase.cpp::FallbackBruteForceAndSapAreBitIdenticalEveryStep` |
| 回归 exit code | `tests/regression/regression_main.cpp` + `profiling.md` §7.1（7 种调用表） |
| 极大速度/薄条有限性 | `test_world_step.cpp`（钳制）；`test_sat.cpp::CollidePolygons.ThinStripAndTinyBoxProduceFiniteResults`；`test_clipping.cpp::ClipSegmentToLine.DoesNotProduceNaNForExtremeCoordinates` |

---

## 5. W8 代码改动清单（只加契约/校验，不改物理）

> 验证：`docs/profiling.md` §8.2、`dev-interfaces.md` §5 的 W8 行；回归弱 hash 未变
> （同一 toolchain 内 MSVC `588d30e70f9b4ddd`）；Step 顺序/物理常数/公共 API 未变。

| 文件 | 改动 | 影响面 | 测试 |
| --- | --- | --- | --- |
| `src/Collision/PolygonShape.cpp` | **3 处溢出拒绝守卫**：① 边长方和（`LengthSquared` → inf）；② 鞋带和（`twiceArea` → inf）；③ 质心加权和（`centroid` 非有限）→ `SetPolygon` 返回 false | 仅非法/极端输入；合法多边形行为不变 | `BoundaryPolygon.FiniteButOverflowingCoordinatesAreRejected`；`BoundaryDensityDeathTest.WorldCreateShapeRejectsOverflowingGeometry` |
| `src/Solver/ContactSolver.cpp` | 速度层 `Prepare` 增加**非有限 manifold normal/penetration assert**（此前会静默 NaN 传播） | 仅非法手搭 contact；World 合法路径不会触发 | `BoundarySolverDeathTest.PrepareRejectsNonFiniteManifoldNormal` |
| `src/Solver/ContactSolverPosition.cpp` | 位置层增加 **body index / pointCount / 非有限 manifold assert**（此前是 UB / 静默 NaN） | 仅非法手搭 contact；合法路径不变 | `BoundarySolverDeathTest.SolvePositionRejectsOutOfRangeBodyIndex`、`SolvePositionRejectsNonFiniteDt` |
| `src/Core/World.cpp` | ① `Step` 的 **Release 兜底**：负迭代数钳到 0（Debug 仍 assert）；② `CheckSane`/`Validate` 覆盖 W7 新状态（`islandId` 范围、`sleepTime` 有限非负、休眠动态体四路速度精确 0） | ① 只影响非法输入在 Release 的行为（定义明确）；② 只影响 Debug 校验；**合法路径逐位不变** | ① `WorldStepBoundaryTest.InvalidStepInputIsRefusedInReleaseBuilds`（release-only）；② `WorldStateInvariantTest.IslandSleepBookkeepingIsConsistentAfterEveryStep` |

> 注意：**没有**为了边界测试修改任何合法输入路径的物理、Step 顺序、`Settings.h` 常数或公共 API。
> W8 的边界测试全部是"非法输入/极端输入"或"不变量检查"，不进入回归 golden 场景。

---

## 6. 复现命令（仓库根目录，Windows PowerShell 5.1）

```powershell
# 0) 全量构建 + 测试（测试规模/三 preset 结果以 docs/testing.md 为准）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 1) 只跑 W8 solver/几何边界测试（10 个常规 + 13 个 death，共 23）
.\build\debug\tests\physics2d_tests.exe --gtest_filter="Boundary*"

# 2) 只跑 W8 World/宽相/休眠边界测试（debug 10 个；release-only 2 个需 Release 构建才可见）
.\build\debug\tests\physics2d_tests.exe --gtest_filter="WorldLifecycleBoundaryTest.*:WorldStepBoundary*:WorldSleepBoundaryTest.*:WorldBroadPhaseBoundaryTest.*:WorldStateInvariantTest.*:WorldBoundaryProfilingTest.*"

# 3) death test 真正执行（工作区外；期望全部 gtest 通过）
$dir = Join-Path $env:LOCALAPPDATA 'Physics2D\testrun'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
Copy-Item "build\debug\tests\physics2d_tests.exe" $dir -Force
Push-Location $dir; .\physics2d_tests.exe; Pop-Location

# 4) 确定性回归（弱 hash 必须不变）
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# 5) 性能/限制数字的复现命令见 docs/profiling.md §1.4（含休眠 on/off 的 Release-only 计时）
```

> 本机环境坑（中文 MSVC `/showIncludes` → Ninja depfile 为空）见
> [`dev-setup.md`](dev-setup.md) §6：改 header 后不要相信增量构建，`build.ps1` 会自动 clean-first，
> canonical 验收一律 clean build。

---

## 7. 与其它文档的分工 / 交叉引用

| 想看什么 | 去哪 |
| --- | --- |
| 契约的**数学/几何推导**（为什么退化多边形会被拒、K_n 为什么趋近 0） | [`math.md`](math.md) §2/§10（§10 是与本文互为索引的输入→行为表） |
| **性能数字**（宽相/休眠/三求解模式/构建耗时）与测量纪律 | [`profiling.md`](profiling.md)（唯一来源；§3.2 休眠口径、§4 三模式、§8 W8 新增、§9 Tracy、§10 噪声） |
| **怎么跑测试**（全量/单测过滤/smoke/bench/回归/death tests/FAQ） | [`testing.md`](testing.md)（测试总数以它为准） |
| 对外**摘要**（已知限制 10 条左右） | [`README.md`](../README.md) §已知限制 |
| **偏差登记**（与设计文档的差异）与不可破坏的**不变量** | [`dev-interfaces.md`](dev-interfaces.md) §5/§6 |
| 每周"卡住的地方"与历史实测数字 | [`devlog.md`](devlog.md) |
| 接口预留状态 / Roadmap | [`ROADMAP.md`](ROADMAP.md)、[`dev-interfaces.md`](dev-interfaces.md) §2/§4 |

> **维护规则**：新增边界行为 = 先在 `tests/unit/test_boundary_*.cpp`（或对应模块测试）里钉住，
> 再更新本文的对应行；新增已知限制 = 在 §2 写"现象/数字 → 为什么不修 → 正解 → 怎么开始"。
> 本文不自行产生性能数字；性能结论引用 `profiling.md`，行为历史引用 `devlog.md`。
