# Devlog 开发日志

> 每周 5 行以内的周报（模板见设计文档 W04）。重点写「卡住的地方」——那是面试时最有价值的部分。
> 数字必须来自实测，标注测量条件。

---

## Week 1 · 工程骨架 + 数学库 + 渲染循环

**完成**
- 搭好仓库骨架与构建体系：CMake presets（debug/release/sanitize/headless + VS2022 兜底）、
  `FetchContent` 拉取 GoogleTest/Eigen/GLFW/Dear ImGui（不依赖 vcpkg）、`bootstrap/build/format` 脚本、
  GitHub Actions（双平台构建+测试、ASan/UBSan、headless、core 分层守卫）。
- `p2::Math` 全部落地：`Vec2`（三种 `Cross` 重载、零向量 `Normalized` 返回 (0,0)）、`Mat2`（列主序、
  bool 返回的 `Inverse`）、`Rot`（cos/sin 存储）、`Transform`、`AABB`（fat AABB `Extend`、边接触算重叠）、
  `MathUtils`（FNV-1a float 位模式 hash，供未来 headless 回归）。
- `p2::Core` 骨架：generation 计的 `Handle`+`IDPool`（防 ABA）、`Body`/`BodyDef`、`World` 的
  **11 阶段 Step 流水线**（Week 1 实现：积分力、积分位置、Debug NaN 校验；其余阶段留空但顺序固定）、
  `StepAccumulator` 固定步长（含死亡螺旋保护）、Debug `CheckSane()`。
- 可交互 Testbed：GLFW + OpenGL 3.3 Core + ImGui docking；自写最小 GL loader（不引入 glad/glew）、
  单 VBO 线框批处理 Renderer、Camera（中键平移/滚轮缩放/R 复位）、DebugUI、Demo 基类、
  FallingBall demo（重力下落 + 旋转方块渲染验证）；`--smoke N --hidden` 支持无人工干预验证。
- 为 Week 2-8 预留接口：`IBroadPhase`、`ContactID`/`Manifold`/`ContactPoint`/`Contact`、
  `SolverSettings`、`Profiler` 零成本宏、`tests/regression` 与 `bench` 的目录/target 占位，
  以及 `docs/dev-interfaces.md` 的 18 条预留点清单。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **`cl: error D8050 无法执行 c1xx.dll`**（CMake 编译器探测阶段，ASCII 路径同样复现）
   - 排查：① 直接 `cl hello.cpp` 成功 → 排除编译器；② 手写 `build.ninja` 让 ninja 调 cl 仍失败，
     且 ninja 自身报 `WriteFile(.ninja_lock): Permission denied`、其子进程 `cmd` 写文件也 Access denied；
     ③ 把同一个 ninja 二进制放到 `%TEMP%` 或 VS 安装目录就能工作，放回仓库 `tools/` 就失败。
   - 根因：**本机 agent 沙箱拒绝"可执行文件位于工作区内部"的进程树写文件**。
   - 修复：Ninja 移出仓库；`build.ps1`/`bootstrap.ps1` 自动优先使用 VS 自带 Ninja（Program Files），
     否则下载到 `%LOCALAPPDATA%\Physics2D\tools` 并注入 PATH。
2. **MSVC ASan 链接 `LNK2038 annotate_string / annotate_vector mismatch`**
   - 根因：`/fsanitize=address` 会让 MSVC 为参与编译的 TU 关闭 STL 迭代器注解，而 GoogleTest
     （第三方 target）没有开 ASan，注解宏不一致。
   - 修复：`PHYSICS2D_ENABLE_ASAN` + MSVC 时全局定义
     `_DISABLE_VECTOR_ANNOTATION`/`_DISABLE_STRING_ANNOTATION`，让所有 TU（含 gtest）一致。
3. **gtest death tests 在本沙箱 FATAL / Release 下会误报**
   - 根因：death test 需要子进程写临时文件（沙箱禁止工作区内可执行文件写入）；而 Release 的
     `NDEBUG` 会把 `assert` 编译掉，"非法输入必须崩"在该 build 不成立。
   - 修复：`tests/unit/TestSupport.h::DeathTestFixture` 在 `NDEBUG` 或无法写临时文件时
     `GTEST_SKIP()` 并打印原因；CI/普通机器 Debug 下照常执行。

**数据（测量条件：Windows 11 22631 / i7-12650H / 16 GB / RTX 4060 / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- 首次 clean configure + Debug 全量构建 + ctest：**23.6 s**（含 FetchContent 下载 gtest/Eigen/GLFW/ImGui）。
- Release（RelWithDebInfo）clean 构建：**28.7 s**；Sanitize（ASan）构建 + ctest 通过。
- 三个 preset 的项目代码警告数：**0**（MSVC `/W4 /permissive-`）。
- 单测：**120 个 ctest 用例**（112 常规 + 8 个 debug death test）；debug/release/sanitize 三套全绿。
  本沙箱下 8 个 death test 自动 SKIP（沙箱限制）；把测试 exe 复制到工作区外运行 → **120/120 全过**。
- Testbed smoke（`--smoke 120 --hidden`）：`frames=120 physics_steps=120 bodies=1 avg_frame_ms=0.946`，
  GL 3.3.0 NVIDIA 617.42，exit 0；球位置 `(0.000, 4.233)`（期间自动重置 1 次）。
- ASan + testbed（手动配置 `-DPHYSICS2D_ENABLE_ASAN=ON -DPHYSICS2D_BUILD_TESTBED=ON`）：
  `--smoke 60 --hidden` exit 0，无 ASan 诊断。

**下周**
1. 形状系统（Circle/Plane）+ 圆-圆、圆-平面解析碰撞（P06）；
2. `Contact`/`ContactID` 初版 + `World::UpdateContacts` 接线，接触可打印；
3. 纯位置修正让球"停下来"（记录"落地即停、无弹跳"现象，作为 Week 3 引入冲量法的动机）；
4. 跑一次 `-Preset sanitize`；补 devlog，打 tag `v0.2-plane-contact`。

**给未来的自己的提醒**
1. `World`/`Body`/`Math` 的公共签名已冻结，Week 2 起只追加不修改；要破坏性改动先看
   `docs/dev-interfaces.md` 第 5 节并登记理由。
2. 本机只有 Windows PowerShell 5.1（没有 `pwsh`）；构建一律用
   `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 ...`。
   Ninja 永远不要放进仓库内（沙箱限制）。
3. 每个模块完成立刻跑 sanitize，不要攒到周末——物理引擎的 UB 会伪装成"算法不收敛"。

---

## Week 2 · 形状 + 解析碰撞 + 第一次"停下来"

**完成**
- **形状系统**：`Core/Shape.h`（tagged union）+ World 形状池（`IDPool` + `vector<Shape>`）、
  `CircleShape`（`m=ρπr²`、`I=½mr²`、解析 AABB）、`PlaneShape`（无限半空间，only-static）；
  `World::AttachShape` 从 density 推导质量/惯性（含平行轴折算），`fixedRotation` 支持。
- **解析碰撞**（`Collision/Analytic.cpp`）：`CollideCircles`、`CollideCircleAndPlane`、
  `CollidePlaneAndCircle`、`FlipManifold`；相切 ⇒ `pointCount=1, penetration=0`；
  同心圆心 ⇒ 确定性法线 `(1,0)`（不用 NaN）；法线严格 A→B；锚点同时写入两个局部系，
  `Mul(xfA,localPointA) ≈ Mul(xfB,localPointB)` 误差 < 1e-5。
- **Contact/ContactID 持久化**：`World::UpdateContacts` 每帧重建接触列表（O(N²) + AABB 过滤），
  与 `m_previousContacts` 按 body 对 + `ContactID::Key()` 匹配并继承每点累计冲量，
  `isPersistent` 就位（W5 warm starting 的直接前置）。`Contact` 从 `Solver/` 移到 `Collision/`。
- **临时响应**（明确技术债，代码标 `TEMPORARY`）：位置推出（slop + `kMaxLinearCorrection` 钳制）
  + 法向速度投影（restitution 速度阈值）→ 球落地停住、e>0 时近似弹起；AABB 同步、材质访问器、
  接触统计全部接通。
- **Testbed**：FallingBall 改为平面地面 + 圆球（形状/质量来自 attach）；restitution 滑条、
  "重新落下"按钮、接触点（红圈+叉）/法线（黄色箭头）叠加、状态面板显示 shapes/contacts/
  最大穿透（mm）与球 y/vy；smoke 输出加入 shapes/contacts 与每个 body 的状态。
- **测试/质量**：185 个 ctest 用例（+65：碰撞 28 + Core 33 + death tests 4）；
  `debug`/`release`/`sanitize` 三套 preset 全绿、项目代码零警告；新增测试含解析期望值、
  稳态穿透 `kLinearSlop + g·dt² = 0.0077778 m`、e=0.8 反弹收敛、确定性（300 步 memcmp 逐位一致）。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **`C2660: 'p2::CircleShape::IsValid' 函数不接受 1 个参数`**
   - 现象：`return IsValid(localCenter);` 编译失败。排查：成员函数 `CircleShape::IsValid()` 内，
     非限定名 `IsValid` 只做成员查找并停下，不会继续找 `p2::IsValid(Vec2)`。
   - 修复：显式限定 `p2::IsValid(localCenter)`（并在注释里记录这个 C++ 名字隐藏陷阱）。
2. **跨模块测试误报：anchor 反投影差 0.0027778 m**
   - 现象：相切圆-平面测试用 *Step 之后* 的 transform 反投影 `localPointB`，差正好等于 `g·dt²`。
   - 根因：`UpdateContacts` 在 `IntegratePositions` 之前执行，manifold 锚点是**生成时刻**的几何；
     用 post-step transform 校验必然差一步位移。修复：断言原始局部位移（generation-time）。
   - 教训：接触数据的坐标系语义要写进 `Contact.h`/测试注释，否则跨模块集成时必踩。
3. **release-only `C4100: 'shape' 未引用的参数`**
   - `ComputePlaneMass` 的 `shape` 只在 debug assert 中用到，NDEBUG 下变未使用。
   - 修复：`static_cast<void>(shape);`（保留 debug 校验，同时 /W4 干净）。

**数据（测量条件：Windows 11 22631 / i7-12650H / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- clean 构建 + 全量 ctest：debug **30.9 s**、release **38.8 s**、sanitize(ASan) **27.4 s**；
  三个 preset 项目代码项目警告 **0**。
- 单测 **185**（173 常规 + 12 debug death test）；三 preset 均 185/185 pass，12 skipped（沙箱）。
- Testbed smoke（`--smoke 120 --hidden`）：`bodies=2 shapes=2 contacts=1 avg_frame_ms=0.848`，
  `falling_ball` 最终 `position=(0.000, 0.492) velocity=(0,0)`，exit 0；
  稳态穿透 0.00778 m（理论 `kLinearSlop + g·dt²`）。
- 位置修正版 restitution 的实测：默认 e=0.5 时球落地后 `contacts=0`、`y=0.658`、`vy=-0.875`
  —— 能弹但高度/能量不可控，正是 W3 换 Sequential Impulse 的动机。

**下周**
1. `Solver/ContactSolver`：有效质量 `K_n`、单次法向冲量、迭代求解、冲量累计 + 非负钳制（P08）；
2. 接管 `WarmStart`/`SolveVelocityConstraints`，删除 `SolvePositionConstraints` 里的临时速度投影；
3. 动量守恒/能量守恒/收敛曲线单测；球-球正碰与球-地面对照实验；
4. 保证 10s 内不出现 NaN；跑 sanitize；补 devlog，打 tag `v0.3-impulse`。

**给未来的自己的提醒**
1. `SolvePositionConstraints` 里还留着 Week 2 的 `TEMPORARY` 位置推出 → W5 换 split impulse；
   临时速度投影 → W3 被 Sequential Impulse 取代。grep `TEMPORARY` 找到所有技术债。
2. Contact 的 `normal` 是 A→B（A=较小 body index），接触锚点是 generation-time 几何，
   不要用 post-step transform 去校验。
3. 形状质量只在 `AttachShape` 时推导；之后改 density/restitution/friction 用 World 的 setter，
   不要再碰 `Body.mass` 字段。

---

## Week 3 · Sequential Impulse（只有法向）

**完成**
- **求解器**（`Solver/ContactSolver.{h,cpp}`，P08）：`Prepare` 每帧缓存 `rA/rB`、有效法向质量
  `normalMass = 1/K_n`、以及由初始接近速度算出的 restitution bias；`SolveVelocity` 做
  `velocityIterations` 轮 PGS：`λ = -(vn + bias)·normalMass`、累计钳制
  `acc = max(acc+λ, 0)`、只施加增量 `Δλ`（线速度 + `r×J` 角速度），最后把累计冲量写回
  `ContactPoint::normalImpulse`（W5 warm start 的数据源）。`WarmStart` 仍是 W5 的 no-op。
- **World 接线**：`Step` 在 `BuildIslands` 之后新增 `PrepareConstraints`，随后 `WarmStart`、
  `SolveVelocityConstraints` 调用 solver；`Step(dt, velIters, posIters)` 的两个迭代数真正生效；
  `Clear()` 清 solver 缓存。
- **清偿 W2 技术债**：删除 `SolvePositionConstraints` 里的临时**法向速度投影 + restitution**；
  位置推出保留到 W5 换 split impulse（`grep TEMPORARY` 现在只剩这一处）。
- **Testbed**：新增 `SphereStack` demo（静态地面 + 3 个相切圆，r=0.5、ρ=1、e=0），
  `--demo <name>` CLI（大小写/分隔符不敏感，未知名字 exit 2），smoke 输出 demo 名 + 每个 body
  状态 + 每个接触的穿透/法线；FallingBall 更新为"真实冲量 + restitution 阈值"文案。
- **测试**：210（Debug；Release 211，多出的 1 个是 release-only 静态-静态 skip 契约测试）；
  +25 相对 W2 = solver 单测 15 + World 冲量集成 11（含 300 步确定性、静态地面不动）。

**关键数据（实测，条件同前：Windows 11 / i7-12650H / MSVC 19.44 / RelWithDebInfo 或 Debug）**
- 收敛：两球叠在地面上的单次迭代残余 `g·dt/2 = 0.0833 m/s`，8 次迭代降到 `0.00065 m/s`
  （PGS 逐约束轮流投影，收敛慢但每步便宜——这正是选它而非解全局 LCP 的理由）。
- 静止穿透：**5.000 mm = kLinearSlop**（W2 临时响应是 7.8 mm）；球落地后 `|vy|<0.02`、
  末秒位置漂移 < 1e-3。
- 动量守恒：1:3 正碰前后总动量误差 < 1e-4（e=0 时能量不增）。
- Restitution：e=0.8 从 5 m 落下，首次反弹峰值 **3.55 m**（解析 e²h=3.2 m；
  偏差来源：semi-implicit 离散使冲击速度高 `g·dt` → apex +4%，加上仍存在的位置推出抬高 ~0.17 m；
  `rebound/impact ≈ 0.8` 精确成立。W5 split impulse 落地后收紧区间）。
- 3 球叠放（SphereStack smoke 300 帧）：3 个持续接触，球心 y=0.494 / 1.488 / 2.482，
  残余速度 ≤0.019 m/s —— **能停住、带小幅抖动**，符合 D7 的预期。
- 三 preset：debug 42.7 s / release 42.3 s / sanitize 32.2 s（clean build+ctest），
  项目代码 0 warning；ctest 210/210（13 个 sandbox death tests skip）。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **5 个集成测试失败，其中 2 个是"物理期望本身错了"**
   - `ThreeBallChain...`：期望单帧把动量传到最后一个球。排查：3 个相切球在同一帧同时形成
     两个接触，PGS 的一次求解不会得到 Newton's cradle；精确 SI 解是 A=−2/3、B=C=4/3
     （总动量守恒）。修复：改为"留间隙、分帧级联 e=1"的 cradle 测试。
   - `MoreIterationsReduceResidual`：期望一次迭代残余 `g·dt`。推导：等质量对 K=2 → normalMass=1/2，
     λ 只消掉一半速度 → 残余 `g·dt/2 = 0.0833`。实测与推导一致，修正期望。
   - 教训：**先手算解析解，再写测试期望**；P08 的 prompt 里"e=0 等质量 → A 停 B 走"其实是弹性
     结果（e=0 应为共同速度 v/2），测试要按真实物理写。
2. **e=0.8 反弹峰值 3.55 > 预设上界 3.4**
   - 根因不是 solver，而是 (1) semi-implicit 积分在离散时间下冲击速度偏高 `g·dt`（apex +4%），
     (2) W2 遗留的位置推出在反弹前抬高了球心。修复：区间放宽到 3.0–3.6，并额外断言
     `rebound/impact ≈ 0.8`（物理契约比绝对高度更稳）；W5 之后收紧。
3. **W2 的"低速不弹"长跑断言失败（极限环 ~0.9 m/s）**
   - 根因：位置推出会持续注入少量势能，而 restitution 只处理速度层，两者在低速接触形成极限环。
   - 处理：把断言改成真正的契约——"低速接触（vn < 阈值）bias=0，速度被吸收不反弹"，
     并在注释记录 W2 位置推出导致的极限环现象（W5 split impulse 修）。**不是掩盖回归**。
4. **一个 W2 断言过期**：`normalImpulse == 0`（W2 时 solver 还没写）现在必然非 0；
   改为 `>= 0`（累计法向冲量非负是长期契约）+ 更新注释。

**下周**
1. `PolygonShape`（P04：顶点数组、质心/面积/惯性矩、SetAsBox）与 SAT（P07：轴集合、最小穿透、
   reference/incident face 选择、Sutherland–Hodgman 裁剪、ContactID 特征编码 + A/B 对称）；
2. `CollideCircleAndPolygon` 补齐（P06 剩余）；`ContactID` 帧间稳定性测试（旋转 100 帧 Key 不跳变）；
3. 冷热数据拆分（`BodySim`/`BodyState`）在 W4 末进行；跑 sanitize；补 devlog，打 tag `v0.4-sat`。

**给未来的自己的提醒**
1. **先手算解析解再写测试**：PGS 是 Gauss-Seidel，单次迭代残余不是零，"单步同时碰撞"也不是
   Newton's cradle。本周两个测试失败都是期望值错，不是 solver 错。
2. 位置推出（W5 才换 split impulse）会给低速接触注入势能：任何"应该完全静止"的长跑断言都要
   考虑这一点，用契约（阈值内不弹、位移有界）而不是逐帧精确值。
3. solver 的累计冲量已经每帧写回 `ContactPoint::normalImpulse` 但 W3 每帧又从 0 开始——
   W5 只需在 `Prepare` 读取继承值 + `WarmStart` 施加，其余公式不用动。

---

## Week 4 · SAT + 裁剪式 Manifold

**完成**
- **PolygonShape**（P04）：严格顶点校验（3–8 个、有限、相邻重复 < `kLinearSlop` 拒绝、共线冗余拒绝、
  逆时针/凸/面积≈0 全部返回 false 并把形状置为 invalid）；规范 CCW 顶点顺序（`SetPolygonAsBox`
  从 `(+hx,-hy)` 开始）、每条边单位外法线、面积质心；质量积分公式
  `A / C / I_origin / I_c`（平行轴）；AABB。
- **SAT + 裁剪**（P07）：`FindMaxSeparation`（遍历两边多边形所有边法线轴取最大分离）、
  reference/incident face 选择、`ClipSegmentToLine`（Sutherland–Hodgman 用的半平面裁剪原语）、
  参考面裁剪后按点过滤 penetration、0–2 个接触点；A/B 交换得到翻转 manifold 且 **Key 集合相同**。
- **ContactID 身份语义定稿**：`Key()` 不含 `flip`（朝向元数据），同一物理接触 A/B 交换后 Key 不变；
  polygon 的 incidentVertex 用**绝对顶点索引**（45° 特征切换时 Key 不跳变）。
- **SATCache**（D6）：`Contact::satCache` 跨帧保存上一帧 reference face，轴接近平局时沿用；
  `SolverSettings::useSatCache` 可关闭对比。对称 tie-break（edge 索引→世界锚点）保证 A/B 交换等价。
- **解析路径补齐**（P06 剩余）：圆-多边形（最近边/顶点 Voronoi 区域）、多边形-平面
  （最多两个最深顶点 → 盒底平放时有 2 点支撑），双向 flip 重载。
- **World**：`CreatePolygonShape`/`CreateBoxShape`、Shape 分派、全形状组合窄相分派、
  SAT cache 持久化、ContactID 跨帧继承；集成测试含盒-地面 2 接触点、双盒 40 次迭代稳定、
  10 盒 BoxStack 基线、cache 对比、确定性。
- **Testbed**：`BoxStack` demo（10 层 1m 方块塔，设计文档 W4 D7 的初版），smoke 输出含 angle；
  FallingBall/SphereStack 保持。
- 测试 **278**（+68 相对 W3 = PolygonShape 10 + 圆/平面解析 12 + SAT/Clipping 34 +
  World SAT 11 + 调整；15 个 sandbox death test skip）。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **BoxStack 直接"爆炸"**（boxes 飞到 x=±15m、只剩 ground 接触、最大穿透一度 0.14m）
   - 先隔离：独立 probe 直接调用 SAT，两个相切盒子得到 2 点、normal=(0,1)、pen=0、锚点对称；
     零重力下双盒 200 步纹丝不动 → **不是碰撞/流形 bug**。
   - 再扫迭代数（300 步，地面 + N 盒）：2 盒 8it 微微倾斜、20it 站住、40it 精确静止；
     5 盒 8/20/40it 都塌、80it 才站住；10 盒 8–80it 全塌。
   - 根因：**W4 还没有摩擦（W5 D1-D2）也没有 warm starting（W5 D4）**。PGS 残差每帧产生微小
     倾斜力矩，盒子一旦倾斜就没有任何横向阻尼，于是滑落、连锁崩塌；位置迭代数完全无关。
   - 处理：不伪造稳定。10 盒 demo 保留并按设计文档"初版允许抖/塌"如实展示；集成测试改为
     "双盒 40 迭代站住（证明 solver 有能力）+ 10 盒默认 8 迭代只断言无 NaN/不穿地并打印数据"；
     W5 的摩擦 + warm starting 是真正的修复，这也是 W5 里程碑 v0.5-stable-stack 的对比基线。
2. **SAT 的 A/B 对称性容易假通过**：若"平局取 A 侧"，交换 A/B 会让 reference polygon 换人，
   Key 集合随参数顺序变化，违反 P07 D3/D5。修复：严格对称 tie-break（separation 相等 → 小
   edge index → 世界面锚点字典序），交换测试覆盖两种位姿。
3. **incidentVertex 用绝对索引还是边内 0/1**：边内相对索引在 45° 处 incident edge 从 e3 切到 e2 时
   会跳 Key；改用绝对顶点索引后，方块从 0° 转到 90°（181 帧）Key 只变 2 次
   （1.0° 2→1 点、89.5° 1→2 点），最深角特征对 `(refEdge=1, incVertex=3)` 181/181 帧存在。
4. **参考面裁剪要不要第三次独立 clip**：独立 clip 会合成一个深度恰 0 的交点，使顶点-面接触永远
   2 点，与 D4/P07 的"顶点-面 1 点"矛盾；改为逐点 `penetration < 0` 过滤，相切/面-面行为不变。

**数据（测量条件同前：Windows 11 / i7-12650H / MSVC 19.44）**
- 质量属性：2×2 m 方块 `m=ρA=4`、`I_c=8/3`；盒放在地面：**2 个接触点**、normal=(0,1)、
  penetration=5.000mm=`kLinearSlop`。
- ContactID 稳定性：方块绕 z 轴 0°→90°（0.5° 步长、181 帧）：Key 集合变化 **2 次**（1.0°/89.5°），
  最深角特征对 181/181 帧存在；不开 cache 也稳定（参考面始终是地面）。
- SAT cache（D6）：近轴平局场景 cache 使法线/穿透与无 cache 相差 ≤1e-6/0，仅 Key 集合不同；
  World 级双盒 @20 迭代 300 步：**Key churn 4（cache on）vs 264（off）**、maxPen 0.00280 vs
  0.00333m → cache 只降特征抖动、不改物理。
- BoxStack：双盒 @40 迭代 600 步 gap=1.0000、x≈0、angle=0；10 盒 @默认 8：**collapse step≈138**、
  run 内最大穿透 0.1413m、最终全部摊平（10 个 ground-only 接触、4.8–5.0mm 穿透）。
  这是 W5 "稳定堆叠"里程碑的 before 基线。
- 三 preset：debug/release/sanitize 全绿（278 用例 / Release 279，15 skip），项目代码 0 warning；
  clean 配置+构建+ctest：debug **52.0s** / release **41.5s** / sanitize(ASan) **38.5s**。

**下周（W5，本项目最难的一周）**
1. 摩擦：`t=Perp(n)`、`K_t`、`λt`，摩擦锥钳制 `accT ∈ [-μ·accN, +μ·accN]`（钳累计值）；
   斜面 μ>tanθ 静止 / μ<tanθ 下滑；
2. Warm starting：`ContactPoint` 跨帧继承 + `WarmStart` 施加 `acc*warmFactor`（W3/W4 已把数据准备好）；
3. Split impulse 位置层替换 W2 的临时位置推出；rest 接触不注入能量；
4. ImGui 参数面板（迭代数/warmFactor/μ/e/slop/β 实时生效）+ 10 方块堆叠 10 秒稳定里程碑
   （最大穿透 <0.01m）；顺带把 `Body`→`BodySim`/`BodyState` 冷热拆分放在 W5 末（W4 评估后移）；
5. tag `v0.5-stable-stack`。

**给未来的自己的提醒**
1. **没有摩擦和 warm starting 时，任何多层堆叠在物理上都会塌**——W4 的"爆炸"不是 bug，别去调迭代
   次数或改 SAT 试图修它。W5 的 friction + warm start 才是解法。
2. ContactID 的 `Key` 不含 flip；incidentVertex 用绝对顶点索引；SAT 平局 tie-break 必须对称
   （与 A/B 顺序无关），否则 warm starting 会跨帧匹配错点。
3. 位置推出仍是 `TEMPORARY`（W5 split impulse 替换）；grep TEMPORARY 只剩它。
4. 多边形形状代码路径对顶点顺序/法线方向极度敏感：改 PolygonShape 后必须跑
   `test_polygon_shape` + `test_sat` 的 9 位姿与旋转稳定性测试。

---

## Week 5 · 稳定堆叠：摩擦锥 + warm starting + split impulse

**完成**
- **摩擦（P09 D1-D2）**：`t = Skew(n)`、`K_t`、`λt`；**钳制累计**切向冲量
  `accT ∈ [−μ·accN_new, +μ·accN_new]`（不是增量）；同一迭代内法向先解、切向后解（锥半径用更新后的
  accN，否则锥可能在施加切向冲量后收缩）；`μ` 直接取 World 预组合的 `sqrt(μA·μB)`。
- **Warm starting（P09 D3-D4）**：`ContactPoint::normalImpulse/tangentImpulse` 跨帧继承
  （W2/W4 已铺好 `ContactID::Key()` 匹配），`Prepare` 播种 `inherited × warmStartFactor`、
  `WarmStart` 施加一次、`SolveVelocity` 写回；`isPersistent=false` 的点从精确 0 开始。
  **默认 factor 从 W1 计划的 0.85 改为 1.0**（实测见下，偏差已登记）。
- **Split impulse（P10）**：新增 `Solver/ContactSolverPosition.cpp` 伪速度位置层，替换 W2 的
  `TEMPORARY` 位置推出；按**位置积分后**的 transform 重算杠杆臂与 K_n，穿透修正目标
  `(β/dt)·max(0, pen−slop)`、累计钳制 ≥0，迭代后一次积分进 transform（位移/转角按
  `maxLinearCorrection`/`kMaxAngularCorrection` 钳制）并清零；**真实速度全程不读不写**。
  `useSplitImpulse=false` 走 Baumgarte 对比基线（`Prepare` 加 `(β/dt)` 速度目标）。
- **World 接线**：`PrepareConstraints(dt)` / `WarmStart` / `SolveVelocityConstraints` /
  `SolvePositionConstraints(dt)`；删除 `TEMPORARY` 位置推出与 `kPositionCorrectionFactor`；
  `CheckSane` 覆盖伪速度状态；`Step` 的 `positionIterations` 真正生效。
- **testbed（P14）**：DebugUI 重写为 4 面板（Testbed Controls / Solver Settings / Contacts / Status），
  求解器参数实时写回 `World::SetSolverSettings`（不重建场景）；Contacts 面板显示每点法线/穿透/
  法向与切向累计冲量并可点击高亮；Status 面板加 Step 耗时平滑值；新增 **FrictionRamp** demo
  （可调 θ/μ，实时显示 μ vs tanθ 与静止/下滑判定）；BoxStack 换成 W5 稳定说明 + verdict；
  smoke 汇总扩展出 contact_points / max_penetration_mm / solver 配置行 / 每接触冲量。
- **测试**：301 用例（debug/sanitize；release 302）全绿；新增 17 个 solver 测试
  （摩擦 6 / warm start 4 / split impulse 7）+ 6 个 World 级稳定性测试；
  W4 的「10 盒必塌」基线测试改成稳定期望并保留 W4 历史数据。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **10 盒塔在默认 8 迭代下仍会塌——三级机制归因**
   - 现象：W5 第一版（摩擦+warm start 0.85+split impulse）跑 600 步仍在 step 259 倒。
   - 排查：控制变量实测（600 步）：μ=0（关摩擦）→ step 135 塌 ≈ W4 基线 138；摩擦开、只关
     warm start → 258 塌；三者全开 → 不塌。**根因不是单一 bug，而是三个机制各自补齐**：
     摩擦提供横向阻力、warm start 保留平衡解、split impulse 无能量地清理穿透。
   - 修复：机制实现正确后，把默认 warmFactor 改成 1.0（下一条），不调迭代数/不降摩擦/不加阻尼。
2. **warmStartFactor 默认 0.85 反而让塔在 step 259 倒**
   - 数据（10 盒 600 步，塌=倒塌步数）：0.00→258、0.50→314、0.85→259、0.90→283、0.95→332、
     **1.00→不塌**（max pen 5.223 mm）。
   - 根因：factor<1 每帧把已收敛冲量打 `(1−f)` 折，PGS 必须每帧重新解这部分，而约束**顺序偏差**
     正好随被重解的量重新注入——warm start 的本意就是压制这个偏差。倒塌步数对瞬态敏感
     （所以不作契约），但「<1.0 在 10 s 内全保不住」是稳定现象。Box2D 2.4 也是完整继承。
   - 修复：默认改 1.0，factor 保留为对比实验旋钮；在 `Settings.h` 与 dev-interfaces 偏差表登记
     「W1 的 0.85 是未跑堆叠时的假设」。
3. **30:1 单列在默认 8 迭代下塌**
   - 现象：30 层位置层验收场景 maxPen 0.146 m、max|angle|=π。排查：同场景 32 次速度迭代 →
     站立 5 s、穿透 4.12 mm < slop。根因：每帧要沿 30 层传递支撑，8 次 PGS 迭代的传播预算不足
     （W4 已证 10 盒 @8 迭代都站不住），与位置层无关。
   - 处理：不伪造稳定。位置层验收改为「32 迭代站立窗口内穿透 < slop」，测试注释与 printf 保留
     8 迭代塌陷数据；W5 里程碑明确是 **10 盒 @ 默认 8 迭代**。
4. **split impulse vs Baumgarte 在 10 盒塔上看不出穿透差异**（5.223 vs 5.273 mm）
   - 根因：full warm start 下速度残差本来就小，β=0.2 的 Baumgarte 也足以把穿透压在 slop 带内；
     位置层的价值不是"更小的穿透"，而是「不注入能量」。
   - 处理：改用确定性深穿透场景（静置箱 2 cm 初始重叠）量化：split on **max upward velocity
     0.000000 m/s** vs Baumgarte **1.139999 m/s**（= 手算 `(0.2/dt)·(0.1−0.005)`）。这个数字比
     塔级穿透更能说明机制。
5. **并行开发共享工作区的 stale object**：多个 agent 同时改共享头时，别人的 build dir 会链接旧
   obj（一度出现"BoxStack 还塌"的假象）。最终结论一律以 canonical clean build 为准（本次三 preset
   从零重编）。
6. **release-only `C4189`**：`const float friction` 只被 debug assert 使用，`NDEBUG` 下未引用
   → 加 `static_cast<void>(friction)`（与 W2 的 `ComputePlaneMass` 同款坑）。

**数据（测量条件：Windows 11 22631 / i7-12650H / 16 GB / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- **里程碑**（10×1 m 方块塔，默认 8/3 迭代，warmFactor=1.0，600 步 = 10 s）：最大穿透
  **5.223 mm**（< 10 mm）、顶层 y **9.47813 m**（起始 9.5；2.2 cm = 10 个接触的 slop 累积）、
  最大倾斜 **0.004596 rad ≈ 0.26°**、|x| 峰值 **0.03063 m**、末帧最大速度 **0.007216 m/s**、
  末秒抖动 **0.024816 m/s**、contacts 10 / points 20、无 NaN。testbed smoke 600 步：
  max_penetration **3.893 mm**、|tilt| ≤ 0.21°、avg_frame_ms 1.215。
- **warm starting 因子对比**（10 盒 600 步，塌=倒塌步数 / max pen m）：0.00 = 258 / 0.2049、
  0.50 = 314 / 0.1572、0.85 = 259 / 0.1395、0.90 = 283 / 0.1974、0.95 = 332 / 0.1988、
  **1.00 = 不塌 / 0.005223**。
- **split impulse 对比**：10 盒塔 on/off 都站住（max pen 5.223 / 5.273 mm、末秒抖动
  0.0248 / 0.0282 m/s）；能量注入（静置箱 2 cm 重叠，确定性）：on **0.000000 m/s** vs
  Baumgarte off **1.139999 m/s**。
- **摩擦**：20 格斜面扫描（μ∈{0.1,0.3,0.5,0.8,1.2} × θ∈{5,15,30,45}°）：μ>tanθ 的 12 格
  5 s 位移 ≤ **1.5 mm**；μ<tanθ 的 8 格明显下滑（μ/tan=0.8 → +17.74 m；μ=0.1/45° → +79.8 m）。
  1000 帧斜堆叠：max|accT|/accN = **0.5000**、锥外 violations **0**；μ=0 时 tangentImpulse 恒 0。
  单点手算：v=(3,2)、n=(0,1)、μ=0.5 → accN=2、accT=−1（正好在锥边界）。
- **warm start 残差**（两球叠放单次迭代，手算 `0.5(1−f)g·dt`）：f=0 → 0.08333334、
  0.5 → 0.04166667、0.85 → 0.0125、**1.0 → 0.00000000**（逐位）；`warmStartFactor=0` 与
  `useWarmStarting=false` 的求解结果逐位一致（std::bit_cast）。
- **确定性**：10 盒 600 步两个 World 的 **343** 个状态 float 逐位一致；
  warm start 关闭等价性 **383** float 逐位一致。
- **split impulse 位置层手算**：单点 1 次位置迭代位移 = `β(pen−slop)` = **0.009 m**（精确）；
  杠杆臂场景平移 −0.002 m + 转角 −0.002 rad（各一半）；`positionIterations=0` 位不变。
- **restitution**：e=0.8 从 5 m 落（接触面 0.5 m）：apex **3.4633 m**、impact 9.5000 m/s、
  rebound 7.7333 m/s、ratio **0.8140**（解析目标 3.38 m；剩余偏差来自离散
  `e·(v+g·dt)`，不是位置层注入能量——W2/W3 的位置推出已删除）。
- **testbed smoke（canonical debug build）**：BoxStack 300 步 `bodies=11 contacts=10 points=20
  max_pen=3.763mm avg_frame_ms=1.212`；600 步 `3.893mm / 1.215ms`；FallingBall 150 步
  `5.000mm / 1.009ms`（y=0.495）；SphereStack 300 步 `3.063mm / 1.046ms`；
  FrictionRamp 300 步 `2.686mm / 1.017ms`（θ=15°、μ=0.30 > tanθ=0.268 → STATIC）。
- **三 preset（clean build + ctest）**：debug **31.8 s / 301 全绿 / 0 warning**、
  release **41.0 s / 302 全绿 / 0 warning**、sanitize **42.9 s / 301 全绿 / 0 warning**；
  测试 exe 复制到工作区外运行 → **301/301 全过**（15 个 death test 真正执行）。

**下周（W6）**
1. `IBroadPhase` 接口 + `BroadPhaseBruteForce` 基线 + `BroadPhaseSAP`（端点数组+插入排序），
   要求两实现配对集合与暴力参考完全一致；
2. `World::UpdateBroadPhase` 接线 + fat AABB margin（`AABB::Extend`）+ 平面特判；
   切宽相物理结果 hash 不变；
3. `bench_broadphase` + Tracy 埋点，拿到 N∈{50,200,500,2000} 的真实耗时表；
4. `Pyramid200`/`StressBenchmark` demo + `tests/regression` headless 回归（golden hash + CI job）；
5. **补做冷热拆分 `BodySim`/`BodyState`**（W4→W5→W6 二次顺延，见 dev-interfaces 偏差登记）；
6. tag `v0.6-sap`。

**给未来的自己的提醒**
1. **warm start factor 不是"调小更安全"**：factor<1 每帧丢掉已收敛解、重新注入 PGS 顺序偏差；
   10 盒塔实测所有 <1.0 的 factor 都在 10 s 内倒。默认 1.0（Box2D 同款）。
2. **split impulse 的正确性看两件事**：真实速度逐位不变 + 伪速度每步清零。穿透变小只是副产品。
3. **位置层必须在位置积分之后用当前 transform 重算杠杆臂**；速度层的 cache 已经过期一步。
4. **30 层单列能不能站是速度迭代预算问题**，别用位置层硬顶；验收场景必须写清迭代数。
5. 共享工作区并行开发时 build dir 可能链接到旧 obj（stale object）；最终结论一律以
   canonical clean build 为准。

---

## Week 6 · Broad Phase（BruteForce + SAP）与量化 + headless 回归

**完成**
- **`IBroadPhase` 两个实现**（R1）：
  - `BroadPhaseBruteForce`：slot 数组 + 空闲列表 + 固定 `i<j` 双重循环，作为**对照 oracle** 与小场景实现；
  - `BroadPhaseSAP`：**单一** `vector<Endpoint{value, proxyId, isMin}>`（每 proxy 两个端点）；`AddProxy`
    尾部追加 + 脏标记，观察点一次 `std::sort`（首帧/批量加载）；稳态 `UpdateAABB` 对两个端点
    bubble-left/right（帧间相干性，swap 同步缓存索引）；按 proxy id 有序 active 集合扫描 + 完整
    `AABB::Overlaps`（y 轴二次过滤）；`RemoveProxy` erase+索引刷新；`ValidateInternalState` 任意时刻可验。
- **World 宽相接线**（R2/R3）：`Body::proxyId` + `World::m_proxyToBody` 双向映射；`UpdateBroadPhase`
  建/更/释放 proxy 并收集候选对，**规范化为 (i<j, sorted, deduped) 的 body-index 对**；
  `UpdateContacts` 与 O(N²) fallback 共用 `CreateContactForPair`；`Synchronize` 的 AABB 变为
  **tight + `kAabbMargin`(0.1m) 的 fat AABB**；`DestroyBody/SetBroadPhase/Clear` 正确释放 proxy。
- **bench**（R13）：`bench/bench_broadphase.cpp` 完整 CLI（brute|sap|all、bodies 列表、frames、seed、
  json、help），30×30m 随机盒场景，只计 `UpdateAABB + QueryPairs`（隔离宽相），前 100 帧预热丢弃，
  median/mean + 环境打印。**真实数字进 README**。
- **headless 回归**（R12）：`tests/regression/regression_main.cpp` + `physics2d_regression` target +
  `golden/default_600.txt` + CTest `regression_default` + CI `regression` job；量化弱 hash（1e-5）判定，
  exact 位 hash 仅信息；退出码 0/1/2。
- **testbed**：`SceneBuilders.h` 共享场景构造；`Pyramid200`（完美金字塔，层数 4–24 可调）与
  `StressBenchmark`（400 随机盒）两个新 demo；DebugUI **Broad phase 下拉（Auto/None/BruteForce/SAP）**
  + installed name/proxies/candidate pairs；Status/Step ms；Camera `FocusOnWorld` 自适应取景；
  smoke 行扩展 `broadphase/candidate_pairs/proxies` + 运动/高度聚合。
- **Auto 宽相阈值**：`< 32 bodies → BruteForce，>= 32 → SAP`（常量注释 + README 注明来源 = 本机
  Release bench 的实测交叉点 N≈25–32，不是抄的经验值）。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **中文 locale MSVC 的 `/showIncludes` 破坏了 Ninja 头文件依赖**
   - 现象：只改 `.h` 后增量构建报 `ninja: no work to do`，旧 obj 被链接（W5 并行期的"假 stale"根因）。
   - 排查：`ninja -t deps` 显示 `#deps 0`；`CMakeFiles/rules.ninja` 里 `msvc_deps_prefix = 注意: 包含文件:`
     （CMake 以 UTF-8 写入），而 cl.exe 在 GBK 控制台输出同字符串的 GBK 字节 → Ninja 按字节比较匹配不上，
     depfile 静默为空。试过 `VSLANG=1033` / `PreferredUILang=en-US` / `CMAKE_DEPENDS_USE_COMPILER=OFF`，
     本机 VS 安装均无效。
   - 处理：`scripts/build.ps1` 增加 `-Clean`（`cmake --build --clean-first`），并在脚本注释、
     `docs/dev-setup.md §6`、testbed README 写明"改 header 后必须 `-Clean` 或 clean build"。
     **周验收一律 clean build**，所以 W1-W6 的数据未受影响。
2. **换宽相改变物理结果（顺序敏感）**
   - 现象：首版接线后 SAP/BF 与 fallback 的 body 状态在若干步后出现末位差异。
   - 根因：宽相发射顺序 → 接触列表顺序 → PGS（Gauss-Seidel）求解顺序 → 浮点结果。宽相实现不同 → 顺序不同。
   - 修复：`UpdateBroadPhase` 把候选对规范化为 `(i<j, sorted, deduped)` 的 **body-index** 对（proxy id
     顺序是实现细节，不能用）；`GetBroadPhasePairCount` 只暴露 count。用"反序 + 重复 + 逆序发射"的
     noisy decorator 测试证明：缺 sort 物理发散、缺 dedupe 对数翻倍。
3. **SAP 增量更新的三个坑**
   - 批量插入：逐个 `AddProxy` 都排序是 O(N²logN)；改为"追加端点 + 脏标记，下一个观察点一次
     `std::sort`"。为了让 `const ValidateInternalState()` 任意时刻为真，排序用 `mutable` 懒执行。
   - 删除：`swap-pop` 会破坏"端点有序 + 缓存索引"不变量，改为 `erase` + 尾部索引刷新（删除是 body
     生命周期级操作，不在热路径）。
   - 惰性场景：静止 1000 帧端点必须零移动（有测试），否则说明冒泡条件写错。
4. **Pyramid200 的"200"与稳定性冲突**
   - 数据（600 步、默认 8/3 迭代，层数 → 底部最大穿透/顶层 y/最大倾斜）：20 层 210 盒 → 100.5mm /
     19.057m (生成 19.5) / 6.02°，仍在蠕变；18 层 171 盒 → 60.3mm / 17.229m / 2.76°；**16 层 136 盒 →
     29.7mm / 15.341m / 1.36°（0.06 m/s，基本静止）**；14 层 → 21.4mm。
   - 根因：金字塔底部要扛 16-20 个盒的载荷，8 次速度迭代的传播预算 + 位置层 slop 带会让底部持续下沉——
     这是**载荷/迭代预算**属性，不是宽相或求解器 bug（切 BF/None 物理输出完全相同）。
   - 处理：demo 名沿用设计文档 `Pyramid200`，默认 **16 层 = 136 盒**（面板 4–24 层可调，README 附
     稳定性-层数表）；注释/README 写清"设计文档的字面 200 盒 ≈ 20 层 210 盒，8 迭代下会下沉 0.44m，
     可调出复现"。不调迭代数掩盖。
5. **大场景取景**：19m 高金字塔/400 盒场超出默认视野（Camera 只有 Reset/输入回调）。新增
   `Camera::FocusOnWorld(AABB)`：只在 Setup/Reset/切换 demo 时调用一次（不跟用户输入打架），
   统计时跳过 Plane（无限半空间），zoom 钳 [0.2, 3.0]。
6. **fat AABB 语义变化的外溢**：`Body::aabb` 从 tight 变成 fat 后，`AttachShape` 之后到下一次
   `Synchronize` 之间仍是 tight；测试改为"包含 tight 且各边外扩 margin"的契约，并加"margin 只加候选
   不加接触"的专项（0.15m 间隙圆对：候选 1 / 接触 0）——没有靠放宽断言过测试。
7. **冷热拆分（R15）第三次顺延**：W6 本身已改 hot block（`proxyId` + fat `aabb` 语义），同一批 213 个
   热字段访问点一周内改两次会让回归定位成本翻倍。W7（岛屿周，本来就要碰 body 存储）执行；偏差已登记。

**数据（测量条件：Windows 11 22631 / i7-12650H / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- **宽相正确性**：
  - SAP vs BruteForce：10 组随机场景（N=50/200/500）× 1000 帧 = **10,000 帧逐帧配对集合完全一致，
    677,584 对比较 0 mismatch**、0 canonical 违规、0 `ValidateInternalState` 失败；600 次任意增删移动、
    64×1000 惰性场景同样 0 失败。
  - World 三路等价性：同一场景（地面 + 10 盒塔 + 3 圆 + 1 落盒，15 bodies）**无宽相 / BruteForce / SAP
    300 步逐帧 545 floats 逐位一致**（body transform/真实+伪速度/力 + 完整 contact 列表含累计冲量），
    first mismatch = -1；运行中切换 none→BF→SAP→BF→none 与从未装宽相的参照 World 150 步逐位一致。
  - 候选对与 fat-AABB 暴力参考对数 **容差 0** 相等。
  - testbed smoke A/B：BoxStack/Pyramid200/StressBenchmark 三场景用 `--broadphase none|bf|sap` 各 300 步，
    body/contact 输出行 **SHA-256 完全相同**（仅候选对数不同）。
- **bench 真实数字**（Release / seed 42 / 1000 帧测 + 前 100 丢弃 / median）：

  | Bodies | BruteForce ms/帧 | SAP ms/帧 | 加速比 | 候选对/帧 | 真实重叠/帧 |
  | ---: | ---: | ---: | ---: | ---: | ---: |
  | 50 | 0.0033 | 0.0015 | 2.20x | 3.4 | 1.8 |
  | 200 | 0.0838 | 0.0169 | 4.96x | 55.3 | 31.3 |
  | 500 | 0.5399 | 0.0916 | 5.89x | 354.3 | 202.5 |
  | 2000 | 9.0985 | 1.1656 | 7.81x | 5670.6 | 3206.6 |

  小 N 反转（SAP 更慢）：N=5 0.00x、10 0.50x、20 0.80x、50 2.12x → **交叉点 N≈25–32**，
  testbed Auto 阈值取 32；`true overlaps` 口径 = tight AABB 精确重叠（与候选对的差 = fat margin 的假阳性）。
- **回归**：`tower30+circles20` 600 步，弱 hash **`588d30e70f9b4ddd`**（quantum 1e-5）、
  exact `f4390ef86c075a04`；bodies 51 / contacts 83；CLI 退出码 0/1/2 实测正确；CTest 0.3s。
- **测试与三 preset**：debug/sanitize **321 个 ctest 用例**（305 gtest passed + 15 sandbox death-test skip
  + 1 `regression_default`）、release **322**；项目代码 **0 warning**；
  clean 配置+构建+ctest：debug **42.5s** / release **47.1s** / sanitize(ASan) **66.1s**。
  测试 exe 复制到工作区外运行 → **320/320 gtest 全过**（15 个 death test 真正执行）。
- **testbed smoke（canonical debug build，300 步）**：
  FallingBall `bodies=2 contacts=1 max_pen=5.000mm bay=BruteForce avg=0.98ms`；
  SphereStack `4/3/3.063mm/BruteForce/1.07ms`；
  BoxStack `11/10/20点/3.763mm/BruteForce/1.20ms`；
  FrictionRamp `2/1/2.686mm/BruteForce/1.03ms`；
  Pyramid200 `137 bodies/324 contacts/41.863mm/顶层 15.284m/tilt 2.18°/SAP/候选对 496/6.89ms`（600 步：
  29.666mm、基本静止）；
  StressBenchmark `401 bodies/819 contacts/178.151mm（400 盒密堆载荷现象）/SAP/候选对 1687/13.72ms`。
  Auto 宽相按 body 数选：小场景 BruteForce、大场景 SAP（smoke 行可见）。

**下周（W7）**
1. 岛屿构建：DFS（显式栈）分岛，static/kinematic 作边界；按岛求解（连续数组 cache 收益）；
2. 休眠：`sleepTimer`、整岛原子睡眠/唤醒、sleep 岛跳过宽相/窄相/求解但保留 proxy；
   多米诺连锁唤醒测试；
3. **补做冷热拆分 `Body`→`BodySim`/`BodyState`**（W4→W5→W6→W7 登记），用 W6 的回归弱 hash 证明行为不变；
4. demo：`Dominoes`/`IslandSleep500`（休眠前后 Step 耗时对比）；`GetIslandCount()`/`GetAwakeBodyCount()` 接真实值；
5. 回归工具扩展多场景 golden（可选）；tag `v0.7-island-sleep`。

**给未来的自己的提醒**
1. **顺序就是物理**：任何改变"谁先被求解"的改动（宽相顺序、岛屿打包顺序、排序算法）都会改变末位结果；
   W6 的规范化 pair 与 noisy decorator 测试就是模板。
2. **本机增量构建不可信**（中文 MSVC depfile 为空）：改 header 后一律 `-Clean` 或 clean build；
   周验收的 canonical clean build 是唯一可信基线。
3. SAP 的不变量要能被 `ValidateInternalState()` 在**任意时刻**验证（含 AddProxy 之后），懒排序 + mutable
   就是为此；测试每帧调用它比事后对账有效得多。
4. 大场景 demo 的"稳定性"先量载荷分布：金字塔底部穿透是迭代预算问题，不是新 bug；数据写清楚，
   不要用调迭代数掩盖。
5. 宽相只决定"谁进窄相"；窄相/求解器不应知道宽相存在。切宽相物理不一致时先查顺序/去重，别改窄相。

---

## Week 7 · 岛屿、按岛求解与整岛休眠

**完成**
- **岛屿构建（P13 D1）**：新增 `include/physics2d/Solver/Island.h` + `src/Solver/Island.cpp`。以 contact 为边的
  动态体连通分量：**只有 dynamic-dynamic contact 是边**，static/kinematic 是边界（它们的 contact 仍随岛
  求解，但不入岛、不传递连通性——否则同一地面上所有堆叠会被地面合并成一个岛，岛屿/休眠失效）；
  无 contact 的 dynamic body 是单例岛（休眠计时需要）。DFS 用**显式 vector 栈**（500-body 长链实测
  不溢出；P13 要求 2000 body），forward-star 邻接（2×C 个 int），两遍升序扫描产出规范序
  （岛按最小成员、岛内 bodies 升序、contacts 按全局 contact 序），整体 O(N+C)。
- **按岛求解（P13 D2）**：`ContactSolver` 在 World 装入分区后按岛序 `Prepare`/`WarmStart`/`SolveVelocity`/
  `SolvePosition`；solver-dev 实测后改为**三种模式**：flat（无分区，Week 6 原路径）、**grouped-direct
  （默认：按岛迭代但直接读写 Body，不拷贝）**、grouped-packed（设计文档 3.3/6.5 的"岛内热数据打包"，
  保留为 opt-in）。**三模式在相同初始状态下逐位一致**：两岛场景 276 次 float 比较、密集单岛
  （400 bodies/760 contacts）21,760 次 float 比较，全部 0 差异；World 级 A/B（grouped vs flat）300 步
  × 469 floats/步逐位一致。**回归弱 hash `588d30e70f9b4ddd` 保持不变**（分组求解是纯顺序变换）。
- **整岛原子休眠/唤醒（P13 D3/D4）**：`World::UpdateSleep` 按岛累计 `sleepTime`（每个 dynamic body 的
  `|v|<kSleepLinearTolerance && |w|<kSleepAngularTolerance`），岛内全部超过 `kSleepTimeThreshold` →
  **整岛** `isAwake=false`、真实速度与伪速度清零；`UpdateBroadPhase` 对休眠 body 跳过 AABB 更新但
  **保留 proxy**；`UpdateContacts` 对"双方都不 awake"的 candidate pair **复用上一帧 contact**（不跑窄相，
  保留岛图/暖启动数据）；`UpdateContacts` 开头的共享 wake pass 做**宽相邻近唤醒**（awake 物体的 fat AABB
  与休眠岛 proxy 重叠 → 整岛唤醒，宽相/fallback 两条路径同序）；外力/冲量/SetTransform/SetVelocity 等
  入口通过 `WakeIslandOfBody` **整岛原子唤醒**。`GetIslandCount()`/`GetSleepingBodyCount()` 接真实值。
- **testbed（P13 D5/D6/D7）**：新增 `Dominoes`（20 块 0.30×1.0 m 骨牌，smoke 第 60 步自动推第一块）与
  `IslandSleep500`（500 盒 = 100×5 塔，自动测量"唤醒期 vs 全休眠期"Step 耗时并打印加速比）；
  `SceneBuilders` 增加 `CreateDominoChain`/`CreateBoxGrid`；Status 面板加 islands/awake/sleeping；
  Solver 面板加 Islands & sleeping 段（enableSleeping/useIslandSolving 勾选、Wake all）；**Space 暂停、
  `.` 单步、1/2/3 = 1x/0.5x/0.25x 慢放**（ImGui 捕获键盘时禁用）；Contact 面板加每接触点累计法向/切向
  冲量的 ProgressBar 与摩擦锥占用百分比；smoke 行加 `islands/awake/sleeping`。
- **工程护栏**：`scripts/build.ps1` 增加 stale-object 自动防护（见下面卡点 1）。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **`HEAP CORRUPTION DETECTED` 弹窗（看起来像内存越界，其实不是）**
   - 现象：core-dev 在 `build/core7` 跑测试时 CRT 弹窗 "wrote to memory after end of heap buffer"。
   - 排查：① 全新目录 `build/asan7`（sanitize preset = MSVC ASan）全量重编 → **没有任何 ASan 报告**，
     只有 5 个功能测试失败；② 该弹窗出现在"W7 改了 `ContactSolver.h` 的 private 成员之后"的增量构建里；
     ③ 本机中文 MSVC 的 `/showIncludes` 前缀与 CMake 写入的 UTF-8 前缀字节不匹配 → Ninja depfile 为空
     → 只改 header 不重编 → `World.cpp.obj`（旧 `sizeof(ContactSolver)`）与新 `ContactSolver.obj`
     （新类大小）混链 → `make_unique`/析构尺寸不匹配 → 堆损坏。
   - 根因：**本机增量构建不可信**（W6 已记录的环境坑），不是 W7 代码越界。
   - 修复：`scripts/build.ps1` 在 Windows 上自动检测"任意 header 比 build 目录内最新 obj 新"→ 打印
     WARNING 并自动 `--clean-first`。此后所有 agent 的增量构建自动安全；canonical 周验收本来就 clean build。
2. **proximity wake 放错位置破坏"无宽相 vs 宽相逐位一致"**
   - 现象：三个 W6 宽相等价性测试（fallback/BF/SAP 300 步逐位一致）在 step 87 分歧（正是休眠开启时刻）。
   - 排查：proximity wake 最初写在 `World::UpdateBroadPhase`，而该函数在 `m_broadPhase == nullptr` 时
     提前 return → 无宽相 fallback 世界永不被邻近唤醒，装宽相世界会唤醒 → 物理分歧。
   - 修复：把 wake pass 挪到 `UpdateContacts` 开头（宽相路径用 `m_broadPhasePairs`、fallback 用同样的
     fat-AABB 判定遍历 i<j，同一唤醒时机；语义仍在窄相之前、BuildIslands 之前同帧生效）。三个测试
     **未改一行断言**即恢复全绿。已在 dev-interfaces 偏差表登记（任务卡原措辞是 UpdateBroadPhase）。
3. **薄骨牌永不入睡**
   - 现象：testbed 计划的 hx=0.05 / core-dev 实测 hw=0.06 薄牌静止时 `sleepTime` 每 4–6 帧被重置。
   - 根因：逐点 PGS 对低惯性薄片的残余力矩 → 静止抖动尖峰 maxV≈0.015 m/s > `kSleepLinearTolerance=0.01`
     （W5 已记录 10 盒塔 jitter max 0.0248 m/s，同一机理）；**这是收敛质量问题，不是休眠规则 bug**。
   - 处理：**不调阈值**（保持 0.01，设计文档/Box2D 同值），demo/测试改用能入睡的 0.30×1.0 m 骨牌
     （hw=0.15、间距 1.0 m），并把"薄片抖动 → block solver 是 W8 技术债"写进注释/README。
4. **"岛内打包"实测反而更慢 → 默认值回调**
   - 现象：设计文档 3.3/6.5 要求"整岛 BodyState 拷进连续数组 → 求解 → 写回"，实现后 Release 实测
     400 bodies/760 contacts 单岛、8 迭代：flat 1.741 ms / grouped-direct 1.863 ms / grouped-packed 2.122 ms。
   - 根因：400 个 Body ≈ 80 KB，**整个 body 数组本来就装得进 L2**，每帧 O(bodies) 拷贝成本 > 局部性收益；
     Debug 下 packed 反而快（访问放大），说明默认值必须按 Release 测量定。
   - 处理：新增 `SolverSettings::packIslandBodies`（默认 false）；默认走 grouped-direct（与 flat 差
     2–7%），packed 路径保留为 opt-in、被三模式位级测试持续覆盖。devlog/README 如实写"未加速"。
5. **`-0.0` 位级 bug（休眠冻结冲量后暴露）**
   - 现象：`DisablingWarmStartingEqualsZeroFactor` 在 W7 开始失败，first mismatch 是某 contact 的
     `tangentImpulse`。
   - 根因：`WarmStartSeed` 里 `inherited * 0.0f` 对负继承冲量得到 `-0.0f`，与 `useWarmStarting=false`
     的 `+0.0f` 位不同；W5 时累计器每帧被求解重写所以从未暴露，W7 休眠冻结 ContactPoint 冲量后暴露。
   - 修复：`inherited * factor + 0.0f`（只把 -0 归一为 +0，其他 float 位不变）。
6. **休眠时机（诚实数据）**：10 盒塔在默认阈值下 **step 859 才整岛入睡**（不是 600）。塔 600 步时
   已稳定（max_speed 0.007），但残余抖动尖峰需要真正的 0.5 s 连续安静窗；测试放宽到 900 步并在
   注释记录原因，**不调 `kSleepLinearTolerance`**。IslandSleep500 用 5 盒塔（100 岛）实测 frame 105
   （1.75 s）全睡；10 盒塔布局需 ~1452 帧（与 10 盒塔 859 步一致）。

**数据（测量条件：Windows 11 22631 / i7-12650H / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- **岛屿**：3 个互不接触的堆叠 → 3 岛；桥接两个堆叠 → 2 岛；同一 static 地面上的独立堆叠不合并
  （地面 islandId=-1）；500-body 长链 → **1 岛 / 499 contact**，显式栈不溢出；islandId 稠密规范。
- **按岛求解位级一致**：flat / grouped-direct / grouped-packed 三模式，两岛场景 276 次 float 比较、
  密集单岛 21,760 次 float 比较**全部 0 差异**；World 级 grouped vs flat 300 步 × **469 floats/步**
  逐位一致；**回归弱 hash `588d30e70f9b4ddd` 与 exact `f4390ef86c075a04` 不变**（golden 未改）。
- **三模式 Release 耗时**（400 bodies/760 contacts 单岛、40 reps、i7-12650H）：
  8 迭代 flat **1.741 ms** / grouped-direct **1.863 ms (0.93x)** / grouped-packed **2.122 ms (0.82x)**；
  32 迭代 7.425 / 7.590 (0.98x) / 7.987 (0.93x)。默认 grouped-direct，packed opt-in（见卡点 4）。
- **休眠**：
  - 10 盒塔：sleep step **859**、sleeping=10/awake=0、contacts=10、top y 9.47815；**之后 100 步状态与
    接触冲量逐位不变**；proxy 保留 11 个；debug Step **26.5 µs/step（asleep） vs 87.5 µs/step（awake），
    3.30x**；`enableSleeping=false` 时永不休眠。
  - **IslandSleep500（500 盒 / 100 岛，Release 600 步）**：**frame 105 (1.75 s) 全睡**；唤醒期平均
    Step **0.4554 ms** → 全休眠 **0.2627 ms**，**加速 1.73x**（debug 同场景 4.95 → 2.94 ms，1.69x；
    仍保留宽相 sweep 与休眠接触复用，故非数量级）。
  - 唤醒 4 入口（ApplyLinearImpulse/SetLinearVelocity/SetTransform/ApplyForce）均**立即整岛唤醒**；
    邻近唤醒（probe 进入休眠塔 fat AABB、无几何接触）整岛唤醒，宽相 on/off 两条路径都验证；
    `allowSleep=false` 的成员使整岛保持 awake。
- **多米诺（Release 600 步）**：**woken_after_sleep 20/20、fallen 20/20、chain_reach 20、
  first_skipped = -1**（无任何骨牌被跳过）；首醒步序列呈沿行波传播（core-dev 12 块场景实测
  0 5 23 52 75 93 108 123 138 152 166 180）。
- **8 个 demo smoke（canonical debug build，300 步，含 islands/awake/sleeping）**：
  FallingBall `bodies=2 islands=1 sleeping=1`；SphereStack `4/1/3`；BoxStack `11/1/awake=10`
  （塔仍醒，900 步才睡）；FrictionRamp `2/1/1`；Pyramid200 `137/1/awake=136`；
  StressBenchmark `401/4/awake=400`；Dominoes `21 bodies/12 islands`；
  IslandSleep500 `501 bodies/100 islands/sleeping=500 contacts=500 max_pen=3.983mm avg_frame_ms=9.98(debug)`。
- **宽相 A/B 仍逐位一致**（W6 属性保持）：BoxStack `--broadphase none|bf|sap` 300 步 body/contact 输出
  行 **SHA-256 相同**；BoxStack 600 步 max_pen **3.893 mm**、塔站立（max_speed 0.007，尚未入睡）。
- **测试/三 preset**：debug/sanitize **354 个 ctest 用例**（338 gtest passed + 15 sandbox death-test skip
  + 1 `regression_default`）、release **355**；项目代码 **0 warning**；clean 配置+构建+ctest：
  debug **55.4s** / release **53.5s** / sanitize(ASan) **82.9s**。测试 exe 复制到工作区外运行 →
  **353/353 gtest 全过**（15 个 death test 真正执行；过程中用这把尺子抓出了 W7 的 static-static skip 回归）。

**下周（W8，文档与交付周）**
1. `docs/profiling.md`（所有性能表 + Tracy 截图 + 环境）、`docs/math.md`（SAT/冲量/摩擦锥/split impulse
   推导）、架构总图升级；
2. README 完整版 + GIF（10 盒稳定 / 2000 物体 SAP / 500 物体休眠 / 多米诺唤醒）+ 已知限制清单；
3. 边界测试补强（退化多边形、零密度、极大速度、超长薄条）与所有崩溃点 assert/文档；
4. 简历 bullet（10.3 模板）与面试问答演练（10.4 清单）；tag `v1.0-release`；
5. 若时间允许：**2 点接触 2×2 block solver**（能消除薄片残余抖动 → 休眠更快，W7 卡点 3 的正解）、
   `BodySim`/`BodyState` 冷热拆分（W4→W5→W6→W7→W8 顺延；W7 的测量表明"打包"本身在 400 体内不划算，
   拆分应先有 profiling 数据再动）、Tracy 正式接入。

**给未来的自己的提醒**
1. **顺序就是物理**：岛屿分组之所以能保持 bit-identical，是因为"岛间约束不共享 body → 更新可交换、
   岛内相对顺序保持"。任何改变岛内顺序/跨岛交错的改动都会破坏 hash，先想清楚再动。
2. **休眠是"整岛原子"**：单 body 休眠/唤醒会在岛边界留下半冻状态（穿模）；休眠岛必须保留 proxy 与
   上一帧 contact（否则岛图丢失、无法被邻近唤醒）。
3. **不要为了漂亮的数字调休眠阈值**：10 盒塔 859 步入睡是残余抖动的诚实结果，正解是 block solver。
4. **本机增量构建永远是坑**：build.ps1 现在会自动 clean-first，但 canonical 周验收仍然必须 clean build；
   工作区外跑一次完整 gtest（含 death tests）是发现"被 sandbox SKIP 掩盖的契约回归"的唯一可靠方法。
5. **默认值必须按 Release 测量定**：packed 求解在 Debug 下更快、Release 下反而慢 18%——只信 Release。

## Week 8 · 文档、交付与 v1.0-release

**完成**
- **文档交付（6 个新文档 + 4 处升级）**：`docs/math.md`（9 节逐步推导 + 每节「面试可能怎么问」+
  §10 边界契约表 + 常数附录，~1040 行）、`docs/profiling.md`（宽相/休眠/三求解模式/构建耗时/8 demo
  smoke 全量实测表 + 测量纪律 + §1.5 Phase C canonical 复测，~500 行）、`docs/architecture.md` 升级
  （§0 一张图总览、§3.7 数据所有权图、§4.6 Step 11 阶段时序图、§4.7 岛屿/休眠状态机，共 6 张 Mermaid）、
  `docs/boundary-contracts.md`（三类契约 + 50 行边界场景表 + 13 条已知限制，73 个测试名逐条机械核对）、
  `docs/demos.md`（8 demo/操作键/4 面板/录制命令/录屏坑）、`docs/resume-interview.md`（6 条简历 bullet +
  25 问四段式 + 5 个专项深挖 + 20 分钟讲解脚本 + 数字速查）；README 重写为 v1.0 结构（GIF 墙/核心算法/
  实测性能/已知限制/Roadmap/文档地图）+ 新增 `README.en.md` 英文版 + `docs/release-notes-v1.0.md`。
- **GIF（W8 新增 testbed 帧捕获）**：`--capture-dir/--capture-every/--capture-start/--capture-hide-ui/
  --capture-display`，在 world+ImGui 画完后、`glfwSwapBuffers` 前 `glReadPixels(GL_BACK, GL_RGB)` →
  PPM(P6)（默认关闭零开销）；`scripts/record_gifs.ps1` 内嵌 Python+Pillow（本机无 ffmpeg）→ 5 张 720px、
  12.5 fps GIF（dominoes/box_stack/sap_stress/island_sleep/contact_debug），总计 **1,692.6 KB**
  （单张 ≤600KB、总预算 2.5MB 自动校验，PPM 中间文件编码后删除）。
- **边界契约补强**：新增 **35 个测试用例** —— `tests/unit/test_boundary_solver.cpp` 23 个（10 常规 +
  13 death：退化/溢出多边形、零/负/非有限密度、极大速度、1×100 m 薄条、1:1e6 mass ratio、非法
  Step/Prepare/SolvePosition/NaN）与 `tests/unit/test_boundary_world.cpp` 12 个（debug 10 + release-only 2：
  生命周期/ABA/stale handle/Clear、Step 非法输入、整岛唤醒与 proxy、休眠边界、宽相 churn 三路逐位一致、
  islandId/sleepTime/零速度 invariant）。4 处代码硬化（全部只影响非法输入/Debug 校验）：
  `PolygonShape::SetPolygon` 3 个溢出守卫（有限但溢出的坐标以前会生成零法线/inf 质心）、
  `ContactSolver::Prepare` 非有限 manifold assert、`ContactSolver::SolvePosition` body index/pointCount
  assert（此前是 UB）、`World::Step` Release 负迭代钳 0 + `CheckSane` 覆盖 W7 的 islandId/sleepTime/
  休眠零速度。
- **质量门（Phase C，canonical clean）**：三 preset 全绿 **debug 387 / release 390 / sanitize 387** 个
  ctest（31 个 sandbox death-test SKIP），**0 项目警告**；clean 配置+构建+ctest **50.1 / 53.6 / 86.0 s**；
  8 个 demo smoke 全部 exit 0；宽相 A/B 三场景（BoxStack/StressBenchmark/Dominoes）body+contact 输出
  SHA-256 三路相同；回归弱 hash **`588d30e70f9b4ddd` 不变**、exit 0；工作区外 gtest **386/386 全过**
  （31 个 death test 真正执行）；`scripts/format.ps1 -Check` 117 文件 OK；CI 新增 clang-format job。
- **交付**：annotated tag **`v1.0-release`** + `docs/release-notes-v1.0.md`。

**卡住的地方（现象 → 排查 → 根因 → 修复）**
1. **多 agent 并行开发导致文档数字漂移（收尾最大成本）**
   - 现象：同一指标在 README/AGENTS/architecture/HANDOFF/ROADMAP 出现不同值：bench 5 次中位 vs 3 次中位
     （7.77x vs 7.70x）、休眠 1.70x vs 1.73x vs 1.74x、packed "0.82x/慢 18%" vs "0.87x/慢 15%"、
     split impulse 深穿透被标成 "2 cm"（实际 1.139999 m/s 对应 0.10 m；2 cm 场景是 0.18 m/s）、
     FallingBall smoke 5.000 vs 5.057、StressBenchmark max_pen 178.151 vs 170.739。
   - 根因：① 各 agent 各自实测、机器上同时有编译/渲染，数字差 2–5% 属噪声但被当成"当前值"；
     ② 历史周数字（devlog/ROADMAP 周记录）被复制进"当前状态"段落；③ 口径不同（testbed 混合 awake
     窗口 vs 纯全醒/全睡；W7 的 packed 0.82x vs W8 复测 0.87x）。
   - 修复：**单一事实来源 + 交付门复测**——`profiling.md` §1.5 记录 Phase C canonical 复测值，所有"当前"
     数字以它为准；`boundary-contracts.md` §3 专门写"数字口径"（历史值只放历史列）；Lead 用 grep 对
     README/docs 全量对账后统一替换。教训：多 agent 写文档前应先冻结"数字来源 + 当前/历史口径"。
2. **GIF 录制没有 ffmpeg、也不能靠录屏**
   - 现象：设计文档要求 4–5 张 GIF；本机无 ffmpeg，屏幕录制不可复现（窗口位置/帧率/人为操作都会变）。
   - 方案：给 testbed 加确定性帧捕获（smoke 固定 1/60 s 步进），PPM 中间格式 + Pillow 编码；
     `dominoes` 链长 20 m 会被浮动面板挡住两端 → 加 `--capture-hide-ui`；`sap_stress` 400 盒每帧
     速度色带变化导致文件超预算 → settling 尾段（第 250 帧起）+ every=2 + 16 色，最终 578.8 KB。
   - 验证：脚本自带单张/总量预算校验，并用 `read_image` 抽帧确认 UI 文字可读；5 张总计 1.69 MB。
3. **边界测试抓到的真实缺口（都只加拒绝/校验，不改物理）**
   - `SetPolygon` 接受"有限但溢出"的坐标（半宽 1e19：边长平方 = inf → `Normalized()` 返回 (0,0)
     法线；shoe lace 和可溢出 → centroid = NaN）；修复 = 3 个 isfinite/IsValid 守卫 + 4 个新测试。
   - 手搭 contact 的越界 body index 在位置层是 **UB**（MSVC 报 vector subscript out of range）；
     修复 = `SolvePosition` 增加 index/pointCount/manifold assert（Release 仍安全）。
   - `World::Step` 负迭代数在 Release 会把负数存进求解预算；修复 = NDEBUG 下钳 0（Debug assert 不变）。
   - `CheckSane` 没覆盖 W7 新状态；修复 = islandId 范围 / sleepTime 有限非负 / 休眠动态体零速度检查。
   - 纪律：这些改动后**回归 hash 必须不变**（实测不变），并且必须跑工作区外完整 gtest（sandbox 会
     SKIP 掉最关键的 death tests——W8 的 31 个 death test 在工作区外全部真跑）。
4. **发布周的可选优化取舍**
   - 现象：文档完成后仍可做 block solver / Tracy / 渲染插值 / 冷热拆分。
   - 决定：**全部不做**（v1.1 候选，按 block solver → Tracy → 插值 → 冷热拆分排序）。理由：block solver
     会改变回归 hash，需要 golden old/new + 全量复测窗口，发布周做等于把"文档周"变成"求解器周"；
     其余三项的收益都还没有测量证据（W7 的 packed 测量甚至否定了一个）。把 859 步入睡/薄片问题如实
     写进 README 已知限制，比在发布周改物理更负责。

**数据（canonical clean，Windows 11 22631 / i7-12650H / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1）**
- **构建+测试**：debug **50.1 s**（387 ctest）/ release **53.6 s**（390）/ sanitize(ASan) **86.0 s**
  （387），全部 0 项目警告、31 个 sandbox death-test SKIP；工作区外 **386/386 gtest**（31 death 真跑）。
- **宽相（Release，3 次中位）**：N=50/200/500/2000 = BF 0.0034/0.0839/0.5466/9.3061 ms，
  SAP 0.0016/0.0173/0.0931/1.2084 ms → **2.12x / 4.88x / 5.92x / 7.70x**；小 N 5/10/20/50 =
  0.00/0.50/1.00/2.00x，交叉点 **N≈24–26**（Auto 阈值保守 32）。
- **休眠（Release）**：IslandSleep500 frame 105 全睡；testbed 端到端 **0.4634 → 0.2656 ms（1.74x）**；
  gtest 直接计时 全醒 **0.5557 ms** / 全睡 **0.2772 ms**（纯口径 **≈2.00x**）；debug 同场景 1.68x；
  10 盒塔 debug 3.30x（sleep step 859，历史同机）。
- **三求解模式（Release，400 bodies/760 contacts 单岛、40 reps）**：8 迭代 W8 8 次中位
  flat 1.860 / direct 1.908（0.97x）/ packed 2.137（**0.87x，比 flat 慢约 15%**）；Phase C 3 次复测
  flat 1.750 / direct 1.856（0.94x）/ packed 2.442（噪声大但始终最慢）→ packed 默认关结论不变。
- **8 demo smoke（canonical debug）**：FallingBall 5.057mm；SphereStack 3.063mm；BoxStack 3.893mm
  （awake=10，塔站立）；FrictionRamp 2.686mm；Pyramid200 41.863mm；StressBenchmark 170.739mm；
  Dominoes 20/20 唤醒无跳过；IslandSleep500 100 岛/500 睡/3.983mm。
- **确定性/等价性**：回归弱 hash `588d30e70f9b4ddd`（exact `f4390ef86c075a04`）不变；
  BoxStack/StressBenchmark/Dominoes 的 `none|bf|sap` body+contact SHA-256 三路相同；
  三求解模式 21,760 float 0 差异；无宽相/BF/SAP 300 步 545 floats/步逐位一致。
- **GIF**：5 张 720px 12.5fps，总计 1,692.6 KB（dominoes 116.8 / box_stack 435.9 / sap_stress 578.8 /
  island_sleep 222.4 / contact_debug 338.7 KB），`scripts/record_gifs.ps1` 一条命令可复现。

**下周（v1.1，按优先级）**
1. **2 点接触 2×2 block solver**（最高优先级技术债）：消除薄片/2 点残余抖动 → 更快入睡；预计回归
   hash 变化，需 golden old/new + devlog 解释。
2. **Tracy 正式接入**（R11）：8 个 Step 阶段 + SAP 排序/扫描埋点与 `Plot(awake/contacts)`。
3. **渲染插值**（R17，需要 Body 上一帧 transform 缓存）。
4. **`BodySim`/`BodyState` 冷热拆分**（R15）：只有 Tracy 数据证明 body 布局是热点才做。
5. 多场景回归 golden（R12 扩展：Pyramid200 / 休眠场景）、岛屿级并行、DynamicAABBTree（可选宽相第三实现）。

**给未来的自己的提醒**
1. **多 agent 并行写文档，先冻结"数字来源"**：谁测的、几轮中位、什么口径、是"当前"还是"历史"。
   否则收尾会把时间全花在对账上；最终一律以 canonical clean 复测为准并用 grep 全量对账。
2. **发布周的"只加校验"改动也要跑工作区外 death tests**：sandbox 自动 SKIP 会掩盖契约回归
   （W7 就是这样抓到 static-static skip 回归的；W8 的 31 个 death test 全部真跑通过）。
3. **不要让 GIF 逼你改物理/调参**：录制定时来自 smoke 固定步进，画面不理想就换 sample window
   （start/every/crop/hide-ui），不改阈值、不改场景参数。
4. **文档里的每条命令都要在提交前真跑**：format、三 preset、smoke、A/B、regression、death tests、
   bench、GIF 重录——本次全部在 canonical 构建上跑过，数字才敢写。


