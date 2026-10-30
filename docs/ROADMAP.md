# Roadmap（8 周执行计划与验收标准）

> 依据设计文档第 5 章。每周结束：打 git tag、录 GIF、写 `docs/devlog.md` 周报、CI 全绿。
> 接口插入点见 [`dev-interfaces.md`](dev-interfaces.md)。

## Week 1 · 数学库 + CMake 骨架 + 渲染循环 ✅

| 日 | 任务 | DoD | 状态 |
| --- | --- | --- | --- |
| D1 | 仓库/CMake/CI 骨架 | `cmake --preset debug` + build + ctest 全绿；CI 首跑（本地已完成 configure/build/test；CI 待 push） | ✅ |
| D2 | `Vec2` | 三种 Cross + 全运算单测 + Eigen 交叉验证 | ✅ |
| D3 | `Mat2` / `Rot` | `A*A⁻¹≈I`；Rot 往返 1e-6；1e6 次乘法无漂移 | ✅ |
| D4 | `Transform` / `AABB` | 九宫格重叠、Extend 单调性 | ✅ |
| D5 | Testbed 骨架 | 窗口 + 旋转方块 + 圆 + ImGui + 相机 + 固定步长主循环 | ✅ |
| D6 | 积分骨架 | Body/IDPool/World；重力下落；重力滑条；IDPool 单测 | ✅ |
| D7 | 缓冲/复盘 | devlog、format、tag `v0.1-falling-ball` | ✅ |

关键交付：`physics2d_core`（零第三方依赖）、`physics2d_testbed`（--smoke 可自动化）、
`physics2d_bench` 占位、Week 2-8 预留接口与文档。

## Week 2 · 平面 + 第一次"停下来" ✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | Static/Kinematic/Dynamic 行为 | static 不受重力；三种类型 Step 行为正确 |
| D2 | 平面形状 + 解析穿透检测 | 打印穿透深度/法线 |
| D3 | 纯位置修正响应（临时） | 球落地不穿透；记录"落地即停"现象 |
| D4 | 圆-圆解析碰撞（只检测） | 相离/相切/相交；浮点边界无 NaN |
| D5 | Contact 持久化 + ContactID 初版 | 帧间可识别同一接触 |
| D6 | restitution 位置修正版（临时） | 写 devlog 记录动机 → 引出冲量法 |
| D7 | 复盘 + tag `v0.2-plane-contact` | — |

要填的预留点：R4 形状系统、R5 圆/平面解析路径、R7 `World::UpdateContacts`、R6 Contact 持久化。

**Week 2 验收结果（2026-10-28，tag `v0.2-plane-contact`）**
- 形状系统（Circle/Plane）+ World 形状池 + `AttachShape` 质量推导；解析碰撞 28 个测试通过；
  Contact/ContactID 帧间持久化（Key 稳定、`isPersistent` 正确）；`Contact` 迁移到 `Collision/`。
- 临时响应（位置推出 + 法向速度投影）使球稳定停在 y≈0.492（稳态穿透 0.00778 m = `kLinearSlop + g·dt²`）；
  e=0 不弹、e>0 近似反弹，行为与"为什么位置修正无法正确表达弹性碰撞"已写入 devlog。
- 单测 185（+65）；debug/release/sanitize 三 preset 全绿、零警告；testbed smoke `contacts=1`、exit 0。
- 遗留技术债（W3/W5 清理）：`World::SolvePositionConstraints` 内 `TEMPORARY` 位置推出与速度投影；
  O(N²) 配对枚举（W6 换 `IBroadPhase`）；圆-多边形碰撞（W4）。

## Week 3 · Sequential Impulse（只有法向） ✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 冲量施加 | `v=J/m`、`ω=Cross(r,J)/I` 单测 |
| D2 | 有效质量 K_n | 手算 3 组；K>0 |
| D3 | 单次法向冲量 | 二球正碰动量守恒（误差<1e-4） |
| D4 | 迭代求解 | 3 球链式传递；1→10 次收敛曲线 |
| D5 | Restitution + 速度阈值 | e=0.8 反弹高度≈3.2m；e=0 不弹 |
| D6 | Step 重排为 4.6 全序（缺宽相/摩擦/warm start） | 球-球、球-平面稳定响应；ASan 干净 |
| D7 | Demo SphereStack + tag `v0.3-impulse` | 3 球叠放能停住（允许抖） |

要填：R8 的 `WarmStart`（占位）/`SolveVelocityConstraints` 法向部分；`Solver/ContactSolver`。

**Week 3 验收结果（2026-10-28，tag `v0.3-impulse`）**
- `Solver/ContactSolver`（只法向）落地：K_n、累计钳制、迭代、restitution 速度阈值；World 的
  `PrepareConstraints`/`SolveVelocityConstraints` 接线，W2 临时速度投影删除（只剩位置推出等 W5）。
- 数据：1 次迭代残余 g·dt/2=0.0833 → 8 次 0.00065 m/s；静止穿透 5.000 mm = kLinearSlop；
  1:3 正碰动量误差 <1e-4；e=0.8 5m 反弹 `rebound/impact≈0.8`（峰值 3.55，W5 收紧）。
- Testbed `SphereStack`：3 球叠放 300 帧保持 3 个持续接触（y≈0.494/1.488/2.482，残余速度≤0.019 m/s），
  符合 D7"能停住、允许抖"。FallingBall 更新为真实冲量语义。
- 测试 210（Release 211，多出 release-only skip 契约测试）；三 preset 全绿、0 警告。
- 遗留技术债：`SolvePositionConstraints` 的位置推出（W5 split impulse）；O(N²) 配对（W6）；
  无摩擦/无 warm start/无 split impulse（W5）；多边形碰撞（W4）。

## Week 4 · SAT + Manifold 生成 ✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | PolygonShape 质量属性 | 质心/面积/惯性矩解析值对照；非法输入被拒 |
| D2 | SAT 主体 | 9 种位姿；分离返回 no-collision |
| D3 | reference/incident face | 交换 A/B 法线反向、深度不变 |
| D4 | Clipping | 面-面 2 点、顶点-面 1 点 |
| D5 | ContactID 特征稳定 | 旋转 100 帧 Key 不跳变 |
| D6 | SAT cache | 记录抖动对比 |
| D7 | BoxStack 初版 + tag `v0.4-sat` | — |

要填：R5 SAT/裁剪、R16 Key 对称映射、R15 冷热拆分（W4 末）。

**Week 4 验收结果（2026-10-28，tag `v0.4-sat`）**
- `PolygonShape`（严格校验/质量积分/SetAsBox/AABB）、`SAT`（FindMaxSeparation + reference/incident +
  裁剪式 manifold）、`Clipping`、`SATCache`、圆-多边形/多边形-平面解析路径全部落地；
  `ContactID.Key()` 改为不含 flip（A/B 交换 Key 集合相同）。
- World 接入 `CreatePolygonShape`/`CreateBoxShape` 与全形状组合窄相分派、SAT cache 跨帧持久化。
- 数据：盒-地面 2 接触点、pen 5.000mm；方块 0°→90° 旋转 181 帧 Key 只变 2 次；双盒 40 迭代稳定
  （gap=1.0000）；SAT cache 使 World 级 Key churn 4 vs 264（物理等价）。
- **BoxStack W4 基线（如实记录）**：10 盒 @默认 8 迭代 collapse step≈138、最大穿透 0.1413m——
  无摩擦 + 无 warm starting 时多层堆叠在物理上不稳定，这是 W5 `v0.5-stable-stack` 的 before 基线。
- 测试 278（15 sandbox skip）；三 preset 全绿、0 警告。
- 偏差：冷热拆分移到 W5 末（见 dev-interfaces 第 5 节）；`SetPolygon` 对近重复顶点是拒绝而非合并。

## Week 5 · 稳定堆叠（本项目最难的一周）✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 摩擦切向冲量 | 斜面 μ>tanθ 静止 / μ<tanθ 下滑 |
| D2 | 摩擦锥钳制 | μ=0 无切向冲量 |
| D3 | 冲量累计钳制 | 1000 帧后 accN≥0 |
| D4 | Warm Starting | warmFactor 0/0.5/0.85/1.0 对比表 |
| D5 | Split Impulse | 30 层堆叠穿透 < slop |
| D6 | ImGui 参数面板 + Contact 面板 | 实时生效不崩 |
| D7 | **里程碑**：10 方块堆叠 10s 稳定（穿透<0.01m）+ tag `v0.5-stable-stack` | 不达标停下查调试手册 |

要填：R8 剩余（摩擦锥/warm start/split impulse）、R15 冷热拆分（W4 移到 W5 末）、R18 参数暴露。

**Week 5 验收结果（2026-10-29，tag `v0.5-stable-stack`）**
- **里程碑达成**：10×1 m 方块塔、默认 8 速度迭代/3 位置迭代、600 步 = 10 s：最大穿透 **5.223 mm
  （< 10 mm）**、顶层 y 9.47813 m、最大倾斜 0.0046 rad、|x| 峰值 0.031 m、末帧最大速度 7.2 mm/s、无 NaN；
  testbed smoke 600 步 max_penetration **3.893 mm**、塔完整（11 bodies / 10 contacts / 20 points）。
- 摩擦：20 格斜面摩擦角扫描全过（μ>tanθ 12 格 5 s 位移 ≤1.5 mm；μ<tanθ 8 格下滑）；1000 帧斜堆叠
  `max|accT|/accN=0.5000`、锥外 violations=0；μ=0 时切向冲量恒 0。
- Warm starting：默认 factor **1.0**（W1 计划的 0.85 实测会让 10 盒塔在 10 s 内倒，见 dev-interfaces 偏差）；
  残差表精确等于手算 `0.5(1−f)g·dt`；`warmFactor=0` 与 `useWarmStarting=false` 逐位一致。
- Split impulse：`ContactSolverPosition.cpp` 伪速度层替换 W2 位置推出；真实速度逐位不变、
  伪速度每步清零；深穿透场景能量注入 on 0.000000 m/s vs Baumgarte off 1.139999 m/s。
- testbed：4 面板（Testbed Controls / Solver Settings / Contacts / Status）+ FrictionRamp demo；
  BoxStack/FallingBall/SphereStack 三个旧 demo 数据保持稳定（5.0 mm / 3.06 mm 穿透）。
- 测试 **301**（debug/sanitize；release 302）全绿；三 preset 全绿、0 警告；
  clean build+ctest：debug 31.8 s / release 41.0 s / sanitize 42.9 s。
- 偏差：`kWarmStartFactor` 0.85→1.0、`kSplitImpulseSlop` 0.01→kLinearSlop、
  `kPositionCorrectionFactor` 移除、`Prepare` 追加 dt 参数、冷热拆分二次顺延到 W6
  （全部登记在 `docs/dev-interfaces.md` 第 5 节）。

## Week 6 · Broad Phase 替换与量化 ✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | IBroadPhase + BruteForce | demo 行为完全不变 |
| D2 | SAP 端点数组 | 5 AABB 手工用例 |
| D3 | SAP 完整实现 | 与 BruteForce 配对集合完全一致 |
| D4 | 增量插入排序 | 与 std::sort 耗时对比 |
| D5 | Tracy + bench | 真实数字表格 |
| D6 | Pyramid200/StressBenchmark | 切换宽相物理结果一致 |
| D7 | headless 回归固化 + tag `v0.6-sap` | CI 新增 regression job |

要填：R1/R2/R3/R13，R12 回归工具，**R15 冷热拆分（W4→W5→W6→W7 三次顺延）**。

**Week 6 验收结果（2026-10-29，tag `v0.6-sap`）**
- **宽相双实现**：`BroadPhaseBruteForce`（O(N²) 参考基线）+ `BroadPhaseSAP`（单 vector 端点数组 +
  首帧 `std::sort` / 稳态冒泡增量排序 + 有序 active 集合 + 完整 AABB 重叠 + `ValidateInternalState`）。
  SAP vs BruteForce：10 组随机场景（N=50/200/500）× 1000 帧 = 10,000 帧逐帧配对集合完全一致，
  677,584 对比较 0 mismatch。
- **World 接线与等价性**：`Body::proxyId` + proxy↔body 映射、`UpdateBroadPhase` 候选对规范化为
  (i<j, sorted, deduped)、`Synchronize` fat AABB（+0.1m）、`UpdateContacts` 与 O(N²) fallback 共用
  `CreateContactForPair`。三路（无宽相 / BF / SAP）300 步**逐帧 545 floats 逐位一致**（含接触列表与
  累计冲量）；运行中切换宽相 150 步与参照 World 逐位一致；testbed 三场景 `--broadphase none|bf|sap`
  smoke 输出 SHA-256 完全相同。
- **bench 真实数字**（Release / seed 42 / 1000 帧）：N=50/200/500/2000 → BruteForce 0.0033/0.0838/
  0.5399/9.0985 ms/帧，SAP 0.0015/0.0169/0.0916/1.1656 ms/帧，**加速比 2.20x/4.96x/5.89x/7.81x**；
  小 N 交叉 N≈25–32（SAP 在 N=20 是 0.80x）；testbed Auto 阈值取 32。
- **headless 回归**：`physics2d_regression` 600 步弱 hash `588d30e70f9b4ddd`（quantum 1e-5）+
  golden + CTest `regression_default` + CI `regression` job；退出码 0/1/2 实测正确。
- **demo**：新增 Pyramid200（完美金字塔，默认 16 层=136 盒，4–24 可调；20 层=210 盒在 8 迭代下底部
  下沉 0.44m 的实测数据已记录）与 StressBenchmark（400 随机盒）；DebugUI Broad phase 下拉
  （Auto/None/BruteForce/SAP）+ candidate pairs/proxies 显示；Camera 自适应取景。
- 测试 **321**（debug/sanitize；release 322）全绿；三 preset 全绿、0 警告；
  clean build+ctest：debug 42.5s / release 47.1s / sanitize 66.1s；
  测试 exe 复制到工作区外 320/320 全过（15 个 death test 真正执行）。
- 偏差：`Body::aabb` 语义改为 fat AABB、`UpdateContacts` 保留 O(N²) fallback、SAP 懒批量排序 +
  mutable、`RemoveProxy` 用 erase、回归用 txt golden + 量化弱 hash（非设计的 CSV+脚本）、
  W6 无 `DetachShape`、bench "真实重叠"口径 = tight AABB、**冷热拆分三次顺延到 W7**
  （全部登记在 `docs/dev-interfaces.md` 第 5 节）。

## Week 7 · 岛屿与休眠 + 打磨 ✅

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 岛屿构建（DFS） | 3 岛/合并岛测试 |
| D2 | 按岛求解 | 与不按岛结果一致 |
| D3 | 休眠 | sleep 后状态不变；碰撞即醒 |
| D4 | 唤醒传播 | 多米诺连锁唤醒 |
| D5 | IslandSleep500 + Dominoes | 休眠前后耗时对比 |
| D6 | 暂停/单步/慢放 | 逐帧观察 manifold |
| D7 | Contact 调试面板完整版 + tag `v0.7-island-sleep` | — |

要填：R9/R10/R14/R17，**R15 冷热拆分（W4→W5→W6→W7→W8 顺延）**，R11 Tracy（可选）。

**Week 7 验收结果（2026-10-29，tag `v0.7-island-sleep`）**
- **岛屿（P13 D1）**：`Solver/Island.{h,cpp}` 显式栈 DFS + forward-star 邻接 + 规范排序；
  只有 dynamic-dynamic contact 是边（static/kinematic 是边界，不传递连通性）；3 独立堆 → 3 岛、
  桥接后 → 2 岛、500-body 长链 → 1 岛/499 contact 不溢出。
- **按岛求解（D2）**：三模式（flat / **grouped-direct（默认）** / grouped-packed（opt-in））**逐位一致**：
  两岛 276 次 + 密集单岛 21,760 次 float 比较 0 差异；World 级 300 步 × 469 floats/步 0 mismatch；
  **回归弱 hash 不变**。分组打包保留但默认关闭：Release 实测 packed 比 flat 慢 18%
  （400 bodies 的 Body 数组本就装得进 L2），默认 grouped-direct 与 flat 差 2–7%。
- **休眠/唤醒（D3/D4）**：整岛原子休眠（`UpdateSleep`）、休眠岛跳过 AABB 更新/窄相/求解但保留 proxy、
  双方休眠 contact 复用上一帧、宽相邻近唤醒（共享 wake pass，宽相 on/off 等价）；4 个外部唤醒入口
  整岛唤醒；休眠后 100 步状态与接触冲量**逐位不变**。
- **性能（D5，真实数字）**：IslandSleep500（500 盒/100 岛，Release 600 步）frame 105 (1.75 s) 全睡；
  Step **0.4554 ms → 0.2627 ms，加速 1.73x**（debug 4.95→2.94ms，1.69x）；10 盒塔 debug 3.30x
  （26.5 vs 87.5 µs/step，sleep step 859——残余抖动所致，阈值保持 0.01，block solver 为 W8 技术债）。
- **多米诺**：Release 600 步 woken_after_sleep 20/20、fallen 20/20、chain_reach 20、first_skipped=-1。
- testbed：Dominoes/IslandSleep500 demo、islands/awake/sleeping 面板、Space 暂停 / `.` 单步 /
  1/2/3 慢放、Contact 冲量柱状条与摩擦锥占用。
- 测试 **354**（debug/sanitize；release 355）全绿；三 preset 0 警告；
  clean build+ctest：debug 55.4s / release 53.5s / sanitize 82.9s；
  测试 exe 工作区外运行 353/353 gtest 全过（15 个 death test 真正执行）。
- 偏差：static/kinematic 不传递岛屿连通性（设计文档措辞歧义，按"3 独立堆=3 岛"解释）、
  proximity wake 从 UpdateBroadPhase 挪到 UpdateContacts 共享 pass、`packIslandBodies` 默认 false、
  休眠阈值不调大（sleep onset 859 步如实记录）、Dominoes 用 0.30×1.0 m 宽牌、R15 冷热拆分顺延到 W8
  （全部登记在 `docs/dev-interfaces.md` 第 5 节）。

## Week 8 · 文档、简历、面试准备 ✅

| 日 | 任务 | DoD | 状态 |
| --- | --- | --- | --- |
| D1 | 架构图升级 | 一张图讲清引擎 | ✅ |
| D2 | `docs/math.md` | 公式能手推 | ✅ |
| D3 | `docs/profiling.md` | 性能表 + 环境 | ✅ |
| D4 | README 完整版 | 陌生人 5 分钟能跑 | ✅ |
| D5 | 已知限制 + 边界测试 | 崩溃点都有 assert/文档 | ✅ |
| D6 | 简历 bullet + 面试演练 | 能闭卷讲 20 分钟 | ✅ |
| D7 | tag `v1.0-release` + Release | — | ✅ |

要填：R11 Tracy 完整接入（如时间允许）、R15 冷热拆分（**只有拿到 profiling 数据证明 body 内存布局是热点才做**；
W7 实测"岛内打包"在 400 体规模反而更慢，说明该规模下 body 数组本就 cache 友好）、
2 点接触 2×2 block solver（消除薄片残余抖动 → 更快入睡，W7 卡点 3 的正解）、W01-W04 文档提示词。

**Week 8 验收结果（2026-10-30，tag `v1.0-release`）**
- **文档（全部落盘并交叉链接）**：`docs/math.md`（9 节逐步推导 + 每节面试追问 + §10 边界契约表，~1040 行）；
  `docs/profiling.md`（宽相/休眠/三求解模式/构建耗时/8 demo smoke 全量实测 + 测量纪律 +
  §1.5 Phase C canonical 复测，~500 行）；`docs/architecture.md` 升级（§0 一张图总览、§3.7 数据所有权图、
  §4.6 Step 11 阶段时序图、§4.7 岛屿/休眠状态机，共 6 张 Mermaid）；`docs/boundary-contracts.md`
  （三类契约 + 50 行边界场景表 + 13 条已知限制，73 个测试名逐条机械核对）；`docs/demos.md`（8 demo/操作键/
  4 面板/GIF 录制）；`docs/resume-interview.md`（6 条简历 bullet + 25 问四段式 + 5 个专项深挖 + 20 分钟讲解
  + 数字速查）；README 重写为 v1.0 结构 + `README.en.md` 英文版 + `docs/release-notes-v1.0.md`；
  AGENTS/HANDOFF/ROADMAP/dev-interfaces/devlog/testing 全部更新到 v1.0。
- **GIF**：testbed 新增确定性帧捕获（`--capture-dir/--capture-every/--capture-start/--capture-hide-ui/
  --capture-display`，`glReadPixels(GL_BACK)` → PPM P6，默认关闭零开销）+ `scripts/record_gifs.ps1`
  （内嵌 Python+Pillow，本机无 ffmpeg）→ **5 张 GIF**（dominoes/box_stack/sap_stress/island_sleep/
  contact_debug，720px/12.5fps，总计 **1,692.6 KB**，单张 ≤600KB）。
- **已知限制/边界测试（设计文档 10 章 DoD 全覆盖）**：新增 **35 个测试**（solver 23：退化/溢出多边形、
  零/负/非有限密度、极大速度、1×100 m 薄条、1:1e6 mass ratio、非法 Step/Prepare/SolvePosition/NaN；
  Core 12：生命周期/ABA/stale handle/Clear、Step 非法输入、整岛唤醒与 proxy、休眠边界、宽相 churn
  三路逐位一致、islandId/sleepTime/零速度 invariant）；4 处代码硬化（PolygonShape 溢出守卫 ×3、
  ContactSolver/SolvePosition assert、World Release 负迭代钳 0 + CheckSane 覆盖 W7 状态）——
  只改非法输入行为/Debug 校验，**合法输入逐位不变**。
- **测试/三 preset（canonical clean）**：debug **387** / release **390** / sanitize **387** 个 ctest
  全绿、**0 项目警告**、31 个 sandbox death-test SKIP；clean 配置+构建+ctest
  **50.1 s / 53.6 s / 86.0 s**；工作区外 gtest **386/386 全过**（31 个 death test 真正执行）。
- **交付门证据**：8 个 demo smoke 全 exit 0（FallingBall 5.057mm、BoxStack 3.893mm、Pyramid200 41.863mm、
  StressBenchmark 170.739mm、Dominoes 20/20、IslandSleep500 100 岛/500 睡 3.983mm）；宽相 A/B 三场景
  body+contact SHA-256 三路相同；回归弱 hash **`588d30e70f9b4ddd` 不变**、exit 0；
  `scripts/format.ps1 -Check` 117 文件 OK；CI 新增 clang-format job。
- **性能（canonical Release）**：宽相 2000 体 **7.70x**（2.12x/4.88x/5.92x/7.70x）、小 N 交叉 N≈24–26；
  休眠 IslandSleep500 端到端 **0.4634 → 0.2656 ms（1.74x）**、纯全醒/全睡 **≈2.00x**；
  三求解模式 packed **0.87x（比 flat 慢约 15%）** 默认关闭；数字与口径全部见 `docs/profiling.md`。
- **可选优化（未做，登记 v1.1）**：2 点接触 2×2 block solver（最高优先级技术债，会改回归 hash，需
  golden old/new + 全量复测窗口）、Tracy 实接（R11）、渲染插值（R17）、BodySim/BodyState 冷热拆分（R15，
  须先有 profiling 数据证明 body 布局是热点）。理由见 `docs/dev-interfaces.md` §5 W8 偏差行。


## 降级路径（时间不够时按序砍）

1. 砍 DynamicAABBTree（保留 BruteForce + SAP 对比）；
2. 砍岛屿休眠（**必须**保留 200 物体 SAP 对比数据）；
3. 砍 Capsule 与 Dominoes；
4. 砍 CI Windows 矩阵；
5. 砍 split impulse 改 Baumgarte（README 写明理由）；
6. **绝不砍**：稳定堆叠、SAT+Manifold、warm starting、SAP 对比数据、单测。
