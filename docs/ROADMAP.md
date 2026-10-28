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

## Week 3 · Sequential Impulse（只有法向）

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

## Week 4 · SAT + Manifold 生成

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

## Week 5 · 稳定堆叠（本项目最难的一周）

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 摩擦切向冲量 | 斜面 μ>tanθ 静止 / μ<tanθ 下滑 |
| D2 | 摩擦锥钳制 | μ=0 无切向冲量 |
| D3 | 冲量累计钳制 | 1000 帧后 accN≥0 |
| D4 | Warm Starting | warmFactor 0/0.5/0.85/1.0 对比表 |
| D5 | Split Impulse | 30 层堆叠穿透 < slop |
| D6 | ImGui 参数面板 + Contact 面板 | 实时生效不崩 |
| D7 | **里程碑**：10 方块堆叠 10s 稳定（穿透<0.01m）+ tag `v0.5-stable-stack` | 不达标停下查调试手册 |

要填：R8 全部、R18 参数暴露、R17 插值（可选）。

## Week 6 · Broad Phase 替换与量化

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | IBroadPhase + BruteForce | demo 行为完全不变 |
| D2 | SAP 端点数组 | 5 AABB 手工用例 |
| D3 | SAP 完整实现 | 与 BruteForce 配对集合完全一致 |
| D4 | 增量插入排序 | 与 std::sort 耗时对比 |
| D5 | Tracy + bench | 真实数字表格 |
| D6 | Pyramid200/StressBenchmark | 切换宽相物理结果一致 |
| D7 | headless 回归固化 + tag `v0.6-sap` | CI 新增 regression job |

要填：R1/R2/R3/R13，R12 回归工具。

## Week 7 · 岛屿与休眠 + 打磨

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 岛屿构建（DFS） | 3 岛/合并岛测试 |
| D2 | 按岛求解 | 与不按岛结果一致 |
| D3 | 休眠 | sleep 后状态不变；碰撞即醒 |
| D4 | 唤醒传播 | 多米诺连锁唤醒 |
| D5 | IslandSleep500 + Dominoes | 休眠前后耗时对比 |
| D6 | 暂停/单步/慢放 | 逐帧观察 manifold |
| D7 | Contact 调试面板完整版 + tag `v0.7-island-sleep` | — |

要填：R9/R10/R14/R17。

## Week 8 · 文档、简历、面试准备

| 日 | 任务 | DoD |
| --- | --- | --- |
| D1 | 架构图升级 | 一张图讲清引擎 |
| D2 | `docs/math.md` | 公式能手推 |
| D3 | `docs/profiling.md` | 性能表 + 环境 |
| D4 | README 完整版 | 陌生人 5 分钟能跑 |
| D5 | 已知限制 + 边界测试 | 崩溃点都有 assert/文档 |
| D6 | 简历 bullet + 面试演练 | 能闭卷讲 20 分钟 |
| D7 | tag `v1.0-release` + Release | — |

要填：R11 Tracy 完整接入（如时间允许）、W01-W04 文档提示词。

## 降级路径（时间不够时按序砍）

1. 砍 DynamicAABBTree（保留 BruteForce + SAP 对比）；
2. 砍岛屿休眠（**必须**保留 200 物体 SAP 对比数据）；
3. 砍 Capsule 与 Dominoes；
4. 砍 CI Windows 矩阵；
5. 砍 split impulse 改 Baumgarte（README 写明理由）；
6. **绝不砍**：稳定堆叠、SAT+Manifold、warm starting、SAP 对比数据、单测。
