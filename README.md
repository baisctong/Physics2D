# Physics2D — 从零手写的 2D 刚体物理引擎

> 一个用于校招简历与面试深挖的 C++20 项目：完整走通
> **宽相 → 窄相 → 接触流形 → Sequential Impulse 求解 → 岛屿休眠** 的链路，
> 物理核心零第三方依赖、可 headless 回归、用 Tracy 量化每一阶段耗时。

**当前进度：Week 1-5 完成**（数学库/积分骨架/渲染循环 + 形状系统与解析碰撞 +
**凸多边形 SAT + 裁剪式 Manifold + SATCache** + **摩擦锥/warm starting/split impulse 求解器** +
**10 方块堆叠 10 秒稳定里程碑**；Week 6-8 的接口已预留）。详细计划见
[`docs/ROADMAP.md`](docs/ROADMAP.md)，接口预留清单见 [`docs/dev-interfaces.md`](docs/dev-interfaces.md)。
**换会话/接手开发先读 [`HANDOFF.md`](HANDOFF.md)**（现状、验证基线、下一周任务卡、红线与坑）。

> 演示 GIF 将在后续周录制：10 方块稳定堆叠（已达成）/ 200 物体 SAP 对比 / 500 物体休眠。

## ✨ 已实现（Week 1-5）

- **数学库**（`p2::Math`）：`Vec2`（三种 `Cross` 重载）、`Mat2`、`Rot`（c/s 存储）、`Transform`、
  `AABB`（fat AABB 语义）、`MathUtils`（FNV-1a）。
- **Core**（`p2::Core`）：`Body`/`BodyDef`/`BodyType`、generation 计的 `Handle`+`IDPool`、
  11 阶段 `Step` 流水线、形状池（Circle/Plane/**Polygon**）、接触列表、固定步长 `StepAccumulator`、
  semi-implicit Euler、单步位移钳制、Debug `CheckSane`。
- **碰撞**（`p2::Collision`）：
  - 解析路径：圆-圆、圆-平面、**圆-多边形（Voronoi 最近特征）**、**多边形-平面（2 点支撑）**；
  - **凸多边形 SAT**：`FindMaxSeparation`、reference/incident face、Sutherland–Hodgman 式
    `ClipSegmentToLine`、面-面 2 点 / 顶点-面 1 点、A/B 交换对称；
  - **ContactID** 特征编码（`Key()` 与朝向无关，帧间稳定）；**SATCache** 轴滞回
    （近轴平局沿用上一帧 reference，`useSatCache` 可关）；`PolygonShape` 严格校验 + 质量积分。
- **求解器**（`p2::Solver`，Week 5 完整）：
  - **摩擦**：`t = Skew(n)`、`K_t`、**累计**切向冲量钳制 `accT ∈ [−μ·accN, +μ·accN]`（摩擦锥），
    法向先解、切向后解；
  - **Warm starting**：`ContactPoint` 累计冲量按 `ContactID::Key()` 跨帧继承，`Prepare` 播种、
    `WarmStart` 施加一次（默认 `warmStartFactor = 1.0`，Box2D 同款）；
  - **Split impulse**：独立伪速度位置层清理穿透，**不向真实速度注入能量**（W2 的临时位置推出已删除）；
  - restitution 速度阈值、Baumgarte 对比基线（`useSplitImpulse=false` 时）、`SolverSettings` 全参数运行时生效。
- **可交互测试台**：GLFW + OpenGL 3.3 Core + Dear ImGui，4 个 demo：
  `FallingBall`（落地+restitution+接触可视化）、`SphereStack`（3 球叠放）、
  `BoxStack`（10 层方块塔，**Week 5 起 10 s 稳定**，最大穿透 3.9 mm）、
  `FrictionRamp`（可调倾角/摩擦的斜面，实时 μ vs tanθ 判定）。
  面板：Solver 参数实时滑条、Contacts 调试（法线/穿透/累计冲量，点击高亮）、Status（Step 耗时等）；
  `--demo <name>` / `--smoke N --hidden` 支持自动化验证。
- **工程质量**：CMake presets（Debug/Release/Sanitize/Headless + VS2022 兜底）、GoogleTest
  （Eigen 交叉验证 + debug death tests）、ASan/UBSan、GitHub Actions CI（双平台+sanitize+headless+分层守卫）。

## 🚀 快速开始（Windows）

```powershell
# 1) 一次性环境检查（VS2022 C++ 工作负载、CMake、Git、Ninja）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/bootstrap.ps1

# 2) 配置 + 编译 + 跑单测
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 3) 启动测试台（下拉框切换 FallingBall / SphereStack / BoxStack）
.\build\debug\testbed\physics2d_testbed.exe
```

Linux：

```bash
sudo apt install -y build-essential cmake ninja-build libgl1-mesa-dev xorg-dev
./scripts/build.sh debug --tests
```

> 依赖（GoogleTest/Eigen/GLFW/Dear ImGui）由 CMake `FetchContent` 在首次 configure 时拉取，
> 不需要 vcpkg。详见 [`docs/dev-setup.md`](docs/dev-setup.md)。

## 🏗️ 架构

```mermaid
flowchart LR
    subgraph L1["physics2d_core（零第三方依赖）"]
        M["Math"] --> C["Core"]
        C --> Col["Collision：解析路径 + SAT/裁剪 + SATCache"]
        Col --> S["Solver：摩擦锥 + warm starting + split impulse（W5）"]
    end
    TB["testbed（GLFW + ImGui + OpenGL）"] --> C
    T["tests（GoogleTest）"] --> C
    B["bench（headless）"] --> C
```

- **分层铁律**：`physics2d_core` 不 include 任何渲染/UI 头（Tracy 例外），CI 有 grep 守卫；
  `World` 只前向声明 `ContactSolver`，Core 公共头不会把 Solver 拖进编译单元。
- 正式文档：[`docs/architecture.md`](docs/architecture.md)。

## 📁 目录

```text
include/physics2d/{Math,Core,Collision,Solver}/   # 公共头
src/{Math,Core,Collision,Solver}/                 # Week 1-5 实现（core 静态库）
testbed/                                          # 渲染 + ImGui 测试台 + demos
tests/unit/                                       # GoogleTest 单测；tests/regression/ 为 W6-7 预留
bench/                                            # headless 基准（W6 预留）
docs/                                             # 架构、接口预留清单、环境、开发日志、Roadmap
scripts/                                          # bootstrap / build / format / pre-commit
```

## 📊 测试与性能

- 单测：**301 个 ctest 用例**（Release 302，含 1 个 release-only 契约测试；其中 15 个 death test
  在沙箱内自动 SKIP、CI 执行）。`debug` / `release` / `sanitize`（MSVC ASan）三套 preset 全绿，
  MSVC `/W4 /permissive-` 项目代码零警告。clean 配置+构建+ctest：debug 31.8s / release 41.0s /
  sanitize(ASan) 42.9s（i7-12650H，含依赖编译）。
- **Week 5 里程碑（10×1 m 方块塔，默认 8 速度迭代/3 位置迭代，10 s = 600 步）**：
  **最大穿透 5.223 mm < 10 mm**、顶层 y 9.478 m、最大倾斜 0.0046 rad、末帧最大速度 7.2 mm/s、
  10 个持续接触（20 点）、无 NaN；两个相同 World 的 343 个状态 float 逐位一致（确定性）。
  testbed smoke 600 步：max_penetration 3.893 mm、塔完整。
- 关键实测（详细条件见 `docs/devlog.md`）：
  - 摩擦角扫描（μ∈{0.1,0.3,0.5,0.8,1.2} × θ∈{5°,15°,30°,45°} 共 20 格）：μ>tanθ 的 12 格
    5 s 位移 ≤1.5 mm，μ<tanθ 的 8 格明显下滑；1000 帧斜堆叠摩擦锥 `max|accT|/accN=0.5000`、0 violation；
  - warm starting 因子（10 盒 600 步，倒塌步数）：0→258、0.5→314、0.85→259、0.9→283、0.95→332、
    **1.0→不塌** —— 部分 warm start 每帧丢弃已收敛解、重新注入 PGS 顺序偏差（详见 devlog W5）；
  - split impulse 能量注入（静置箱 2 cm 初始重叠）：开 **0.000000 m/s** vs 关（Baumgarte）**1.139999 m/s**；
  - 摩擦锥与动量/能量：warm start 残差表精确等于手算 `0.5(1−f)g·dt`；`warmStartFactor=0` 与
    `useWarmStarting=false` 逐位一致；
  - e=0.8 从 5 m 落：apex 3.4633 m、rebound/impact = 0.8140（离散冲击速度偏差，非位置层注入能量）；
  - ContactID 稳定性：方块 0°→90°（181 帧）Key 集合只变 2 次；SAT cache Key churn 4 vs 264；
  - BoxStack W4 基线（before）：10 盒 @默认 8 迭代 collapse step≈138、最大穿透 0.1413 m
    —— 无摩擦 + 无 warm start 时多层堆叠物理上不稳定；W5 用真实机制修复，不是调迭代数。
- Testbed smoke：
  - `--demo BoxStack --smoke 600 --hidden` → `bodies=11 contacts=10 points=20 max_pen=3.893mm`，塔站立；
  - `--demo FallingBall --smoke 150 --hidden` → `contacts=1`，球停在 y=0.495（穿透 5.000mm）；
  - `--demo SphereStack --smoke 300 --hidden` → `bodies=4 contacts=3`，3 球站立（穿透 3.063mm）；
  - `--demo FrictionRamp --smoke 300 --hidden` → θ=15°、μ=0.30>tanθ=0.268，方块保持静止。
- 性能数据表（SAP vs BruteForce、休眠、Tracy 数字）将在 Week 6-8 用实测数字填入
  `docs/profiling.md` 与 README —— 本仓库不写没有测量条件的数字。

## 🗺️ Roadmap

| 周 | 主题 | 状态 |
| --- | --- | --- |
| W1 | 数学库 + CMake + 渲染循环 + 积分骨架 | ✅ |
| W2 | 形状/平面/圆解析碰撞 + Contact 持久化 | ✅ |
| W3 | Sequential Impulse（法向）+ SphereStack | ✅ |
| W4 | SAT + 裁剪式 Manifold + SATCache + BoxStack 初版 | ✅ |
| W5 | 摩擦/warm starting/split impulse + **10 方块堆叠 10 s 稳定**（tag `v0.5-stable-stack`） | ✅ |
| W6 | Broad Phase（BruteForce + SAP）与量化 + 冷热拆分 | ⬜ |
| W7 | 岛屿休眠 + demo 集 + headless 回归 | ⬜ |
| W8 | 文档/性能表/面试准备 | ⬜ |

完整验收标准见 [`docs/ROADMAP.md`](docs/ROADMAP.md)。

## ⚠️ 已知限制（Week 1-5）

- **30:1 单列高塔在默认 8 速度迭代下会塌**：PGS 每帧要沿 30 层传递支撑，8 次迭代的传播预算不足
  （10 盒是 8 迭代的验收场景；30 盒需要 ~32 次速度迭代才能站住）。这是迭代预算限制，不是求解器 bug。
- 宽相仍是 O(N²) + AABB 过滤（Week 6 换 SAP/BruteForce + 基准数据）。
- `Body` 暂为单一结构（内含冷/热分区），冷热拆分顺延到 Week 6（W4→W5→W6，见偏差登记）。
- 碰撞形状只有凸多边形（≤8 顶点）/圆/无限平面；不支持凹多边形（需拆分）、CCD。
- 测试台需要可用的 OpenGL 上下文；无显示环境用 `--smoke`。
- Tracy 剖析仅保留零成本钩子，正式接入在 Week 6-8。

## 📚 参考

Box2D（Erin Catto）、*Iterative Dynamics with Temporal Coherence* (GDC 2005)、
*Real-Time Collision Detection* (Christer Ericson)、Randy Gaul 的 impulse solver 系列文章。

## License

[MIT](LICENSE)
