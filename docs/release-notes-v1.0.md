# Physics2D v1.0-release 发布说明

> 从零手写的 C++20 2D 刚体物理引擎，8 周（W1-W8）完成"宽相 → 窄相 → 接触流形 → Sequential Impulse
> 求解 → 岛屿/休眠"完整链路 + 量化、确定性回归与交互式测试台。本文件是 `v1.0-release` annotated tag
> 与 GitHub Release 的说明草稿；所有数字均可在本仓库用一条命令复现（见文末"验证清单"）。

## 一句话

**一个零第三方物理依赖、可 headless 回归、每一阶段都有实测数据的 2D 刚体物理引擎**：
10 方块塔 10 s 稳定（穿透 5.223 mm）、SAP 宽相 2000 物体 7.70x、整岛休眠 500 物体 1.74x、
切换宽相物理结果逐位一致、回归弱 hash 全程不变。

## 亮点

- **完整链路**：`Math → Core → Collision → Solver` 四层，`physics2d_core` 只用 C++ 标准库；
  `World::Step` 11 阶段固定流水线（积分力→宽相→窄相→建岛→warm start→速度求解→积分位置→位置求解→
  同步→休眠→校验）。
- **窄相**：圆/平面解析路径 + 凸多边形 **SAT**（reference/incident face + Sutherland–Hodgman 裁剪，
  面-面 2 接触点）+ 特征编码 `ContactID`（帧间稳定 → warm starting 前提）+ `SATCache` 轴滞回。
- **求解器**：Sequential Impulse（PGS）+ 累计冲量钳制 + **库仑摩擦锥** + **warm starting**
  （完整继承，实测 factor<1 会单调提前倒塌）+ **split impulse** 独立伪速度位置层
  （静置 **0.10 m 深穿透**箱真实速度注入 0.000000 m/s vs Baumgarte 1.139999 m/s）
  + restitution 速度阈值。
- **稳定堆叠里程碑**：10×1 m 方块塔、默认 8 速度迭代 / 3 位置迭代、600 步（10 s）稳定，
  最大穿透 **5.223 mm**（< 10 mm）、顶层 y 9.478 m、最大倾斜 0.0046 rad、末帧最大速度 7.2 mm/s。
- **宽相**：`BroadPhaseBruteForce`（O(N²) 参考基线）与 `BroadPhaseSAP`（单 vector 端点数组 +
  首帧 `std::sort` / 稳态冒泡增量排序 + 有序 active 集合 + 完整 AABB 二次过滤）；World 侧 fat AABB、
  proxy 生命周期、候选对规范化为 `(i<j, sorted, deduped)`。**2000 物体 7.70x**（N=50/200/500/2000：
  2.12x/4.88x/5.92x/7.70x；小 N 交叉 N≈24–26，Auto 阈值保守取 32）。
- **岛屿与休眠**：显式栈 DFS 分区（只有 dynamic-dynamic contact 是边）、规范序、三种按岛求解模式
  （flat / **grouped-direct 默认** / grouped-packed opt-in）**逐位一致**、**整岛原子休眠/唤醒**
  （4 个外部入口 + 宽相邻近唤醒）；休眠岛零工作但保留 proxy 与上一帧 contact。
  **IslandSleep500（500 盒 / 100 岛）frame 105 全睡，Step 0.4634 ms → 0.2656 ms（1.74x）**；
  纯"全醒 vs 全睡"口径约 **2.0x**；多米诺 **20/20 连锁唤醒无跳过**。
- **确定性回归**：`physics2d_regression` 重放固定 600 步场景，量化弱 hash（quantum 1e-5）
  **按 toolchain 的 golden** 一致（MSVC `588d30e70f9b4ddd` / GCC·Linux `001d846118f05125`）；W5-W8 的宽相切换、岛屿分组、休眠、W8 边界校验等全部改动后
  hash 不变。"换宽相不改物理"的证据：无宽相/BF/SAP 300 步**逐帧 545 floats 逐位一致**；SAP vs BF
  10 组场景 ×1000 帧 = 10,000 帧配对集合 0 mismatch（677,584 对比较）；testbed `none|bf|sap` 输出
  SHA-256 相同。
- **测试与工程**：GoogleTest（含 Eigen 交叉验证、death tests）+ 三 preset（Debug/Release/Sanitize）
  + GitHub Actions（Linux/Windows + sanitize + headless + regression + 分层守卫）；
  **debug/sanitize 387、release 390 个 ctest 全绿**（386 gtest，其中 31 个 death test 在沙箱 SKIP、
  工作区外真跑 386/386）、MSVC `/W4 /permissive-` 项目代码 **0 警告**；
  clean 构建+ctest：debug 50.1 s / release 53.6 s / sanitize 86.0 s。
- **交互式测试台**：GLFW + OpenGL 3.3 Core + Dear ImGui，8 个 demo、实时调参、宽相切换、
  岛屿/休眠状态、接触法线/穿透/累计冲量柱状条与摩擦锥占用、Space 暂停 / `.` 单步 / 1-2-3 慢放。
  W8 新增确定性帧捕获（`--capture-dir`，PPM→Pillow）→ **5 个 GIF** 一条命令可复现
  （`scripts/record_gifs.ps1`，总计 1.69 MB）。
- **文档与面试材料**：架构总图（模块依赖 + Step 时序 + 岛屿/休眠状态机 + 数据所有权）、
  `docs/math.md` 逐步推导、`docs/profiling.md` 全量实测表与测量纪律、
  `docs/boundary-contracts.md` 边界契约、`docs/demos.md` 演示/操作/录制指南、
  `docs/resume-interview.md` 简历 bullet + 25 问 + 20 分钟脚本；README 中英双语。

## v1.0 的边界（诚实清单）

- 30:1 单列塔在默认 8 速度迭代下会塌；16 层以上金字塔底部有超 10 mm 的载荷穿透（迭代预算属性，
  非 bug）。
- 10 盒塔需 ~859 步入睡；薄片（<0.2 m）在当前逐点 PGS 下几乎无法入睡 →
  **2 点接触 2×2 block solver 是 v1.1 最高优先级技术债**（不靠调大休眠阈值掩盖）。
- `Body` 冷热拆分未做（W8 profiling 无证据表明 body 布局是热点）；`packIslandBodies` 默认关闭
  （Release 实测 0.87x，比 flat 慢约 15%）。
- 无 CCD/TOI、无关节、无凹多边形/Capsule、无多线程；Tracy 实接与渲染插值推迟到 v1.1。
- 跨平台确定性靠量化弱 hash 判定，不保证 bit-exact。

## 升级 / 兼容性

- 公共 API 在 W1-W8 内**只追加不改签名**（冻结清单见 `docs/dev-interfaces.md` §1）；
  v1.0 没有物理行为变化（回归 hash 不变）。
- 依赖通过 CMake `FetchContent` 拉取（gtest/Eigen/GLFW/ImGui），无需 vcpkg。

## 验证清单（任选，全部可在本机复现）

```powershell
# 三 preset 构建 + 全量测试（0 警告）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -RunTests

# 确定性回归（MSVC 期望 hash=588d30e70f9b4ddd；Linux/GCC 为 default_600_gcc.txt、exit 0）
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# 宽相性能（Release；2000 物体 ~7.7x）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42

# 8 个 demo 冒烟 + 宽相 A/B + GIF 复现
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
```

完整手动验收脚本见 [`testing.md`](testing.md)；全部性能表见 [`profiling.md`](profiling.md)。

## 相关链接

- 架构：[`architecture.md`](architecture.md) · 数学推导：[`math.md`](math.md) ·
  性能：[`profiling.md`](profiling.md) · 边界契约：[`boundary-contracts.md`](boundary-contracts.md) ·
  演示与录制：[`demos.md`](demos.md) · 简历/面试：[`resume-interview.md`](resume-interview.md) ·
  开发日志：[`devlog.md`](devlog.md) · 交接：[`../HANDOFF.md`](../HANDOFF.md)
