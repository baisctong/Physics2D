# 性能剖析与量化数据（Profiling）

> **Week 8 / v1.0-release**。本页汇总 **Physics2D v1.0 全部可引用的性能数字**：宽相 A/B、岛屿休眠、
> 三种按岛求解模式、构建/测试耗时、8 个 demo 的 smoke 数据，以及测量环境与测量纪律。
>
> **本页没有任何"抄来的旧数字"**：所有表格都是 2026-10-30 在本机一次集中测量会话里用 Release 构建
> 实测的（第 3 节 10 盒塔的 debug 数据明确标注为 Week 7 历史实测引用，仅作量级参考）。每条命令可直接
> 复制复现；本页数字与仓库里的 `physics2d_bench`、testbed smoke、gtest 打印一一对应。
>
> 复现方式：先按 [`docs/testing.md`](testing.md) 构建 Release，再执行各节命令。测量机器同时服务多个
> 并行开发的 agent，**最终对外数字以 Lead Phase C 的 canonical clean Release 复测为准**；本页第 1.2
> 节记录了本次会话的并发情况。

---

## 0. 结论摘要（TL;DR）

| # | 结论 | 实测数字 |
| --- | --- | --- |
| 1 | **SAP 宽相在大 N 下大幅领先 BruteForce** | 2000 物体 9.29 → **1.22 ms/帧（7.77x）**；500 → 5.91x |
| 2 | **小 N 下 SAP 反而更慢**（端点维护常数），交叉点实测 | N≈22–26 之间打平，N=20 SAP 0.80x；testbed Auto 阈值 32 保守 |
| 3 | **整岛休眠让静止场景变便宜** | IslandSleep500（500 盒/100 岛）：frame 105 全睡；testbed awake 窗口 0.4540 → 全睡 0.2671 ms（**1.70x**），gtest 中位 0.4646 → 0.2675 ms（1.72x；两轮会话 1.64–1.74x） |
| 4 | **全醒 vs 全睡的真实差距约 2x** | 永不休眠稳态 0.540 ms vs 全睡 0.267–0.277 ms（≈1.98–2.02x）；1.64–1.74x 是 testbed 的"awake 窗口"口径（该窗口内已有个别岛先睡，见 §3.2） |
| 5 | **按岛打包（grouped-packed）没有收益，默认关闭** | 400 bodies/760 contacts、8 迭代（8 次运行中位）：flat 1.860 / grouped-direct 1.908（0.97x，噪声内）/ packed 2.137 ms（**0.87x**）；32 迭代 7.056 / 7.570（0.93x）/ 7.928（0.89x） |
| 6 | **交付质量**：三 preset 0 警告、全绿、回归 hash 不变、换宽相物理逐位一致 | debug 387 / release 390 / sanitize 387 个 ctest 全过；弱 hash `588d30e70f9b4ddd`；`none/bf/sap` body+contact 输出 SHA-256 相同 |

---

## 1. 测量环境与测量纪律

### 1.1 环境（每张表都适用）

| 项 | 值 |
| --- | --- |
| OS | Windows 11（NT 10.0.22631，22631） |
| CPU | 12th Gen Intel Core i7-12650H，10 核 / 16 线程 |
| 内存 | 15.7 GB（16 GB） |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU（驱动 617.42，GL 3.3.0；testbed smoke 实测） |
| 编译器 | MSVC 19.44（VS2022 14.44.35207），`/W4 /permissive-`，项目代码 0 警告 |
| 构建 | CMake 3.31.12 + Ninja 1.12.1，`RelWithDebInfo`（`-Preset release`；下文称 Release）、`-Preset debug`、`-Preset sanitize`（MSVC ASan，Debug） |
| 电源 | 交流供电（Win32_Battery BatteryStatus = 2 = 接电），非电池模式 |
| 测量会话 | 2026-10-30 20:28–20:41（UTC+8，两轮），仓库 `<repo-root>` |
| 本次使用的构建目录 | `build/w8corerel`（release）、`build/w8core`（debug）、`build/w8core-san`（sanitize）——与 canonical `build/release\|debug\|sanitize` 同 preset、同源码 |

### 1.2 本次测量会话的并发情况（诚实记录）

Week 8 是多个 agent 并行开发（Lead/`solver-dev`/`core-dev`/`testbed-dev` 共用同一个工作区）。本会话
测量分两轮，以**最终源码状态**那一轮为准：

- **第一轮 20:28–20:31**：bench / 三模式 / 休眠 / Release smoke 的初测；
- **第二轮 20:39–20:41**：等 partner 对 `src/Solver`、`src/Core`、`testbed/` 的最后修改（20:16–20:37）
  落地后，重建 Release/Debug 并重测 §3/§4/§6 全部受影响的数字（§3/§4/§6 采用这一轮）；
- §2 的 bench 只依赖宽相代码（partner 未改动），两轮同源；§7 的证据在最终 debug 二进制上复核。

具体并发情况：

- **bench 大 N**：5 次运行，其中 3 次前后都没有其他编译进程、另外 2 次邻近有另一个 agent 的
  testbed/GIF 录制进程；取 5 次**中位数**，数字与 3 次纯静默运行一致（差异 <3%）；
- **三模式计时**：5 次运行，4 次全程静默、1 次旁边有 testbed 渲染进程；取中位数（逐次波动见 §4）；
- **休眠 gtest 计时**：3 次运行全程无其他编译/渲染进程（`QUIET` 状态）；
- **testbed smoke**：逐个串行执行；debug smoke 期间偶有另一个 testbed 渲染进程，因此
  `avg_frame_ms` 只作参考（它含 GPU 渲染时间），**物理字段（body/contact/max_pen）不受影响**；
- **构建耗时**：三个 preset 各自在新的隔离目录里 clean 配置+构建+ctest，首次配置包含 FetchContent
  下载；机器同时服务其他 teammate 的任务，所以耗时是**参考值**，canonical 数字由 Lead Phase C 复测；
- **测试数在本会话期间继续增长**（其他 teammate 的边界测试并行落地）：例如 debug 的 ctest 用例数
  从 364 → 387。各表标注了该次测量时的用例数，**最终以 `v1.0-release` tag 时的 canonical 数字为准**。

### 1.3 测量纪律（为什么这些数字可信）

1. **只有 Release 数字可引用**：Debug 的绝对耗时无意义（断言/无内联），本页 Debug 数据只用于功能
   smoke；ASan（sanitize）数字更不可引用。bench 会打印 build type，Release 才被当作基准。
2. **预热丢弃**：bench 前 100 帧为 warm-up（SAP 首次 `std::sort`、CPU cache 冷），不计入；休眠场景
   testbed/gtest 的统计窗口从第 10/30 帧之后开始。
3. **中位数**：bench 每帧样本取 median；跨运行取 3–5 次运行的中位数（报告区间）；三模式计时取 40 reps
   总耗时再跨 5 次运行取中位。
4. **固定场景/种子**：bench `--seed 42`；两种宽相在同一场景、同一帧序列上比较；休眠/求解模式场景
   由代码确定（无随机）。
5. **只测该测的东西**：bench 只计 `UpdateAABB + QueryPairs`（不跑物理，所以本页宽相表**没有**
   "Step 总耗时"列，完整 Step 数据见 §3/§4/§6）；三模式计时只计 `SolveVelocity` 迭代（不含宽相/
   窄相）。
6. **口径写清楚**：候选对 = `QueryPairs` 输出（fat AABB）；"真实重叠" = 两紧 AABB 也重叠的子集
   （差值 = fat margin 的假阳性成本）。
7. **同时报告绝对值与相对值**：本页所有加速比都带两侧绝对 ms，避免"基线 0.1 ms 所以 5x 无意义"。
8. **不可用 Debug 复现 Release 数字**：需要复现量级时请用 `-Preset release`，并在干净构建上跑。

### 1.4 复现命令（仓库根目录，Windows PowerShell 5.1）

```powershell
# 构建（canonical；本文数字来自等价的隔离目录，把 build/release 换成 build/w8corerel 即可）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release -RunTests

# 宽相大 N（本页 §2.1：跑 3-5 次取中位）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42

# 宽相小 N 反转 + 交叉点（§2.2）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 5,10,20,50 --frames 500 --seed 42
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 22,24,26,28,30,32,36,40,44 --frames 500 --seed 42

# 三求解模式计时（§4；测试内打印 [solver7] 行，跑 3-5 次取中位）
.\build\release\tests\physics2d_tests.exe --gtest_filter=SolverIslandTest.SolveModeTimingComparisonIsPrinted --gtest_color=no

# 休眠 on/off 直接计时（§3.2；测试内打印 [boundary8] islandsleep500 行）
.\build\release\tests\physics2d_tests.exe --gtest_filter=WorldBoundaryProfilingTest.SleepingOnVsOffStepTimingIsPrinted --gtest_color=no

# IslandSleep500 端到端 smoke（§3.1；带 demo 自己统计的 awake/asleep 窗口）
.\build\release\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden

# 8 个 demo 的功能 smoke（§6；Debug 即可）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 600 --hidden
# … 其余 7 个 demo 同格式，见 §6 表格
```

> 说明：`build.ps1` 会在 header 比 obj 新时自动 `--clean-first`（本机中文 MSVC 的 `/showIncludes`
  使 Ninja depfile 为空，见 [`docs/dev-setup.md`](dev-setup.md) §6）；周验收一律 clean build。

### 1.5 Phase C canonical 复测（Lead，2026-10-30 20:42–20:51，交付门）

在 canonical `build/{debug,release,sanitize}` 上**删目录 clean 重编**（`_deps` 一并删除，含 FetchContent），
对本文各表逐项复核。数字若与 §2–§8 的测量会话有 ±2–5% 差异即视为噪声；**对外引用以本节 canonical
值为准**（§2–§8 的表格保留测量会话原始值，便于对照）。

| 项 | §2–§8（core-dev 测量会话） | Phase C canonical（本次） | 结论 |
| --- | --- | --- | --- |
| debug clean build+ctest | 80.3 s（387 用例） | **50.1 s**（387 用例、31 skip、0 警告） | 并发/网络差异，canonical 为准 |
| release clean build+ctest | 64.1 s（390 用例） | **53.6 s**（390 用例、31 skip、0 警告） | 同上 |
| sanitize clean build+ctest | 110.1 s（387 用例） | **86.0 s**（387 用例、31 skip、0 警告） | 同上 |
| bench N=50/200/500/2000 加速比 | 2.12x / 5.00x / 5.91x / 7.77x（5 次中位） | **2.12x / 4.88x / 5.92x / 7.70x**（3 次中位） | 噪声内一致 |
| bench 绝对 ms（BF / SAP，2000 体） | 9.2895 / 1.2159（7.77x） | **9.3061 / 1.2084**（7.70x） | 噪声内一致 |
| 小 N 交叉点 | N≈22–26 | N=22/24 ≈1.00x、26 = 1.17x、50 = 2.00x | 一致（交叉点 N≈24–26） |
| IslandSleep500 端到端（testbed smoke） | 0.4540 → 0.2671 ms（1.70x） | **0.4634 → 0.2656 ms（1.74x）** | 同一混合窗口口径，噪声内一致 |
| 休眠 on/off 直接计时（Release gtest） | 全醒 0.5400 / 全睡 0.2675（≈2.02x） | **全醒 0.5557 / 全睡 0.2772（≈2.00x）** | 一致 |
| 三模式 8 迭代（flat / direct / packed） | 1.860 / 1.908（0.97x）/ 2.137（0.87x） | **1.750 / 1.856（0.94x）/ 2.442\***（0.72x） | \*packed 单次 2.13–2.54 波动大；direct≈flat、packed 最慢的结论不变 |
| 8 demo smoke | §6 表 | **8 个 demo 全部 exit 0，字段与 §6 表逐项一致**（FallingBall 5.057mm、BoxStack 3.893mm、Pyramid200 41.863mm、StressBenchmark 170.739mm、Dominoes 20/20、IslandSleep500 100 岛/500 睡） | 一致 |
| 宽相 A/B SHA-256（BoxStack 300 步） | `E74DC08…`（none/bf/sap 相同） | **`E74DC08335693F03…` 三者相同**；另测 StressBenchmark `FBB723E8C43778DE`、Dominoes `F1267B04DF3BB637` 也都三路一致 | 一致 |
| 回归弱 hash / exit code | `588d30e70f9b4ddd` / 0 | **同值 / exit 0**（exact `f4390ef86c075a04`） | 不变 |
| 工作区外 death tests | 386/386（31 个真跑） | **386/386 passed、exit 0** | 一致 |
| GIF（§4 的 `docs/images/`） | — | 用 canonical `build/release` testbed 重录 5 张（总计 1,692.6 KB） | 见 `docs/demos.md` |

---

## 2. 宽相：BruteForce vs SAP

命令（§1.4 第一条），机器可读的完整表由 bench 自己打印（含环境、seed、候选对口径）。物理**不参与**
计时：每帧只做"移动盒子 → 刷新 fat AABB → `QueryPairs`"。

### 2.1 大 N（N 个随机盒、1000 帧、seed 42，5 次运行取中位数）

| Bodies | BruteForce ms/帧（中位，范围） | SAP ms/帧（中位，范围） | 加速比（BF/SAP，逐次比值中位） | 候选对/帧 | 真实重叠/帧 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 50 | 0.0034（0.0032–0.0039） | 0.0016（0.0015–0.0018） | **2.12x** | 3.4 | 1.8 |
| 200 | 0.0837（0.0813–0.0891） | 0.0165（0.0160–0.0182） | **5.00x** | 55.3 | 31.3 |
| 500 | 0.5333（0.5301–0.6377） | 0.0934（0.0883–0.1074） | **5.91x** | 354.3 | 202.5 |
| 2000 | 9.2895（9.1695–10.6520） | 1.2159（1.1804–1.2386） | **7.77x** | 5670.6 | 3206.6 |

- 候选对/帧、真实重叠/帧与 W6 的实测一致（场景固定、seed 固定，属于预期）。
- N=2000 的 BruteForce 单次波动较大（9.17–10.65 ms，±8%）：单帧 9 ms 级别的 O(N²) 扫描对 CPU 频率
  很敏感；SAP 侧仅 1.18–1.24 ms，稳定得多。
- **结论**：加速比随 N 单调上升（2.12x → 7.77x）——SAP 的增量排序把 O(N²) 配对降到接近
  O(N log N + pairs)，这正是选它做主线宽相的理由。

### 2.2 小 N 反转与交叉点（N=5/10/20/50，500 帧；交叉点细化 3 次运行）

SAP 每个 proxy 维护两个端点（插入排序/冒泡），常数开销大于小 N 的双重循环：

| Bodies | BruteForce ms/帧 | SAP ms/帧 | BF/SAP |
| ---: | ---: | ---: | ---: |
| 5 | 0.0000 | 0.0001 | **0.00x**（SAP 更慢） |
| 10 | 0.0001 | 0.0002 | **0.50x** |
| 20 | 0.0004 | 0.0005 | **0.80x** |
| 50 | 0.0035 | 0.0016 | **2.19x** |

细化扫描（3 次运行，逐次比值基本一致）：

| Bodies | 22 | 24 | 26 | 28 | 30 | 32 | 36 | 40 | 44 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| BF/SAP | 1.00x | 1.00x | 1.17x | 1.14x | 1.25–1.29x | 1.38–1.50x | 1.40–1.50x | 1.67–1.75x | 1.83–1.85x |

- **交叉点：本次实测 N≈22–26**（N=20 时 SAP 仍慢 20%，N=26 起 SAP 稳定更快）。W6 记录的是
  "N≈25–32"，本次细化把交叉点定位到更小的一侧；量级一致、方向一致。
- testbed 的 Auto 阈值取自该实测并**保守取 32**（[`docs/architecture.md`](architecture.md) /
  `World` 注释）：小场景走 BruteForce 不慢，大场景走 SAP 更快，阈值落在交叉点右侧避免抖动。
- 5/10/20 的绝对耗时是 **0.1–0.5 µs/帧**量级，已接近计时器噪声（`steady_clock` 分辨率、函数调用
  开销），**只看趋势，不要引用绝对值**。

### 2.3 口径与解读

- bench 的世界是 30×30 m、半边长 0.1–0.5 m、速度 ≤2 m/s 的随机盒，边界反弹保持密度均匀；
  proxy 用与 `World::Synchronize` 相同的 fat AABB（tight + `kAabbMargin=0.1 m`）；
- **候选对**是 `QueryPairs` 的输出（含 fat margin 假阳性）；**真实重叠**是两紧 AABB 也重叠的子集。
  两者差值就是窄相要多面对的"假阳性成本"，本机约 40–47%（如 N=2000：5671 vs 3207/帧）；
- 计时区域只含 `UpdateAABB + QueryPairs`（回调里只做计数）。真实应用中回调要做窄相/建接触，
  两者同口径对比不受影响；
- 换宽相**不得改变物理**（W6 契约）：候选对在 World 内被规范化为 `(i<j, sorted, deduped)` 的
  body-index 对，因此接触顺序、PGS 求解顺序、浮点结果都不变。端到端验证见 §7.2。

---

## 3. 岛屿休眠（IslandSleep500）

场景：静态地面 + **100 座 5 层塔 × 1 m 盒 = 500 个动态体**（横向 pitch 1.25 m，每座塔独立成岛），
默认求解器设置（8/3 迭代、warm start 1.0、split impulse、SAP 宽相），固定 dt=1/60。

### 3.1 端到端（testbed Release smoke，600 步 = 10 s）

```powershell
.\build\release\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 600 --hidden
```

| 指标 | 实测（本次会话两次 Release smoke 一致） |
| --- | --- |
| 全睡时刻 | **frame 105（t = 1.750 s）** |
| 物理状态（第 600 步） | bodies=501、islands=100、awake=0、sleeping=500、contacts=500、contact_points=1000 |
| 最大穿透 | 3.983 mm（< kLinearSlop 5 mm） |
| 残留速度 / 最大倾斜 | 0.000 m/s / 0.18° |
| 宽相 | SAP、candidate_pairs=900/帧、proxies=501（休眠后仍保留） |
| **awake 窗口平均 Step** | **0.4540 ms**（frames 11–104，94 帧；另一轮 0.4566） |
| **全睡后平均 Step** | **0.2671 ms**（frames 105–600，496 帧；另一轮 0.2704） |
| **加速比** | **1.70x**（两轮 1.69–1.70x；gtest 直接计时 3 次中位 1.72x，两轮会话 1.64–1.74x） |
| testbed avg_frame_ms | 2.55 ms（**含 GPU 渲染**，不是物理耗时，勿引用） |

> 这正是 README/周报里引用的"500 物体休眠后 ≈1.7x"的复测值（Week 7 历史为 1.73x，同机；差异来自
> 测量窗口与机器负载，量级一致）。

### 3.2 "awake 窗口"的实测澄清（重要，面试可讲）

testbed 的 `awake` 窗口是"**还没全睡的阶段**"，不是"100 个岛全都醒着"：岛是独立睡眠的，实测它们
从 frame≈39 起**陆续入睡**。用一个一次性探针（不提交，`build/w8probe/`，链接 `physics2d_core.lib`）
按帧打印 `GetSleepingBodyCount()`：

| 帧 | 60 | 90 | 104 | 105 |
| --- | ---: | ---: | ---: | ---: |
| 已睡盒数（/500） | **90** | **495** | **495** | 500 |

因此 `awake` 窗口的平均耗时被"已经睡着的岛"稀释了。把同一场景设为 `enableSleeping=false`（永不
休眠）做对照，用 gtest 直接 `steady_clock` 包住 `World::Step`（最终源码轮三次运行中位；另一轮
会话的值放在括注里）：

| 配置 / 窗口 | 平均 Step | 说明 |
| --- | ---: | --- |
| 休眠开：awake 窗口（frames 11–104） | 0.4646 ms | testbed 口径（含已入睡的岛）；另一轮 0.4530 |
| 休眠开：全睡后（frames 105–600） | **0.2675 ms** | 休眠岛零工作；另一轮 0.2773 |
| 休眠关：600 帧全程（全醒稳态） | **0.5400 ms** | 永不睡，真实全醒成本；另一轮 0.5481 |
| **全醒 vs 全睡（纯口径）** | **0.5400 / 0.2675 ≈ 2.02x** | 同场景、同 dt（另一轮 1.98x） |

结论：
- **对外引用 1.64–1.74x** 时，它衡量的是"工程上的 end-to-end 收益（同一段 600 步仿真前后）"；
- **"休眠到底省了多少"的纯物理口径约 2.0x**：全醒稳态 ≈0.54–0.55 ms，全睡 ≈0.27–0.28 ms；
- 两个数字都真实，取决于定义；写文档时把定义写清楚（本页即如此），比只给一个倍数更可信。

### 3.3 单塔/小场景（本次边界测试顺带实测，Release）

| 场景 | 首次全睡步数 | 备注 |
| --- | ---: | --- |
| 单座 5 盒塔（边界测试 `WorldSleepBoundaryTest`） | **step 84** | 休眠前后宽相 proxy 保持 6/6；`WakeBody` 唤醒整岛 5/5 |
| 两座 3 盒塔（`WorldStateInvariantTest`） | **step 37** | 两岛相互独立：唤醒一座后另一座保持 sleeping=3 |
| 10 盒塔（Week 5/7 稳定性场景） | step **859** | 见 §3.4：残余抖动使"连续 0.5 s 安静"窗口要等到 859 步 |

### 3.4 10 盒塔（Week 7 历史实测，同机，仅作量级引用）

> 以下为 `docs/devlog.md` Week 7 的实测数据，**本次会话未重跑**（debug 绝对耗时本来就不可引用），
> 标注"历史实测，本机同机"供对照：

- debug：sleep step **859**、整岛 10 体全睡；asleep **26.5 µs/step** vs awake **87.5 µs/step**，
  **3.30x**；休眠后 100 步状态与接触冲量逐位不变，proxy 保留 11 个；
- 10 盒塔入睡慢（859 步 = 14.3 s）是**逐点 PGS 对 2 点接触的顺序偏差**造成的残余抖动尖峰
  （W5 实测抖峰 0.0248 m/s > `kSleepLinearTolerance=0.01`），**不是休眠规则 bug**；正解是
  2 点接触 2×2 block solver（v1.0 已知限制，留给 v1.1）。

### 3.5 为什么加速不是数量级（1.7x 而非 5x）

休眠岛确实跳过 AABB 刷新/窄相/求解，但**每帧仍有固定成本**：

- 宽相仍在跑（休眠 proxy 保留：proxies=501、candidate_pairs=900/帧），否则无法做邻近唤醒；
- 休眠-休眠的接触复用上一帧 manifold（不跑窄相，但列表仍要遍历/匹配）；
- World 的 O(N) 扫描（积分、同步、休眠分类、校验）与岛分区本身仍在；
- 500 盒场景里真正贵的是窄相/求解，这两块才是休眠剔除的；宽相 sweep 与接触复用是"保留语义"的
  代价，所以 1.7–2.0x 是合理区间（若连宽相也整岛跳过，邻近唤醒就失效了）。

---

## 4. 三种按岛求解模式（flat / grouped-direct / grouped-packed）

- 场景：**400 bodies / 760 contacts 单岛**（密集网格），由 `test_solver_island.cpp` 构造；
- 计时范围：只计 `ContactSolver::SolveVelocity`（40 reps 求和；不含宽相/窄相/位置层）；
- 命令（§1.4 第三条），两轮共 8 次运行取中位数（逐次波动 ±10–15%，CPU 频率敏感，见下表"范围"）。

| 迭代 | flat（无分区，W6 路径） | grouped-direct（默认） | grouped-packed（opt-in） |
| ---: | ---: | ---: | ---: |
| **8 迭代** | **1.860 ms**（1.69–2.07） | **1.908 ms**（1.83–2.14，flat/该模式 = 0.97x） | **2.137 ms**（2.12–2.36，flat/该模式 = **0.87x**） |
| **32 迭代** | **7.056 ms**（6.47–7.62） | **7.570 ms**（7.34–7.88，0.93x） | **7.928 ms**（7.34–8.44，0.89x） |

（表中"flat/该模式"<1 表示该模式比 flat 慢，如 0.87x = 慢约 15%；数字是 **8 次运行的中位数**：
第一轮 5 次（20:28，partner 源码状态）+ 最终源码轮 3 次（20:39）；两轮分开看 direct 相对 flat 在
0.90–1.05x 之间波动，属运行间噪声。）

对照 Week 7 历史（同机、同场景，40 reps）：8 迭代 flat 1.741 / direct 1.863（0.93x）/ packed 2.122
（0.82x）；32 迭代 7.425 / 7.590（0.98x）/ 7.987（0.93x）。本次复测方向一致：
**grouped-direct 与 flat 基本同级（0.93–0.97x，个别运行甚至略快，属噪声；默认值可接受，换取按岛
剔除休眠岛与多岛剔除）**，**grouped-packed 在两轮里都稳定更慢（0.82–0.91x）**。

结论与设计决策（已登记 `docs/dev-interfaces.md` §5）：

- **`packIslandBodies = false` 默认关闭**：400 个 `Body` ≈ 80 KB，整个 body 数组本来就装得进 L2，
  每帧把热数据拷进连续数组的成本 > 局部性收益（Debug 下 packed 反而快是访问放大效应，不能作为
  默认值依据——**默认值只按 Release 测量定**）；
- 岛屿的真实收益不是"打包更快"，而是 **休眠跳过 + 多岛剔除**（§3 的 1.7–2.0x）；
- 三模式在相同初始状态下**逐位一致**（W7：21,760 个 float 比较 0 差异），所以切换模式不改物理，
  仅作性能对照。

---

## 5. 构建与测试耗时（clean 配置 + 构建 + ctest）

命令：`scripts/build.ps1 -Preset <preset> -BuildDir <dir> -RunTests`（sanitize 另加 `-NoTestbed`）。
三个目录都是**首次 clean 配置**（`_deps` 为空，含 FetchContent 下载 gtest/Eigen/GLFW/ImGui）。

| Preset | 构建目录 | 总墙钟（配置+构建+ctest） | 其中 ctest | 测试数（该次） | 项目警告 | 备注 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| debug | `build/w8core` | **80.3 s** | 47.6 s | 387 | **0** | 含 testbed/bench；ctest 当时受并行 agent 编译影响偏慢（后一次 20.2 s） |
| release | `build/w8corerel` | **64.1 s** | 10.5 s | 355（首次时） | **0** | RelWithDebInfo；最终版 390 用例（新增测试重编仅 ~1–2 s） |
| sanitize | `build/w8core-san` | **110.1 s** | 75.1 s | 387 | **0** | MSVC ASan（Debug 语义）；`-NoTestbed`；ASan 下 ctest 约 3.5x 慢 |

> 说明：构建耗时**不含**首次 `FetchContent` 之外的网络波动，且机器同时服务其他 agent 的任务；
> 依赖缓存后的增量重编明显更快（本次最终源码 debug+release 两次重编 + 两次 ctest 合计 100.2 s）。
> **canonical 周验收请删 `build/<preset>` 后 clean 重编**，以 Lead Phase C 数字为准。
>
> 最终源码状态（partner 修改 20:16–20:37 之后）重建复核：release + debug 增量重编各 12.3 s / 24.3 s，
> ctest **390 / 387 全绿、0 警告**；sanitize（20:32–20:33 构建，已含 partner 的 solver 修改）387 全绿。

测试规模快照（2026-10-30 20:34–20:40，`-RunTests`）：

| Preset | ctest 用例 | 结果 | 其中 sandbox SKIP 的 death test |
| --- | ---: | --- | ---: |
| debug | **387** | 100% passed | 31 |
| release | **390** | 100% passed | 31 |
| sanitize | **387** | 100% passed | 31 |

- release 比 debug 多 3 个用例：1 个既有 release-only 契约测试 + 本次新增 2 个 release-only 测试
  （非法 Step 输入的 release 行为、Release 休眠 on/off 计时打印）；
- death test 在本沙箱因"工作区内可执行文件不能写临时文件"而 SKIP；把测试 exe 复制到工作区外运行
  时 **386/386 gtest 全过（含 31 个 death test 真正执行，14.2 s）**，见 §7.3。

---

## 6. 8 个 demo smoke（Debug 终版二进制）

命令模板：`.\build\debug\testbed\physics2d_testbed.exe --demo <Name> --smoke <N> --hidden`，
exit 0 = 通过。下表是**最终源码重建后的 debug 二进制**（20:40 复测）的机器可读输出摘要；Debug 数字
只用于功能回归，不是性能基准。

| Demo（步数） | bodies | islands | awake/sleeping | contacts(点数) | max_pen (mm) | max_speed | max_tilt | broadphase / candidates |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| FallingBall (150) | 2 | 1 | 0 / 1 | 1 (1) | 5.057 | 0.000 | 0.00° | BruteForce / 1 |
| SphereStack (300) | 4 | 1 | 0 / 3 | 3 (3) | 3.063 | 0.000 | 0.00° | BruteForce / 5 |
| BoxStack (600) | 11 | 1 | 10 / 0 | 10 (20) | 3.893 | 0.007 | 0.21° | BruteForce / 19 |
| FrictionRamp (300) | 2 | 1 | 0 / 1 | 1 (2) | 2.686 | 0.000 | 15.00° | BruteForce / 1 |
| Pyramid200 (300) | 137 | 1 | 136 / 0 | 324 (595) | 41.863 | 0.113 | 2.18° | SAP / 496 |
| StressBenchmark (300) | 401 | 4 | 400 / 0 | 820 (1227) | 170.739 | 0.472 | 179.98° | SAP / 1679 |
| Dominoes (600) | 21 | 5 | 20 / 0 | 35 (52) | 6.496 | 1.292 | 90.26° | BruteForce / 39 |
| IslandSleep500 (600) | 501 | 100 | 0 / 500 | 500 (1000) | 3.983 | 0.000 | 0.18° | SAP / 900 |

- **Dominoes** 额外输出：`woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1`
  （睡眠后推第一块，20 块全部被唤醒并倒下，无跳过）；
- **IslandSleep500** debug 窗口（最终源码重建后）：early 4.9574 ms、awake 4.9616 ms、
  asleep 2.9577 ms（**1.68x**）、first_all_asleep_frame=105——与 Release 的 1.70x 同方向
  （Debug 绝对值不可引用）；
- **BoxStack 600 步 awake=10** 是正常的：10 盒塔 step 859 才睡（§3.4），600 步时塔仍站立
  （max_center_y=9.478、max_speed=0.007）；
- Pyramid200/StressBenchmark 的几十~一百多 mm 穿透是**载荷/迭代预算**现象（16 层载荷、400 盒密堆，
  8 次速度迭代），不是 bug；切宽相物理输出不变（§7.2）；
- `avg_frame_ms` 未列入：它包含 GPU 渲染，且 smoke 期间可能与其他 testbed 同机，只作参考。

---

## 7. 回归、宽相 A/B 与契约验证（v1.0 交付证据）

### 7.1 确定性回归工具（`physics2d_regression`）

```powershell
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt
```

- 场景：`tower30+circles20`（30 盒塔 + 20 随机圆，51 bodies），600 步；**弱 hash（quantum 1e-5）**
  `588d30e70f9b4ddd` 与 golden 一致，`exact=f4390ef86c075a04` 仅供信息；
- 退出码语义本次逐条复核：

| 调用 | 期望 | 实测 |
| --- | --- | --- |
| `--check <正确 golden>` | 0 | **0**（打印 `golden OK: hash=588d30e70f9b4ddd`） |
| 无参数（打印 hash） | 0 | **0** |
| `--write-golden <path>` | 0 | **0**（生成文件与仓库 golden 逐行一致） |
| `--check <被篡改 hash 的 golden>` | 1 | **1**（`golden MISMATCH`） |
| `--check <不存在的文件>` | 2 | **2** |
| 未知参数 | 2 | **2** |
| `--help` | 0 | **0** |

- **hash 不变的纪律**：Week 6 起弱 hash 就是 `588d30e70f9b4ddd`；Week 7 岛屿/休眠、Week 8 的
  Core 校验与边界测试都是纯顺序变换/纯校验，**hash 未变**（本次复核仍为同值）。

### 7.2 宽相 A/B（换宽相物理输出逐位一致）

BoxStack 300 步，分别用 `--broadphase none|bf|sap` 运行，取所有 `[smoke] body=`/`[smoke] contact=`
行做 SHA-256：

| broad phase | body+contact 输出 SHA-256 |
| --- | --- |
| none（O(N²) fallback） | `E74DC08335693F039F7E38AF01859F3C6804C1C0FEB6D3BD74437A4018DE8991` |
| BruteForce | `E74DC08335693F039F7E38AF01859F3C6804C1C0FEB6D3BD74437A4018DE8991` |
| SAP | `E74DC08335693F039F7E38AF01859F3C6804C1C0FEB6D3BD74437A4018DE8991` |

**三者完全相同**（只有 candidate_pairs 数字不同）——W6 契约在 v1.0 保持。命令：

```powershell
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden --broadphase none|bf|sap
```

### 7.3 契约（death test）在工作区外真实执行

沙箱内 31 个 death test 会显示 SKIP（`docs/testing.md` §8）；复制到工作区外运行：

| 项 | 结果 |
| --- | --- |
| `build\debug\tests\physics2d_tests.exe` 复制到 `%LOCALAPPDATA%\Physics2D\testrun\` 后运行 | **386/386 passed（66 suites，14.2 s）**，31 个 death test 真正执行（含本次新增 3 个 Step 非法输入契约） |

---

## 8. Week 8 边界测试与 Core 校验新增（本次交付）

### 8.1 新增 `tests/unit/test_boundary_world.cpp`（12 个用例；debug 10 + release-only 2）

| 套件 / 用例 | 钉住的契约 |
| --- | --- |
| `WorldLifecycleBoundaryTest.DoubleDestroyAndStaleHandlesAreTotal` | `DestroyBody` 幂等；stale 句柄的读/写/唤醒全部安全失效；槽位复用后旧句柄绝不 alias 新 body（ABA）；世界只剩死槽也能 Step |
| `WorldLifecycleBoundaryTest.ClearDropsBodiesShapesProxiesAndIslandState` | 休眠中的场景 `Clear()` 释放 body/shape/proxy/岛分区；旧句柄全失效；`Clear` 幂等；世界可复用且槽位重置 |
| `WorldStepBoundaryDeathTest.ZeroDeltaTimeAsserts`（Debug） | `Step(0)` 触发 assert "dt must be finite and > 0" |
| `WorldStepBoundaryDeathTest.NegativeAndNonFiniteDeltaTimeAsserts`（Debug） | `Step(-dt/NaN/Inf)` 同样 assert |
| `WorldStepBoundaryDeathTest.NegativeIterationCountsAssert`（Debug） | 负迭代数触发 assert "iteration counts must be non-negative" |
| `WorldStepBoundaryTest.InvalidStepInputIsRefusedInReleaseBuilds`（Release only） | NDEBUG 下非法 dt 安全返回且状态不变；负迭代数被钳到 0（定义明确的"零迭代求解"）；不产生 NaN/Inf |
| `WorldSleepBoundaryTest.WakeBodyIsWholeIslandAndProxiesSurviveSleep` | 5 盒塔 step 84 全睡；休眠后 proxy 保留 6/6、速度精确为 0、`islandId`/`sleepTime` 合法；`WakeBody` **整岛**唤醒（5/5）；再次入睡无 proxy 泄漏 |
| `WorldSleepBoundaryTest.DisablingSleepingAfterSettleWakesTheIslandAgain` | 运行中把 `enableSleeping` 关掉会唤醒已睡岛（真"休眠开关"而非永久冻结）；重新打开后能再次入睡 |
| `WorldBroadPhaseBoundaryTest.SwitchingBroadPhaseWhileAsleepPreservesSleepAndProxies` | 睡着时 `SetBroadPhase`：旧 proxy 立即释放、下一步按体数重建（含休眠体）；休眠状态与接触列表保持不变；切换后仍可被整岛唤醒 |
| `WorldBroadPhaseBoundaryTest.PairSourcesStayBitIdenticalAcrossLifecycleChurn` | 200 步 × {无宽相/BF/SAP}：场景中途销毁接触中的 body、再创建复用槽位，逐帧 body+contact 浮点**逐位一致**（延伸 W6 的静态场景等价性到生命周期 churn） |
| `WorldStateInvariantTest.IslandSleepBookkeepingIsConsistentAfterEveryStep` | 每步后岛员 `islandId` 在 `[0, islandCount)`、`sleepTime` 有限非负、休眠动态体四路速度精确为 0、伪速度每步清零；两岛独立唤醒 |
| `WorldBoundaryProfilingTest.SleepingOnVsOffStepTimingIsPrinted`（Release only） | 第 §3.2 的休眠 on/off 直接计时（打印 `[boundary8]`，供本页复现） |

### 8.2 `src/Core/World.cpp` 的两处 W8 追加（不改物理，只加契约/校验）

1. **`Step` 的 Release 兜底**：非法迭代数在 Debug 仍是 assert（契约不变）；NDEBUG 下钳到 0，
   不再把负数存成"负的求解预算"（非法输入在 Release 得到定义明确的行为）。
2. **`CheckSane`/`Validate` 覆盖 W7 新状态**：除既有的 NaN/Inf 扫描（含伪速度）外，新增
   `islandId` 范围检查、`sleepTime` 有限非负、以及"休眠动态体速度必须精确为 0"的检查
   （错误信息里同时打印 islandId/islandCount/sleepTime/awake/type，便于定位）。Debug 下每步
   `Validate` 都会跑；健康场景全部通过（上述 12 个边界测试逐步调用 `CheckSane()` 验证无假阳性）。

> 这两处只影响**非法输入行为**与**Debug 校验**：合法输入路径逐位不变，回归弱 hash 仍是
> `588d30e70f9b4ddd`（§7.1），三 preset 全绿（§5）。
>
> 注：本次只负责 `test_boundary_world.cpp` 与 `src/Core/`；退化多边形/零密度/极大速度/超长薄条/
> 极端 mass ratio/NaN 输入等数学·几何·求解器侧边界由 `solver-dev` 同批的
> `tests/unit/test_boundary_solver.cpp` 等覆盖（细节以该文件及其任务卡为准）。

---

## 9. Tracy（R11）现状与后续

**v1.0 不正式接入 Tracy**（R11 推迟到 v1.1），本页数字全部来自"零依赖计时"而不是 Tracy 时间线：

| 项 | 现状 |
| --- | --- |
| `Core/Profiler.h` | 只有零成本宏（`PHYSICS2D_ENABLE_TRACY=OFF` 时展开为空），Week 1 预留 |
| CMake | `PHYSICS2D_ENABLE_TRACY` 选项存在，但需要 `-DPHYSICS2D_TRACY_ROOT=<本地 Tracy 源码>`；默认 OFF，v1.0 不构建 |
| 为什么推迟 | ① 交付周优先保证文档/边界测试/三 preset 全绿；② Tracy 需外部源码与截图流程，且开启后有 1–3% 开销；③ 当前所有优化决策已可由现有计时证明（packed 0.87x、休眠 1.7–2.0x），没有"必须看时间线才能定位"的热点 |
| 现有计时手段（本页数字来源） | `physics2d_bench`（`steady_clock`，逐帧 median/mean + JSON）、testbed Status/Demo 面板（平滑与原始 Step ms、IslandSleep500 的 early/awake/asleep 窗口）、gtest 打印（`[solver7] solve timing`、`[boundary8]`）、回归弱/精确 hash |
| v1.1 计划（设计文档 8.1） | 按设计文档只埋 8 个 `ZoneScopedN`：`World::Step` 及 IntegrateForces/BroadPhase/NarrowPhase/BuildIslands/WarmStart/SolveVelocities/IntegratePositions/SolvePositions/Sleep；`SAP::Sort`/`SAP::Scan`；计数器/曲线：`AwakeBodies`、`ContactPairs`、`ManifoldPoints`；截图与本文并列 |

---

## 10. 已知局限与噪声（引用本页数字前先读）

1. **跨机器不可比**：绝对 ms 只在本机（i7-12650H + MSVC 19.44 + 接通电源）有意义；换机器请重测，
   只沿用"加速比/趋势/是否默认开启"的结论。
2. **±2–5% 噪声**：bench 单次数字；跨运行取中位后仍可能差几 %。三模式计时逐次波动可达 ±10–15%
   （CPU 频率/调度），所以本页给中位 + 范围，不把 1–2% 的差异当结论。
3. **微秒级数字不要引用**：小 N 宽相（5/10/20）是计时器噪声量级，只看交叉点趋势。
4. **Debug/ASan 数字不可引用**：只用于功能回归；本页 Debug 数据仅出现在 §6（功能 smoke）与
   §3.4（历史引用，已标注）。
5. **testbed `avg_frame_ms` 含 GPU 渲染**，且 smoke 期间可能与其他 agent 的 testbed 同机，不是物理
   性能；物理耗时用 `Step` 计时（gtest/testbed demo 窗口）或 bench。
6. **并发 agent**：本页在 §1.2 记录了并发情况；Week 8 的 canonical 交付验收由 Lead 在干净环境
   clean Release 复跑，若与本节数字漂移 >5%，以 canonical 为准并更新此页（doc 纪律：数字必须实测）。
7. **bench 不跑物理**：宽相表没有 Step 总耗时列；"完整 Step 有多贵"看 §3（休眠）与 §4（求解器）。

---

## 附：原始数据文件位置（不入库）

本次测量的原始 stdout 保存在 `build/w8meas/`（第一轮）与 `build/w8meas2/`（最终源码轮）
（`build/` 被 gitignore，属于临时证据）：

- `bench_large_run1..5.txt` / `bench_small_run*.txt` / `bench_crossover_run*.txt`：bench 全表；
- `solver_modes_run1..5.txt`（第一轮）+ `w8meas2/solver_modes_run1..3.txt`：三模式计时（`[solver7]` 行）；
- `sleep_timing_run1..3.txt`（第一轮）+ `w8meas2/sleep_timing_run1..3.txt`：休眠 on/off 直接计时；
- `testbed_release_islandsleep500.txt`（两轮各一份）：Release 端到端休眠窗口；
- `smoke_debug_<Demo>.txt` / `w8meas2/smoke_debug_<Demo>.txt`：8 个 demo smoke 全输出（后者为最终源码）；
- `regression_cli.txt`、`broadphase_ab.txt`、`gtest_outside_workspace.txt`：§7 的交付证据。

一次性探针（§3.2 的逐步睡眠计数）在 `build/w8probe/`，不提交、不参与构建。
