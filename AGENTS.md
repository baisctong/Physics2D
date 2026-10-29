# AGENTS.md — Physics2D 仓库工作手册（给后续 AI/开发者）

> 任何新的工作轮次（人或 AI）**先读本文件**，再读 `docs/dev-interfaces.md` 与 `docs/ROADMAP.md`。
> 本文件是"项目宪法 + 交接协议"，避免每个会话重新发明约定、把符号约定搞混。
>
> **换会话/换人接手时，先读 [`HANDOFF.md`](HANDOFF.md)**：一页纸交接（现状、验证基线、
> 该做/不该做、Week 5 任务卡、环境坑、团队流程）。本文件负责长期规则，HANDOFF 负责当前状态。

---

## 0. 项目是什么

`Physics2D`：从零手写的 C++20 2D 刚体物理引擎，用于校招简历与面试深挖。
评价标准是「**链路完整 + 原理正确 + 有量化数据 + 代码可读可讲**」，不是功能多。
设计文档原文在仓库外：`../2D物理引擎项目完整工作流与DeepSeek提示词库.md`（第 5 章按周执行，第 12 章提示词库）。

**当前状态：Week 1-7 已完成**（数学库、Body/World 积分骨架、形状系统 + 圆族解析碰撞、
**凸多边形 + SAT + 裁剪式 Manifold + SATCache**、**摩擦锥 + warm starting + split impulse 求解器**、
**10 方块堆叠 10 秒稳定里程碑**、**BruteForce + SAP 可插拔宽相（2000 物体 7.8x）**、
**岛屿分区 + 按岛求解（三模式逐位一致）+ 整岛原子休眠（500 物体休眠后 1.73x）**、
**headless 确定性回归（量化 hash + golden + CI job）**、Testbed 8 个 demo 与完整调试面板；
Week 8 为文档/交付周）。详见 `docs/ROADMAP.md` 与 `docs/devlog.md`。

---

## 1. 项目宪法（不可协商）

### 1.1 技术栈（不要建议更换）

- C++20；CMake ≥ 3.25 + Ninja（Windows 用 VS2022 / MSVC 19.3x；CI 双平台）。
- `physics2d_core` **零第三方依赖**（只用 C++ 标准库；Tracy 是唯一例外且默认关闭）。
- testbed：GLFW 3.4 + OpenGL 3.3 Core + Dear ImGui（docking 分支，commit 固定）。
- 测试：GoogleTest；数学交叉验证用 Eigen（仅测试 target）。
- 依赖统一走 CMake `FetchContent`（`cmake/Dependencies.cmake`），**不使用 vcpkg**。

### 1.2 分层铁律

- `include/physics2d/`、`src/` 内**绝对禁止** `#include` GLFW / OpenGL / ImGui / SDL（Tracy 例外，走 `Core/Profiler.h`）。
- `physics2d_tests` / `physics2d_bench` 只依赖 core；`physics2d_testbed` 才允许渲染依赖。
- 物理代码不出现"像素"：`kPixelsPerMeter` 只在 `testbed/Camera` 里。

### 1.3 单位与符号约定（违反会导致数值失稳）

- MKS：米/千克/秒/弧度；刚体尺寸 0.1–10 m；默认重力 `(0,-10)`；渲染 `kPixelsPerMeter = 40`。
- 右手系，+x 右，+y 上，逆时针角度为正。
- 接触法线 **从形状 A 指向形状 B**；相对速度 `vRel = (vB + ωB×rB) − (vA + ωA×rA)`。
- 三个 Cross 重载严禁混用：`Cross(Vec2,Vec2)->float`、`Cross(Vec2,float)->Vec2`、`Cross(float,Vec2)->Vec2`。
- 积分：semi-implicit Euler（先速度后位置）；固定步长 `dt=1/60`，可变步长禁止。

### 1.4 编码规范

- `namespace p2`；类型 PascalCase；函数 PascalCase；成员 `m_` 前缀；常量 `kPascalCase`；
  文件名 = 主类型名；公共头 `include/physics2d/<模块>/`，实现 `src/<模块>/`。
- 有返回值的函数一律 `[[nodiscard]]`；浮点统一 `float`；除法/归一化必须有零值保护。
- 核心库不抛异常：Debug 用带消息的 `assert`，Release 用返回值/`std::optional`；**不静默修正非法输入**。
- 所有魔法数字集中在 `include/physics2d/Core/Settings.h`；本文件没有的常量名不允许散落。
- 注释用英文解释**为什么**（设计取舍），不是复述代码。

### 1.5 既定设计（不要另起炉灶）

- Broad Phase：`IBroadPhase` 接口 + BruteForce 基线 + SAP 主线（DynamicAABBTree 选做）。
- Narrow Phase：凸多边形 SAT + 面-面裁剪（≤2 接触点）；圆相关走解析路径。
- ContactID 基于特征编码（referenceEdge/incidentVertex/type/flip），帧间稳定是 warm starting 前提。
- Solver：Sequential Impulse（PGS），必须含 warm starting、累计冲量钳制、库仑摩擦锥、restitution 速度阈值、split impulse。
- 岛屿：DFS 分岛，整岛原子休眠/唤醒。
- `World::Step` 流水线顺序固定：积分力→宽相→窄相→建岛→warm start→速度求解→积分位置→位置求解→同步→休眠→Debug 校验。

### 1.6 范围红线（不做，也不建议做）

3D、软体/流体/布料、GPU 加速、完整关节系统、CCD/TOI（除非明确要求）、兼容 Box2D API、引入游戏引擎。
新想法一律先进 `docs/ROADMAP.md` 的 Roadmap，不进当前迭代。

### 1.7 输出要求（AI 协作时）

- 一次只处理一张任务卡/一个模块；不要顺手改别的文件。
- 先给 3-6 行计划（要改哪些文件、风险），再写代码。
- 代码必须完整可编译（含所有 include），不写 TODO/伪代码/"此处省略"。
- 每个非显然决定用一行注释说明理由；需求与宪法冲突时先指出再给替代方案。

---

## 2. 环境与构建（本机实测）

**Windows 注意事项（很重要）**

1. 本机**已安装 PowerShell 7（`pwsh` 7.6.9，WindowsApps 别名可用），但仓库命令/构建脚本统一用
   Windows PowerShell 5.1**：`powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 ...`
   （`build.ps1` 在 PS7 下会在 `$isWindows` 只读自动变量处报错，尚未适配；`pwsh` 只用于临时命令，
   不参与验证基线）。
2. **Ninja 不能放在仓库内**：本机 DSH agent 沙箱会拒绝"可执行文件位于工作区内部"的子进程写文件，
   表现为 `cl ... error D8050` / `.ninja_lock Permission denied`。
   `scripts/build.ps1` 会自动使用 VS 自带 Ninja（或 `%LOCALAPPDATA%\Physics2D\tools`），不要复制到 `tools/`。
3. 首次 configure 会 FetchContent 下载 gtest/Eigen/GLFW/ImGui（约 1-2 分钟）。
4. **不要用 PowerShell 的 `Get-Content/Set-Content` 改写 UTF-8 中文文档**：PS 5.1 会把无 BOM 的
   UTF-8 当 ANSI 读，导致中文变成乱码。改文档用编辑器工具（或确认编码后处理）。

常用命令：

```powershell
# 全量：配置 + 编译 + 测试（推荐给日常开发/KPI 验证）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 并行工作隔离构建目录（多人/多 agent 同时开发）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/mywork -NoTestbed -RunTests

# Release（性能测量必须用）/ Sanitizer（MSVC 只有 ASan，UBSan 走 Linux）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset sanitize -BuildDir build/sanitize -NoTestbed -RunTests

# 没有 Ninja 的环境：VS 生成器兜底
cmake --preset vs2022-debug
cmake --build --preset vs2022-debug
ctest --preset vs2022-debug
```

---

## 3. 文件地图（新增代码放哪里）

| 你要做的事 | 文件位置 | 是否要改 CMake |
| --- | --- | --- |
| 新增 core 模块实现 | `src/<模块>/*.cpp` | 要：`src/CMakeLists.txt` 显式源文件列表追加 |
| 新增公共头 | `include/physics2d/<模块>/*.h` | 不要（PUBLIC include 目录已配置） |
| 新增单测 | `tests/unit/test_*.cpp` | 不要（GLOB CONFIGURE_DEPENDS 自动收集） |
| 新增 demo | `testbed/demos/*.cpp` | 不要（GLOB 自动收集；demo 需在 `testbed` 注册表登记） |
| 新增依赖 | `cmake/Dependencies.cmake` | 只允许 testbed/tests/bench 消费；core 不允许 |
| 新增常量 | `include/physics2d/Core/Settings.h` | 不要 |
| 文档 | `docs/*.md`、根 README / AGENTS | 不要 |

**公共 API 冻结**：`World` 全部 public 方法、`Body/BodyDef/Handle/IDPool/TimeStep/Settings`、
`Math` 全部签名、`Shape/ShapeType/MassData/{Circle,Plane,Polygon}Shape`、
`Collide*`/`FlipManifold`/`SAT`/`CollidePolygons`/`FindMaxSeparation`/`ClipSegmentToLine`/`SATCache` 签名、
`IBroadPhase/ContactID/Manifold/Contact` 字段语义。
只允许**追加**；破坏性修改必须在 `docs/dev-interfaces.md` 第 5 节登记理由。

**当前技术债（明确标记，按周清理）**：

- ~~`TEMPORARY (Week 2)` 位置推出~~ → **W5 已删除**（split impulse 伪速度层取代）。
- ~~无摩擦 / 无 warm start / 无 split impulse~~ → **W5 已完成**。
- ~~O(N²) 配对枚举 / headless 回归占位~~ → **W6 已完成**（SAP/BruteForce + regression 工具）。
- ~~岛屿/休眠未实现~~ → **W7 已完成**：岛屿 DFS、三种按岛求解模式（默认 grouped-direct）、
  整岛原子休眠/唤醒、休眠岛跳过宽相 AABB 更新/窄相/求解、邻近唤醒、统计接入。
- **2 点接触 2×2 block solver（W8 可选，最高优先技术债）**：逐点 PGS 对薄片/2 点接触的顺序偏差造成
  静止残余抖动尖峰（maxV 0.015–0.025 m/s > `kSleepLinearTolerance=0.01`），导致 10 盒塔 ~859 步才入睡、
  更薄形状几乎无法入睡。**不要靠调大休眠阈值掩盖**。
- **`BodySim`/`BodyState` 冷热拆分（W4→W5→W6→W7→W8 顺延，W8 可选）**：W7 实测"岛内打包" Release
  反而慢 18%（400 体 Body 数组 == ~80 KB，本就装得进 L2），说明该规模下 body 布局不是热点；正确顺序是
  先 Tracy profiling 证明热点再做（纯重构，回归 hash 必须不变）。
- **按岛 body 打包默认关闭**（`SolverSettings::packIslandBodies=false`）：保留 opt-in + 位级测试覆盖。
- Tracy 正式接入（R11）在 W8；CI format/clang-tidy 在 W8。
- **本机增量构建坑**：中文 locale MSVC 的 `/showIncludes` 不被 Ninja 识别 → 类布局混链会 HEAP CORRUPTION。
  `scripts/build.ps1` 已自动检测 header 比 obj 新并强制 `--clean-first`；canonical 周验收仍须 clean build。
- `ContactSolver::SolvePosition` 每帧分配局部约束 vector（未证明是热点，先测量）。
- 30:1 单列/金字塔底部在默认 8 迭代下的穿透是载荷/迭代预算属性（记录在 README，不修）。

---

## 4. 每周工作的标准动作（Definition of Done）

完成一个任务前必须全部通过：

- [ ] `clang-format`（或保持与 `.clang-format` 一致的手工风格）
- [ ] `scripts/build.ps1 -Preset debug` **零项目警告**（MSVC /W4）
- [ ] `scripts/build.ps1 -Preset debug -RunTests` 全绿（新增测试覆盖新代码 + 边界）
- [ ] `-Preset sanitize` + ctest 干净（至少每周一次；MSVC 为 ASan）
- [ ] 新增公共 API 有 `[[nodiscard]]`（若有返回值）
- [ ] 没有魔法数字散落（全在 Settings.h）
- [ ] 没有渲染头文件进入 core（CI 分层 job 会抓）
- [ ] `docs/devlog.md` 写 3 行说明（用自己的话）
- [ ] commit message 用 Conventional Commits（`feat(solver): ...` / `fix(sat): ...` / `perf(broadphase): ...` / `test(...)`）
- [ ] 周结束：打 tag（`v0.2-plane-contact`、`v0.3-impulse` …）、写周报、更新 README 性能表（如有新数字）

**修复循环上限 3 轮**：编译/测试错误改 3 轮还不绿 → 拆任务卡或人工读代码，禁止无限 AI 自循环。

---

## 5. 下一周（Week 8：文档与交付周）从哪开始

1. 读 `docs/dev-interfaces.md` 第 2 节剩余项（R11 Tracy、R15 冷热拆分可选、R17 插值可选）与第 4 节
   W8 行；读设计文档第 8 章（profiling 输出格式与测量纪律）、第 10 章（README 结构/简历 bullet/面试
   25 问）、第 12.5 节（W01-W04 文档提示词）。
2. W8 的产出（详细验收见 `docs/ROADMAP.md` W8 表）：
   - `docs/profiling.md`：汇总 bench（SAP vs BruteForce）、休眠前后 Step、求解器模式对比的**全部真实
     数字 + 测量环境**；若接 Tracy 则附时间线截图（R11）；
   - `docs/math.md`：SAT 推导、冲量方程、摩擦锥、split impulse bias、warm start 的收敛解释——每一条
     都要能手推/口述；
   - `docs/architecture.md` 总图升级（模块依赖 + Step 时序 + 岛屿/休眠状态机）；
   - `README.md` 完整版：GIF（10 盒稳定 / 2000 物体 SAP / 500 物体休眠 / 多米诺唤醒）、快速开始、
     架构、性能表、已知限制、Roadmap；确保陌生人 5 分钟内能编译运行；
   - **已知限制 + 边界测试补强**：退化多边形、零密度、极大速度、超长薄条、极端 mass ratio——所有已知
     崩溃点都要有 assert 或明确文档；`CheckSane`/`Validate` 覆盖新状态（伪速度、islandId、sleepTime）；
   - 简历 bullet（设计文档 10.3 模板）与面试问答演练（10.4 清单）；
   - **tag `v1.0-release`** + GitHub Release 说明（附 GIF）。
3. 可选优化（只在文档完成后、且以测量为前提）：**2 点接触 2×2 block solver**（最高优先技术债：消除
   薄片残余抖动 → 更快入睡；改求解器前先读 `docs/dev-interfaces.md` 第 6 节不变量，回归弱 hash 必须
   保持不变或解释变化）、R11 Tracy 正式接入、R17 渲染插值、R15 冷热拆分（**先 profiling 证明热点**）。
4. 收尾：三 preset clean 验证、8 个 demo smoke、`--broadphase` A/B、回归 `--check`、工作区外 death test
   全量，然后更新全部 docs 并打 tag。

### W7 的岛屿/休眠约定（改 World/求解器前必读）

- **分组求解必须逐位等价**：`flat` / `grouped-direct`（默认）/ `grouped-packed`（opt-in）三模式在同一
  初始状态下 body 状态与 contact 累计冲量**逐位一致**（岛间独立 → 更新可交换；岛内相对顺序保持）。
  改岛序/岛内顺序/唤醒时机前先想清楚这一点。
- **休眠是整岛原子**：不允许单 body 冻结/唤醒；休眠岛保留 broad-phase proxy 与上一帧 contact；
  休眠岛跳过 AABB 更新/窄相/warm start/速度与位置求解，其累计冲量冻结不动。
- **唤醒必须是整岛**：外力/冲量/SetTransform/SetVelocity 走 `WakeIslandOfBody`；宽相邻近唤醒放在
  宽相与 fallback **共享**的 `UpdateContacts` 开头（不要放 `UpdateBroadPhase`，那里无宽相会提前 return）。
- **阈值不动**：`kSleepLinearTolerance=0.01` 等保持设计值；入睡慢用 block solver 解决，不是调阈值。
- **打包默认关闭**：`packIslandBodies=false`（Release 实测 packed 慢 18%）；打包路径保留且被位级测试覆盖。
- 不变量清单见 `docs/dev-interfaces.md` 第 6 节「W7 起生效的岛屿/休眠不变量」。

### W6 的宽相约定（改宽相/World 接线前必读）

- **换宽相不得改物理**：候选对必须规范化为 `(i<j, sorted, deduped)` 的 body-index 对再进窄相；
  验收 = 同一场景 无宽相/BruteForce/SAP 跑 N 步 body 状态与接触列表**逐位一致**。
- **宽相只知道 AABB**：`IBroadPhase` 不出现 Body/World 类型；proxy↔body 映射由 World 维护
  （`Body::proxyId` + `World::m_proxyToBody`）。改接口属破坏性变更，先登记偏差。
- **proxy 生命周期**：`UpdateBroadPhase` 是唯一建/更 proxy 的地方；`DestroyBody`/`SetBroadPhase`/`Clear`
  必须同步释放；无 shape 的 body 没有 proxy。
- **fat AABB 只加候选**：`Synchronize` 写 tight + `kAabbMargin`；漏检先查 pair 规范化/时序，不要调大 margin。
- **`ValidateInternalState` 是契约**：SAP 任意时刻（增删/移动后）都必须 true。
- **回归 hash 纪律**：`tests/regression/golden/default_600.txt` 只在**有意的物理/场景变更**时更新，
  devlog 写 old/new 与原因；纯重构（宽相切换、BodySim 拆分、排序优化）hash 不变。
- 不变量清单见 `docs/dev-interfaces.md` 第 6 节。

### W5 的求解器约定（改求解器前必读）

- **摩擦锥钳制累计值**：`accT = Clamp(accT + λt, −μ·accN_new, +μ·accN_new)`；法向先解、切向后解；
  `μ` 用 `contact.friction`（World 已做 `sqrt(μA·μB)`），不要二次组合。
- **warm start 只施加一次**：`Prepare` 播种 `inherited × warmStartFactor`，`WarmStart` 施加 cache
  原值；默认 `warmStartFactor = 1.0`（小于 1 会每帧丢已收敛解、重新注入 PGS 顺序偏差，实测 10 盒塔
  所有 <1.0 的 factor 都倒）。
- **split impulse 只碰伪速度**：位置层不得读写真实速度；每步结束清零伪速度；位置层要用
  **位置积分后**的 transform 重算杠杆臂（速度层 cache 已过期一步）。
- 不变量清单见 `docs/dev-interfaces.md` 第 6 节「W5 起生效的求解器不变量」。

---

## 6. 参考

- 设计文档：第 5 章（周计划）、第 6 章（公式/伪代码）、第 7 章（测试）、第 12 章（提示词）。
- Box2D v2.4.1：`b2ContactSolver` / `b2CollidePolygon` / `b2DynamicTree`（入门优先读 v2.4.1）。
- Erin Catto, *Iterative Dynamics with Temporal Coherence*（GDC 2005）。
