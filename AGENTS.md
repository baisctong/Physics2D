# AGENTS.md — Physics2D 仓库工作手册（给后续 AI/开发者）

> 任何新的工作轮次（人或 AI）**先读本文件**，再读 `docs/dev-interfaces.md` 与 `docs/ROADMAP.md`。
> 本文件是"项目宪法 + 交接协议"，避免每个会话重新发明约定、把符号约定搞混。

---

## 0. 项目是什么

`Physics2D`：从零手写的 C++20 2D 刚体物理引擎，用于校招简历与面试深挖。
评价标准是「**链路完整 + 原理正确 + 有量化数据 + 代码可读可讲**」，不是功能多。
设计文档原文在仓库外：`../2D物理引擎项目完整工作流与DeepSeek提示词库.md`（第 5 章按周执行，第 12 章提示词库）。

**当前状态：Week 1 已完成**（数学库、Body/World 积分骨架、Testbed 骨架、单测/CI/构建体系、
Week 2-8 预留接口）。详见 `docs/ROADMAP.md` 与 `docs/devlog.md`。

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

1. 本机只有 **Windows PowerShell 5.1，没有 `pwsh`**。命令一律用：
   `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 ...`
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
`Math` 全部签名、`IBroadPhase/ContactID/Manifold/Contact` 字段语义。
只允许**追加**；破坏性修改必须在 `docs/dev-interfaces.md` 第 5 节登记理由。

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

## 5. 下一周（Week 2）从哪开始

1. 读 `docs/dev-interfaces.md` 第 2 节 R4/R5/R6/R7 行与第 4 节 W2 行。
2. 读设计文档第 5 章 Week 2、第 6.3 节、提示词 P05(完整)/P06。
3. W2 的产出：形状系统（Circle/Plane；Polygon 留到 W4 的 P04）、圆-圆/圆-平面解析碰撞、
   `Contact` + `ContactID` 初版、`World::UpdateContacts` 接线、第一个"球停在地面上"demo。
4. 完成后更新 `docs/ROADMAP.md` 状态、`docs/dev-interfaces.md` R 表格状态、`docs/devlog.md`、打 tag `v0.2-plane-contact`。

---

## 6. 参考

- 设计文档：第 5 章（周计划）、第 6 章（公式/伪代码）、第 7 章（测试）、第 12 章（提示词）。
- Box2D v2.4.1：`b2ContactSolver` / `b2CollidePolygon` / `b2DynamicTree`（入门优先读 v2.4.1）。
- Erin Catto, *Iterative Dynamics with Temporal Coherence*（GDC 2005）。
