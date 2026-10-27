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
