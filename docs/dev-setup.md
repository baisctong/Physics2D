# 开发环境与构建（Week 1 现状）

> 目标读者：第一次 clone 本项目的人（包括未来的你）。按本文三步即可从零构建。

## 1. 前置条件

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| 操作系统 | Windows 10/11 x64 或 Ubuntu 22.04+ | CI 覆盖 Windows + Ubuntu |
| 编译器 | MSVC 19.3x（VS 2022 "使用 C++ 的桌面开发" 工作负载）/ GCC 12+ / Clang 15+ | C++20 |
| CMake | ≥ 3.25（presets 文件用了 v6 语法） | 3.31 已验证 |
| Git | 任意较新版本 | |
| Ninja | `bootstrap.ps1` 优先使用 Visual Studio 自带的 Ninja，没有才下载到 `%LOCALAPPDATA%\Physics2D\tools`（刻意放在仓库外） | 也支持 VS 生成器预设 |
| Python | 可选，仅 `bootstrap.ps1` 用来下载 clang-format | |

**本仓库不使用 vcpkg。** 设计文档允许 FetchContent 与 vcpkg 二选一，本项目选 FetchContent，
保证 `git clone` 后无需预装 vcpkg 即可配置。详见 `cmake/Dependencies.cmake`：

| 依赖 | 版本 | 谁用 |
| --- | --- | --- |
| GoogleTest | v1.15.2 | 单测 |
| Eigen | 3.4.0 | 仅单测做数学交叉验证（含 `EIGEN_MPL2_ONLY` 语义） |
| GLFW | 3.4 | testbed 窗口/输入 |
| Dear ImGui | docking 分支，commit `d88f238d`（固定） | testbed 调试 UI |

> 依赖在首次 configure 时下载到 `build/<preset>/_deps/`，之后可离线使用
> （`-DPHYSICS2D_OFFLINE=ON` 或复用已配置的构建目录）。

## 2. Windows 三步构建

在 **仓库根目录** 打开 PowerShell（Windows PowerShell 5.1 即可，无需 PowerShell 7）：

```powershell
# 1) 一次性环境引导：检查 VS/CMake/Git，并确保有一个可用的 Ninja
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/bootstrap.ps1

# 2) 配置 + 编译 + 跑单测（推荐：脚本会自动 import MSVC 环境）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 3) 运行测试台
.\build\debug\testbed\physics2d_testbed.exe
```

> 本机没有 `pwsh`（PowerShell 7），请使用 `powershell`；命令中的 `-NoProfile` 可以避免
> 用户 profile 干扰 MSVC 环境变量。

### 2.1 预设一览（Ninja）

| preset | 配置 | 产物 |
| --- | --- | --- |
| `debug` | Debug，testbed+tests+bench | 开发主用 |
| `release` | RelWithDebInfo（性能测量必须用这个） | 性能/演示 |
| `sanitize` | Debug + ASan + UBSan，无 testbed | 内存/UB 检查（Linux 全量；MSVC 只有 ASan） |
| `headless` | RelWithDebInfo，无 testbed，`PHYSICS2D_HEADLESS_ONLY=ON` | CI 无显示环境 |

`build.ps1` 还支持 `-BuildDir build/xxx`（并行开发时隔离构建目录）、`-Target <name>`、
`-NoTestbed`、`-ConfigureOnly`、`-RunTests`。

### 2.2 没有 Ninja / 不想用脚本时

仓库提供 Visual Studio 生成器预设（无需 Ninja、无需手动 vcvars，CMake 会自动找到 VS）：

```powershell
cmake --preset vs2022-debug
cmake --build --preset vs2022-debug
ctest --preset vs2022-debug
```

如果你已经在 "x64 Native Tools Command Prompt for VS 2022" 里，且 Ninja 在 PATH 中，
也可以直接 `cmake --preset debug`。

## 3. Linux / CI 构建

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build git \
                    libgl1-mesa-dev xorg-dev clang-format gdb
./scripts/build.sh debug --tests        # 等价于 configure + build + ctest
```

CI（`.github/workflows/ci.yml`）四个 job：双平台 debug/release 构建+测试、Linux ASan+UBSan、
headless 构建、以及 `core 分层守卫`（grep 确保 `physics2d_core` 不 include GLFW/ImGui/OpenGL）。

## 4. 常用命令

```powershell
# 只构建某个 target
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -Target physics2d_tests

# 格式化检查（clang-format 由 bootstrap 可选安装到 %LOCALAPPDATA%）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1 -Check

# 就地格式化
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/format.ps1

# 生成 compile_commands.json（clangd/VSCode 用）：configure 时已默认开启
```

## 5. 测试台操作（Week 1-4）

| 操作 | 按键 / 方式 |
| --- | --- |
| 平移 | 鼠标中键拖拽 |
| 缩放 | 滚轮（以鼠标位置为中心） |
| 复位视角 | `R` |
| 暂停 / 单步 | 左侧 ImGui 面板 |
| 重力调节 | 左侧 ImGui 滑块（实时写回 `World::SetGravity`） |
| 场景切换 | 左侧场景下拉框（`FallingBall` / `SphereStack` / `BoxStack`） |
| 恢复系数（FallingBall） | 场景面板的 restitution 滑块（实时 `World::SetRestitution`） |
| 显示接触点 / 法线 | 左侧 Display 开关（Contact points / Contact normals） |
| 重新落下 / 重置叠放 | 场景面板按钮 |

状态面板显示 shapes / contacts / 最大穿透（mm）、FPS、Step 耗时；`SphereStack`/`BoxStack`
场景面板还会显示每个球/方块的位置、速度、倾斜角。Week 4 的求解器仍只有法向冲量
（无摩擦、无 warm starting），所以 **BoxStack 的 10 层塔会塌**——这是 W4 的预期基线，
Week 5 加摩擦 + warm starting 后会站立（`v0.5-stable-stack` 里程碑）。

自动化冒烟测试（不依赖人眼看窗口）：

```powershell
# 单球落地：150 帧后 contacts=1，球心 y≈0.495（半径 0.5，穿透 5.000 mm = kLinearSlop）
.\build\debug\testbed\physics2d_testbed.exe --demo FallingBall --smoke 150 --hidden

# 3 球叠放：300 帧后 bodies=4 contacts=3，三球站立
.\build\debug\testbed\physics2d_testbed.exe --demo SphereStack --smoke 300 --hidden

# 10 层方块塔（W4 初版）：300 帧后塌落为 ground-only 接触（预期；W5 修复）
.\build\debug\testbed\physics2d_testbed.exe --demo BoxStack --smoke 300 --hidden
```

`--demo` 名字大小写/分隔符不敏感；未知名字打印可用列表并以退出码 2 退出。

## 6. 疑难杂症

| 症状 | 原因 / 处理 |
| --- | --- |
| `pwsh : 无法将"pwsh"项识别为...` | 本机只有 Windows PowerShell 5.1，把命令里的 `pwsh` 换成 `powershell` |
| `Ninja not found` | 跑一次 `scripts/bootstrap.ps1`，或用 `vs2022-*` 预设 |
| 把 Ninja 放进仓库 `tools/` 后报 `Permission denied` / `cl error D8050` | 本环境会限制"可执行文件位于工作区内部"的子进程写文件；Ninja 必须放在仓库外（VS 自带或 `%LOCALAPPDATA%`），`build.ps1` 已自动处理 |
| configure 报 `cl.exe ... is not able to compile a simple test program` | 确认 VS 装了 C++ 工作负载；用 `build.ps1`（自动 import vcvars）而不是裸 `cmake`；清掉 `build/<preset>` 重试 |
| 中文/空格路径 | 本机已验证 `E:\Big Fat Fish's Workstation\2D物理引擎项目\...` 可正常编译；若换到其它工具链报路径错误，建议放到纯 ASCII 路径 |
| testbed 启动失败、`--smoke` 返回 2 | 当前会话没有可用的 OpenGL 上下文（远程桌面/无显示 CI）。物理核心与单测不受影响；CI 只编译 testbed 不运行 |
| 依赖下载失败 | 检查网络代理；或 `-DPHYSICS2D_OFFLINE=ON` 复用已有 `_deps` |
| ctest 里 death tests 被 SKIP | 正常：death test 需要测试进程能写临时文件；本沙箱限制工作区内可执行文件的写入。CI（Linux/普通机器）会真正执行。把测试 exe 复制到工作区外运行即可看到它们通过 |
| ASan 链接/运行失败（MSVC） | MSVC 只支持 ASan；`sanitize` 预设已在开启 ASan 时移除与 `/fsanitize=address` 冲突的 `/RTC1`；UBSan 请用 Linux job。ASan 运行需要 vcvars 的 PATH（ASan runtime DLL） |
| **只改 header 后增量构建不重编（stale object）** | **本机特有**：中文 locale 的 cl.exe 把 `/showIncludes` 打成 `注意: 包含文件:`，而 CMake 往 `CMakeFiles/rules.ninja` 写的是 UTF-8 前缀，Ninja 按字节比较匹配不上 → depfile 为空，头文件改动不触发重编（实测 `VSLANG=1033` 在本机 VS 安装上无效）。**处理**：改过 header 后用 `build.ps1 ... -Clean`（`cmake --build --clean-first`），或做 canonical clean build（删 `build/<preset>`）。canonical 周验收一直是 clean build，所以历史数据未受影响 |
| `physics2d_testbed.exe` 里金字塔/随机盒场看不全 | 正常：W6 起 Camera 会在场景 Setup/Reset/切换时自适应取景（`FocusOnWorld`），`R` 回默认视野；若手动缩放过，按 `R` 重置 |

## 7. 目录速览

```text
Physics2D/
├── CMakeLists.txt / CMakePresets.json   # 构建入口（Ninja + VS2022 预设）
├── cmake/                               # 依赖、警告、Sanitizer 模块
├── include/physics2d/                   # 公共头（Math/Core/Collision/Solver）
├── src/                                 # 实现（core 静态库，零第三方依赖）
├── testbed/                             # OpenGL + ImGui 测试台（Layer 2）
├── tests/unit/                          # GoogleTest 单测
├── tests/regression/                    # 预留：headless 确定性回归（Week 6-7）
├── bench/                               # 预留：宽相基准（Week 6）
├── docs/                                # 架构、接口预留清单、开发日志
├── scripts/                             # bootstrap / build / format / pre-commit
└── .github/workflows/ci.yml             # CI
```
