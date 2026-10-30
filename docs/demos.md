# 演示与操作指南（Testbed Demos，Week 8）

> 一句话：`physics2d_testbed` 有 **8 个 demo + 4 个实时调试面板 + 5 个 README GIF**。本文讲
> 每个场景看什么、怎么点、每个面板有什么用、GIF 是怎么录的以及怎么复现。
>
> 相关文档：[`docs/testing.md`](testing.md)（30 秒验收 / 人工验收脚本 / smoke 基线）、
> [`testbed/README.md`](../testbed/README.md)（testbed 实现细节 / 面板实现）、
> [`docs/architecture.md`](architecture.md)（引擎侧数据流）。GIF 文件在 [`docs/images/`](images/)。

---

## 0. 30 秒版

```powershell
# 1) 构建（debug 已足够；录 GIF 用 release，数字和 README 一致）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release

# 2) 打开交互测试台（默认 FallingBall；下面的路径按构建目录替换）
.\build\debug\testbed\physics2d_testbed.exe
.\build\debug\testbed\physics2d_testbed.exe --demo Dominoes
.\build\debug\testbed\physics2d_testbed.exe --demo StressBenchmark

# 3) 一条命令重录全部 README GIF（Python 3 + Pillow 即可，不需要 ffmpeg）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
```

交互模式需要可用的桌面会话 + OpenGL 3.3；没有 GL 环境时 `--smoke` 会以 **exit code 2** 报告
"环境不支持"（物理与单测不受影响）。

---

## 1. 8 个 demo 一览

所有场景只用 `World` 的公共 API 搭建、只读 `World` 状态绘制，物理参数（迭代数、休眠阈值、摩擦、
restitution）一律用默认值——demo 是"展示 + 量化"，不是靠调参变好看。相机规则见
[`testbed/README.md` §Camera framing](../testbed/README.md)：小场景保持原点居中的 32×20 m 默认
视野（40 px/m），更大的场景在**每次构建/重置时自动取景一次**（之后由用户的平移/缩放接管）。

| # | Demo（`--demo`） | 场景内容 | 相机 | 面板可调参数 | 10 秒观看要点 | smoke 期望（见 §5） |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `FallingBall`（默认） | 动态圆 r=0.5、density 1，从 y=4 落到 y=0 的静态地面 | 默认 1x | Solver 面板的 **Restitution (e)**、restitution threshold；demo 面板 `Drop again` | e=0 落地即停；e=0.8 → 弹起约 3.4 m；速度低于阈值后不再弹 | 150 步：bodies=2、contacts=1、max_pen≈5.0 mm |
| 2 | `SphereStack` | 3 个 r=0.5 圆沿 y 轴精确相切堆叠（y=0.5/1.5/2.5） | 默认 1x | 每个球的 y / vy 滑杆、`Reset stack` | 球堆站立、有轻微抖动是正常；塌掉才是失败 | 300 步：bodies=4、contacts=3、max_pen≈3.1 mm、sleeping=3 |
| 3 | `BoxStack` | 10 个 1×1 m 方块（density 1、friction 0.3、restitution 0）在 y=0.5…9.5 精确堆叠 | 塔高超过默认视野 → 自动取景 | `Reset tower`、`Drop impact ball`、逐层 y/tilt 读数、`Stack intact` 判定 | **不抖、塔站立**：max_pen≈3.8 mm、顶层 y≈9.478 m；900 步后整岛入睡 | 600 步：bodies=11、contacts=10、contact_points=20、max_pen≈3.9 mm；900 步 awake=0 sleeping=10 |
| 4 | `FrictionRamp` | 1×1 m 方块放在可调倾角的静态斜面上 | 默认 1x | **Incline theta（0–45°）**、**Friction mu（0–1）** 实时生效；`Reset block` | 面板同时显示 μ vs tanθ 与理论判定（**STATIC iff μ > tanθ**）；15°/0.30 → 静止，μ 拉到 0.1 → 下滑 | 300 步：contacts=1、方块静止 |
| 5 | `Dominoes` | 20 块 0.30×1.0 m 多米诺，间距 1.0 m，初始无盒-盒接触 | 20 m 长链 → 自动取景 | 数量滑杆（10–40）+ `Rebuild`、`Push first`、`Reset`；面板显示 islands/awake/sleeping、woken after sleep、fallen、**chain reach** | 开场约 1 s（第 60 帧）整行入睡（冷色）→ 自动推倒第 0 块 → **整岛原子唤醒的波浪**（橙→绿）逐块传递；结束 20/20 倒下、chain reach 20、first skipped=-1 | 600 步：`woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1` |
| 6 | `IslandSleep500` | **100 座五层塔（共 500 个 1×1 m 盒）**，塔间 0.25 m 间隙（避免宽相邻近唤醒级联） | 62.5 m 宽 → 自动取景 | `Wake all`、`Reset`；面板显示 boxes/awake/sleeping、islands/contacts、Step（smoothed/last）、first all-asleep frame、early/awake/asleep 三段 Step 均值与 **speedup** | 500 块橙色（awake）→ 约 1.75 s（第 105 帧）全部变冷色（asleep）→ demo 面板给出实测 **awake vs asleep Step ms 与提速比**；`Wake all` 可反复观察入睡过程 | 600 步：islands=100、awake=0、sleeping=500、contacts=500、first_all_asleep_frame=105、speedup≈1.7x |
| 7 | `Pyramid200` | 完美 2D 方块金字塔，默认 **16 层=136 块**（层数滑杆 4–24 + `Rebuild`） | 20 m 高 → 自动取景 | 层数滑杆 + `Rebuild`；面板显示 bodies/contacts/max penetration/max tilt/top height + 站立判定 | 加载的底部行穿透几十 mm 属正常（载荷/迭代预算），看点是**不塌、不滑**：tilt < 2°、顶层高度基本保持 | 300 步：bodies=137、contacts=324、不塌、broadphase=SAP |
| 8 | `StressBenchmark` | 400 个随机尺寸 OBB（seed 42，0.2–1.0 m，随机角度，初速 ≤4 m/s）落在 36 m 宽场地 | 场地超过默认视野 → 自动取景 | 盒数滑杆（50–800）+ `Rebuild`、`Reset field`；面板显示 bodies/contacts/candidates/proxies/Step ms/FPS/已安装宽相 | 快速运动的颜色渐变（蓝=静止、橙=运动）；这是**压力场景**不是稳定 demo：底部堆叠穿透可达 170 mm 量级；在 Solver Settings 里切 None/BruteForce/SAP，**物理结果不变**、proxies/candidates 变化 | 300 步：bodies=401、contacts≈820、broadphase=SAP |

> 所有场景共用引擎级覆盖层：red ring+cross = 接触点（manifold anchor），yellow 0.5 m 箭头 = 接触
> 法线（A→B，默认关闭），cyan = 在 Contacts 面板点选的接触；`Body AABBs` 画的是宽相实际看到的
> **fat AABB**（tight + 0.1 m margin）。

---

## 2. 操作键（键盘 + 鼠标 + 面板）

交互模式下，左上 `Testbed Controls` 面板有全部按钮；键盘是快捷方式。**快捷键在 ImGui 捕获键盘
（输入框、ImGui demo 窗口）时自动失效**，不会和面板打架。

| 输入 | 作用 |
| --- | --- |
| 鼠标中键拖拽 | 平移相机 |
| 滚轮 | 以光标为中心缩放（0.1x–20x） |
| `R` | 相机回默认居中原点视野（不重建场景，也不重新自动取景） |
| `Space` | 暂停 / 继续 |
| `.` | **单步**：暂停时每按一次前进一个固定 1/60 s 步（用于逐帧看接触点/冲量） |
| `1` / `2` / `3` | 播放速度 1x / 0.5x / 0.25x（慢放不改变物理，只改变累积器喂入的模拟时间） |
| 场景下拉框 | 切换 demo（重建 World；Solver/材料/宽相设置保留） |
| `Pause` / `Step` / `Reset` / `Quit` 按钮 | 等价的鼠标操作 |
| `Wake all`（IslandSleep500 / Solver 面板） | 对所有动态体调用 `World::WakeBody`（整岛原子唤醒） |

单步与正常播放使用**同一个固定 dt**，所以"暂停后第 300 次单步"和"正常运行 300 步"的状态一致。

---

## 3. DebugUI 四个面板

> 面板实现见 [`testbed/DebugUI.h`](../testbed/DebugUI.h) / [`DebugUI.cpp`](../testbed/DebugUI.cpp)。
> 所有 Solver 面板的修改都写回 `World::SetSolverSettings`，**下一步固定步生效，不重建 body**。

### 3.1 Testbed Controls（左上）

- 场景选择下拉框（切换 = 重建 World）。
- 播放控制：`Pause/Play`、`Step`（仅暂停时可用）、`Reset`、速度 1x/0.5x/0.25x、`Quit`。
- 快捷键提示行。
- **Scene parameters**（每个 demo 的 `Ui()`）：见 §1 的"面板可调参数"列。

### 3.2 Status（右上）

只读状态：FPS / frame ms、smoothed Step ms + steps/frame、场景名、bodies/shapes、**islands / awake /
sleeping**（W7）、touching contacts / contact points / max penetration、已安装宽相 `Name()` + 模式 +
**proxies / candidate pairs**（W6）、gravity、solver 迭代数与 warm/split/sleeping/island-solve 标志、
固定 dt、暂停/速度状态。

### 3.3 Solver Settings（右下）

- **Broad phase**：下拉 `Auto (by body count)` / `None (O(N^2) fallback)` / `BruteForce` / `SAP`；显示
  实际安装的实现、proxies、candidate pairs、fat AABB margin。`Auto` 在每次 build/reset 后按 body
  数分派（<32 BruteForce，≥32 SAP，来自 Release bench 交叉点）；手动选择会一直保留到选回 Auto。
- **Iterations**：velocity / position 迭代（0–40，默认 8/3）。
- **Warm starting**：开关 + factor（0–1，默认 1.0）。
- **Split impulse**：开关 + beta + slop；关闭时显示 Baumgarte（两者不叠加）。
- **Contact tolerances**：linear slop、max linear correction、SAT cache、restitution threshold。
- **Islands & sleeping**：`Use island solving`、`Enable sleeping`、实时 islands/awake/sleeping、
  `Wake all bodies`。
- **Gravity**：Gravity Y（-30…0）。
- **Material defaults**：default friction / restitution + `Apply to current scene`（场景 build 时自动
  应用，这就是"把 friction 拉到 0 看塔塌"的实验入口）。
- **Display**：World axes、**Body AABBs**、**Contact points**、**Contact normals**、**Velocity
  arrows (overlay)**、Dear ImGui demo window。

### 3.4 Contacts（左下）

实时接触表（每步重建，按 body 对而不是行号标识）：pair 名、point 数、穿透（mm）、法线（A→B）、累计
法向/切向冲量。**点击一行** → 场景中该接触画 cyan 环 + 1 m 法线；面板下方给出选中接触的逐点
**累计法向冲量柱状条**、**切向冲量 + 库仑摩擦锥占用**（|accT|/(μ·accN)，0%=静摩擦、100%=滑动）、
法线/μ/深度。再点一次或 `Clear highlight` 取消。

---

## 4. README GIF 清单

5 个 GIF，全部在 [`docs/images/`](images/)，由 `scripts/record_gifs.ps1` **一条命令**从确定性 smoke
循环录制并编码（不截屏、不调物理参数）。

| GIF | 内容 | 采样（smoke/every/start） | 分辨率 | 捕捉帧 / GIF 帧 | 时长 | 大小 |
| --- | --- | --- | --- | --- | --- | --- |
| [`dominoes.gif`](images/dominoes.gif) | 20 块多米诺：入睡 → 第 60 帧推倒第一块 → 整岛唤醒波传递到最后一格（**建议放最前**） | 700 / 7 / 20，`--capture-hide-ui`（面板会挡住 20 m 长链两端） | 720×180（banner 裁切） | 98 / 94 | 7.84 s | 116.8 KB |
| [`box_stack.gif`](images/box_stack.gif) | 10 层方块塔：不抖、站立；接触点覆盖 + 面板 max penetration/顶层高度读数 | 900 / 8 / 1 | 720×450 | 113 / 113 | 9.04 s | 437.4 KB |
| [`sap_stress.gif`](images/sap_stress.gif) | 400 个随机 OBB 的 SAP 压力场景（settling 尾段），面板显示 `broadphase: SAP` / proxies / candidate pairs / Step ms | 400 / 2 / 250，`contacts=off,axes=off` | 720×450 | 76 / 76 | 6.08 s | 579.2 KB |
| [`island_sleep.gif`](images/island_sleep.gif) | 500 盒 / 100 岛：awake 500→0，面板给出 early/awake/asleep 三段 Step ms 与 speedup | 600 / 8 / 1 | 720×450 | 75 / 75 | 6.00 s | 222.5 KB |
| [`contact_debug.gif`](images/contact_debug.gif) | 接触调试：接触点 + 法线箭头覆盖 + Contacts 冲量表（可调试性展示） | 300 / 4 / 1，`normals=on` | 720×450 | 75 / 75 | 6.00 s | 338.7 KB |

合计 **≈1.65 MB**（预算：单个 ≤600 KB、总计 ≤2.5 MB）。帧率统一 **12.5 fps**（80 ms/帧），循环播放。
"捕捉帧"是 testbed 写出的 PPM 数；"GIF 帧"可能更少，因为 Pillow 会把**完全相同的连续帧**合并并累加
显示时长（例如 dominoes 前段静止的入睡画面）。

> 预算换来的取舍（诚实记录）：`sap_stress` 的内容最贵——400 根带**速度色带**的线框每帧都在变，整个
> 下落阶段编成 720×450/16 色的 GIF 会 >1.5 MB。因此它截取 **settling 尾段（第 250 帧起）**、按
> `every=2` 慢放采样、关掉 contact 覆盖，并把调色板压到 16 色才压进 600 KB；面板数字仍是真实运行时
> 读数。`friction_ramp` 的 μ vs tanθ 需要拖动滑杆/改角度，无输入的 smoke 循环无法演示，故不出 GIF，
> 交互验收见 [`docs/testing.md` §4](testing.md)。

### 4.1 一条命令重录全部

```powershell
# 前置：已构建 release（或 debug）testbed；Python 3 带 Pillow；有桌面会话 + OpenGL 3.3
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1

# 指定构建目录（并行开发时常见）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 `
    -Testbed build/w8tbrel/testbed/physics2d_testbed.exe

# 只重录其中几个（-Only 支持逗号分隔）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 -Only dominoes,island_sleep

# Pillow 不在系统 python 里时，显式指定解释器（例如 DSH 环境捆绑的 Python）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 -PythonExe <python.exe 的绝对路径>
```

脚本行为：对每个 GIF

1. 清空并重建 `build/capture/<name>/`（**gitignored**，只在这里做删除，脚本会校验路径在 capture 根内）；
2. 运行 `physics2d_testbed --demo <scene> --smoke N --hidden --capture-dir ... --capture-every K
   --capture-start S`（+ 可选 `--capture-hide-ui` / `--capture-display`），校验 exit 0 与
   `[capture] frames=N` 行；
3. 用脚本内嵌的 Python/Pillow 编码器把 PPM 转成 `docs/images/<name>.gif`（Lanczos 缩小 + 全局调色板 +
   无 dither + optimize 帧间 delta）；
4. **删除 PPM 中间文件**；
5. 最后打印每个 GIF 的帧数/大小与总计，超过单张 600 KB 或总计 2.5 MB 会报错退出。

> 脚本会先跑一次 `physics2d_testbed --help` 检测二进制是否带 Week 8 的 `--capture-dir` 支持；如果
> `build/release`、`build/debug` 还是旧 tag 的产物（没有捕获参数），脚本会跳过它们并提示先重建，或用
> `-Testbed` 指向新构建目录。默认候选顺序是 `build/release` → `build/debug`。

脚本内嵌的编码器源码会写到 `build/capture/encode_gifs.py`（同样 gitignored）——所以仓库里只需要
`scripts/record_gifs.ps1` 一个文件。所有采样参数都在脚本顶部的 `$specs` 表里，改那一处即可。

### 4.2 捕获机制（testbed 侧）

新增的 CLI（`--help` 可见，详见 [`testbed/README.md`](../testbed/README.md)）：

| 参数 | 作用 |
| --- | --- |
| `--capture-dir DIR` | 启用帧捕获：每帧在 **world + ImGui 都画完、`glfwSwapBuffers` 之前**用 `glReadPixels(GL_BACK, GL_RGB)` 读回，翻转 y 后写 `frame_000123.ppm`（二进制 PPM/P6，1-based 帧号）。需要同时给 `--smoke N`。默认关闭 = 零开销。 |
| `--capture-every K` | 每 K 个 smoke 帧捕获一帧（默认 1）。 |
| `--capture-start N` | 从第 N 帧（1-based）才开始捕获，默认 1（用于跳过下落/开场）。 |
| `--capture-hide-ui` | 捕获时跳过 DebugUI 面板（只录场景），用于 dominoes。 |
| `--capture-display OPTS` | 覆盖显示开关，逗号分隔 `key=on/off`，key ∈ `contacts,normals,aabb,axes,velocities`（与 Solver Settings 的 Display 勾选相同）。用于 `sap_stress` 关 contact 覆盖、`contact_debug` 开法线。 |

退出码与 smoke 一致：`0` 成功；`2` 无 GL 环境 / 未知 demo；`1` 参数错误。成功时最后会多打一行
`[capture] frames=<写入帧数> every=<K> start=<S> dir=<目录>`，录制脚本用这一行做交叉校验。

**为什么在 swap 之前读**：`glReadPixels(GL_BACK)` 在 swap 之前读到的是刚画完的这一帧；swap 之后 back
buffer 已属于下一帧（未定义内容）。同时要先把 `GL_PACK_ALIGNMENT` 设为 1——RGB 行字节数不保证是 4 的
倍数，默认 4 字节对齐会错行。

---

## 5. Smoke 期望值（录制/回归的机器可判定基线）

完整命令与期望输出在 [`docs/testing.md` §5](testing.md)。录制脚本只做**采集与编码**，物理正确性仍由
smoke 的 `[smoke] demo=...` 一行断言。逐个 demo 的关键期望（本机 debug，允许噪声）：

| Demo | 关键字段 |
| --- | --- |
| `FallingBall --smoke 150` | `contacts=1`、`max_penetration_mm≈5.0` |
| `SphereStack --smoke 300` | `bodies=4 contacts=3 max_penetration_mm≈3.1 sleeping=3` |
| `BoxStack --smoke 600` | `bodies=11 contacts=10 contact_points=20 max_penetration_mm≈3.9 max_center_y≈9.478` |
| `FrictionRamp --smoke 300` | `contacts=1`、方块静止（θ=15°、μ=0.30） |
| `Dominoes --smoke 600` | `woken_after_sleep=20/20 fallen=20/20 chain_reach=20 first_skipped=-1` |
| `IslandSleep500 --smoke 600` | `islands=100 awake=0 sleeping=500 contacts=500 first_all_asleep_frame=105 speedup≈1.7x` |
| `Pyramid200 --smoke 300` | `bodies=137 contacts=324 broadphase=SAP`、不塌 |
| `StressBenchmark --smoke 300` | `bodies=401 contacts≈820 broadphase=SAP`、与宽相选择无关 |

录制时不要改物理参数来"更好看"：唯一允许的差异是**相机/显示开关/采样步长**，这些都记录在
`scripts/record_gifs.ps1` 的 `$specs` 里。

---

## 6. 截图 / 单帧复现

需要某张静帧（例如文档里配图）时，不必录 GIF：

```powershell
# 1) 只捕获一帧（第 400 帧，== 400 个固定步；换成你自己的构建目录也可）
.\build\release\testbed\physics2d_testbed.exe --demo IslandSleep500 --smoke 401 --hidden `
    --capture-dir build/capture/still --capture-every 1000 --capture-start 400

# 2) PPM -> PNG（Pillow；也可直接看 .ppm）
python -c "from PIL import Image; Image.open('build/capture/still/frame_000400.ppm').save('docs/images/still.png')"
```

注意 `docs/images/*.png` 也在 `.gitignore` 白名单内（`!docs/**/*.png`），但**只提交真正需要的小图**，
中间 PPM 永远不要提交（单帧 1280×800 的 PPM 就有 3.07 MB）。

---

## 7. 常见录屏 / 编码坑

1. **隐藏窗口读 back buffer**：本机（NVIDIA 3.3）`--hidden` 下 `glReadPixels(GL_BACK)` 可靠，所以录制
   时不会闪窗口。若换到某些驱动/远程桌面读回全黑或花屏，**同一个命令去掉 `--hidden`** 即可用可见
   窗口录制（物理与帧序完全不变，只是窗口可见）。
2. **读的时机**：必须在 `glfwSwapBuffers` 之前读（见 §4.2）；在 swap 之后读会得到下一帧的未定义内容。
3. **确定性**：只有 `--smoke` 模式保证"1 帧 = 1 个 1/60 s 步"；**不要用交互模式录制**（累积器吃真实
   帧时间，同样的帧数对应不同步数）。录制脚本全部走 smoke。
4. **中间文件体积**：1280×800×3 B ≈ 3.07 MB/帧；一轮全录 ~360 帧 ≈ 1.1 GB。脚本按 GIF 逐个
   "捕获→编码→删除"，峰值只有单个 GIF 的量；`build/capture/` 已被 gitignore。
5. **GIF 大小超预算**按这个顺序调整：① 采样窗口换成运动更慢的一段（本仓库 `sap_stress` 就是这么
   做的）→ ② 用 `--capture-display` 关掉与主题无关的覆盖层（接触点/坐标轴）→ ③ 用
   `--capture-hide-ui` + 裁切，或整体降宽度（720→640）→ ④ 降 `--colors` 阶梯（脚本超预算时会自动
   降）。帧率（12–15 fps）与时长（6–10 s）是硬约束，不要靠删帧解决。
6. **颜色太少会花屏/发色**：全局调色板 + 无 dither 时，16 色左右 ImGui 文本会出现彩色镶边（仍可读）；
   128 色基本无损。脚本的 `Colors` 阶梯就是为这个准备的。
7. **`-Only a,b` 的引号**：用 `powershell -File` 调用时逗号列表会作为**一个字符串**传入，脚本内部已
   兼容（直接 `powershell -File ... -Only dominoes island_sleep` 也可）。仓库命令统一用 Windows
   PowerShell 5.1（见 [`docs/dev-setup.md`](dev-setup.md) §6）。
8. **Pillow 缺失**：报错会提示 `-PythonExe`；DSH 环境里可用 `load_workspace_dependencies` 的捆绑 Python
   （自带 Pillow）。
9. **不要截屏录窗口**：OBS/截图工具会引入非确定性帧（掉帧、窗口遮挡、DPI 缩放），并且无法和 smoke 的
   步数对齐；用 `--capture-*` 的确定性路径。
10. **ImGui 默认字体不含中文**：面板标签都是英文（这也是 GIF 里 UI 的样子）；中文文档在 `docs/`。

---

## 8. 验收清单（改了 testbed / 录了新 GIF 后）

- [ ] `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -BuildDir build/w8tb -RunTests`
      → 0 警告、ctest 全绿（测试数随 W8 边界测试增长，以当次输出为准）；
- [ ] 8 个 demo 的 smoke 全 exit 0（命令与期望见 [`docs/testing.md` §5](testing.md)）；
- [ ] `--broadphase none|bf|sap` 的 body/contact 输出逐字节一致（只有 candidate_pairs 不同）；
- [ ] `scripts/record_gifs.ps1` 一条命令跑通，5 个 GIF 都 ≤600 KB、总计 ≤2.5 MB；
- [ ] 抽查 GIF 中间帧：画面正常、面板数字可读（Pillow `seek` 抽帧存 PNG 看，或直接打开文件）；
- [ ] `docs/images/` 里没有 PPM；提交的只有 GIF（`.gitignore` 已白名单 `docs/**/*.gif`）。
