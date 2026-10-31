# Physics2D 数学推导（v1.0-release · Week 8）

> **读者假设**：会看线性代数与微积分，但不熟悉物理引擎。每个公式都给出完整推导步骤，
> 每一步都能手推；每个常数/数字都标注它来自哪个源文件的哪一段，或哪次实测（`docs/devlog.md`）。
> 符号严格遵循项目宪法 `AGENTS.md` §1.3：接触法线 **A → B**、`vRel = (vB + ωB×rB) − (vA + ωA×rA)`、
> 右手系（+x 右、+y 上、逆时针为正）。
>
> **文档状态**：对应 tag `v1.0-release` 的代码（Week 1-7 已实现；Week 8 是文档/边界契约周，
> 除"非法输入校验与 assert"外不改物理）。文中所有实现引用都可以在仓库里搜到；所有实测数字的测量环境
> 统一为 Windows 11 22631 / i7-12650H / 16 GB / MSVC 19.44 / CMake 3.31.12 / Ninja 1.12.1
> （Debug 或 RelWithDebInfo 在每处单独标注）。

---

## 0. 符号表与三个约定

| 符号 | 含义 | 代码中的名字 |
| --- | --- | --- |
| `m`, `1/m` | 质量、逆质量 | `Body::mass`, `Body::invMass` |
| `I`, `1/I` | 绕**旋转中心**（body 原点）的转动惯量、逆惯量 | `Body::inertia`, `Body::invInertia` |
| `p`, `q`, `xf` | 位置、旋转、变换 | `Vec2`, `Rot{c,s}`, `Transform` |
| `v`, `ω` | 线速度、角速度（标量，沿 +z） | `Body::linearVelocity`, `Body::angularVelocity` |
| `n` | 单位接触法线，**从 A 指向 B** | `Manifold::normal` |
| `rA`, `rB` | 从 A/B 的 body 原点到接触锚点的世界杠杆臂 | `ContactConstraintPoint::rA/rB` |
| `vRel` | 接触点相对速度 `(vB + ωB×rB) − (vA + ωA×rA)` | `ContactSolver::ComputeRelativeVelocity` |
| `K_n`, `K_t` | 法向/切向有效质量（`Δ(vRel·n)/Δλ`） | `ComputeEffectiveNormalMass/TangentMass` |
| `λ` | 冲量大小（`J = λ·n` 或 `λ_t·t`） | `lambda`, `normalMass = 1/K_n` |
| `μ`, `e` | 摩擦系数、恢复系数 | `Contact::friction`, `Contact::restitution` |
| `dt` | 固定物理步长 `1/60 s` | `kFixedTimeStep` |
| `β`, slop | 位置修正系数、穿透死区 | `kSplitImpulseBeta`, `kLinearSlop` |

**约定 1：法线 A → B，`vRel` 是 B 减 A。** 于是 `vRel·n > 0` 表示两个接触点正在分开，
`vRel·n < 0` 表示还在接近。这个符号决定了"正冲量 = 推开"，一行也不能反：
`AGENTS.md` §1.3 与 `include/physics2d/Collision/Manifold.h` 的头部注释都是规范。

**约定 2：2D 角速度是标量 `ω`，`ω × r` 用 `Cross(ω, r)` 重载。** `include/physics2d/Math/Vec2.h`
固定了三个 `Cross` 重载，混用是 2D 引擎最常见的 AI bug：

```text
Cross(a, b) -> float : a.x*b.y - a.y*b.x        （二维叉积的 z 分量 / 有向面积）
Cross(a, s) -> Vec2  : ( s*a.y, -s*a.x)         （向量 × 标量）
Cross(s, a) -> Vec2  : (-s*a.y,  s*a.x)         （标量 × 向量 = s * Skew(a)）
```

`Skew(v) = (-v.y, v.x)` 是逆时针转 90°，即 `Cross(1, v)`。`ω × r` 用 `Cross(float, Vec2)`。

**约定 3：所有魔法数字集中在 `include/physics2d/Core/Settings.h`。** 本文引用的常数全部来自该文件，
例如 `kLinearSlop = 0.005 m`、`kVelocityIterations = 8`、`kPositionIterations = 3`。
运行时可改的子集在 `SolverSettings`（`World::SetSolverSettings`）。

### 0.1 代码索引：一个公式该去哪里看

| 主题 | 实现入口 |
| --- | --- |
| 积分 | `src/Core/World.cpp` 的 `IntegrateForces` / `IntegratePositions` |
| 质量属性 | `src/Collision/PolygonShape.cpp::ComputePolygonMass`、`CircleShape.cpp::ComputeCircleMass`、`World::AttachShape` |
| 法向/摩擦求解 | `src/Solver/ContactSolver.cpp`（`ComputeEffectiveNormalMass`、`SolveConstraintPoint`） |
| 位置层（split impulse） | `src/Solver/ContactSolverPosition.cpp::SolvePosition` |
| SAT + 裁剪 | `src/Collision/SAT.cpp`（`FindMaxSeparation` / `CollidePolygons`）、`src/Collision/Clipping.cpp` |
| 宽相 SAP | `src/Collision/BroadPhaseSAP.cpp` |
| 岛屿/休眠 | `src/Solver/Island.cpp`、`src/Core/World.cpp` 的 `BuildIslands` / `UpdateSleep` / `WakeIslandOfBody` |

---

## 1. 运动学与积分

### 1.1 刚体的状态与位置积分

2D 刚体的状态是位置 `p ∈ R²`、旋转 `q`、线速度 `v`、角速度 `ω`。
本项目不存角度、存 `Rot{c = cos θ, s = sin θ}`（`include/physics2d/Math/Rot.h`）：
角度在长时间积分后会累积误差且需要 wrap，而 `(c,s)` 每次乘法后归一化即可长期稳定。

刚体上一点 `P`（杠杆臂 `r = P − p`）的**点速度**：

```text
v_P = v + ω × r
```

2D 中 `ω = (0,0,ω)`，`ω × r = (−ω·r.y, ω·r.x) = Cross(ω, r)`。这是后面所有接触速度的基础。

### 1.2 semi-implicit Euler：为什么位置用"新速度"

`World::Step` 的固定流水线（`include/physics2d/Core/World.h` 顶部 11 阶段）里，
先积分速度（phase 1），经过求解器后再用**更新后的**速度积分位置（phase 7）：

```text
（phase 1）v_{n+1} = v_n + dt * (g + F/m)
          ω_{n+1} = ω_n + dt * (τ/I)
（求解器 5-6 修改 v_{n+1}、ω_{n+1}）
（phase 7）p_{n+1} = p_n + dt * v_{n+1}
          q_{n+1} = Normalize(q_n · Rot(ω_{n+1}·dt))
```

**为什么不用显式 Euler（先按旧速度更新位置）？** 用简谐振子 `x'' = −ω²x` 做局部模型，
状态向量 `(x, v)`：

```text
显式 Euler:    x' = x + dt·v,        v' = v − dt·ω²·x
               M_exp = [ 1      dt ]
                       [ −dtω²   1  ]       det(M_exp) = 1 + dt²ω² > 1

半隐式 Euler:  v' = v − dt·ω²·x,     x' = x + dt·v'
               M_sym = [ 1 − dt²ω²   dt ]
                       [ −dtω²       1  ]   det(M_sym) = 1
```

显式 Euler 的放大因子每步把"相空间面积"放大 `1 + dt²ω²`，能量单调增长（弹簧会越振越大）；
半隐式 Euler 的行列式为 1，当 `dt·ω < 2` 时特征值是复共轭且 `|λ|² = det = 1`，
能量只在有界范围内振荡。这就是"symplectic Euler"名字的由来，也是
`IntegratePositions` 的注释里"用旧速度会获得能量"的量化依据。

单位验证（`tests/unit/test_world_step.cpp::FreeFallMatchesSemiImplicitAnalyticSolution`）：
从 `y_0 = 10 m`、`v_0 = 0`、`g = −10 m/s²`、`dt = 1/60` 开始重复 `n` 步：

```text
v_n = −g·n·dt
y_n = y_0 − g·dt²·(1 + 2 + … + n) = y_0 − g·dt²·n(n+1)/2
n = 6: y_6 = 10 − 10·(1/3600)·21 = 9.9416667 m,  v_6 = −1 m/s      （测试断言）
```

显式 Euler 会得到不同的 `y_n`（落后/超前半个 `g·dt²`），该测试把 phase 顺序钉死。

### 1.3 单步位移钳制（隧穿防线）

`IntegratePositions` 在两个地方做钳制（`Settings.h`）：

```text
|dt·v| 的方向保留，长度缩到 kMaxTranslation = 2.0 m
|dt·ω| 夹到 ±kMaxRotation = π/2 rad
```

这不是速度钳制：速度不变，所以下一帧仍会被钳到 2 m/步（`WorldTest.LinearDisplacementIsClampedPerStep`
断言 1e6 m/s 的物体每步恰好前进 2 m）。代价是没有真 CCD：速度大于 `kMaxTranslation/dt = 120 m/s`
的物体可能在一帧内穿过薄障碍，这是**明确记录的已知限制**（见 §10）。

### 1.4 固定步长 + accumulator

物理永远只看到 `dt = kFixedTimeStep = 1/60 s`（`Settings.h`）。渲染帧率是可变的，
转换发生在**唯一入口** `StepAccumulator::Advance(frameTime)`（`include/physics2d/Core/TimeStep.h`）：

```text
accumulator += frameTime
steps = min(floor(accumulator / dt), kMaxStepsPerFrame)
accumulator -= steps * dt
（触到上限时用 fmod 丢弃超出的整步，保留不足一步的余量 → 下一帧不会补跑，alpha 连续）
alpha = accumulator / dt      ∈ [0,1)   （渲染插值用，W7 起提供 GetAlpha()）
```

`kMaxStepsPerFrame = 5` 是"死亡螺旋"保护：如果一帧已经卡到要跑 6 步，多出来的时间**丢弃**
（不排队），否则下一帧会更卡、要跑更多步。

**为什么必须固定 dt？** 三个机制都假设它是常数：(a) warm starting 继承的冲量对应"上一帧解"，
dt 突变会让继承的冲量与新的速度目标不匹配；(b) split impulse 的 bias 是**速度目标** `(β/dt)·excess`，
位置层的净修正量 `β·excess` 也按同样 dt 积分；(c) 确定性回归（`tests/regression`）要求同样的输入
产生逐位相同的输出。`World::Step(dt)` 只接受调用方传入的固定 dt，可变步长在架构上被禁止。

> **实测（`docs/devlog.md` W1）**：自由落体与解析解一致；W7 的 10 盒塔在 600 步（10 s）内
> 顶层 y = 9.47813 m（起始 9.5 m，2.2 cm = 10 个接触的 slop 累积），最大穿透 5.223 mm。

### 1.5 延伸思考

1. **"为什么用半隐式 Euler？显式 Euler 差在哪？"**
   答：显式 Euler 对振荡系统的放大矩阵行列式 `1 + dt²ω² > 1`，能量单调增长；半隐式（symplectic）
   行列式为 1，`|λ| = 1`，能量有界。代价是一阶精度、`dt` 固定，但对刚体堆叠足够稳定，且每步只做
   两次乘加——比 RK4 便宜一个数量级。
2. **"固定步长和可变步长怎么选？"**
   答：物理只用固定步长（warm starting / split impulse / 确定性都依赖 dt 恒定），用 accumulator 把
   帧时间换成整数步；上限 5 步防死亡螺旋，`alpha` 留给渲染插值。
3. **"高速物体会穿墙吗？"**
   答：默认不做 CCD。两道防线是单步位移钳制（≤2 m/步）和宽相 fat AABB；真正的解法是 TOI/保守推进，
   在 Roadmap 里。我可以给出 1e6 m/s 的实测：每步恰好移动 2 m，不产生 NaN。

---

## 2. 刚体质量属性

### 2.1 从三角形分解推导凸多边形面积、质心、惯性矩

设顶点 `P_0 … P_{n−1}` 按**逆时针（CCW）**排列。把多边形看成以原点为一个顶点、
每条边 `(P_i, P_{i+1})` 为一条边的**有向三角形**之和。有向面积：

```text
A_i = 1/2 · Cross(P_i, P_{i+1}) = 1/2 · (P_i.x·P_{i+1}.y − P_i.y·P_{i+1}.x)
A   = Σ A_i = 1/2 · Σ Cross(P_i, P_{i+1})                       （鞋带公式）
```

CCW 时每个 `Cross(P_i, P_{i+1}) > 0`（原点多边形内部一侧），总和为多边形面积的正值；
顺时针则全负——**符号方向就是靠这个交叉积定义的**，不做 `abs`。

每个三角形 `(0, P_i, P_{i+1})` 的质心是 `(P_i + P_{i+1})/3`（三个顶点平均）。按面积加权：

```text
C = ( Σ A_i · (P_i + P_{i+1})/3 ) / A
  = [ (1/2) Σ Cross(P_i,P_{i+1}) · (P_i + P_{i+1}) / 3 ] / [ (1/2) Σ Cross(P_i,P_{i+1}) ]
  = Σ (P_i + P_{i+1})·Cross(P_i,P_{i+1}) / (3 · twiceArea),   twiceArea = 2A
```

对应 `src/Collision/PolygonShape.cpp::SetPolygon` 里的
`centroidSum * (1.0f / (3.0f * twiceArea))`。

**绕原点的极惯性矩**（面积分 `J_O = ∬ (x² + y²) dA`）。对三角形 `(0, a, b)` 用格林公式展开：

```text
∬_T (x² + y²) dA = (1/6) · Cross(a, b) · ( |a|² + a·b + |b|² )
```

验证：`a = (1,0), b = (0,1)`，左边逐步积分 `∫₀¹∫₀^{1−x}(x²+y²)dydx = 1/12 + 1/12 = 1/6`，
右边 `(1/6)·1·(1+0+1) = 1/6` ✓。对每条边求和（`a = P_i, b = P_{i+1}`）：

```text
J_O = (1/6) Σ Cross(P_i,P_{i+1}) · ( P_i·P_i + P_i·P_{i+1} + P_{i+1}·P_{i+1} )
I_O = ρ · J_O = (ρ/12) Σ Cross(P_i,P_{i+1}) · ( |P_i|² + P_i·P_{i+1} + |P_{i+1}|² )
```

这就是 `ComputePolygonMass` 的 `inertiaOrigin = (density / 12) * inertiaIntegral`（`cross * (Dot(current,current) +
Dot(current,following) + Dot(following,following))` 的和）。

### 2.2 平行轴定理（两处用法，方向相反）

平行轴定理：绕任意点 `O` 的惯量 = 绕质心 `C` 的惯量 + `m·|C − O|²`。

```text
I_O = I_C + m·|C − O|²          ⇒  I_C = I_O − m·|C|²   （O 取 body 原点）
```

实现里有**两种方向**的用法，别混：

1. `ComputePolygonMass` 已经算出绕原点的 `I_O`，返回给调用方的是**绕质心**的
   `inertia = I_O − m|C|²`（`MassData.inertia` 契约：关于 `MassData.center`）。
2. `World::AttachShape` 把形状的质心惯量折算到**body 原点**（本引擎的旋转中心）：
   `newInertia = massData.inertia + mass·|massData.center|²`。
   圆形 `localCenter ≠ 0` 时也用同一式子（W2 的简化：body 原点即旋转中心）。

### 2.3 圆形与平面

`ComputeCircleMass`（`src/Collision/CircleShape.cpp`）用闭式解，不走三角形分解：

```text
A = πr²,  m = ρA,  I_C = ½·m·r²        （实心圆盘绕自身圆心，标准结果）
```

`PlaneShape` 是无限半空间，质量/惯性记为 0（无穷大质量用 **0 逆质量** 表示，绝不出现 inf，
这样积分器/求解器里所有乘法都保持有限；`WorldShapeTest.StaticBodyStaysInfiniteMassAfterAttach` 钉死）。

### 2.4 顶点顺序、法线与符号方向

`PolygonShape` 的存储契约（`include/physics2d/Collision/PolygonShape.h`）：
CCW、无重复首尾点、`normals[i]` 是边 `vertices[i] → vertices[i+1]` 的**单位外法线**。

对 CCW 多边形，边方向 `e = P_{i+1} − P_i` 的外法线 = 边方向**顺时针转 90°**（即旋转 −90°）：

```text
n_out = Normalize( (e.y, −e.x) )        ← 代码：Vec2{v[next].y − v[i].y, v[i].x − v[next].x}
```

`SetPolygonAsBox` 固定从 `(+hx, −hy)` 开始按 CCW 生成 4 个顶点（`BuildBoxVertices`），
所以同一个盒子的 edge index 永远相同——这是 `ContactID` 帧间稳定（warm starting 的前提）的基础。

### 2.5 验证与手算案例

| 形状 | 手算结果 | 测试位置 |
| --- | --- | --- |
| 2×2 方块（hx=hy=1，ρ=1） | `A=4, m=4, I_C = m(2²+2²)/12 = 8/3` | `test_polygon_shape.cpp` |
| 4×2 矩形（hx=2, hy=1，ρ=1） | `m=8, I_C = 40/3` | 同上 |
| 等边三角形（边长 2，ρ=1） | `A=√3, C=(0,√3/3), I_C=√3/3` | 同上 |
| 正八边形（R=1） | `A = 0.5·8·sin(π/4) = 2√2` | 同上 |

`SetPolygon` 对以下输入**一律拒绝并返回 false**（不静默修复，`docs/dev-interfaces.md` §5）：
顶点数不在 [3, 8]、非有限坐标、相邻顶点距离 < `kLinearSlop`（重复/过近）、冗余共线顶点、
顺时针绕向、凹顶点、面积 ≤ `kLinearSlop² = 2.5e-5 m²`。Week 8 又补了**溢出保护**：
坐标有限但鞋带和/边长平方溢出 float（如半宽 1e19）时同样拒绝——否则 `Normalized()` 会
返回 `(0,0)`，形状看似有效、实际所有 SAT 轴失效（见 §10，测试
`test_boundary_solver.cpp::BoundaryPolygon.FiniteButOverflowingCoordinatesAreRejected`）。

### 2.6 延伸思考

1. **"多边形惯性矩为什么可以用鞋带公式？"**
   答：把多边形分解成以原点为公共顶点的有向三角形，对每个三角形用格林公式
   `∬(x²+y²)dA = (1/6)Cross(a,b)(|a|²+a·b+|b|²)`，求和即得；CCW 保证每一项符号一致。
2. **"质心和旋转中心不一致会怎样？"**
   答：需要平行轴定理把惯量从质心折算到旋转中心（本引擎的 body 原点）；
   本引擎 `AttachShape` 已折算，`MassData.center` 就是局部质心。
3. **"为什么拒绝而不是修复退化多边形？"**
   答：设计原则是"绝不静默修正用户输入"（`AGENTS.md` §1.4）：静默合并重复顶点会让调用方的数据
   错误永远发现不了；返回确定的 false，调用方自己修数据。

---

## 3. 冲量与约束（Sequential Impulse）

### 3.1 从相对速度变化推出有效质量 `K_n`

法向约束是**不等式**：接触点不允许相互接近。

```text
C_dot = vRel · n ≥ 0
vRel  = (vB + ωB × rB) − (vA + ωA × rA)          （B 减 A，n 从 A 指向 B）
```

施加法向冲量 `J = λ·n`：A 收到 `−J`、B 收到 `+J`（牛顿第三定律，动量守恒）。质点 + 刚体的
速度响应：

```text
Δv_A = −invMassA · J,         Δω_A = −invIA · Cross(rA, J)
Δv_B = +invMassB · J,         Δω_B = +invIB · Cross(rB, J)
```

接触点速度的变化（只看法向分量 `·n`）：

```text
Δ(vRel·n) = Δ[(vB + ωB×rB)·n] − Δ[(vA + ωA×rA)·n]
```

对 B 项：`ΔvB·n = λ·invMassB`；角速度项 `ΔωB × rB`。2D 中
`ΔωB × rB = ΔωB · Skew(rB)`，而 `Dot(Skew(rB), n) = Cross(rB, n)`，所以

```text
Dot(ΔωB × rB, n) = invIB · λ · Cross(rB,n) · Cross(rB,n) = λ·invIB·Cross(rB,n)²
```

A 项同理（符号相反）。合并：

```text
Δ(vRel·n) = λ · [ invMassA + invMassB + invIA·Cross(rA,n)² + invIB·Cross(rB,n)² ]
         = λ · K_n
```

即 `K_n = Δ(vRel·n)/Δλ`。代码：`ContactSolver::ComputeEffectiveNormalMass`，注释里
`ComputeEffectiveNormalMass` 的推导与此一致；`tests/unit/test_contact_solver.cpp` 用三组手算
（零杠杆、单端杠杆、双端杠杆）钉死公式。

### 3.2 单次冲量

希望一次冲量把 `vRel·n` 修正到目标 `−bias`（`bias` 见 §5/§6）：

```text
λ = −(vRel·n + bias) / K_n
```

累计冲量 `accN` 必须满足非负（接触只能推、不能拉），所以对**累计值**做投影：

```text
accN_new = max(accN + λ, 0)
Δλ      = accN_new − accN
把 Δλ·n 施加到 A（−）和 B（+）
```

代码：`ContactSolver::SolveConstraintPoint`。`accN` 与 `Δλ` 分开是必须的：只施加增量，
累计值才是"该接触点已经给出的总冲量"，warm starting 继承的就是这个总冲量（§3.5）。

### 3.3 迭代：Gauss-Seidel（PGS）

一个接触点的冲量会改变相邻接触点的相对速度（通过共享的刚体速度），所以不能一次联立解。
PGS 按固定顺序**逐个约束投影**：

```text
for iteration in 0..velocityIterations−1:        （默认 kVelocityIterations = 8）
    for each contact point (in World's contact order):
        解法向约束（§3.2）
        解切向约束（§4）
```

这是 Gauss-Seidel：每次用**刚更新过的**速度解下一个约束，所以结果依赖迭代顺序
（"顺序就是物理"——切换宽相/改岛序都会改变末位浮点，见 `docs/dev-interfaces.md` §6）。
收敛是线性的：每轮把残差乘上一个与约束图条件数相关的因子。

**PGS 与 LCP 的关系（2–3 句话）**：整个接触系统可写成线性互补问题（LCP）：
求冲量 `λ ≥ 0`，使每个接触点满足互补条件 `λ_i ≥ 0 ⊥ (vRel_i·n_i + bias_i) ≥ 0`，
同时速度满足 `v = v_free + K·λ`。PGS 是 LCP 的一种**迭代解法**：逐个约束做"投影到可行集"的
Gauss-Seidel 更新（正冲量夹到 ≥0、摩擦夹到锥内），天然处理不等式与钳制，不需要矩阵分解；
代价是收敛慢（迭代数 × 约束数），但每步极便宜，这就是本项目（以及 Box2D）选它的原因。

**实测收敛**（`docs/devlog.md` W3，`test_contact_solver.cpp::IterationsReduceResidualNormalVelocity`）：
两球叠在地面上的单次迭代残余 `g·dt/2 = 0.0833 m/s`，8 次迭代降到 `0.00065 m/s`。

### 3.4 迭代顺序的几个硬约定

- **法向先、切向后**：摩擦锥半径是 `μ·accN`，必须用**本轮刚更新**的 `accN`（§4.3）。
- **钳累计值，不钳增量**：见 §4.2。
- **`K_n` 必须 > 0**：`Prepare` 对 `!(K_n > kEpsilon)` 的约束做 debug assert 并跳过
  （static-static 或数据退化；release 下跳过，保证求解器是 total function，
  `test_contact_solver.cpp::StaticStaticContactAssertsOnNonPositiveMass` 与 release-only 测试钉死）。

### 3.5 warm starting：把上一帧的解当作初值

PGS 的收敛慢是"从零开始"的代价。物理场景帧间高度相干，上一帧的冲量**几乎就是**这一帧的解，
所以：

```text
Prepare:  accN = inherited·warmStartFactor,  accT = inherited·warmStartFactor
WarmStart: 把 (accN·n + accT·t) 一次性施加到两个刚体     （只施加一次！）
SolveVelocity: 从 warm start 后的速度继续 PGS 迭代
写回: ContactPoint::normalImpulse/tangentImpulse = accN/accT
```

继承按 `ContactID::Key()` 匹配（`World::MatchAndInheritImpulses`）：同一物理特征点才能继承，
否则冲量会加到错误的锚点上（堆叠抖动的第一根因）。`isPersistent == false` 的接触点从精确 0 开始。

**默认 `warmStartFactor = 1.0`（完整继承）**。W1 计划里的 0.85 是没跑过堆叠时的假设；
W5 实测（10 盒 × 1 m 塔、600 步 = 10 s、默认 8/3 迭代、μ=0.3、split impulse 开）：
factor 0 → step 258 塌、0.5 → 314、0.85 → 259、0.90 → 283、0.95 → 332，
**1.0 → 不塌**（max penetration 5.223 mm、top y 9.478、末秒抖动 0.0248 m/s）。
机制：factor < 1 每帧丢掉 `(1−f)` 的已收敛冲量，PGS 必须重解这部分，把约束**顺序偏差**
重新注入——warm start 本意正是压制它。该偏差已登记在 `docs/dev-interfaces.md` §5。

> **一个位级细节（W7）**：`WarmStartSeed` 写的是 `inherited * factor + 0.0f`。
> 后加的 `+ 0.0f` 把 `负继承冲量 × 0 = −0.0` 归一为 `+0.0`，否则 `useWarmStarting=false`
> 与 `warmStartFactor=0` 两种配置在休眠冻结冲量后会出现 −0/+0 的位级差异。
> 这是"看似多余的 +0 其实承载契约"的典型例子。

### 3.6 延伸思考

1. **"K_n 是怎么来的？为什么要 `Cross(r,n)²`？"**
   答：冲量在锚点产生线速度 `J/m` 和角速度 `r×J/I`；角速度再通过杠杆臂贡献回锚点速度，
   `r×J` 里含 `r×n`，与法线点乘后就是 `Cross(r,n)²`。当杠杆臂平行于法线时扭矩为 0，该项消失。
2. **"为什么不用全局 LCP 求解器？"**
   答：PGS 每步只需 O(迭代×约束) 次标量乘加、天然处理不等式与摩擦锥钳制、无矩阵存储；
   堆叠场景 8 迭代就够用。全局 LCP 收敛更好但实现/调试成本高，不是本项目目标。
3. **"warm start 的 factor 为什么不取 0.85 多'保险'？"**
   答：实测 10 盒塔所有 <1.0 的 factor 都在 10 s 内倒；factor<1 每帧丢掉已收敛分量、
   重新注入 PGS 顺序偏差。Box2D 2.4 也是完整继承。

---

## 4. 摩擦（库仑摩擦锥）

### 4.1 切向约束

2D 中切向只有一条方向：`t = Skew(n) = (−n.y, n.x)`（逆时针转 90°，单位向量）。切向相对速度
`vRel·t` 同样用有效质量修正，`K_t` 与 `K_n` 同形（把 `n` 换成 `t`）：

```text
K_t  = invMassA + invMassB + invIA·Cross(rA,t)² + invIB·Cross(rB,t)²
λ_t  = −(vRel·t) / K_t
```

代码：`ComputeEffectiveTangentMass`、`SolveConstraintPoint` 的第二段。

### 4.2 库仑锥与"钳累计值"

库仑摩擦要求切向冲量不超过法向冲量乘摩擦系数：

```text
|accT| ≤ μ · accN
```

**钳的是累计切向冲量**：

```text
maxF      = μ · accN_new                        （用更新后的法向累计值！）
accT_new  = Clamp(accT + λ_t, −maxF, +maxF)
Δλ_t      = accT_new − accT
```

**为什么不能钳增量？** PGS 是"把当前总解投影到可行域"的迭代：每个约束的解必须只依赖当前累计值，
这样无论从哪个初始值出发（warm start / 零），最终都收敛到同一个投影解。若改成
`Δλ_t' = Clamp(λ_t, ...)`，单步增量有界，但累计值可以一步步滑出摩擦锥（每步都合法、总和不合法），
迭代结果依赖迭代历史与初始值，堆叠会出现"缓慢侧滑 + 抖动"。
Box2D 的经典解释：clamp the accumulated impulse = project the *total* impulse onto the cone。

**为什么法向先解？** 锥半径 `maxF = μ·accN` 依赖法向累计值。如果先解切向后解法向，
切向冲量施加后法向更新可能让锥**收缩**，使已施加的 `accT` 落在锥外，直到下一次迭代才被纠正——
等价于允许了短暂的越界。

### 4.3 μ 的组合公式与取舍

`World::CreateContactForPair`（`src/Core/World.cpp`）建接触时组合材质：

```text
friction    = sqrt(μA · μB)      （几何平均）
restitution = max(eA, eB)        （更"弹"的表面主导）
```

求解器**直接使用** `contact.friction`，绝不二次组合（`ContactSolver::BuildConstraintsForContact`
的注释与 assert 都强调这点，`docs/dev-interfaces.md` §6 也是不变量）。

几何平均的取舍：只要一方 μ=0，组合结果就是 0（冰面行为"传染"，物理上正确）；
算术平均 `(μA+μB)/2` 会给 μ=0 的表面凭空 0.5 的摩擦。几何平均对 μ 的两极都更温和，
也是 Box2D 的选择。代价是"期望值"不直观（μ=0.2 与 0.8 组合 ≈ 0.4 而不是 0.5），
所以必须写进文档而不是让使用者猜。

### 4.4 实测

- **摩擦角扫描**（W5，`docs/devlog.md`）：20 格（μ ∈ {0.1,0.3,0.5,0.8,1.2} × θ ∈ {5,15,30,45}°）。
  `μ > tanθ` 的 12 格 5 s 位移 ≤ **1.5 mm**（静止）；`μ < tanθ` 的 8 格明显下滑
  （μ/tanθ = 0.8 → +17.74 m；μ=0.1、45° → +79.8 m）。
- **锥不变量**：1000 帧斜堆叠 `max|accT|/accN = 0.5000`、锥外 violations = **0**；μ=0 时
  `tangentImpulse` 恒 0。
- 单点手算：`v=(3,2), n=(0,1), μ=0.5` → `accN = 2, accT = −1`（正好落在锥边界）。

### 4.5 延伸思考

1. **"摩擦为什么钳累计值而不是单次增量？"**
   答：PGS 的解是可行域上的投影，累计值代表"该点已给的总冲量"；钳增量会让累计值漂出锥，
   解依赖迭代历史，warm starting 的继承也会把越界值带入下一帧。
2. **"μ 为什么用 sqrt(μA·μB)？"**
   答：一方无摩擦时组合必须为 0（几何平均满足、算术平均不满足）；两极行为温和、对称，
   且与 Box2D 一致。代价是不直观，需文档说明。
3. **"为什么先解法向？"**
   答：锥半径 μ·accN 依赖法向累计值；先切后法会让锥在施加切向冲量后收缩，短暂越界。

---

## 5. 恢复系数（restitution）

### 5.1 bias 的推导

完全非弹性（e=0）时目标相对法向速度是 0；弹性碰撞要求分离速度 = `e × 接近速度`。
在 `Prepare` 里（**warm start 之前**）捕获初始法向相对速度 `vn0 = vRel·n`（接近时为负）：

```text
bias = e · vn0                        if  vn0 < −restitutionThreshold
     = 0                              otherwise
```

`bias < 0`，求解器的目标 `vRel·n = −bias = e·|vn0| > 0` —— 正好是弹性碰撞的分离速度。
`e = contact.restitution = max(eA, eB)`。

### 5.2 为什么必须是"初始速度"+"速度阈值"

- **必须在 warm start 之前捕获**：warm start 会先改变 `vn`，若之后才计算 `bias`，静止接触每帧都会
  因为 warm start 的微小速度变化被重新赋予弹性目标 → 永远轻弹。
- **速度阈值 `kRestitutionVelocityThreshold = 1.0 m/s`**：静止堆叠中接触点每帧仍有
  `g·dt ≈ 0.167 m/s` 量级的微小接近速度（重力在一步内给出的速度）。若没有阈值，每个静止接触
  每帧都会被补一个 `e·0.167` 的分离速度，形成极限环（devlog W3 记录了 W2 位置推出导致的
  ~0.9 m/s 极限环；W5 的摩擦 + warm start + split impulse 之后彻底消失）。阈值让 "静止" 与
  "碰撞" 在速度层被明确区分。

### 5.3 实测

- e=0.8 从 5 m 落下（W5）：apex **3.4633 m**、impact **9.5000 m/s**、rebound **7.7333 m/s**、
  ratio **0.8140**。解析目标 `e²h = 3.2 m`；偏差来自离散时间下冲击速度偏高 `g·dt`（semi-implicit
  的已知一阶误差）——`rebound/impact ≈ e` 精确成立，所以测试断言的是**速度比**而不是绝对高度。
- 低速接触（`vn > −1 m/s`）`bias` 严格为 0，速度被吸收不反弹（W3 起的长跑契约测试）。
- 10 盒塔 10 s 站立：所有接触的接近速度都低于阈值，restitution 不参与，稳定性只靠摩擦 +
  warm starting + split impulse（W5 三级归因）。

### 5.4 延伸思考

1. **"restitution 的 bias 为什么用初始接近速度？"**
   答：弹性碰撞的物理量是碰撞前后的速度比；只有"碰撞时刻"的接近速度有意义。放到迭代里会让
   每个约束互相污染，放到 warm start 之后会被 warm start 的速度变化污染。
2. **"为什么需要速度阈值？"**
   答：静止堆叠的微小接近速度（g·dt 量级）会被 e 放大成每帧的分离速度，永远停不下来；
   1.0 m/s 把"静止"和"真实碰撞"分开（Box2D 同款）。
3. **"e 怎么组合？"**
   答：`max(eA, eB)`，更弹的表面主导；与摩擦的几何平均不同，这是选型决定而非推导结果。

---

## 6. 位置修正：Baumgarte 与 split impulse

### 6.1 为什么需要位置修正

速度层只能把 `vRel·n` 压到 0；PGS 有限迭代 + 离散积分（`p += dt·v`）会让接触每帧留下一点点
穿透。几百帧后高塔会缓慢下沉、互相嵌入。所以需要**位置层**。

### 6.2 Baumgarte：简单，但会注入能量

把穿透转成速度目标，直接塞进速度约束的 bias：

```text
bias = −(β/dt) · max(0, penetration − linearSlop)      （only when useSplitImpulse == false）
```

`−bias = (β/dt)·excess` 是分离速度目标：一步 dt 后位置修正 `β·excess`，几何级数收敛。
问题：这个分离速度是**真实速度**，它会给系统注入能量。两个可见后果：

- 静止接触每帧获得一个向上的真实速度（抖动/轻弹），
- 高速碰撞时物体会"弹过头"（能量不守恒）。

W5 devlog 记录的确定性对照（更深的初始穿透实验）：Baumgarte 的真实速度峰值 **1.139999 m/s**，
恰好等于手算 `(β/dt)·(pen − slop) = 12·(0.1 − 0.005)`（即该实验的初始穿透 0.1 m），
而 split impulse 版本是 **0.000000 m/s**，真实速度逐位不变。
当前测试 `SolverSplitImpulse.RestingContactDoesNotGainVelocityFromPositionCorrection` 用 2 cm 重叠：
目标分离速度 `12·0.015 = 0.18 m/s`；split impulse 全程真实速度 < 0.05 m/s、Baumgarte > 0.1 m/s。

### 6.3 split impulse：独立伪速度层

**核心思想**：位置修正也用一个"速度 → 位置"的求解，但速度存在单独的**伪速度**字段里，
只积分进 transform，永远不进入真实速度、不进入动能。

`src/Solver/ContactSolverPosition.cpp::SolvePosition`（phase 8，位置积分之后）：

**① 用积分后的 transform 重建约束。** 速度层的 `rA/rB`/`K` 缓存已过期一步（phase 8 在
`IntegratePositions` 之后），位置层必须用当前位置的杠杆臂：

```text
worldA = xfA · localPointA,  worldB = xfB · localPointB
separation_now = (worldB − worldA)·n            （生成时刻为 0）
penetration_now = manifold.penetration − separation_now
```

`manifold.penetration` 是接触**生成时刻**的穿透，`separation_now` 是这期间锚点沿法线分开的距离，
两者相减得到当前穿透。

**② 伪速度迭代。** 每个点求一个"分离速度目标"，用伪速度满足：

```text
target = (β/dt) · max(0, penetration_now − splitImpulseSlop)     ≥ 0
pseudoVn = pseudo 相对法向速度
λp = −(pseudoVn − target) / K_n                                   （K_n 与速度层同形）
accP_new = max(accP + λp, 0)                                      （只能推、不能拉）
把 Δλp·n 施加到 pseudoLinearVelocity/pseudoAngularVelocity
```

位置在迭代期间冻结，所以 `target` 恒定；迭代 3 次（`kPositionIterations`）让伪速度在约束之间传播。

**③ 一次积分 + 清零。**

```text
xf.p += dt · pseudoLinearVelocity            （长度钳到 maxLinearCorrection = 0.2 m）
xf.q  = Normalize(xf.q · Rot(Clamp(dt·pseudoAngularVelocity, ±kMaxAngularCorrection)))  （8°）
pseudoLinearVelocity = pseudoAngularVelocity = 0
```

**为什么单独一个锚点一步恰好修正 `β·excess`？** 单约束迭代收敛到 `pseudoVn = target`，
伪速度 `= target = (β/dt)·excess`，积分一步 `dt·target = β·excess`。
实测（`test_solver_split_impulse.cpp`）：初始穿透 0.05 m、slop 0.005 → excess 0.045，
一次迭代位移 `0.2 × 0.045 = 0.009 m`，**精确**；有杠杆臂时平移 −0.002 m + 转角 −0.002 rad
各贡献一半，锚点分离仍恰好 0.004 m。

**为什么不污染真实速度？** 伪速度是独立字段，位置层**不读不写** `linearVelocity/angularVelocity`；
`SolvePosition` 首尾都清零伪速度，防止"免费速度"泄漏到下一步。测试用 `std::bit_cast`
逐位比较前后真实速度（`PositionLayerLeavesRealVelocitiesUntouched`）。这不是"更小的穿透"
（full warm start 下 Baumgarte 的穿透也很小），而是**不注入能量**——这是 split impulse 的价值。

### 6.4 常数与偏差登记

| 常数 | 值 | 说明 |
| --- | --- | --- |
| `kSplitImpulseBeta` | 0.2 | 每步修正 20% 的 excess |
| `kSplitImpulseSlop` | `kLinearSlop` = 0.005 m | W1 占位值是 0.01（= 验收线，永远达不到），W5 改为通用接触容差并登记偏差 |
| `kMaxLinearCorrection` | 0.2 m | 单步伪位移上限，防深穿透"瞬移" |
| `kMaxAngularCorrection` | 8° | 单步伪旋转上限 |
| `kBaumgarte` | 0.2 | 仅 `useSplitImpulse=false` 的对照基线使用 |

实测：10 盒塔 600 步 max penetration **5.223 mm**（< 10 mm 验收线）、顶层 y 9.47813 m、
最大倾斜 0.004596 rad、末秒抖动 0.0248 m/s；`BoxStack` smoke 3.893 mm。

### 6.5 延伸思考

1. **"Baumgarte 的问题是什么？split impulse 怎么解决？"**
   答：Baumgarte 把穿透转成速度 bias，修正量进了真实速度 → 注入能量（静止接触抖动/高速弹过头）。
   split impulse 用独立的伪速度层，只积分进位置、从不回写真实速度，所以修正不改变动能。
2. **"为什么位置层要重算杠杆臂？"**
   答：phase 8 在 `IntegratePositions` 之后，速度层的缓存是积分前的位置；深穿透/大转动时
   用旧杠杆臂修正的位置会偏，且 `separation_now` 必须用生成时刻的锚点定义。
3. **"伪速度为什么每步必须清零？"**
   答：不清零会变成求解器看不见的"免费速度"，下一步位置直接漂移，等价于能量注入；
   而且每个 body 的伪速度应为本步位置层的瞬时状态。

---

## 7. SAT 与裁剪式 Manifold

### 7.1 分离轴定理（2D 的轴集合）

**定理**：两个凸多边形不相交，当且仅当存在一条轴 `a`，使两者的投影区间不相交。

```text
interval_A = [min_j (a·A_j), max_j (a·A_j)]
interval_B = [min_j (a·B_j), max_j (a·B_j)]
分离 ⇔ 区间不重叠
```

2D 中**候选轴只需要两多边形的边法线**（共 `nA + nB` 条，两个盒子是 4 条）：
3D SAT 需要的"边的叉积轴"在 2D 不存在。完备性可以这样理解：若分离，最近特征对必是
"顶点-边"或"边-顶点"（多边形是分段线性的，最近点落在线段上），该轴垂直于某条边。

`FindMaxSeparation(polygon, xf, other, xfOther)`（`src/Collision/SAT.cpp`）对 `polygon` 的每条边法线
`n_i` 求**最大分离**：

```text
separation_i = min_j [ n_i · (otherWorldV_j − edgeStart_i) ]
             = −（other 沿 n_i 深入该边平面的深度）
取 separation 最大的边 i（= 穿透最小的轴）
```

`separation > 0` → 存在分离轴 → 不相交，返回空 manifold。`CollidePolygons` 对 A、B **各算一次**；
任一方向为正即分离。若都 ≤ 0，则分离值较大的那条轴是"最小平移向量"方向。

### 7.2 reference face / incident face

```text
reference polygon = 拥有"最大分离轴"的多边形
reference face    = 该轴所属的边（世界坐标端点 V1, V2 与单位外法线 n_ref）
incident polygon  = 另一个多边形
incident face     = incident 上外法线与 n_ref 最反平行的边（Dot 最小）
```

### 7.3 裁剪（Sutherland–Hodgman 半平面裁剪）

`Clipping.cpp::ClipSegmentToLine(input[2], output[2], normal, offset)` 的半平面约定是
**保留 `Dot(normal, x) ≤ offset` 的点**；端点都在界内保留 2 点，一内一外追加交点，
都在界外返回 0。

对 incident 边做**两次侧向裁剪**（用 reference face 的两个端点平面）：

```text
start plane: normal = −t, offset = −Dot(t, V1)
end   plane: normal = +t, offset = +Dot(t, V2)        其中 t = Normalize(V2 − V1)
```

然后在参考面平面的**内侧**保留（等价于第三次裁剪，但折进逐点测试以避免"深度恰 0 的交点"
把顶点-面接触变成 2 点）：

```text
pointPenetration = Dot(n_ref, V1) − Dot(n_ref, point)
保留 pointPenetration ≥ 0 的点；manifold.penetration = max(pointPenetration)
```

ASCII 图（A 在上、B 在下，A 的下边是 reference face）：

```text
                  A（reference polygon，下边 = reference face）
        V1 ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● V2      n_ref 朝下（A→B）
           │  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  │
           │  ┃   incident face（B 的上边）   ┃  │        裁剪窗口 = 两侧平面
           │  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  │
                  B（incident polygon）
        两个交点都落在 reference face 内侧 → 2 点接触（face-face）
        只有一个角进入 → 1 点接触（vertex-face）
```

法线方向（A→B）：reference 属于 A 时 `n = n_ref`；属于 B 时 `n = −n_ref`（`CollidePolygons` 末尾）。
`Manifold::normal` 的契约在 `include/physics2d/Collision/Manifold.h`。

### 7.4 ContactID：让 warm starting 跨帧稳定

每个接触点带特征编码（`include/physics2d/Collision/ContactID.h`）：

```text
Key() = referenceEdge | (incidentVertex << 4) | (type << 8)     （uint16，10 bit 有效）
type: 0 vertex-face / 1 face-face / 2 circle-circle / 3 plane
flip  : 存了但**不参与** Key —— 朝向是元数据，不是身份
```

`flip` 不进 Key 是 warm starting 的前提：A/B 交换后同一物理接触的 Key 集合必须相同，
否则冲量会继承到错误锚点。**A/B 交换对称性**靠 `SAT.cpp::SelectReference` 的对称 tie-break
（分离值相等时比 edge index，再比世界锚点字典序），与参数顺序无关。

**SAT cache（D6 滞回）**：当两个候选轴的分离值在相对容差内接近时，沿用上一帧的 reference face。
它只影响 manifold 的轴选择（进而 Key 的帧间稳定性），不改变物理：
`SAT.cpp` 用 `tolerance = kRelativeEpsilon · max(1, |separation|)`（`kRelativeEpsilon = 1e-5`，
近接触时 = 1e-5 m）。实测（W4/W5）：双盒 300 步 Key churn **4（cache on）vs 264（off）**，
maxPen 0.00280 vs 0.00333 m——cache 只降特征抖动、不改物理。

### 7.5 派生路径

圆-圆、圆-平面、圆-多边形、多边形-平面**不走 SAT**（`Collision/Analytic.cpp`），
用解析几何直接出法线/穿透/锚点，避免裁剪退化。多边形-平面（盒底平放）返回**2 个最深顶点**
作为接触点——这是 `BoxStack` 塔能稳住的关键（2 点支撑 vs 1 点支撑）。
圆-多边形按最近边/顶点 Voronoi 区域分类（P06）。

### 7.6 延伸思考

1. **"2D SAT 为什么不需要叉积轴？"**
   答：3D 中两个面片的分离方向可以是两边方向的叉积；2D 中"面片"退化成线段，分离轴必垂直于
   某条边。所以候选轴 = 两多边形的边法线集合，穷举即完备。
2. **"为什么取穿透最小的轴作为法线？"**
   答：最小穿透方向是把两物体推开所需的最短平移（MTV）方向；沿此方向裁剪得到的接触
   最符合物理直觉（面-面接触时法线就是面法线）。
3. **"ContactID 的 flip 为什么不参与 Key？"**
   答：Key 是物理接触的身份；A/B 参数顺序不该改变身份（P07 D3/D5 要求交换后 Key 集合相同）。
   flip 只记录"reference 面属于哪个多边形"，是生成元数据。

---

## 8. SAP 宽相（Sweep and Prune）

### 8.1 算法

`IBroadPhase`（`include/physics2d/Core/BroadPhaseInterface.h`）只知道 AABB、不知道 Body；
`BroadPhaseSAP`（`src/Collision/BroadPhaseSAP.cpp`）是主线实现，`BroadPhaseBruteForce` 是对照 oracle。

**一维端点表示**：每个 proxy 在 x 轴上有两个端点 `(min.x, id, IS_MIN)`、`(max.x, id, IS_MAX)`，
存在**单个** `vector<Endpoint>`（12 字节/端点）。单数组是为了增量交换：一次 swap 只动 12 字节，
`value/id/isMin` 不会失配。

```text
端点数组（按 value 排序；value 相等时 MIN 在 MAX 前——含端点重叠约定）：
   x:  |----A----|        |-----B-----|      |---C---|
     minA     maxA      minB       maxB    minC   maxC

扫描（按端点顺序）：
   minA → active = {A}
   minB → 对 active 中每个 proxy 做完整 AABB::Overlaps（y 轴二次过滤）；A/B 重叠 → 输出 (A,B)
          active = {A,B}
   maxA → active = {B}
   ...
```

复杂度：首帧 `std::sort` O(N log N)；扫描 O(N + K)（K = 候选对数）。

### 8.2 帧间相干性：稳态为什么近似 O(N)

物理场景中 AABB 每帧只移动一点点，端点数组**几乎有序**。`UpdateAABB` 只对移动的两个端点做
**冒泡**（插入排序思路：变小就向左、变大就向右），平均 O(1)~O(log N) 次交换：

```text
UpdateAABB(id, newAABB):
    改两个端点的 value
    min 端点 bubble-left（while 前一个更大就 swap）
    max 端点 bubble-right
    同步更新该 proxy 缓存的端点索引（minIndex/maxIndex）
```

反例（测试）：静止场景 1000 帧端点**零移动**；`ValidateInternalState()` 每帧验证
"数组有序 + 缓存索引一致 + MIN/MAX 数量 == 存活 proxy 数"。

**批量插入**：`AddProxy` 追加到尾部并置 dirty，下一个观察点做一次 `std::sort`；
避免场景加载时逐个插入的 O(N² log N)。为让 `const ValidateInternalState()` 任意时刻可验证，
排序走 `mutable` 懒执行——这是契约需要，不是设计缺陷。

### 8.3 候选对的顺序就是物理

宽相只决定"谁进窄相"，但候选对顺序 → 接触列表顺序 → **PGS 求解顺序** → 浮点末位。
所以 `World::UpdateBroadPhase` 把候选对**规范化**为 `(i < j, sorted, deduped)` 的 body-index 对，
与 W2-W5 的 O(N²) 枚举顺序逐位一致。验收：无宽相 / BruteForce / SAP 三路 300 步
**逐帧 545 floats 逐位一致**（含 body 状态与接触累计冲量）；运行中切换宽相 150 步与从未安装的
参照 World 逐位一致。规范序不是优化，是"优化不改变物理"的证据。

### 8.4 垂直堆叠场景的退化 & 假阳性

SAP 只按 x 轴排序。一整列垂直堆叠的盒子 x 区间**几乎完全重叠** → x 扫描时对每一对都产生候选，
只能靠 `AABB::Overlaps` 的 y 轴过滤剔除。候选对与真实重叠对的差 = fat margin 造成的假阳性：

| N（随机盒场景，Release / seed 42 / 1000 帧） | BruteForce ms/帧 | SAP ms/帧 | 加速比 | 候选对/帧 | 真实重叠/帧 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 50 | 0.0033 | 0.0015 | **2.20x** | 3.4 | 1.8 |
| 200 | 0.0838 | 0.0169 | **4.96x** | 55.3 | 31.3 |
| 500 | 0.5399 | 0.0916 | **5.89x** | 354.3 | 202.5 |
| 2000 | 9.0985 | 1.1656 | **7.81x** | 5670.6 | 3206.6 |

（"真实重叠"口径 = tight AABB 精确重叠，与候选对的差就是 fat margin 的假阳性成本；
见 `docs/profiling.md` 与 bench 输出。）

小 N 反转：SAP 的排序/维护常数在 N 小时不如 O(N²) 直接枚举，实测交叉点 **N ≈ 25–32**
（N=20 时 SAP 只有 0.80x），所以 testbed 的 Auto 宽相阈值取 **32**（实测来源，不是抄经验值）。

### 8.5 延伸思考

1. **"SAP 为什么比 BruteForce 快？复杂度差多少？"**
   答：BruteForce 每帧 O(N²)；SAP 首帧 O(N log N) 排序、稳态利用帧间相干性近似 O(N+K)，
   K 是候选对数。实测 2000 物体 7.81x、小 N 会反转（交叉点 N≈25-32）。
2. **"端点为什么用单数组而不是 SoA？"**
   答：增量更新本质是一串 swap；单数组 12 字节一次交换，value/id/flag 不会失配，
   排序比较器也 cache 友好。SoA 每次 swap 要动三个数组。
3. **"垂直堆叠下 SAP 有什么问题？"**
   答：x 轴区间完全重叠，扫描退化为靠 y 过滤的 O(N²) 候选检查，假阳性由 fat margin 决定；
   所以宽相结果必须规范化顺序交给窄相，物理正确性不依赖宽相质量。

---

## 9. 岛屿与休眠

### 9.1 接触图与连通分量

把每个 **dynamic** body 看作节点；两个 dynamic body 之间的 contact 是边。连通分量就是**岛屿**：

```text
static / kinematic body = 边界（它们的 contact 仍随岛求解，但不属于任何岛、不传递连通性）
原因：若共享地面也算边，同一地面上所有独立堆叠会合并成一个岛，岛屿/休眠全部失效。
无 contact 的 dynamic body = 单例岛（休眠计时需要）。
```

`BuildIslandPartition`（`src/Solver/Island.cpp`）用**显式 vector 栈的迭代 DFS**
（2000 body 长链不能爆调用栈），forward-star 邻接（head 数组 + 2×contact 个 int），
两遍升序扫描产出**规范序**：

```text
岛按最小成员 body slot 升序；岛内 bodies 升序；岛内 contacts 按全局 contact 序
```

复杂度 O(N + C)。实测：3 个互不接触的堆叠 → 3 岛；同一静态地面上的独立堆叠**不合并**；
500-body 长链 → 1 岛 / 499 contact，不溢出。

### 9.2 为什么按岛求解与平铺求解逐位一致

PGS 是 Gauss-Seidel：不同岛之间的约束**不共享任何 body**，所以跨岛的更新可以任意交换，
不影响任何 float 结果；只要**岛内**约束相对顺序与全局顺序一致，分组求解就是同一个浮点计算序列。
实测：两岛场景 276 次、密集单岛 21,760 次 float 比较 **0 差异**；
World 级 grouped vs flat 300 步 × 469 floats/步逐位一致；回归弱 hash 保持不变。
这也是 `SolverSettings::useIslandSolving` / `packIslandBodies` 只是实验开关（默认 grouped-direct）的原因。

### 9.3 整岛原子休眠

```text
每步 UpdateSleep：
  对每个岛：
    island 内所有 dynamic body 满足 |v| < kSleepLinearTolerance(0.01 m/s)
                            且 |ω| < kSleepAngularTolerance(2°/s)
    → 每个 body 的 sleepTime += dt
  岛内所有 body 的 sleepTime > kSleepTimeThreshold(0.5 s)
    → 整岛 isAwake = false；真实速度/伪速度清零；sleepTime = 0
```

状态机：

```text
         [awake] ──(整岛连续 0.5 s 都低于速度阈值)──▶ [asleep]
            ▲                                          │
            │                                          │ 休眠期间：
            └──── 外力/冲量/SetTransform/SetVelocity ───┘
                  / 宽相邻近（fat AABB 与某个 awake 物体重叠）
```

**为什么整岛原子**：如果一个 body 睡了、邻居还醒着，邻居会移动而它冻结 → 岛边界立刻穿模。
所以休眠/唤醒的最小单位是岛。**为什么唤醒必须整岛**：同理。

**休眠岛零工作**：跳过 AABB 更新（proxy 保留）、窄相（复用上一帧 contact，保留岛图与 warm-start
数据）、warm start、速度/位置求解；其累计冲量冻结不动。邻近唤醒放在 `UpdateContacts` 开头的
**共享 wake pass**（宽相与 O(N²) fallback 两条路径同序），而不是 `UpdateBroadPhase`（那里无宽相会
提前 return）。这是 W7 的偏差登记条目。

### 9.4 实测

- **10 盒塔**：sleep step **859**（不是 600）；之后 100 步 body 状态与接触累计冲量逐位不变；
  debug Step 从 **87.5 µs/step（awake）到 26.5 µs/step（asleep），3.30x**。
  859 步的诚实原因是 2 点接触逐点 PGS 的顺序偏差造成残余抖动尖峰
  （末秒 maxV 0.0248 m/s > 0.01 阈值，W5 已记录）——正解是 2×2 block solver（W8 技术债），
  **不是**调大休眠阈值。
- **IslandSleep500**（500 盒 / 100 岛，Release 600 步）：frame 105（1.75 s）全睡；
  唤醒期平均 Step **0.4554 ms → 全休眠 0.2627 ms，1.73x**（debug 4.95 → 2.94 ms，1.69x）。
  加速没有数量级是因为仍保留宽相 sweep 与休眠接触复用。
- **多米诺**（Release 600 步）：休眠后推倒，`woken_after_sleep = 20/20`、`fallen = 20/20`、
  `first_skipped = -1`（无骨牌被跳过）。
- **唤醒入口**：`ApplyLinearImpulse` / `SetLinearVelocity` / `SetTransform` / `ApplyForce` 均立即整岛唤醒；
  邻近唤醒（探针进入休眠塔的 fat AABB 但无几何接触）同样整岛唤醒，宽相 on/off 两条路径都验证。

### 9.5 延伸思考

1. **"岛屿怎么划分？为什么 static body 不传递连通性？"**
   答：dynamic body 是节点、dynamic-dynamic contact 是边，连通分量即岛；static/kinematic 是边界。
   若地面传递连通性，同一地面上所有堆叠会合并成一个岛，休眠/分岛求解全部失效
   （设计文档的"两盒叠地上同岛"由 box-box 接触满足）。
2. **"为什么按岛求解能保持结果不变？"**
   答：岛间不共享 body，Gauss-Seidel 更新可交换；岛内保持全局相对顺序即可。
   所以分组模式与平铺模式逐位一致（21,760 floats 0 差异 + 回归 hash 不变）。
3. **"休眠为什么必须整岛？"**
   答：单 body 休眠会在岛边界留下"一半睡一半醒"的半冻状态，醒着的邻居移动就穿模；
   唤醒同理必须整岛，且邻近唤醒要在宽相/无宽相两条路径同序执行。

---

## 10. 边界契约与已知限制（输入 → 行为 → 测试位置）

> **Death test 的执行方式（重要）**：所有 assert 只在 Debug 生效（Release 编译掉，`NDEBUG`）；
> Release 契约是**返回值/安全跳过**（核心库不抛异常）。本沙箱禁止工作区内的测试 exe 写临时文件，
> 所以 ctest 里 31 个 death test 显示 `Skipped`——这是正常的；CI 与把 exe 复制到工作区外运行会真正执行。
> 命令见 `docs/testing.md` §8。

| 输入 / 场景 | 契约行为 | 测试位置 |
| --- | --- | --- |
| 退化多边形：面积≈0、凹、顺时针、重复/过近顶点、共线顶点、非有限坐标、顶点数 ∉ [3,8] | `SetPolygon` 返回 `false`，形状置 invalid，**不崩**；`World::CreatePolygonShape/CreateBoxShape` 返回 invalid handle + Debug assert | `test_polygon_shape.cpp`、`test_world_sat.cpp`（`PolygonValidationDeathTest`）、`test_boundary_solver.cpp`（`BoundaryPolygon.*`、`BoundaryDensityDeathTest.WorldCreateShapeRejectsOverflowingGeometry`） |
| 有限但**溢出**的坐标（鞋带和/边长平方超 float，如半宽 1e19） | Week 8 起同样拒绝：`SetPolygon` 返回 false（否则 `Normalized()` 返回 (0,0)，SAT 轴失效）；World API 同上 assert | `test_boundary_solver.cpp::BoundaryPolygon.FiniteButOverflowingCoordinatesAreRejected` |
| 密度 = 0（dynamic body） | `CreateBody` **接受**（形状未附着时用 `BodyDef.mass/inertia` fallback）；`AttachShape` 需要 density > 0 → Debug assert + Release 返回 false | `test_boundary_solver.cpp::BoundaryDensity.ZeroDensityDynamicBodyUsesTheBodyDefMassFallback` + `BoundaryDensityDeathTest.ZeroDensityDynamicAttachAsserts` |
| 负密度 / 非有限密度 | `CreateBody` 返回 invalid handle（返回失败，**不 assert**，所有 build 一致） | `test_boundary_solver.cpp::BoundaryDensity.NonFiniteAndNegativeDensityAreRejectedAtCreateBody` |
| 密度大到质量溢出（如 3.4e38） | `ComputePolygonMass` Debug assert（"non-finite mass"）；Release 返回非物理数据前已被 `AttachShape` 的非有限检查拒绝 | `test_boundary_solver.cpp::BoundaryDensityDeathTest.HugeDensityOverflowingMassAsserts` |
| static/kinematic body + 任意密度 + 形状 | 合法：不推导质量，质量/惯量保持 0（无穷大质量的表示） | `test_world_shapes.cpp`、`test_boundary_solver.cpp::BoundaryDensity.StaticAndKinematicBodiesIgnoreDensityWhenAttaching` |
| 极大速度（> `kMaxTranslation/dt` = 120 m/s） | 单步位移钳制 2 m、方向保留；速度不变；无 NaN/Inf；SAP 内部状态有效 | `test_world_step.cpp`、`test_boundary_solver.cpp::BoundaryVelocity.*` |
| 极大速度撞薄障碍 | **已知限制：无 CCD，可能隧穿**。契约只是"状态有限、不崩" | `test_boundary_solver.cpp::BoundaryVelocity.HugeDownwardVelocityAgainstGroundStaysFinite`；README 已知限制 |
| 超长薄条 1×100 m | 支持（AABB 100 m、SAP 代理、接触杠杆臂均正常，无 NaN）；物理上长杠杆+2 点 PGS 会有残余转动，不承诺"完全平躺" | `test_boundary_solver.cpp::BoundaryThinBar.OneHundredMetreBarOnGroundStaysFinite` |
| 极端质量比 1:1e6 | `K_n = 1 + 1e-6` 有限正、不倒除零；单步冲量动量守恒到 float 精度；World 级堆叠无 NaN | `test_boundary_solver.cpp::BoundaryMassRatio.*` |
| static-static / 双方无逆质量的接触 | `Prepare` Debug assert（"non-positive effective normal mass"）并跳过该点；Release 跳过不崩 | `test_contact_solver.cpp`（`SolverDeathTest` + release-only 测试） |
| 手搭 contact：越界 body index / pointCount 越界 | Week 8 起 Debug assert（速度层 `Prepare` 与位置层 `SolvePosition` 都校验）；此前位置层是 UB | `test_boundary_solver.cpp::BoundarySolverDeathTest.*` |
| contact 的非有限 normal/penetration | Week 8 起 Debug assert（速度层与位置层） | `test_boundary_solver.cpp::BoundarySolverDeathTest.PrepareRejectsNonFiniteManifoldNormal` |
| `World::Step(dt ≤ 0 / NaN)` | Debug assert；Release 提前 return（不污染状态） | `test_boundary_solver.cpp::BoundaryWorldDeathTest.StepRejectsInvalidDt`、`test_world_step.cpp` |
| `World::Step` 负迭代数 | Debug assert；Release 视为 0 次迭代 | `test_boundary_solver.cpp::BoundaryWorldDeathTest.StepRejectsNegativeIterationCounts` |
| `World` 构造非有限重力、`SetLinearVelocity`/`SetTransform` 非有限值 | Debug assert；**Release 无二次校验（调用方契约：不得传非有限值）**——非法值会被写入，并在下一步 `Step` 末尾的 `Validate`/`CheckSane` 中暴露 | `test_boundary_solver.cpp::BoundaryWorldDeathTest.*`、`test_world_step.cpp` |
| `SetFriction`/`SetRestitution` 非有限或负值 | Debug assert；**Release 防御性提前返回（拒绝写入）** | `test_boundary_solver.cpp::BoundaryWorldDeathTest.*` |
| 平面形状非法 normal/offset、平面挂到非 static body | Debug assert + invalid handle / 返回 false | `test_world_shapes.cpp`（`ShapeValidationDeathTest`） |
| 销毁仍被 body 附着的 shape | Debug assert 并拒绝（先销毁 body 或替换 shape） | `test_world_shapes.cpp`（`ShapeValidationDeathTest.DestroyAttachedShapeAsserts`） |
| 场景尺度超出 0.1–10 m（AGENTS.md §1.3） | 不支持/不保证：精度与溢出行为未定义；Week 8 的溢出保护只保证"拒绝而不崩" | 本文 §2.5、§10；`test_boundary_solver.cpp` |

**其它已知限制（README 同步引用）**：

1. **无 CCD/TOI**：依靠 2 m/步位移钳制 + fat AABB，可能隧穿。
2. **8 速度迭代的传播预算**：30:1 单列、金字塔底部（16-20 层）在默认迭代下会缓慢下沉/穿透
   （载荷属性，不是 bug）；10 盒塔是验收里程碑（5.223 mm）。
3. **薄片/2 点接触残余抖动**：逐点 PGS 对 2 点接触的顺序偏差 → 更薄形状入睡慢
   （10 盒塔 859 步）；正解 2×2 block solver（W8 技术债，不在 v1.0 范围）。
4. **无关节系统**、无 3D、无软体/流体/GPU（范围红线，`AGENTS.md` §1.6）。
5. **`solvePosition` 每帧分配局部 vector**：未证明是热点，先测量（技术债清单）。

---

## 11. 数字复现命令（测什么、跑哪条）

```powershell
# 全量单元测试（debug；沙箱里 31 个 death test 自动 SKIP 属正常）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset debug -RunTests

# 单独跑边界契约测试（本文 §10 的大部分）
.\build\debug\tests\physics2d_tests.exe --gtest_filter="Boundary*"

# death test 真正执行（工作区外）
$dir = Join-Path $env:LOCALAPPDATA 'Physics2D\testrun'; New-Item -ItemType Directory -Force $dir | Out-Null
Copy-Item .\build\debug\tests\physics2d_tests.exe $dir -Force; Push-Location $dir; .\physics2d_tests.exe; Pop-Location

# 确定性回归（弱 hash 必须不变）
.\build\debug\tests\physics2d_regression.exe --check tests\regression\golden\default_600.txt

# 宽相 bench（Release；§8.4 表格）
.\build\release\bench\physics2d_bench.exe --broadphase all --bodies 50,200,500,2000 --frames 1000 --seed 42
```

> 文档中的性能数字统一来自 `docs/profiling.md`（测量环境与原始命令都在那里）；
> 调度/物理行为数字来自 `docs/devlog.md` 的每周实测记录。改代码后若数字变化，先改 devlog/profiling
> 再改本文——**本文不自行产生数字**。

---

## 附录 A：关键常数一览（`include/physics2d/Core/Settings.h`）

| 常数 | 值 | 用途 |
| --- | --- | --- |
| `kLinearSlop` | 0.005 m | 接触容差/穿透死区（W5 起也是 split impulse slop） |
| `kAngularSlop` | 2° | 角度容差 |
| `kMaxLinearCorrection` | 0.2 m | 位置层单步位移上限 |
| `kMaxAngularCorrection` | 8° | 位置层单步转角上限 |
| `kBaumgarte` | 0.2 | Baumgarte 对照基线系数 |
| `kSplitImpulseBeta` | 0.2 | split impulse 每步修正比例 |
| `kSplitImpulseSlop` | = `kLinearSlop` | 位置层死区（偏差登记：W1 占位 0.01） |
| `kWarmStartFactor` | 1.0 | warm start 继承比例（偏差登记：W1 计划 0.85） |
| `kRestitutionVelocityThreshold` | 1.0 m/s | restitution 速度阈值 |
| `kMaxTranslation` / `kMaxRotation` | 2.0 m / π/2 | 单步位移/转角钳制 |
| `kAabbMargin` | 0.1 m | fat AABB 外扩（只加候选，不加接触） |
| `kVelocityIterations` / `kPositionIterations` | 8 / 3 | 默认迭代数 |
| `kSleepLinearTolerance` / `kSleepAngularTolerance` | 0.01 m/s / 2° | 休眠速度阈值 |
| `kSleepTimeThreshold` | 0.5 s | 入睡连续安静时间 |
| `kFixedTimeStep` / `kMaxStepsPerFrame` | 1/60 s / 5 | 固定步长 / 每帧步数上限 |
| `kDefaultGravityY` | −10 m/s² | 默认重力 |
| `kPlaneAabbExtent` | 1e6 m | 平面的超大 AABB 盒 |
| `kMaxManifoldPoints` | 2 | 2D 多边形接触点上限 |
| `kMaxPolygonVertices` | 8 | 凸多边形顶点上限（`ShapeTypes.h`） |
| `kEpsilon` / `kRelativeEpsilon` | 1e-6 / 1e-5 | 除零保护 / SAT cache 相对容差（`MathUtils.h`） |
