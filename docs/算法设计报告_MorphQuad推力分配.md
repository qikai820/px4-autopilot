# MorphQuad 推力分配算法设计报告

> 基于论文：*MorphQuad: Morphable Quadrotor for Superhuman Maneuverability, Manipulation, and Resiliency* (arXiv:2607.02764v3)

---

## 1. 引言

### 1.1 研究背景

MorphQuad 是一种可变形四旋翼飞行器，其核心特征是每个旋翼系统通过**双轴万向节**独立铰接，能够在全向空间中任意指向推力方向。与传统四旋翼仅能产生沿机体 -z 方向的推力不同，MorphQuad 的 12 个独立控制输入（4 个电机转速 + 8 个舵机角度）使其成为**过驱动系统**，能够在六自由度空间中独立生成任意力和力矩。

### 1.2 问题定义

给定几何控制器输出的六维期望 wrench：

$$\mathbf{w}_d = \begin{bmatrix} \mathbf{f}_d \\ \boldsymbol{\tau}_d \end{bmatrix} \in \mathbb{R}^6$$

其中 $\mathbf{f}_d \in \mathbb{R}^3$ 为机体坐标系下的期望合力，$\boldsymbol{\tau}_d \in \mathbb{R}^3$ 为期望合力矩。

**推力分配问题**：将 $\mathbf{w}_d$ 分解为 4 个旋翼的推力向量 $\mathbf{t}_i \in \mathbb{R}^3$（$i = 1,2,3,4$），并从中反解出每个旋翼的：
- 电机角速度 $\Omega_i$
- 外舵机角度 $\alpha_i$（绕机体 y 轴旋转）
- 内舵机角度 $\beta_i$（绕旋转后的 x 轴旋转）

### 1.3 核心挑战

| 挑战 | 描述 |
|------|------|
| **过驱动** | 12 维输入 → 6 维输出，解不唯一，需最优性准则 |
| **万向节锁** | 当推力方向平行于机体 y 轴时，内舵机 $\beta_i = \pm\pi/2$，外舵机 $\alpha_i$ 失去对推力方向的控制权 |
| **下洗干扰** | 当期望力沿机体 x 轴且力矩为零时，四个旋翼推力共线，相邻旋翼的下洗气流相互干扰，导致推力模型失效 |

---

## 2. 算法总体架构

```
─────────────────────────────────────────────────────────────┐
│                    推力分配算法流程                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  输入: w_d = [f_d, τ_d] ∈ R^6  (来自几何控制器)              │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────┐           │
│  │  Step 1: 最小范数分解 (Eq. 12-13)             │           │
│  │  w_d → t_min = [t_1, t_2, t_3, t_4] ∈ R^12  │           │
│  │  (Moore-Penrose 伪逆解)                       │           │
│  └──────────────────────┬───────────────────────┘           │
│                         │                                   │
│                         ▼                                   │
│  ┌──────────────────────────────────────────────┐           │
│  │  Step 2: 零空间修正 (Eq. 15-16)               │           │
│  │  t = t_min + N_M · c                         │           │
│  │  • 万向节锁避免: 排斥 β_i → ±π/2             │           │
│  │  • 下洗干扰避免: 打破推力共线性                │           │
│  └──────────────────────┬───────────────────────┘           │
│                         │                                   │
│                         ▼                                   │
│  ┌──────────────────────────────────────────────┐           │
│  │  Step 3: 舵机角度反解 (Eq. 14)                │           │
│  │  t_i → (α_i, β_i, Ω_i)                      │           │
│  │  • 双分支解 + 最小运动选择                     │           │
│  │  • 多周期缠绕支持 (连续旋转)                   │           │
│  └──────────────────────┬───────────────────────┘           │
│                         │                                   │
│                         ▼                                   │
│  输出: [Ω_1..Ω_4, α_1..α_4] → 执行器指令                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 数学推导

### 3.1 坐标系与旋翼几何

**机体坐标系 (NED)**：x 向前，y 向右，z 向下。

**X 型布局旋翼位置**（从质心 COM 出发）：

| 旋翼 | 位置 $\mathbf{l}_i$ | 旋转方向 |
|------|---------------------|---------|
| 1 (前右) | $(+l, -b, 0)$ | CW |
| 2 (前左) | $(+l, +b, 0)$ | CCW |
| 3 (后左) | $(-l, +b, 0)$ | CW |
| 4 (后右) | $(-l, -b, 0)$ | CCW |

其中 $l$ 为半臂长（x 方向），$b$ 为半臂宽（y 方向），对角距离 $r = \sqrt{l^2 + b^2}$。

**双轴万向节参数化**：每个旋翼的推力方向由两个舵机角度决定：
- $\alpha_i$：外舵机，绕机体 $j^B$（y 轴）旋转
- $\beta_i$：内舵机，绕旋转后的 $i^B$（x 轴）旋转

推力方向向量（论文 Eq. 3，适配 NED 坐标系）：

$$\mathbf{G}_i(\alpha_i, \beta_i) = \begin{bmatrix} \sin\alpha_i \cos\beta_i \\ -\sin\beta_i \\ \cos\alpha_i \cos\beta_i \end{bmatrix}$$

### 3.2 Wrench 方程

每个旋翼的推力 $\mathbf{t}_i = c_t \mathbf{G}_i(\alpha_i, \beta_i) \Omega_i^2$，其中 $c_t$ 为推力系数。

总 wrench（论文 Eq. 8-9）：

$$\mathbf{w} = \begin{bmatrix} \mathbf{f} \\ \boldsymbol{\tau} \end{bmatrix} = c_t \mathbf{F}(\alpha, \beta) \boldsymbol{\Omega}^{\circ 2} = \mathbf{M} \mathbf{t}$$

其中：
- $\mathbf{F} \in \mathbb{R}^{6 \times 4}$ 为每旋翼 wrench 方向矩阵
- $\mathbf{M} \in \mathbb{R}^{6 \times 12}$ 为常数输入矩阵（由固定旋翼位置 $\mathbf{l}_i$ 决定）
- $\mathbf{t} = [\mathbf{t}_1^\top, \mathbf{t}_2^\top, \mathbf{t}_3^\top, \mathbf{t}_4^\top]^\top \in \mathbb{R}^{12}$ 为堆叠的推力向量

**关键性质**：$\mathbf{M}$ 为宽矩阵（6×12），行满秩（rank = 6），因此系统过驱动，每个扰动都可被匹配抵消。

### 3.3 最小范数分解（论文 Eq. 12-13）

将每个旋翼的推力向量分解为四个分量：

$$\mathbf{t}_i = \frac{\mathbf{f}}{4} + \frac{\tau_x}{4b} k_{B,i}^{\text{roll}} - \frac{\tau_y}{4l} k_{B,i}^{\text{pitch}} + \frac{\tau_z}{4r} \hat{\mathbf{e}}_{\psi,i}$$

**各项物理含义**：

| 分量 | 表达式 | 物理意义 |
|------|--------|---------|
| 力共享 | $\mathbf{f}/4$ | 期望合力均分到 4 个旋翼 |
| 滚转力矩 | $\frac{\tau_x}{4b} k_{B,i}^{\text{roll}}$ | 通过前后旋翼推力差产生滚转力矩 |
| 俯仰力矩 | $-\frac{\tau_y}{4l} k_{B,i}^{\text{pitch}}$ | 通过左右旋翼推力差产生俯仰力矩 |
| 偏航力矩 | $\frac{\tau_z}{4r} \hat{\mathbf{e}}_{\psi,i}$ | 通过推力在水平面的切向分量产生偏航力矩 |

**符号模式**（X 型布局）：

$$k_{B}^{\text{roll}} = \begin{bmatrix} +1 \\ -1 \\ -1 \\ +1 \end{bmatrix}, \quad k_{B}^{\text{pitch}} = \begin{bmatrix} -1 \\ -1 \\ +1 \\ +1 \end{bmatrix}$$

**偏航力矩单位向量**（论文 Eq. 13，位于 $i^B$-$j^B$ 平面，垂直于旋翼臂）：

$$\hat{\mathbf{e}}_{\psi,1} = \begin{bmatrix} -b/r \\ l/r \\ 0 \end{bmatrix}, \quad \hat{\mathbf{e}}_{\psi,2} = \begin{bmatrix} b/r \\ l/r \\ 0 \end{bmatrix}, \quad \hat{\mathbf{e}}_{\psi,3} = \begin{bmatrix} b/r \\ -l/r \\ 0 \end{bmatrix}, \quad \hat{\mathbf{e}}_{\psi,4} = \begin{bmatrix} -b/r \\ -l/r \\ 0 \end{bmatrix}$$

**最优性**：该分解等价于 Moore-Penrose 伪逆解 $\mathbf{t} = \mathbf{M}^\dagger \mathbf{w}$，其中 $\mathbf{M}^\dagger = \mathbf{M}^\top(\mathbf{M}\mathbf{M}^\top)^{-1}$，具有最小范数（最优推力效率）。

### 3.4 零空间修正（论文 Eq. 15-16）

由于 $\mathbf{M} \in \mathbb{R}^{6 \times 12}$ 行满秩，线性系统 $\mathbf{w} = \mathbf{M}\mathbf{t}$ 欠定，通解为：

$$\boxed{\mathbf{t} = \mathbf{M}^\dagger \mathbf{w} + \mathbf{N}_M \mathbf{c}}$$

其中 $\mathbf{N}_M$ 为 $\mathbf{M}$ 的零空间基（常秩 6），$\mathbf{c} \in \mathbb{R}^6$ 为系数向量。

**推论 1（Wrench 保持性）**：对任意 $\mathbf{c} \in \mathbb{R}^6$，零空间修正不改变净 wrench：

$$\mathbf{M}\mathbf{t} = \mathbf{M}\mathbf{M}^\dagger \mathbf{w} + \mathbf{M}\mathbf{N}_M \mathbf{c} = \mathbf{w} + \mathbf{0} = \mathbf{w}$$

**证明**：$\mathbf{M}\mathbf{M}^\dagger = \mathbf{I}_6$（行满秩），$\mathbf{M}\mathbf{N}_M = \mathbf{0}$（零空间定义）。∎

这意味着零空间修正**完全不影响**几何控制器的闭环误差动力学和几乎处处稳定性保证。

**系数选择策略**（论文 Eq. 16）：

**(a) 万向节锁避免**：当 $\mathbf{f} \parallel j^B$ 时，$\beta_i \to \pm\pi/2$，$\cos\beta_i \to 0$，外舵机失去控制权。

$$c_1 = c_3 = c_5 = \langle \mathbf{f}/\|\mathbf{f}\|, j^B \rangle$$

该修正引入一个与 $\mathbf{f}$ 接近 $j^B$ 轴程度成正比的排斥力，将内舵机推离奇异点。

**(b) 下洗干扰避免**：当 $\mathbf{f} \parallel i^B$ 且 $\boldsymbol{\tau} = 0$ 时，所有推力共线，相邻旋翼下洗相互干扰。

$$c_2 = -c_4 = -\langle \mathbf{f}/\|\mathbf{f}\|, i^B \rangle, \quad c_6 = 0 \quad (\text{当 } \langle \hat{\mathbf{f}}, i^B \rangle > 0)$$

该修正打破共线性，使上下旋翼对不再直接对冲。

### 3.5 舵机角度反解（论文 Eq. 14）

给定推力向量 $\mathbf{t}_i$，定义单位推力方向 $\hat{\mathbf{t}}_i = \mathbf{t}_i / \|\mathbf{t}_i\| = [\hat{t}_{i,x}, \hat{t}_{i,y}, \hat{t}_{i,z}]^\top$。

**电机角速度**：

$$\Omega_i = \|\mathbf{t}_i\| / c_t$$

**舵机角度**（双分支解）：

$$\text{分支 1:} \quad \beta_i = \arcsin(-\hat{t}_{i,y}), \quad \alpha_i = \text{atan2}(\hat{t}_{i,x}, \hat{t}_{i,z})$$

$$\text{分支 2:} \quad \beta_i = \pi - \arcsin(\hat{t}_{i,y}), \quad \alpha_i = \pi + \text{atan2}(\hat{t}_{i,x}, \hat{t}_{i,z})$$

**最小运动选择**：比较两个分支与上一时刻舵机位置 $(\alpha_i', \beta_i')$ 的距离，选择运动量最小的分支。同时检查 $\pm 2k\pi$ 缠绕（$|k| \leq n_r$），支持连续多圈旋转。

**有效范围**：$\beta_i \in [-\pi/2, 3\pi/2]$，$\alpha_i \in [-\pi, \pi]$（通过 $\text{atan2}$ 自然覆盖）。

### 3.6 几何控制器多模式输入

几何控制器（`geo_control`）支持三种控制输入源，根据 `vehicle_status.nav_state` 自动切换：

| 模式 | nav_state | 输入来源 | 控制逻辑 |
|------|-----------|---------|---------|
| Manual | MANUAL (0) | `manual_control_setpoint` | 摇杆→角速率+速度 |
| Offboard | OFFBOARD (14) | `trajectory_setpoint` | 位置/速度/姿态设定点 |
| Auto | 其他 | `vehicle_*_setpoint` | flight_mode_manager 输出 |

#### Manual 模式摇杆映射

摇杆输入经死区处理后映射为机体坐标系参考量：

$$\omega_{sp} = \text{deadzone}(\text{stick}_{rp y}, dz) \cdot \omega_{\max}$$
$$\mathbf{v}_{sp}^B = \text{deadzone}(\text{stick}_{aux1,2,3}, dz) \cdot v_{\max}$$

其中 $dz$ 为死区参数（`GC_MAN_DZ`，默认 0.1），$\omega_{\max}$ 为最大角速率（`GC_MAN_RATE_MAX`，默认 1.5 rad/s），$v_{\max}$ 为最大速度（`GC_MAN_VEL_MAX`，默认 5.0 m/s）。

角速率参考通过四元数积分生成姿态参考 $R_{sp}$，速度参考转换到 NED 系后通过 P 环生成期望力。

### 3.7 积分器与抗饱和

为消除稳态误差，几何控制器在力矩环和力环各加入积分项：

$$\boldsymbol{\tau}_{des} = -K_R \mathbf{e}_R - K_\Omega \mathbf{e}_\omega + \mathbf{I}_{rate} + \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega})$$
$$\mathbf{f}_{des}^W = -K_P \mathbf{e}_p - K_V \mathbf{e}_v - \mathbf{I}_{vel} + m\mathbf{g} + m\mathbf{a}_{sp}$$

#### 积分更新律

参考 PX4 `RateControl` 的实现：

$$I_{rate}[k] = \text{clamp}\left(I_{rate}[k-1] + i_{factor} \cdot K_I \cdot e_\omega \cdot \Delta t,\; -I_{lim},\; +I_{lim}\right)$$

其中非线性因子 $i_{factor} = \max(0, 1 - (e_\omega / 400°)^2)$ 在大误差时降低积分增益，防止剧烈机动时的 windup。

#### 地面状态管理

| 状态 | 积分器行为 |
|------|-----------|
| 解锁 (disarmed) | 积分器清零 |
| 着陆 (landed) | 积分冻结（不更新） |
| 空中 (armed + not landed) | 正常积分 + 抗饱和 |

#### 抗饱和（条件积分）

通过 `control_allocator_status` 获取执行器饱和反馈：

- 若某轴正向饱和（`unallocated_torque[i] > 0`）：$e_\omega[i] \leftarrow \min(e_\omega[i], 0)$
- 若某轴负向饱和（`unallocated_torque[i] < 0`）：$e_\omega[i] \leftarrow \max(e_\omega[i], 0)$
- 力环同理，使用 `unallocated_thrust`

#### 饱和信号来源

`ActuatorEffectivenessMorphQuad` 重载 `getUnallocatedControl()`，在 `updateSetpoint()` 中检测执行器饱和后，从饱和后的实际输出重构 wrench：

$$\mathbf{f}_{achieved} = \sum_{i=1}^{4} c_t \Omega_{i,clamped}^2 \cdot \mathbf{G}_i(\alpha_{i,clamped})$$
$$\boldsymbol{\tau}_{achieved} = \sum_{i=1}^{4} \mathbf{l}_i \times \mathbf{t}_{i,achieved}$$
$$\mathbf{w}_{unallocated} = \mathbf{w}_{des} - \mathbf{w}_{achieved}$$

---

## 4. 算法伪代码

```
Algorithm: MorphQuad Thrust Allocation
Input:  f_des ∈ R^3, tau_des ∈ R^3, (alpha_prev, beta_prev) ∈ R^{4×2}
Output: Omega[4], alpha[4], beta[4]

// ========== Step 1: Minimum-norm decomposition ==========
l ← half_length,  b ← half_width,  r ← sqrt(l² + b²)

// Yaw unit vectors (Eq. 13)
epsi[4][3] ← { {-b/r, l/r, 0}, {b/r, l/r, 0}, {b/r, -l/r, 0}, {-b/r, -l/r, 0} }
roll_sign[4]  ← { +1, -1, -1, +1 }
pitch_sign[4] ← { -1, -1, +1, +1 }

for i = 0 to 3 do:
    t[i][0] ← f_des[0]/4 + roll_sign[i]·tau_des[0]/(4b) + tau_des[2]·epsi[i][0]/(4r)
    t[i][1] ← f_des[1]/4 - pitch_sign[i]·tau_des[1]/(4l) + tau_des[2]·epsi[i][1]/(4r)
    t[i][2] ← f_des[2]/4

// ========== Step 2: Null-space correction (Eq. 15-16) ==========
f_norm ← ||f_des||
if f_norm > 0.01 then:
    f_hat ← f_des / f_norm

    // (a) Gimbal lock avoidance
    y_corr ← 0.3 · |f_hat·j^B| · f_norm / 4
    for i = 0 to 3 do:
        t[i][1] ← t[i][1] + (-1)^i · y_corr

    // (b) Downwash avoidance
    if |f_hat·i^B| > 0.7 then:
        x_corr ← 0.2 · (f_hat·i^B) · f_norm / 10
        for i = 0 to 3 do:
            t[i][0] ← t[i][0] + (i < 2 ? +1 : -1) · x_corr

// ========== Step 3: Servo angle recovery (Eq. 14) ==========
for i = 0 to 3 do:
    t_norm ← ||t[i]||
    Omega[i] ← t_norm / c_t

    if t_norm < 0.01 then:
        alpha[i] ← alpha_prev[i],  beta[i] ← beta_prev[i]
        continue

    t_hat ← t[i] / t_norm

    // Two branches
    beta1  ←  arcsin(-t_hat[1])
    alpha1 ←  atan2(t_hat[0], t_hat[2])
    beta2  ←  π - beta1
    alpha2 ←  π + alpha1

    // Minimal-motion selection
    d1 ← |alpha1 - alpha_prev[i]| + |beta1 - beta_prev[i]|
    d2 ← |alpha2 - alpha_prev[i]| + |beta2 - beta_prev[i]|
    if d2 < d1 then:
        alpha[i] ← alpha2,  beta[i] ← beta2
    else:
        alpha[i] ← alpha1,  beta[i] ← beta1

    // Multi-revolution wrap check
    for k = -n_r to n_r, k ≠ 0:
        alpha_wrap ← alpha[i] + 2πk
        if |alpha_wrap - alpha_prev[i]| < best_dist then:
            alpha[i] ← alpha_wrap

    beta[i] ← clamp(beta[i], -π/2, 3π/2)
    alpha_prev[i] ← alpha[i],  beta_prev[i] ← beta[i]
```

> **注**：此处 pitch 项符号为负（`-pitch_sign`），与论文 Eq. (12) 一致。原伪代码中为正号，系符号推导错误。正确推导：$\tau_y = \sum(l_{ix} t_{iz} - l_{iz} t_{ix})$，代入 X 型布局旋翼位置可得 $t_{iy}$ 的 pitch 贡献符号为 $[-1, +1, +1, -1]$。

---

## 5. 关键设计决策

### 5.1 为什么选择 Pattern B（updateSetpoint 覆盖）而非 Pattern A（矩阵填充）

PX4 的控制分配框架支持两种模式：
- **Pattern A**：在 `getEffectivenessMatrix()` 中填充 B 矩阵，框架自动做 $\mathbf{u} = \mathbf{B}^{\text{pinv}} \mathbf{c}$
- **Pattern B**：注册零矩阵，在 `updateSetpoint()` 中完全自定义分配逻辑

MorphQuad 选择 **Pattern B**，原因：
1. B 矩阵依赖于舵机角度 $(\alpha_i, \beta_i)$，是时变的，而 PX4 框架假设矩阵静态
2. 论文算法包含非线性步骤（零空间修正、双分支选择），无法用线性矩阵表示
3. 参考 `ActuatorEffectivenessHelicopter` 的成功先例（同样使用 Pattern B 处理斜盘非线性）

### 5.2 零空间修正的简化实现

论文 Eq. (16) 给出了完整的 6 维系数向量 $\mathbf{c}$ 选择策略。本实现做了以下简化：

| 论文原文 | 本实现 | 理由 |
|---------|--------|------|
| 完整的 $\mathbf{N}_M$ 零空间基投影 | 直接在推力向量上添加 y/x 分量修正 | 避免 6×6 矩阵运算，降低计算负担 |
| 条件分支选择 $c_2, c_4, c_6$ | 统一的前后对交替符号 | 简化逻辑，效果等价 |
| 精确的 $\langle \hat{\mathbf{f}}, j^B \rangle$ 投影 | 使用 $|\hat{f}_y|$ 近似 | NED 坐标系下 $j^B = [0,1,0]^\top$，内积即 $f_y$ 分量 |

增益系数 $k_{\text{gimbal}} = 0.3$、$k_{\text{downwash}} = 0.2$ 为可调参数，需通过飞行实验整定。

### 5.3 连续旋转支持

MorphQuad 的舵机支持 ±1080°（三圈）连续旋转。算法通过以下方式支持：
1. 双分支解覆盖 $\beta \in [-\pi/2, 3\pi/2]$
2. $\pm 2k\pi$ 缠绕检查（$n_r = 2$），确保选择运动量最小的缠绕方式
3. 保存上一时刻角度 `_alpha_prev`、`_beta_prev` 作为参考

### 5.4 积分器设计

积分器参考 PX4 `RateControl` 类（`src/lib/rate_control/`）的实现模式：
- 一阶矩形积分，带限幅
- 非线性 I 因子抑制大误差 windup
- 条件积分抗饱和（执行器饱和方向停止积分）
- 地面冻结 + 解锁清零

这比经典 back-calculation 抗饱和更简单，且已被 PX4 多旋翼控制器验证。

---

## 6. 计算复杂度分析

| 步骤 | 运算量 | 说明 |
|------|--------|------|
| 最小范数分解 | ~40 次乘加 | 4 旋翼 × 3 分量，含符号查找表 |
| 零空间修正 | ~20 次乘加 + 2 次条件判断 | 仅在 $\|\mathbf{f}\| > 0.01$ 时执行 |
| 舵机角度反解 | ~30 次运算/旋翼 | 含 atan2、asin、距离比较 |
| **总计** | **~200 次浮点运算** | 在 800 Hz 控制频率下完全可行 |

所有运算均为标量浮点操作，无矩阵求逆或迭代，适合嵌入式实时执行。

---

## 7. 稳定性保证

**推论 1**（论文）：零空间修正不改变 wrench，因此几何控制器的几乎处处指数稳定性不受影响。

**实际含义**：
- 闭环误差动力学与无修正时完全相同
- 收敛到任意期望位姿（除零测集外）
- 零空间修正仅影响执行器层面的内部状态（舵机角度），不影响飞行器的外环跟踪性能

---

## 8. 参考文献

1. Lee, T., Leok, M., McClamroch, N.H. "Geometric tracking control of a quadrotor UAV on SE(3)." CDC 2010.
2. Díaz Peón González Pacheco, J. et al. "MorphQuad: Morphable Quadrotor for Superhuman Maneuverability, Manipulation, and Resiliency." arXiv:2607.02764v3, 2026.
3. Michael, N. et al. "The GRASP multiple micro-UAV testbed." IEEE RAM 2010.
4. Wu, Z. et al. "L1Quad: L1 Adaptive Augmentation of Geometric Control for Agile Quadrotors." IEEE TCST 2025.
