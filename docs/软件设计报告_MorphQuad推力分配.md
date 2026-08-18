# MorphQuad 推力分配软件设计报告

## 1. 目的与范围

本文档描述 PX4 中 MorphQuad 的六维力/力矩（wrench）控制分配实现，以及它与 `geo_control`、PX4 控制分配框架和 JSBSim 保持架仿真的接口。本文档以当前源码为准，不以早期任务书或原型实现为准。

参考论文为 *MorphQuad: Morphable Quadrotor for Superhuman Maneuverability, Manipulation, and Resiliency*（arXiv:2607.02764v3）。论文的推力向量分配、零空间修正和逆运动学分支选择被采用；实际平台的舵机行程为 ±180°，因此没有采用论文硬件的 ±1080° 多圈行程。

本设计覆盖：

- `ActuatorEffectivenessMorphQuad` 中的推力向量分配；
- `geo_control` 到 `ControlAllocator` 的 6D wrench 接口；
- 12 路执行器（4 路电机、8 路两轴万向节舵机）映射；
- JSBSim 模型、保持架验证和校准参数。

## 2. 架构与数据流

```text
geo_control
  vehicle_thrust_setpoint + vehicle_torque_setpoint  (Body-FRD, N / Nm)
       │
       ▼
ControlAllocator
       │  [Mx, My, Mz, Fx, Fy, Fz]
       ▼
ActuatorEffectivenessMorphQuad::updateSetpoint()
       │
       ├─ actuator_motors[0..3]  : 归一化转速
       └─ actuator_servos[0..7]  : alpha0,beta0,...,alpha3,beta3
       │
       ▼
SimulatorMavlink → MAVLink HIL_ACTUATOR_CONTROLS → jsbsim_bridge → JSBSim
```

`ActuatorEffectivenessMorphQuad` 采用 PX4 自定义分配路径（Pattern B）：

- `getEffectivenessMatrix()` 只注册 4 个 `MOTORS` 和 8 个 `SERVOS`；
- 实际的非线性分配在 `updateSetpoint()` 完成；
- 这不是普通四旋翼 mixer，也不是由当前舵角决定的 6×4 伪逆。先求解 12 维推力向量，再反解舵角和电机指令。

## 3. 坐标、几何和执行器编号

### 3.1 坐标约定

所有力、力矩和旋翼位置都在 PX4 Body-FRD 坐标系中表示：x 向前、y 向右、z 向下。悬停推力为负 z。

四个旋翼系统的位置为：

| 索引 | 位置 | 说明 |
|---|---|---|
| 0 | `(+L, +B, Z)` | 右前 RF |
| 1 | `(+L, -B, Z)` | 左前 LF |
| 2 | `(-L, -B, Z)` | 左后 LB |
| 3 | `(-L, +B, Z)` | 右后 RB |

JSBSim 模型的旋翼平面在质心上方 4 cm，因此默认 `CA_MQ_Z=-0.04 m`。这一项不能省略：水平推力会通过 `r × t` 产生滚转/俯仰力矩。

### 3.2 执行器编号

| PX4 输出索引 | 含义 | 范围 |
|---|---|---|
| `actuator_motors[0..3]` | 四个同轴旋翼系统的归一化转速 | `[0, 1]` |
| `actuator_servos[0]`、`[1]` | `alpha0`、`beta0` | `[-1, 1]` |
| `actuator_servos[2]`、`[3]` | `alpha1`、`beta1` | `[-1, 1]` |
| `actuator_servos[4]`、`[5]` | `alpha2`、`beta2` | `[-1, 1]` |
| `actuator_servos[6]`、`[7]` | `alpha3`、`beta3` | `[-1, 1]` |

舵机的唯一归一化约定是

$$u_{\alpha_i}=\alpha_i/\pi, \qquad u_{\beta_i}=\beta_i/\pi.$$

也就是说，`-1 … +1` 对应实际 `-180° … +180°`。`px4-jsbsim-bridge/configs/morphquad.xml` 必须以 `scale=π` 把这八路值恢复为弧度；不得使用 `π/2`，否则实际倾角会被缩小一半。

## 4. 控制分配设计

### 4.1 推力方向和推力幅值

每个旋翼系统的单位推力方向为

$$
\mathbf g(\alpha,\beta)=
\begin{bmatrix}
 \sin\alpha\cos\beta\\
 \sin\beta\\
 -\cos\alpha\cos\beta
\end{bmatrix}.
$$

中性舵角为向上推力（负 z）。当 `alpha=±π`、`beta=0` 时，推力沿正 z；因此实际 ±180° 机构可覆盖球面方向，但不能执行多圈连续旋转。

控制分配器使用简化的归一化推力模型：

$$T_i=T_{max}u_{m,i}^2, \qquad \mathbf t_i=T_i\mathbf g_i.$$

因此输出电机指令是 `u_m = sqrt(T / CA_MQ_TMAX)`。JSBSim 的电机/螺旋桨模型是动态、非严格二次的；该差异用保持架基线增量测试进行评估，而非改变分配几何。

### 4.2 常数输入矩阵

令

$$
\mathbf w=\begin{bmatrix}\mathbf f\\\boldsymbol\tau\end{bmatrix},
\qquad
\mathbf t=\begin{bmatrix}\mathbf t_0\\\mathbf t_1\\\mathbf t_2\\\mathbf t_3\end{bmatrix}.
$$

对任意旋翼位置 `r_i`，有

$$
\mathbf f=\sum_i\mathbf t_i, \qquad
\boldsymbol\tau=\sum_i\mathbf r_i\times\mathbf t_i,
\qquad \mathbf w=\mathbf M\mathbf t.
$$

`M` 是由实际旋翼位置构造的 6×12 常数矩阵。对第 i 个旋翼，其矩阵块为 `[I; [r_i]_×]`。实现保留了 `Z` 项：

$$
\mathbf r_i\times\mathbf t_i=
\begin{bmatrix}
y_i t_{z,i}-Zt_{y,i}\\
Zt_{x,i}-x_i t_{z,i}\\
x_i t_{y,i}-y_i t_{x,i}
\end{bmatrix}.
$$

这与论文的推力向量形式一致，并比忽略旋翼高度差的平面近似更符合 JSBSim 模型。

### 4.3 最小范数和零空间分配

首先计算最小范数解：

$$\mathbf t_0=\mathbf M^+\mathbf w.$$

随后按论文 Eq. (15)、(16)、(19) 应用显式零空间基：

$$\mathbf t=\mathbf t_0+\mathbf N_M\mathbf c.$$

`c` 由期望力的单位方向确定，用于远离 β 轴奇异位姿并打破沿 x 轴的共线推力。源码在运行时检查 `M * N_M`；公共高度偏移 `Z` 不改变该零空间基的零合力性质。

若任一旋翼系统的推力幅值超过 `CA_MQ_TMAX`，分配器将整组 12 维推力向量按同一比例缩小。它不会分别裁剪 x/y/z 分量，以免改变推力方向。饱和后的实际 wrench 通过 `control_allocator_status.unallocated_*` 反馈给上层抗积分饱和逻辑。

### 4.4 逆运动学与机械约束

对非零 `t_i`，采用论文的两个等价反解分支：

$$
\beta_1=\arcsin(\hat t_y),\quad
\alpha_1=\operatorname{atan2}(\hat t_x,-\hat t_z),
$$

$$
\beta_2=\pi-\beta_1,\quad
\alpha_2=\pi+\alpha_1.
$$

实现比较两个分支及其 `2π` 等价角，选择与上一周期角度距离最小的一对。最后施加 `CA_MQ_ALPM` 和 `CA_MQ_BETM` 限制；本机默认均为 `180°`。零推力时保持上一舵角，避免无意义的抖动。

## 5. 参数与标定

| 参数 | 当前 JSBSim 默认值 | 含义 |
|---|---:|---|
| `CA_MQ_L` | `0.25 m` | 质心到前/后旋翼的半臂长 |
| `CA_MQ_B` | `0.225 m` | 质心到左/右旋翼的半臂宽 |
| `CA_MQ_Z` | `-0.04 m` | 旋翼平面相对质心的 FRD z 坐标 |
| `CA_MQ_TMAX` | `39.5 N` | 单个旋翼系统的分配模型最大推力 |
| `CA_MQ_ALPM` | `180 deg` | alpha 机械限位 |
| `CA_MQ_BETM` | `180 deg` | beta 机械限位 |
| `GC_THRUST_MAX` | `158 N` | 四个旋翼系统总最大推力，用于 `geo_control` 调试归一化 |

`CA_MQ_TMAX=39.5 N` 来自保持架校准：`esc≈0.638` 时每个同轴旋翼系统约为 `16.05 N`。它是分配模型参数，不替代螺旋桨全工作包线标定。

## 6. JSBSim 与保持架验证

### 6.1 模型接口

`px4-jsbsim-bridge` 中的关键接口如下：

- `configs/morphquad.xml`：保持 `<hold_down>1</hold_down>`，并将舵机通道缩放为 `π`；
- `models/morphquad/GimbalVectoring.xml`：将 `(alpha,beta)` 转换为 JSBSim 推进器 pitch/yaw；pitch 必须使用
  `atan2(cos(alpha)cos(beta), hypot(tx,ty))`，以保留 z 方向符号；
- `sensor_imu_plugin.cpp`：在保持架模式发布符合 FRD 的重力比力，供 PX4 EKF2 正确初始化；
- `SimulatorMavlink` 的 `SIM_MQ_BENCH=1`：仅在未解锁的保持架验证中直通这 12 路指令，避免正常控制器和解锁策略干扰测试。

`SIM_MQ_BENCH` 默认关闭，正常飞行路径不使用该旁路。正常飞行若需要解锁，应先进入 Acro，再按 PX4 标准解锁流程执行。

### 6.2 `geo_control verify`

`geo_control verify` 在保持架模式执行 60 组单轴 6D wrench 测试：Fx、Fy、Fz、Tx、Ty、Tz 各 10 个幅值。每个样本：

1. 先施加悬停 wrench 并采集 JSBSim 稳态基线；
2. 注入目标 wrench，且在同步 PXH 命令内部持续发布，避免 `hold_down` 时没有本地位置更新而停止发布；
3. 从 `actuator_motors` 和 `actuator_servos` 做前向运动学重构；
4. 从 bridge UDP `:9871` 接收 JSBSim 物理力/力矩，比较相对悬停基线的增量；
5. 清零注入并回到悬停后进入下一组。

完整端到端链路为：

```text
geo_control → ControlAllocator → SimulatorMavlink → MAVLink
→ jsbsim_bridge → JSBSim → UDP wrench feedback → geo_control
```

自动脚本为 `scripts/auto_verify.sh`。它通过 bridge 日志确认 server 已监听，不能使用 `nc -z` 探测端口：JSBSim server 只有一个客户端槽位，探测连接会抢占并断开真正的 bridge。

最近一次 60 组保持架复测的关键结果：

- 所有样本的分配器前向重构残差为 0（未饱和样本）；
- 修复前，请求 `Fz=+10.27 N` 会在 JSBSim 得到约 `-12.01 N`；
- 修复 `GimbalVectoring.xml` 的 z 符号后，同一请求得到约 `+12.01 N`；
- 物理增量与 `esc²·TMAX` 的差异在低推力时更明显，属于螺旋桨动态/静态模型标定误差，应通过推力曲线标定改善。

### 6.3 `geo_control att_verify`

`att_verify` 是自由运动仿真中的自动姿态闭环测试，不能在 `hold_down=1` 时使用。启动条件为：

1. 将 JSBSim 的 `hold_down` 设为 `0` 并重启仿真；
2. 确认 `SIM_MQ_BENCH=0`；
3. 先切入 Acro，再按 PX4 标准流程解锁；
4. 执行 `geo_control att_verify`。

命令会依次测试 `roll ±10°`、`pitch ±10°`、`yaw ±15°` 和水平回中，共 7 步。每步保持 4 秒，后 2 秒统计三个欧拉角的平均绝对误差和机体平均角速度；默认判据为每轴平均误差不大于 3°、平均角速度不大于 0.35 rad/s。每一步输出 `PASS` 或 `FAIL`，完成后输出总通过数，并自动切换到零姿态的悬停保持。测试完成后应执行 `geo_control att_test stop`，再切换安全模式或 `disarm`。

`att_verify` 不自动修改飞行模式、解锁状态或 `hold_down`，以避免测试命令绕过 PX4 的飞行安全状态机。

## 7. 文件清单

| 文件 | 职责 |
|---|---|
| `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMorphQuad.cpp` | 推力向量分配、零空间、逆运动学、饱和反馈 |
| `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMorphQuad.hpp` | 参数与内部状态声明 |
| `src/modules/control_allocator/module.yaml` | `CA_MQ_*` 参数定义 |
| `src/modules/geo_control/geo_control.cpp` | wrench 发布和 60 组端到端验证 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/19001_jsbsim_morphquad` | JSBSim MorphQuad 默认参数 |
| `../px4-jsbsim-bridge/configs/morphquad.xml` | HIL/保持架和 12 路通道映射 |
| `../px4-jsbsim-bridge/models/morphquad/GimbalVectoring.xml` | 万向节方向到 JSBSim 推进器姿态的映射 |
| `../scripts/auto_verify.sh` | 自动保持架验证 |
| `../scripts/auto_attitude_verify.sh` | 自动自由飞行姿态闭环验证（先 Acro 后解锁） |

## 8. 限制与后续工作

- 当前分配器假设所有旋翼系统共面且共享同一 `CA_MQ_Z`；若硬件四个旋翼高度不同，需要扩展为逐旋翼位置参数。
- `T=Tmax·esc²` 是用于分配和前向重构的近似。应在多组转速、正反向舵角下标定 JSBSim 或实机推力曲线。
- 当前硬件受 ±180° 限制，因此不具备论文硬件的连续多圈机动能力；若更换多圈舵机，需要同时扩展 PX4 归一化协议、机械参数和 bridge 映射。
- 保持架验证用于核对执行器/模型/分配链路，不替代自由飞行的闭环稳定性与安全验证。

## 9. 版本记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v0.7 | 2026-08-18 | 以当前代码重写：加入 `CA_MQ_Z`、±180°映射、JSBSim z 符号修复、保持架验证路径与校准说明 |
| v0.8 | 2026-08-18 | 新增 `geo_control att_verify` 自动姿态闭环阶跃测试和评分标准 |
