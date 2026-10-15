# Dock Guidance 控制律与软件设计

上机配置、模式进入、参数速查和诊断步骤见 [DockGuidance 用户使用说明](DockGuidance_用户使用说明.md)。

## 1. 设计目标

Dock Guidance 同时支持固定安装相机和单轴俯仰舵机相机。正常高速阶段以 `DOCK_VEL_MAX` 为三维速度预算，目标居中时速度方向沿目标 LOS。固定相机用垂直平移修正图像误差；俯仰舵机相机用慢速机身相对角度调整保持构图，舵机不承担姿态增稳。

本版本不再用接近速度误差直接增加总油门，也不再强制固定前倾角。Dock Guidance 生成同时包含接近分量和图像跟踪分量的运动学速度设定值，由 PX4 `mc_pos_control` 完成速度、高度、姿态和推力耦合。

高速对接的目标需求是：预对准和普通接近阶段保持目标在视场中，进入末段后切换到 `TERMINAL_DASH`，以允许的最大三维闭合速度完成对接，不根据距离、目标框尺寸或 FOV 裕量执行末端减速。目标在极近距离充满画面并短时丢失时，终端阶段保持最后有效 LOS 和航向，在有界时间内继续最大速度指令，直到收到对接完成信号或终端超时。

> 实现状态：当前源码已实现 `TERMINAL_DASH`、目标框/距离触发和有界终端盲飞。真实接触/对接完成输入尚未接入；在此之前，`DOCK_TERM_BLIND` 与 `DOCK_TERM_MAX_T` 仅作为故障边界，不能解释为对接成功。

## 2. 控制链

```text
viztra_observation
    -> DockGuidance
    -> trajectory_setpoint
    -> mc_pos_control
    -> mc_att_control
    -> mc_rate_control
    -> control_allocator

MT6701 (I2C) -> sensor_encoder -> DockGuidance 相机角度历史
DockGuidance -> gimbal_controls -> Gimbal Pitch (421) 舵机输出
```

编码器只提供相机相对机体的**实测俯仰角**，参与视觉 LOS 的坐标变换；舵机仍由视觉俯仰误差生成原有命令，编码器不进入舵机控制律。`DOCK_CAM_ENC_EN=0` 时继续使用命令角历史。

Dock 外部模式使能：

```text
flag_control_velocity_enabled
flag_control_altitude_enabled
flag_control_climb_rate_enabled
flag_control_attitude_enabled
flag_control_rates_enabled
flag_control_allocation_enabled
```

进入模式需要有效的本地位置、高度、姿态和角速度估计。

## 3. 输入与输出

### 3.1 输入

| 主题 | 用途 |
|---|---|
| `viztra_observation` | 目标画面偏差、相对 NED 位置、有效位和时间戳 |
| `vehicle_attitude` | 相机 LOS 到 NED 的旋转和偏航对准 |
| `vehicle_angular_velocity` | 延迟对齐历史和模式健康检查 |
| `vehicle_local_position` | EKF 融合速度、接近速度和位置有效性 |
| `vehicle_status` | Dock 模式激活状态 |
| `sensor_encoder` | MT6701 原始绝对角、采样时间和设备 ID；启用编码器后用于曝光时刻的相机俯仰角 |

### 3.2 输出

| 主题 | 用途 |
|---|---|
| `trajectory_setpoint` | NED 三维速度和偏航设定值 |
| `dock_guidance_setpoint` | 阶段、速度向量、距离、FOV 余量和有效位 |
| `config_control_setpoints` | 声明 Dock 模式使用 PX4 位置/姿态/角速度控制链 |
| `gimbal_controls` | 单轴相机舵机的俯仰命令；不接受编码器角度反馈 |

## 4. LOS 与相对位置

相机坐标系为 X 向前、Y 向右、Z 向下。视觉 pitch 向上为正。

像素偏差转角度：

\[
\theta=\operatorname{atan}\left(\frac{2x}{N}\tan\frac{FOV}{2}\right)
\]

相机 LOS 单位向量：

\[
l_c=\begin{bmatrix}
\cos\theta_c\cos\psi_c\\
\cos\theta_c\sin\psi_c\\
-\sin\theta_c
\end{bmatrix}
\]

通过相机安装矩阵和机体姿态转换到 NED，并对单位向量低通滤得到 `los_ned_unit`。

相机安装矩阵使用 `DOCK_CAM_ROLL`、**曝光时刻俯仰角**、`DOCK_CAM_YAW` 构造。曝光时刻为 `viztra_observation.timestamp_sample - DOCK_CAM_DLY`。关闭 `DOCK_CAM_ENC_EN` 时，俯仰角来自历史舵机命令（固定相机时为 `DOCK_CAM_PITCH`）；开启时，来自 `sensor_encoder.raw_angle` 经零位、方向换算后的实测历史。机体姿态、本机位置和相机角度在同一个曝光时刻取样；实测历史不足时拒绝该帧，不把当前角或舵机命令代入旧图像。详细换算、时序与故障条件见 [MT6701 相机角度使用说明](DockGuidance_MT6701_相机角度使用说明.md)。

`relative_n_m/e_m/d_m` 用于计算斜距：

\[
d=\sqrt{r_N^2+r_E^2+r_D^2}
\]

有相对 NED 位置时，高速接近同时具备 LOS、距离和本地速度，可用于阶段判定和监视。兼容旧 21 字节 Viztra 帧时，`DOCK_LEGACY_APP=1` 允许仅依赖 LOS 进入高速接近。目标终端冲刺设计不使用距离减速，但仍优先保留相对位置，因为它可为终端进入条件、轨迹一致性检查和日志复盘提供可靠依据。

## 5. 视场安全度量

\[
q_x=\frac{|e_{yaw}|}{FOV_X/2},\qquad
q_y=\frac{|e_{pitch}|}{FOV_Y/2}
\]

\[
q=\max(q_x,q_y)
\]

`q=1` 表示目标到达画面边缘。默认阈值：

| 参数 | 默认值 | 行为 |
|---|---:|---|
| `DOCK_FOV_ALIGN` | 0.30 | 进入高速流程前必须连续保持在此区域 |
| `DOCK_FOV_SOFT` | 0.65 | 退出高速阶段并进入 `RECOVERY` |
| `DOCK_FOV_HARD` | 0.85 | 连续越界后进入 `EMERGENCY_RECOVERY`，目标仍有效时不退出 Dock |

## 6. 固定相机图像跟踪

仅沿 LOS 飞行只能保持地理系 LOS 不变，不能补偿多旋翼加速倾斜造成的固定相机光轴运动。控制器首先计算相机光轴 `f_c` 相对目标 LOS `l` 的切平面误差：

\[
e_{point}=f_c-l(l^T f_c)
\]

水平指向误差由偏航设定值直接处理。为了避免平移与偏航互相对抗，先从 `e_point` 中移除能够由机体 yaw 修正的分量，剩余误差再转换为平移速度：

\[
v_{track}=-d K_{img}e_{remaining}
\]

其中 `d` 为目标斜距，`K_img=DOCK_IMG_P`。`v_track` 再次投影到 LOS 垂直平面并限制到 `DOCK_IMG_VMAX`。目标在固定相机上方时会产生受限的上升修正，目标在下方时产生下降修正；所有方向均通过相机安装矩阵计算，不依赖相机绕光轴的安装角。

若旧帧没有距离，图像伺服退化为按剩余视线角比例生成的角度纠偏速度。`ALIGN` 阶段受 `DOCK_ALIGN_SPD` 限制，接近阶段受 `DOCK_IMG_VMAX` 限制。图像修正最多使用 `DOCK_IMG_FRAC` 比例的总速度预算，并禁止最终水平速度反向；因此固定镜头修正不会再次耗尽95%的接近速度预算。

### 6.1 单轴俯仰舵机版本

`DOCK_CAM_SERVO=1` 时，垂直图像居中改由相机俯仰舵机负责，飞行器不再同时生成垂直图像平移修正，避免两个执行机构争抢同一误差。水平误差仍由机体 yaw 设定值处理。

舵机只使用相机坐标系中的持续俯仰误差：误差先经过 `DOCK_CAM_P_DB` 死区，再除以 `DOCK_CAM_P_TC` 得到慢速角度变化率，并受 `DOCK_CAM_P_RATE` 限制。舵机指令不使用机体姿态、角速度或编码器反馈，因此不是增稳云台。`DOCK_CAM_ENC_EN=0` 时，曝光时刻的舵机命令历史用于相机到机体矩阵；`DOCK_CAM_ENC_EN=1` 时改用 MT6701 实测角历史，舵机命令本身不变。

将一个未使用的 PWM 输出功能设置为 `Gimbal Pitch (421)`。`DOCK_CAM_P_MIN/MAX` 中较小/较大角度分别映射到 `DOCK_CAM_OUT_LO/HI`，默认值为 -1/+1；可按实测相机角与 PWM 值调整。该直接输出方式不需要启动 PX4 gimbal 模块，建议保持 `MNT_MODE_IN=-1`，避免多个发布者同时控制同一个输出。

### 6.2 对准阶段

`ALIGN` 阶段不产生沿 LOS 的高速指令，只使用图像跟踪速度，并额外限制到 `DOCK_ALIGN_SPD`。目标在对准区连续保持 `DOCK_ALIGN_T` 后，有距离时直接进入 `ACCELERATE`；无距离时仅在 `DOCK_LEGACY_APP=1` 时进入。

## 7. 最大速度接近

图像跟踪速度与 LOS 接近速度共享同一个三维速度预算。由于 `v_track` 垂直于 LOS：

\[
v_{close,cmd}=\sqrt{v_{cmd}^2-\|v_{track}\|^2}
\]

\[
v_{sp}=v_{close,cmd}l_{NED}+v_{track}
\]

`ACCELERATE` 中：

\[
\dot v_{cmd}\le DOCK\_ACC\_MAX
\]

距离足够且达到速度上限后：

\[
\|v_{sp}\|=v_{cmd}=DOCK\_VEL\_MAX
\]

因此目标居中时 `v_track=0`，全部速度用于接近；目标发生偏移时，接近分量暂时让出一部分速度预算给图像跟踪，但最终三维速度不会超过 `DOCK_VEL_MAX`。最终三维速度向量还受 `DOCK_ACC_MAX/DOCK_DEC_MAX` 斜率限制，LOS 或图像修正方向变化不能形成瞬时反向指令。

当前实现中，没有相对位置时即使显式启用 `DOCK_LEGACY_APP`，速度仍受 `DOCK_LEG_VMAX` 限制。该模式只用于受控兼容测试，不作为默认高速方案；把它扩展到 `40 m/s` 必须先完成第 7.1 节的限值修改和逐级验证。

当前普通接近阶段，当 FOV 从 `DOCK_FOV_ALIGN` 接近 `DOCK_FOV_SOFT` 时，允许速度按剩余视场裕量连续降到零，避免在加速和恢复状态之间形成二值振荡。目标设计进入 `TERMINAL_DASH` 后不再应用该速度缩放。

为了避免 `mc_pos_control` 在各轴分别限幅后改变速度方向，Dock Guidance 在发布前根据以下限制等比缩放整个速度向量：

- `MPC_XY_VEL_MAX`
- `MPC_Z_VEL_MAX_UP`
- `MPC_Z_VEL_MAX_DN`

因此“最大速度”表示三维闭合速度，不是单独的 `vehicle_local_position.vx`。可用三维速度上限为：

\[
v_{available}=\min\left(
v_{dock},
\frac{MPC\_XY\_VEL\_MAX}{\|l_{NE}\|},
\frac{MPC\_Z\_VEL\_MAX\_UP}{-l_D}
\right)
\]

其中只在相应 LOS 分量非零时使用该轴约束，`v_dock` 为有距离模式的 `DOCK_VEL_MAX` 或无距离模式的 `min(DOCK_VEL_MAX, DOCK_LEG_VMAX)`。

### 7.1 40 m/s 速度定义

固定相机安装角和目标实际仰角约为 60° 时：

\[
v_{horizontal}=v_{3D}\cos60^\circ,\qquad
v_{up}=v_{3D}\sin60^\circ
\]

若三维闭合速度要求为 `40 m/s`：

```text
三维闭合速度 = 40.0 m/s
水平速度     = 20.0 m/s
爬升速度     = 34.6 m/s
```

因此日志中 `vx=2.5 m/s`、`vz=-4.33 m/s` 对应的正是约 `5 m/s` 三维闭合速度，而不是控制器只能达到 `2.5 m/s`。如果需求是水平速度本身达到 `40 m/s`，同一几何下需要约 `80 m/s` 三维速度和 `69.3 m/s` 爬升速度，这属于不同且显著更高的需求。

当前无距离路径还存在两项实现限制：`DOCK_LEG_VMAX` 参数元数据上限为 `20 m/s`，控制代码也将其硬限制到 `20 m/s`。实现无距离 `40 m/s` 前必须同时提高这两个上限，并确保 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP` 和实际动力系统能够满足上述分量。

加速距离必须单独校核：

\[
t_{acc}=\frac{v_{target}}{a},\qquad
d_{acc}=\frac{v_{target}^2}{2a}
\]

从零加速到 `40 m/s` 时，`DOCK_ACC_MAX=3 m/s^2` 需要约 `13.3 s/266.7 m`；提高到 `8 m/s^2` 后仍需要约 `5 s/100 m`。参数上限、模型可达性和真实飞行器结构/动力余量必须分别验证。

## 8. 最大速度终端冲刺

### 8.1 设计原则

- 最后阶段不执行距离制动、目标框制动或 FOV 速度包络降速。
- `TERMINAL_DASH` 中速度预算保持为当前模式允许的最大值：有距离时为 `DOCK_VEL_MAX`，无距离时为 `min(DOCK_VEL_MAX, DOCK_LEG_VMAX)`。
- 视觉有效时继续更新 LOS、图像修正和航向，但图像修正只能改变速度方向，不能降低最终三维速度预算。
- 极近距离视觉失效是允许事件：冻结最后有效 LOS、图像修正和航向，在 `DOCK_TERM_BLIND` 内继续发布最大速度指令。
- 终端冲刺必须由实际接触开关或对接完成信号结束；若没有完成信号，则由 `DOCK_TERM_MAX_T` 强制结束并进入故障处理，禁止无限盲飞。
- 本地速度、姿态或控制链失效仍属于立即退出条件，不能被终端冲刺屏蔽。

### 8.2 进入条件

终端进入条件采用两个可选来源，任一满足即可：

- 相对位置有效且 `relative_distance_m <= DOCK_TERM_DIST`；
- 目标框归一化尺寸达到 `DOCK_TERM_SIZE`：

\[
s_{target}=\max\left(
\frac{target\_width\_px}{DOCK\_RES\_X},
\frac{target\_height\_px}{DOCK\_RES\_Y}
\right)
\]

目标框只用于确认“已经进入末段”，不能用于生成减速指令。进入条件需要连续满足 `DOCK_TERM_CFM`，避免单帧尺寸异常误触发。进入 `TERMINAL_DASH` 后必须锁存，不能因为目标框变小或距离抖动退回普通接近阶段。

### 8.3 退出条件

优先级从高到低：

1. 对接完成/接触信号：预留给后续接入；接入后结束 Dock Guidance，并交给对接后的专用模式处理。
2. 本地状态或控制链失效：立即故障退出。
3. 视觉丢失超过 `DOCK_TERM_BLIND`，或终端阶段总时长超过 `DOCK_TERM_MAX_T`：锁存故障并执行系统配置的安全回退。

视觉失效不能再直接把速度设定从最大值跳到零，否则会产生较大的俯仰角速度指令。终端超时后的具体安全动作由上层模式/任务设计确定，但不能在 Dock Guidance 内无限保持最大速度。

### 8.4 与普通接近逻辑的兼容关系

通过 `DOCK_TERM_EN` 控制两种行为：

- `DOCK_TERM_EN=0`：保留当前 `TERMINAL_BRAKE`、FOV 恢复和视觉失效制动逻辑。
- `DOCK_TERM_EN=1`：静态与动态正常接近从一开始就不使用 `DOCK_STOP_DIST` 距离制动，
  满足终端条件后进入 `TERMINAL_DASH`；终端内也不使用 FOV 速度缩放。
  以现有 `PASS_PREDICTED` 判定预测通过后，才开始正常恢复制动。

当前实现已通过模块编译；仍需用新 ULog 验证实际穿越精度、末段速度保持和超时边界。

## 9. 视觉丢失和恢复

- `TERMINAL_DASH` 之前，目标超过软边界：进入 `RECOVERY`，按 `DOCK_DEC_MAX` 降低速度指令。
- `TERMINAL_DASH` 之前，目标连续超过硬边界 `DOCK_FOV_H_T`：进入 `EMERGENCY_RECOVERY`，用两倍恢复减速度停止接近，但继续图像/舵机跟踪并保持 Dock 健康。
- `RECOVERY` 只有在速度接近零、目标回到对准区并连续稳定 `DOCK_REC_T` 后才能返回 `ALIGN`。
- 普通接近阶段短时断流：保持模式 valid，但速度指令立即开始减小；超过 `max(DOCK_OBS_TOUT, DOCK_VIS_GRACE)` 后锁存故障。
- `TERMINAL_DASH` 中短时断流：保持最后有效 LOS、图像修正和航向，并继续最大速度；超过 `DOCK_TERM_BLIND` 才锁存故障。
- 相对位置超过 `DOCK_REL_TOUT`：`DOCK_LEGACY_APP=0` 时减速回到 `ALIGN`；开启旧帧接近时继续角度跟踪，但不使用距离制动。
- 本地速度无效：立即 invalid 并锁存。目标进入硬视场边界本身不再触发 RTL。

## 10. 航向抑振

水平图像误差小于 `DOCK_YAW_DB` 时保持航向。LOS 水平分量在 0.25 到 0.60 之间时连续降低航向修正权重，小于 0.25 时保持航向，避免高仰角/俯角下地理系方位投影放大。最终航向设定由 `DOCK_YAW_RATE` 限制变化率；`RECOVERY`、`EMERGENCY_RECOVERY` 和短时视觉降级期间保持最后有效航向。

## 11. 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `DOCK_CAM_DLY` | 0 ms | 相机延迟补偿 |
| `DOCK_CAM_SERVO` | 0 | 0=固定镜头，1=单轴俯仰舵机镜头 |
| `DOCK_CAM_ROLL/PITCH/YAW` | -90/0/0 deg | 相机到机体安装欧拉角；启用编码器时俯仰分量由实测角替换，`DOCK_CAM_PITCH` 不与实测值相加 |
| `DOCK_CAM_ENC_EN` | 0 | 1=用 MT6701 实测俯仰角计算相机到机体矩阵；需要有效零位和新鲜数据 |
| `DOCK_CAM_ENC_Z` | -1 count | 相机相对机体零度时的 MT6701 原始计数；-1 表示未标定 |
| `DOCK_CAM_ENC_REV` | 0 | 1=反转实测俯仰角符号；与舵机 PWM 反向设置无关 |
| `DOCK_CAM_P_MIN/MAX` | 0/90 deg | 舵机机械角度范围，对应输出 -1/+1 |
| `DOCK_CAM_P_RATE` | 5 deg/s | 舵机最大慢速转动速率 |
| `DOCK_CAM_P_TC` | 2 s | 图像俯仰误差到舵机角速度的时间常数 |
| `DOCK_CAM_P_DB` | 0.5 deg | 舵机图像误差死区 |
| `DOCK_OBS_TOUT` | 150 ms | 视觉帧超时 |
| `DOCK_REL_TOUT` | 200 ms | 相对 NED 位置有效期 |
| `DOCK_FOV_X/Y` | 60/45 deg | 水平/垂直视场 |
| `DOCK_FOV_ALIGN` | 0.30 | 对准区比例 |
| `DOCK_FOV_SOFT` | 0.65 | 软边界比例 |
| `DOCK_FOV_HARD` | 0.85 | 硬边界比例 |
| `DOCK_FOV_H_T` | 0.15 s | 硬边界确认时间，过滤单帧噪声 |
| `DOCK_REC_T` | 1.0 s | 恢复区稳定保持时间 |
| `DOCK_ALIGN_T` | 1.0 s | 连续对准时间 |
| `DOCK_ALIGN_SPD` | 1.0 m/s | 对准平移速度上限 |
| `DOCK_IMG_P` | 0.3 1/s | 相机指向误差到 LOS 角度修正速度的比例增益 |
| `DOCK_IMG_VMAX` | 1.0 m/s | 图像跟踪平移速度上限 |
| `DOCK_IMG_FRAC` | 0.40 | 图像修正最多占总速度预算的比例 |
| `DOCK_LEGACY_APP` | 0 | 允许旧21字节帧执行无距离接近，默认关闭 |
| `DOCK_LEG_VMAX` | 5.0 m/s | 无距离兼容接近速度上限 |
| `DOCK_YAW_DB` | 1.0 deg | 水平图像误差死区 |
| `DOCK_YAW_RATE` | 10.0 deg/s | 航向设定最大变化率 |
| `DOCK_VEL_MAX` | 15.0 m/s | 高速接近速度 |
| `DOCK_ACC_MAX` | 3.0 m/s^2 | 速度指令增加率 |
| `DOCK_DEC_MAX` | 3.0 m/s^2 | 制动/恢复速度减少率 |
| `DOCK_STOP_DIST` | 2.0 m | 速度包络降为零的距离 |
| `DOCK_VIS_GRACE` | 300 ms | 短时视觉丢失宽限 |

相机与编码器参数的取值范围、标定命令、启用步骤、诊断字段及故障处理见 [MT6701 相机角度使用说明](DockGuidance_MT6701_相机角度使用说明.md)。

### 11.1 终端冲刺参数

| 参数 | 建议初值 | 说明 |
|---|---:|---|
| `DOCK_TERM_EN` | 1 | 0=保持普通制动逻辑，1=启用最大速度终端冲刺 |
| `DOCK_TERM_DIST` | 0 m | 距离进入阈值；0 表示禁用距离触发 |
| `DOCK_TERM_SIZE` | 0.10 | 目标框占画面比例达到该值后进入终端冲刺 |
| `DOCK_TERM_CFM` | 0.05 s | 终端进入条件的连续确认时间 |
| `DOCK_TERM_BLIND` | 0.50 s | 终端视觉丢失后保持最后有效速度向量的最长时间 |
| `DOCK_TERM_MAX_T` | 1.5 s | 终端阶段允许的总时长上限 |
| `DOCK_TERM_FILT` | 10.0 Hz | 末段 LOS 滤波截止频率；减小近距离高速跟踪滞后 |

对接完成/接触信号不是参数，必须来自真实硬件或上层任务状态。若没有该信号，`DOCK_TERM_MAX_T` 只能作为故障边界，不能被解释为对接成功。

## 12. 高速测试必要日志

```text
listener viztra_observation -n 5
listener sensor_encoder -n 5
listener dock_guidance_setpoint -n 5
listener trajectory_setpoint -n 5
listener vehicle_local_position -n 5
listener vehicle_local_position_setpoint -n 5
listener vehicle_attitude_setpoint -n 5
listener vehicle_status -n 5
```

启用编码器时，检查 ULog 的 `sensor_encoder`、`dock_guidance_setpoint.exposure_timestamp`、`sample_mount_pitch_deg` 和 `camera_time_aligned`；`camera_mount_pitch_deg` 是舵机命令角，可与实测角对照。其余高速测试所需 ULog 包含 `viztra_observation`、`trajectory_setpoint`、`vehicle_local_position`、`vehicle_local_position_setpoint`、`vehicle_attitude_setpoint` 和 `vehicle_status`。实现终端冲刺后还必须记录终端进入原因、终端计时器、最后有效 LOS、视觉丢失计时和对接完成输入。编译通过只证明软件接口正确，不代表高速飞行已经通过仿真或实飞验证。
