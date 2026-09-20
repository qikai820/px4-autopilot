# Dock Guidance 控制律与软件设计

## 1. 设计目标

Dock Guidance 同时支持固定安装相机和单轴俯仰舵机相机。正常高速阶段以 `DOCK_VEL_MAX` 为三维速度预算，目标居中时速度方向沿目标 LOS。固定相机用垂直平移修正图像误差；俯仰舵机相机用慢速机身相对角度调整保持构图，舵机不承担姿态增稳。

本版本不再用接近速度误差直接增加总油门，也不再强制固定前倾角。Dock Guidance 生成同时包含接近分量和图像跟踪分量的运动学速度设定值，由 PX4 `mc_pos_control` 完成速度、高度、姿态和推力耦合。

## 2. 控制链

```text
viztra_observation
    -> DockGuidance
    -> trajectory_setpoint
    -> mc_pos_control
    -> mc_att_control
    -> mc_rate_control
    -> control_allocator
```

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

### 3.2 输出

| 主题 | 用途 |
|---|---|
| `trajectory_setpoint` | NED 三维速度和偏航设定值 |
| `dock_guidance_setpoint` | 阶段、速度向量、距离、FOV 余量和有效位 |
| `config_control_setpoints` | 声明 Dock 模式使用 PX4 位置/姿态/角速度控制链 |

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

`relative_n_m/e_m/d_m` 用于计算斜距：

\[
d=\sqrt{r_N^2+r_E^2+r_D^2}
\]

有相对 NED 位置时，高速接近同时具备 LOS、距离和本地速度，可执行完整末端制动。兼容旧 21 字节 Viztra 帧时，`DOCK_LEGACY_APP=1` 允许仅依赖 LOS 进入高速接近；但由于没有距离，不具备自动末端制动能力。

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

舵机只使用相机坐标系中的持续俯仰误差：误差先经过 `DOCK_CAM_P_DB` 死区，再除以 `DOCK_CAM_P_TC` 得到慢速角度变化率，并受 `DOCK_CAM_P_RATE` 限制。舵机指令不使用机体姿态或角速度，因此不是增稳云台。当前舵机指令角同时更新相机到机体的安装矩阵，用于下一帧 LOS 转换。

将一个未使用的 PWM 输出功能设置为 `Gimbal Pitch (421)`。`DOCK_CAM_P_MIN/MAX` 分别映射到归一化输出 -1/+1；方向相反时使用该 PWM 通道的反向参数。该直接输出方式不需要启动 PX4 gimbal 模块，建议保持 `MNT_MODE_IN=-1`，避免多个发布者同时控制同一个输出。

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

没有相对位置时，即使显式启用 `DOCK_LEGACY_APP`，速度仍受 `DOCK_LEG_VMAX` 限制。该模式没有距离制动，只用于受控兼容测试，不作为默认高速方案。

当 FOV 从 `DOCK_FOV_ALIGN` 接近 `DOCK_FOV_SOFT` 时，允许速度按剩余视场裕量连续降到零，避免在加速和恢复状态之间形成二值振荡。

为了避免 `mc_pos_control` 在各轴分别限幅后改变速度方向，Dock Guidance 在发布前根据以下限制等比缩放整个速度向量：

- `MPC_XY_VEL_MAX`
- `MPC_Z_VEL_MAX_UP`
- `MPC_Z_VEL_MAX_DN`

因此“最大速度”表示 `DOCK_VEL_MAX` 与 PX4 当前各轴限制共同允许的最大 LOS 速度。若需要水平达到 `15 m/s`，`MPC_XY_VEL_MAX` 也必须不小于 `15 m/s`。

## 8. 末端制动

允许速度由剩余距离决定：

\[
v_{brake}=\sqrt{2a_{dec}\max(d-d_{stop},0)}
\]

\[
v_{desired}=\min(v_{max},v_{brake})
\]

其中 `a_dec=DOCK_DEC_MAX`，`d_stop=DOCK_STOP_DIST`。当 `v_brake < v_max` 时进入 `TERMINAL_BRAKE`。

## 9. 视觉丢失和恢复

- 目标超过软边界：进入 `RECOVERY`，按 `DOCK_DEC_MAX` 降低速度指令。
- 目标连续超过硬边界 `DOCK_FOV_H_T`：进入 `EMERGENCY_RECOVERY`，用两倍恢复减速度停止接近，但继续图像/舵机跟踪并保持 Dock 健康。
- `RECOVERY` 只有在速度接近零、目标回到对准区并连续稳定 `DOCK_REC_T` 后才能返回 `ALIGN`。
- 短时断流：保持模式 valid，但速度指令立即开始减小。
- 断流超过 `max(DOCK_OBS_TOUT, DOCK_VIS_GRACE)`：设定值 invalid，发布零速度并锁存故障；驾驶员必须先切离 Dock 才能清除锁存，防止 Dock/RTL 自动循环。
- 相对位置超过 `DOCK_REL_TOUT`：`DOCK_LEGACY_APP=0` 时减速回到 `ALIGN`；开启旧帧接近时继续角度跟踪，但不使用距离制动。
- 本地速度无效：立即 invalid 并锁存。目标进入硬视场边界本身不再触发 RTL。

## 10. 航向抑振

水平图像误差小于 `DOCK_YAW_DB` 时保持航向。LOS 水平分量在 0.25 到 0.60 之间时连续降低航向修正权重，小于 0.25 时保持航向，避免高仰角/俯角下地理系方位投影放大。最终航向设定由 `DOCK_YAW_RATE` 限制变化率；`RECOVERY`、`EMERGENCY_RECOVERY` 和短时视觉降级期间保持最后有效航向。

## 11. 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `DOCK_CAM_DLY` | 0 ms | 相机延迟补偿 |
| `DOCK_CAM_SERVO` | 0 | 0=固定镜头，1=单轴俯仰舵机镜头 |
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

## 12. 高速测试必要日志

```text
listener viztra_observation -n 5
listener dock_guidance_setpoint -n 5
listener trajectory_setpoint -n 5
listener vehicle_local_position -n 5
listener vehicle_local_position_setpoint -n 5
listener vehicle_attitude_setpoint -n 5
listener vehicle_status -n 5
```

ULog 必须包含 `viztra_observation`、`dock_guidance_setpoint`、`trajectory_setpoint`、`vehicle_local_position`、`vehicle_local_position_setpoint`、`vehicle_attitude_setpoint` 和 `vehicle_status`。编译通过只证明软件接口正确，不代表 15 m/s 高速实飞已验证。
