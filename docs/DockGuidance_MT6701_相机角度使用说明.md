# DockGuidance MT6701 相机俯仰角使用说明

Dock 模式的完整配置与操作流程见 [DockGuidance 用户使用说明](DockGuidance_用户使用说明.md)。

## 1. 适用范围与数据流

本方案适用于 MT6701 与相机俯仰轴 **1:1 传动**、相机实际活动范围约 **0°～90°** 的单轴机构。MT6701 每圈给出 0～16383 的绝对计数；这里不累计圈数，也不以舵机命令推算相机实际角度。相机相对机体的俯仰零度由一次人工摆位标定；正俯仰表示相机朝机体上方抬起。90° 只用于核对方向与传动比，不设置第二个标定点。

```text
MT6701 I²C -> sensor_encoder.raw_angle + timestamp_sample
            -> 零位/方向换算 -> 实测角历史
Viztra timestamp_sample - DOCK_CAM_DLY -> 曝光时刻
            -> 查询曝光时刻的相机角、机体姿态及位置
            -> 相机 LOS -> 机体系 -> NED -> DockGuidance 速度计算

视觉俯仰误差 -> 原有舵机命令 -> gimbal_controls -> Gimbal Pitch (421)
```

`DOCK_CAM_ENC_EN=1` 时，相机到机体的欧拉角使用 `DOCK_CAM_ROLL`、曝光时刻**实测俯仰角**、`DOCK_CAM_YAW`。`DOCK_CAM_PITCH` 不叠加到实测俯仰角上。舵机仍按原视觉误差、死区、时间常数和速率限制生成命令；编码器不参与舵机闭环控制。`DOCK_CAM_ENC_EN=0` 时继续用曝光时刻的舵机命令历史，固定相机则使用配置的安装俯仰角。

驱动以 100 Hz 读取 MT6701 的 14 位角并发布 `sensor_encoder`。DockGuidance 只消费 `raw_angle`、采样时间和设备 ID；话题中的 `angle` 弧度值不参与零位换算。单台 MT6701、单个话题实例是当前接入方式。

## 2. 参数详解

下表默认值和范围对应当前 `src/modules/dock_guidance/module.yaml`。角度均为**相机相对机体**的度数，计数为 MT6701 的 14 位原始值。

| 参数 | 默认值；允许范围 | 作用与设置方法 |
|---|---|---|
| `DOCK_CAM_ENC_EN` | `0`；`0/1` | `0` 使用命令角历史；`1` 使用 MT6701 实测角历史计算相机到机体旋转。零位未标定、传感器不新鲜或角度无效时，不允许进入/继续 Dock 引导。**不改变舵机命令。** |
| `DOCK_CAM_ENC_Z` | `-1`；`-1～16383` count | 相机摆在机体相对俯仰零度时的原始计数；`-1` 为未标定。用 `dock_guidance calib_zero` 采集并保存，不需手算，也不会写 MT6701 EEPROM。重装磁铁、传感器或改变机械零位后重新标定。 |
| `DOCK_CAM_ENC_REV` | `0`；`0/1` | `0` 为计数增大对应相机正俯仰；如果实际正俯仰时标定角变负，设为 `1`。只反转坐标计算中的**测量角**，不反转舵机 PWM。 |
| `DOCK_CAM_SERVO` | `0`；`0/1` | `1` 启用视觉误差驱动的单轴舵机；`0` 为固定相机路径。与 `DOCK_CAM_ENC_EN` 是两个独立开关。 |
| `DOCK_CAM_ROLL` | `-90`；`-180～180` deg | 相机到机体安装旋转的 roll 分量；由实际相机安装方向确定，不是编码器零位。 |
| `DOCK_CAM_PITCH` | `0`；`-180～180` deg | 关闭编码器时的固定安装俯仰角，或启用舵机时的初始命令角；启用编码器后**不与实测角相加**。 |
| `DOCK_CAM_YAW` | `0`；`-180～180` deg | 相机到机体安装旋转的 yaw 分量；由安装方向确定。 |
| `DOCK_CAM_P_MIN` | `0`；`-180～180` deg | 舵机指令的一个行程端点；与 `DOCK_CAM_P_MAX` 中较小的值映射到归一化输出 `-1`，也是编码器有效角下界，允许 5° 端点余量。 |
| `DOCK_CAM_P_MAX` | `90`；`-180～180` deg | 舵机指令的另一行程端点；两个端点中较大的值映射到归一化输出 `+1`，也是编码器有效角上界，允许 5° 端点余量。舵机使用时两端点跨度需至少 1°。默认 0°～90° 对应本机构行程。 |
| `DOCK_CAM_P_RATE` | `5`；`0.1～30` deg/s | 舵机命令角的最大变化率；不是编码器测量滤波参数。 |
| `DOCK_CAM_P_TC` | `2`；`0.2～20` s | 视觉俯仰误差经死区后换算为舵机命令角速度的时间常数；越大，命令变化越慢。 |
| `DOCK_CAM_P_DB` | `0.5`；`0～10` deg | 视觉俯仰误差死区，小于该角度不推动舵机命令。 |
| `DOCK_CAM_DLY` | `0`；`0～5000` ms | 从图像曝光到 Viztra `timestamp_sample` 接收的延迟；只减一次，用于对齐机体姿态、位置和相机角。**不是**舵机机械延迟，不能拿来补偿舵机滞后。 |

计数换算采用最短有符号单圈差值：

```text
delta = wrap_to_[-8192,8191](raw_angle - DOCK_CAM_ENC_Z)
pitch_deg = (DOCK_CAM_ENC_REV ? -1 : +1) × delta × 360 / 16384
```

一个计数对应约 0.02197° 的量化步距；这只是数字分辨率，不代表磁铁安装或机构的角度精度。若相机从 0° 转到 90°，原始计数差的绝对值应约为 4096（须按 16384 回绕处理）。如果差值明显不符，先检查 1:1 传动和安装；当前算法没有用第二点拟合比例。

## 3. 在 CUAV 7-Nano test 上启动

`cuav_7-nano_test` 已编入 MT6701 驱动和 DockGuidance。板级 I²C 外部总线为 1、2，内部总线为 3、4；命令里的总线号应按实际接线选择。驱动需要启动，启用反馈参数本身不会启动驱动。以下以外部 I²C 1、默认地址 `0x06` 为例：

```sh
mt6701 start -X -b 1
mt6701 info
listener sensor_encoder -n 5
dock_guidance status
```

若传感器地址已配置为 `0x46`，在启动命令中加 `-a 0x46`；内部总线改用 `-I` 和实际总线号。`mt6701 info` 展示寄存器原始字节、计数、角度、最近样本年龄及 I²C 错误计数。若 `dock_guidance status` 提示模块未运行，先执行 `dock_guidance start`。检查接线、驱动计数随手动转动平稳变化，再标定。

单轴舵机配置沿用原控制路径：将所接 PWM 输出设为 `Gimbal Pitch (421)`，`DOCK_CAM_SERVO=1`，并检查舵机行程和 PWM 通道方向。该方向设置只决定舵机如何转；与 `DOCK_CAM_ENC_REV` 分开核对。若此输出由 DockGuidance 直接发布，保持 `MNT_MODE_IN=-1`，避免其他 gimbal 发布者同时占用。

## 4. 零位标定与启用

1. **保持未解锁。** 将相机实际光轴放在定义好的机体相对俯仰 **0°** 位置，并保持静止。先用 `mt6701 info` / `listener sensor_encoder` 确认有连续的新数据。
2. 执行 `dock_guidance calib_zero`。命令校验未解锁状态，在约 0.9 秒的尝试窗口内收集 16 个新鲜样本；若样本间相对第一帧偏移超过 20 count、数据超时或解锁则报错。成功后自动保存 `DOCK_CAM_ENC_Z` 到 PX4 参数存储，不写传感器 EEPROM。
3. 用 `param show DOCK_CAM_ENC_Z` 和 `dock_guidance status` 核对零位。手动使相机**正向抬起**一个已知角度，例如约 10°～20°。若状态中的 `measured` 变负，执行 `param set DOCK_CAM_ENC_REV 1`；反之保持 `0`。修改参数后等待新样本，再检查 `measured`。可在约 90° 位置核对计数变化约 4096，**不需要标定 90°**。
4. 核对 `DOCK_CAM_ROLL/YAW` 的安装方向、`DOCK_CAM_P_MIN/MAX` 的真实行程，以及相机坐标轴定义。确认 `dock_guidance status` 显示 `ready=1`、`fault=0`、`age` 持续较小，才执行 `param set DOCK_CAM_ENC_EN 1`，再执行 `param save`。启用或修改几何参数会清空相关历史，需等待新编码器和视觉帧建立时间对齐。
5. 在有有效 Viztra 图像帧的地面验证中，观察 `listener dock_guidance_setpoint -n 5` 中的 `sample_mount_pitch_deg`、`camera_time_aligned` 和 `exposure_timestamp`。启用舵机时 `camera_mount_pitch_deg` 是舵机**命令角**，它与曝光时刻的实测角可以有滞后差；目标 LOS 计算使用 `sample_mount_pitch_deg`。检查 `dock_guidance status` 中 `exposure pitch` 与相机实物姿态一致，再进入下一步系统验证。

可复制的标定与启用命令（每步均在确认前一步结果后执行）：

```sh
dock_guidance calib_zero
param show DOCK_CAM_ENC_Z
dock_guidance status
# 手动抬起相机并检查 measured 的符号；需要时再执行下一行
param set DOCK_CAM_ENC_REV 1
param set DOCK_CAM_ENC_EN 1
param save
listener dock_guidance_setpoint -n 5
```

上面 `DOCK_CAM_ENC_REV 1` 是**条件命令**，正向抬起时 `measured` 已为正就不要执行。建议在未进入 Dock 模式时修改零位、方向、相机安装角或延迟；这些参数变化会重置观测历史，运行中修改可能中断引导。

## 5. 时间对齐、故障与诊断

接收样本须来自 MT6701 设备类型、同一设备 ID，原始计数在 0～16383 且时间戳有效；采样年龄最多 50 ms。标定角须处在 `DOCK_CAM_P_MIN/MAX` 所定义范围外加两端各 5° 的容差。相邻不足 100 ms 的样本若跳变超过 `5° + 90°/s × 采样间隔` 则拒绝。曝光时刻落在两个有效样本之间时，只在样本间隔不超过 40 ms 时插值；比最新样本晚时最多保持 20 ms。超出这些时序范围的图像不使用失真的相机姿态。

启用编码器后，未标定、传感器不新鲜或测量异常会阻止 Dock 模式健康检查通过；已在 Dock 中发生故障则使引导设定无效并走安全速度/退出路径。系统**不会悄悄退回舵机命令角**。编码器故障处理不把编码器反馈送入舵机控制器，原有周期舵机命令发布仍保留。

| 检查命令/字段 | 含义 |
|---|---|
| `mt6701 info` | 当前寄存器读取、计数、角度、最近发布样本年龄和 I²C 错误计数 |
| `listener sensor_encoder -n 5` | `raw_angle`、`timestamp_sample`、`device_id` 是否稳定、新鲜 |
| `dock_guidance status` | `zero`、`reverse`、`raw`、`measured`、`age`、`ready`、`fault`、`exposure pitch` |
| `listener dock_guidance_setpoint -n 5` | `camera_mount_pitch_deg`=命令角；`sample_mount_pitch_deg`=该帧曝光时刻采用的角；`camera_time_aligned` 和 `exposure_timestamp` 用于核对时序 |
| ULog `sensor_encoder` / `dock_guidance_setpoint` | 对照原始角、命令角、曝光时刻实测角和引导有效性；`sensor_encoder` 已配置为可选记录话题 |

需要回退到原先的命令角坐标变换时，在未进入 Dock 模式时执行：

```sh
param set DOCK_CAM_ENC_EN 0
param save
```

当前软件编译和历史对齐测试不能证明实际安装精度；零位方向、总线时序、舵机行程及相机画面坐标应在实物和地面记录中逐项核对，再开展飞行验证。
