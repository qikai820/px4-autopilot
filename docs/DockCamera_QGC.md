# 在 QGC 显示相机舵机指令与编码器角度

本功能需要同时使用本仓库的 PX4 固件和配套修改后的 QGroundControl。MicoAir H743 Dock 固件运行 `dock_guidance` 后，每 200 ms 发布一次名为 `DockCam` 的 MAVLink `DEBUG_VECT`：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `x` | 相机舵机的机体相对俯仰指令角 | 度 |
| `y` | MT6701 经 `DOCK_CAM_ENC_90`（或旧版 `DOCK_CAM_ENC_Z`）和 `DOCK_CAM_ENC_REV` 校准后的机体相对俯仰测量角 | 度 |

指令角是 DockGuidance 最后一次发给舵机通道的目标角，不代表 PWM 已输出或舵机已到位。未启用变俯仰舵机、未解锁或尚未发布过指令时，`x` 为无效值。编码器未校准、失联、超时或校验失败时，`y` 为无效值。QGC 会把收到的无效值显示为占位符；`DockCam` 消息中断超过 3 秒后会保留最后读数，并将该数据组标记为遥测不可用。链路中断时，保留值不代表实时角度。

## 添加到飞行界面

1. 连接飞控，确认 `dock_guidance` 和 `mt6701` 已启动，按实际安装完成编码器零位和方向校准。
2. 在 QGC 飞行界面底部的数值栏上右击（触屏长按），解锁编辑；用 `+` 增加位置。
3. 点击待修改的数值，选择 **Group → DockCamera**，再分别选择 **Value → commandedPitch** 和 **measuredPitch**。界面标签可以自定义。
4. 点击锁图标保存。

排查时，先在 PX4 MAVLink 控制台运行 `listener debug_vect`，确认名称为 `DockCam`、`x/y` 有效；再到 QGC 的 MAVLink Inspector 查看 `DEBUG_VECT` 是否到达。普通遥测链路默认以 5 Hz 发送 `DEBUG_VECT`；若使用自定义 MAVLink 链路模式，可在飞控控制台执行 `mavlink stream -d <端口> -s DEBUG_VECT -r 5`。
