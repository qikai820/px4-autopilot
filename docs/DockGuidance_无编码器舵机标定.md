# DockGuidance 无编码器相机舵机标定

本文适用于相机相对机体俯仰角由舵机命令估计、`DOCK_CAM_ENC_EN=0` 的情况。标定角量的是相机**光轴相对机体**的角度。当前 MicoAir H743 Dock 的 ES08MA 为 1:1 传动，实测 **90° 对应 1500 μs、30° 对应 2100 μs**；向 30° 转时 PWM 增大，物理可动范围为 30°～90°。该板型的 Dock 启动脚本不自动启动 MT6701 驱动。

## 映射与参数

`DOCK_CAM_P_MIN/MAX` 限制相机命令角；`DOCK_CAM_OUT_LO/HI` 分别指定低/高角端的归一化 `Gimbal Pitch` 输出。默认 `-1/+1` 保留旧固件行为。无编码器时，DockGuidance 把命令角作为曝光时刻的相机姿态，因此 PWM 到真实光轴角的对应关系必须实测。MicoAir 板级初始映射为：

| 光轴角 | 归一化输出 | PWM5 输出 |
|---:|---:|---:|
| 90° | 0 | 1500 μs |
| 60°（线性插值） | +0.5 | 1800 μs |
| 30°（实测端点、软件下限） | +1 | 2100 μs |

以 90° 为基准、在两实测端点之间采用线性插值：`PWM = 1500 + 10 × (90° - 角度)`，仅用于 30°～90° 命令区间。60°/1800 μs 是插值结果；1:1 传动不等于舵机在中间位置绝对线性，需要实测中间位置才能确认角度精度。不可把外推得到的 0°/2400 μs 当成可用行程。

MicoAir H743 Dock 的 PWM5 是 `PWM_MAIN` 第 5 路，功能应为 `Gimbal Pitch (421)`。`PWM_MAIN_CENT5` 决定归一化 0 的 PWM，`PWM_MAIN_MAX5` 决定 +1 的 PWM；未解锁和失效输出 `PWM_MAIN_DIS5/FAIL5` 也设为 1500 μs，保持 90° 基准。第 5 路不能反向，否则 +1 会用 `PWM_MAIN_MIN5`；检查 `PWM_MAIN_REV` 的第 4 位（数值 16）为 0，保留其他通道现有设置。板级脚本已改为配置 `PWM_MAIN_FUNC5=421`，不再使用原来错误的 `PWM_AUX_FUNC1`。

## DockGuidance 舵机方向测试指令

新固件增加 `dock_guidance servo_test <PWM 微秒>`，只针对 MicoAir H743 的 `PWM_MAIN5=Gimbal Pitch (421)`。它经 Commander 的执行器测试通路请求该功能输出 1 秒，随后自动回到已检查的 1500 μs 未解锁值；不需要解锁，也不进入 Dock 模式。指令只接受 1500～2050 μs，并检查飞控处于未解锁状态、PWM5 是唯一的 Gimbal Pitch 输出、PWM5 的禁用值为 1500 μs、失效值为 `-1` 或 1500 μs、`COM_PREARM_MODE=0`，以及请求值处于配置的 PWM 上下限内。预解锁控制可能在测试超时后恢复云台命令，因此本指令拒绝非零预解锁模式。Commander 仍会检查安全开关和 `COM_MOT_TEST_EN`。执行器测试确认的是 PWM 通道与机械方向，不证明 DockGuidance 的视觉控制方向或相机角标定正确。

当前 `h7.params` 中 `PWM_MAIN_DIS5=2100`，指令会拒绝测试。先**拆下全部螺旋桨**，在舵机断电或摇臂脱开时把 `PWM_MAIN_DIS5` 和 `PWM_MAIN_FAIL5` 设为已实测的安全 1500 μs，保存并重启，确认接通舵机电源时不会顶住机构，再测试：

```sh
param set PWM_MAIN_DIS5 1500
param set PWM_MAIN_FAIL5 1500
param save
# 重启后，确认未解锁的 PWM5/相机位于安全 1500 μs 基准
dock_guidance servo_test 1500
dock_guidance servo_test 1600
```

若要直接输入相机**相对机体的俯仰角**，先确认 `DOCK_CAM_SERVO=1`，并用 `param show` 核对 `DOCK_CAM_P_MIN/MAX`、`DOCK_CAM_OUT_LO/HI`、`PWM_MAIN_CENT5/MAX5` 已按实测结果标定。然后保持未解锁运行：

```sh
dock_guidance servo_test_angle 90
dock_guidance servo_test_angle 80
```

`servo_test_angle` 使用 DockGuidance 正常运行时同一套角度到归一化 `Gimbal Pitch` 的映射，再按 PWM5 中位、上下限和反向位估算脉宽；接受 Commander 的 1 秒测试后会报告“请求角度 → 预计 PWM”。对于上述 90°/1500 μs、30°/2100 μs 的初始标定，80° 预计为 1600 μs。指令不会把超出 `DOCK_CAM_P_MIN/MAX` 的角度悄悄夹到端点，也会拒绝方向相反、高角端不在 1500 μs 或换算结果不在 1500～2050 μs 的配置；因此初始标定下的 30°/2100 μs 会被拒绝。报告的角度是**开环目标值**，不是舵机实测角度。

记录两次输出时的**相机光轴相对机体俯仰角**。若 1500 μs 约为 90°、1600 μs 使角度减小，说明 PWM 增大使镜头下俯；这与旧固件的 `DOCK_CAM_P_MIN/MAX → -1/+1` 映射相反。不要只改 `PWM_MAIN_REV` 后直接试完整行程：旧保存值 `PWM_MAIN_MAX5=2200` 仍可能越过约 30°/2100 μs 的机械端点。1600 μs 的方向确认之后，再依照实测数据设置新固件的 `DOCK_CAM_OUT_LO/HI` 和安全行程。

实机若已把 PWM5 的最小值、中位、最大值设为 1500/1500/2100 μs，当前 DockGuidance 只发 0～+1，实际插值使用的是中位 1500 到最大 2100，最小值不参与这段输出。当前 PX4 参数元数据给 `PWM_MAIN_MIN5` 的上限是 1400；因此板级脚本不写入超出元数据范围的 1500 默认值。刷机重启后用 `param show PWM_MAIN_MIN5` 核对实机保存值，角度行程仍以 `DOCK_CAM_P_MIN/MAX` 限定。

在 QGC 执行器页面先确认 PWM5 没有分配给电机，并检查 PWM6；PWM5 和 PWM6 共用 `PWM_MAIN_TIM1` 定时器。普通 PWM 舵机通常用 50 Hz，但应按实际舵机规格设置，不能把同组电机的输出频率一起改成 50 Hz。拆下螺旋桨、保持未解锁后，在执行器测试中先给 1500 μs 确认 **90° 基准**，再缓慢增加 PWM，核对 1800、2000、2100 μs 的真实光轴角，遇到机械阻挡立即停止。虽然 30°/2100 μs 是准确测点，仍需确认端点没有持续顶住机械止挡；如有负载，应将软件命令端点收进安全区间。

确认 PWM5 只接舵机、PWM6 的定时器协议兼容，以及第 5 路未反向后，可在新的固件上以以下参数作为**地面初始标定**。板级脚本也给出这些新默认值；已有自定义保存值仍需用 `param show` 核对：

```sh
param set PWM_MAIN_FUNC5 421
param set PWM_MAIN_DIS5 1500
param set PWM_MAIN_FAIL5 1500
param set PWM_MAIN_CENT5 1500
param set PWM_MAIN_MAX5 2100
param set DOCK_CAM_SERVO 1
param set DOCK_CAM_ENC_EN 0
param set DOCK_CAM_P_MIN 30
param set DOCK_CAM_P_MAX 90
param set DOCK_CAM_OUT_LO 1
param set DOCK_CAM_OUT_HI 0
param set DOCK_CAM_PITCH 90
param save
```

`PWM_MAIN_TIM1` 如需改成舵机支持的频率，必须先确认同组 PWM6 的用途，修改后重启。PWM5 功能修改后也建议重启并重新核对。软件下限为实测 30°/2100 μs；若需要给机械止挡留余量，可提高 `DOCK_CAM_P_MIN` 并按 `(该角度实测 PWM - 1500) / (PWM_MAIN_MAX5 - 1500)` 更新 `DOCK_CAM_OUT_LO`，保持 90°/1500 μs 的基准不变。

只有在固件包含 `DOCK_CAM_OUT_LO/HI` 参数并完成实体舵机测试后，以上设置才有效。软件映射测试和编译无法证明 PWM5 的实际波形、转向、机械余量或无编码器时的角度精度。
