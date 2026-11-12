# Voliro T：以模式 0 PID 为基线的模式 1

2026-11-10。此说明取代早期模式 1 的角加速度 PID 调参说明。固件构建和主机模型测试
不等同于刷写、板上 HIL 或实机飞行验证。

## 控制契约

- `VT_CTRL_MODE=0` 保留原控制路径，仍为默认。
- `VT_CTRL_MODE=1`、`CA_AIRFRAME=17`：姿态增益采用已保存的
  `VC_ROLL_P / VC_PITCH_P / VC_YAW_P`，映射为 bank / independent pitch / heading 任务方向。
  在独立 pitch 绝对值不超过 45° 时保留模式 0 的增益强度；近竖直姿态见下文。
- 两种模式共用同一个 `VectorRateCore`，使用相同的 `VC_*RATE_P/I/D/K/FF`、
  `VC_RR/PR/YR_INT_LIM`、饱和方向门控与积分状态。惯量不缩放反馈与积分限幅；
  不再用模型残差幅值主动回算角速度积分。模式 0 的落地冻结、未解锁/显式复位语义保留。
- 标准 `vehicle_torque_setpoint` 与 `rate_ctrl_status` 保持归一化力矩单位。
- `VT_BANK_P / VT_PITCH_P / VT_HEAD_P / VT_RATE_*` 保留旧参数文件兼容，
  分组改为 `Voliro Legacy`，没有控制作用。不需要把旧模型测试增益手动搬到 VC 参数。

## 前馈与偏航

```text
omega_ff_body = R^T * omega_reference_ned
alpha_ff_body = R^T * alpha_reference_ned - omega_body cross omega_ff_body
u = Kp_VC*(rate_sp-rate) + integral_VC
    - Kd_VC*(measured_alpha-alpha_ff_body) + Kff_VC*rate_sp
    + (J / torque_scale)*alpha_ff_body
```

前馈需要参考与标准目标时间戳精确配对、100 ms 内新鲜、无未来时间、相同 estimator epoch，
并通过有限数值与四元数检查。参考失效时加速度前馈为零，PID 不换增益。
`J*alpha_ff` 只是参考加速度的附加项；D 项使用加速度误差，避免抵消计划中的加速度。

标准 `yaw_sp_move_rate` 在无有效完整角速度参考时始终保留，包括自稳、定高、固定倾转
和故障回退。完整参考有效时使用 NED 三轴角速度参考，避免重复叠加偏航。
`VEC_GYRO_FF` 在两个模式中均控制可选的 `omega cross J*omega`，默认关闭。

实机日志 `d5ceca7f-e82f-4d3f-8c4f-011aee2d734c` 的 +45.57 deg/s 请求、
仿真日志 `3705c5c1-6624-458c-955e-a12806863b53` 的 -150 deg/s 请求，
均作为“无完整参考时不能吞掉偏航速度”的回归场景。后者还受 80 deg/s 机体速率上限约束。

## 近竖直姿态的外环带宽

可逆尾桨在接近 ±90° 时处于零配平附近，换向确认、零驻留和电机滞后增加延迟。
将水平飞行的姿态增益直接搬到这一工况会导致闭环振荡。因此 `VT_ATT_SCALE` 只调度
**姿态外环及俯仰积分的累积速度**：在 45° 到 75° 之间平滑降低任务方向比例增益，在 75°..105° 区间
采用设定比例，接近倒置水平时恢复原增益。默认比例为 1/3，范围 0.1..1。

独立俯仰参考的运动速度也按相同比例降低，避免外环已经降速但参考仍高速穿越竖直。
该调度不改变已保存的角速度 PID 增益、积分状态值、积分可用范围；参考导数仍按实际受限运动生成，
前馈本身不再额外缩放；
近竖直时只有俯仰积分的增量乘以相同比例，正常角度下积分增量也保持原样。设为 1 可关闭调度，
但这不保证带尾桨保护和较大延迟的竖直保持稳定。该比例必须结合真实机构验证。

## 分配、诊断与参数

保留模式 1 的反扭矩模态滤波、尾桨保护预演、限位恢复与固定倾转/故障回退。
模式 0 固定机体 Z 轴滤波保持原行为。两种模式并不承诺所有受限分配命令逐样本相同。
`VT_POS_AW` 是位置积分设置，与已删除的角速度幅值回算分开。

新增 `vector_rate_control_status.pid_output / integral_output` 明确使用归一化力矩；
`torque_accel_feedforward` 使用 Nm。旧 `alpha_feedback / alpha_integral` 字段仅保留为
“归一化 PID/积分 × torque_scale / J”的等效诊断，不再表示角加速度 PID 状态。
这些命令和模型残差均不是实测物理力矩。

更改 `VT_CTRL_MODE` 需要重启。启动后检查三个 vector 模块的 status；角速度模块应另外输出：

```text
Rate feedback: mode-0 VC_* normalized-torque PID and per-axis integral limits
```

不自动覆盖已保存的 VC 增益、机架几何、惯量或执行器标定。惯量仍需用于前馈和倾转监测，
不能把“PID 不再依赖惯量”误解为惯量参数可以随意设置。

## 检查入口

```sh
python3 src/modules/vector_pos_control/tests/run_tests.py build/cuav_7-nano_volirot
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/cuav_7-nano_volirot
python3 Tools/tests/stm32_dshot/test_dshot.py
make cuav_7-nano_volirot -j8
```

核心回归覆盖模式 0 PID 数值等价、每轴积分能力与饱和方向门控、无参考时改变惯量不影响反馈、
加速度前馈与 D 补偿、偏航指令在水平/±90°/±160° 的投影及不重复前馈。
闭环测试使用同一个生产 PID，覆盖大角度保持/转换、执行器延迟、惯量变化、前馈丢失、
固定倾转交接与倾转故障。新增或失败工况必须保留证据，不用放宽通过门限代替修复。

## 本次验证记录（2026-11-10）

| 检查 | 结果 |
| --- | --- |
| 7 个核心/刚体回归程序 | 通过；包含 PID 等价、完整积分范围、两份日志的原始四元数偏航重放、±160°、50..150 ms 电机延迟、0.8/1.2 倍惯量、前馈丢失、倾转故障及固定倾转交接 |
| Voliro 分配器与尾桨保护、DShot | 通过 |
| SITL 六组启动检查 | 通过；MC、MorphQuad、Voliro 0/1 与监测开关组合 |
| 5.04 kg 历史 JSBSim 快照的大角度保持/转换 | 通过；固定使用 build/voliro_board_update_20261103/model，记录 XML SHA256 |
| 同快照固定倾转恢复 AUX 的原生 JSBSim 检查 | 未通过：39.8 s 实际 pitch 16.123°，目标 10°，超过原 1° 门限；刚体版本通过 |
| 当前 4.7 kg/原生尾桨 JSBSim 模型水平保持 | 未通过原门限：模式 0 峰值/末段误差 11.16°/3.414°，模式 1 为 11.10°/3.357°；新模型/执行器与测试假设的匹配仍待验证 |
| CUAV 7 Nano Voliro T 固件、SITL 构建 | 通过 |

扩展模型失败没有被放宽门限掩盖。当前结果证明代码基线与已列明的回归范围，
不能证明最新模型的所有工况通过，也没有进行飞控刷写、板上 HIL 或实机飞行验收。
