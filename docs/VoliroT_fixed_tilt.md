# Voliro T 遥控固定舵机模式

适用于 `CA_AIRFRAME=17`，支持 `VT_CTRL_MODE=0/1`。`RC_MAP_VECTOR_SW=0`
时保持原来的行为；映射开关后，打开选择矢量模式，关闭选择固定舵机的普通多旋翼位速控制。

## 拨杆与配置

例如把两段开关放在 CH9：

```text
param set RC_MAP_VECTOR_SW 9
param set VEC_AUX_ATT 1
param set RC_MAP_AUX1 6
param set VT_PITCH_MAX 10
param set VT_PITCH_RATE 20
```

示例中 CH9 为模式开关，CH6 为 AUX1 旋钮/滑杆，AUX1 中点对应 0°、两端对应 ±10°。
通道号应按实际遥控器选择，模式开关和 AUX1 应使用各自独立的通道，不能与前后摇杆共用。代码判断的是经过 RC 校准与反向设置后的
归一化通道值；在 QGC 通道监视器确认哪一档高于中点。

| VECTOR 开关 | LEVEL 开关 | 行为 |
| --- | --- | --- |
| 打开：高于中点 | 关闭/未映射 | 矢量模式，由 AUX1 控制独立俯仰，舵机参与推力补偿 |
| 打开：高于中点 | 高于中点 | 矢量模式，独立俯仰目标回零，舵机继续参与位置控制 |
| 关闭：低于或等于中点 | 任意 | 先用矢量控制回平，确认恢复后进入普通多旋翼模式并固定舵机，前后移动通过机体俯仰产生 |

在 Position 或 Position Slow 中，关闭 VECTOR 后正常推/拉前后摇杆：摇杆给出轨迹/速度指令，
位置控制器自动计算俯仰，松杆后减速并保持位置。Hold 中仍按原 Hold 轨迹保持位置。
该模式也应用于当前已启动的多旋翼位置/高度控制路径；不改变导航模式，不改变摇杆的轨迹映射。
`VEC_AUX_ATT` 不作为该模式的启用条件。固定模式期间 AUX1 不参与独立俯仰；
VECTOR 打开时，`VEC_AUX_ATT=1` 和 `RC_MAP_AUX1` 启用 AUX1 独立俯仰，
不需要反复修改参数。`RC_MAP_VECTOR_SW=0` 时也保留该独立俯仰功能。
AUX1 独立俯仰仅在 Position、Position Slow 和 Hold 中、已解锁离地且无 failsafe 时生效；
矢量模式中 LEVEL 优先将独立俯仰回零，释放后恢复当前 AUX1 目标。
普通多旋翼模式中 LEVEL 不改变模式，也不会让舵机重新参与控制。

## 控制与切换

- 固定模式复用普通多旋翼的位速控制、推力/倾角限制和自身抗积分饱和，保留 Voliro T 五电机构型。
  水平推力来自机体姿态；固定倾角分配不会通过舵机补偿前后力或俯仰力矩。
- 从非零倾转切入时先保持矢量力分配、位速控制及动态参考前馈，独立俯仰目标归零，按
  `min(VT_PITCH_RATE, CA_CT_RATE)` 回平，动态模式同时遵守俯仰加速度与跟踪能力约束。
  机体与姿态目标的独立俯仰、倾角观测/预测及最终倾角指令均在 ±5° 内，且机体角速度
  不超过 10°/s，连续满足 0.2 s 后，才切换固定倾角分配并把剩余小倾角回中。
  姿态/角速度或执行器消息过时会阻止空中交接；中途重新打开 VECTOR 会取消等待。
  有效倾角观测或执行器预测/最终指令用于补偿剩余倾角；这些都不等于真实舵机位置传感器。
- 进入和退出时清除水平速度积分及独立姿态参考，姿态目标沿四元数最短路径限速，使用 `VT_BANK_RATE`。
  固定模式的俯仰幅度由普通位置控制的倾角限制决定，不使用大角度 AUX 的 `VT_PITCH_MAX`。
- 有效 RC、已解锁且位置/高度控制路径运行时才选择固定模式。RC 超过 500 ms 未更新、通道无效、
  failsafe、退出位置控制或解除武装后释放手动选择。分配器只接受本次解锁周期内、100 ms 内的模式状态。
- 舵机故障监测独立运行，故障回退优先且仍锁存至解除武装；拨回 VECTOR 不能清除故障。
  尾电机换向保护、执行器限制和最终输出分配残差反馈继续有效。

`vector_pos_control status` 显示 `RC fixed tilt: enabled/disabled` 和 `RC return to level: pending/inactive`。
`voliro_tilt_status.fixed_tilt_pending` 记录回平等待阶段，`fixed_tilt` 只在交接完成后为真，
`fallback` 仍专指锁存故障回退。四个动态参考/执行器状态日志改用延迟订阅，避免 logger
启动时尚未发布而整段漏记；自定义 `logger_topics.txt` 仍需自行包含这些主题。

2026-11-05 的 HIL 日志 `c8c87619-52b0-496c-a949-d4540e986354` 暴露了直接回中问题：
137.262 s 时从约 −73° 切出，舵机按 200°/s 在约 0.4 s 内回中，此时机体仍约 −58°，
138.000 s 下落速度已达约 2.96 m/s。倾转故障回退始终未触发；138.622 s 才进入终止。
新增 `test_voliro_fixed_tilt_transition.cpp` 比较直接回中与分阶段切换，覆盖 ±73°、±120°、
200°/s 舵机、30°/s 俯仰回平、执行器滞后、交接确认和取消。该测试为无触地刚体闭环，
不能代替板上 HIL 或飞行验证。下表保留的是此前小角度直接切换实现的历史结果。

## 仿真验证（2026-11-04）

网络隔离的原生 JSBSim 使用本机
`jsbsim_realtime/data/aviation/aircraft/voliro_t` 模型的临时副本，保留实际 FCS 推力曲线和刚体动力学。
测试直接运行真实位置、姿态、角速度和分配算法，分别以 100 Hz、400 Hz、400 Hz 闭环。
它不经过接收机、EKF 或飞控 uORB 调度，因此不是整机 SITL/HIL 或飞行验证。

90 秒场景包括：VECTOR 打开时用 AUX1 给出 +10° 俯仰、关闭 VECTOR 进入普通多旋翼模式、前进到 +2 m、
后退到 -2 m、返回悬停、重新打开 VECTOR 后自动恢复当前 AUX1 目标和矢量动作、在非零舵机角度下
再次关闭 VECTOR 并回中，以及最后退出普通多旋翼模式并返回原点。全程不改变参数。

| 检查 | 刚体模型 | 原生 JSBSim |
| --- | ---: | ---: |
| +2 m 目标误差（t=14 s） | 0.145 m | 0.152 m |
| -2 m 目标误差（t=26 s） | 0.137 m | 0.137 m |
| 回到原点悬停误差（t=37 s） | 0.016 m | 0.022 m |
| 最终位置误差（t=90 s） | 0.001 m | 0.003 m |
| 回中后固定区间最大舵机指令/实际角 | 0° / <0.01° | 0° / 0° |
| 退出固定模式后最大舵机动作 | 31.50° | 21.10° |
| 再次切入时舵机角度 | 26.39° | 18.80° |
| 固定模式机体俯仰范围 | -13.96° 至 +17.54° | -13.99° 至 +20.88° |
| 相邻 100 Hz 姿态目标最大变化 | 0.8° | 0.8° |
| 首次切入前 / 退出后 AUX1 俯仰（目标 +10°） | 9.989° / 9.983° | 9.813° / 9.903° |

分配器包装测试还直接运行 `ActuatorEffectivenessVoliroT.cpp`，覆盖两种控制模式、回中限速、
前后推力/俯仰力矩变化下保持零舵机、退出普通模式恢复矢量分配、过时/未来模式消息和故障锁存优先级。

复现命令：

```sh
python3 src/modules/vector_pos_control/tests/run_tests.py build/px4_sitl_vector
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/px4_sitl_vector
python3 src/modules/vector_pos_control/tests/run_tests.py build/px4_sitl_vector --fixed-only \
  --jsbsim-model /home/qikai820/src/jsbsim_realtime/data/aviation/aircraft/voliro_t \
  --jsbsim-prefix /home/qikai820/src/px4_jsbsim/build/jsbsim_install \
  --jsbsim-systems /home/qikai820/src/jsbsim_realtime/data/aviation/systems
```
