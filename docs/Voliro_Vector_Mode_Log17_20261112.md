# Voliro Vector 无法进入：log17 诊断与修复

输入日志：`log_17_2026-11-12-19-15-52.ulg`，CUAV 7 Nano，`SYS_HITL=3`，持续约 292.18 s；以下时间从 ULog 起点计算。这是 HIL 证据，不是实机飞行验证。

## 日志事实

- `CA_AIRFRAME=17`，`VT_CTRL_MODE=1`，`VEC_AUX_ATT=1`，`RC_MAP_AUX1=6`，`RC_MAP_VECTOR_SW=8`；`VEC_REF_EN=0` 对 Voliro 的模式选择无影响。
- `VT_TILT_EN=1`，误差门限 10 deg，确认时间 0.5 s；全程解锁，未出现 Commander failsafe。
- 31.696 s：`Voliro: Normal mode active (servo fixed)`。
- 40.106 s：`Tilt fault: MC pos/vel fallback (err 15.9 deg)`；50 Hz 状态在 40.116 s 首次记录 `fallback=1`，直到日志结束均未清除。
- 触发前舵机指令和期望角均为 0，运动观测约 -15.9 deg；40.1 s 水平速度约 17.33 m/s，体轴比力 X 约 -2.73 m/s²。运动观测将这部分比力解释为倾转误差。
- 66.659 s：输入 CH8 从 993 切到 2034，是日志内首次 Vector 开关高位；此时故障早已锁存。
- 后续 AUX1 到达正负满量程，但 `vector_attitude_reference.valid=0`，舵机输出保持 0。

倾转观测没有实际舵机角度反馈，因此日志不能证明机构真的卡滞或实际偏转了 -15.9 deg。高速运动时气动力未包含在该观测模型内，是本次判定不可信的因素；不能仅凭本地模型文件认定日志中加载的仿真模型版本或精确气动力来源。

## 修复

1. 新增 `VT_TILT_VMAX`，默认 5 m/s。超过三维 NED 地速幅值范围，或者速度无效/陈旧时，观测变为未知并重置预热、故障确认计时。不限制飞行速度。
2. 保留真实故障锁存：速度超限、关闭监测、切换飞行模式或开关均不清除已经锁存的故障，只有上锁重置。
3. `VT_TILT_VMAX=0` 可显式恢复旧观测行为。地速不等于空速，低地速下的强风/接触力仍可能使观测模型无效；该修改没有增加气动力补偿。
4. 当锁存阻止 RC 模式切换时，明确发出 `Voliro: tilt fault latched; disarm to reset`，避免原先不再报告任何切换消息的情况。
5. `voliro_tilt_status` 新增 `speed`、`motion_valid`，区分运动条件无效和实际检测到的倾转故障。

不修改 Voliro/MorphQuad 分配算法、PID 增益、倾转机械映射或在机参数。

## 回归

`tests/testdata/tilt_monitor_log17.csv` 仅保留日志 37..42 s 的 250 行传感器片段，按传感器时间对齐稀疏记录，不声称完整复制实际任务调度。

- 同一片段在旧行为（无速度门控）中重现故障，加入门控后不锁存。
- 超速、速度无效/负值、丢失/陈旧/未来时间戳不能被判为舵机故障。
- 回到有效运动范围后重新预热，低速真实异常仍按门限和确认时间锁存；锁存只能由上锁清除。
- 真实三个控制模块/uORB 的合成输入测试：Normal → 高速不可信观测 → 低速 Vector → 真实持续异常 → 开关提示 → 上锁清除 → 再次 Vector。
- 原 Voliro 的 396,000 个参考/PID/分配浮点值继续与修改前基线逐位一致；原分配器回归和 MorphQuad 回归通过。

```sh
python3 src/modules/vector_pos_control/tests/run_tests.py build/cuav_7-nano_volirot
python3 src/modules/vector_pos_control/tests/runtime_reference.py build/px4_sitl_default --voliro
python3 src/modules/vector_pos_control/tests/runtime_reference.py build/px4_sitl_default
python3 src/modules/vector_pos_control/tests/runtime_smoke.py build/px4_sitl_default
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/cuav_7-nano_volirot
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_morphquad_framework.py build/cuav_7-nano_volirot
```

源码修复已构建为 `build/cuav_7-nano_volirot/cuav_7-nano_volirot.px4`；本次未刷写飞控或修改运行中的参数。
