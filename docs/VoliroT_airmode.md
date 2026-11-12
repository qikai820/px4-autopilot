# Voliro T 手动 AirMode：推力权限修正（2026-11-11）

此版本取代最初的 `voliro_airmode_20261111` 固件。旧版只验收了低油门姿态收敛，
允许为了 Pitch 力矩大幅增加共同推力，也没有覆盖起飞和姿态变化时的升力权限。
实机日志 `baa9a7b4-04f7-47d3-b338-43aae281b760`（SYS_HITL=0）暴露了这个缺陷：
约 7.82 s 请求总推力 4.8%，分配后约 33.1%，随后上升速度达到约 7.9 m/s。
旧版 AirMode 固件不应继续用于实机起飞。

## 操作和优先级

`CA_AIRFRAME=17` 使用 **VC_AIRMODE**，无需改变 MC_AIRMODE 或 CA_METHOD。

| 值 | 行为 |
|---|---|
| 0 | 保留关闭 AirMode 时的原路径 |
| 1 | 手动 Stabilized/Acro 的 Roll/Pitch 分配，在有限推力预算内纠偏 |
| 2 | 同时考虑 Yaw 差动；不能突破同一推力预算 |

支持 VT_CTRL_MODE=0/1。高度/爬升率/位置/速度/加速度/自动/Offboard 不使用此手动补偿。
控制模式消息必须有效且不超过 1 s。解锁本身不授予无限增加共同推力的权限。

**推力约束优先于不可实现的力矩请求：**

- 地面、可能着陆、接地，或着陆信息缺失/过期/未来时间：不允许 AirMode 增推。
- 确认离地：增推预算为 `min(0.02 * total_max, 0.25 * requested_force)`。
  例如请求 0.04，输出命令范数最多为 0.05；请求 0.16，最多为 0.18。
- 请求为零时预算也为零，姿态误差不能自行产生升力。
- 不再保存历史补偿推力下限。收油门立即收紧输出命令上限。
- 在 M5 斜率、死区、迟滞和换向等待之后重算前推力，并在通用斜率处理之后再次检查
  最终输出命令。降低油门上限可以比名义电机斜率更快地减小命令；不会因此授予反向权限，
  反向确认和零输出等待仍生效。
- 通过最终命令重建原始请求的分配残差，未实现的 Pitch/Roll/Yaw 力矩明确保留为残差。
  不再通过小 CG 力臂的除法强行抬高前电机推力。

这里的“推力”是由配置的几何和推力映射计算的输出命令，不是测量得到的真实推力。
实际转速、桨叶、电调映射和机械力臂仍需标定；本修正不修改这些参数。

## 起飞前后保持相同杆量映射

Voliro AirMode 下，手动曲线以零为下端点，最低油门在曲线之后作为下限钳制，
不再作为不断变化的插值端点。曲线模式和上半段映射继续服从 MPC_THR_CURVE。

例如 MPC_THR_HOVER=0.2、油门杆=-0.748：原映射随着 landed 退出从约 0.0504
移动到约 0.0803；新映射在地面和空中均为约 0.0504。只在杆量映射低于最低油门时
才触及下限。因此飞行中最底油门仍遵守 MPC_MANTHR_MIN；它不等于解除武装。

已有 VC_*RATE_P/I/D/K/FF、积分状态和积分上限保留。手动低油门 AirMode 的 Pitch
积分累积仍按 `clamp(collective/MPC_THR_HOVER, 0.25, 1)` 缩放，
并与原有大角度调度取较小值。

## 回归验收

```sh
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py --sanitize
python3 src/modules/vector_pos_control/tests/run_tests.py build/cuav_7-nano_volirot
```

原生 JSBSim 闭环：

```sh
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py \
  --jsbsim-source /path/to/jsbsim_realtime \
  --jsbsim-library /path/to/jsbsim_realtime/build/native/src/libJSBSim.a \
  --output /tmp/voliro-airmode-traces
```

测试包括：

1. 新实机日志首次 Kill 之前的 375 组油门、力矩和接地状态输入，经生产分配器回放；
   每帧检查地面/空中的推力限制。既用原上游推力隔离验证分配器，也验证修正后的油门曲线。
   `testdata/manual_takeoff_baa9a7b4.csv` 不含位置、设备身份或全量日志。
2. 固定油门，分别扫 Roll/Pitch/Yaw 力矩，要求同一个共同推力上限始终成立。
3. 地面/离地/着陆信息缺失、过期和未来时间；零油门、收油门、加油门和通用斜率残留输出。
4. 三种手动曲线的单调性、起飞判定前后固定杆量不跳变，以及最低油门下限。
5. 原有刚体和原生 JSBSim 低油门姿态测试，同时验收角度收敛与输出推力上限；
   覆盖 50/100/150 ms 电机延迟、0.75/1/1.5 倍惯量、两种 VT 模式、非对称 M5。
6. 尾桨保护、受限后的残差、倾转/固定倾转回退及原有 PID/参考/大角度回归。

原生测试的临时模型副本只移除网络输入/输出；保留电机、螺旋桨、质量、CG、惯量和基础控制参数。
输入回放不会预测修复后的真实飞行轨迹；主机闭环也不代表新的板上 HIL 或实飞验收。

19003 新参数存储默认 VC_AIRMODE=1。已有保存值需核对。固件升级后可显式设置：

```sh
param set VC_AIRMODE 1
param save
```

新固件必须先做受控台架/板上验证。确认油门不动时姿态纠偏的输出推力上限、收油门响应和
起飞判定切换均符合预期后，才能进一步验收实机行为。
