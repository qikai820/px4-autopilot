# Voliro T 大角度姿态控制实现与验证

本次修改给现有 `vector_pos_control → vector_att_control → vector_rate_control →
ActuatorEffectivenessVoliroT` 增加 `VT_CTRL_MODE=1`。默认值为 0；新模式需要重启，
三个控制模块的 `status` 会显示实际启动模式。当前交付完成了源码修改、本机闭环测试和
SITL/CUAV 7 Nano 构建；尚未部署到飞控、仿真板或完成板上 HIL 验证。

## 1. 控制坐标和算法

机体坐标为 FRD，导航坐标为 NED，四元数表示机体到导航系的旋转。机体惯量
`J=diag(VEC_IXX,VEC_IYY,VEC_IZZ)` 固定在机体系。俯仰到 90° 时，世界航向转动
主要落到机体 X 轴，力平面的横向倾斜主要落到机体 Z 轴；控制所用的映射随姿态旋转。

### 连续航向和参考生成

保留现有力平面构型 `R_d = Rz(heading) Rx(bank) Ry(independent_pitch)`。
从机体 Y 轴在水平面的投影计算 `heading=atan2(-R01,R11)`，可穿过俯仰 ±90°。
该航向用于 FlightTask 的摇杆/保持逻辑，防止目标生成端重新锁定到跳变的常规欧拉航向。
常规欧拉角仍可用于显示。

力平面的 bank 仍有范围限制；机体 Y 轴接近竖直时此航向定义失效。本实现的默认
bank 上限为现有 `MPC_TILTMAX_AIR=20°`，内部上限 80°，并非任意翻滚姿态的全局航向定义。

`VectorAttitudeReference.hpp` 同时生成四元数、NED 角速度、NED 角加速度以及与
受限 bank 一致的力。参考模型约束 bank/pitch/heading 的速度和加速度，并提前减速到
角度限位；参考限幅损失的力反馈给位置环抗饱和。姿态误差增大时只减慢独立俯仰动作，
保留横向恢复速度。位置环的旧 RC/模式/落地/故障门限继续决定大角度动作是否允许。

NED 角速度和角加速度由上述旋转乘积求导，包含轴旋转的交叉项。
`yaw=NAN` 的速率控制请求会积分受限的航向速率，不反复锁定测量欧拉角。

### 姿态环与角速度环

姿态环使用现有完整四元数误差 `e_q=2*imag(canonical(q^-1*q_d))`。
令 `C=R(q)^T Rz(heading)`，新模式的机体角速度目标为：

```text
omega_sp = C * diag(VT_BANK_P, VT_PITCH_P, VT_HEAD_P) * C^T * e_q
           + R(q)^T * omega_reference_ned
```

现有 `VC_*RMAX` 继续约束机体角速度。任务轴增益随姿态映射，避免把固定机体 Z 轴
始终当成航向稳定通道。角速度环以角加速度作为反馈和积分的单位：

```text
omega_ff_body = R^T * omega_reference_ned
alpha_ff_body = R^T * alpha_reference_ned - omega_body × omega_ff_body
alpha_fb = P * (omega_sp - omega_body) + integral
           - D * (measured_angular_acceleration - alpha_ff_body)
torque_body_Nm = J * (alpha_ff_body + alpha_fb) + omega_body × (J * omega_body)
normalized_torque = torque_body_Nm / (VEC_TQ_R, VEC_TQ_P, VEC_TQ_Y)
```

`VT_RATE_P*` 单位为 1/s，`VT_RATE_I*` 为 1/s²，`VT_RATE_D*` 为无量纲，积分限幅为
rad/s²。新模式始终包含刚体交叉项，旧 `VEC_GYRO_FF` 开关仅控制旧 PID 的可选补偿。
积分同时使用分配残差的方向门控和幅值回算，残差先从归一化力矩还原为 Nm，再除以惯量。
位置积分的回算将归一化 NED 力残差还原到加速度单位。

### 分配和执行器

新模式将旧的固定机体 Z 轴力矩低通移至前桨反扭矩模态：

```text
reaction = sin(common_tilt) * torque_x - cos(common_tilt) * torque_z
```

该标量用 `VT_REACT_FILT` 滤波。俯仰约 90° 时机体 Z 轴的横向稳定力矩不会经过该滤波。

尾桨保持原有的斜率、死区、反转确认和零转速等待。分配器预演一份尾桨保护状态的副本，
在尾桨暂时不可用时重配前桨合力，保留部分俯仰阻尼；实际保护状态仍只推进一次。
重配在角度/推力限幅前最多增加请求合力模长 15% 的修正；执行器限幅后总力误差可能更大。
最终命令反算出的原始请求残差始终上报，覆盖尾桨保护和最终输出限幅。

新增 `CA_CT_T5RMAX` 表示独立标定的反向尾桨最大推力。0 保留旧的正反向对称模型。
正反向曲线统一用于命令映射、分配和残差反算。归一化总推力仍是
`4*CA_CT_TMAX + CA_CT_T5MAX`。

`VT_MOT_TAU`、`VT_SRV_TAU` 默认均为 0。启用后的预测结果标记为 estimate，
不是传感器反馈；舵机预测落后可减慢独立俯仰参考，电机预测仅用于诊断。
抗饱和的反馈使用最终命令模型，未把预测力矩当成实测力矩。

## 2. 时序、失效和日志

- `vector_attitude_reference` 与 `vehicle_attitude_setpoint` 使用完全相同的 timestamp。
- `vector_rate_reference` 与 `vehicle_rates_setpoint` 同样精确配对，并保存参考来源时间。
- 私有参考先发布，标准目标后发布。配对不符、超过 100 ms、未来时间、无效数据或
  `quat_reset_counter` 不同均禁止前馈。缺失私有参考时继续跟踪标准目标，并保留新模式
  的任务轴反馈增益，避免因丢失前馈而突然切回另一组增益。
- 估计器复位后位置模块旋转参考并清零导数；本地位置复位处理只调整输入轨迹，避免重复补偿。
- 退出位置控制或参考无效时清除参考状态。解锁状态无效、落地、显式 reset_integral 时清除新积分。
- `VT_CTRL_MODE=0` 使用旧增益和旧固定 Z 轴滤波；新模式不注入针对旧 VC 参数的自动调参信号。

新增日志主题均以最高约 50 Hz 记录：

| 主题 | 用途 |
| --- | --- |
| `vector_attitude_reference` | 原始/受限力、四元数、逻辑航向、bank/pitch、角速度和角加速度参考 |
| `vector_rate_reference` | 实际发布的角速度目标对应的前馈和估计器 epoch |
| `vector_rate_control_status` | 角加速度前馈/反馈/积分，Nm 命令、交叉项、分配残差，前馈有效标志 |
| `vector_actuator_status` | 最终命令与预测推力/力矩、公共倾角、反扭矩滤波、预测和 armed 标志 |

## 3. 参数匹配和启用

新模式是对角惯量模型，必须先匹配实际加载的仿真模型或实机标定结果。
目前本机 XML、旧测试日志和仓库示例机架的几何并不相同：

| 来源 | Ixx/Iyy/Izz (kg m²) | 前桨 X 力臂 / 尾桨力臂 (m) |
| --- | --- | --- |
| 本次本机原生 JSBSim fixture | 0.22 / 0.25 / 0.43 | 0.03 / 0.75 |
| 早期 MCAP 所对应的解析测试 fixture | 0.18 / 0.12 / 0.43 | 0.09 / 0.69 |
| 仓库 19003 示例机架 | 未覆盖通用惯量默认值 | 0.14 / 0.64 |

**以下值只用于本次本机 XML，不能据此认定仿真板已加载相同模型。** 本机测试读取
`/home/qikai820/src/jsbsim_realtime/data/aviation/aircraft/voliro_t`，测试输出包含 XML SHA256。
该模型质量 5.04 kg，四个前电机的命令等效最大推力各 39.5 N，尾桨正向 10 N、反向 6 N；
反向推力仍是模型估计值。示例匹配项为：

```text
CA_AIRFRAME=17
VEC_IXX=0.22  VEC_IYY=0.25  VEC_IZZ=0.43
CA_CT_X=0.03  CA_CT_Y=0.45  CA_CT_Z=0  CA_CT_TAIL=0.75
CA_CT_TMAX=39.5  CA_CT_T5MAX=10  CA_CT_T5RMAX=6  CA_CT_KM=0.02
CA_CT_ANGLE=120  CA_CT_RATE=90  CA_R_REV=16
VEC_TQ_R=10  VEC_TQ_P=10  VEC_TQ_Y=10
MPC_THR_HOVER=0.2942
```

确认模型和归一化一致后设置 `VT_CTRL_MODE=1` 并重启。分别运行三个 vector 控制模块的
`status`，均应显示 `Voliro dynamics: enabled (startup setting)`。
`VT_PITCH_MAX` 仍默认 10°，不会因新模式自动放宽大角度范围；参考速率仍由 `VT_PITCH_RATE` 控制。
RC AUX1、向量开关、回平开关及允许模式沿用原实现。更改模式或几何不应在飞行中热切换。

新模式初始增益：任务 bank/pitch/heading 为 2/2/1；角速度 P 为 8/12/8，
I 为 1/0.3/1，D 为 0.3/0.3/0.3。它们是本次模型测试的起点，需在匹配模型的板上 HIL 中复测。
保留公共机架示例原有几何、惯量和小角度默认范围，避免静默改变其他机型配置。

## 4. 可复现验证

在仓库根目录运行：

```sh
CCACHE_DISABLE=1 make px4_sitl_vector
python3 src/modules/vector_pos_control/tests/run_tests.py build/px4_sitl_vector
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/px4_sitl_vector
python3 src/modules/vector_pos_control/tests/runtime_smoke.py build/px4_sitl_vector
CCACHE_DISABLE=1 make cuav_7-nano_volirot
```

`run_tests.py` 覆盖旧 MC 行为等价、四元数正负号、俯仰 ±120° 的航向定义、±90° 任务轴映射、
参考旋转求导、角度边界减速、力/姿态一致性、复位、时序配对、积分回算和尾桨重配。
多速率闭环使用位置 100 Hz、姿态 200 Hz、角速度/分配 400 Hz，执行器包含斜率、反转保护、
50–150 ms 电机滞后，惯量整体偏差 ±20%。覆盖 0、±60、±85、±90、±100° 保持，
正反向 100° 过渡及过渡中的航向变化，另有连续 25 s 丢失角速度/角加速度前馈的退化工况。
每个工况运行 75 s，以最后 15 s 统计稳态。
新模式要求保持峰值误差 <3°、过渡峰值 <10°、稳态姿态 RMS <1°、位置 RMS <0.1 m、
最终 bank 峰值 <0.5°，失败直接退出。连续前馈丢失时允许 20° 航向指令的暂态跟踪滞后
增大，峰值要求 <20°，稳态指标保持相同；这项验证退化后仍收敛，不代表前馈丢失时性能相同。
旧控制仅作为相同模型下的对照打印。

可选的原生 JSBSim 测试直接链接本机库，用临时模型副本移除网络 IO 后运行：

```sh
python3 src/modules/vector_pos_control/tests/run_tests.py build/px4_sitl_vector \
  --jsbsim-model /home/qikai820/src/jsbsim_realtime/data/aviation/aircraft/voliro_t \
  --jsbsim-prefix /home/qikai820/src/px4_jsbsim/build/jsbsim_install \
  --jsbsim-systems /home/qikai820/src/jsbsim_realtime/data/aviation/systems
```

该项读取 XML 惯量和平均几何，保留实际 FCS 推力曲线、同轴安装偏置和刚体动力学，
不经过网络、EKF、RC 输入及 uORB 控制包装模块。加载模型前固定 400 Hz 步长，
避免电机滞后滤波器在另一时间步初始化后形成不同的有效时间常数。

2026-11-03 本机结果（姿态为四元数物理误差）：

| 原生 JSBSim 工况 | 峰值误差 | 最后 15 s 姿态 RMS | 位置 RMS |
| --- | ---: | ---: | ---: |
| 旧控制 +90° 保持 | 8.89° | 5.487° | 0.171 m |
| 新控制 +90° 保持 | 0.24° | <0.001° | <0.001 m |
| 新控制 −90° 保持 | 0.42° | 0.332° | 0.035 m |
| 新控制 0 → +100 → 0° | 3.97° | 0.174° | 0.002 m |
| 新控制 0 → −100 → 0° | 3.97° | 0.119° | 0.001 m |

新旧对照同时包含控制律、增益、滤波和尾桨处理的整体差异，不能把改善单独归因于某一个改动。
解析模型中正常前馈工况稳态姿态 RMS ≤0.268°，过渡峰值 ≤6.91°。
原生 JSBSim 中连续丢失前馈时，航向指令变化期间暂态姿态误差峰值 16.28°，
最终姿态 RMS ≤0.173°，位置 RMS ≤0.004 m，最终横向摆动未增长。
所有参考/控制/分配测试、四种独立未解锁模块启动检查、SITL 和 CUAV 构建均通过。
这些结果证明上述软件工况通过；传感器噪声、估计器、任务输入、板上时延、实测推力和实际飞行仍需验证。
