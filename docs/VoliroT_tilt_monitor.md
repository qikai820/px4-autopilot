# Voliro T 倾转监测与普通多旋翼位速控制降级

适用于 `CA_AIRFRAME=17`，同时支持 `VT_CTRL_MODE=0/1`。功能默认关闭。
这是基于运动响应的倾角观测，不是机械位置传感器反馈，也不能保证故障后仍有足够控制余量。

## 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `VT_TILT_EN` | 0 | 1：启用倾角监测及持续异常后的自动降级 |
| `VT_TILT_ERR` | 10° | 观测角与正常滞后/速率模型角的绝对误差阈值 |
| `VT_TILT_T` | 0.5 s | 连续有效观测超限的确认时间 |

示例配置（默认阈值是验证起点，尚非实机整定结果）：

```text
param set VT_TILT_ERR 10
param set VT_TILT_T 0.5
param set VT_TILT_EN 1
```

还需校准现有 `CA_CT_X/Z/TAIL`、`CA_CT_TMAX/T5MAX`、`VEC_IXX/IYY/IZZ`、
`MPC_THR_HOVER`、`CA_CT_RATE` 和 `VT_SRV_TAU`。质量使用与位置控制相同的归一化关系：
`mass = MPC_THR_HOVER * (4*CA_CT_TMAX + CA_CT_T5MAX) / g`，不通过故障中的悬停推力估计重新学习。
`VT_SRV_TAU` 为 0 时不预测一阶滞后；速率限制仍有效。它现在也用于旧模式的独立监测模型。

## 角度估算与异常确认

机体 FRD 坐标下，四个前桨合推力为 S，带符号尾桨推力为 T5：

```text
Fx = S sin(a)
Fz = -S cos(a) - T5
My = z Fx + x S cos(a) - tail T5

u = Fx
v = (My - tail Fz - z Fx) / (x + tail)
a_observed = atan2(u, v)
```

使用 `vehicle_acceleration` 的机体系比力得到 `F/m`；这里不再减一次重力。
使用 `vehicle_angular_velocity.xyz/xyz_derivative` 及对角惯量计算
`My = [J*angular_acceleration + omega cross (J*omega)]_y`，消去未知尾桨推力。
不对欧拉俯仰角求二阶导；大角度估算仍在机体系进行。

最终舵机输出由 `vector_actuator_status.tilt_command` 提供，包括旧模式。
指令经过 `CA_CT_RATE` 和 `VT_SRV_TAU` 模型，再与观测角使用相同的 0.15 s 一阶滤波。
比较机械有符号角度，不将 +160° 与 -160° 当作相邻机械位置。

仅在已解锁、位置/速度控制有效、未落地、未接地、未疑似落地时采样。
加速度、角运动和舵机指令均需在 100 ms 内；加速度与角运动采样差不超过 30 ms。
重复样本不累计时间。低前桨推力、明显横向外力、极端角运动、NaN、过时或未来时间戳
使观测无效，清除待确认时间，但不清除已经触发的降级。

连续有效观测先预热 `max(0.5 s, 3*VT_SRV_TAU)`，随后超限持续 `VT_TILT_T` 才触发。
短时误差和正常舵机响应时间不应直接触发。

## 自动降级行为

1. 首次降级时向 QGC 发送一条 Critical 级别 MAVLink `STATUSTEXT` 提醒：
   `Tilt fault: MC pos/vel fallback (err XX.X deg)`，表示倾转异常，位置/速度控制已降级，
   并给出触发时的绝对倾角误差。消息不附加制表符，不依赖 QGC 下载自定义事件描述。
   `voliro_tilt_fallback` 事件仍按内部 Critical 级别记录，关闭其外部提醒以避免重复通知。
   锁存期间不循环打印；解除武装后若下一次飞行再次触发，可再次提醒。清除水平速度积分。
2. 原位置 P、速度 PID 和轨迹输入继续运行，力与姿态生成使用复制的普通多旋翼算法。
   AUX 独立俯仰、向量开关和向量姿态前馈停止参与目标生成；导航模式不改变。
3. 保留 Voliro T 的五电机、公共倾转构型，不热切换 `CA_AIRFRAME`，不启动第二套控制器。
4. 按观测倾角进行固定倾角分配，优先满足可行俯仰力矩，再拟合合力。
   尾桨斜率、死区和换向保护继续生效。最终输出在观测倾角下重建力矩分配残差并反馈角速度环抗饱和；
   这仍是带观测角的模型计算，并非实测力矩。
   位置环使用普通多旋翼自身的推力饱和处理，不继续使用矢量分配短缺回算，避免将需要靠
   机身倾斜实现的水平力误当成持续不可用并抵消位置/速度指令。
5. 普通多旋翼姿态目标补偿固定倾角产生的合推力方向偏置，包括尾桨的俯仰配平力。
   姿态目标从上次输出沿四元数最短路径限速，速度上限为 `VT_BANK_RATE`（默认 80°/s）；
   不用较慢的独立俯仰速率限制普通位速控制的姿态响应。估计器复位时同步旋转目标。
6. 倾转舵机以 `0.5*min(CA_CT_RATE, VT_PITCH_RATE)` 的角速度尝试回中。
   只要观测仍显示卡在非零角度，就继续按该角度补偿和分配，不把回中指令当成实际回中。
   观测无效或状态通信超时则保持最后可信倾角，并暂停回中指令的继续变化。
7. 降级锁存到解除武装。遥控开关、观测恢复、退出再进入位置模式或飞行中关闭 `VT_TILT_EN`
   都不会自动切回向量控制。解除武装后清除监测和分配器状态，拒绝上一飞行周期的状态消息。

固定倾角补偿采用零俯仰力矩的合力方向：

```text
beta = atan2(tail*sin(a), (tail+x)*cos(a) + z*sin(a))
q_target = q_multicopter * Ry(beta)
```

机身可能保持非零俯仰以抵消卡滞倾角；这是普通位速闭环在故障构型下的推力方向补偿。
执行器推力/角度限制保持有效。严重卡滞、推力不足或丢失观测可能无法维持位置。
外部接触、风阻和 IMU 杆臂效应不在该简化模型中；接触作业不应启用此监测。

## 日志与验证

`vector_pos_control status` 显示启用、观测有效性、锁存状态、角度和角度误差。
ULog 新增 `voliro_tilt_status`，包含最终指令、正常预测角、观测角、误差、超限时间、
启用、armed、观测有效和 fallback 标志。无效误差为 NaN，观测角保持最后有效值。
该主题加入默认日志配置，使用常规订阅，默认记录间隔为 20 ms（最高约 50 Hz）；
即使 logger 启动时主题尚未发布，也会在后续发布后订阅并记录，不按 optional 主题跳过。
若 SD 卡上使用自定义 `etc/logging/logger_topics.txt` 覆盖默认列表，需在该文件加入：

```text
voliro_tilt_status 20
```

```sh
CCACHE_DISABLE=1 make px4_sitl_vector -j4
python3 src/modules/vector_pos_control/tests/run_tests.py build/px4_sitl_vector
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/px4_sitl_vector
python3 src/modules/vector_pos_control/tests/runtime_smoke.py build/px4_sitl_vector
CCACHE_DISABLE=1 make cuav_7-nano_volirot -j4
```

新增回归覆盖 ±160° 观测、尾桨消元、陀螺耦合、正常滞后/换向、超限持续时间、过时/重复/无效样本、
解除武装清除、普通多旋翼位速映射、卡滞倾角补偿、姿态目标限速、分配残差及消息丢失时锁存。
闭环刚体故障注入用于验证算法恢复，不代表已完成板上 HIL、真实舵机台架或飞行验证。
