# Voliro 主模型 50 ms / Ixx 0.28 板端 HIL 验证（2026-11-13）

已将仿真机主模型 `voliro_t` 的前四电机实际 RPM 一阶响应从 9.6/s（名义 104.17 ms）改为 20/s（名义 50 ms），并在 CUAV 7 Nano + 仿真板上完成匹配 Ixx 的滚转激励对照。角速度期望的残余振荡、超调和回中衰减均改善。测试后保留 50 ms 主模型，飞控上锁。

## 变更与保留范围

- 仿真板：`192.168.3.82`；飞控：`192.168.3.10`；实际加载路径 `/mnt/home/jsbsim/aircraft/voliro_t`。
- 唯一功能变更：`VoliroTRotorSpeeds.xml` 的 `fcs/voliro_t/rpm-model/lag-rate` 从 `9.6` 改为 `20`。前四电机推力、稳态反扭矩和自旋加速反作用均使用同一 RPM 状态。
- Ixx 固定为 **0.28 kg·m²**；自旋项启用 `1`，`J_eff=0.00079 kg·m²/rotor`；其余模型 XML 和运行配置在试验后逐文件回读，符合测试前备份。
- 保留飞控原控制器、原控制分配、固件和增益：`VC_ROLL_P=5.5`、`VC_ROLLRATE_P=0.26`、`I=0.20`、`D=0.003`。两次 ULog 中 VC/VEC/VT、CA/MPC/MC、RC、板载姿态偏置和 IMU 标定参数对比一致。
- 同步更新 `jsbsim_realtime/data/aviation`、`ROMFS/jsbsim` 两份 RPM 模型、MSC21 生成脚本和两份 README。只应用备份前后生成的局部补丁，保留仓库其他已有修改。
- RPM XML SHA-256：`d3999b633c741fad615af64706dd286937fe98923e227967001fd58e9d770dbb`。

## 实际测试方法

两次完整飞行都使用主模型名 `voliro_t`，Ixx=0.28。先做 104 ms 基线，正常降落上锁并封存记录，再部署 50 ms 并重新加载；每次新加载后在地面重启飞控并等待正常 preflight OK。

8 m 自动起飞后进入 Hold，依次输入 **+5° / −5° / +5°** 相对滚转脉冲，每次持续 1 s，回中观察 5 s，组间 Hold。Offboard 用完整机体系悬停推力向量，短时垂向伴随控制维持高度；两轮使用相同程序和保护。姿态、角速度控制及分配仍由实际飞控执行。

回中误差相对每组激励前的悬停滚转配平角计算；尾段指回中观察的最后 2 s。稳定时间为首次进入并保持 ±0.2° 到该组观察结束，且至少还有 0.6 s 连续观测。表格是三组指标的算术平均。

测试前发现 RC 第 8 通道为 993 μs（普通模式）；首次基线因固定倾转舵机导致水平漂移，在保护阈值 15 m 处自动转 Hold/降落并正常上锁。该次单独归档为 `baseline_mode_preflight_failed`，不参与对照。正式两轮都临时设置 `RC_MAP_VECTOR_SW=0` 固定矢量选择，并验证 `fixed_tilt=False`、`fallback=False`；落地后恢复原映射 8。`MIS_TAKEOFF_ALT` 也在落地后恢复 2.5 m。控制增益保持原值。

## 对照结果

| 指标（三组均值） | 约 104 ms 基线 | 50 ms 主模型 | 降幅 |
|---|---:|---:|---:|
| 滚转角速度期望尾段 RMS | 0.6738 °/s | 0.0692 °/s | 89.7% |
| 5° 激励平均超调 | 1.6712 ° | 0.5254 ° | 68.6% |
| 回中尾段角度误差 RMS | 0.1227 ° | 0.0126 ° | 89.7% |
| 回中尾段实际滚转角速度 RMS | 1.1694 °/s | 0.0980 °/s | 91.6% |
| 回中进入并保持 ±0.2° 的时间 | 3.4116 s | 0.7526 s | 77.9% |

![滚转响应与角速度期望对照](/home/qikai820/src/PX4-Autopilot/build/cuav_7-nano_volirot/voliro_main50_ixx028_20261113/roll-main50-comparison.png)

图中三组负向激励已翻转符号，阴影表示组间一个标准差。角速度期望直接取飞控 ULog 的 `vehicle_rates_setpoint.roll`；实际姿态/角速度来自飞控遥测，并另存模型 MCAP 真值作检查。模型真值的平均超调为 1.682° → 0.519°，与飞控测得趋势一致。两轮在滚转测试窗口内 `reaction_p_scale=1`、`feedforward_valid=0`，无故障回退、failsafe 或前电机零推力样本。

本轮对照支持：保留控制增益和自旋反作用时，缩短主模型的电机响应能明显改善回中振荡。角速度期望并非需要降至零的输入常量，外环会根据姿态误差持续修正；本轮其回中尾段的残余变化也随误差衰减而减小。

## 运行与回归证据

| 试验 | 飞控 ULog（UTC 文件名） | 仿真 MCAP | 完整性 |
|---|---|---|---|
| baseline | `08_36_35.ulg` | `jsbsim_voliro__1794558971.mcap` | ULog 无损坏、0 dropout；MCAP 129 个块 CRC 通过 |
| tau050 | `08_39_21.ulg` | `jsbsim_voliro__1794559137.mcap` | ULog 无损坏、0 dropout；MCAP 129 个块 CRC 通过 |

- 全量 ULog 通过 MAVFTP 下载并核对远端文件长度；SHA-256 见 `ULog-downloads.json`。概览和限定激励窗口摘要见各试验的 `ulog-overview.json`、`ulog-focus.json`。
- MCAP 结束标记及所有 LZ4 数据块 CRC 校验通过。实际采样属性分别为 9.6/s 与 20/s，自旋启用均为 1，Ixx 均为 0.2064988126 slug·ft²（按当前 JSBSim XML 换算常数 1.35594，对应输入 0.28 kg·m²）；与磁盘文件回读和启动日志一致。
- 原生 JSBSim 在装载前设置 dt=0.0025 s：旧 pole 的 RPM 63.2% 阶跃时间约 **105.4 ms**，新 pole 约 **51.3 ms**。离散状态输出包含约半个步长的时序差异，名义时间常数为 50 ms。
- MSC21 17 个厂家 F/Q 点、插值、低转速二次外推、完整 PWM→RPM→F/Q→共轴链、独立状态、H60 零端点、关机、单位和单桨恢复回归通过；见 `native-msc21.log`。两份模型与生成脚本输出逐字一致。
- 三旋翼/MorphQuad 的模型参数和共享飞控代码保持原样。本轮验证范围为 Voliro 主模型与板端小滚转脉冲，没有新增其他机型 HIL 飞行，也不据此推定大 pitch 或真机飞行性能。

## 交付状态与复测

主模型已在仿真机运行，前电机 pole=20/s、Ixx=0.28、自旋启用=1；飞控正常上锁、无 failsafe，临时遥测 14610 已停止，原 HIL 14561 保留。所有临时参数恢复原值。

遥控矢量开关映射恢复为第 8 通道，继续手动矢量悬停复测时，需要把该开关拨到矢量位置。50 ms 是本轮经 HIL 验证的工程设置，仍应以实际电机/桨的动态测量作后续物理标定。

备份、局部源补丁、启动日志、ULog、MCAP、测试程序和逐文件回读清单位于：

`/home/qikai820/src/PX4-Autopilot/build/cuav_7-nano_volirot/voliro_main50_ixx028_20261113`

本机备份为 `board-before/`、`source-before/`；板端旧 RPM 模型保存在 `VoliroTRotorSpeeds.xml.before-main50-20261113`。所有当前验证结果保留在 `final-state.json`、`source-validation.json` 和 `log-verification.json`。本轮没有提交或推送 Git 变更。
