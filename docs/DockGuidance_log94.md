# log_94 静止目标垂向偏差修正

2026-10-08。输入为 `/home/qikai820/share/log_94_2026-10-8-12-01-30.ulg`
和 `/home/qikai820/share/export34.csv`，本报告区分 HIL / 模拟器记录与修改后的离线模型结果。

## 观测结论

CSV 目标固定在 N=150 m、E=0 m、高度=100 m，速度为 0。
按 CSV 内 `/dock_guidance_setpoint/timestamp` 对齐，主机时间与飞控时间的中位差
为 5935251.553948 s，ULog 起点为飞控时间 61.059439 s；对齐残差的
5% / 95% 分位约 -9.78 / +12.46 ms。以下 t 均为日志起点后的秒数。
对稀疏 CSV 字段分别取非空样本再插值，没有跨话题整行填充。

- t=23.725 s，模拟器真值相对北向距离过零时，飞机仍低于目标 **8.278 m**。
- t=23.882 s，三维距离的最近采样值 **6.622 m**。北向过零与最近点是不同指标。
- t=13.323 s 进入 Dock；相对引导分支确实生效。
- t=19.583 s 起，`control_allocator_status.thrust_setpoint_achieved` 持续为 false；
  总推力指令接近 1，电机输出接近上限。
- t=23.615 s 起视觉失效并进入恢复；t=23.907 s 指令失效，t=23.957 s 转 RTL。
  本次没有进入 TERMINAL_DASH / PASS_PREDICTED。

关键控制量，向上速度在本表取正（代码及日志采用 NED，向上为负）：

| t (s) | 指令前向速度 (m/s) | 实际前向速度 (m/s) | 指令爬升速度 (m/s) | 实际爬升速度 (m/s) |
|---|---:|---:|---:|---:|
| 20.0 | 25.69 | 21.13 | 15.79 | 13.84 |
| 21.0 | 29.88 | 26.17 | 19.80 | 14.35 |
| 22.0 | 32.40 | 29.65 | 23.70 | 13.63 |
| 23.0 | 28.77 | 29.03 | 27.74 | 15.51 |
| 23.5 | 25.07 | 26.60 | 29.42 | 17.55 |

来源为 `dock_guidance_setpoint.velocity_setpoint_ned_m_s`、
`vehicle_local_position.vx/vz`；真值高度差来自 CSV `/viztra/relative_d_m`。
t=22 s 相对估计北向 / 垂向为 49.05 / -36.64 m，模拟器约为 49.01 / -34.82 m。
该时刻新分支的 `trajectory_miss` 仍预测约 0.327 m。

**主要原因判断：旧轨迹模型高估了同时前向加速和爬升的可实现能力。**
此时爬升指令已经很大，但实际爬升速度受总推力、倾斜和阻力制约；
旧的一阶速度模型继续预测飞机可及时补齐高度差，导致修正方向过晚。
源码 `PositionControl::_velocityControl()` 先按 `MPC_THR_XY_MARG` 预留水平推力，
再限制垂向与剩余水平推力。本次 `MPC_THR_XY_MARG=0.8`、
`MPC_THR_MAX=1`、有效悬停推力约 0.35–0.36；前倾接近 50 度时，
满总推力并不等于满垂向推力。日志中的实际响应差异是直接证据，具体阻力模型属于近似。

## 实现修正

1. `RelativeResponse.hpp` 使用本机速度、净加速度、上一周期速度指令、机体推力和姿态，
   在线估计等效响应偏差及各轴有效二次阻力。没有加入目标绝对速度输入。
   无效、乱序和过期数据被拒绝；位置估计重置、参数校准变化、退出跟踪时清理状态。
2. `RelativeGuidance.hpp` 候选轨迹加入加速度、最大推力和水平预留约束。
   在速度 governor 后预测实际能产生的加速度，提前分配前向接近与爬升时间。
   保留原有总速度、垂向速度、jerk 和在线 FOV 约束。
3. `DockGuidanceRelative.cpp` 读取有效悬停推力估计（启用 HTE 时）或配置悬停推力，
   接入响应模型。冻结末段也使用同样的动力约束；本机响应残差扩大误差界，
   没有新鲜本机响应数据时禁止新提交盲穿。
4. `dock_guidance_relative` 新增响应偏差、有效阻力、响应残差、推力能力和有效标志，
   用于下一轮直接核对模型与实际速度。
5. 修复日志格式空间。原始日志报 `[logger] Format dock_guidance_prediction too long`，
   缺少该话题的 F 记录，标准 pyulog 即使过滤话题也会报 KeyError。
   格式缓冲区从 1600 改为 2048 字节，移到 Logger 成员中，避免增大任务栈。
   新增编译期容量检查及全部生成格式的解压/展开测试。

本次没有更改位置控制器增益、`MPC_THR_XY_MARG` 或板级默认参数。
模型仍使用局部辨识和有限候选搜索，不是精确气动模型或任意机动目标的命中保证。
短暂缺失响应数据时可保留普通跟踪，但不会提交新的盲穿。

## 日志读取修复边界

原始文件未改动。分析副本 `/tmp/dock-log94/readable.ulg` 仅删除没有格式定义的
`dock_guidance_prediction` 订阅 A 记录及对应 2847 条 D 记录。
其他字段、时戳和格式记录保留；没有重建缺失预测数据。
本报告使用有效的 `dock_guidance_relative` 及控制/真值话题。

## 修改后验证

新增 `test_relative_response.cpp`，链接生产代码的 `PositionControl`。
执行器模型按 250 Hz 独立积分推力方向滞后、重力与二次阻力；
引导层通过测量在线辨识阻力，不把模型真实阻力系数直接送入制导。
使用日志中的速度限制、速度 PID、上向加速度 4 m/s²、水平推力预留 0.8，
总速度上限 50 m/s。初始目标相对位置为 (150, 0, -98) m。

| 独立动力模型用例 | 最近距离 (m) | 到最近点时间 (s) | 最近点飞机速度 (m/s) |
|---|---:|---:|---:|
| 旧一阶预测、受限动力执行 | 9.238 | 8.720 | 29.603 |
| 修正后，静止目标 | 0.146 | 8.808 | 32.052 |
| 修正后，阻力系数乘 0.7 | 0.038 | 8.268 | 37.876 |
| 修正后，阻力系数乘 1.3 | 0.178 | 9.344 | 28.179 |
| 修正后，姿态响应 0.06 s | 0.143 | 8.804 | 32.057 |
| 修正后，姿态响应 0.15 s | 0.131 | 8.816 | 32.037 |
| 修正后，同向目标 10 m/s | 0.200 | 11.948 | 32.354 |
| 修正后，横移目标 5 m/s | 0.115 | 8.872 | 33.291 |

同一静止用例用时增加 0.088 s、交会速度增加约 2.45 m/s，不能据此宣称整段对接时间已缩短。
这些模型用例用准确相对几何并放宽 FOV 阈值，以隔离动力限制；
传感器噪声和 FOV 有既有独立回归，尚未把完整相机/姿态/观测链放入新动力模型。
初始状态、视觉延迟与 log_94 全链不同，因此表内绝对时间不能直接与日志对比。

另验证了受推力限制的冻结预测与名义预测存在显著差异、响应不确定度扩大误差界、
非法模型拒绝、过期/乱序/非有限数据保护。既有相对引导、TargetMotion 和预测对齐
测试通过；新增动力测试通过 ASan/UBSan。当前全部 **243** 种 uORB 格式可完整展开，
最大字段字符串 **1566** 字节，连同话题名和解码剩余字节均可放入 2048 字节缓冲区。

构建 `ninja -C build/cuav_7-nano_test` 成功，产物
`build/cuav_7-nano_test/cuav_7-nano_test.px4`。
未刷机；下一轮需用修正版 HIL 确认实际垂向误差、速度、耗时、
`response_valid`、`response_bias` 与 `planner_runtime_us`。

### 动力回归复现

在仓库根目录、生成 CUAV 构建头文件后：

```sh
g++ -std=c++14 -O2 -Wall -Wextra -Wshadow -Werror \
  -D__PX4_POSIX -D__PX4_LINUX -D__EXPORT= \
  -Iplatforms/common/include -Iplatforms/posix/include -Iplatforms/common \
  -Ibuild/cuav_7-nano_test -Isrc -Isrc/include -Isrc/lib -Isrc/lib/matrix \
  src/modules/dock_guidance/test_relative_response.cpp \
  src/modules/mc_pos_control/PositionControl/PositionControl.cpp \
  src/modules/mc_pos_control/PositionControl/ControlMath.cpp \
  -o /tmp/test_relative_response
/tmp/test_relative_response
```

### 日志格式回归复现

```sh
gcc -O2 -DHEATSHRINK_DYNAMIC_ALLOC=0 \
  -c src/lib/heatshrink/heatshrink/heatshrink_decoder.c -o /tmp/dock_heatshrink.o
g++ -std=gnu++14 -O2 -ffunction-sections -fdata-sections \
  -D__PX4_POSIX -D__PX4_LINUX -D__EXPORT= '-DMODULE_NAME="test_formats"' \
  -Iboards/px4/sitl/src -Iplatforms/common/include -Iplatforms/posix/include \
  -Iplatforms/common -Iplatforms/posix/src/px4/generic/generic/include \
  -Iplatforms/posix/src/px4/common/include -Ibuild/cuav_7-nano_test \
  -Isrc -Isrc/include -Isrc/lib -Isrc/lib/matrix \
  src/modules/logger/test_format_capacity.cpp \
  platforms/common/uORB/uORBMessageFields.cpp platforms/common/uORB/uORB.cpp \
  build/cuav_7-nano_test/msg/topics_sources/uORBMessageFieldsGenerated.cpp \
  /tmp/dock_heatshrink.o -Wl,--gc-sections -o /tmp/test_format_capacity
/tmp/test_format_capacity
```

详细文件清单见 [DockGuidance_relative.md](DockGuidance_relative.md)。
