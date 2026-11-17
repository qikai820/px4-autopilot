# 通用矢量控制框架

2026-11-13：Voliro mode 1 新增 `VT_RCT_P_SCALE=0.5`，仅降低反扭矩方向的速率 P 带宽，10–30°平滑介入。第二版保持最小比例到倒置端，保留原分配器、D/I/前馈，MorphQuad与普通MC不启用。软件回归、19角度原生测试通过；已刷写并完成±160°目标板载HIL，端点姿态RMS为0.207°/0.175°，位置变化均小于3 cm，无前电机触底。HIL使用Ixx=0.26、P=0.4、尾反推试验标度6.6及清零水平修正；测试后已恢复原参数。仍有约0.2°残余摆动，正向端点未满足原生严格精度阈值，不代表实机飞行验证。详见 `docs/Voliro_Reaction_Feedback_Fix_20261113.md`；下方2026-11-12记录为历史验证。

本目录统一矢量控制器的能力描述、姿态参考和参考有效性处理。位置、姿态、角速度三环继续使用现有实现；控制分配保留在各自的 `ActuatorEffectiveness` 中。

## 能力与约束

`VectorCapabilities.hpp` 描述推力方向能力，不代表任意角度、推力或故障情况下均具有完整控制能力。

| 机型 | 推力方向能力 | 独立姿态 | 机构约束 |
| --- | --- | --- | --- |
| MorphQuad，CA_AIRFRAME=16 | Spatial，双轴倾转 | 横滚、俯仰 | 四组旋翼、八个独立倾转轴 |
| Voliro T，CA_AIRFRAME=17 | LongitudinalPlane，单轴倾转 | 俯仰；横滚服从侧向力 | 前部公共倾转、固定方向可逆尾电机 |
| 普通多旋翼 | FixedAxis | 使用原 MC 控制链 | 固定旋翼方向 |

Voliro 是方向能力受约束的实例：双轴推力方向中固定 beta=0 即得到 `(sin(alpha), 0, -cos(alpha))`。公共倾转联动、反扭矩和尾电机继续由原 Voliro 分配器处理。原 MorphQuad 的 6×12 推力向量分配、零空间和舵机逆解保持原实现。

## 公共参考链

`AttitudeReference` 统一使用 `Rz(heading)*Rx(bank)*Ry(pitch)` 生成姿态及一致的 NED 角速度、角加速度。

- 平面能力：bank 来自侧向力；按受限 bank 修正可实现的力。原有 Voliro 参考算法、参数和增益调度保留。
- 空间能力：bank 和 pitch 均来自独立姿态请求；力不随这两个目标投影。参考速度、加速度和跟踪误差限制使 AUX、反向及回中保持连续。
- 使用同一个 `vector_attitude_reference`，并通过姿态环的 `vector_rate_reference` 传递前馈。时间戳、样本时间及估计器重置计数配对规则保留。
- 平面能力保留任务方向反馈；空间能力使用原机体系反馈增益，叠加相同坐标约定的角运动前馈。参考无效时使用标准航向速度前馈。
- 位置、姿态、角速度和 FlightTask 使用相同的启动模式选择函数。切换相关参数后重启。

## 参数与启用

Voliro 仍由 `VT_CTRL_MODE` 选择参考模式。`VEC_REF_EN` 对 Voliro 和普通 MC 不产生模式选择影响。

MorphQuad 的 19001/19002 配置默认启用 `VEC_REF_EN=1`；已经显式保存的用户值仍然优先。`VEC_REF_EN=0` 保留旧 AUX 路径。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| VEC_REF_EN | 全局 0；MorphQuad 机型默认 1 | 空间推力公共参考链，重启生效 |
| VEC_ATT_MAX | 20 deg，范围 0–120 deg | 独立横滚、俯仰幅度 |
| VEC_ATT_RATE | 20 deg/s | 独立姿态参考速度 |
| VEC_ATT_ACC | 60 deg/s² | 独立姿态参考加速度 |
| VEC_REF_FREQ | 8 rad/s | 参考模型频率 |
| VEC_HEAD_RATE | 45 deg/s | 航向参考速度 |
| VEC_HEAD_ACC | 90 deg/s² | 航向参考加速度 |
| VEC_POS_AW | 1 /s | 空间推力分配残差的积分回算 |

独立姿态还需要 `VEC_AUX_ATT=1`，并分别配置 `RC_MAP_AUX1`、`RC_MAP_AUX2`。增强路径在 Position、Position Slow、Hold 且已解锁、离地、无失效保护时接受独立姿态。无效输入、Level 或关闭 Vector 请求时，在该路径内平滑回中。自动任务、定高和姿态/角速度 Offboard 不接受这条 AUX 请求，保持相应任务目标。

2026-11-17：`VEC_ATT_MAX` 的参数元数据和运行期限幅统一扩展到 120°，原来的 60° 硬限幅已移除。空间航向使用参考/连续航向选择 `Rz*Rx*Ry` 的分支，横滚穿过 ±90°及航向估计器复位时保持连续。标准 ZYX 欧拉角的俯仰显示范围是 ±90°，单轴俯仰 120°可能显示成俯仰 60°并伴随横滚/偏航 180°；使用四元数和 `vector_attitude_reference.bank/pitch` 判断实际目标。

本次软件验证覆盖单轴横滚/俯仰 ±120°后分别回正的理想刚体闭环，以及双轴 ±120°的真实控制模块参考生成、估计器复位与 RC 失效回正。原分配器保持原实现。直接在多个大姿态之间组合切换的扩展实验暴露了舵机 ±180°边界选解和限速问题，未纳入通过范围；实验性的边界处理因位置保持检查失败已撤回。上述测试不证明任意组合姿态或实机飞行能力。

同日已将镜像 SHA256 `25137f683e5798bb379f7a19ec55765f276f9e72238a2977fe497952a187a0ca` 刷入 CUAV 7 Nano 并通过完整 CRC 校验。使用网口 `MorphQuadNet` 模型和电脑 USB/UDP 转发完成板载 HIL：横滚 +120°/-120°的稳态均值为 +119.905°/-120.042°，俯仰 +120°/-120°为 +120.007°/-120.012°；每次单轴动作后分别回正。动作期间记录的水平位置偏移最大约 5.6 cm，垂直位置跨度约 8.5 cm。另行完成正常着陆、上锁与 RC 接收驱动恢复。该结果仍仅覆盖单轴 HIL，组合大姿态和实机飞行未验证。证据位于本机 `/home/qikai820/tmp/morphquad-angle120-20261117/hil-validation-summary.json`。

本次修改范围是框架与参考链，未扩展原分配算法、舵机故障降级、连续翻转或接触/大姿态着陆能力。

## 回归命令

在仓库根目录执行，先构建生成参数和 uORB 头文件：

```sh
make cuav_7-nano_volirot
make px4_sitl_default
python3 src/modules/vector_pos_control/tests/run_tests.py build/cuav_7-nano_volirot
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/cuav_7-nano_volirot
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_morphquad_framework.py build/cuav_7-nano_volirot
python3 src/modules/vector_pos_control/tests/runtime_smoke.py build/px4_sitl_default
python3 src/modules/vector_pos_control/tests/runtime_reference.py build/px4_sitl_default
python3 src/modules/vector_pos_control/tests/runtime_reference.py build/px4_sitl_default --spatial-large
```

`runtime_reference.py` 在独立 SITL 实例中加载测试模块，通过真实 uORB 驱动三个控制模块，验证参考持续更新、RC 回中、估计器重置和前馈消息传递。测试没有运行传感器、Commander、地面站连接或硬件输出驱动，不是飞行测试。

修改前源码快照或 checkout 可用于逐浮点值比较：

```sh
python3 src/modules/vector_pos_control/tests/compare_baseline.py /path/to/baseline build/cuav_7-nano_volirot
```

比较器用同一个回放驱动分别编译两个版本，覆盖 Voliro 旧/动力学参考、姿态和角速度反馈、积分、饱和、原分配输出，要求 12,000 个样本的 396,000 个浮点值逐位一致。普通回归继续覆盖固定舵机切换、倾转故障、尾电机换向和 AirMode 起飞输入回放。

## 本次验证记录（2026-11-12）

- CUAV 7 Nano `volirot` 与 SITL 构建通过。
- Voliro 修改前后 396,000 个浮点值逐位一致；原有控制器回归的数值输出一致。
- MorphQuad 原分配器六轴力/力矩正反向、单位、映射和零输入测试通过。
- 公共空间参考、前馈、原 MorphQuad 分配器组成的理想刚体闭环通过六组独立/组合姿态和回中；该模型没有实机舵机动态。
- 10 组 SITL 启动配置通过，包含 MorphQuad 新旧参考、Voliro 两种模式与监测开关、普通 MC，以及空间开关对 Voliro 的隔离。
- 真实三控制模块的 uORB 集成测试通过：双轴参考到达 ±10°，估计器重置前后参考有效，角速度参考有效，RC 失效后回中。
- 原生 JSBSim 手动低油门 AirMode 扩展测试未通过：使用当前 `data/aviation/aircraft/voliro_t` 模型，旧源码与新源码均复现相同不收敛，两个已生成 CSV 逐字节一致。它是本次基线中已有的模型/工况失败，没有计入通过项，也未通过修改分配器掩盖。
- 板上 HIL、刷写及实机飞行不在本次已完成验证范围。

## log17：高速运动导致倾转故障锁存

2026-11-12 的 HIL 日志显示，约 40.1 s 的高速运动触发了倾转故障，首次请求 Vector（约 66.7 s）前已经锁存。运动观测现在由 `VT_TILT_VMAX`（默认 5 m/s 的三维地速幅值）限制适用范围，缺失/陈旧速度或超限仅使观测无效，不清除既有故障。0 恢复旧行为；强风与接触力仍超出模型保证范围。`voliro_tilt_status.speed/motion_valid` 可诊断门控原因。

锁存期间拨动开关会报告需要上锁重置。传感器片段回放、低速真实故障检测和模块级模式转换测试见 `tests/test_voliro_monitor_replay.cpp` 及 `runtime_reference.py --voliro`，详细记录在仓库 `docs/Voliro_Vector_Mode_Log17_20261112.md`。
