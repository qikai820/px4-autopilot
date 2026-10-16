# Viztra 目标位置预引导

## 数据链路

`h7_viztra` 仿真插件保留原有 `9870` 相对位置诊断 JSON、`9871` 目标真值 JSON 和 21 字节视觉串口帧；另以每秒 2 次向 QGC 所在主机的 UDP `9872` 端口发送目标位置二进制帧。QGC 飞行视图 Actions 下方的 **Target** 开关默认关闭；打开后只转发当前选中飞行器的目标数据，切换飞行器会自动关闭。界面显示 `Waiting`、`Sending` 或 `Stale`。

`9871` 调试 JSON 继续包含仿真时间、经纬度、MSL 高度和 NED 速度，例如：

```json
{"type":"dock_target","session":1,"seq":2,"sim_time_s":12.34,"lat_deg":47.0,"lon_deg":8.0,"alt_msl_m":500.0,"vn_m_s":5.0,"ve_m_s":0.0,"vd_m_s":0.0}
```

以下二进制格式只用于 `9872` QGC 输入，与调试 JSON 相互独立。

UDP `9872` 使用 64 字节小端 `UavInfo` 帧：`EB 90` 同步头、长度 `64`、序号、帧 ID `0x30`，经纬度原始 `int32 × 1e-6°`，MSL 高度原始 `uint16 × 0.1 m`，水平地速原始 `uint16 × 0.1 m/s`，目标机头航向原始 `uint16 × 0.1°`；其余字节沿用现有布局，结尾为 `90 EB`。**仿真发送端与 QGC 接收端源码已同步，运行前仍须确认两端都使用新构建程序。**完整字节偏移、示例和可表示范围见 [仿真到 QGC 的目标位置协议与使用说明](Viztra_仿真到QGC目标协议与使用说明.md)。协议不校验 checksum，也不从 `recv[35]` 解析速度。QGC 从连续位置估计航迹方向，结合地速形成 NED 速度，再通过当前飞行器主链路发送 MAVLink 2 `VIZTRA_TARGET`（ID 47001，源 component 25）。PX4 只接收发给本机且源 component 为 25 的有效消息，并发布 `viztra_target` uORB。此消息的 XML 定义在 PX4 与 QGC 的 `src/modules/mavlink/viztra.xml`、`src/MAVLink/viztra.xml`，两份必须保持一致。

## DockGuidance 行为

`DOCK_PRE_EN` 默认开启。在进入 Dock 模式时，若目标消息与本机全局定位均新鲜，先进入 `PRE_POSITION`。预引导按目标速度短时外推，速度、加速度和 jerk 受现有 DockGuidance 限制约束。参考点的方向由 `DOCK_PRE_REAR` 选择；目标速度为零且没有可用历史方向时，后方路径以本机到目标的接近线作为参考。

`DOCK_PRE_REAR=1`（默认）时，预引导从目标历史轨迹沿运动方向反推 `DOCK_PRE_DIST`（默认 100 m）作为后方参考点。连续有效视觉跟踪达到 `DOCK_PRE_VIS_T`（默认 0.3 s），且本机进入移动后方参考点附近后，才交接原有 `ALIGN` 视觉对接逻辑。接近阈值为 `DOCK_PRE_DIST` 的 25%，限制在 2～25 m；默认后方距离 100 m 时阈值为 25 m。交接还要求目标位置流新鲜、后方参考点有效且起飞爬升已结束。等待交接时继续跟随移动参考点并匹配目标速度。若先进入 `DOCK_PRE_LOCK_D` 的三维距离门槛，则必须已有稳定视觉，满足时直接交接，不再等待后方参考点。

`DOCK_PRE_REAR=0` 时，参考点改为**本机当前位置到预测目标位置**的接近线上、距目标 `DOCK_PRE_DIST` 的水平停靠点，不再绕到目标轨迹后方。已经处于该距离内时，参考点取本机当前位置；移动目标仍有目标速度前馈。只要目标流新鲜、参考点有效、起飞爬升结束，且视觉连续有效达到 `DOCK_PRE_VIS_T`，就立即交接视觉 `ALIGN`，不必先到停靠点。视觉一直无效时仍减速趋向停靠点，不会直接按外部目标坐标穿越。

通常情况下，后方模式的视觉交接要求先到轨迹后方参考点；正面模式只等待稳定视觉。两种模式都不会仅凭外部位置进入视觉对接。`DOCK_PRE_LOCK_D` 默认 100 m，按本机到预测目标的三维直线距离判断：进入门槛时视觉尚未连续有效达到 `DOCK_PRE_VIS_T` 就故障退出；已经稳定锁定则立即交接视觉，即使后方参考点尚未到达。设为 0 可关闭此距离门槛。交接后若视觉米制尺度尚未建立，也不允许在门槛内继续使用 QGC 位置作远距离加速参考。若尚未交接而到达各自参考点，继续用外部目标流跟随移动参考点，并从就位时开始计时；`DOCK_PRE_WAIT_T` 默认 2 s，超时无视觉时按现有故障制动链和 `DOCK_EXIT_MODE` 退出。

轨迹后方路径若直线穿过预测目标的 `DOCK_PRE_CLEAR_D` 水平保护区，会生成绕行参考点，再回到轨迹后方参考点。默认保护距离 50 m，速度轨迹仍受 `DOCK_PRE_RADIUS` 和加速度约束；若已进入保护区且无法形成安全绕行，Dock 故障退出。`guidance_position_ned` 是当前控制用绕行点，`reference_position_ned` 仍是真正的交接参考点。

满足对应模式的交接条件时，向 QGC 发送一次 `Dock visual locked` 提示并进入 `ALIGN`：正面模式不要求参考点就位，后方模式仍要求就位。同一次 Dock 激活只提示一次。

![俯视示意：0° 位于目标轨迹后方，90° 位于侧方](assets/dock_guidance/preposition_approach_angles.svg)

图中的 0° 和 90° 是相对来向示意，不是可设置的方位角参数；`DOCK_PRE_REAR=0` 始终根据本机当前位置形成接近线，图中虚线不代表实际飞行轨迹。

`DOCK_PRE_RADIUS` 默认 250 m，限制预引导速度轨迹的水平曲率：水平速度达到 2 m/s 后，侧向加速度不超过速度平方除以设定半径；目标更急的转弯会被放缓。设为 0 可关闭此限制。该参数约束发布的速度轨迹，不保证实测航迹半径；实测还受位置控制器、机体响应和风影响。起步与接近停止时允许重新选择运动方向，避免低速锁死。

预引导追后方参考点时，机头持续朝向预测目标。该阶段用 `DOCK_PRE_YAW_R` 限制航向设定值变化率，默认 30°/s；进入视觉 `ALIGN` 后仍用 `DOCK_YAW_RATE`。在 `log_38` 中，目标方位在接近后点时约以 16°/s 旋转，而旧预引导沿用 10°/s 的 `DOCK_YAW_RATE`，画面水平偏移扩大到约 -946 px 后失效。提高预引导航向速率只解决这一项指令限速；实际机头响应和图像是否仍在视场内要用新 ULog 核对，不能用外部目标坐标代替视觉有效帧。

预引导中有有效图像时，相机舵机沿用图像俯仰误差控制；没有有效图像而目标位置流与机体姿态新鲜时，先用目标位置预对准相机。目标位置不代替 Viztra 有效视觉，也不能单独触发交接。目标流超过 `DOCK_PRE_TOUT`（默认 800 ms，适配约 2 Hz 输入）后减速；持续中断会触发既有模式故障退出策略。已有飞控的保存值不会因固件默认值变化而自动更新，应在参数界面或控制台确认 `DOCK_PRE_TOUT=800`。预引导状态可从 `dock_preposition_status` uORB 与 ULog 查看。

ULog 默认以最高 50 Hz 记录 `dock_preposition_status`，并记录收到的 `viztra_target`。前者包含目标帧序号和源时间、目标与机体数据年龄、预测时长、目标预测位置与速度、真正的交接参考点和当前控制参考点、机体位置与速度、参考点误差、限速前请求速度、最终指令速度、期望偏航及视觉稳定时间。状态码 0～4 依次为退出、等待、引导、目标流过期、视觉交接；5 表示本机定位无效，6 表示模式故障，7 表示已到参考点等待视觉。`reference_reached`、`visual_wait_s`、`bypass_active` 可分别核对就位、无锁等待和绕行。`target_valid` 表示目标流新鲜，`reference_valid` 表示已经算出有效参考点；参考点无效时距离和误差记为 NaN，不把旧参考点误认为当前有效指令。`DOCK_PRE_REAR=0` 时可用 `reference_position_ned` 与 `vehicle_position_ned` 核对是否沿当前接近线，`state=4` 后核对 `dock_guidance_setpoint.stage=ALIGN`。可按时间戳对齐 `trajectory_setpoint` 和 `vehicle_local_position` 检查下层跟踪结果。

参数：`DOCK_PRE_DIST` 目标到预置参考点的距离，`DOCK_PRE_LOCK_D` 允许仅凭外部位置引导的最小三维目标距离，`DOCK_PRE_WAIT_T` 就位后的视觉等待时间，`DOCK_PRE_CLEAR_D` 绕行保护距离，`DOCK_PRE_REAR` 选择轨迹后方或当前位置接近线，`DOCK_PRE_RADIUS` 引导速度轨迹最小转弯半径，`DOCK_PRE_YAW_R` 预置阶段航向设定值变化率，`DOCK_PRE_ZOFF` 相对预置参考点的高度偏移（正值表示更高），`DOCK_PRE_VIS_T` 有效视觉持续时间，`DOCK_PRE_TOUT` 目标流超时，`DOCK_PRE_PRED_T` 最大预测时间。关闭 `DOCK_PRE_EN` 后，下次进入 Dock 模式直接使用视觉路径；入模前必须已有有效视觉引导设定值，否则 Commander 会拒绝切换。

飞行前确认 QGC 已连接使用 `viztra` dialect 的固件、飞行器链路为 MAVLink 2、Target 开关处于 `Sending`，并核对 `viztra_target` 与 `dock_preposition_status` 的位置和时间戳。编译通过不代表飞行闭环已验证。

进入 `PASS_PREDICTED` 并首次发布减速设定值时，飞控向 QGC 消息栏发送一次 `Dock nearest 3D est X.XX m`，同时写入 ULog 消息；此前没有可信米制视觉距离时显示 `Dock nearest 3D distance N/A`。距离来自有效的 Viztra 米制观测、未裁剪目标框的标定距离及受限末段的相对状态估计，不使用有传输时延的目标 GPS 位置流冒充接触真值。该数字是减速开始前本次 Dock 过程的最近三维估计距离，`PASS_PREDICTED`、制动完成和这个数字均不代表实体接触确认。
