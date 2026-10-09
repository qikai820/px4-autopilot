# Viztra 目标位置预引导

## 数据链路

`h7_viztra` 仿真插件保留原有 `9870` 相对位置诊断 JSON、`9871` 目标真值 JSON 和 21 字节视觉串口帧；另以每秒 20 次向 QGC 所在主机的 UDP `9872` 端口发送目标位置二进制帧。QGC 飞行视图 Actions 下方的 **Target** 开关默认关闭；打开后只转发当前选中飞行器的目标数据，切换飞行器会自动关闭。界面显示 `Waiting`、`Sending` 或 `Stale`。

`9871` 调试 JSON 继续包含仿真时间、经纬度、MSL 高度和 NED 速度，例如：

```json
{"type":"dock_target","session":1,"seq":2,"sim_time_s":12.34,"lat_deg":47.0,"lon_deg":8.0,"alt_msl_m":500.0,"vn_m_s":5.0,"ve_m_s":0.0,"vd_m_s":0.0}
```

以下二进制格式只用于 `9872` QGC 输入，与调试 JSON 相互独立。

UDP 使用 64 字节小端 `UavInfo` 帧：`EB 90` 同步头、长度 `64`、序号、帧 ID `0x30`，经纬度为 `deg × 1e7` 的 `int32`，MSL 高度为整数米 `uint16`，地速为 `0.01 m/s` 的 `uint16`，yaw 为目标机头朝向 `0.01°` 的 `uint16`；其余未约定字段置零，结尾为 `90 EB`。当前协议不校验 checksum，也不从 `recv[35]` 解析速度。QGC 从连续位置估计航迹方向，结合地速形成 NED 速度，再通过当前飞行器主链路发送 MAVLink 2 `VIZTRA_TARGET`（ID 47001，源 component 25）。PX4 只接收发给本机且源 component 为 25 的有效消息，并发布 `viztra_target` uORB。此消息的 XML 定义在 PX4 与 QGC 的 `src/modules/mavlink/viztra.xml`、`src/MAVLink/viztra.xml`，两份必须保持一致。

## DockGuidance 行为

`DOCK_PRE_EN` 默认开启。在进入 Dock 模式时，若目标消息与本机全局定位均新鲜，先进入 `PRE_POSITION`。预引导从目标历史轨迹沿运动方向反推 `DOCK_PRE_DIST`（默认 100 m），并按目标速度短时外推；速度、加速度和 jerk 受现有 DockGuidance 限制约束。目标速度为零且没有可用历史方向时保持安全减速，不猜测轨迹后方。

`DOCK_PRE_REAR` 默认开启：连续有效视觉跟踪达到 `DOCK_PRE_VIS_T`（默认 0.3 s），且本机进入移动后方参考点附近后，才交接原有 `ALIGN` 追击对接逻辑。接近阈值为 `DOCK_PRE_DIST` 的 25%，限制在 2～25 m；默认后方距离 100 m 时阈值为 25 m。交接还要求目标位置流新鲜、后方参考点有效且起飞爬升已结束。等待交接时继续跟随移动参考点并匹配目标速度，视觉暂时无效则继续留在目标后方。关闭 `DOCK_PRE_REAR` 恢复仅按视觉稳定时间交接的旧行为。

`DOCK_PRE_RADIUS` 默认 150 m，限制预引导速度轨迹的水平曲率：水平速度达到 2 m/s 后，侧向加速度不超过速度平方除以设定半径；目标更急的转弯会被放缓。设为 0 可关闭此限制。该参数约束发布的速度轨迹，不保证实测航迹半径；实测还受位置控制器、机体响应和风影响。起步与接近停止时允许重新选择运动方向，避免低速锁死。

预引导中有有效图像时，相机舵机沿用图像俯仰误差控制；没有有效图像而目标位置流与机体姿态新鲜时，先用目标位置预对准相机。目标位置不代替 Viztra 有效视觉，也不能单独触发交接。目标流超过 `DOCK_PRE_TOUT`（默认 300 ms）后减速；持续中断会触发既有模式故障退出策略。预引导状态可从 `dock_preposition_status` uORB 与 ULog 查看。

ULog 默认以最高 50 Hz 记录 `dock_preposition_status`，并记录收到的 `viztra_target`。前者包含目标帧序号和源时间、目标与机体数据年龄、预测时长、目标预测位置与速度、后方参考点、机体位置与速度、参考点误差、限速前请求速度、最终指令速度、期望偏航及视觉稳定时间。状态码 0～4 依次为退出、等待、引导、目标流过期、视觉交接；5 表示本机定位无效，6 表示模式故障。`target_valid` 表示目标流新鲜，`reference_valid` 表示已经算出有效后方参考点；参考点无效时距离和误差记为 NaN，不把旧参考点误认为当前有效指令。可按时间戳对齐 `trajectory_setpoint` 和 `vehicle_local_position` 检查下层跟踪结果。

参数：`DOCK_PRE_DIST` 后方距离，`DOCK_PRE_REAR` 是否到达目标后方再交接，`DOCK_PRE_RADIUS` 引导速度轨迹最小转弯半径，`DOCK_PRE_ZOFF` 相对后方轨迹点的高度偏移（正值表示更高），`DOCK_PRE_VIS_T` 视觉品质持续时间，`DOCK_PRE_TOUT` 目标流超时，`DOCK_PRE_PRED_T` 最大预测时间。关闭 `DOCK_PRE_EN` 后，下次进入 Dock 模式直接使用原对接流程。

飞行前确认 QGC 已连接使用 `viztra` dialect 的固件、飞行器链路为 MAVLink 2、Target 开关处于 `Sending`，并核对 `viztra_target` 与 `dock_preposition_status` 的位置和时间戳。编译通过不代表飞行闭环已验证。
