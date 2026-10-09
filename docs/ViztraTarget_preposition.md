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

连续有效视觉跟踪达到 `DOCK_PRE_VIS_T`（默认 0.3 s）即交接原有 `ALIGN` 对接逻辑；到达 100 m 参考点不是交接条件。预引导中相机舵机沿用原有图像俯仰误差控制，目标 GPS 不直接驱动舵机。目标流超过 `DOCK_PRE_TOUT`（默认 300 ms）后减速；持续中断会触发既有模式故障退出策略。预引导状态可从 `dock_preposition_status` uORB 与 ULog 查看。

ULog 默认以最高 50 Hz 记录 `dock_preposition_status`，并记录收到的 `viztra_target`。前者包含目标帧序号和源时间、目标与机体数据年龄、预测时长、目标预测位置与速度、后方参考点、机体位置与速度、参考点误差、限速前请求速度、最终指令速度、期望偏航及视觉稳定时间。状态码 0～4 依次为退出、等待、引导、目标流过期、视觉交接；5 表示本机定位无效，6 表示模式故障。`target_valid` 表示目标流新鲜，`reference_valid` 表示已经算出有效后方参考点；参考点无效时距离和误差记为 NaN，不把旧参考点误认为当前有效指令。可按时间戳对齐 `trajectory_setpoint` 和 `vehicle_local_position` 检查下层跟踪结果。

参数：`DOCK_PRE_DIST` 后方距离，`DOCK_PRE_ZOFF` 相对后方轨迹点的高度偏移（正值表示更高），`DOCK_PRE_VIS_T` 视觉品质持续时间，`DOCK_PRE_TOUT` 目标流超时，`DOCK_PRE_PRED_T` 最大预测时间。关闭 `DOCK_PRE_EN` 后，下次进入 Dock 模式直接使用原对接流程。

飞行前确认 QGC 已连接使用 `viztra` dialect 的固件、飞行器链路为 MAVLink 2、Target 开关处于 `Sending`，并核对 `viztra_target` 与 `dock_preposition_status` 的位置和时间戳。编译通过不代表飞行闭环已验证。
