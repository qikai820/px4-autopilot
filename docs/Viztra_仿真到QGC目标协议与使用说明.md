# Viztra 仿真到 QGC 的目标位置协议与使用说明

本文记录 `h7_viztra` 仿真到定制 QGroundControl（QGC）的 **UDP 9872 换算系数修订目标**及链路使用方式。这里的“目标”是仿真中的对接目标；UDP `9872` 传的是目标位置，不是飞控常规遥测，也不是 Viztra 视觉串口观测。

> **源码已同步，运行程序需分别确认。**仿真 `SensorViztraPlugin` 与 QGC `DockTargetForwarder` 源码已采用第 2 节的新系数；旧二进制仍按旧系数（经纬度 `1e-7°`、高度 `1 m`、地速 `0.01 m/s`、航向 `0.01°` 每整数单位）工作。两端都重新构建并确认运行的是新程序前，不要将旧 UDP 数据按新系数解读。PX4 的 MAVLink `VIZTRA_TARGET` 定义不在本次 UDP 系数修订范围内。

## 1. 数据链路与端口

```text
JSBSim SensorViztraPlugin
  ├─ UDP 9870：相对位置诊断 JSON
  ├─ UDP 9871：目标真值诊断 JSON
  ├─ 串口：21 字节 Viztra 视觉帧 → PX4 视觉输入
  └─ UDP 9872：64 字节目标位置帧 → QGC DockTargetForwarder
                                      └─ MAVLink 2 VIZTRA_TARGET (47001)
                                         → 当前选中飞行器的主链路
                                         → PX4 viztra_target uORB
                                         → DockGuidance 预引导
```

`9872` 是仿真到 QGC 的**自定义 UDP 二进制协议**；QGC 到 PX4 才是 MAVLink。`9870`、`9871` 仅用于诊断，QGC 的目标转发器不读取它们。PX4 与 JSBSim 之间的 HIL 链路、QGC 与 PX4 的常规遥测链路也各自独立，不能用 `9872` 代替。

当前 `px4_jsbsim/px4-jsbsim-bridge/configs/h7_viztra.xml` 设置 `qgc_target_udp_enable=1`、目标地址 `127.0.0.1`、端口 `9872`、发送频率 `20 Hz`。QGC 默认监听 IPv4 `0.0.0.0:9872`。仿真和 QGC 在不同主机时，将 `qgc_target_udp_address` 改为 **QGC 主机可达的 IPv4 地址**；`127.0.0.1` 只适合同机。若改端口，QGC 的 `listenPort` 也要一致。目标地址是发送目的地址，QGC 的 `0.0.0.0` 是本机监听地址，两者含义不同。

## 2. UDP 9872 帧格式

修订后的每个 UDP 数据报仍恰好一帧，共 **64 字节**。多字节整数为**小端**；偏移从 0 开始。下表中“物理量”均按 `原始整数 × 系数` 解码；发送端按 `物理量 ÷ 系数` 四舍五入编码。字节布局、字段类型和非换算字段保持原定义。

| 偏移 | 长度 | 类型 | 内容 |
| ---: | ---: | --- | --- |
| 0–1 | 2 | 字节 | 同步头 `EB 90` |
| 2 | 1 | `uint8` | 帧长 `64`（`0x40`） |
| 3 | 1 | `uint8` | 仿真发送序号，逐帧递增，按 256 回绕；QGC 当前不据此去重 |
| 4 | 1 | `uint8` | 帧 ID `0x30` |
| 5–8 | 4 | `int32` | **经度**，原始值 × `1e-6°`；编码为 WGS84 经度（度）× `1e6` |
| 9–12 | 4 | `int32` | **纬度**，原始值 × `1e-6°`；编码为 WGS84 纬度（度）× `1e6` |
| 13–14 | 2 | `uint16` | 目标 MSL 海拔，原始值 × `0.1 m`；编码为高度（米）× `10` |
| 15–16 | 2 | `uint16` | 水平地速，原始值 × `0.1 m/s`；编码为地速（米/秒）× `10` |
| 17–18 | 2 | `uint16` | 目标机头航向，原始值 × `0.1°`；编码为航向（度）× `10`，归一化至 `[0°, 360°)` |
| 19–20 | 2 | 字节 | 当前发送端置零，QGC 不读取 |
| 21–22 | 2 | `uint16` | 当前发送端写入小端 `1`，QGC 不读取；不要据此推断目标 ID 语义 |
| 23–61 | 39 | 字节 | 当前发送端置零，QGC 不读取，包括未定义的 checksum/`recv` 区域 |
| 62–63 | 2 | 字节 | 帧尾 `90 EB` |

换算示例：经度 `8.123456°` → 原始整数 `8123456`，纬度 `47.123456°` → `47123456`，MSL 高度 `500.1 m` → `5001`，水平地速 `5.2 m/s` → `52`，航向 `90.1°` → `901`。按上述系数回乘可得到原物理量；经纬度分辨率为 `1e-6°`，其余三个量的分辨率为 `0.1` 对应单位。

发送端应先检查目标坐标和速度为有限值，再将仿真 NED 坐标换算为经纬度、MSL 高度、水平地速和机头航向。经纬度仍须位于纬度 `[-90,90]`、经度 `[-180,180]`。`uint16 × 0.1 m` 的高度可表示范围为 **0～6553.5 m**；超过此范围不能直接转为 `uint16`，应拒绝发送，不能截断或回绕。地速和航向仍按现有业务范围分别限制为不超过 `100 m/s` 和 `360°`。`9871` 调试 JSON 独立于本协议，继续保留浮点高度及 NED 速度。

QGC 应先按新系数将原始值还原为物理量，再检查纬度、经度、高度、地速和航向范围。对应的原始整数上限为：纬度绝对值 `90000000`、经度绝对值 `180000000`、高度 `65535`（即 `6553.5 m`）、现有 `100 m/s` 地速限制对应 `1000`、`360°` 航向限制对应 `3600`。原有 `50000 m` 高度业务上限超出了新 `uint16` 格式的可表示范围；修订后有效上限受帧格式限制为 `6553.5 m`。协议仍不校验 checksum，也不从帧内其他字节解析三维速度；帧中不包含仿真时间戳。

## 3. QGC 如何转为 MAVLink

QGC 收到有效帧后，用接收时刻的连续目标位置估计航迹方向和垂直速度；水平速度大小取 UDP 帧中的地速。机头 yaw 只用于地图目标图标的朝向，不直接当作航迹方向。首次接收或样本时间跨度不足 **800 ms** 时，速度尚未有效，打开 Target 后会显示 `Waiting`。位置跳变、位置变化与声明地速明显不符，或垂直速度超限，也会阻止本帧转发；正常样本窗口最长约 **1.2 s**。这些时间按 QGC 的接收时钟计算，不按仿真时间计算。

QGC 的 Target 开关默认关闭。打开后，它只向**当前选中飞行器**的主链路发 MAVLink 2 `VIZTRA_TARGET`（消息 ID `47001`）。切换当前飞行器会自动关闭转发；若无飞行器、无主链路、链路仍为 MAVLink 1 或速度还未估出，则不会发送。QGC 以源 component `25`（`MAV_COMP_ID_USER1`）编码消息，设置 `target_system` 为当前飞行器系统 ID、`target_id=1`、`valid_flags=0x03`；消息带 QGC 接收时刻、QGC 递增序号、经纬度、浮点 MSL 高度及 NED 速度 `[vn, ve, vd]`。UDP 经纬度按 `1e-6°` 还原为度后，写入 MAVLink 的 `lat/lon` 仍须按原定义使用 `degE7`，即对 UDP 原始整数乘 `10`；高度则用换算后的浮点米数。**这里的序号是 QGC 生成的 `uint32` 序号，不是 UDP 偏移 3 的 8 位序号。**

PX4 仅接受目标系统匹配、源 component 为 `25`、`target_id=1`、位置及速度有效位齐全，且坐标、海拔、速度通过范围检查的消息，然后发布 `viztra_target` uORB。PX4 的 `src/modules/mavlink/viztra.xml` 与 QGC 的 `src/MAVLink/viztra.xml` 必须使用相同消息定义；仅有普通 QGC 或未编入 `viztra` dialect 的 PX4，不能完成此链路。

## 4. 使用步骤

以下是链路操作步骤。**须确认发送端与 QGC 接收端都运行新构建程序**，才能按第 2 节验证新换算；任一端仍运行旧程序都会误解同一 UDP 帧。

1. **核对主机和端口。**同机使用当前 `127.0.0.1:9872` 即可；跨主机先在 `configs/h7_viztra.xml` 修改 `qgc_target_udp_address` 为 QGC 主机实际 IPv4 地址。确认防火墙允许该主机的 UDP `9872`。该配置里的 `fc_addr=192.168.3.10` / `fc_port=14561` 是另一条 PX4 仿真链路，应按现有仿真环境配置，不能当作 QGC 地址。
2. **启动匹配的 PX4 仿真/HIL 端点，再构建并启动 JSBSim。**先确认 PX4 使用包含 `viztra` dialect 的固件、仿真链路与 XML 中的 `fc_addr`/`fc_port` 对应。发送逻辑编在 `jsbsim_server` 中，`jsbsim_bridge` 会启动它；修改发送端 C++ 后两个目标都应重新构建。以下命令按当前本机目录布局执行：

   ```bash
   cd /home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge
   cmake --build build --target jsbsim_bridge jsbsim_server -j2
   PATH="$PWD/build:$PATH" HEADLESS=1 ./build/jsbsim_bridge h7_viztra -s scene/LSZH.xml
   ```

   启动输出应出现 `[ViztraSim] QGC target UDP output: ...:9872 at 20 Hz`。仿真目标的**运动**要等本机上升速度和离地高度同时达到配置阈值才开始；目标静止时也可发送位置帧。若沿用其他启动器，应确认实际加载的仍是 `h7_viztra.xml`，且启动的是含 `SensorViztraPlugin` 的新 `jsbsim_server`。
3. **启动本地定制 QGC 并连接飞行器。**当前本机构建可运行 `/home/qikai820/src/qgroundcontrol/build-3dgqt/Release/run-qgc.sh`；其他机器应运行包含 `DockTargetForwarder` 的 QGC 构建。确认 QGC 能看见目标 PX4、该主链路使用 MAVLink 2。QGC 默认监听 `0.0.0.0:9872`；若自定义了 `listenAddress` 或 `listenPort`，要与发送地址、端口匹配。
4. **在飞行视图的工具栏打开 `Target`。**地图目标标记由 `Show target on map` 设置控制，即使 Target 开关未打开也可显示接收位置；只有打开开关才会向当前飞行器转发。看到 `Waiting` 时先等连续有效位置形成速度；`Sending` 表示 QGC 已把消息交给飞行器主链路的发送接口；`Stale` 表示目标 UDP 超过约 `300 ms` 未更新。停止约 `2 s` 后地图标记隐藏。`Sending` 本身不证明 PX4 已接收或 DockGuidance 已采用目标。
5. **在 PX4 核对接收。**在 PX4 控制台执行 `listener viztra_target`，确认时间戳、位置和速度随目标更新；需要检查预引导时，再看 `listener dock_preposition_status` 或对应 ULog。进入 Dock 模式前还需满足该模式自身的定位、参数和视觉交接条件，详见 [Viztra 目标位置预引导](ViztraTarget_preposition.md)。

## 5. 快速排查

在 QGC 主机可先用 `ss -lunp | rg ':9872\b'` 检查监听，再用 `sudo tcpdump -ni any -c 5 'udp dst port 9872'` 被动确认数据报是否到达；抓包不会占用 QGC 的监听端口。跨主机时，抓包应在 QGC 主机上进行。

| 现象 | 优先检查 |
| --- | --- |
| QGC 无飞行器，Target 不可点击 | QGC 与 PX4 的主链路是否建立；UDP `9872` 不负责发现飞行器 |
| 没有地图目标、一直 `Waiting` | 仿真是否加载 `h7_viztra`、`jsbsim_server` 是否含新插件、目的 IP/端口、QGC 监听端口，以及是否持续收到合法 64 字节帧 |
| 收到目标但仍 `Waiting` | 等待至少 800 ms 的连续样本；检查位置是否跳变、地速与轨迹是否一致，以及主链路是否为 MAVLink 2 |
| `Sending`，但 `listener viztra_target` 无更新 | 检查选中的飞行器、PX4 `viztra` dialect、系统 ID、主链路是否真正通到 PX4；不要把 QGC 发送状态当成 PX4 接收确认 |
| 很快变 `Stale` | 仿真发送是否停止、跨主机路由/防火墙、QGC 是否收到连续数据；QGC 按接收时间约 300 ms 判过期 |
| 目标位置正常而 Dock 不进入预引导 | 检查 `DOCK_PRE_EN`、本机全局定位、目标流新鲜度和 Dock 状态；`9872` 目标流不能替代有效视觉观测 |

## 6. 实现位置与验证边界

- 发送配置：`/home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge/configs/h7_viztra.xml`
- UDP 打包：`/home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge/src/sensor_viztra_plugin.cpp` 的 `sendQgcTargetUdp()`
- QGC 解包与转发：`/home/qikai820/src/qgroundcontrol/src/FollowMe/DockTargetForwarder.cc`
- QGC 默认监听设置：`/home/qikai820/src/qgroundcontrol/src/Settings/Viztra.SettingsGroup.json`
- PX4 MAVLink 接收：`src/modules/mavlink/mavlink_receiver.cpp` 的 `handle_message_viztra_target()`

仿真发送端和 QGC 接收端的打包、解包及范围检查已按本页同步修改。两端必须同时使用新程序，避免同一 UDP 帧被按不同单位解释。构建、QGC 显示、MAVLink 转发、PX4 接收与实际闭环飞行是不同验证层次；按第 4 节逐层确认，不以 `Sending` 或构建成功代替飞控接收和飞行验证。
