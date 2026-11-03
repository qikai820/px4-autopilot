# PX4 + JSBSim 纯软件 UDP 仿真

Voliro T 新增大角度控制模式的算法、参数匹配和离线闭环验证见
[大角度控制实现与验证](../../../docs/VoliroT_large_angle_control.md)。
`VT_CTRL_MODE` 默认关闭；本入口沿用所选机架和已有参数，不会自动启用或覆盖模型标定。

从 JSBSim 项目目录使用新入口。模型、执行器配置和场景来自原来的
`./scripts/run.sh voliro_t -s scene/LSZH.xml`；新入口为其生成私有 UDP 配置，
并启动本机 PX4 POSIX 实例。默认不解锁。

```sh
cd /home/qikai820/src/px4_jsbsim
# 首次构建或源码更新后添加 --build。
./scripts/run_sitl.sh voliro_t -s scene/LSZH.xml --qgc-port 1480 --build
```

当前工作区已经完成构建，后续可以省略 `--build`。QGC 标准 UDP 端口默认为
14550；这台电脑的 QGC 当前监听 1480，因此示例明确指定 `--qgc-port 1480`。

```sh
./scripts/run_sitl.sh status
./scripts/run_sitl.sh console -- commander status
./scripts/run_sitl.sh console -- listener vehicle_attitude -n 1
./scripts/run_sitl.sh console -- logger status
./scripts/run_sitl.sh logs
./scripts/run_sitl.sh stop
```

默认配置：

| 项目 | 值 |
| --- | --- |
| 模型/场景 | `voliro_t` / `scene/LSZH.xml` |
| PX4 实例 / MAVLink System ID | 1 / 2 |
| 仿真方式 | POSIX SITL，`SYS_HITL=0`，`simulator_mavlink` 接收模拟传感器 |
| PX4 模拟器 UDP | 127.0.0.1:15460 |
| JSBSim 桥接 UDP | 127.0.0.1:15461 |
| JSBSim bridge/server 内部 TCP | 127.0.0.1:15800 |
| 运行目录 | `build/px4_sitl_vector/run-voliro_t-1` |
| 启动输出 | 运行目录内 `px4.log`、`jsbsim.log` |
| 飞行日志 | 运行目录内 `rootfs/log`，文件名默认使用 UTC |

参数与日志全部保存在独立 rootfs。为调试日志问题，使用 `SDLOG_MODE=2` 从启动持续
记录到退出，`SDLOG_BACKEND=1` 写电脑文件。默认 `--log-profile 1`，需要 EKF 回放
等额外主题时可指定 `--log-profile 131`。停止实例后 ULog 会关闭并保留。

脚本不会访问实体飞控的串口或参数，也不会向其 IP 发送仿真数据。启动时检查端口
占用；停止时仅处理保存了 PID 与启动标识的本实例进程，不使用全局 `pkill`。
已运行的实例再次 start 会提示先停止。`--instance`、`--px4-port`、`--bridge-port`、
`--server-port` 和 `--runtime-dir` 可用于配置其他独立实例；这些选项在后续命令中
需要保持一致。

默认关闭 FlightGear 输出，避免与其他 JSBSim 实例争用可视化通道。
使用 `--flightgear` 可以启用原来的 FlightGear UDP 输出。

实现位于 PX4 仓库 `Tools/simulation/jsbsim/run_vector_sitl.py`，
JSBSim 仓库中的 `scripts/run_sitl.sh` 是转发入口，可通过 `PX4_ROOT` 指定 PX4 路径。
JSBSim 桥接器增加了可选环境变量：

* `JSBSIM_CONFIG_FILE`：bridge 与自动生成的 server 使用同一个临时配置。
* `JSBSIM_SERVER_PORT`：覆盖原来的固定 TCP 5800，默认行为仍是 5800。
* `JSBSIM_WRENCH_FEEDBACK=0`：关闭本实例固定 localhost:9871 的可选反馈。

这些修改已经应用并构建。供其他同版本工作区复用的源码差异保存在
`standalone-overrides.patch`，相对于 JSBSim 项目根目录应用；新入口脚本需有执行权限。

本机验证：Voliro T/LSZH 的启动、停止、端口释放、重新启动通过；姿态和角速度
持续更新且有限；PC ULog 文件正常写入。实例保持未解锁，尚未进行起飞和航线验证。
