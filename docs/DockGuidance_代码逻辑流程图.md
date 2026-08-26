# Dock Guidance 代码逻辑流程图

本文反映当前 `src/modules/dock_guidance` 的实际逻辑。模块由 `viztra_observation` 的新视觉帧触发，输出独立的 `dock_guidance_setpoint`；当前版本**不直接发布** `vehicle_rates_setpoint`，也尚未接管飞行器控制权。

```mermaid
flowchart TD
    A[视觉板] --> B[viztra 驱动]
    B --> C[viztra_observation]
    C --> D{DockGuidance::Run\n收到新视觉帧?}
    D -->|否| Z[等待下一帧]
    D -->|是| E[更新参数\n计算本帧 dt]
    E --> F[读取 vehicle_attitude\nvehicle_angular_velocity\nvehicle_local_position]
    F --> G[写入姿态/角速度\n延时历史环形缓冲]
    G --> H{观测有效且偏移为有限值?}

    H -->|否| I[清空 LOS、滤波器和阶段状态]
    I --> J[发布 dock_guidance_setpoint\nvalid = false, stage = NONE]

    H -->|是| K{视觉偏移是否已是角度?}
    K -->|是| L[直接使用 offset_x / offset_y]
    K -->|否| M[按图像分辨率与 FOV\n像素偏移转换为 yaw / pitch]
    L --> N[从延时缓冲取姿态与角速度]
    M --> N

    N --> O[构造相机系 LOS 单位向量]
    O --> P[相机系 -> 机体系 -> NED\n得到地理系 LOS]
    P --> Q[一阶低通滤波]
    Q --> R[保存 7 个样本\n首尾差分估计 LOS 变化率]
    R --> S[LOS 角速度由地理系\n转换回相机系]
    S --> T[计算相机在 NED 中的姿态]

    T --> U{相机俯仰误差 > 60°?}
    U -->|是| U1[阶段 1：PITCH_CAPTURE\n只生成俯仰捕获指令]
    U -->|否| V{目标是否超出\nDOCK_VIS_LIM?}
    V -->|是| V1[阶段 2：ANGLE_CAPTURE\n滚转水平 + 俯仰 + 偏航捕获]
    V -->|否| W[阶段 3：LOS_RATE\nLOS 角速度抑制 + 前进俯仰\n+ 残余角度修正]

    U1 --> X[视场边界方向保护]
    V1 --> X
    W --> X
    X --> Y[相机系角速度指令\n转换为机体系]
    Y --> AA[发布 dock_guidance_setpoint\nvalid = true\nroll/pitch/yaw rate + throttle]
```

## 三阶段控制目的

| 阶段 | 进入条件 | 控制目标 |
|---|---|---|
| `PITCH_CAPTURE` | 相机俯仰误差大于 60° | 优先改变俯仰，将目标从相机上下边界拉回。 |
| `ANGLE_CAPTURE` | 目标仍在 `DOCK_VIS_LIM` 可视中心范围外 | 使用滚转水平、俯仰与偏航，将目标移动到画面中心区域。 |
| `LOS_RATE` | 目标位于可视中心范围内 | 通过 LOS 变化率降低视线漂移，并叠加前进俯仰和残余角度修正，形成跟踪/对接引导。 |

## 当前输出边界

```mermaid
flowchart LR
    A[DockGuidance] --> B[dock_guidance_setpoint]
    B -. 当前仅供监视/后续仲裁 .-> C[尚未接入 vehicle_rates_setpoint]
    D[mc_pos_control / mc_att_control] --> E[vehicle_rates_setpoint]
    E --> F[mc_rate_control]
    F --> G[控制分配器 / 电机]
```

后续增加 Dock 飞行模式时，应仅在 Dock 模式且视觉目标有效时，将 `dock_guidance_setpoint` 经仲裁接入 `vehicle_rates_setpoint`；退出模式或视觉失效时，恢复原有位置控制链。
