# DockGuidance 相对运动交会优化

2026-10-08。对应 `src/modules/dock_guidance/` 当前实现。

## 生效条件

`DOCK_DYN_EN=1`、`DOCK_TERM_EN=1`、`DOCK_PRED_EN=1` 同时成立时，
实际静止和运动目标都走 `DockGuidanceRelative.cpp`。不要求外部输入目标速度，
也不再等待 `TargetMotion.valid()`、静止/运动分类或本机速度与目标速度匹配。
其他参数组合继续使用已有静态或目标跟随分支。参数默认值没有提高。

仍须有可信尺度：观测中的相对 NED 位置，或者已标定实际宽高且未裁剪的目标框。
纯角度不能确定交会距离。`DOCK_TGT_W/H`、相机延迟与安装角的标定仍然影响精度。

## 控制变化

1. **直接估计相对位置和相对变化率。** `RelativeMotion.hpp` 在曝光时刻融合尺度和
   LOS；以同一时刻的本机位置、速度补偿自身运动。新增本机速度历史，位置/速度
   EKF reset、标定变更和退出模式时清理对应状态。更新最多约 11 Hz；至少 4 个
   接受的观测、跨度 0.3 s、最新接受观测年龄不超过 0.25 s 后允许交会规划。
   采用三维位置/速率协方差、5 sigma 创新门控，目标尺寸系统误差保留为不确定度下限。
2. **爬升和前向接近共同规划。** `RelativeGuidance.hpp` 推演真实使用的速度、
   加速度和 jerk 限制，以及 `DOCK_PRED_TAU` 表示的飞机速度响应。
   log_94 修正后增加本机响应偏差、由推力/加速度辨识的有效阻力及总推力约束，
   export35 修正后同时辨识线性与二次阻力，避免把线性阻力按速度平方外推，
   按位置控制器的水平推力预留量计算未来可实现的加速度。搜索三维速度候选，
   先满足视场约束；预测误差落入设计容差后，优先选择更早到达的候选。
   没有单独的目标高度停车包络，也没有“先在启动高度稳定再加速”的阶段。
   `DOCK_START_H` 仅在相对运动尚未可用时保留启动爬升参考，之后交给目标交会轨迹。
   设计容差为 `min(0.25 m, 0.25 * DOCK_PASS_ERR)`，不代表真实飞行保证。
3. **加速连续。** 规划间隔最短 50 ms，实际每帧重新执行限速、加速度、jerk 和 FOV
   governor；保留当前速度候选给转向分配余量。初始速率建立期间不会把已有高速
   突然指定为 `DOCK_CAP_VMAX`。绝对目标速度估计变为有效不会重入 ALIGN。
4. **视场保护保持在线。** 相机俯仰和 yaw 跟踪保留，图像平移纠偏作为规划候选。
   目标框变大不再单独触发减速恢复；框裁剪仍拒绝尺度更新。目标中心进入软边界
   时抑制继续加速，持续硬边界保留恢复与机头保护。长于 0.5 s 的尺度丢失退出
   已建立的相对引导；普通视觉丢失继续走既有有时限的退化处理。
5. **盲穿使用独立证据。** 可转向的轨迹预测只用于制导。末段许可另以当前已经发布
   的速度做冻结预测，使用同一受推力限制的响应模型，检查最近距离、协方差、
   到达时间和越过平面的余量；把本机响应残差乘以剩余时间加入误差界。
   本机响应数据超过 150 ms 未更新时不允许新提交盲穿。
   观测年龄、穿越后清空距离和 30 ms 裕量计入 `DOCK_TERM_BLIND`，并受
   `DOCK_TERM_MAX_T`、`DOCK_BLIND_TGO`、确认时间和视场限制。
   许可后冻结相对观测模型和速度，每个看门狗周期用新鲜本机状态重验穿越平面。
   通过后进入已有 `PASS_PREDICTED` 制动与退出链；超时/超出误差走故障处理。
6. **修正近距预测边界。** `PredictiveCorrection.hpp` 现在能找到首个预测采样间隔
   内的最近点，例如 40 m/s、剩余 1 m 的 25 ms 交会。预测窗口终点仍不能直接
   充当交会证据。

这是固定数量候选的滚动预测搜索，不是全局最短时间的数学保证。速率仍来自观测；
无需输入绝对目标速度不等于在任意测距误差、目标机动或初始几何下都能命中。

## 新增日志

`dock_guidance_relative` 默认以 20 ms 日志间隔记录：

| 字段 | 含义 |
|---|---|
| `active` / `rate_ready` | 相对引导分支启用 / 相对观测时效与采样条件满足 |
| `relative_position` / `relative_rate` | 当前 NED 相对位置 / 相对变化率 |
| `trajectory_miss` / `trajectory_tgo` | 允许未来调整速度时的候选轨迹结果 |
| `frozen_miss_bound` / `frozen_time_upper` | 当前发布速度的冻结预测误差界和到达时间上界 |
| `observation_age` / `plan_age` | 接受图像的年龄 / 最近一次完整搜索的年龄 |
| `planner_runtime_us` | 飞控上实测的最近一次完整搜索耗时 |
| `terminal_committed` | 已进入冻结末段 |
| `response_valid` / `response_bias` | 本机响应数据新鲜 / 实测速度响应的等效偏差，m/s |
| `drag_coefficient` | 各 NED 轴有效二次阻力系数，1/m；局部辨识结果 |
| `linear_drag` | 各 NED 轴有效线性阻力系数，1/s；与二次项共同预测阻力 |
| `response_uncertainty` | 本机响应残差，m/s，用于扩大末段误差界 |
| `thrust_acceleration` | 当前悬停推力折算的最大推力加速度，m/s²，含重力补偿需求 |

旧 `target_motion_valid`、目标速度和估计器协方差仍表示旧绝对目标估计器，
不能再作为新分支是否允许加速的判断依据。`trajectory_valid` 有效期最多 100 ms，
退化、失效与冻结末段不把旧轨迹继续报告为新的制导预测。

## 验证与复现

新增独立模型回归：

```sh
g++ -std=c++14 -Wall -Wextra -Wshadow -Werror -O2 \
  -D__PX4_POSIX -D__PX4_LINUX \
  -Iplatforms/common/include -Iplatforms/posix/include -Ibuild/cuav_7-nano_test \
  -Isrc/lib -Isrc/lib/matrix \
  src/modules/dock_guidance/test_relative_guidance.cpp -o /tmp/test_relative_guidance
/tmp/test_relative_guidance
ninja -C build/cuav_7-nano_test
```

测试采用 50 Hz 指令更新、最短 50 ms 搜索间隔、100 ms 图像延迟和独立 1 kHz
一阶飞机响应模型；不使用轨迹推演公式生成真值。主要用例初始水平距离 150 m、
目标高 100 m，总速/水平上限 50 m/s，垂向速度上限 15 m/s，总加速度 8 m/s²、
上向加速度上限 8 m/s²、下向 3 m/s²、jerk 8 m/s³，响应 0.5 s。
这些是测试配置，不是写入飞控的参数。

| 模型用例 | 最近距离 | 到最近点时间 | 该时刻飞机速度 |
|---|---:|---:|---:|
| 静止目标 | 0.142 m | 9.334 s | 27.055 m/s |
| 同向 10 m/s | 0.173 m | 9.985 s | 42.532 m/s |
| 迎面 10 m/s | 0.236 m | 9.082 s | 16.049 m/s |
| 横移 5 m/s | 0.170 m | 9.378 s | 28.046 m/s |
| NED 速度 (5, 3, -2) m/s | 0.205 m | 10.903 s | 30.685 m/s |

另覆盖响应 0.2/0.7 s、目标在下方、已有高速入模、目标中途停止、过期/乱序/
异常观测、FOV 候选、命令约束，以及冻结末段的误差和时限。
带 15% 尺度不确定度、1% 正弦距离噪声和 5 cm 横向/垂向噪声的静止用例，
最近距离 0.112 m，完成末段锁定及预测穿越。
来不及修正的近距高度差用例误差约 8.16 m，必须拒绝盲穿，而不能伪报通过。

精度/时间模型用例将 FOV 评分阈值放宽，用来隔离三维交会与飞机响应；
FOV 边界有独立单步测试。这些结果不构成完整相机/姿态/控制器闭环的 HIL 验证。
已运行既有 `test_target_motion`、`test_predictive_alignment` 回归，以及新测试的
ASan/UBSan。CUAV 完整构建产物为 `build/cuav_7-nano_test/cuav_7-nano_test.px4`。
上述一阶模型没有模拟高速度下的推力饱和。用户随后提供的 log_94 / export34
显示静止目标仍存在明显垂向偏差，诊断及本机响应模型修正在
[DockGuidance_log94.md](DockGuidance_log94.md)。后续 export35 的剩余偏差及线性阻力修正见
[DockGuidance_export35.md](DockGuidance_export35.md)。最新修正版尚未刷机或进行新一轮 HIL / 实飞；
实际精度、整段时间改善和飞控计算耗时需要新日志确认。

本次修改保留了工作树原有的板级参数、`DockGuidance_dynamic.md`、
`DockGuidancePrediction.msg` 和未接入的 `InterceptGuidance` 草稿。

## 本次文件清单

| 路径 | 修改内容 |
|---|---|
| `src/modules/dock_guidance/RelativeMotion.hpp` | 新增相对运动滤波 |
| `src/modules/dock_guidance/RelativeGuidance.hpp` | 新增轨迹推演、候选搜索及冻结末段证据 |
| `src/modules/dock_guidance/RelativeResponse.hpp` | 本机响应及有效线性/二次阻力辨识 |
| `src/modules/dock_guidance/DockGuidanceRelative.cpp` | 新增状态机控制和相对诊断发布 |
| `src/modules/dock_guidance/DockGuidance.cpp` | 接入控制分支、速度历史、EKF reset 和独立末段看门狗 |
| `src/modules/dock_guidance/DockGuidance.hpp` | 新增状态、历史、方法和发布者 |
| `src/modules/dock_guidance/DockGuidanceDynamic.cpp` | 曝光时刻位置/速度插值与相对滤波输入 |
| `src/modules/dock_guidance/TargetMotion.hpp` | 为轨迹推演复用速度 governor，保留在线 FOV 检查默认行为 |
| `src/modules/dock_guidance/PredictiveCorrection.hpp` | 修复首个采样间隔内的最近点搜索 |
| `src/modules/dock_guidance/test_relative_guidance.cpp` | 新增回归与独立响应模型 |
| `src/modules/dock_guidance/test_relative_response.cpp` | 实际 PositionControl 与独立推力/阻力模型回归 |
| `src/modules/dock_guidance/CMakeLists.txt` | 编译新控制分支 |
| `src/modules/dock_guidance/module.yaml` | 更新已有参数的适用条件和说明 |
| `msg/DockGuidanceRelative.msg` | 新增诊断话题 |
| `msg/CMakeLists.txt` | 注册新话题 |
| `src/modules/logger/logged_topics.cpp` | 记录新诊断话题 |
| `src/modules/logger/messages.h`、`logger.h`、`logger.cpp` | 修复扩展消息格式溢出，格式缓冲区改为成员存储 |
| `src/modules/logger/test_format_capacity.cpp` | 验证当前构建的全部压缩消息格式均可展开并容纳名称 |
| `docs/DockGuidance_relative.md` | 本说明及验证方法 |
| `docs/DockGuidance_log94.md` | 新日志诊断、修正依据和受限动力模型验证 |
