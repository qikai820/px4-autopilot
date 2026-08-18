# MorphQuad 推力分配算法设计报告

> 代码实现基准：`ActuatorEffectivenessMorphQuad`。理论参考：*MorphQuad: Morphable Quadrotor for Superhuman Maneuverability, Manipulation, and Resiliency*，arXiv:2607.02764v3。

## 1. 问题定义与约束

给定 `geo_control` 发布的 Body-FRD 期望 wrench：

$$
\mathbf w_d=
\begin{bmatrix}
\mathbf f_d\\\boldsymbol\tau_d
\end{bmatrix}
\in\mathbb R^6,
$$

分配器求解四个旋翼系统的三维推力向量 `t_i`，再转化为四个归一化电机转速和八个舵机指令。

实现约束如下：

- Body-FRD：x 向前、y 向右、z 向下；悬停推力为负 z；
- 旋翼顺序为 RF、LF、LB、RB，位置分别为
  `(+L,+B,Z)`、`(+L,-B,Z)`、`(-L,-B,Z)`、`(-L,+B,Z)`；
- 当前 JSBSim 机型参数：`L=0.25 m`、`B=0.225 m`、`Z=-0.04 m`；
- 每个旋翼系统最大分配推力为 `Tmax=CA_MQ_TMAX`；
- 实际 α、β 舵机的机械范围为 ±180°，不是论文原型的 ±1080°。

## 2. 推力方向和执行器模型

在 PX4/JSBSim 的 FRD 坐标约定中，两个舵机角定义的单位推力方向是

$$
\mathbf g(\alpha,\beta)=
\begin{bmatrix}
\sin\alpha\cos\beta\\
\sin\beta\\
-\cos\alpha\cos\beta
\end{bmatrix}.
$$

中性位 `alpha=beta=0` 对应向上推力。`alpha=±π, beta=0` 对应向下推力，故正、负 z 力均能通过反转万向节方向实现。

分配器的电机模型为

$$
T_i=T_{max}u_{m,i}^2,\qquad
\mathbf t_i=T_i\mathbf g(\alpha_i,\beta_i),\qquad
u_{m,i}=\sqrt{T_i/T_{max}}.
$$

JSBSim 使用电机和螺旋桨动态模型，物理推力不严格等于该二次模型；这属于模型标定误差，不改变推力方向与 wrench 几何。

## 3. 12 维推力向量分配

定义堆叠向量

$$
\mathbf t=[\mathbf t_0^\top,\mathbf t_1^\top,\mathbf t_2^\top,\mathbf t_3^\top]^\top\in\mathbb R^{12}.
$$

总力和总力矩为

$$
\mathbf f=\sum_{i=0}^3\mathbf t_i,\qquad
\boldsymbol\tau=\sum_{i=0}^3\mathbf r_i\times\mathbf t_i.
$$

因此

$$\mathbf w=\mathbf M\mathbf t,$$

其中 `M` 是由 `r_i` 构成的常数 6×12 输入矩阵。对每个旋翼，力矩项完整保留高度项：

$$
\mathbf r_i\times\mathbf t_i=
\begin{bmatrix}
y_i t_{z,i}-Zt_{y,i}\\
Zt_{x,i}-x_i t_{z,i}\\
x_i t_{y,i}-y_i t_{x,i}
\end{bmatrix}.
$$

这对应论文的推力向量形式。早期仅使用 `r_i=(x_i,y_i,0)` 会漏掉水平推力带来的滚转/俯仰耦合，不能匹配当前 JSBSim 几何。

### 3.1 最小范数解

代码调用 `matrix::geninv()` 求

$$\mathbf t_0=\mathbf M^+\mathbf w_d.$$

该 12 维解先满足线性 wrench 方程；尚未施加单旋翼幅值和舵机机械约束。

### 3.2 论文零空间修正

实现使用论文 Eq. (19) 的显式 12×6 零空间基 `N_M`，而不是临时的经验性 x/y 分量修改：

$$\mathbf t=\mathbf t_0+\mathbf N_M\mathbf c.$$

代码中的 Eq. (16) 系数规则为：

$$
c_1=c_3=c_5=\hat f_y,
$$

当 `hat(f)_x > 0` 时：

$$c_2=-\hat f_x,\quad c_4=\hat f_x,\quad c_6=0,$$

否则：

$$c_2=c_4=0,\quad c_6=-\hat f_x.$$

其中 `hat(f)=f/||f||`；当 `||f||` 很小时，`c=0`。该修正用于规避 β 轴奇异位置和沿 x 轴的共线下洗。运行时检查 `M N_M≈0`，以防基矩阵与机体几何不一致。

### 3.3 推力幅值约束

对每个 `t_i` 计算范数。如果最大范数超过 `Tmax`，所有 12 个分量使用同一缩放因子：

$$s=\min_i\left(1,\frac{T_{max}}{\|\mathbf t_i\|}\right),\qquad \mathbf t\leftarrow s\mathbf t.$$

统一缩放保持每个推力向量的方向；逐轴裁剪会破坏方向并引入未建模的耦合。饱和产生的达到 wrench 在 `computeUnallocatedWrench()` 中重构并回报给 PX4。

## 4. 逆运动学和限位

对非零 `t_i`，令 `hat(t_i)=[tx,ty,tz]^T`。两组等价解为

$$
\beta_1=\arcsin(ty),\qquad \alpha_1=\operatorname{atan2}(tx,-tz),
$$

$$
\beta_2=\pi-\beta_1,\qquad \alpha_2=\pi+\alpha_1.
$$

算法枚举两个分支及有限个 `2π` 等价角，最小化

$$|\alpha-\alpha_{prev}|+|\beta-\beta_{prev}|.$$

接着将两个角限制为

$$
|\alpha|\leq\min(CA\_MQ\_ALPM,180°),\qquad
|\beta|\leq\min(CA\_MQ\_BETM,180°).
$$

零推力时保持上一组角度；若限位造成方向无法实现，则该差异记入 servo saturation 和 unallocated wrench。当前硬件只支持 ±180°，因此角度的 `2π` 枚举仅用于等价分支选择，不表示已经支持连续多圈旋转。

归一化输出为

$$u_\alpha=\alpha/\pi,\qquad u_\beta=\beta/\pi.$$

## 5. JSBSim 一致性要求

桥接配置 `configs/morphquad.xml` 必须将八路舵机输出以 `scale=π` 转为弧度。JSBSim 的 `GimbalVectoring.xml` 需要把推力方向转换为推进器欧拉角：

$$
pitch=\operatorname{atan2}(\cos\alpha\cos\beta,\sqrt{t_x^2+t_y^2}),
\qquad yaw=\operatorname{atan2}(t_y,t_x).
$$

这里 `pitch` 的第一项等于 `-t_z`，不能替换为始终非负的 `sqrt(1-t_x^2-t_y^2)`。后者会丢失 z 方向符号，使 `alpha=±π` 仍然产生向上推力。

## 6. 端到端验证算法

`geo_control verify` 在 JSBSim `<hold_down>1</hold_down>` 下验证整条分配链。测试集合为 6 个自由度各 10 个递增幅值，共 60 组。每一组执行：

```text
设置 hover wrench
等待 JSBSim 稳态，记录 W_hover
持续发布目标 wrench W_test
读取电机与舵机输出，按同一模型前向重构 W_alloc
等待 JSBSim 稳态，记录 W_jsb
比较 W_alloc - W_hover 与 W_jsb - W_hover
回到 hover，进入下一点
```

持续发布是必要的：`verify` 是同步 PXH 命令，保持架模式可能不触发本地位置更新，不能依赖普通控制循环发布测试 wrench。

验证分为两层：

1. **前向重构**：检测分配器、归一化、索引和几何是否一致；未饱和点应为零请求残差。
2. **JSBSim 物理增量**：检测 bridge、万向节方向、推进器模型和实际力矩符号；以悬停基线相减，隔离绝对推力曲线误差。

已完成的 60 组保持架复测确认：`Fz=+10.27 N` 的 JSBSim 结果从修复前约 `-12.01 N` 变为约 `+12.01 N`。这验证了 z 符号映射修复；幅值差异仍需要推力曲线标定。

### 6.1 自动姿态闭环测试

`geo_control att_verify` 用于验证几何姿态环，而非控制分配的静态正确性。它只允许在已解锁的 Acro 模式下启动；由于需要真实姿态变化，JSBSim 必须关闭 `hold_down`，且 `SIM_MQ_BENCH` 必须为 0。

测试序列为：`roll +10°`、`roll -10°`、`pitch +10°`、`pitch -10°`、`yaw +15°`、`yaw -15°`、水平回中。每个序列点保持 4 秒，2 秒后开始累积：

$$
\bar e=\frac{1}{N}\sum_{k=1}^{N}|e_{RPY,k}|,\qquad
\bar\omega=\frac{1}{N}\sum_{k=1}^{N}\|\omega_k\|.
$$

默认通过条件是三个分量的 `bar(e)` 均不大于 3°，且 `bar(omega)` 不大于 0.35 rad/s。所有点结束后，控制器保持水平悬停，等待操作员停止测试并执行安全退出。

## 7. 算法伪代码

```text
input: f_des, tau_des, previous servo angles
M = build_wrench_matrix(L, B, Z)
t = pinv(M) * [f_des; tau_des]
t += N_M * paper_eq16_coefficient(f_des)

if any norm(t_i) > Tmax:
    t *= common_scale_to_tmax(t)

for each rotor i:
    if norm(t_i) is near zero:
        alpha, beta = previous_angles[i]
        motor = 0
    else:
        alpha, beta = nearest_valid_inverse_kinematics(t_i, previous_angles[i])
        motor = sqrt(norm(t_i) / Tmax)

    write motor[i], alpha/pi, beta/pi

reconstruct achieved wrench if a thrust, servo, or actuator limit was active
```

## 8. 边界与后续工作

- `CA_MQ_TMAX` 是分配近似模型的标定值；应补充覆盖低/高转速和反向推力的推力曲线。
- 每个旋翼目前共享相同的 `Z` 值；实际结构若不共面，应使用逐旋翼位置参数。
- 保持架验证确认分配和模型方向，不等价于自由飞行稳定性测试。
- 若换用多圈舵机，需一并更改 PX4 输出协议、`CA_MQ_ALPM/BETM` 上限、bridge 缩放和连续角状态管理；不能仅提高参数上限。

## 9. 版本记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v0.7 | 2026-08-18 | 以当前分配器和最终保持架验证结果重写；明确 `CA_MQ_Z`、±180°限位、z 符号和标定限制 |
| v0.8 | 2026-08-18 | 新增 `att_verify` 姿态闭环自动阶跃与评分算法 |
