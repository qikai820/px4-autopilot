/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessMorphQuad.cpp
 *
 * MorphQuad 可变构型四旋翼的推力分配（控制分配）实现。
 *
 * 算法来源于论文：
 *   "MorphQuad: Morphable Quadrotor for Superhuman Maneuverability"
 *   (arXiv:2607.02764)
 *
 * 采用 Pattern B（模式 B）实现方式：在 getEffectivenessMatrix() 中仅注册作动器
 * （4 个电机 + 8 个舵机），注册一个"占位"的零效力矩阵；真正的分配逻辑全部在
 * updateSetpoint() 中完成。这样框架会把控制集（control setpoint）原样交给我们，
 * 由本类自行把 6 维期望广义力（力 + 力矩）映射为各电机油门与各舵机角度。
 */

#include "ActuatorEffectivenessMorphQuad.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

#include <cmath>

using namespace matrix;

ActuatorEffectivenessMorphQuad::ActuatorEffectivenessMorphQuad(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool ActuatorEffectivenessMorphQuad::getEffectivenessMatrix(
	Configuration &configuration, EffectivenessUpdateReason external_update)
{
	// 注册 4 个电机。
	// 注意：此处填入的是"占位"推力方向（默认沿 -z 轴），因为真正的推力方向由云台角在
	// updateSetpoint() 中动态决定；框架仅需知道作动器的数量和类型即可。
	for (int i = 0; i < NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{0.f, 0.f, -1.f});
	}

	// 注册 8 个舵机，顺序为：4 个外环 alpha（绕偏航）+ 4 个内环 beta（绕俯仰）。
	// 在 actuator_sp 向量中，电机占据索引 0..3，舵机占据索引 4..11
	// （每两个一组：4+2*i 为第 i 个旋翼的 alpha，5+2*i 为 beta）。
	for (int i = 0; i < 2 * NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessMorphQuad::updateSetpoint(
	const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
	ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min,
	const ActuatorVector &actuator_max)
{
	// 从控制集（control setpoint）中提取期望的广义力。
	// control_sp 的 6 个分量顺序为：
	//   [滚转力矩, 俯仰力矩, 偏航力矩, 推力_x, 推力_y, 推力_z]
	// 其中前三项是力矩（N·m），后三项是力（N）。
	Vector3f f_des(control_sp(ControlAxis::THRUST_X),
		       control_sp(ControlAxis::THRUST_Y),
		       control_sp(ControlAxis::THRUST_Z));
	Vector3f tau_des(control_sp(ControlAxis::ROLL),
			 control_sp(ControlAxis::PITCH),
			 control_sp(ControlAxis::YAW));

	// 步骤 1+2：计算各旋翼推力矢量（最小范数解 + 零空间冗余度修正 + 云台角重定向）。
	// 得到的 t[i] 是机体坐标系下第 i 个旋翼的三轴推力分量。
	float t[NUM_ROTORS][3];
	computeThrustVectors(f_des, tau_des, t);

	// 步骤 3：由推力矢量反解舵机角度，并由推力大小换算电机转速。
	float omega_raw[NUM_ROTORS];   // 归一化转速（未钳位）
	float alpha_raw[NUM_ROTORS];   // 外环角（未钳位，已除以 π）
	float beta_raw[NUM_ROTORS];    // 内环角（未钳位，已除以 π）

	const float tmax = _param_ca_mq_tmax.get();   // 满油门单旋翼最大推力 TMAX

	for (int i = 0; i < NUM_ROTORS; ++i) {
		float alpha, beta;
		recoverServoAngles(t[i], i, alpha, beta);

		// 转子物理模型：T_i = c_T * Ω_i^2  =>  Ω_i = sqrt(T_i / c_T)。
		// JSBSim morphquad 模型接收的 esc 指令为归一化转速 esc = Ω / Ω_max，
		// 其中 esc=1.0 对应单旋翼最大推力 TMAX。由于 TMAX = c_T * Ω_max^2，
		// 因此归一化电机指令应为 sqrt(T_i / TMAX)，而【不是】线性的 T_i/TMAX。
		// 若错误地使用 T_i/TMAX（历史上的一个 bug），相当于把转子线性化，
		// 会在悬停与满油门附近产生错误的（非线性）电机响应。
		const float t_norm = sqrtf(t[i][0] * t[i][0] + t[i][1] * t[i][1] + t[i][2] * t[i][2]);
		omega_raw[i] = (t_norm < 0.01f) ? 0.f : sqrtf(t_norm / tmax);
		alpha_raw[i] = alpha / M_PI_F;   // 角度换算到 [-1,1] 比例的指令区间
		beta_raw[i] = beta / M_PI_F;
	}

	// 写入原始（钳位前）指令值，同时检测并记录是否发生饱和。
	_motor_saturated = false;
	_servo_saturated = false;

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// 电机饱和检查：把归一化转速钳制到作动器允许范围。
		const float motor_clamped = math::constrain(omega_raw[i], actuator_min(i), actuator_max(i));

		if (fabsf(motor_clamped - omega_raw[i]) > FLT_EPSILON) {
			_motor_saturated = true;   // 实际值与钳位值不一致 => 该电机已饱和
		}

		actuator_sp(i) = motor_clamped;

		// 外环舵机（alpha）：每个电机交错存放于 actuator_sp[4 + 2*i]。
		const int alpha_idx = 4 + 2 * i;
		const float alpha_clamped = math::constrain(alpha_raw[i], actuator_min(alpha_idx), actuator_max(alpha_idx));

		if (fabsf(alpha_clamped - alpha_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(alpha_idx) = alpha_clamped;

		// 内环舵机（beta）：每个电机交错存放于 actuator_sp[5 + 2*i]。
		const int beta_idx = 5 + 2 * i;
		const float beta_clamped = math::constrain(beta_raw[i], actuator_min(beta_idx), actuator_max(beta_idx));

		if (fabsf(beta_clamped - beta_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(beta_idx) = beta_clamped;
	}

	// 根据饱和情况计算未能实现的广义力。
	// 一旦某个作动器饱和，实际输出就会偏离期望输出，由此求出残余力/力矩。
	computeUnallocatedWrench(f_des, tau_des, omega_raw, alpha_raw, beta_raw,
				 actuator_sp, actuator_min, actuator_max);
}

void ActuatorEffectivenessMorphQuad::computeThrustVectors(
	const Vector3f &f_des, const Vector3f &tau_des,
	float t_out[NUM_ROTORS][3])
{
	const float l = _param_ca_mq_l.get();   // 机臂半长
	const float b = _param_ca_mq_b.get();   // 机臂半宽
	const float tmax = _param_ca_mq_tmax.get();   // 满油门单旋翼最大推力
	// 归一化尺度：把力/力矩误差分别按 tmax、tmax*l 缩放，使 6 维误差在量纲上一致，
	// 便于在角度迭代中使用统一步长。
	const float f_scale = math::max(tmax, 1.f);          // 力的归一化尺度
	const float t_scale = math::max(tmax * l, 0.01f);    // 力矩的归一化尺度

	// 旋翼在机体 X 布局下的臂位置（从右前 RF 起，逆时针 CCW 依次为 RF、LF、LB、RB）：
	//   lx 为 X 方向偏移，ly 为 Y 方向偏移。
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = { b, -b, -b,  b};

	// 装配期望广义力向量 W = [力; 力矩]，共 6 维，机体坐标系，单位 N / N·m。
	matrix::Vector<float, 6> W;
	W(0) = f_des(0); W(1) = f_des(1); W(2) = f_des(2);
	W(3) = tau_des(0); W(4) = tau_des(1); W(5) = tau_des(2);

	const float k_ang = _param_ca_mq_angle_gain.get();   // 角度重定向步长系数
	const float w_pow = _param_ca_mq_ns_pow.get();       // 零空间电机功率代价权重
	const float w_dw  = _param_ca_mq_dwg.get();          // 零空间下洗代价权重
	const float k_ns  = _param_ca_mq_ns_gain.get();      // 零空间下降增益
	const float a_max = math::radians(_param_ca_mq_alpm.get());   // 外环角机械限位（弧度）
	const float b_max = math::radians(_param_ca_mq_betm.get());   // 内环角机械限位（弧度）

	// ─────────────────────────────────────────────────────────────────────────
	// 迭代"重定向"云台角，使实现的广义力 W_ach 跟踪期望 W。
	//
	// 若只在固定云台角的 A_g(q) 下做推力大小的最小范数解，只能缩放 |T_i|，
	// 永远无法让推力矢量倾斜，因此侧向力/离轴力矩将无法实现，舵机也永远不会转动。
	// 为此交替进行两步：
	//   1) 在当前角度下求解最小范数推力大小 u = A_g⁺ W；
	//   2) 对 (α_i,β_i) 做一步高斯-牛顿（含 Levenberg–Marquardt 阻尼）修正，
	//      重定向推力矢量、最小化归一化后的广义力误差；
	// 重复若干次。由于本函数每个控制周期都会执行，角度会逐步收敛到能实现完整
	// 6 自由度广义力的构型。这正是 MorphQuad 全驱动分配的核心思想。
	// ─────────────────────────────────────────────────────────────────────────
	// 保存最后一次有效迭代的结果，用于最终的推力矢量重建。
	matrix::Matrix<float, 6, NUM_ROTORS> A_g_final{};
	matrix::Matrix<float, NUM_ROTORS, 6> A_pinv_final{};
	matrix::Vector<float, NUM_ROTORS> u0_final{};

	const int max_iter = 6;   // 每个控制周期最多迭代次数（warm-start 下足够收敛）

	for (int iter = 0; iter < max_iter; ++iter) {
		// 用当前（以上一周期为热启动的）云台角构建分配矩阵 A_g。
		// 单个旋翼：力 f_i = T_i g_i，力矩 τ_i = r_i × f_i，合并得 W = A_g T。
		// 其中推力方向单位矢量 g_i = R_x(α)R_y(β)e3，与 recoverServoAngles() 的正运动学一致。
		matrix::Matrix<float, 6, NUM_ROTORS> A_g;

		for (int i = 0; i < NUM_ROTORS; ++i) {
			const float sa = sinf(_alpha_prev[i]);
			const float ca = cosf(_alpha_prev[i]);
			const float sb = sinf(_beta_prev[i]);
			const float cb = cosf(_beta_prev[i]);
			const Vector3f g(sa * cb, sb, -ca * cb);   // 推力方向单位矢量
			A_g(0, i) = g(0); A_g(1, i) = g(1); A_g(2, i) = g(2);   // 力的三轴分量
			const Vector3f r_i(lx[i], ly[i], 0.f);   // 该旋翼的臂位置
			const Vector3f r_cross_g = r_i.cross(g);   // r × g
			A_g(3, i) = r_cross_g(0); A_g(4, i) = r_cross_g(1); A_g(5, i) = r_cross_g(2);   // 力矩三轴分量
		}

		matrix::Matrix<float, NUM_ROTORS, 6> A_pinv;

		// 若分配矩阵奇异（不可逆），无法求解，将所有推力矢量清零后直接返回。
		if (!matrix::geninv(A_g, A_pinv)) {
			for (int i = 0; i < NUM_ROTORS; ++i) { t_out[i][0] = t_out[i][1] = t_out[i][2] = 0.f; }

			return;
		}

		const matrix::Vector<float, NUM_ROTORS> u0 = A_pinv * W;   // 最小范数推力大小
		const matrix::Vector<float, 6> W_ach = A_g * u0;           // 当前角度下能实现的广义力
		const matrix::Vector<float, 6> e = W - W_ach;              // 广义力误差 = 期望 - 实现

		// 将误差做无量纲处理，使角度步长量纲一致（力误差除以 f_scale，力矩误差除以 t_scale）。
		matrix::Vector<float, 6> e_n{};
		e_n(0) = e(0) / f_scale; e_n(1) = e(1) / f_scale; e_n(2) = e(2) / f_scale;
		e_n(3) = e(3) / t_scale; e_n(4) = e(4) / t_scale; e_n(5) = e(5) / t_scale;

		// ── 基于归一化残差的高斯-牛顿角度更新 ──
		// 在当前角度附近对实现广义力做线性化：
		//   W_ach(θ+Δθ) ≈ W_ach(θ) + J_ang·Δθ,   J_ang = ∂W_ach/∂θ  （6 × 2N 雅可比）
		// 并对 Δθ 最小化 ½‖e_n‖²（e_n 为归一化后的 W − W_ach）：
		//   Δθ = (J_angᵀ J_ang + λI)⁻¹ J_angᵀ e_n      （Levenberg–Marquardt 形式）
		// 该步一次性、协调地重定向所有推力矢量以跟踪期望广义力，取代了以往逐角独立的梯度更新。
		// 仅当该旋翼在产生推力（T_i>0）时才重定向其推力，避免无推力旋翼的角度无意义漂移。
		const int n_ang = 2 * NUM_ROTORS;   // 角度变量总数：每个旋翼 2 个（α,β）
		matrix::Matrix<float, 6, n_ang> J_ang{};

		for (int i = 0; i < NUM_ROTORS; ++i) {
			const float T_i = math::max(u0(i), 0.f);   // 该旋翼当前推力大小（截断负值）
			const float sa = sinf(_alpha_prev[i]);
			const float ca = cosf(_alpha_prev[i]);
			const float sb = sinf(_beta_prev[i]);
			const float cb = cosf(_beta_prev[i]);

			// 推力方向 g 对 α、β 的偏导
			const Vector3f dg_da(ca * cb, 0.f, sa * cb);
			const Vector3f dg_db(-sa * sb, cb, ca * sb);
			const Vector3f r_i(lx[i], ly[i], 0.f);

			// 第 2i 列     = ∂W_ach/∂α_i，第 2i+1 列 = ∂W_ach/∂β_i（均已按 f/t 归一化）。
			J_ang(0, 2 * i) = T_i * dg_da(0) / f_scale; J_ang(1, 2 * i) = T_i * dg_da(1) / f_scale; J_ang(2, 2 * i) = T_i * dg_da(2) / f_scale;
			const Vector3f rxd = r_i.cross(dg_da);
			J_ang(3, 2 * i) = T_i * rxd(0) / t_scale; J_ang(4, 2 * i) = T_i * rxd(1) / t_scale; J_ang(5, 2 * i) = T_i * rxd(2) / t_scale;

			J_ang(0, 2 * i + 1) = T_i * dg_db(0) / f_scale; J_ang(1, 2 * i + 1) = T_i * dg_db(1) / f_scale; J_ang(2, 2 * i + 1) = T_i * dg_db(2) / f_scale;
			const Vector3f rxd2 = r_i.cross(dg_db);
			J_ang(3, 2 * i + 1) = T_i * rxd2(0) / t_scale; J_ang(4, 2 * i + 1) = T_i * rxd2(1) / t_scale; J_ang(5, 2 * i + 1) = T_i * rxd2(2) / t_scale;
		}

		// 归一化残差 e_n（即 W − W_ach 经 f/t 缩放后的值）已在上面算得。
		const matrix::Matrix<float, n_ang, 6> Jt = J_ang.transpose();
		matrix::Matrix<float, n_ang, n_ang> JtJ = Jt * J_ang;
		const float lambda = 1e-3f;   // LM 阻尼系数：在奇异位形附近保持 JtJ 可逆、提高稳定性

		for (int k = 0; k < n_ang; ++k) { JtJ(k, k) += lambda; }

		matrix::Matrix<float, n_ang, n_ang> JtJ_inv;

		if (!matrix::geninv(JtJ, JtJ_inv)) {
			// JtJ 奇异：保留当前角度，终止迭代。
			A_g_final = A_g; A_pinv_final = A_pinv; u0_final = u0;
			break;
		}

		const matrix::Vector<float, n_ang> Jte = Jt * e_n;
		const matrix::Vector<float, n_ang> dtheta = JtJ_inv * Jte;   // 高斯-牛顿角度增量

		// 信任域缩放的 GN 步（CA_MQ_ANGLE_GAIN 为步长系数），并对单周期偏转角做上限钳制以保证稳定。
		const float max_step = math::radians(20.f);

		for (int i = 0; i < NUM_ROTORS; ++i) {
			_alpha_prev[i] += math::constrain(k_ang * dtheta(2 * i),     -max_step, max_step);
			_beta_prev[i]  += math::constrain(k_ang * dtheta(2 * i + 1), -max_step, max_step);
		}

		// 每一轮迭代后都钳制到机械限位，防止解算超出物理可达范围。
		for (int i = 0; i < NUM_ROTORS; ++i) {
			_alpha_prev[i] = math::constrain(_alpha_prev[i], -a_max, a_max);
			_beta_prev[i]  = math::constrain(_beta_prev[i],  -b_max, b_max);
		}

		// 记录最后一次有效解，供最终重建使用。
		A_g_final = A_g;
		A_pinv_final = A_pinv;
		u0_final = u0;
	}

	// ── 零空间 N = I - A_g⁺ A_g。任意 z ∈ 零空间均满足 A_g N z = 0，──
	// ── 因此 u = u0 + N z 实现的是"同一个（已重定向后的）广义力 W"。      ──
	const matrix::Matrix<float, NUM_ROTORS, NUM_ROTORS> I = matrix::eye<float, NUM_ROTORS>();
	const matrix::Matrix<float, NUM_ROTORS, NUM_ROTORS> N_mat = I - A_pinv_final * A_g_final;

	// ── 零空间优化 —— 在冗余自由度上最小化代价函数 J(u) ──
	// 采用梯度下降：z = -k * grad(J)。具有非零梯度的代价项包括：
	//   • 电机功率 ：J_pow = w_pow * Σ u_i^2        →  ∂/∂u_i = 2 w_pow u_i
	//   • 尾流下洗 ：J_dw  = w_dw  * Σ d_ij u_i u_j →  ∂/∂u_i = w_dw Σ (d_ij+d_ji) u_j
	// d_ij 衡量旋翼 i 对 j 的下洗影响（j 位于 i 下游时为正，并随垂直距离衰减）。
	float d_ij[NUM_ROTORS][NUM_ROTORS] {};

	if (w_dw > 0.f) {
		// 仅当下洗权重非零时才计算下洗影响矩阵 d_ij。
		for (int i = 0; i < NUM_ROTORS; ++i) {
			const float sa = sinf(_alpha_prev[i]);
			const float ca = cosf(_alpha_prev[i]);
			const float sb = sinf(_beta_prev[i]);
			const float cb = cosf(_beta_prev[i]);
			const Vector3f g_i(sa * cb, sb, -ca * cb);   // 旋翼 i 的推力方向

			for (int j = 0; j < NUM_ROTORS; ++j) {
				if (i == j) { continue; }   // 不与自身比较

				const Vector3f rel(lx[j] - lx[i], ly[j] - ly[i], 0.f);   // 从 i 指向 j 的平面向量
				const float along = -rel.dot(g_i);   // >0 表示 j 位于 i 的推力下游（被下洗影响）

				if (along <= 0.f) { continue; }   // 仅当 j 在 i 下游时才计入

				const float perp = sqrtf(math::max(rel.dot(rel) - along * along, 0.f));   // 垂直距离
				d_ij[i][j] = along / (perp + 0.05f);   // 沿程分量越大、垂直分量越小，下洗影响越强
			}
		}
	}

	// 累加各项代价梯度。
	matrix::Vector<float, NUM_ROTORS> grad_J{};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		grad_J(i) += 2.f * w_pow * u0_final(i);   // 电机功率项梯度

		if (w_dw > 0.f) {
			float g_dw = 0.f;

			for (int j = 0; j < NUM_ROTORS; ++j) {
				if (i == j) { continue; }

				g_dw += (d_ij[i][j] + d_ij[j][i]) * u0_final(j);
			}

			grad_J(i) += w_dw * g_dw;   // 下洗项梯度
		}
	}

	const matrix::Vector<float, NUM_ROTORS> z = -k_ns * grad_J;   // 沿负梯度方向下降
	const matrix::Vector<float, NUM_ROTORS> u = u0_final + N_mat * z;   // 叠加零空间修正，广义力保持不变

	// 用优化后的推力大小与"最终（重定向后）"的云台方向重建推力矢量。
	// 负推力在物理上不可实现 —— 因此做下限钳制为 0。
	for (int i = 0; i < NUM_ROTORS; ++i) {
		const float T_i = math::max(u(i), 0.f);
		const float sa = sinf(_alpha_prev[i]);
		const float ca = cosf(_alpha_prev[i]);
		const float sb = sinf(_beta_prev[i]);
		const float cb = cosf(_beta_prev[i]);
		const Vector3f g(sa * cb, sb, -ca * cb);
		t_out[i][0] = T_i * g(0);
		t_out[i][1] = T_i * g(1);
		t_out[i][2] = T_i * g(2);
	}
}

void ActuatorEffectivenessMorphQuad::recoverServoAngles(
	const float t[3], int idx,
	float &alpha_out, float &beta_out)
{
	const float t_norm = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);

	// 若推力幅值可忽略（接近零），沿用上一周期的角度，避免舵机因数值噪声抖动。
	if (t_norm < 0.01f) {
		alpha_out = _alpha_prev[idx];
		beta_out = _beta_prev[idx];
		return;
	}

	// 单位化的推力方向。
	const float t_hat_x = t[0] / t_norm;
	const float t_hat_y = t[1] / t_norm;
	const float t_hat_z = t[2] / t_norm;

	// 双支解（对应论文公式 Eq.14）：由于 asin 的多值性，beta 存在两支等价解。
	// 分支 1：beta 落在 [-π/2, π/2] 区间。
	const float beta1 = asinf(math::constrain(t_hat_y, -1.0f, 1.0f));
	const float alpha1 = atan2f(t_hat_x, -t_hat_z);

	// 分支 2：beta 落在 [π/2, 3π/2] 区间（与分支 1 几何等价但角度取值不同）。
	const float beta2 = M_PI_F - beta1;
	const float alpha2 = M_PI_F + alpha1;

	// 最小运动选择：比较两支解与历史角度的总偏移，选取离历史最近的支，避免跳变。
	const float d1 = fabsf(alpha1 - _alpha_prev[idx]) + fabsf(beta1 - _beta_prev[idx]);
	const float d2 = fabsf(alpha2 - _alpha_prev[idx]) + fabsf(beta2 - _beta_prev[idx]);

	float alpha, beta;

	if (d2 < d1) {
		alpha = alpha2;
		beta = beta2;

	} else {
		alpha = alpha1;
		beta = beta1;
	}

	// 多圈环绕检查：alpha 是绕偏航的连续角度，可加减整数圈（2π）后等价。
	// 在 NR 圈范围内寻找使"相对历史运动最小"的环绕解，避免大角度绕回。
	float best_dist = fabsf(alpha - _alpha_prev[idx]) + fabsf(beta - _beta_prev[idx]);
	float best_alpha = alpha;

	for (int k = -NR; k <= NR; ++k) {
		if (k == 0) {
			continue;   // 已包含 k=0 的情况
		}

		const float alpha_wrap = alpha + 2.0f * M_PI_F * k;
		const float dist = fabsf(alpha_wrap - _alpha_prev[idx]) + fabsf(beta - _beta_prev[idx]);

		if (dist < best_dist) {
			best_dist = dist;
			best_alpha = alpha_wrap;
		}
	}

	alpha = best_alpha;

	// 钳制到参数定义的机械限位角，保证解在物理可达范围。
	const float alpha_max_rad = math::radians(_param_ca_mq_alpm.get());
	const float beta_max_rad = math::radians(_param_ca_mq_betm.get());
	alpha = math::constrain(alpha, -alpha_max_rad, alpha_max_rad);
	beta = math::constrain(beta, -beta_max_rad, beta_max_rad);

	// 更新该旋翼的历史角度，供下一周期"最小运动"选择使用。
	_alpha_prev[idx] = alpha;
	_beta_prev[idx] = beta;

	alpha_out = alpha;
	beta_out = beta;
}

void ActuatorEffectivenessMorphQuad::getUnallocatedControl(int matrix_index,
		control_allocator_status_s &status)
{
	// 用我们自己的、考虑饱和情况的残余广义力计算结果，覆盖框架默认的未分配控制量。
	// 框架默认实现并不知道舵机/电机的饱和，因此这里把 computeUnallocatedWrench()
	// 算得的残余力/力矩上报给控制器（用于抗饱和反馈）。
	for (int i = 0; i < 3; i++) {
		status.unallocated_torque[i] = _unallocated_torque[i];
		status.unallocated_thrust[i] = _unallocated_thrust[i];
	}
}

void ActuatorEffectivenessMorphQuad::computeUnallocatedWrench(
	const Vector3f &f_des, const Vector3f &tau_des,
	const float omega_raw[NUM_ROTORS], const float alpha_raw[NUM_ROTORS],
	const float beta_raw[NUM_ROTORS],
	const ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min,
	const ActuatorVector &actuator_max)
{
	// 先初始化为零（表示没有未分配的广义力）。
	for (int i = 0; i < 3; i++) {
		_unallocated_torque[i] = 0.f;
		_unallocated_thrust[i] = 0.f;
	}

	// 若没有任何作动器饱和，则期望广义力被完全实现，无需进一步计算。
	if (!_motor_saturated && !_servo_saturated) {
		return;
	}

	// 从钳位后的实际作动器输出，重建"真正被实现"的广义力。
	// 对每个旋翼，根据其饱和后的电机指令与舵机角度，算出实际推力矢量并累加。
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float tmax = _param_ca_mq_tmax.get();

	Vector3f f_achieved(0.f, 0.f, 0.f);    // 实际合力
	Vector3f tau_achieved(0.f, 0.f, 0.f);  // 实际合力矩

	// 旋翼在机体 X 布局下的臂位置（从右前 RF 起，逆时针依次为 RF、LF、LB、RB）。
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = { b, -b, -b,  b};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// 从饱和后的作动器输出恢复实际 alpha、beta（每电机交错存放：
		// alpha 在索引 4+2*i，beta 在索引 5+2*i），并换算回弧度。
		const float alpha_actual = actuator_sp(4 + 2 * i) * M_PI_F;
		const float beta_actual  = actuator_sp(5 + 2 * i) * M_PI_F;

		// 实际推力大小：actuator_sp(i) 为归一化转速（esc = Ω/Ω_max），
		// 由转子模型 T = esc^2 * TMAX = c_T * Ω^2。
		const float t_mag = actuator_sp(i) * actuator_sp(i) * tmax;

		// 由双轴云台角求推力方向（alpha=外环/偏航，beta=内环/俯仰）。
		// 正运动学：R = Ry(alpha) * Rx(beta)，推力沿机体 -z 时为竖直向上。
		const float sa = sinf(alpha_actual);
		const float ca = cosf(alpha_actual);
		const float sb = sinf(beta_actual);
		const float cb = cosf(beta_actual);
		Vector3f t_vec(t_mag * sa * cb, t_mag * sb, -t_mag * ca * cb);

		// 累加力。
		f_achieved += t_vec;

		// 累加力矩：tau = l_i × t_i。
		Vector3f l_i(lx[i], ly[i], 0.f);
		tau_achieved += l_i.cross(t_vec);
	}

	// 未分配量 = 期望 - 实际实现。
	const Vector3f unalloc_f = f_des - f_achieved;
	const Vector3f unalloc_tau = tau_des - tau_achieved;

	for (int i = 0; i < 3; i++) {
		_unallocated_thrust[i] = unalloc_f(i);
		_unallocated_torque[i] = unalloc_tau(i);
	}
}
