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

#include "geo_control.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <lib/perf/perf_counter.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

GeoControl::GeoControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

bool GeoControl::init()
{
	if (!_sub_vehicle_local_position.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void GeoControl::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

void GeoControl::resetIntegrators()
{
	_rate_int.zero();
	_vel_int.zero();
}

void GeoControl::updateIntegrators(const Vector3f &e_omega, const Vector3f &e_v_world, float dt)
{
	const float rate_i_gain = _param_gc_rate_i.get();
	const float rate_int_lim = _param_gc_rate_int_lim.get();
	const float vel_i_gain = _param_gc_vel_i.get();
	const float vel_int_lim = _param_gc_vel_int_lim.get();

	// ── 角速率/力矩积分器 (含抗饱和) ──
	for (int i = 0; i < 3; i++) {
		float rate_error = e_omega(i);

		// 抗饱和: 饱和方向上停止积分
		if (_torque_sat_pos(i)) {
			rate_error = math::min(rate_error, 0.f);
		}

		if (_torque_sat_neg(i)) {
			rate_error = math::max(rate_error, 0.f);
		}

		// 非线性 I 因子: 大误差时降低增益 (防止翻滚时积分饱和)
		float i_factor = rate_error / math::radians(400.f);
		i_factor = math::max(0.f, 1.f - i_factor * i_factor);

		float rate_i = _rate_int(i) + i_factor * rate_i_gain * rate_error * dt;

		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -rate_int_lim, rate_int_lim);
		}
	}

	// ── 速度/力积分器 (含抗饱和) ──
	for (int i = 0; i < 3; i++) {
		float vel_error = e_v_world(i);

		// 推力饱和方向上的抗饱和
		if (_thrust_sat_pos(i)) {
			vel_error = math::min(vel_error, 0.f);
		}

		if (_thrust_sat_neg(i)) {
			vel_error = math::max(vel_error, 0.f);
		}

		float vel_i = _vel_int(i) + vel_i_gain * vel_error * dt;

		if (PX4_ISFINITE(vel_i)) {
			_vel_int(i) = math::constrain(vel_i, -vel_int_lim, vel_int_lim);
		}
	}
}

void GeoControl::publish_wrench(const Vector3f &f_des_body, const Vector3f &tau_des,
				uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();

	vehicle_thrust_setpoint_s thrust_msg{};
	thrust_msg.timestamp = now;
	thrust_msg.timestamp_sample = timestamp_sample;
	thrust_msg.xyz[0] = f_des_body(0);
	thrust_msg.xyz[1] = f_des_body(1);
	thrust_msg.xyz[2] = f_des_body(2);
	_thrust_pub.publish(thrust_msg);

	vehicle_torque_setpoint_s torque_msg{};
	torque_msg.timestamp = now;
	torque_msg.timestamp_sample = timestamp_sample;
	torque_msg.xyz[0] = tau_des(0);
	torque_msg.xyz[1] = tau_des(1);
	torque_msg.xyz[2] = tau_des(2);
	_torque_pub.publish(torque_msg);

	// 缓存最新期望力/力矩，供 `info` 命令查询
	_f_des_last = f_des_body;
	_tau_des_last = tau_des;
	_wrench_timestamp = now;
}

void GeoControl::publish_setpoints(const Vector3f &f_des_world, const Vector3f &f_des_body,
				   const Vector3f &tau_des, const Quatf &q_att_ref,
				   const Vector3f &vel_sp_world, const Vector3f &omega,
				   uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();
	const float thrust_max = math::max(_param_gc_thrust_max.get(), 1.f);

	// ── 归一化总推力估计 (总力占比) ──
	// throttle_norm = ‖f_des_world‖ / (N_rotors * TMAX) = ‖f‖/GC_THRUST_MAX。
	// 这是总力占比，而非单路电机指令。悬停时 ‖f‖ = mass*g，
	// 因此 throttle_norm ≈ 0.41 (44.7/110)。实际单路电机指令为
	// 归一化转速 sqrt(T_i/TMAX) ≈ 0.64 (悬停)，参见 ActuatorEffectivenessMorphQuad。
	const float throttle_norm = math::constrain(f_des_world.norm() / thrust_max, 0.f, 1.f);

	// ── vehicle_local_position_setpoint (位置/速度调试 + 油门估计) ──
	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.vx = vel_sp_world(0);
	local_pos_sp.vy = vel_sp_world(1);
	local_pos_sp.vz = vel_sp_world(2);
	local_pos_sp.acceleration[0] = 0.f;
	local_pos_sp.acceleration[1] = 0.f;
	local_pos_sp.acceleration[2] = 0.f;
	local_pos_sp.thrust[0] = f_des_body(0) / thrust_max;
	local_pos_sp.thrust[1] = f_des_body(1) / thrust_max;
	local_pos_sp.thrust[2] = throttle_norm;
	local_pos_sp.yaw = Eulerf(q_att_ref).psi();
	local_pos_sp.yawspeed = 0.f;
	_local_pos_sp_pub.publish(local_pos_sp);

	// ── vehicle_attitude_setpoint (期望姿态 + 机体推力) ──
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.q_d[0] = q_att_ref(0);
	att_sp.q_d[1] = q_att_ref(1);
	att_sp.q_d[2] = q_att_ref(2);
	att_sp.q_d[3] = q_att_ref(3);
	att_sp.thrust_body[0] = f_des_body(0) / thrust_max;
	att_sp.thrust_body[1] = f_des_body(1) / thrust_max;
	att_sp.thrust_body[2] = throttle_norm;
	att_sp.yaw_sp_move_rate = 0.f;
	_att_sp_pub.publish(att_sp);

	// ── vehicle_angular_acceleration_setpoint ──
	// 期望角加速度: τ = J·α + ω×Jω  →  α = J⁻¹(τ − ω×Jω)
	Matrix3f J = matrix::diag(Vector3f(
					   _param_gc_inertia_xx.get(),
					   _param_gc_inertia_yy.get(),
					   _param_gc_inertia_zz.get()));
	const Vector3f gyro_term = omega.cross(J * omega);
	const Vector3f ang_accel = J.I() * (tau_des - gyro_term);

	vehicle_angular_acceleration_setpoint_s ang_accel_sp{};
	ang_accel_sp.timestamp = now;
	ang_accel_sp.timestamp_sample = timestamp_sample;
	ang_accel_sp.xyz[0] = ang_accel(0);
	ang_accel_sp.xyz[1] = ang_accel(1);
	ang_accel_sp.xyz[2] = ang_accel(2);
	_ang_accel_sp_pub.publish(ang_accel_sp);
}

matrix::Vector3f GeoControl::computeAttitudeTorque(const Dcmf &R, const Dcmf &R_sp,
		const Vector3f &omega, const Vector3f &omega_sp,
		const Vector3f &omega_d_in_B, float dt)
{
	// 姿态误差 (Lee SO(3)): e_R = 1/2 (Rdᵀ R − Rᵀ Rd)^∨
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	const Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());
	Matrix3f J = matrix::diag(Vector3f(_param_gc_inertia_xx.get(), _param_gc_inertia_yy.get(),
					  _param_gc_inertia_zz.get()));

	// 完整 Lee SO(3) 角加速度前馈:
	//   τ_ff = −J ( Ω̂ RᵀRd Ωd − RᵀRd Ω̇d )
	// Ωd = 期望机体角速率 (omega_sp); Ω̇d 由速率参考的有限差分得到。
	// 注意 Ω̂ 是当前角速度，故 Ω̂ RᵀRd Ωd = ω × ω_d_in_B。
	const Dcmf RtRd = R.transpose() * R_sp;
	const Vector3f omega_dot_d = (omega_sp - _omega_sp_prev) / dt;
	_omega_sp_prev = omega_sp;
	const Vector3f ff = -J * (omega.cross(omega_d_in_B) - RtRd * omega_dot_d);

	return -kR.emult(e_R) - kOmega.emult(omega - omega_d_in_B) + _rate_int + omega.cross(J * omega) + ff;
}

// ─────────────────────────────────────────────────────────────────────────────
// 手动模式: RC 摇杆 → 角速率 + 速度参考
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_manual(const Quatf &R_q, const Vector3f &omega,
				     uint64_t timestamp_sample)
{
	// 手动控制: 当无有效 RC/摇杆输入时 (SITL/HIL 常见)，
	// 不回退为零力/力矩，而是退回到零摇杆指令，
	// 使速度控制器仍输出悬停推力 (-mass*g)。
	// 这样 vehicle_thrust_setpoint 保持非零，避免 land detector
	// 判定"已着陆"而自动上锁。
	manual_control_setpoint_s manual_sp{};
	const bool has_manual = _sub_manual_control_sp.update(&manual_sp) && manual_sp.valid;

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);

	const float rate_max = _param_gc_man_rate_max.get();
	const float dz = _param_gc_man_dz.get();
	const float mass = _param_gc_mass.get();

	float p_sp = 0.f;
	float q_sp = 0.f;
	float r_sp = 0.f;
	float fx_body = 0.f;
	float fy_body = 0.f;
	float fz_body = -mass * 9.81f;   // 无摇杆输入时保持悬停推力 (向上)

	if (has_manual) {
		// 姿态: aux1/aux2 → roll/pitch 角速率 [rad/s] (闭环);
		// yaw 摇杆 → yaw 角速率
		p_sp = math::deadzone(manual_sp.aux1, dz) * rate_max;
		q_sp = math::deadzone(manual_sp.aux2, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;

		// 力: roll/pitch → 机体 x/y 力 [N] (开环, 无反馈);
		// throttle → 机体 z 力。机体 z 向下，悬停推力 -mass*g (向上)，
		// 推油门增加向上力 (爬升)
		const float fmax = _param_gc_man_fmax.get();
		fy_body = math::deadzone(manual_sp.roll, dz) * fmax;
		fx_body = math::deadzone(manual_sp.pitch, dz) * fmax;
		fz_body = -mass * 9.81f - math::deadzone(manual_sp.throttle, dz) * fmax;
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);
	Vector3f f_des_body(fx_body, fy_body, fz_body);

	// 角速率积分 → 姿态参考 (四元数积分)
	const Vector3f omega_vec(omega_sp(0), omega_sp(1), omega_sp(2));
	const float omega_norm = omega_vec.norm();

	if (omega_norm > 1e-6f) {
		const float angle = omega_norm * dt;
		const Vector3f axis = omega_vec / omega_norm;
		const float ha = angle * 0.5f;
		const float sha = sinf(ha);
		const float cha = cosf(ha);
		Quatf dq(cha, axis(0) * sha, axis(1) * sha, axis(2) * sha);
		_q_att_ref = _q_att_ref * dq;
		_q_att_ref.normalize();
	}

	// ── 姿态环 → 期望力矩 (闭环, Lee SO(3) 含前馈) ──
	const Dcmf R(R_q);
	const Dcmf R_sp(_q_att_ref);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	// 开环力模式: 速度/位置反馈关闭，仅更新角速率/姿态积分器
	if (!_landed) {
		updateIntegrators(e_omega, Vector3f(0.f, 0.f, 0.f), dt);
	}

	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	const Vector3f f_des_world = R * f_des_body;   // 仅用于 throttle-norm 调试

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, Vector3f(0.f, 0.f, 0.f), omega,
			  timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// 位置模式: 级联位置外环 + 速度内环 (闭环)
// 需要有效位置估计 (GPS / 光流 / VIO)。摇杆回中 → 位置保持。
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_position(const Quatf &R_q, const Vector3f &pos,
				       const Vector3f &vel, const Vector3f &omega,
				       uint64_t timestamp_sample)
{
	// 与手动模式相同的无 RC 回退: 保持上次/零指令，
	// 使位置环保持悬停，而非回退为零力/力矩
	manual_control_setpoint_s manual_sp{};
	const bool has_manual = _sub_manual_control_sp.update(&manual_sp) && manual_sp.valid;

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);

	const float rate_max = _param_gc_man_rate_max.get();
	const float vel_max = _param_gc_man_vel_max.get();
	const float dz = _param_gc_man_dz.get();
	const float mass = _param_gc_mass.get();

	float p_sp = 0.f;
	float q_sp = 0.f;
	float r_sp = 0.f;
	float vx_sp_b = 0.f;
	float vy_sp_b = 0.f;
	float vz_sp_b = 0.f;

	if (has_manual) {
		// 姿态: 摇杆 [-1,1] → 角速率 [rad/s]
		p_sp = math::deadzone(manual_sp.roll,  dz) * rate_max;
		q_sp = math::deadzone(manual_sp.pitch, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;

		// 摇杆 [-1,1] → 机体速度 [m/s] (aux1/aux2 水平, throttle 垂向)
		vx_sp_b = math::deadzone(manual_sp.aux1, dz) * vel_max;
		vy_sp_b = math::deadzone(manual_sp.aux2, dz) * vel_max;
		vz_sp_b = math::deadzone(manual_sp.throttle, dz) * vel_max;
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);
	Vector3f vel_sp_body(vx_sp_b, vy_sp_b, vz_sp_b);

	const Dcmf R(R_q);
	Vector3f vel_sp_world_stick = R * vel_sp_body;

	// 位置外环: 维护位置指令。摇杆推时跟随速度移动，回中时保持当前位置
	if (!_pos_sp_init) {
		_pos_sp = pos;
		_pos_sp_init = true;
	}

	if (vel_sp_world_stick.norm() > 0.05f) {
		_pos_sp += vel_sp_world_stick * dt;
	}

	Vector3f kp_pos(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f vel_sp_world = vel_sp_world_stick + kp_pos.emult(_pos_sp - pos);

	// 角速率积分 → 姿态参考 (四元数积分)
	const Vector3f omega_vec(omega_sp(0), omega_sp(1), omega_sp(2));
	const float omega_norm = omega_vec.norm();

	if (omega_norm > 1e-6f) {
		const float angle = omega_norm * dt;
		const Vector3f axis = omega_vec / omega_norm;
		const float ha = angle * 0.5f;
		const float sha = sinf(ha);
		const float cha = cosf(ha);
		Quatf dq(cha, axis(0) * sha, axis(1) * sha, axis(2) * sha);
		_q_att_ref = _q_att_ref * dq;
		_q_att_ref.normalize();
	}

	// ── 速度环 → 期望力 ──
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_v = vel - vel_sp_world;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kv.emult(e_v) - _vel_int - mass * g;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── 姿态环 → 期望力矩 (Lee SO(3) 含前馈) ──
	const Dcmf R_sp(_q_att_ref);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	if (!_landed) {
		updateIntegrators(e_omega, e_v, dt);
	}

	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, vel_sp_world, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Offboard 模式: 双路径控制
//   路径1: trajectory_setpoint → 全状态位置+姿态控制
//   路径2: vehicle_attitude_setpoint → 姿态跟踪+悬停保持
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_offboard(const Quatf &R_q, const Vector3f &pos,
				       const Vector3f &vel, const Vector3f &omega,
				       uint64_t timestamp_sample)
{
	trajectory_setpoint_s traj_sp;

	// ── 路径1: 全状态位置+姿态控制 (SET_POSITION_TARGET_LOCAL_NED) ──
	if (_sub_trajectory_sp.update(&traj_sp)) {

		const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
						 0.001f, 0.05f);
		const float mass = _param_gc_mass.get();
		const Dcmf R(R_q);

		Vector3f pos_sp(traj_sp.position[0], traj_sp.position[1], traj_sp.position[2]);
		Vector3f vel_sp(traj_sp.velocity[0], traj_sp.velocity[1], traj_sp.velocity[2]);
		Vector3f acc_sp(traj_sp.acceleration[0], traj_sp.acceleration[1], traj_sp.acceleration[2]);

		const float yaw_sp = traj_sp.yaw;
		const Eulerf euler_sp(0.f, 0.f, yaw_sp);
		const Dcmf R_sp(euler_sp);

		for (int i = 0; i < 3; i++) {
			if (!PX4_ISFINITE(pos_sp(i))) { pos_sp(i) = pos(i); }

			if (!PX4_ISFINITE(vel_sp(i))) { vel_sp(i) = vel(i); }

			if (!PX4_ISFINITE(acc_sp(i))) { acc_sp(i) = 0.f; }
		}

		// ── 位置+速度环 → 期望力 ──
		Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
		Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
		Vector3f e_p = pos - pos_sp;
		Vector3f e_v = vel - vel_sp;
		Vector3f g(0.f, 0.f, 9.81f);
		Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) - _vel_int - mass * g + mass * acc_sp;
		Vector3f f_des_body = R.transpose() * f_des_world;

		// ── 姿态环 → 期望力矩 (Lee SO(3) 含前馈) ──
		Vector3f omega_sp(0.f, 0.f, traj_sp.yawspeed);
		Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
		Vector3f e_omega = omega - omega_d_in_B;

		if (!_landed) {
			updateIntegrators(e_omega, e_v, dt);
		}

		Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

		publish_wrench(f_des_body, tau_des, timestamp_sample);
		publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, omega, timestamp_sample);
		return;
	}

	// ── 路径2: 仅姿态控制 (外部 MAVLink SET_ATTITUDE_TARGET) ──
	// 当无 trajectory_setpoint 时，回退到 mavlink_receiver 从
	// SET_ATTITUDE_TARGET 发布的 vehicle_attitude_setpoint。
	// 此路径使外部姿态激励测试 (如 attitude_test.py) 可通过
	// 正常 offboard MAVLink 通道工作。
	vehicle_attitude_setpoint_s att_sp;

	if (!_sub_vehicle_attitude_setpoint.update(&att_sp)) {
		return;
	}

	const Quatf q_sp(att_sp.q_d);

	// 验证四元数有效性 (范数 ≈ 1.0)
	if (fabsf(q_sp.norm() - 1.0f) > 0.01f) {
		return;
	}

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);
	const float mass = _param_gc_mass.get();
	const Dcmf R(R_q);
	const Dcmf R_sp(q_sp);

	// ── 推力: 将外部归一化推力转换为牛顿 ──
	// MAVLink fill_thrust() 对旋翼机: thrust_body[2] = -thrust (NED: 负 = 向上)
	// 例: thrust=0.45 → thrust_body[2]=-0.45 → force_Z = -0.45 * GC_THRUST_MAX ≈ -49.5 N ≈ 悬停
	float thrust_cmd = att_sp.thrust_body[2] * _param_gc_thrust_max.get();

	if (fabsf(thrust_cmd) < 0.5f) {
		// 无有效推力指令时回退到悬停推力
		thrust_cmd = -mass * 9.81f;
	}

	// ── 力: 机体 Z 推力 + 世界系 XY 速度阻尼 (抗漂移) ──
	Vector3f f_des_body(0.f, 0.f, thrust_cmd);
	Vector3f f_des_world = R * f_des_body;

	// 轻量 XY 速度阻尼 (仅抗漂移，非位置保持)
	const float damp_xy = mass * 0.5f;  // ~0.5 m/s 时间常数
	f_des_world(0) -= damp_xy * vel(0);
	f_des_world(1) -= damp_xy * vel(1);
	f_des_body = R.transpose() * f_des_world;

	// ── 姿态环 (Lee SO(3)): 跟踪外部四元数 ──
	Vector3f omega_sp(0.f, 0.f, 0.f);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	if (!_landed) {
		updateIntegrators(e_omega, Vector3f(0.f, 0.f, 0.f), dt);
	}

	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Auto 模式: flight_mode_manager 指令 → 几何控制律
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_auto(const Quatf &R_q, const Vector3f &pos,
				   const Vector3f &vel, const Vector3f &omega,
				   uint64_t timestamp_sample)
{
	vehicle_local_position_setpoint_s local_pos_sp;
	vehicle_attitude_setpoint_s att_sp;

	if (!_sub_local_pos_sp.update(&local_pos_sp) ||
	    !_sub_vehicle_attitude_setpoint.update(&att_sp)) {
		return;
	}

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);
	const float mass = _param_gc_mass.get();
	const Dcmf R(R_q);

	Vector3f pos_sp(local_pos_sp.x, local_pos_sp.y, local_pos_sp.z);
	Vector3f vel_sp(local_pos_sp.vx, local_pos_sp.vy, local_pos_sp.vz);
	Vector3f acc_sp(local_pos_sp.acceleration[0],
			local_pos_sp.acceleration[1],
			local_pos_sp.acceleration[2]);

	const Dcmf R_sp(Quatf(att_sp.q_d));

	// ── 位置+速度环 → 期望力 ──
	Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_p = pos - pos_sp;
	Vector3f e_v = vel - vel_sp;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) - _vel_int - mass * g + mass * acc_sp;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── 姿态环 → 期望力矩 (Lee SO(3) 含前馈) ──
	Vector3f omega_sp(0.f, 0.f, 0.f);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	if (!_landed) {
		updateIntegrators(e_omega, e_v, dt);
	}

	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// 主控制调度
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step(const vehicle_local_position_s &local_pos)
{
	// 分配验证: 激活时每周期注入固定机体力/力矩，绕过正常模式/解锁调度。
	// `verify` 命令驱动 _verify_wrench 并从真实 ControlAllocator 读回
	// 执行器输出，使分配器在已知力/力矩上运行，不影响飞行控制器。
	if (_verify_active) {
		Vector3f f_v(_verify_wrench(0), _verify_wrench(1), _verify_wrench(2));
		Vector3f tau_v(_verify_wrench(3), _verify_wrench(4), _verify_wrench(5));
		publish_wrench(f_v, tau_v, hrt_absolute_time());
		return;
	}

	// 姿态测试: 跟踪期望欧拉角
	if (_att_test_active) {
		control_step_att_test(local_pos.timestamp_sample);
		return;
	}

	// 速度测试: 跟踪期望机体速度 (含姿态内环)
	if (_vel_test_active) {
		control_step_vel_test(local_pos.timestamp_sample);
		return;
	}

	// Offboard 测试: 内部注入 trajectory_setpoint 实现闭环位置控制
	if (_offboard_test_active) {
		if (_offboard_test_dur_us > 0) {
			if (hrt_absolute_time() - _offboard_test_start >= _offboard_test_dur_us) {
				_offboard_test_active = false;
				PX4_INFO("OFFBOARD_TEST: 完成");
				publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
				return;
			}
		}
		control_step_offboard_test(local_pos.timestamp_sample);
		return;
	}

	// Sweep 模式: 自动 6D 力/力矩扫频测试 (逐轴)
	// 每个采样点打印指令/实际力/力矩 + 电机/舵机输出
	if (_sweep_active) {
		control_step_sweep(local_pos.timestamp_sample);
		return;
	}

	// Acro 模式: 开环固定力/力矩注入 (控制分配验证)
	// 无论解锁状态均运行。_acro_duration_us 后自动关闭
	if (_acro_active) {
		if (_acro_duration_us > 0) {
			const uint64_t elapsed = hrt_absolute_time() - _acro_start_time;
			if (elapsed >= _acro_duration_us) {
				_acro_active = false;
				PX4_INFO("ACRO: %.1f 秒后自动关闭", (double)(elapsed * 1e-6));
				publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
				return;
			}
		}
		control_step_acro(local_pos.timestamp_sample);
		return;
	}

	vehicle_attitude_s att;
	vehicle_angular_velocity_s angular_vel;
	vehicle_status_s vehicle_status{};

	if (!_sub_vehicle_attitude.update(&att) ||
	    !_sub_vehicle_angular_velocity.update(&angular_vel)) {
		return;
	}

	// vehicle_status 使用 copy() 而非 update()：它是低频状态话题，
	// 仅在 control_step 内读取。update() 的"上次读取后新发布"语义
	// 可能使结构体卡在解锁前的值 (arming_state=0)，导致解锁后
	// 控制器仍输出零力/力矩。copy() 始终获取最新值。
	_sub_vehicle_status.copy(&vehicle_status);

	// 更新着陆/解锁状态
	vehicle_land_detected_s land_detected{};

	if (_sub_vehicle_land_detected.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	_armed = vehicle_status.arming_state == vehicle_status.ARMING_STATE_ARMED;
	_nav_state_last = vehicle_status.nav_state;

	// 未解锁时: 清零积分器并发布零力/力矩，使执行器处于中立初值
	// (否则会发布悬停推力 + 姿态保持力矩，导致分配器在零推力退化点把舵机推到饱和)
	if (!_armed) {
		resetIntegrators();
		_pos_sp_init = false;   // 下次解锁时重新初始化位置保持指令
		publish_wrench(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
			       local_pos.timestamp_sample);
		publish_setpoints(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Quatf(1.f, 0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
				  local_pos.timestamp_sample);
		return;
	}

	// 更新控制分配器饱和反馈
	control_allocator_status_s ca_status;

	if (_sub_control_allocator_status.update(&ca_status)) {
		_torque_sat_pos = Vector3<bool>(false, false, false);
		_torque_sat_neg = Vector3<bool>(false, false, false);
		_thrust_sat_pos = Vector3<bool>(false, false, false);
		_thrust_sat_neg = Vector3<bool>(false, false, false);

		if (!ca_status.torque_setpoint_achieved) {
			for (size_t i = 0; i < 3; i++) {
				if (ca_status.unallocated_torque[i] > FLT_EPSILON) {
					_torque_sat_pos(i) = true;

				} else if (ca_status.unallocated_torque[i] < -FLT_EPSILON) {
					_torque_sat_neg(i) = true;
				}
			}
		}

		if (!ca_status.thrust_setpoint_achieved) {
			for (size_t i = 0; i < 3; i++) {
				if (ca_status.unallocated_thrust[i] > FLT_EPSILON) {
					_thrust_sat_pos(i) = true;

				} else if (ca_status.unallocated_thrust[i] < -FLT_EPSILON) {
					_thrust_sat_neg(i) = true;
				}
			}
		}
	}

	// 状态向量
	const Quatf R_q(att.q);
	const Dcmf R(R_q);
	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
	const Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);

	// 根据 nav_state 确定控制模式
	ControlMode mode = ControlMode::AUTO;

	if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_MANUAL) {
		mode = ControlMode::MANUAL;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_POSCTL) {
		mode = ControlMode::POSITION;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_OFFBOARD) {
		mode = ControlMode::OFFBOARD;
	}

	switch (mode) {
	case ControlMode::MANUAL:
		control_step_manual(R_q, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::POSITION:
		control_step_position(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::OFFBOARD:
		control_step_offboard(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::AUTO:
		control_step_auto(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::ACRO:
		// 在 control_step 前面已处理 (acro 绕过解锁检查)
		break;
	}
}

void GeoControl::Run()
{
	if (should_exit()) {
		_sub_vehicle_local_position.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	parameters_update();

	vehicle_local_position_s local_pos;

	if (_sub_vehicle_local_position.update(&local_pos)) {
		_last_run = local_pos.timestamp_sample;
		control_step(local_pos);
	}

	perf_end(_loop_perf);
}

int GeoControl::task_spawn(int argc, char *argv[])
{
	GeoControl *instance = new GeoControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

GeoControl *GeoControl::instantiate(int argc, char *argv[])
{
	return new GeoControl();
}

int GeoControl::custom_command(int argc, char *argv[])
{
	if (argc > 0) {
		if (!strcmp(argv[0], "verify")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->verify_allocation();
		}

		if (!strcmp(argv[0], "acro")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->acro_command(argc, argv);
		}

		if (!strcmp(argv[0], "sweep")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->sweep_command(argc, argv);
		}

		if (!strcmp(argv[0], "hover")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->hover_command(argc, argv);
		}

		if (!strcmp(argv[0], "offboard")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->offboard_command(argc, argv);
		}

		if (!strcmp(argv[0], "att_test")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->att_test_command(argc, argv);
		}

		if (!strcmp(argv[0], "vel_test")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->vel_test_command(argc, argv);
		}
	}

	return print_usage("未知命令");
}

int GeoControl::hover_command(int argc, char *argv[])
{
	// 用法: geo_control hover [duration_s]
	// 注入 Fz = -mass*g (悬停推力)，零力矩，实现悬停。
	// 每 0.5s 监控速度收敛情况。
	float dur_s = 10.f;

	if (argc >= 2) { dur_s = atof(argv[1]); }

	if (dur_s < 1.f) { dur_s = 1.f; }

	if (dur_s > 60.f) { dur_s = 60.f; }

	const float mass = _param_gc_mass.get();
	const float hover_fz = -mass * 9.81f;

	PX4_INFO("HOVER 测试: Fz=%.1f N (%.2f kg × g), 持续=%.0f s", (double)hover_fz, (double)mass, (double)dur_s);
	PX4_INFO("  监控: 速度应收敛到零附近");

	// 委托给 acro，注入悬停力/力矩
	const int new_argc = 9;
	const char *new_argv[9] = {
		"acro",
		"0", "0", nullptr, "0", "0", "0", nullptr, nullptr
	};

	char fz_buf[16], dur_buf[16];
	snprintf(fz_buf, sizeof(fz_buf), "%.1f", (double)hover_fz);
	snprintf(dur_buf, sizeof(dur_buf), "%.0f", (double)dur_s);
	new_argv[3] = fz_buf;
	new_argv[7] = dur_buf;

	return acro_command(new_argc, (char **)new_argv);
}

void GeoControl::control_step_offboard_test(uint64_t timestamp_sample)
{
	// 读取当前状态
	vehicle_attitude_s att;
	vehicle_angular_velocity_s angular_vel;

	if (!_sub_vehicle_attitude.update(&att) ||
	    !_sub_vehicle_angular_velocity.update(&angular_vel)) {
		return;
	}

	const Quatf R_q(att.q);
	const Dcmf R(R_q);
	const Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);
	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f), 0.001f, 0.05f);
	const float mass = _param_gc_mass.get();

	// offboard 指令中的期望状态
	const Vector3f pos_sp = _offboard_pos_sp;
	const Vector3f vel_sp(0, 0, 0);  // 位置保持
	const Vector3f acc_sp(0, 0, 0);
	const float yaw_sp = _offboard_yaw_sp;
	const Eulerf euler_sp(0, 0, yaw_sp);
	const Dcmf R_sp(euler_sp);

	// 从 local_position 订阅读取位置
	vehicle_local_position_s local_pos;

	if (!_sub_vehicle_local_position.update(&local_pos)) {
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), timestamp_sample);
		return;
	}

	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);

	// ── 位置+速度环 → 期望力 ──
	Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_p = pos - pos_sp;
	Vector3f e_v = vel - vel_sp;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) - _vel_int - mass * g + mass * acc_sp;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── 姿态环 → 期望力矩 ──
	Vector3f omega_sp(0.f, 0.f, 0.f);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	updateIntegrators(e_omega, e_v, dt);
	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	publish_wrench(f_des_body, tau_des, timestamp_sample);

	// 每 0.5s 打印诊断
	static hrt_abstime last_p = 0;
	hrt_abstime now = hrt_absolute_time();

	if (now - last_p > 500000) {
		last_p = now;
		float pos_err = e_p.norm();
		float vel_norm = vel.norm();
		PX4_INFO("OFFBOARD pos=(%.2f,%.2f,%.2f) err=%.3f vel=%.3f f_body=(%.1f,%.1f,%.1f)",
			 (double)pos(0), (double)pos(1), (double)pos(2),
			 (double)pos_err, (double)vel_norm,
			 (double)f_des_body(0), (double)f_des_body(1), (double)f_des_body(2));
	}
}

int GeoControl::offboard_command(int argc, char *argv[])
{
	// 用法: geo_control offboard x y z [yaw_deg] [duration_s]
	// 对指定 NED 指令运行闭环位置控制。
	if (argc < 2) {
		PX4_INFO_RAW("用法: geo_control offboard X Y Z [yaw_deg] [duration_s]\n");
		PX4_INFO_RAW("  X/Y/Z: 目标位置 NED 坐标系 (m), Z 负 = 向上\n");
		PX4_INFO_RAW("  yaw_deg: 目标偏航角 度 (默认 0)\n");
		PX4_INFO_RAW("  duration_s: 测试持续时间 (默认 20)\n");
		PX4_INFO_RAW("示例:\n");
		PX4_INFO_RAW("  geo_control offboard 0 0 -2      # 起飞 2m, 悬停 20s\n");
		PX4_INFO_RAW("  geo_control offboard 0 0 -5 30  # 悬停在 -5m, 持续 30s\n");
		PX4_INFO_RAW("  geo_control offboard stop\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		_offboard_test_active = false;
		resetIntegrators();
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
		PX4_INFO("OFFBOARD_TEST: 已停止");
		return 0;
	}

	if (argc < 4) {
		PX4_ERR("需要至少 X Y Z 三个参数");
		return -1;
	}

	_offboard_pos_sp(0) = atof(argv[1]);
	_offboard_pos_sp(1) = atof(argv[2]);
	_offboard_pos_sp(2) = atof(argv[3]);

	_offboard_yaw_sp = 0.f;

	if (argc >= 5) { _offboard_yaw_sp = math::radians(atof(argv[4])); }

	float dur_s = 20.f;

	if (argc >= 6) { dur_s = atof(argv[5]); }

	_offboard_test_dur_us = (uint64_t)(dur_s * 1e6f);
	_offboard_test_start = hrt_absolute_time();
	resetIntegrators();
	_offboard_test_active = true;

	PX4_INFO("OFFBOARD: 目标=(%.1f,%.1f,%.1f) yaw=%.0f 持续=%.0fs",
		 (double)_offboard_pos_sp(0), (double)_offboard_pos_sp(1), (double)_offboard_pos_sp(2),
		 (double)math::degrees(_offboard_yaw_sp), (double)dur_s);
	return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// 姿态测试: 跟踪期望欧拉角 (roll, pitch, yaw)
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_att_test(uint64_t timestamp_sample)
{
	vehicle_attitude_s att;
	vehicle_angular_velocity_s angular_vel;

	if (!_sub_vehicle_attitude.update(&att) ||
	    !_sub_vehicle_angular_velocity.update(&angular_vel)) {
		return;
	}

	const Quatf R_q(att.q);
	const Dcmf R(R_q);
	const Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);

	// 从欧拉角 RPY (弧度) 得到期望姿态
	const Eulerf euler_sp(_att_test_roll, _att_test_pitch, _att_test_yaw);
	const Dcmf R_sp(euler_sp);
	const Vector3f omega_sp(0, 0, 0);

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f), 0.001f, 0.05f);
	const Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	const Vector3f e_omega = omega - omega_d_in_B;

	updateIntegrators(e_omega, Vector3f(0,0,0), dt);
	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	// 力: 仅悬停推力
	const float mass = _param_gc_mass.get();
	Vector3f f_des_world(0, 0, -mass * 9.81f);
	Vector3f f_des_body = R.transpose() * f_des_world;

	publish_wrench(f_des_body, tau_des, timestamp_sample);

	static hrt_abstime last_p = 0;
	hrt_abstime now = hrt_absolute_time();

	if (now - last_p > 500000) {
		last_p = now;
		Eulerf euler_cur(R_q);
		float roll_err = _att_test_roll - euler_cur.phi();
		float pitch_err = _att_test_pitch - euler_cur.theta();
		float yaw_err = _att_test_yaw - euler_cur.psi();

		while (yaw_err > M_PI_F) { yaw_err -= 2*M_PI_F; }

		while (yaw_err < -M_PI_F) { yaw_err += 2*M_PI_F; }

		PX4_INFO("ATT_TEST 指令(r=%.0f,p=%.0f,y=%.0f)° 实际(r=%.0f,p=%.0f,y=%.0f)° 误差(r=%.1f,p=%.1f,y=%.1f)° tau=(%.2f,%.2f,%.2f)",
			 (double)math::degrees(_att_test_roll), (double)math::degrees(_att_test_pitch),
			 (double)math::degrees(_att_test_yaw),
			 (double)math::degrees(euler_cur.phi()), (double)math::degrees(euler_cur.theta()),
			 (double)math::degrees(euler_cur.psi()),
			 (double)math::degrees(roll_err), (double)math::degrees(pitch_err),
			 (double)math::degrees(yaw_err),
			 (double)tau_des(0), (double)tau_des(1), (double)tau_des(2));
	}
}

int GeoControl::att_test_command(int argc, char *argv[])
{
	// 用法: geo_control att_test <roll_deg> <pitch_deg> <yaw_deg> [dur_s]
	if (argc < 2) {
		PX4_INFO_RAW("用法: geo_control att_test ROLL PITCH YAW [dur_s]\n");
		PX4_INFO_RAW("  角度单位度，跟踪期望姿态。\n");
		PX4_INFO_RAW("  自动添加悬停推力用于飞行测试。\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		_att_test_active = false;
		resetIntegrators();
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
		PX4_INFO("ATT_TEST: 已停止");
		return 0;
	}

	// 解析 roll, pitch, yaw (度)
	_att_test_roll  = math::radians(atof(argv[1]));
	_att_test_pitch = (argc >= 3) ? math::radians(atof(argv[2])) : 0.0;
	_att_test_yaw   = (argc >= 4) ? math::radians(atof(argv[3])) : 0.0;

	float dur_s = 10.f;

	if (argc >= 5) { dur_s = atof(argv[4]); }

	_offboard_test_dur_us = (uint64_t)(dur_s * 1e6f);
	_offboard_test_start = hrt_absolute_time();
	resetIntegrators();
	_att_test_active = true;

	PX4_INFO("ATT_TEST: 目标(r=%.0f,p=%.0f,y=%.0f)° 持续=%.0fs",
		 (double)atof(argv[1]),
		 (double)(argc >= 3 ? atof(argv[2]) : 0.0),
		 (double)(argc >= 4 ? atof(argv[3]) : 0.0),
		 (double)dur_s);
	return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// 速度测试: 跟踪期望机体速度
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_vel_test(uint64_t timestamp_sample)
{
	vehicle_local_position_s local_pos;

	if (!_sub_vehicle_local_position.update(&local_pos)) { return; }

	vehicle_attitude_s att;
	vehicle_angular_velocity_s angular_vel;

	if (!_sub_vehicle_attitude.update(&att) ||
	    !_sub_vehicle_angular_velocity.update(&angular_vel)) {
		return;
	}

	const Quatf R_q(att.q);
	const Dcmf R(R_q);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
	const Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);
	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f), 0.001f, 0.05f);
	const float mass = _param_gc_mass.get();

	// 世界系速度误差
	Vector3f vel_sp_world = R * _vel_test_sp;  // 机体 → 世界
	Vector3f e_v = vel - vel_sp_world;

	// 力控制律: 速度误差 PD + 重力补偿
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kv.emult(e_v) - _vel_int - mass * g;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// 姿态环
	const Eulerf euler_sp(0, 0, 0);
	const Dcmf R_sp(euler_sp);
	const Vector3f omega_sp(0, 0, 0);
	const Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	const Vector3f e_omega = omega - omega_d_in_B;

	updateIntegrators(e_omega, e_v, dt);
	Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);

	publish_wrench(f_des_body, tau_des, timestamp_sample);

	static hrt_abstime last_p = 0;
	hrt_abstime now = hrt_absolute_time();

	if (now - last_p > 500000) {
		last_p = now;
		PX4_INFO("VEL_TEST 指令=(%.1f,%.1f,%.1f) 速度=(%.2f,%.2f,%.2f) 误差=%.2f f=(%.1f,%.1f,%.1f)",
			 (double)_vel_test_sp(0), (double)_vel_test_sp(1), (double)_vel_test_sp(2),
			 (double)vel(0), (double)vel(1), (double)vel(2),
			 (double)e_v.norm(),
			 (double)f_des_body(0), (double)f_des_body(1), (double)f_des_body(2));
	}
}

int GeoControl::vel_test_command(int argc, char *argv[])
{
	// 用法: geo_control vel_test VX VY VZ [dur_s]
	if (argc < 2) {
		PX4_INFO_RAW("用法: geo_control vel_test VX_BODY VY_BODY VZ_BODY [dur_s]\n");
		PX4_INFO_RAW("  速度在机体 FRD 坐标系 (m/s)。\n");
		PX4_INFO_RAW("  Z 负 = 向上。自动重力补偿。\n");
		PX4_INFO_RAW("示例:\n");
		PX4_INFO_RAW("  geo_control vel_test 0 0 -1 10   # 上升 1 m/s, 持续 10s\n");
		PX4_INFO_RAW("  geo_control vel_test 1 0 0 5     # 前飞 1 m/s, 持续 5s\n");
		PX4_INFO_RAW("  geo_control vel_test stop\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		_vel_test_active = false;
		resetIntegrators();
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
		PX4_INFO("VEL_TEST: 已停止");
		return 0;
	}

	if (argc < 4) {
		PX4_ERR("需要 VX_BODY VY_BODY VZ_BODY 三个参数");
		return -1;
	}

	_vel_test_sp(0) = atof(argv[1]);
	_vel_test_sp(1) = atof(argv[2]);
	_vel_test_sp(2) = atof(argv[3]);

	float dur_s = 10.f;

	if (argc >= 5) { dur_s = atof(argv[4]); }

	_offboard_test_dur_us = (uint64_t)(dur_s * 1e6f);
	_offboard_test_start = hrt_absolute_time();
	resetIntegrators();
	_vel_test_active = true;

	PX4_INFO("VEL_TEST: v_body=(%.1f,%.1f,%.1f) 持续=%.0fs",
		 (double)_vel_test_sp(0), (double)_vel_test_sp(1), (double)_vel_test_sp(2), (double)dur_s);
	return 0;
}

int GeoControl::verify_allocation()
{
	// ── 端到端验证 MorphQuad 控制分配 ──
	// 双重对比: (1)前向运动学重构  (2)JSBSim 物理仿真实际力/力矩
	//
	// 链路: geo_control → ControlAllocator → MAVLink → bridge → JSBSim
	//       JSBSim 合力/力矩 → bridge UDP :9871 → geo_control 读取
	//
	// JSBSim FK: t_i = T_i·(sinα cosβ, sinβ, -cosα cosβ),  T_i = esc_i²·TMAX
	// 重构误差小 → 分配器收敛; JSBSim 误差小 → 物理模型与分配器一致
	const float tmax = math::max(_param_ca_mq_tmax.get(), 1.f);
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();

	static constexpr int NQ = 4;
	const float lx[NQ] = { l,  l, -l, -l};
	const float ly[NQ] = { b, -b, -b,  b};
	static constexpr uint16_t WRENCH_PORT = 9871;  // bridge 力/力矩 UDP 反馈端口

	uORB::Subscription sub_motors{ORB_ID(actuator_motors)};
	uORB::Subscription sub_servos{ORB_ID(actuator_servos)};

	// ── 打开 UDP socket 接收 JSBSim 实际力/力矩 ──
	int wrench_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (wrench_fd < 0) {
		PX4_ERR("verify: 无法创建 UDP socket");
		return -1;
	}
	{
		int opt = 1;
		setsockopt(wrench_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
		struct sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		addr.sin_port = htons(WRENCH_PORT);
		if (bind(wrench_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			PX4_ERR("verify: UDP bind :%d 失败 (bridge 是否已启动?)", (int)WRENCH_PORT);
			::close(wrench_fd);
			return -1;
		}
		// 非阻塞模式
		fcntl(wrench_fd, F_SETFL, O_NONBLOCK);
	}

	// 60 组固定 6D 力/力矩: 每个自由度单轴斜坡 (6轴 × 10 个幅值)，
	// 每个控制方向以递增幅值测试。
	const float scale[6] = {80.f, 80.f, 110.f, 18.f, 18.f, 10.f};  // fx,fy,fz,tx,ty,tz

	PX4_INFO("MorphQuad 端到端分配验证 — 60 组固定 6D 力/力矩");
	PX4_INFO("  FK: t_i = T_i·(sinα cosβ, sinβ, -cosα cosβ),  T_i = esc_i²·TMAX = %.2f N", (double)tmax);
	PX4_INFO("  JSBSim 反馈: UDP :%d (bridge 力/力矩)", (int)WRENCH_PORT);

	_verify_active = true;

	// 记录上一帧 JSBSim 时间戳，用于对齐检测
	double jsb_timestamp_prev = 0.0;

	for (int k = 0; k < 60; ++k) {
		const int axis = k / 10;          // 0..5
		const int m = (k % 10) + 1;       // 1..10
		matrix::Vector<float, 6> W{};
		W(axis) = (float)m / 10.f * scale[axis];

		_verify_wrench = W;

		// ── 数据对齐: 等管道稳定 + 取新时间戳帧 ──
		// 链路: geo_control → allocator → MAVLink → bridge → server → JSBSim
		//       → server FK → bridge UDP → geo_control。往返 ~30-40ms。
		double jsb_wrench[6] = {};
		double jsb_timestamp = 0.0;
		bool jsb_valid = false;
		{
			struct pollfd pfd{};
			pfd.fd = wrench_fd;
			pfd.events = POLLIN;

			// (1) 排空缓冲区 (旧数据)
			double pkt[7];
			while (::poll(&pfd, 1, 0) > 0) {
				recvfrom(wrench_fd, pkt, sizeof(pkt), 0, nullptr, nullptr);
				pfd.revents = 0;
			}

			// (2) 等管道稳定 + 新数据到达 (最多等 300ms)
			bool aligned = false;
			for (int retry = 0; retry < 30; retry++) {
				px4_usleep(10000);  // 10ms

				while (::poll(&pfd, 1, 0) > 0) {
					ssize_t n = recvfrom(wrench_fd, pkt, sizeof(pkt), 0, nullptr, nullptr);
					if (n == (ssize_t)sizeof(pkt) && pkt[0] > jsb_timestamp_prev) {
						jsb_timestamp = pkt[0];
						memcpy(jsb_wrench, &pkt[1], sizeof(jsb_wrench));
						jsb_valid = true;
						aligned = true;
					}
					pfd.revents = 0;
				}

				if (aligned) break;
			}
		}
		jsb_timestamp_prev = jsb_timestamp;

		// ── 读取执行器输出 (FK 重构) ──
		actuator_motors_s motors{};
		actuator_servos_s servos{};
		sub_motors.copy(&motors);
		sub_servos.copy(&servos);

		matrix::Vector3f f_ach(0.f, 0.f, 0.f);
		matrix::Vector3f tau_ach(0.f, 0.f, 0.f);

		PX4_INFO_RAW("[%2d] 指令 F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)\n",
			     k, (double)W(0), (double)W(1), (double)W(2),
			     (double)W(3), (double)W(4), (double)W(5));

		// 逐旋翼明细
		for (int i = 0; i < NQ; ++i) {
			const float esc = (i < actuator_motors_s::NUM_CONTROLS) ? motors.control[i] : NAN;
			const float alpha = (2 * i < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i] * M_PI_F : NAN;
			const float beta  = ((2 * i + 1) < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i + 1] * M_PI_F : NAN;

			const float T = (PX4_ISFINITE(esc) ? esc * esc : 0.f) * tmax;
			const float sa = sinf(alpha), ca = cosf(alpha);
			const float sb = sinf(beta),  cb = cosf(beta);
			const matrix::Vector3f t_vec(T * sa * cb, T * sb, -T * ca * cb);

			f_ach += t_vec;
			const matrix::Vector3f r_i(lx[i], ly[i], 0.f);
			tau_ach += r_i.cross(t_vec);

			PX4_INFO_RAW("    rotor%d: esc=%6.3f  a=%7.2f deg  b=%7.2f deg  |T|=%7.2f N\n",
				     i, (double)esc, (double)math::degrees(alpha),
				     (double)math::degrees(beta), (double)T);
		}

		// ── FK 重构对比 ──
		float err_fk = 0.f;
		{
			matrix::Vector<float, 6> Wfk{};
			Wfk(0)=f_ach(0); Wfk(1)=f_ach(1); Wfk(2)=f_ach(2);
			Wfk(3)=tau_ach(0); Wfk(4)=tau_ach(1); Wfk(5)=tau_ach(2);
			for (int j=0; j<6; ++j) { float e=W(j)-Wfk(j); err_fk += e*e; }
			err_fk = sqrtf(err_fk);
		}

		// ── JSBSim 物理对比 ──
		double err_jsb = -1.0;
		if (jsb_valid) {
			err_jsb = 0.0;
			for (int j = 0; j < 6; ++j) {
				double e = (double)W(j) - jsb_wrench[j];
				err_jsb += e * e;
			}
			err_jsb = sqrt(err_jsb);
		}

		PX4_INFO_RAW("    FK:  F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  |err|=%7.3f\n",
			     (double)f_ach(0), (double)f_ach(1), (double)f_ach(2),
			     (double)tau_ach(0), (double)tau_ach(1), (double)tau_ach(2),
			     (double)err_fk);

		if (jsb_valid) {
			PX4_INFO_RAW("    JSB: F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  |err|=%7.3f  t=%.3fs\n",
				     jsb_wrench[0], jsb_wrench[1], jsb_wrench[2],
				     jsb_wrench[3], jsb_wrench[4], jsb_wrench[5],
				     (double)err_jsb, jsb_timestamp);
		} else {
			PX4_INFO_RAW("    JSB: (无数据 — bridge 力/力矩 UDP 反馈未到达)\n");
		}
	}

	::close(wrench_fd);
	_verify_active = false;
	PX4_INFO("验证完成 — 注入已关闭，控制器恢复");
	return 0;
}

void GeoControl::control_step_acro(uint64_t timestamp_sample)
{
	// 复位保持: 发布一个小悬停偏置力 (Fz=-5N ≈ 10% 悬停) 以主动
	// 将舵机角拉回 α=β=0。纯零力/力矩会让舵机因 e=0 → dθ=0
	// 而冻结，需要弱向下偏置实现主动收敛。
	if (_acro_reset_hold > 0) {
		_acro_reset_hold--;
		publish_wrench(Vector3f(0,0,-5), Vector3f(0,0,0), timestamp_sample);
		return;
	}

	Vector3f f_cmd(_acro_wrench(0), _acro_wrench(1), _acro_wrench(2));
	Vector3f tau_cmd(_acro_wrench(3), _acro_wrench(4), _acro_wrench(5));

	publish_wrench(f_cmd, tau_cmd, timestamp_sample);

	// 从控制分配器实际输出读回力/力矩。
	// 前向运动学重构使用与 ActuatorEffectivenessMorphQuad
	// 和 JSBSim morphquad.xml 相同的公式:
	//   t_i = T_i * (sinα cosβ, sinβ, -cosα cosβ),  T_i = esc_i² * TMAX
	const float tmax = math::max(_param_ca_mq_tmax.get(), 1.f);
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();

	static constexpr int NQ = 4;
	const float lx[NQ] = { l,  l, -l, -l};
	const float ly[NQ] = { b, -b, -b,  b};

	uORB::Subscription sub_motors{ORB_ID(actuator_motors)};
	uORB::Subscription sub_servos{ORB_ID(actuator_servos)};

	actuator_motors_s motors{};
	actuator_servos_s servos{};
	sub_motors.copy(&motors);
	sub_servos.copy(&servos);

	Vector3f f_ach(0.f, 0.f, 0.f);
	Vector3f tau_ach(0.f, 0.f, 0.f);
	float esc_vals[NQ] = {};
	float alpha_vals[NQ] = {};
	float beta_vals[NQ] = {};

	for (int i = 0; i < NQ; ++i) {
		const float esc = (i < actuator_motors_s::NUM_CONTROLS) ? motors.control[i] : NAN;
		const float alpha = (2 * i < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i] * M_PI_F : NAN;
		const float beta  = ((2 * i + 1) < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i + 1] * M_PI_F : NAN;

		esc_vals[i] = esc;
		alpha_vals[i] = alpha;
		beta_vals[i] = beta;

		const float T = (PX4_ISFINITE(esc) ? esc * esc : 0.f) * tmax;
		const float sa = sinf(alpha), ca = cosf(alpha);
		const float sb = sinf(beta),  cb = cosf(beta);
		const Vector3f t_vec(T * sa * cb, T * sb, -T * ca * cb);

		f_ach += t_vec;
		const Vector3f r_i(lx[i], ly[i], 0.f);
		tau_ach += r_i.cross(t_vec);
	}

	// 每 0.5s 打印 (节流避免日志刷屏)
	static hrt_abstime _last_acro_print = 0;
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_acro_print > 500000) {
		_last_acro_print = now;

		PX4_INFO_RAW("=== ACRO 力/力矩 [机体 FRD] ===\n");
		PX4_INFO_RAW("  指令  F=(%8.2f,%8.2f,%8.2f) N  T=(%6.2f,%6.2f,%6.2f) Nm\n",
			     (double)f_cmd(0), (double)f_cmd(1), (double)f_cmd(2),
			     (double)tau_cmd(0), (double)tau_cmd(1), (double)tau_cmd(2));
		PX4_INFO_RAW("  实际  F=(%8.2f,%8.2f,%8.2f) N  T=(%6.2f,%6.2f,%6.2f) Nm\n",
			     (double)f_ach(0), (double)f_ach(1), (double)f_ach(2),
			     (double)tau_ach(0), (double)tau_ach(1), (double)tau_ach(2));

		float err_f = (f_cmd - f_ach).norm();
		float err_tau = (tau_cmd - tau_ach).norm();
		PX4_INFO_RAW("  误差  |ΔF|=%.3f N  |Δτ|=%.3f Nm\n", (double)err_f, (double)err_tau);

		for (int i = 0; i < NQ; ++i) {
			PX4_INFO_RAW("  rotor%d: esc=%-6.3f  α=%7.1f°  β=%7.1f°\n",
				     i, (double)esc_vals[i],
				     (double)math::degrees(alpha_vals[i]),
				     (double)math::degrees(beta_vals[i]));
		}
	}
}

int GeoControl::acro_command(int argc, char *argv[])
{
	// 用法:
	//   geo_control acro Fx Fy Fz Tx Ty Tz [duration_s]
	//   geo_control acro stop
	if (argc < 2) {
		PX4_INFO_RAW("用法:\n");
		PX4_INFO_RAW("  geo_control acro Fx Fy Fz Tx Ty Tz [duration_s]\n");
		PX4_INFO_RAW("  geo_control acro stop\n");
		PX4_INFO_RAW("  (Fx/Fy/Fz 单位 N, Tx/Ty/Tz 单位 Nm, 机体 FRD 坐标系)\n");
		PX4_INFO_RAW("  示例:\n");
		PX4_INFO_RAW("    geo_control acro 0 0 -44.7 0 0 0   # 悬停推力 (4.56kg)\n");
		PX4_INFO_RAW("    geo_control acro 10 0 -44.7 0 0 0  # 悬停 + 10N 前向\n");
		PX4_INFO_RAW("    geo_control acro 0 0 -44.7 1 0 0 5 # 悬停 + 1Nm roll, 5s\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		_acro_active = false;
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
		PX4_INFO("ACRO: 已停止");
		return 0;
	}

	if (argc < 7) {
		PX4_ERR("ACRO: 需要 6 个力/力矩值 (Fx Fy Fz Tx Ty Tz)");
		return -1;
	}

	float fx = atof(argv[1]);
	float fy = atof(argv[2]);
	float fz = atof(argv[3]);
	float tx = atof(argv[4]);
	float ty = atof(argv[5]);
	float tz = atof(argv[6]);

	_acro_wrench(0) = fx;
	_acro_wrench(1) = fy;
	_acro_wrench(2) = fz;
	_acro_wrench(3) = tx;
	_acro_wrench(4) = ty;
	_acro_wrench(5) = tz;

	_acro_duration_us = 0;

	if (argc >= 8) {
		float dur_s = atof(argv[7]);
		if (dur_s > 0) {
			_acro_duration_us = (uint64_t)(dur_s * 1e6f);
		}
	}

	_acro_reset_hold = ACRO_RESET_HOLD;   // 先短暂零力/力矩复位舵机
	_acro_active = true;
	_acro_start_time = hrt_absolute_time();

	PX4_INFO("ACRO: ON  F=(%.1f,%.1f,%.1f)N  T=(%.2f,%.2f,%.2f)Nm  %s",
		 (double)fx, (double)fy, (double)fz,
		 (double)tx, (double)ty, (double)tz,
		 _acro_duration_us > 0 ? "(定时)" : "(持续, 用 'acro stop' 停止)");
	return 0;
}

void GeoControl::control_step_sweep(uint64_t timestamp_sample)
{
	// 复位保持: 发布一个小悬停偏置力主动将舵机拉回 α=β=0
	if (_sweep_reset_hold > 0) {
		_sweep_reset_hold--;
		publish_wrench(Vector3f(0,0,-5), Vector3f(0,0,0), timestamp_sample);
		return;
	}

	// 计算当前采样点的力/力矩值
	const float fraction = (_sweep_samples > 1)
		? (float)_sweep_step / (float)(_sweep_samples - 1)
		: 1.0f;
	const float value = fraction * _sweep_max[_sweep_axis];

	matrix::Vector<float, 6> W{};
	W(_sweep_axis) = value;

	Vector3f f_cmd(W(0), W(1), W(2));
	Vector3f tau_cmd(W(3), W(4), W(5));
	publish_wrench(f_cmd, tau_cmd, timestamp_sample);

	// 保持 SWEEP_HOLD 周期后再读回
	if (_sweep_hold_count < SWEEP_HOLD) {
		_sweep_hold_count++;
		return;
	}

	// 读回实际力/力矩
	const float tmax = math::max(_param_ca_mq_tmax.get(), 1.f);
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	static constexpr int NQ = 4;
	const float lx[NQ] = { l,  l, -l, -l};
	const float ly[NQ] = { b, -b, -b,  b};

	uORB::Subscription sub_motors{ORB_ID(actuator_motors)};
	uORB::Subscription sub_servos{ORB_ID(actuator_servos)};
	actuator_motors_s motors{};
	actuator_servos_s servos{};
	sub_motors.copy(&motors);
	sub_servos.copy(&servos);

	Vector3f f_ach(0,0,0), tau_ach(0,0,0);
	float esc_vals[NQ]{}, alpha_deg[NQ]{}, beta_deg[NQ]{};

	for (int i = 0; i < NQ; ++i) {
		const float esc = (i < actuator_motors_s::NUM_CONTROLS) ? motors.control[i] : NAN;
		const float alpha = (2*i < actuator_servos_s::NUM_CONTROLS) ? servos.control[2*i] * M_PI_F : NAN;
		const float beta  = ((2*i+1) < actuator_servos_s::NUM_CONTROLS) ? servos.control[2*i+1] * M_PI_F : NAN;
		esc_vals[i] = esc; alpha_deg[i] = math::degrees(alpha); beta_deg[i] = math::degrees(beta);

		const float T = (PX4_ISFINITE(esc) ? esc*esc : 0.f) * tmax;
		const float sa = sinf(alpha), ca = cosf(alpha), sb = sinf(beta), cb = cosf(beta);
		const Vector3f t_vec(T * sa * cb, T * sb, -T * ca * cb);
		f_ach += t_vec;
		tau_ach += Vector3f(lx[i], ly[i], 0.f).cross(t_vec);
	}

	static const char *axis_names[6] = {"Fx","Fy","Fz","Tx","Ty","Tz"};
	static const char *axis_units[6] = {"N","N","N","Nm","Nm","Nm"};

	PX4_INFO_RAW("[sweep %s %5.1f/%5.1f %s] 指令(%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%6.2f) "
		     "实际(%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%6.2f) "
		     "误差(|ΔF|=%6.3f |Δτ|=%6.3f) "
		     "esc[%4.2f,%4.2f,%4.2f,%4.2f] "
		     "α[%6.1f,%6.1f,%6.1f,%6.1f]° "
		     "β[%6.1f,%6.1f,%6.1f,%6.1f]°\n",
		     axis_names[_sweep_axis], (double)value, (double)_sweep_max[_sweep_axis], axis_units[_sweep_axis],
		     (double)W(0),(double)W(1),(double)W(2),(double)W(3),(double)W(4),(double)W(5),
		     (double)f_ach(0),(double)f_ach(1),(double)f_ach(2),
		     (double)tau_ach(0),(double)tau_ach(1),(double)tau_ach(2),
		     (double)(f_cmd-f_ach).norm(), (double)(tau_cmd-tau_ach).norm(),
		     (double)esc_vals[0],(double)esc_vals[1],(double)esc_vals[2],(double)esc_vals[3],
		     (double)alpha_deg[0],(double)alpha_deg[1],(double)alpha_deg[2],(double)alpha_deg[3],
		     (double)beta_deg[0],(double)beta_deg[1],(double)beta_deg[2],(double)beta_deg[3]);

	// 推进到下一步
	_sweep_step++;
	_sweep_hold_count = 0;

	if (_sweep_step >= _sweep_samples) {
		// 当前轴完成
		_sweep_step = 0;
		_sweep_axis++;

		if (_sweep_axis >= 6) {
			// 全部 6 轴完成
			_sweep_active = false;
			PX4_INFO("SWEEP 完成: 全部 6 轴已测试");
			publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), timestamp_sample);
			return;
		}

		PX4_INFO("SWEEP: 开始轴 %d (%s), 最大值=%.2f %s", _sweep_axis,
			 axis_names[_sweep_axis], (double)_sweep_max[_sweep_axis], axis_units[_sweep_axis]);
	}
}

int GeoControl::sweep_command(int argc, char *argv[])
{
	// 用法:
	//   geo_control sweep <samples> <Fx_max> <Fy_max> <Fz_max> <Tx_max> <Ty_max> <Tz_max>
	//   geo_control sweep stop
	if (argc < 2) {
		PX4_INFO_RAW("用法: geo_control sweep SAMPLES Fx_max Fy_max Fz_max Tx_max Ty_max Tz_max\n");
		PX4_INFO_RAW("  SAMPLES: 每轴采样点数 (如 10 = 0%%,11%%,...,100%% of max)\n");
		PX4_INFO_RAW("  Fx/y/z_max: 各轴最大力 [N]\n");
		PX4_INFO_RAW("  Tx/y/z_max: 各轴最大力矩 [Nm]\n");
		PX4_INFO_RAW("每轴独立测试 (0 → max)。共 6×SAMPLES 个采样点。\n");
		PX4_INFO_RAW("示例:\n");
		PX4_INFO_RAW("  geo_control sweep 11 80 80 110 18 18 10   # 11 点 × 6 轴 = 66 次\n");
		PX4_INFO_RAW("  geo_control sweep 5  40 0  44  0  0  0    # 5 点, 仅 Fx+Fz\n");
		PX4_INFO_RAW("每个采样点打印: 指令/实际力/力矩, |ΔF|, |Δτ|, esc, α/β 角\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		_sweep_active = false;
		publish_wrench(Vector3f(0,0,0), Vector3f(0,0,0), hrt_absolute_time());
		PX4_INFO("SWEEP: 已停止");
		return 0;
	}

	if (argc < 8) {
		PX4_ERR("SWEEP: 需要 7 个参数: SAMPLES Fx_max Fy_max Fz_max Tx_max Ty_max Tz_max");
		return -1;
	}

	_sweep_samples = atoi(argv[1]);
	if (_sweep_samples < 2) { _sweep_samples = 2; }
	if (_sweep_samples > 100) { _sweep_samples = 100; }

	_sweep_max[0] = atof(argv[2]);  // Fx
	_sweep_max[1] = atof(argv[3]);  // Fy
	_sweep_max[2] = atof(argv[4]);  // Fz
	_sweep_max[3] = atof(argv[5]);  // Tx
	_sweep_max[4] = atof(argv[6]);  // Ty
	_sweep_max[5] = atof(argv[7]);  // Tz

	_sweep_axis = 0;
	_sweep_step = 0;
	_sweep_hold_count = 0;
	_sweep_reset_hold = SWEEP_RESET_HOLD;  // 先短暂零力/力矩复位舵机
	_sweep_active = true;

	PX4_INFO("SWEEP: %d 点/轴 × 6 轴 = %d 次测试", _sweep_samples, _sweep_samples * 6);
	PX4_INFO("  Fx_max=%.1f Fy_max=%.1f Fz_max=%.1f  Tx_max=%.2f Ty_max=%.2f Tz_max=%.2f",
		 (double)_sweep_max[0], (double)_sweep_max[1], (double)_sweep_max[2],
		 (double)_sweep_max[3], (double)_sweep_max[4], (double)_sweep_max[5]);
	PX4_INFO("  开始轴 0 (Fx)...");

	return 0;
}

int GeoControl::print_status()
{
	PX4_INFO("运行中");

	// 诊断: 解锁后力/力矩为零通常表示控制器处于不处理的模式
	// (nav_state 非 MANUAL/POSCTL/OFFBOARD → AUTO，后者需要
	// flight_mode_manager 指令) 或未曾观察到 arming_state==ARMED
	PX4_INFO("armed: %d, landed: %d, nav_state: %u, mass: %.3f",
		 _armed, _landed, (unsigned)_nav_state_last, (double)_param_gc_mass.get());

	// 最近一次期望机体力/力矩 (FRD 坐标系)
	const float f_mag = _f_des_last.norm();
	const float tau_mag = _tau_des_last.norm();
	const float age_ms = (_wrench_timestamp > 0) ?
			     (hrt_absolute_time() - _wrench_timestamp) / 1000.f : -1.f;

	PX4_INFO_RAW("MorphQuad 期望力/力矩 (机体 FRD 坐标系):\n");
	PX4_INFO_RAW("  力     [N ]:  x=%9.3f  y=%9.3f  z=%9.3f   |F|=%9.3f\n",
		     (double)_f_des_last(0), (double)_f_des_last(1),
		     (double)_f_des_last(2), (double)f_mag);
	PX4_INFO_RAW("  力矩   [Nm]:  x=%9.3f  y=%9.3f  z=%9.3f   |t|=%9.3f\n",
		     (double)_tau_des_last(0), (double)_tau_des_last(1),
		     (double)_tau_des_last(2), (double)tau_mag);
	PX4_INFO_RAW("  (%.1f ms 前更新)\n", (double)age_ms);

	return 0;
}

int GeoControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
MorphQuad 几何控制器，输出推力与力矩指令。

### 实现
在 SE(3) 上实现几何跟踪控制器，支持三种输入模式:
- **手动**: RC 摇杆 (roll/pitch/yaw → 角速率, aux1/2/3 → 机体速度)
- **Offboard**: 来自 companion computer 的 trajectory_setpoint (位置+姿态)
  或 SET_ATTITUDE_TARGET (仅姿态，回退到悬停推力+速度阻尼)
- **Auto**: flight_mode_manager 指令 (posctl, loiter, mission, ...)

积分器含抗饱和:
- 角速率/力矩积分器: 累积姿态误差, 受 GC_RATE_INT_LIM 限制
- 速度/力积分器: 累积速度误差, 受 GC_VEL_INT_LIM 限制
- 通过 control_allocator_status 饱和反馈实现抗饱和
- 着陆时冻结积分器，未解锁时清零

### 示例
$ geo_control start
$ geo_control status
$ geo_control verify
$ geo_control stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("geo_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("verify", "注入 60 组固定 6D 力/力矩, 打印各旋翼电机/舵机输出");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int geo_control_main(int argc, char *argv[])
{
	return GeoControl::main(argc, argv);
}
