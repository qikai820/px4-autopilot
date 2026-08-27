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
#include <cmath>
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
	if (!_sub_vehicle_attitude_setpoint_event.registerCallback() ||
	    !_sub_vehicle_attitude_event.registerCallback() ||
	    !_sub_vehicle_local_position_event.registerCallback()) {
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
	_att_int.zero();
	_vel_int.zero();
	_vel_int_z_accel = 0.f;
}

void GeoControl::updateIntegrators(const Vector3f &e_omega, const Vector3f &e_v_world, float dt)
{
	// Never accumulate control error while the vehicle is on the ground. This
	// also protects modes whose callers may be running during land-detector
	// transitions; all integrators are reset every landed control cycle.
	if (_landed) {
		_rate_int.zero();
		_att_int.zero();
		_vel_int.zero();
		_vel_int_z_accel = 0.f;
		return;
	}

	const Vector3f rate_i_gain(_param_gc_roll_rate_i.get(), _param_gc_pitch_rate_i.get(),
					   _param_gc_yaw_rate_i.get());
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

		// e_omega = omega - omega_sp, while the control law applies the
		// integral torque directly. Therefore the integral must oppose the
		// measured rate error, just like the proportional rate term.
		float rate_i = _rate_int(i) - i_factor * rate_i_gain(i) * rate_error * dt;

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

			if (i == 2) {
				// PX4 multicopter convention: vertical velocity I is an
				// acceleration correction, not a force correction.
				_vel_int_z_accel = _vel_int(i);
			}
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
				   const Vector3f &vel_sp_world, const Vector3f &accel_sp_world,
				   const Vector3f &omega,
				   const Vector3f &omega_sp,
				   uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();
	const float thrust_max = math::max(_param_gc_thrust_max.get(), 1.f);

	// ── 归一化总推力估计 (总力占比) ──
	// throttle_norm = ‖f_des_world‖ / (N_rotors * TMAX) = ‖f‖/GC_THRUST_MAX。
	// 这是总力占比，而非单路电机指令。悬停时 ‖f‖ = mass*g，
	// 因此 throttle_norm ≈ 0.28 (44.7/158)。实际单路电机指令为
	// 归一化转速 sqrt(T_i/TMAX) ≈ 0.53 (悬停)，参见 ActuatorEffectivenessMorphQuad。
	const float throttle_norm = math::constrain(f_des_world.norm() / thrust_max, 0.f, 1.f);

	// ── vehicle_local_position_setpoint (位置/速度调试 + 油门估计) ──
	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.vx = vel_sp_world(0);
	local_pos_sp.vy = vel_sp_world(1);
	local_pos_sp.vz = vel_sp_world(2);
	local_pos_sp.acceleration[0] = accel_sp_world(0);
	local_pos_sp.acceleration[1] = accel_sp_world(1);
	local_pos_sp.acceleration[2] = accel_sp_world(2);
	local_pos_sp.thrust[0] = f_des_body(0) / thrust_max;
	local_pos_sp.thrust[1] = f_des_body(1) / thrust_max;
	local_pos_sp.thrust[2] = throttle_norm;
	local_pos_sp.yaw = Eulerf(q_att_ref).psi();
	local_pos_sp.yawspeed = 0.f;
	_local_pos_sp_pub.publish(local_pos_sp);

	// Dedicated debug topic for the geometric controller's desired attitude.
	// This must not use vehicle_attitude_setpoint, which is an Offboard input.
	geo_control_attitude_setpoint_s attitude_sp_debug{};
	attitude_sp_debug.timestamp = now;
	attitude_sp_debug.timestamp_sample = timestamp_sample;
	for (int i = 0; i < 4; ++i) { attitude_sp_debug.q[i] = q_att_ref(i); }
	const Eulerf euler_sp(q_att_ref);
	attitude_sp_debug.roll = euler_sp.phi();
	attitude_sp_debug.pitch = euler_sp.theta();
	attitude_sp_debug.yaw = euler_sp.psi();
	attitude_sp_debug.rates[0] = omega_sp(0);
	attitude_sp_debug.rates[1] = omega_sp(1);
	attitude_sp_debug.rates[2] = omega_sp(2);
	_attitude_sp_debug_pub.publish(attitude_sp_debug);

	// vehicle_attitude_setpoint is an Offboard input topic. Do not republish
	// debug data on it: otherwise the next Offboard cycle can consume our own
	// positive-magnitude debug thrust as an external MAVLink thrust command
	// (where negative Z means upward thrust), flipping the vertical force.

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
		const Vector3f &omega, const Vector3f & /* omega_sp */,
		const Vector3f &omega_d_in_B, float dt)
{
	// 姿态误差 (Lee SO(3)): e_R = 1/2 (Rdᵀ R − Rᵀ Rd)^∨
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	const Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_roll_p.get(), _param_gc_pitch_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_roll_rate_p.get(), _param_gc_pitch_rate_p.get(), _param_gc_yaw_rate_p.get());
	Matrix3f J = matrix::diag(Vector3f(_param_gc_inertia_xx.get(), _param_gc_inertia_yy.get(),
					  _param_gc_inertia_zz.get()));

	// Do not differentiate the external desired rate. SET_ATTITUDE_TARGET is
	// normally 50 Hz while this controller follows the state callback rate;
	// finite-differencing it produced large, non-physical torque impulses.
	// Integrate the attitude error separately from the rate error. The rate
	// error can be zero while a constant aerodynamic/allocation torque bias
	// still leaves a non-zero attitude error.
	if (!_landed && PX4_ISFINITE(dt) && dt > 0.f && dt <= 0.05f) {
		const float att_i = _param_gc_att_i.get();
		const float att_lim = _param_gc_rate_int_lim.get();
		for (int i = 0; i < 3; ++i) {
			const float v = _att_int(i) - att_i * e_R(i) * dt;
			_att_int(i) = PX4_ISFINITE(v) ? math::constrain(v, -att_lim, att_lim) : 0.f;
		}
	}

	const Vector3f ff = -J * omega.cross(omega_d_in_B);

	return -kR.emult(e_R) - kOmega.emult(omega - omega_d_in_B) + _rate_int + _att_int
	       + omega.cross(J * omega) + ff;
}

// ─────────────────────────────────────────────────────────────────────────────
// 手动模式: RC 摇杆 → 角速率 + 速度参考
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::update_manual_level_hold(const Quatf &R_q, bool enabled)
{
	if (enabled && !_manual_level_hold_active) {
		// AUX3 is a level-hold switch. Remove roll and pitch immediately while
		// preserving the heading at the instant the switch is engaged.
		const float yaw = Eulerf(R_q).psi();
		_q_att_ref = Quatf(Eulerf(0.f, 0.f, yaw));
		mavlink_log_info(&_mavlink_log_pub, "Geo control: level hold enabled");

	} else if (!enabled && _manual_level_hold_active) {
		mavlink_log_info(&_mavlink_log_pub, "Geo control: level hold disabled");
	}

	_manual_level_hold_active = enabled;
}

void GeoControl::limit_manual_attitude_reference()
{
	Eulerf attitude_ref(_q_att_ref);
	const float roll_limit = math::radians(math::constrain(_param_gc_roll_max.get(), 1.f, 85.f));
	const float pitch_limit = math::radians(math::constrain(_param_gc_pitch_max.get(), 1.f, 85.f));
	const float roll = math::constrain(attitude_ref.phi(), -roll_limit, roll_limit);
	const float pitch = math::constrain(attitude_ref.theta(), -pitch_limit, pitch_limit);
	_q_att_ref = Quatf(Eulerf(roll, pitch, attitude_ref.psi()));
	_q_att_ref.normalize();
}

void GeoControl::control_step_manual(const Quatf &R_q, const Vector3f &omega,
				     uint64_t timestamp_sample)
{
	// 手动控制: 当无有效 RC/摇杆输入时 (SITL/HIL 常见)，
	// 不回退为零力/力矩，而是退回到零摇杆指令，
	// 使速度控制器仍输出悬停推力 (-mass*g)。
	// 这样 vehicle_thrust_setpoint 保持非零，避免 land detector
	// 判定"已着陆"而自动上锁。
	manual_control_setpoint_s manual_sp{};
	const bool has_manual = _sub_manual_control_sp.copy(&manual_sp) && manual_sp.valid
					&& (hrt_elapsed_time(&manual_sp.timestamp) < 500_ms);

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);

	const float rate_max = _param_gc_man_rate_max.get();
	const float dz = _param_gc_man_dz.get();
	const float mass = math::max(_param_gc_mass.get(), 0.1f);

	float p_sp = 0.f;
	float q_sp = 0.f;
	float r_sp = 0.f;
	float fx_body = 0.f;
	float fy_body = 0.f;
	float fz_body = -mass * 9.81f;   // 无摇杆输入时保持悬停推力 (向上)

	if (has_manual) {
		const bool level_hold = manual_sp.aux3 > 0.5f;
		update_manual_level_hold(R_q, level_hold);

		p_sp = math::deadzone(manual_sp.aux1,  dz) * rate_max;
		q_sp = math::deadzone(manual_sp.aux2, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;
		// Do not allow yaw-stick rotation while the vehicle is still landed.
		if (_landed) { r_sp = 0.f; }


		const float fmax = _param_gc_man_fmax.get();
		fx_body = math::deadzone(manual_sp.pitch, dz) * fmax;
		fy_body = math::deadzone(manual_sp.roll, dz) * fmax;
		const float throttle = math::constrain((manual_sp.throttle + 1.f) * 0.5f, 0.f, 1.f);
		const float thrust_max = math::max(_param_gc_thrust_max.get(), 0.f);
		fz_body = -thrust_max * throttle * throttle;

		if (level_hold) {
			// Ignore attitude and horizontal-force sticks while level hold is
			// active. Keep the throttle live so vertical control remains usable.
			p_sp = 0.f;
			q_sp = 0.f;
			fx_body = 0.f;
			fy_body = 0.f;
		}

	} else {
		update_manual_level_hold(R_q, false);
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);

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

	limit_manual_attitude_reference();

	// ── 姿态环 → 期望力矩 (闭环, Lee SO(3) 含前馈) ──
	const Dcmf R(R_q);
	// 集体推力保持在世界坐标系竖直方向。飞机滚转/俯仰时，
	// 这里自动生成相反的机体系 Fx/Fy，使推力矢量不随机体倾斜。
	// aux1/aux2 的水平开环力仍叠加在机体系中。
	const Vector3f f_vertical_body = R.transpose() * Vector3f(0.f, 0.f, fz_body);
	const Vector3f f_des_body = f_vertical_body + Vector3f(fx_body, fy_body, 0.f);
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
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, Vector3f(0.f, 0.f, 0.f),
			  Vector3f(0.f, 0.f, 0.f), omega, omega_sp, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// 定高模式: 姿态/水平力手动，油门控制垂向速度闭环
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_altitude(const Quatf &R_q, const Vector3f &vel,
					       const Vector3f &omega, uint64_t timestamp_sample)
{
	manual_control_setpoint_s manual_sp{};
	const bool has_manual = _sub_manual_control_sp.copy(&manual_sp) && manual_sp.valid
					&& (hrt_elapsed_time(&manual_sp.timestamp) < 500_ms);
	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);
	const float rate_max = _param_gc_man_rate_max.get();
	const float dz = _param_gc_man_dz.get();
	const float fmax = _param_gc_man_fmax.get();
	const float vz_max = math::max(_param_gc_vz_max.get(), 0.1f);

	float p_sp = 0.f;
	float q_sp = 0.f;
	float r_sp = 0.f;
	float fx_body = 0.f;
	float fy_body = 0.f;
	float vz_sp = 0.f;

	if (has_manual) {
		const bool level_hold = manual_sp.aux3 > 0.5f;
		update_manual_level_hold(R_q, level_hold);
		// Keep the exact Manual mapping: AUX1/AUX2 command roll/pitch rates.
		p_sp = math::deadzone(manual_sp.aux1, dz) * rate_max;
		q_sp = math::deadzone(manual_sp.aux2, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw, dz) * rate_max;
		if (_landed) { r_sp = 0.f; }
		fx_body = math::deadzone(manual_sp.pitch, dz) * fmax;
		fy_body = math::deadzone(manual_sp.roll, dz) * fmax;
		// Match Position mode: positive throttle commands a climb.
		vz_sp = -math::deadzone(manual_sp.throttle, dz) * vz_max;

		if (level_hold) {
			p_sp = 0.f;
			q_sp = 0.f;
			fx_body = 0.f;
			fy_body = 0.f;
		}

	} else {
		update_manual_level_hold(R_q, false);
	}

	const Vector3f omega_sp(p_sp, q_sp, r_sp);
	const float omega_norm = omega_sp.norm();

	if (omega_norm > 1e-6f) {
		const float angle = omega_norm * dt;
		const Vector3f axis = omega_sp / omega_norm;
		const float ha = angle * 0.5f;
		_q_att_ref = _q_att_ref * Quatf(cosf(ha), axis(0) * sinf(ha), axis(1) * sinf(ha), axis(2) * sinf(ha));
		_q_att_ref.normalize();
	}

	limit_manual_attitude_reference();

	const Dcmf R(R_q);
	const float thrust_max = math::max(_param_gc_thrust_max.get(), 1.f);
	const float hover_thr = math::constrain(_param_gc_hover_thr.get(), 0.05f, 0.95f);
	const float hover_force = hover_thr * thrust_max;

	// Slew-limit the vertical velocity setpoint, as in Position mode.
	if (!_vel_sp_limited_init) {
		_vel_sp_limited(0) = vel(0);
		_vel_sp_limited(1) = vel(1);
		_vel_sp_limited(2) = vel(2);
		_vel_sp_limited_init = true;
	}
	const float az_max = math::max(_param_gc_az_max.get(), 0.1f);
	const float delta = math::constrain(vz_sp - _vel_sp_limited(2), -az_max * dt, az_max * dt);
	const float vz_sp_limited = _vel_sp_limited(2) + delta;
	const float accel_ff_z = delta / dt;
	_vel_sp_limited(2) = vz_sp_limited;

	const float e_v_z = vel(2) - vz_sp_limited;
	float accel_cmd_z = accel_ff_z - _param_gc_z_vel_p.get() * e_v_z - _vel_int_z_accel;
	accel_cmd_z = math::constrain(accel_cmd_z, -az_max, az_max);
	const float fz_world = hover_force * (accel_cmd_z / 9.81f - 1.f);
	const Vector3f f_vertical_body = R.transpose() * Vector3f(0.f, 0.f, fz_world);
	const Vector3f f_des_body = f_vertical_body + Vector3f(fx_body, fy_body, 0.f);

	const Dcmf R_sp(_q_att_ref);
	const Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	const Vector3f e_omega = omega - omega_d_in_B;
	if (!_landed) {
		updateIntegrators(e_omega, Vector3f(0.f, 0.f, e_v_z), dt);
	}

	const Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);
	const Vector3f f_des_world = R * f_des_body;
	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref,
			  Vector3f(0.f, 0.f, vz_sp_limited), Vector3f(0.f, 0.f, accel_cmd_z),
			  omega, omega_sp, timestamp_sample);
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
	// manual_control_setpoint 的发布频率通常低于几何控制器。使用
	// update() 会在两帧遥控消息之间返回 false，使油门瞬间回到 0，
	// 从而造成 vehicle_local_position_setpoint.vz 在油门值和零之间跳变。
	// 始终读取最近一帧，并仅在超过 500 ms 未更新时判定为失联。
	const bool has_manual = _sub_manual_control_sp.copy(&manual_sp) && manual_sp.valid
					&& (hrt_elapsed_time(&manual_sp.timestamp) < 500_ms);

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f),
					 0.001f, 0.05f);

	const float rate_max = _param_gc_man_rate_max.get();
	const float vx_max = math::max(_param_gc_vx_max.get(), 0.1f);
	const float vy_max = math::max(_param_gc_vy_max.get(), 0.1f);
	const float vz_max = math::max(_param_gc_vz_max.get(), 0.1f);
	const float dz = _param_gc_man_dz.get();
	const float mass = math::max(_param_gc_mass.get(), 0.1f);

	float p_sp = 0.f;
	float q_sp = 0.f;
	float r_sp = 0.f;
	float vx_sp_b = 0.f;
	float vy_sp_b = 0.f;
	float vz_sp_b = 0.f;

	if (has_manual) {
		const bool level_hold = manual_sp.aux3 > 0.5f;
		update_manual_level_hold(R_q, level_hold);

		// 姿态: 摇杆 [-1,1] → 角速率 [rad/s]
		p_sp = math::deadzone(manual_sp.roll,  dz) * rate_max;
		// RC pitch convention: pulling the stick toward the pilot is negative
		// manual pitch. In body FRD/NED control, a positive q command is nose-up,
		// so invert the RC pitch sign here.
		q_sp = -math::deadzone(manual_sp.pitch, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;
		if (_landed) { r_sp = 0.f; }

		// 摇杆 [-1,1] → 机体速度 [m/s] (aux1/aux2 水平, throttle 垂向)
		vx_sp_b = math::deadzone(manual_sp.aux1, dz) * vx_max;
		vy_sp_b = math::deadzone(manual_sp.aux2, dz) * vy_max;
		vz_sp_b = math::deadzone(manual_sp.throttle, dz) * vz_max;

		if (level_hold) {
			// Hold the current level attitude and ignore only the attitude sticks.
			// The three body-velocity commands remain effective.
			p_sp = 0.f;
			q_sp = 0.f;
		}

	} else {
		update_manual_level_hold(R_q, false);
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);
	Vector3f vel_sp_body(vx_sp_b, vy_sp_b, -vz_sp_b);

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
	vel_sp_world(0) = math::constrain(vel_sp_world(0), -vx_max, vx_max);
	vel_sp_world(1) = math::constrain(vel_sp_world(1), -vy_max, vy_max);
	vel_sp_world(2) = math::constrain(vel_sp_world(2), -vz_max, vz_max);

	// PX4-style acceleration limiting: constrain the velocity-setpoint slew
	// rate, |delta velocity| <= acceleration_limit * dt, independently in NED.
	const Vector3f accel_limit(math::max(_param_gc_ax_max.get(), 0.1f),
				   math::max(_param_gc_ay_max.get(), 0.1f),
				   math::max(_param_gc_az_max.get(), 0.1f));

	if (!_vel_sp_limited_init) {
		_vel_sp_limited(0) = math::constrain(vel(0), -vx_max, vx_max);
		_vel_sp_limited(1) = math::constrain(vel(1), -vy_max, vy_max);
		_vel_sp_limited(2) = math::constrain(vel(2), -vz_max, vz_max);
		_vel_sp_limited_init = true;
	}

	const Vector3f vel_sp_previous = _vel_sp_limited;

	for (int axis = 0; axis < 3; ++axis) {
		const float max_delta = accel_limit(axis) * dt;
		const float delta = math::constrain(vel_sp_world(axis) - _vel_sp_limited(axis), -max_delta, max_delta);
		_vel_sp_limited(axis) += delta;
	}

	const Vector3f accel_trajectory_world = (_vel_sp_limited - vel_sp_previous) / dt;
	vel_sp_world = _vel_sp_limited;

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

	limit_manual_attitude_reference();

	// ── 速度环 → 期望力 ──
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_v = vel - vel_sp_world;
	Vector3f g(0.f, 0.f, 9.81f);

	// Match PX4 multicopter vertical control: use a calibrated normalized
	// hover thrust for the vertical acceleration-to-thrust conversion.  The
	// mass remains available for the horizontal force model, but no longer
	// changes the vertical velocity feedback or gravity feed-forward directly.
	const float thrust_max = math::max(_param_gc_thrust_max.get(), 1.f);
	const float hover_thr = math::constrain(_param_gc_hover_thr.get(), 0.05f, 0.95f);
	const float hover_force = hover_thr * thrust_max;

	// Apply the acceleration limits to the actual translational command, not
	// only to the published setpoint. The trajectory acceleration is the
	// feed-forward term; velocity P/I feedback is combined in acceleration
	// units, then the non-gravity acceleration is clamped.
	Vector3f accel_cmd_world = accel_trajectory_world - kv.emult(e_v) / mass - _vel_int / mass;
	accel_cmd_world(2) = accel_trajectory_world(2)
			   - kv(2) * e_v(2)
			   - _vel_int_z_accel;

	for (int axis = 0; axis < 3; ++axis) {
		accel_cmd_world(axis) = math::constrain(accel_cmd_world(axis), -accel_limit(axis), accel_limit(axis));
	}

	Vector3f f_des_world = mass * (accel_cmd_world - g);
	// Convert vertical acceleration using the calibrated hover force.  This
	// keeps the velocity loop response stable when GC_MASS is approximate.
	f_des_world(2) = hover_force * (accel_cmd_world(2) / g(2) - 1.f);
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
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, vel_sp_world, accel_cmd_world, omega, omega_sp,
			  timestamp_sample);
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
	// ── Path 0: body-rate control (SET_ATTITUDE_TARGET with attitude ignored) ──
	// MavlinkReceiver publishes this topic when all three body rates are valid.
	// It must be handled before the attitude path: vehicle_attitude_setpoint is
	// intentionally not updated for a rate-only MAVLink command.
	vehicle_rates_setpoint_s rate_sp;

	if (_sub_vehicle_rates_setpoint.update(&rate_sp)) {
		_offboard_rate_sp = rate_sp;
		_offboard_rate_sp_timestamp = hrt_absolute_time();
	}

	// The MAVLink setpoint stream is slower than the state callback. Keep the
	// latest valid rate request active in every control cycle; after 500 ms let
	// the trajectory/attitude paths take over again.
	if (_offboard_rate_sp_timestamp != 0
	    && hrt_elapsed_time(&_offboard_rate_sp_timestamp) < 500_ms) {
		control_step_offboard_rates(R_q, pos, vel, omega, _offboard_rate_sp, timestamp_sample);
		return;
	}

	trajectory_setpoint_s traj_sp;

	// ── 路径1: 全状态位置+姿态控制 (SET_POSITION_TARGET_LOCAL_NED) ──
	if (_sub_trajectory_sp.update(&traj_sp)) {
		_offboard_external_alt_hold_active = false;

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
		publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, acc_sp, omega, omega_sp, timestamp_sample);
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

	if (!_offboard_external_alt_hold_active) {
		_offboard_external_alt_sp = pos(2);
		_offboard_external_alt_hold_active = true;
	}

	// ── 推力: 将外部归一化推力转换为牛顿 ──
	// MAVLink fill_thrust() 对旋翼机: thrust_body[2] = -thrust (NED: 负 = 向上)
	// 例: thrust=0.45 → thrust_body[2]=-0.45 → force_Z = -0.45 * GC_THRUST_MAX ≈ -49.5 N ≈ 悬停
	float thrust_cmd = att_sp.thrust_body[2] * _param_gc_thrust_max.get();

	if (fabsf(thrust_cmd) < 0.5f) {
		// 无有效推力指令时回退到悬停推力
		thrust_cmd = -mass * 9.81f;
	}

	// ── 力: 机体 Z 推力 + 起始高度保持 + 世界系 XY 速度阻尼 ──
	Vector3f f_des_body(0.f, 0.f, thrust_cmd);
	Vector3f f_des_world = R * f_des_body;

	// SET_ATTITUDE_TARGET 不带高度目标；以进入该路径时的高度为参考，
	// 闭合 Z 位置/速度环，消除悬停推力建模误差造成的长期漂移。
	f_des_world(2) -= _param_gc_z_p.get() * (pos(2) - _offboard_external_alt_sp);
	f_des_world(2) -= _param_gc_z_vel_p.get() * vel(2);

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
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel, Vector3f(0.f, 0.f, 0.f), omega, omega_sp,
			  timestamp_sample);
}

void GeoControl::control_step_offboard_rates(const Quatf &R_q, const Vector3f &pos,
		const Vector3f &vel,
		const Vector3f &omega, const vehicle_rates_setpoint_s &rate_sp,
		uint64_t timestamp_sample)
{
	Vector3f omega_sp(rate_sp.roll, rate_sp.pitch, rate_sp.yaw);

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(omega_sp(i))) {
			omega_sp(i) = 0.f;
		}
	}

	const float dt = math::constrain(static_cast<float>((hrt_absolute_time() - _last_run) * 1e-6f), 0.001f, 0.05f);
	const float mass = _param_gc_mass.get();
	const Dcmf R(R_q);

	if (!_offboard_external_alt_hold_active) {
		_offboard_external_alt_sp = pos(2);
		_offboard_external_alt_hold_active = true;
	}

	float thrust_cmd = rate_sp.thrust_body[2] * _param_gc_thrust_max.get();

	if (fabsf(thrust_cmd) < 0.5f) {
		thrust_cmd = -mass * 9.81f;
	}

	Vector3f f_des_body(0.f, 0.f, thrust_cmd);
	Vector3f f_des_world = R * f_des_body;
	f_des_world(2) -= _param_gc_z_p.get() * (pos(2) - _offboard_external_alt_sp);
	f_des_world(2) -= _param_gc_z_vel_p.get() * vel(2);
	const float damp_xy = mass * 0.5f;
	f_des_world(0) -= damp_xy * vel(0);
	f_des_world(1) -= damp_xy * vel(1);
	f_des_body = R.transpose() * f_des_world;

	// With no attitude target, use the current attitude as R_d. This removes
	// the attitude-error term and leaves the Lee rate loop plus rigid-body
	// compensation to track the requested body angular velocity.
	const Dcmf R_sp(R);
	const Vector3f omega_d_in_B = omega_sp;
	const Vector3f e_omega = omega - omega_sp;

	if (!_landed) {
		updateIntegrators(e_omega, Vector3f(0.f, 0.f, 0.f), dt);
	}

	const Vector3f tau_des = computeAttitudeTorque(R, R_sp, omega, omega_sp, omega_d_in_B, dt);
	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, R_q, Vector3f(0.f, 0.f, 0.f),
			  Vector3f(0.f, 0.f, 0.f), omega, omega_sp, timestamp_sample);
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
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, acc_sp, omega, omega_sp, timestamp_sample);
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

	// 自动姿态闭环验证: 依次运行多轴阶跃并评分。
	if (_att_verify_active) {
		control_step_att_verify(local_pos.timestamp_sample);
		return;
	}

	// 姿态测试: 跟踪期望欧拉角
	if (_att_test_active) {
		if (_att_test_duration_us > 0 &&
		    hrt_absolute_time() - _att_test_start >= _att_test_duration_us) {
			_att_test_active = false;
			PX4_INFO("ATT_TEST: 完成，请切换到安全模式或执行 disarm");
			publish_wrench(Vector3f(0.f, 0.f, 0.f), Vector3f(0.f, 0.f, 0.f), hrt_absolute_time());
			return;
		}

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

	const bool was_armed = _armed;
	_armed = vehicle_status.arming_state == vehicle_status.ARMING_STATE_ARMED;

	// Start each armed interval from the actual vehicle attitude.  Otherwise
	// a vehicle powered up with a non-zero heading can receive a large yaw
	// correction immediately after arming, even with all sticks centered.
	if (_armed && !was_armed) {
		_q_att_ref = Quatf(att.q[0], att.q[1], att.q[2], att.q[3]);
		_q_att_ref.normalize();
		resetIntegrators();
	}
	_nav_state_last = vehicle_status.nav_state;

	// 未解锁时: 清零积分器并发布零力/力矩，使执行器处于中立初值
	// (否则会发布悬停推力 + 姿态保持力矩，导致分配器在零推力退化点把舵机推到饱和)
	if (!_armed) {
		resetIntegrators();
		_pos_sp_init = false;   // 下次解锁时重新初始化位置保持指令
		_vel_sp_limited_init = false;
		_offboard_external_alt_hold_active = false;
		publish_wrench(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
			       local_pos.timestamp_sample);
		publish_setpoints(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Quatf(1.f, 0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
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

	if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_ACRO) {
		mode = ControlMode::ACRO;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_MANUAL) {
		mode = ControlMode::MANUAL;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_POSCTL) {
		mode = ControlMode::POSITION;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_ALTCTL) {
		mode = ControlMode::ALTITUDE;

	} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_OFFBOARD) {
		mode = ControlMode::OFFBOARD;
	}

	if (mode != ControlMode::OFFBOARD) {
		_offboard_external_alt_hold_active = false;
		_offboard_rate_sp_timestamp = 0;
	}

	if (mode != ControlMode::POSITION && mode != ControlMode::ALTITUDE) {
		_vel_sp_limited_init = false;
	}

	switch (mode) {
	case ControlMode::MANUAL:
		control_step_manual(R_q, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::POSITION:
		control_step_position(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::ALTITUDE:
		control_step_altitude(R_q, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::OFFBOARD:
		control_step_offboard(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::AUTO:
		control_step_auto(R_q, pos, vel, omega, local_pos.timestamp_sample);
		break;

	case ControlMode::ACRO:
		control_step_acro(local_pos.timestamp_sample);
		break;
	}
}

void GeoControl::Run()
{
	if (should_exit()) {
		_sub_vehicle_attitude_setpoint_event.unregisterCallback();
		_sub_vehicle_attitude_event.unregisterCallback();
		_sub_vehicle_local_position_event.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	parameters_update();

	vehicle_attitude_setpoint_s att_sp_event{};
	vehicle_attitude_s attitude_event_data{};
	vehicle_local_position_s local_event{};
	vehicle_local_position_s local_pos{};

	const bool attitude_event = _sub_vehicle_attitude_setpoint_event.update(&att_sp_event);
	const bool attitude_state_event = _sub_vehicle_attitude_event.update(&attitude_event_data);
	const bool position_event = _sub_vehicle_local_position_event.update(&local_event);

	// Prefer vehicle_attitude_setpoint as the timing source. If it is not
	// published in the current flight-mode configuration, use local position
	// as a fallback so the controller cannot stop producing actuator commands.
	if ((attitude_event || attitude_state_event || position_event) &&
	    _sub_vehicle_local_position.copy(&local_pos)) {
		const hrt_abstime now = hrt_absolute_time();

		if (_last_run == 0) {
			_last_run = now;
		}

		// vehicle_attitude_setpoint is the wake-up source. The parameter limits
		// execution to a lower rate when required; the actual rate cannot exceed
		// the attitude-setpoint publication rate.
		// The timestamp is deliberately checked after fetching the newest sample
		// so the next callback always starts from fresh state data.
		const float rate_hz = math::constrain(_param_gc_rate_hz.get(), 10.f, 400.f);
		const hrt_abstime period_us = static_cast<hrt_abstime>(1e6f / rate_hz);

		if ((now - _last_run) >= period_us) {
			control_step(local_pos);
			_last_run = now;
		}
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

		if (!strcmp(argv[0], "att_verify")) {
			GeoControl *inst = get_instance();

			if (!inst) {
				PX4_ERR("geo_control 未运行 — 请先启动");
				return -1;
			}

			return inst->att_verify_command(argc, argv);
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

	Eulerf euler_cur(R_q);
	float roll_err = _att_test_roll - euler_cur.phi();
	float pitch_err = _att_test_pitch - euler_cur.theta();
	float yaw_err = _att_test_yaw - euler_cur.psi();

	while (yaw_err > M_PI_F) { yaw_err -= 2.f * M_PI_F; }

	while (yaw_err < -M_PI_F) { yaw_err += 2.f * M_PI_F; }

	_att_test_error_last = Vector3f(roll_err, pitch_err, yaw_err);
	_att_test_omega_last = omega;
	_att_test_feedback_valid = true;

	static hrt_abstime last_p = 0;
	hrt_abstime now = hrt_absolute_time();

	if (now - last_p > 500000) {
		last_p = now;
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
		_att_test_duration_us = 0;
		_att_test_feedback_valid = false;
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

	_att_verify_active = false;
	_att_test_duration_us = (uint64_t)(dur_s * 1e6f);
	_att_test_start = hrt_absolute_time();
	_att_test_feedback_valid = false;
	resetIntegrators();
	_att_test_active = true;

	PX4_INFO("ATT_TEST: 目标(r=%.0f,p=%.0f,y=%.0f)° 持续=%.0fs",
		 (double)atof(argv[1]),
		 (double)(argc >= 3 ? atof(argv[2]) : 0.0),
		 (double)(argc >= 4 ? atof(argv[3]) : 0.0),
		 (double)dur_s);
	return 0;
}

void GeoControl::control_step_att_verify(uint64_t timestamp_sample)
{
	struct AttitudeVerifyStep {
		const char *name;
		float roll_deg;
		float pitch_deg;
		float yaw_deg;
	};

	static constexpr AttitudeVerifyStep steps[] = {
		{"roll +10",  10.f,   0.f,   0.f},
		{"roll -10", -10.f,   0.f,   0.f},
		{"pitch +10",  0.f,  10.f,   0.f},
		{"pitch -10",  0.f, -10.f,   0.f},
		{"yaw +15",    0.f,   0.f,  15.f},
		{"yaw -15",    0.f,   0.f, -15.f},
		{"level",      0.f,   0.f,   0.f},
	};
	static constexpr int step_count = sizeof(steps) / sizeof(steps[0]);
	static constexpr hrt_abstime step_duration_us = 4_s;
	static constexpr hrt_abstime score_start_us = 2_s;
	static constexpr float max_mean_error_rad = 3.f * M_PI_F / 180.f;
	static constexpr float max_mean_rate_rad_s = 0.35f;

	if (_att_verify_step >= step_count) {
		_att_verify_active = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_att_verify_step_start == 0) {
		_att_verify_step_start = now;
		_att_verify_error_sum.zero();
		_att_verify_rate_sum = 0.f;
		_att_verify_score_count = 0;
		_att_test_feedback_valid = false;
		resetIntegrators();

		PX4_INFO("ATT_VERIFY [%d/%d] %s: r=%.0f p=%.0f y=%.0f deg",
			 _att_verify_step + 1, step_count, steps[_att_verify_step].name,
			 (double)steps[_att_verify_step].roll_deg,
			 (double)steps[_att_verify_step].pitch_deg,
			 (double)steps[_att_verify_step].yaw_deg);
	}

	_att_test_roll = math::radians(steps[_att_verify_step].roll_deg);
	_att_test_pitch = math::radians(steps[_att_verify_step].pitch_deg);
	_att_test_yaw = math::radians(steps[_att_verify_step].yaw_deg);
	control_step_att_test(timestamp_sample);

	const hrt_abstime elapsed = now - _att_verify_step_start;

	if (elapsed >= score_start_us && _att_test_feedback_valid) {
		_att_verify_error_sum += Vector3f(fabsf(_att_test_error_last(0)),
						   fabsf(_att_test_error_last(1)),
						   fabsf(_att_test_error_last(2)));
		_att_verify_rate_sum += _att_test_omega_last.norm();
		++_att_verify_score_count;
	}

	if (elapsed < step_duration_us) {
		return;
	}

	Vector3f mean_error{};
	float mean_rate = INFINITY;

	if (_att_verify_score_count > 0) {
		mean_error = _att_verify_error_sum / static_cast<float>(_att_verify_score_count);
		mean_rate = _att_verify_rate_sum / static_cast<float>(_att_verify_score_count);
	}

	const bool pass = _att_verify_score_count > 0
			  && mean_error(0) <= max_mean_error_rad
			  && mean_error(1) <= max_mean_error_rad
			  && mean_error(2) <= max_mean_error_rad
			  && mean_rate <= max_mean_rate_rad_s;

	if (pass) {
		++_att_verify_pass_count;
	}

	PX4_INFO("ATT_VERIFY [%d/%d] %s: %s, mean_err=(%.2f,%.2f,%.2f) deg, mean_rate=%.3f rad/s, samples=%d",
		 _att_verify_step + 1, step_count, steps[_att_verify_step].name,
		 pass ? "PASS" : "FAIL",
		 (double)math::degrees(mean_error(0)),
		 (double)math::degrees(mean_error(1)),
		 (double)math::degrees(mean_error(2)),
		 (double)mean_rate, _att_verify_score_count);

	++_att_verify_step;
	_att_verify_step_start = 0;

	if (_att_verify_step >= step_count) {
		_att_verify_active = false;
		_att_test_active = true;
		_att_test_roll = 0.f;
		_att_test_pitch = 0.f;
		_att_test_yaw = 0.f;
		_att_test_duration_us = 0;
		_att_test_start = now;
		_att_test_feedback_valid = false;
		resetIntegrators();

		PX4_INFO("ATT_VERIFY 完成: %d/%d 通过。已切换到水平悬停保持；完成后执行 'geo_control att_test stop' 再 disarm。",
			 _att_verify_pass_count, step_count);
	}
}

int GeoControl::att_verify_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[1], "stop")) {
		_att_verify_active = false;
		_att_test_active = false;
		_att_test_feedback_valid = false;
		resetIntegrators();
		publish_wrench(Vector3f(0.f, 0.f, 0.f), Vector3f(0.f, 0.f, 0.f), hrt_absolute_time());
		PX4_INFO("ATT_VERIFY: 已停止，请切换到安全模式或执行 disarm");
		return 0;
	}

	if (argc > 1) {
		PX4_INFO_RAW("用法: geo_control att_verify [stop]\n");
		PX4_INFO_RAW("  已切 Acro 且解锁后，自动运行 ±10° roll/pitch、±15° yaw 和回中阶跃。\n");
		return 0;
	}

	vehicle_status_s vehicle_status{};
	_sub_vehicle_status.copy(&vehicle_status);

	if (vehicle_status.arming_state != vehicle_status.ARMING_STATE_ARMED) {
		PX4_ERR("ATT_VERIFY: 请先切到 Acro 并解锁");
		return -1;
	}

	if (vehicle_status.nav_state != vehicle_status.NAVIGATION_STATE_ACRO) {
		PX4_ERR("ATT_VERIFY: 当前不是 Acro 模式，请先执行 commander mode acro");
		return -1;
	}

	if (_verify_active || _att_test_active || _vel_test_active || _offboard_test_active ||
	    _sweep_active || _acro_active) {
		PX4_ERR("ATT_VERIFY: 另一个测试正在运行，请先停止它");
		return -1;
	}

	_att_verify_active = true;
	_att_verify_step = 0;
	_att_verify_step_start = 0;
	_att_verify_pass_count = 0;
	_att_verify_score_count = 0;
	_att_verify_error_sum.zero();
	_att_verify_rate_sum = 0.f;
	_att_test_feedback_valid = false;
	resetIntegrators();

	PX4_INFO("ATT_VERIFY: 已开始，7 个阶跃、每步 4 秒，后 2 秒用于评分");
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
	// 双重对比: (1)前向运动学重构  (2)JSBSim 物理仿真实际力/力矩增量
	//
	// 链路: geo_control → ControlAllocator → MAVLink → bridge → JSBSim
	//       JSBSim 合力/力矩 → bridge UDP :9871 → geo_control 读取
	//
	// JSBSim FK: t_i = T_i·(sinα cosβ, sinβ, -cosα cosβ),  T_i = esc_i²·TMAX
	// 重构误差小 → 分配器收敛。
	// JSBSim 使用电机/螺旋桨动态模型, 绝对推力通常不等于 esc²·TMAX。
	// 因此 JSBSim 侧对比 hover 基线后的增量, 用于判断力/力矩耦合。
	const float tmax = math::max(_param_ca_mq_tmax.get(), 1.f);
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float z = _param_ca_mq_z.get();

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

	// ── 重力补偿: 每组测试力/力矩都叠加悬停推力, 飞机不落地 ──
	// NED 下 Fz 向下为正, 悬停推力 = -mass*g (向上)
	const float hover_fz = -_param_gc_mass.get() * 9.81f;
	matrix::Vector<float, 6> hover_wrench{};
	hover_wrench(2) = hover_fz;

	PX4_INFO("MorphQuad 端到端分配验证 — 60 组固定 6D 力/力矩 (hover 基线 Fz=%.1f N)", (double)hover_fz);
	PX4_INFO("  FK: t_i = T_i·(sinα cosβ, sinβ, -cosα cosβ),  T_i = esc_i²·TMAX = %.2f N", (double)tmax);
	PX4_INFO("  JSBSim 反馈: UDP :%d, 每组先采 hover 基线, 再比较 Δwrench", (int)WRENCH_PORT);

	_verify_active = true;

	// `verify` is a synchronous PXH command.  With JSBSim hold-down enabled
	// there can be no new local-position samples, so Run() is not guaranteed to
	// call control_step() while this command is waiting for the plant.  Publish
	// the test wrench here as well; this keeps the real ControlAllocator fed at
	// the same rate as the sampling loop instead of relying on that callback.
	auto publish_verify_wrench = [&]() {
		publish_wrench(Vector3f(_verify_wrench(0), _verify_wrench(1), _verify_wrench(2)),
			      Vector3f(_verify_wrench(3), _verify_wrench(4), _verify_wrench(5)),
			      hrt_absolute_time());
	};

	// 记录上一帧 JSBSim 时间戳，用于对齐检测
	double jsb_timestamp_prev = 0.0;

	auto sample_jsb_wrench = [&](double out[6], double &out_timestamp, bool &stable) -> bool {
		for (int j = 0; j < 6; ++j) {
			out[j] = 0.0;
		}

		out_timestamp = 0.0;
		stable = false;

		struct pollfd pfd{};
		pfd.fd = wrench_fd;
		pfd.events = POLLIN;

		double pkt[7];

		// 排空旧数据, 后续只平均新命令稳定后的反馈。
		while (::poll(&pfd, 1, 0) > 0) {
			recvfrom(wrench_fd, pkt, sizeof(pkt), 0, nullptr, nullptr);
			pfd.revents = 0;
		}

		// JSBSim 的电机/螺旋桨具有明显动态。固定 150 ms 后直接平均会把
		// 瞬态当成稳态（特别是由 hover 降到低油门时），因此先等待管道传播，
		// 再比较相邻两个平均窗口，直到所有 6 个量均收敛。
		static constexpr int INITIAL_SETTLE_STEPS = 75; // 750 ms
		static constexpr int WINDOW_STEPS = 25;         // 250 ms
		static constexpr int MAX_WINDOWS = 6;
		static constexpr double FORCE_SETTLE_N = 0.20;
		static constexpr double TORQUE_SETTLE_NM = 0.05;

		for (int settle = 0; settle < INITIAL_SETTLE_STEPS; ++settle) {
			publish_verify_wrench();
			px4_usleep(10000);
		}

		double previous[6] {};
		bool have_previous = false;

		for (int window = 0; window < MAX_WINDOWS; ++window) {
			double sum[6] {};
			int samples = 0;

			for (int retry = 0; retry < WINDOW_STEPS; ++retry) {
				publish_verify_wrench();
				px4_usleep(10000);

				while (::poll(&pfd, 1, 0) > 0) {
					const ssize_t n = recvfrom(wrench_fd, pkt, sizeof(pkt), 0, nullptr, nullptr);

					if (n == (ssize_t)sizeof(pkt) && pkt[0] > jsb_timestamp_prev) {
						for (int j = 0; j < 6; ++j) {
							sum[j] += pkt[j + 1];
						}

						out_timestamp = pkt[0];
						jsb_timestamp_prev = pkt[0];
						++samples;
					}

					pfd.revents = 0;
				}
			}

			if (samples <= 0) {
				return false;
			}

			double current[6] {};

			for (int j = 0; j < 6; ++j) {
				current[j] = sum[j] / (double)samples;
			}

			if (have_previous) {
				stable = true;

				for (int j = 0; j < 6; ++j) {
					const double tolerance = (j < 3) ? FORCE_SETTLE_N : TORQUE_SETTLE_NM;

					if (std::fabs(current[j] - previous[j]) > tolerance) {
						stable = false;
						break;
					}
				}
			}

			for (int j = 0; j < 6; ++j) {
				out[j] = current[j];
				previous[j] = current[j];
			}

			if (stable) {
				return true;
			}

			have_previous = true;
		}

		return true; // valid data, but the caller must not score an unsettled sample
	};

	for (int k = 0; k < 60; ++k) {
		const int axis = k / 10;          // 0..5
		const int m = (k % 10) + 1;       // 1..10
		matrix::Vector<float, 6> dW{};
		dW(axis) = (float)m / 10.f * scale[axis];

		// 叠加悬停推力, 保持飞机不落地
		matrix::Vector<float, 6> W = hover_wrench + dW;

		// 每个采样点先回到纯 hover 并采集 JSBSim 基线。这样可以消除
		// JSBSim 电机/螺旋桨模型和 esc²·TMAX 静态模型之间的绝对推力偏差。
		_verify_wrench = hover_wrench;
		for (int settle = 0; settle < 40; ++settle) {
			publish_verify_wrench();
			px4_usleep(8000);
		}

		double jsb_hover[6] = {};
		double jsb_hover_timestamp = 0.0;
		bool jsb_hover_stable = false;
		const bool jsb_hover_valid = sample_jsb_wrench(jsb_hover, jsb_hover_timestamp, jsb_hover_stable);

		// 施加当前测试增量。
		_verify_wrench = W;

		double jsb_wrench[6] = {};
		double jsb_timestamp = 0.0;
		bool jsb_stable = false;
		const bool jsb_valid = sample_jsb_wrench(jsb_wrench, jsb_timestamp, jsb_stable);

		// ── 读取执行器输出 (FK 重构) ──
		actuator_motors_s motors{};
		actuator_servos_s servos{};
		sub_motors.copy(&motors);
		sub_servos.copy(&servos);

		matrix::Vector3f f_ach(0.f, 0.f, 0.f);
		matrix::Vector3f tau_ach(0.f, 0.f, 0.f);

		PX4_INFO_RAW("[%2d] 指令 F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  Δaxis=%d Δ=%.2f\n",
			     k, (double)W(0), (double)W(1), (double)W(2),
			     (double)W(3), (double)W(4), (double)W(5),
			     axis, (double)dW(axis));

		// 逐旋翼明细
		for (int i = 0; i < NQ; ++i) {
			const float esc = (i < actuator_motors_s::NUM_CONTROLS) ? motors.control[i] : NAN;
			// 归一化 ±1 -> ±π rad，与分配器和 bridge 的舵机行程一致。
			const float alpha = (2 * i < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i] * M_PI_F : NAN;
			const float beta  = ((2 * i + 1) < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i + 1] * M_PI_F : NAN;

			const float T = (PX4_ISFINITE(esc) ? esc * esc : 0.f) * tmax;
			const float sa = sinf(alpha), ca = cosf(alpha);
			const float sb = sinf(beta),  cb = cosf(beta);
			const matrix::Vector3f t_vec(T * sa * cb, T * sb, -T * ca * cb);

			f_ach += t_vec;
			const matrix::Vector3f r_i(lx[i], ly[i], z);
			tau_ach += r_i.cross(t_vec);

			PX4_INFO_RAW("    rotor%d: esc=%6.3f  a=%7.2f deg  b=%7.2f deg  |T|=%7.2f N\n",
				     i, (double)esc, (double)math::degrees(alpha),
				     (double)math::degrees(beta), (double)T);
		}

		// ── FK 重构对比 ──
		matrix::Vector<float, 6> allocated_wrench{};
		allocated_wrench(0)=f_ach(0); allocated_wrench(1)=f_ach(1); allocated_wrench(2)=f_ach(2);
		allocated_wrench(3)=tau_ach(0); allocated_wrench(4)=tau_ach(1); allocated_wrench(5)=tau_ach(2);

		float err_fk = 0.f;

		for (int j = 0; j < 6; ++j) {
			const float e = W(j) - allocated_wrench(j);
			err_fk += e * e;
		}

		err_fk = sqrtf(err_fk);

		// ── JSBSim 物理增量对比 ──
		double err_jsb = -1.0;
		double jsb_delta[6] {};

		if (jsb_hover_valid && jsb_valid && jsb_hover_stable && jsb_stable) {
			err_jsb = 0.0;
			for (int j = 0; j < 6; ++j) {
				jsb_delta[j] = jsb_wrench[j] - jsb_hover[j];
				// Compare the plant with the wrench actually represented by the
				// actuator outputs, not the original request. This keeps intentional
				// Fz clamping and thrust/servo saturation out of the plant error.
				const double allocated_delta = (double)allocated_wrench(j) - (double)hover_wrench(j);
				const double e = allocated_delta - jsb_delta[j];
				err_jsb += e * e;
			}
			err_jsb = sqrt(err_jsb);
		}

		PX4_INFO_RAW("    FK:  F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  |request residual|=%7.3f\n",
			     (double)allocated_wrench(0), (double)allocated_wrench(1), (double)allocated_wrench(2),
			     (double)allocated_wrench(3), (double)allocated_wrench(4), (double)allocated_wrench(5),
			     (double)err_fk);

		if (jsb_hover_valid && jsb_valid) {
			PX4_INFO_RAW("    JSB0:F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  t=%.3fs\n",
				     jsb_hover[0], jsb_hover[1], jsb_hover[2],
				     jsb_hover[3], jsb_hover[4], jsb_hover[5],
				     jsb_hover_timestamp);
			PX4_INFO_RAW("    JSB: F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  t=%.3fs\n",
				     jsb_wrench[0], jsb_wrench[1], jsb_wrench[2],
				     jsb_wrench[3], jsb_wrench[4], jsb_wrench[5],
				     jsb_timestamp);

			if (jsb_hover_stable && jsb_stable) {
				PX4_INFO_RAW("    ΔJSB:F=(%7.2f,%7.2f,%7.2f) T=(%6.2f,%6.2f,%6.2f)  |err vs allocated|=%7.3f\n",
					     jsb_delta[0], jsb_delta[1], jsb_delta[2],
					     jsb_delta[3], jsb_delta[4], jsb_delta[5],
					     (double)err_jsb);

			} else {
				PX4_WARN("verify: JSBSim wrench did not settle; this point is not scored");
			}
		} else {
			PX4_INFO_RAW("    JSB: (无数据 — bridge 力/力矩 UDP 反馈未到达)\n");
		}

		// ── 采样后冷却: 回到纯悬停 ~500ms, 电机回中准备下一组 ──
		_verify_wrench = hover_wrench;
		for (int cool = 0; cool < 60; cool++) {
			publish_verify_wrench();
			px4_usleep(8000);
		}
	}

	_verify_wrench = matrix::Vector<float, 6>{};
	publish_verify_wrench();
	::close(wrench_fd);
	_verify_active = false;
	PX4_INFO("验证完成 — 注入已关闭，控制器恢复");
	return 0;
}

void GeoControl::control_step_acro(uint64_t timestamp_sample)
{
	if (!_acro_active) {
		// 正常 ACRO 飞行模式: 确保6维力/力矩全部为零,
		// 阻断 mc_rate_control 归一化推力 (-1..1) 被 MorphQuad 分配器
		// 误当作牛顿解读，导致标准伪逆分配输出满油。
		publish_wrench(Vector3f(0.f, 0.f, 0.f), Vector3f(0.f, 0.f, 0.f), timestamp_sample);
		return;
	}

	// acro_command 模式: 注入固定力/力矩用于分配器验证
	Vector3f f_cmd(_acro_wrench(0), _acro_wrench(1), _acro_wrench(2));
	Vector3f tau_cmd(_acro_wrench(3), _acro_wrench(4), _acro_wrench(5));

	publish_wrench(f_cmd, tau_cmd, timestamp_sample);

	// 每秒打印分配器输出的电机/舵机值
	{
		static hrt_abstime last = 0;
		hrt_abstime now = hrt_absolute_time();
		if (now - last > 500000) {
			last = now;
			uORB::Subscription sub_motors{ORB_ID(actuator_motors)};
			uORB::Subscription sub_servos{ORB_ID(actuator_servos)};
			actuator_motors_s motors{};
			actuator_servos_s servos{};
			sub_motors.copy(&motors);
			sub_servos.copy(&servos);
			PX4_INFO_RAW("[ACRO] cmd F=(%.1f,%.1f,%.1f) T=(%.2f,%.2f,%.2f) esc=[%.3f,%.3f,%.3f,%.3f] servo=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]\n",
				(double)f_cmd(0), (double)f_cmd(1), (double)f_cmd(2),
				(double)tau_cmd(0), (double)tau_cmd(1), (double)tau_cmd(2),
				(double)motors.control[0], (double)motors.control[1],
				(double)motors.control[2], (double)motors.control[3],
				(double)servos.control[0], (double)servos.control[1],
				(double)servos.control[2], (double)servos.control[3],
				(double)servos.control[4], (double)servos.control[5],
				(double)servos.control[6], (double)servos.control[7]);
		}
	}

	// actuator_motors_s motors{};
	// actuator_servos_s servos{};
	// sub_motors.copy(&motors);
	// sub_servos.copy(&servos);

	// Vector3f f_ach(0.f, 0.f, 0.f);
	// Vector3f tau_ach(0.f, 0.f, 0.f);
	// float esc_vals[NQ] = {};
	// float alpha_vals[NQ] = {};
	// float beta_vals[NQ] = {};

	// for (int i = 0; i < NQ; ++i) {
	// 	const float esc = (i < actuator_motors_s::NUM_CONTROLS) ? motors.control[i] : NAN;
	// 	const float alpha = (2 * i < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i] * M_PI_F : NAN;
	// 	const float beta  = ((2 * i + 1) < actuator_servos_s::NUM_CONTROLS) ? servos.control[2 * i + 1] * M_PI_F : NAN;

	// 	esc_vals[i] = esc;
	// 	alpha_vals[i] = alpha;
	// 	beta_vals[i] = beta;

	// 	const float T = (PX4_ISFINITE(esc) ? esc * esc : 0.f) * tmax;
	// 	const float sa = sinf(alpha), ca = cosf(alpha);
	// 	const float sb = sinf(beta),  cb = cosf(beta);
	// 	const Vector3f t_vec(T * sa * cb, T * sb, -T * ca * cb);

	// 	f_ach += t_vec;
	// 	const Vector3f r_i(lx[i], ly[i], 0.f);
	// 	tau_ach += r_i.cross(t_vec);
	// }

	// // 每 0.5s 打印 (节流避免日志刷屏)
	// static hrt_abstime _last_acro_print = 0;
	// const hrt_abstime now = hrt_absolute_time();

	// if (now - _last_acro_print > 500000) {
	// 	_last_acro_print = now;

	// 	PX4_INFO_RAW("=== ACRO 力/力矩 [机体 FRD] ===\n");
	// 	PX4_INFO_RAW("  指令  F=(%8.2f,%8.2f,%8.2f) N  T=(%6.2f,%6.2f,%6.2f) Nm\n",
	// 		     (double)f_cmd(0), (double)f_cmd(1), (double)f_cmd(2),
	// 		     (double)tau_cmd(0), (double)tau_cmd(1), (double)tau_cmd(2));
	// 	PX4_INFO_RAW("  实际  F=(%8.2f,%8.2f,%8.2f) N  T=(%6.2f,%6.2f,%6.2f) Nm\n",
	// 		     (double)f_ach(0), (double)f_ach(1), (double)f_ach(2),
	// 		     (double)tau_ach(0), (double)tau_ach(1), (double)tau_ach(2));

	// 	float err_f = (f_cmd - f_ach).norm();
	// 	float err_tau = (tau_cmd - tau_ach).norm();
	// 	PX4_INFO_RAW("  误差  |ΔF|=%.3f N  |Δτ|=%.3f Nm\n", (double)err_f, (double)err_tau);

	// 	for (int i = 0; i < NQ; ++i) {
	// 		PX4_INFO_RAW("  rotor%d: esc=%-6.3f  α=%7.1f°  β=%7.1f°\n",
	// 			     i, (double)esc_vals[i],
	// 			     (double)math::degrees(alpha_vals[i]),
	// 			     (double)math::degrees(beta_vals[i]));
	// 	}
	// }
}

int GeoControl::acro_command(int argc, char *argv[])
{
	// 用法:
	//   geo_control acro Fx Fy Fz Tx Ty Tz [duration_s]
	//   geo_control acro stop    (别名: q / quit)
	if (argc < 2) {
		PX4_INFO_RAW("用法:\n");
		PX4_INFO_RAW("  geo_control acro Fx Fy Fz Tx Ty Tz [duration_s]\n");
		PX4_INFO_RAW("  geo_control acro stop    (别名: q / quit)\n");
		PX4_INFO_RAW("  (Fx/Fy/Fz 单位 N, Tx/Ty/Tz 单位 Nm, 机体 FRD 坐标系)\n");
		PX4_INFO_RAW("  示例:\n");
		PX4_INFO_RAW("    geo_control acro 0 0 -44.7 0 0 0   # 悬停推力 (4.56kg)\n");
		PX4_INFO_RAW("    geo_control acro 10 0 -44.7 0 0 0  # 悬停 + 10N 前向\n");
		PX4_INFO_RAW("    geo_control acro 0 0 -44.7 1 0 0 5 # 悬停 + 1Nm roll, 5s\n");
		return 0;
	}

	if (!strcmp(argv[1], "stop") || !strcmp(argv[1], "q") || !strcmp(argv[1], "quit")) {
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
		 _acro_duration_us > 0 ? "(定时)" : "(持续, 用 'acro stop' / 'acro q' 停止)");
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
	PX4_INFO("armed: %d, landed: %d, nav_state: %u, mass: %.3f, rate_limit: %.1f Hz",
		 _armed, _landed, (unsigned)_nav_state_last, (double)_param_gc_mass.get(),
		 (double)_param_gc_rate_hz.get());

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
$ commander mode acro
$ commander arm
$ geo_control att_verify
$ geo_control stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("geo_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("verify", "注入 60 组固定 6D 力/力矩, 打印各旋翼电机/舵机输出");
	PRINT_MODULE_USAGE_COMMAND_DESCR("att_verify", "已解锁 Acro 下自动测试 roll/pitch/yaw 姿态闭环");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int geo_control_main(int argc, char *argv[])
{
	return GeoControl::main(argc, argv);
}
