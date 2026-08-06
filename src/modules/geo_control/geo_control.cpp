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

	// ── Rate/torque integrator with anti-windup ──
	for (int i = 0; i < 3; i++) {
		float rate_error = e_omega(i);

		// Anti-windup: prevent further integration in the saturated direction
		if (_torque_sat_pos(i)) {
			rate_error = math::min(rate_error, 0.f);
		}

		if (_torque_sat_neg(i)) {
			rate_error = math::max(rate_error, 0.f);
		}

		// Nonlinear I-factor: reduce gain for large errors (prevents windup during flips)
		float i_factor = rate_error / math::radians(400.f);
		i_factor = math::max(0.f, 1.f - i_factor * i_factor);

		float rate_i = _rate_int(i) + i_factor * rate_i_gain * rate_error * dt;

		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -rate_int_lim, rate_int_lim);
		}
	}

	// ── Velocity/force integrator with anti-windup ──
	for (int i = 0; i < 3; i++) {
		float vel_error = e_v_world(i);

		// Anti-windup from thrust saturation
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
}

void GeoControl::publish_setpoints(const Vector3f &f_des_world, const Vector3f &f_des_body,
				   const Vector3f &tau_des, const Quatf &q_att_ref,
				   const Vector3f &vel_sp_world, const Vector3f &omega,
				   uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();
	const float thrust_max = math::max(_param_gc_thrust_max.get(), 1.f);

	// ── Normalized collective-throttle estimate (matching morphquad esc table) ──
	// ‖f_des_world‖ spans ~0 (idle) … mass*g (hover≈0.4) … N_rotors*Tmax (full≈1.0).
	const float throttle_norm = math::constrain(f_des_world.norm() / thrust_max, 0.f, 1.f);

	// ── vehicle_local_position_setpoint (position/velocity debug + throttle est) ──
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

	// ── vehicle_attitude_setpoint (desired attitude + body thrust) ──
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	// Quatf components are stored as (w, x, y, z)
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
	// Desired angular accel from the torque setpoint: tau = J·α + ω×Jω  →  α = J⁻¹(τ − ω×Jω)
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

// ─────────────────────────────────────────────────────────────────────────────
// Manual mode: RC sticks → angular rate + velocity references
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_manual(const Quatf &R_q, const Vector3f &omega,
				     uint64_t timestamp_sample)
{
	// Manual control: if no valid RC/joystick input is available (common in
	// SITL/HIL without a transmitter), do NOT bail out with a zero wrench.
	// Fall back to zero stick setpoints instead, so the velocity controller
	// still commands the hover thrust (-mass*g). This keeps
	// vehicle_thrust_setpoint non-zero, which stops the land detector from
	// declaring "landed" and auto-disarming the vehicle
	// ("系统油门值始终为0 导致自动上锁"). When a real stick input exists,
	// behaviour is unchanged.
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
	float fz_body = -mass * 9.81f;   // hover trim (upward) when no stick input

	if (has_manual) {
		// Attitude: aux1/aux2 → roll/pitch angular rate [rad/s] (closed loop);
		// yaw stick → yaw rate.
		p_sp = math::deadzone(manual_sp.aux1, dz) * rate_max;
		q_sp = math::deadzone(manual_sp.aux2, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;

		// Force: roll/pitch → body x/y force [N] (OPEN LOOP, no feedback);
		// throttle → body z force. Body z is down, so hover trim is -mass*g
		// (upward) and throttle-up commands more upward force (climb).
		const float fmax = _param_gc_man_fmax.get();
		fx_body = math::deadzone(manual_sp.roll, dz) * fmax;
		fy_body = math::deadzone(manual_sp.pitch, dz) * fmax;
		fz_body = -mass * 9.81f - math::deadzone(manual_sp.throttle, dz) * fmax;
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);
	Vector3f f_des_body(fx_body, fy_body, fz_body);

	// Integrate rate commands into attitude reference (quaternion integration)
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

	// ── Attitude loop → desired torque (closed loop) ──
	const Dcmf R(R_q);
	const Dcmf R_sp(_q_att_ref);
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());

	Matrix3f J = matrix::diag(Vector3f(
					   _param_gc_inertia_xx.get(),
					   _param_gc_inertia_yy.get(),
					   _param_gc_inertia_zz.get()));

	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	// Open-loop force mode: velocity/position feedback disabled, so only the
	// rate/attitude integrator is updated (velocity integrator stays zeroed).
	if (!_landed) {
		updateIntegrators(e_omega, Vector3f(0.f, 0.f, 0.f), dt);
	}

	Vector3f tau_des = -kR.emult(e_R) - kOmega.emult(e_omega) + _rate_int + omega.cross(J * omega);

	const Vector3f f_des_world = R * f_des_body;   // for throttle-norm debug only

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, Vector3f(0.f, 0.f, 0.f), omega,
			  timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Position mode: cascaded position outer loop + velocity inner loop (closed loop).
// Requires a valid position estimate (GPS / optical-flow / VIO). Stick centered → hold.
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_position(const Quatf &R_q, const Vector3f &pos,
				       const Vector3f &vel, const Vector3f &omega,
				       uint64_t timestamp_sample)
{
	// Same no-RC fallback as manual mode: keep the last/zero setpoint so the
	// position loop holds hover instead of bailing out with a zero wrench.
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
		// Attitude: stick [-1,1] → angular rate [rad/s]
		p_sp = math::deadzone(manual_sp.roll,  dz) * rate_max;
		q_sp = math::deadzone(manual_sp.pitch, dz) * rate_max;
		r_sp = math::deadzone(manual_sp.yaw,   dz) * rate_max;

		// Stick [-1,1] → body velocity [m/s] (aux1/aux2 horizontal, throttle vertical)
		vx_sp_b = math::deadzone(manual_sp.aux1, dz) * vel_max;
		vy_sp_b = math::deadzone(manual_sp.aux2, dz) * vel_max;
		vz_sp_b = math::deadzone(manual_sp.throttle, dz) * vel_max;
	}

	Vector3f omega_sp(p_sp, q_sp, r_sp);
	Vector3f vel_sp_body(vx_sp_b, vy_sp_b, vz_sp_b);

	const Dcmf R(R_q);
	Vector3f vel_sp_world_stick = R * vel_sp_body;

	// Position outer loop: maintain a position setpoint. Follow the stick velocity
	// while the pilot is commanding motion; hold the current position when neutral.
	if (!_pos_sp_init) {
		_pos_sp = pos;
		_pos_sp_init = true;
	}

	if (vel_sp_world_stick.norm() > 0.05f) {
		_pos_sp += vel_sp_world_stick * dt;
	}

	Vector3f kp_pos(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f vel_sp_world = vel_sp_world_stick + kp_pos.emult(_pos_sp - pos);

	// Integrate rate commands into attitude reference (quaternion integration)
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

	// ── Velocity loop → desired force ──
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_v = vel - vel_sp_world;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kv.emult(e_v) - _vel_int - mass * g;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── Attitude loop → desired torque ──
	const Dcmf R_sp(_q_att_ref);
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());

	Matrix3f J = matrix::diag(Vector3f(
					   _param_gc_inertia_xx.get(),
					   _param_gc_inertia_yy.get(),
					   _param_gc_inertia_zz.get()));

	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	if (!_landed) {
		updateIntegrators(e_omega, e_v, dt);
	}

	Vector3f tau_des = -kR.emult(e_R) - kOmega.emult(e_omega) + _rate_int + omega.cross(J * omega);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, _q_att_ref, vel_sp_world, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Offboard mode: trajectory_setpoint → geometric control law
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step_offboard(const Quatf &R_q, const Vector3f &pos,
				       const Vector3f &vel, const Vector3f &omega,
				       uint64_t timestamp_sample)
{
	trajectory_setpoint_s traj_sp;

	if (!_sub_trajectory_sp.update(&traj_sp)) {
		return;
	}

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

	// ── Position + velocity loop → desired force ──
	Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_p = pos - pos_sp;
	Vector3f e_v = vel - vel_sp;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) - _vel_int - mass * g + mass * acc_sp;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── Attitude loop → desired torque ──
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());

	Vector3f omega_sp(0.f, 0.f, traj_sp.yawspeed);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	Matrix3f J = matrix::diag(Vector3f(
					   _param_gc_inertia_xx.get(),
					   _param_gc_inertia_yy.get(),
					   _param_gc_inertia_zz.get()));

	if (!_landed) {
		updateIntegrators(e_omega, e_v, dt);
	}

	Vector3f tau_des = -kR.emult(e_R) - kOmega.emult(e_omega) + _rate_int + omega.cross(J * omega);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Auto mode: flight_mode_manager setpoints → geometric control law
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

	// ── Position + velocity loop → desired force ──
	Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
	Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
	Vector3f e_p = pos - pos_sp;
	Vector3f e_v = vel - vel_sp;
	Vector3f g(0.f, 0.f, 9.81f);
	Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) - _vel_int - mass * g + mass * acc_sp;
	Vector3f f_des_body = R.transpose() * f_des_world;

	// ── Attitude loop → desired torque ──
	const Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

	Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
	Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());

	Vector3f omega_sp(0.f, 0.f, 0.f);
	Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
	Vector3f e_omega = omega - omega_d_in_B;

	Matrix3f J = matrix::diag(Vector3f(
					   _param_gc_inertia_xx.get(),
					   _param_gc_inertia_yy.get(),
					   _param_gc_inertia_zz.get()));

	if (!_landed) {
		updateIntegrators(e_omega, e_v, dt);
	}

	Vector3f tau_des = -kR.emult(e_R) - kOmega.emult(e_omega) + _rate_int + omega.cross(J * omega);

	publish_wrench(f_des_body, tau_des, timestamp_sample);
	publish_setpoints(f_des_world, f_des_body, tau_des, Quatf(R_sp), vel_sp, omega, timestamp_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main control dispatch
// ─────────────────────────────────────────────────────────────────────────────
void GeoControl::control_step(const vehicle_local_position_s &local_pos)
{
	vehicle_attitude_s att;
	vehicle_angular_velocity_s angular_vel;
	vehicle_status_s vehicle_status{};

	if (!_sub_vehicle_attitude.update(&att) ||
	    !_sub_vehicle_angular_velocity.update(&angular_vel)) {
		return;
	}

	_sub_vehicle_status.update(&vehicle_status);

	// Update landed/armed state
	vehicle_land_detected_s land_detected{};

	if (_sub_vehicle_land_detected.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	_armed = vehicle_status.arming_state == vehicle_status.ARMING_STATE_ARMED;

	// 未解锁时:清零积分器并发布零 wrench,使执行器处于中立初值
	// (否则会发布悬停推力 + 姿态保持力矩,导致分配器在零推力退化点把舵机推到饱和)
	if (!_armed) {
		resetIntegrators();
		_pos_sp_init = false;   // re-init position hold setpoint on next arm
		publish_wrench(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
			       local_pos.timestamp_sample);
		publish_setpoints(matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Quatf(1.f, 0.f, 0.f, 0.f),
				  matrix::Vector3f(0.f, 0.f, 0.f), matrix::Vector3f(0.f, 0.f, 0.f),
				  local_pos.timestamp_sample);
		return;
	}

	// Update saturation feedback from control allocator
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

	// State vectors
	const Quatf R_q(att.q);
	const Dcmf R(R_q);
	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
	const Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);

	// Determine control mode from nav_state
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
	return print_usage("unknown command");
}

int GeoControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Geometric controller for MorphQuad, outputting thrust and torque setpoints.

### Implementation
Implements a geometric tracking controller on SE(3) with three input modes:
- **Manual**: RC sticks (roll/pitch/yaw → angular rate, aux1/2/3 → body velocity)
- **Offboard**: trajectory_setpoint from companion computer
- **Auto**: flight_mode_manager setpoints (posctl, loiter, mission, ...)

Integrator with anti-windup:
- Rate/torque integrator: accumulates attitude error, clamped by GC_RATE_INT_LIM
- Velocity/force integrator: accumulates velocity error, clamped by GC_VEL_INT_LIM
- Anti-windup via control_allocator_status saturation feedback
- Integrator frozen when landed, reset when disarmed

### Examples
$ geo_control start
$ geo_control status
$ geo_control stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("geo_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int geo_control_main(int argc, char *argv[])
{
	return GeoControl::main(argc, argv);
}
