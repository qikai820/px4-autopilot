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
 * Thrust allocation for MorphQuad morphable quadrotor.
 * Implements the algorithm from:
 *   "MorphQuad: Morphable Quadrotor for Superhuman Maneuverability"
 *   (arXiv:2607.02764)
 *
 * Uses Pattern B: registers zero effectiveness matrix and performs
 * all allocation logic in updateSetpoint().
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
	// Register 4 motors (thrust direction placeholder — actual allocation in updateSetpoint)
	for (int i = 0; i < NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{0.f, 0.f, -1.f});
	}

	// Register 8 servos (4 outer alpha + 4 inner beta)
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
	// Extract desired wrench from control setpoint
	// control_sp: [roll_torque, pitch_torque, yaw_torque, thrust_x, thrust_y, thrust_z]
	Vector3f f_des(control_sp(ControlAxis::THRUST_X),
		       control_sp(ControlAxis::THRUST_Y),
		       control_sp(ControlAxis::THRUST_Z));
	Vector3f tau_des(control_sp(ControlAxis::ROLL),
			 control_sp(ControlAxis::PITCH),
			 control_sp(ControlAxis::YAW));

	// Step 1+2: Compute thrust vectors (minimum-norm + null-space correction)
	float t[NUM_ROTORS][3];
	computeThrustVectors(f_des, tau_des, t);

	// Step 3: Recover servo angles and motor speeds
	float omega_raw[NUM_ROTORS];
	float alpha_raw[NUM_ROTORS];
	float beta_raw[NUM_ROTORS];

	const float tmax = _param_ca_mq_tmax.get();

	for (int i = 0; i < NUM_ROTORS; ++i) {
		float alpha, beta;
		recoverServoAngles(t[i], i, alpha, beta);

		// Motor command = normalized thrust fraction in [0,1] that the JSBSim
		// morphquad model expects for fcs/esc-cmd-norm (esc=0.4 -> hover thrust,
		// esc=1.0 -> TMAX per-rotor thrust). Previously this was omega/omega_max
		// (~0.009 at hover), which the model interpreted as ~zero thrust and
		// made the motors appear idle regardless of throttle.
		const float t_norm = sqrtf(t[i][0] * t[i][0] + t[i][1] * t[i][1] + t[i][2] * t[i][2]);
		omega_raw[i] = (t_norm < 0.01f) ? 0.f : (t_norm / tmax);
		alpha_raw[i] = alpha / M_PI_F;
		beta_raw[i] = beta / M_PI_F;
	}

	// Write raw (pre-saturation) values, then detect and record saturation
	_motor_saturated = false;
	_servo_saturated = false;

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Motor saturation check
		const float motor_clamped = math::constrain(omega_raw[i], actuator_min(i), actuator_max(i));

		if (fabsf(motor_clamped - omega_raw[i]) > FLT_EPSILON) {
			_motor_saturated = true;
		}

		actuator_sp(i) = motor_clamped;

		// Outer servo (alpha) saturation check
		const int alpha_idx = NUM_ROTORS + i;
		const float alpha_clamped = math::constrain(alpha_raw[i], actuator_min(alpha_idx), actuator_max(alpha_idx));

		if (fabsf(alpha_clamped - alpha_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(alpha_idx) = alpha_clamped;

		// Inner servo (beta) saturation check
		const int beta_idx = 2 * NUM_ROTORS + i;
		const float beta_clamped = math::constrain(beta_raw[i], actuator_min(beta_idx), actuator_max(beta_idx));

		if (fabsf(beta_clamped - beta_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(beta_idx) = beta_clamped;
	}

	// Compute unallocated wrench from saturation
	// When saturated, the actual output differs from the requested output.
	computeUnallocatedWrench(f_des, tau_des, omega_raw, alpha_raw, beta_raw,
				 actuator_sp, actuator_min, actuator_max);
}

void ActuatorEffectivenessMorphQuad::computeThrustVectors(
	const Vector3f &f_des, const Vector3f &tau_des,
	float t_out[NUM_ROTORS][3])
{
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float r = sqrtf(l * l + b * b);
	const float gimbal_lock_gain = _param_ca_mq_glg.get();
	const float downwash_gain = _param_ca_mq_dwg.get();

	// X-layout sign patterns (rotor order: FR=0, FL=1, BL=2, BR=3)
	// roll_sign:  +1 for rotors on the -y side (FR, BR), -1 for +y side (FL, BL)
	const float roll_sign[NUM_ROTORS]  = {+1.0f, -1.0f, -1.0f, +1.0f};
	// pitch_sign: -1 for front (+x), +1 for back (-x)
	const float pitch_sign[NUM_ROTORS] = {-1.0f, -1.0f, +1.0f, +1.0f};

	// Yaw unit vectors (Eq.13): perpendicular to rotor arm in xy-plane
	// e_psi_i = [-l_iy/r, +l_ix/r, 0]
	// Rotor positions: FR(+l,-b), FL(+l,+b), BL(-l,+b), BR(-l,-b)
	const float eps_x[NUM_ROTORS] = {-b / r,  b / r,  b / r, -b / r};
	const float eps_y[NUM_ROTORS] = { l / r,  l / r, -l / r, -l / r};

	// Step 1: Minimum-norm decomposition (Eq.12)
	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Force sharing + roll torque + yaw torque (tangential)
		t_out[i][0] = f_des(0) / 4.0f
			      + roll_sign[i] * tau_des(0) / (4.0f * b)
			      + tau_des(2) * eps_x[i] / (4.0f * r);

		// Force sharing + pitch torque + yaw torque (tangential)
		// Corrected sign: pitch contribution uses -pitch_sign to match
		// the wrench equation tau_y = sum(l_ix * t_iz - l_iz * t_ix)
		t_out[i][1] = f_des(1) / 4.0f
			      - pitch_sign[i] * tau_des(1) / (4.0f * l)
			      + tau_des(2) * eps_y[i] / (4.0f * r);

		// z-component: only force sharing (torque from z-force is coupled
		// with roll/pitch through arm offsets, handled by the above terms)
		t_out[i][2] = f_des(2) / 4.0f;
	}

	// Step 2: Null-space correction (Eq.15-16)
	const float f_norm = f_des.norm();

	if (f_norm > 0.01f) {
		const float f_hat_y = f_des(1) / f_norm;
		const float f_hat_x = f_des(0) / f_norm;

		// (a) Gimbal lock avoidance: repel beta_i away from +/-pi/2
		// When f is aligned with j^B (y-axis), inner servo loses control.
		// Add alternating y-component correction proportional to |f_hat . j^B|.
		const float y_corr = gimbal_lock_gain * fabsf(f_hat_y) * f_norm / 4.0f;

		for (int i = 0; i < NUM_ROTORS; ++i) {
			// Alternating sign: (-1)^i
			const float sign = (i % 2 == 0) ? 1.0f : -1.0f;
			t_out[i][1] += sign * y_corr;
		}

		// (b) Downwash avoidance: break thrust collinearity when f || i^B
		// When |f_hat . i^B| > 0.7, add alternating x-component to front/back pairs.
		if (fabsf(f_hat_x) > 0.7f) {
			const float x_corr = downwash_gain * fabsf(f_hat_x) * f_norm / 10.0f;

			for (int i = 0; i < NUM_ROTORS; ++i) {
				// Front pair (0,1): +x_corr, Back pair (2,3): -x_corr
				const float sign = (i < 2) ? 1.0f : -1.0f;
				t_out[i][0] += sign * x_corr;
			}
		}
	}
}

void ActuatorEffectivenessMorphQuad::recoverServoAngles(
	const float t[3], int idx,
	float &alpha_out, float &beta_out)
{
	const float t_norm = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);

	// If thrust is negligible, keep previous angles to avoid jitter
	if (t_norm < 0.01f) {
		alpha_out = _alpha_prev[idx];
		beta_out = _beta_prev[idx];
		return;
	}

	// Unit thrust direction
	const float t_hat_x = t[0] / t_norm;
	const float t_hat_y = t[1] / t_norm;
	const float t_hat_z = t[2] / t_norm;

	// Two-branch solution (Eq.14)
	// Branch 1: beta in [-pi/2, pi/2]
	const float beta1 = asinf(math::constrain(t_hat_y, -1.0f, 1.0f));
	const float alpha1 = atan2f(t_hat_x, -t_hat_z);

	// Branch 2: beta in [pi/2, 3*pi/2]
	const float beta2 = M_PI_F - beta1;
	const float alpha2 = M_PI_F + alpha1;

	// Minimal-motion selection
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

	// Multi-revolution wrap check: find the wrapping that minimizes motion
	float best_dist = fabsf(alpha - _alpha_prev[idx]) + fabsf(beta - _beta_prev[idx]);
	float best_alpha = alpha;

	for (int k = -NR; k <= NR; ++k) {
		if (k == 0) {
			continue;
		}

		const float alpha_wrap = alpha + 2.0f * M_PI_F * k;
		const float dist = fabsf(alpha_wrap - _alpha_prev[idx]) + fabsf(beta - _beta_prev[idx]);

		if (dist < best_dist) {
			best_dist = dist;
			best_alpha = alpha_wrap;
		}
	}

	alpha = best_alpha;

	// Clamp to mechanical limits from parameters
	const float alpha_max_rad = math::radians(_param_ca_mq_alpm.get());
	const float beta_max_rad = math::radians(_param_ca_mq_betm.get());
	alpha = math::constrain(alpha, -alpha_max_rad, alpha_max_rad);
	beta = math::constrain(beta, -beta_max_rad, beta_max_rad);

	// Update history
	_alpha_prev[idx] = alpha;
	_beta_prev[idx] = beta;

	alpha_out = alpha;
	beta_out = beta;
}

void ActuatorEffectivenessMorphQuad::getUnallocatedControl(int matrix_index,
		control_allocator_status_s &status)
{
	// Override the framework's default unallocated control with our
	// saturation-aware computation from computeUnallocatedWrench().
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
	// Initialize as zero (no unallocated control)
	for (int i = 0; i < 3; i++) {
		_unallocated_torque[i] = 0.f;
		_unallocated_thrust[i] = 0.f;
	}

	// If nothing saturated, the full wrench was achieved
	if (!_motor_saturated && !_servo_saturated) {
		return;
	}

	// Reconstruct the actually achieved wrench from the clipped actuator outputs.
	// For each rotor, compute the actual thrust vector from the saturated outputs.
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float tmax = _param_ca_mq_tmax.get();

	Vector3f f_achieved(0.f, 0.f, 0.f);
	Vector3f tau_achieved(0.f, 0.f, 0.f);

	// Rotor positions (X-layout: FR, FL, BL, BR)
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = {-b,  b,  b, -b};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Recover actual alpha, beta from saturated actuator outputs
		const float alpha_actual = actuator_sp(NUM_ROTORS + i) * M_PI_F;
		const float beta_actual = actuator_sp(2 * NUM_ROTORS + i) * M_PI_F;

		// Achieved thrust magnitude: actuator_sp(i) is the normalized thrust
		// fraction in [0,1] (esc-cmd-norm), so T = fraction * TMAX.
		const float t_mag = actuator_sp(i) * tmax;

		// Thrust direction from dual-axis gimbal angles (alpha=outer/Yaw, beta=inner/Pitch)
		// Forward kinematics: R = Ry(alpha) * Rx(beta), thrust along local z
		const float sa = sinf(alpha_actual);
		const float ca = cosf(alpha_actual);
		const float sb = sinf(beta_actual);
		const float cb = cosf(beta_actual);
		Vector3f t_vec(t_mag * sa * cb, t_mag * sb, -t_mag * ca * cb);

		// Accumulate force
		f_achieved += t_vec;

		// Accumulate torque: tau = l_i x t_i
		Vector3f l_i(lx[i], ly[i], 0.f);
		tau_achieved += l_i.cross(t_vec);
	}

	// Unallocated = requested - achieved
	const Vector3f unalloc_f = f_des - f_achieved;
	const Vector3f unalloc_tau = tau_des - tau_achieved;

	for (int i = 0; i < 3; i++) {
		_unallocated_thrust[i] = unalloc_f(i);
		_unallocated_torque[i] = unalloc_tau(i);
	}
}
