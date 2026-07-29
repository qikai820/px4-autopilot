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

	for (int i = 0; i < NUM_ROTORS; ++i) {
		float alpha, beta, omega;
		recoverServoAngles(t[i], i, alpha, beta, omega);

		omega_raw[i] = omega / _omega_max;
		alpha_raw[i] = alpha / M_PI_F;
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

		// Servo saturation check
		const int sv_idx = NUM_ROTORS + i;
		const float servo_clamped = math::constrain(alpha_raw[i], actuator_min(sv_idx), actuator_max(sv_idx));

		if (fabsf(servo_clamped - alpha_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(sv_idx) = servo_clamped;
	}

	// Compute unallocated wrench from saturation
	// When saturated, the actual output differs from the requested output.
	// The ratio of actual/requested gives the achieved fraction per axis.
	computeUnallocatedWrench(f_des, tau_des, omega_raw, alpha_raw,
				 actuator_sp, actuator_min, actuator_max);
}

void ActuatorEffectivenessMorphQuad::computeThrustVectors(
	const Vector3f &f_des, const Vector3f &tau_des,
	float t_out[NUM_ROTORS][3])
{
	const float l = _half_length;
	const float b = _half_width;
	const float r = sqrtf(l * l + b * b);

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
		const float y_corr = _gimbal_lock_gain * fabsf(f_hat_y) * f_norm / 4.0f;

		for (int i = 0; i < NUM_ROTORS; ++i) {
			// Alternating sign: (-1)^i
			const float sign = (i % 2 == 0) ? 1.0f : -1.0f;
			t_out[i][1] += sign * y_corr;
		}

		// (b) Downwash avoidance: break thrust collinearity when f || i^B
		// When |f_hat . i^B| > 0.7, add alternating x-component to front/back pairs.
		if (fabsf(f_hat_x) > 0.7f) {
			const float x_corr = _downwash_gain * fabsf(f_hat_x) * f_norm / 10.0f;

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
	float &alpha_out, float &beta_out, float &omega_out)
{
	const float t_norm = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);

	// Motor speed: Omega = ||t|| / c_t
	omega_out = t_norm / _thrust_coeff;

	// If thrust is negligible, keep previous angles to avoid jitter
	if (t_norm < 0.01f) {
		alpha_out = _alpha_prev[idx];
		beta_out = _beta_prev[idx];
		omega_out = 0.f;
		return;
	}

	// Unit thrust direction
	const float t_hat_x = t[0] / t_norm;
	const float t_hat_y = t[1] / t_norm;
	const float t_hat_z = t[2] / t_norm;

	// Two-branch solution (Eq.14)
	// Branch 1: beta in [-pi/2, pi/2]
	const float beta1 = asinf(math::constrain(-t_hat_y, -1.0f, 1.0f));
	const float alpha1 = atan2f(t_hat_x, t_hat_z);

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

	// Clamp beta to mechanical limits [-pi/2, 3*pi/2]
	beta = math::constrain(beta, -M_PI_F / 2.0f, 3.0f * M_PI_F / 2.0f);

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
	const float l = _half_length;
	const float b = _half_width;

	Vector3f f_achieved(0.f, 0.f, 0.f);
	Vector3f tau_achieved(0.f, 0.f, 0.f);

	// Rotor positions (X-layout: FR, FL, BL, BR)
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = {-b,  b,  b, -b};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Recover actual omega and alpha from saturated actuator outputs
		const float omega_actual = actuator_sp(i) * _omega_max;
		const float alpha_actual = actuator_sp(NUM_ROTORS + i) * M_PI_F;

		// Thrust magnitude from actual motor speed: t = c_t * Omega^2
		const float t_mag = _thrust_coeff * omega_actual * omega_actual;

		// Thrust direction from actual servo angle (simplified: only outer servo alpha)
		// G_i(alpha) = [sin(alpha), 0, cos(alpha)] (NED, ignoring beta for saturation estimate)
		const float sa = sinf(alpha_actual);
		const float ca = cosf(alpha_actual);
		Vector3f t_vec(t_mag * sa, 0.f, t_mag * ca);

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
