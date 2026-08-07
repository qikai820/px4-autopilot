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

		// Rotor physical model: T_i = c_T * Ω_i^2  =>  Ω_i = sqrt(T_i / c_T).
		// The JSBSim morphquad model consumes esc-cmd-norm = Ω / Ω_max, where
		// esc=1.0 corresponds to TMAX per-rotor thrust. Since TMAX = c_T * Ω_max^2,
		// the normalized motor command is sqrt(T_i / TMAX) — NOT the linear
		// T_i/TMAX. Using T_i/TMAX (a previous bug) linearizes the rotor and
		// produces the wrong (nonlinear) motor response at hover and full throttle.
		const float t_norm = sqrtf(t[i][0] * t[i][0] + t[i][1] * t[i][1] + t[i][2] * t[i][2]);
		omega_raw[i] = (t_norm < 0.01f) ? 0.f : sqrtf(t_norm / tmax);
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

		// Outer servo (alpha) — interleaved per motor: actuator_sp[4 + 2*i]
		const int alpha_idx = 4 + 2 * i;
		const float alpha_clamped = math::constrain(alpha_raw[i], actuator_min(alpha_idx), actuator_max(alpha_idx));

		if (fabsf(alpha_clamped - alpha_raw[i]) > FLT_EPSILON) {
			_servo_saturated = true;
		}

		actuator_sp(alpha_idx) = alpha_clamped;

		// Inner servo (beta) — interleaved per motor: actuator_sp[5 + 2*i]
		const int beta_idx = 5 + 2 * i;
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

	// Rotor arm positions (X-layout: RF, LF, LB, RB, CCW from RF)
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = { b, -b, -b,  b};

	// Wrench W = [force; torque] (body frame, Newtons / N·m)
	matrix::Vector<float, 6> W;
	W(0) = f_des(0); W(1) = f_des(1); W(2) = f_des(2);
	W(3) = tau_des(0); W(4) = tau_des(1); W(5) = tau_des(2);

	// ── Build the allocation matrix A_g(q) from the CURRENT (warm-started) ──
	// ── gimbal angles.  f_i = T_i g_i ,  τ_i = r_i × f_i  ⇒  W = A_g T.   ──
	// g_i = R_x(α_i) R_y(β_i) e_3 — must match recoverServoAngles() forward kinematics.
	matrix::Matrix<float, 6, NUM_ROTORS> A_g;
	for (int i = 0; i < NUM_ROTORS; ++i) {
		const float sa = sinf(_alpha_prev[i]);
		const float ca = cosf(_alpha_prev[i]);
		const float sb = sinf(_beta_prev[i]);
		const float cb = cosf(_beta_prev[i]);
		const Vector3f g(sa * cb, sb, -ca * cb);
		A_g(0, i) = g(0); A_g(1, i) = g(1); A_g(2, i) = g(2);
		const Vector3f r_i(lx[i], ly[i], 0.f);
		const Vector3f r_cross_g = r_i.cross(g);
		A_g(3, i) = r_cross_g(0); A_g(4, i) = r_cross_g(1); A_g(5, i) = r_cross_g(2);
	}

	// ── Step 1: minimum-norm thrust magnitudes u0 = A_g^+ W ──
	matrix::Matrix<float, NUM_ROTORS, 6> A_pinv;

	if (!matrix::geninv(A_g, A_pinv)) {
		// Degenerate geometry (should not happen): fall back to zero thrust.
		for (int i = 0; i < NUM_ROTORS; ++i) { t_out[i][0] = t_out[i][1] = t_out[i][2] = 0.f; }

		return;
	}

	const matrix::Vector<float, NUM_ROTORS> u0 = A_pinv * W;

	// ── Null space N = I - A_g^+ A_g. Any z in this space satisfies A_g N z = 0, ──
	// ── so u = u0 + N z realises the SAME wrench W (ANz = 0 is guaranteed,   ──
	// ── unlike the previous ad-hoc component perturbations).                   ──
	const matrix::Matrix<float, NUM_ROTORS, NUM_ROTORS> I = matrix::eye<float, NUM_ROTORS>();
	const matrix::Matrix<float, NUM_ROTORS, NUM_ROTORS> N_mat = I - A_pinv * A_g;

	// ── Step 2: null-space optimisation — minimise a cost J(u) over the ──
	// ── redundant DOF by gradient descent: z = -k * grad(J).            ──
	// Cost terms with non-zero gradient w.r.t. u (so the projection bites):
	//   • motor power : J_pow = w_pow * Σ u_i^2        → ∂/∂u_i = 2 w_pow u_i
	//   • downwash    : J_dw  = w_dw  * Σ d_ij u_i u_j → ∂/∂u_i = w_dw Σ (d_ij+d_ji) u_j
	// (Gimbal-lock / servo-angle avoidance is directional (α,β) and handled by the
	//  IK branch selection in recoverServoAngles(); it is not part of this 4-DOF u-space.)
	const float w_pow = _param_ca_mq_ns_pow.get();
	const float w_dw  = _param_ca_mq_dwg.get();

	// Downwash interaction weights d_ij: rotor j sits in the slipstream of rotor i
	// when j is downstream of i's thrust axis and close to it.
	float d_ij[NUM_ROTORS][NUM_ROTORS] {};

	if (w_dw > 0.f) {
		for (int i = 0; i < NUM_ROTORS; ++i) {
			const float sa = sinf(_alpha_prev[i]);
			const float ca = cosf(_alpha_prev[i]);
			const float sb = sinf(_beta_prev[i]);
			const float cb = cosf(_beta_prev[i]);
			const Vector3f g_i(sa * cb, sb, -ca * cb);

			for (int j = 0; j < NUM_ROTORS; ++j) {
				if (i == j) { continue; }

				const Vector3f rel(lx[j] - lx[i], ly[j] - ly[i], 0.f);
				const float along = -rel.dot(g_i);   // >0: j downstream of i

				if (along <= 0.f) { continue; }

				const float perp = sqrtf(math::max(rel.dot(rel) - along * along, 0.f));
				d_ij[i][j] = along / (perp + 0.05f);
			}
		}
	}

	matrix::Vector<float, NUM_ROTORS> grad_J{};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		grad_J(i) += 2.f * w_pow * u0(i);

		if (w_dw > 0.f) {
			float g_dw = 0.f;

			for (int j = 0; j < NUM_ROTORS; ++j) {
				if (i == j) { continue; }

				g_dw += (d_ij[i][j] + d_ij[j][i]) * u0(j);
			}

			grad_J(i) += w_dw * g_dw;
		}
	}

	const float k_ns = _param_ca_mq_ns_gain.get();
	const matrix::Vector<float, NUM_ROTORS> z = -k_ns * grad_J;
	const matrix::Vector<float, NUM_ROTORS> u = u0 + N_mat * z;

	// Reconstruct the thrust VECTOR from the optimised magnitude and the current
	// direction. Negative thrust is unphysical — clamp. Downstream IK keeps the
	// recovered α,β consistent with this direction.
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

	// Rotor positions (X-layout: RF, LF, LB, RB, counter-clockwise from RF)
	const float lx[NUM_ROTORS] = { l,  l, -l, -l};
	const float ly[NUM_ROTORS] = { b, -b, -b,  b};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Recover actual alpha, beta from saturated actuator outputs
		// (interleaved per motor: alpha at 4+2*i, beta at 5+2*i)
		const float alpha_actual = actuator_sp(4 + 2 * i) * M_PI_F;
		const float beta_actual  = actuator_sp(5 + 2 * i) * M_PI_F;

		// Achieved thrust magnitude: actuator_sp(i) is the normalized rotor
		// speed (esc-cmd-norm = Ω/Ω_max), so T = (esc)^2 * TMAX = c_T * Ω^2.
		const float t_mag = actuator_sp(i) * actuator_sp(i) * tmax;

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
