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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

class ActuatorEffectivenessMorphQuad : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMorphQuad(ModuleParams *parent);
	virtual ~ActuatorEffectivenessMorphQuad() = default;

	bool getEffectivenessMatrix(Configuration &configuration,
				    EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "MorphQuad"; }

private:
	static constexpr int NUM_ROTORS = 4;
	static constexpr int NR = 2;

	// Geometry
	float _half_length{0.17f};    ///< l: half arm length in x direction [m]
	float _half_width{0.17f};     ///< b: half arm width in y direction [m]
	float _thrust_coeff{1.0f};    ///< c_t: thrust coefficient [N/(rad/s)^2]
	float _omega_max{1200.0f};    ///< max motor angular velocity [rad/s]

	// Null-space correction gains
	float _gimbal_lock_gain{0.3f};
	float _downwash_gain{0.2f};

	// Previous servo angles for minimal-motion selection
	float _alpha_prev[NUM_ROTORS] {};
	float _beta_prev[NUM_ROTORS] {};

	// Saturation tracking for anti-windup feedback to geo_control
	bool _motor_saturated{false};     ///< any motor hit omega_max limit
	bool _servo_saturated{false};     ///< any servo hit angle limit
	float _unallocated_torque[3] {};  ///< residual torque not achieved
	float _unallocated_thrust[3] {};  ///< residual thrust not achieved

	/**
	 * Compute 4 thrust vectors from desired wrench (Eq.12 + Eq.15-16).
	 * @param f_des  desired force in body frame [N]
	 * @param tau_des desired torque in body frame [Nm]
	 * @param t_out  output 4x3 thrust vectors
	 */
	void computeThrustVectors(const matrix::Vector3f &f_des, const matrix::Vector3f &tau_des,
				  float t_out[NUM_ROTORS][3]);

	/**
	 * Recover servo angles and motor speed from a thrust vector (Eq.14).
	 * @param t         3D thrust vector for one rotor
	 * @param idx       rotor index (0-3)
	 * @param alpha_out outer servo angle [rad]
	 * @param beta_out  inner servo angle [rad]
	 * @param omega_out motor angular velocity [rad/s]
	 */
	void recoverServoAngles(const float t[3], int idx,
				float &alpha_out, float &beta_out, float &omega_out);

	/**
	 * Compute the unallocated wrench when actuators are saturated.
	 * Compares requested vs achieved actuator outputs to determine
	 * how much of the requested force/torque could not be delivered.
	 */
	void computeUnallocatedWrench(const matrix::Vector3f &f_des, const matrix::Vector3f &tau_des,
				      const float omega_raw[NUM_ROTORS], const float alpha_raw[NUM_ROTORS],
				      const ActuatorVector &actuator_sp,
				      const ActuatorVector &actuator_min,
				      const ActuatorVector &actuator_max);
};
