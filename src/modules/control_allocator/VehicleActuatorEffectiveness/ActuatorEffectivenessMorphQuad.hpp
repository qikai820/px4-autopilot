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

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MQ_L>)     _param_ca_mq_l,      ///< half arm length [m]
		(ParamFloat<px4::params::CA_MQ_B>)     _param_ca_mq_b,      ///< half arm width [m]
		(ParamFloat<px4::params::CA_MQ_CT>)    _param_ca_mq_ct,     ///< thrust coefficient [N/(rad/s)^2]
		(ParamFloat<px4::params::CA_MQ_OMAX>)  _param_ca_mq_omax,   ///< max motor speed [rad/s]
		(ParamFloat<px4::params::CA_MQ_ALPM>)  _param_ca_mq_alpm,   ///< max alpha servo angle [deg]
		(ParamFloat<px4::params::CA_MQ_BETM>)  _param_ca_mq_betm,   ///< max beta servo angle [deg]
		(ParamFloat<px4::params::CA_MQ_DWG>)   _param_ca_mq_dwg,    ///< downwash cost weight (null-space)
		(ParamFloat<px4::params::CA_MQ_TMAX>)  _param_ca_mq_tmax,   ///< max per-rotor thrust at full throttle [N]
		(ParamFloat<px4::params::CA_MQ_NS_GAIN>) _param_ca_mq_ns_gain, ///< null-space descent gain k
		(ParamFloat<px4::params::CA_MQ_NS_POW>)  _param_ca_mq_ns_pow   ///< null-space motor-power cost weight
	)

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
	 * Recover servo angles from a thrust vector (Eq.14). The motor speed is
	 * no longer derived here; the caller normalizes the thrust magnitude into
	 * the [0,1] throttle fraction expected by the JSBSim morphquad model.
	 * @param t         3D thrust vector for one rotor
	 * @param idx       rotor index (0-3)
	 * @param alpha_out outer servo angle [rad]
	 * @param beta_out  inner servo angle [rad]
	 */
	void recoverServoAngles(const float t[3], int idx,
				float &alpha_out, float &beta_out);

	/**
	 * Compute the unallocated wrench when actuators are saturated.
	 * Compares requested vs achieved actuator outputs to determine
	 * how much of the requested force/torque could not be delivered.
	 */
	void computeUnallocatedWrench(const matrix::Vector3f &f_des, const matrix::Vector3f &tau_des,
				      const float omega_raw[NUM_ROTORS], const float alpha_raw[NUM_ROTORS],
				      const float beta_raw[NUM_ROTORS],
				      const ActuatorVector &actuator_sp,
				      const ActuatorVector &actuator_min,
				      const ActuatorVector &actuator_max);
};
