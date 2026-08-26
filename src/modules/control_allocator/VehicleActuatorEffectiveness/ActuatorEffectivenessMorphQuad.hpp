/****************************************************************************
 * MorphQuad actuator effectiveness / thrust allocation
 ****************************************************************************/
#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

class ActuatorEffectivenessMorphQuad : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMorphQuad(ModuleParams *parent);
	~ActuatorEffectivenessMorphQuad() override = default;

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
	static constexpr int NUM_THRUST_COMPONENTS = 3 * NUM_ROTORS;
	static constexpr int WRENCH_AXES = 6;
	static constexpr int NR = 2;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MQ_L>)     _param_ca_mq_l,      ///< half arm length [m]
		(ParamFloat<px4::params::CA_MQ_B>)     _param_ca_mq_b,      ///< half arm width [m]
		(ParamFloat<px4::params::CA_MQ_Z>)     _param_ca_mq_z,      ///< rotor-plane z relative to CG, BODY FRD [m]
		(ParamFloat<px4::params::CA_MQ_CT>)    _param_ca_mq_ct,     ///< thrust coefficient [N/(rad/s)^2]
		(ParamFloat<px4::params::CA_MQ_OMAX>)  _param_ca_mq_omax,   ///< maximum rotor-pair speed [rad/s]
		(ParamFloat<px4::params::CA_MQ_ALPM>)  _param_ca_mq_alpm,   ///< alpha mechanical limit [deg]
		(ParamFloat<px4::params::CA_MQ_BETM>)  _param_ca_mq_betm,   ///< beta mechanical limit [deg]
		(ParamFloat<px4::params::CA_MQ_GLG>)   _param_ca_mq_glg,    ///< retained for parameter compatibility; paper uses unit null-space coefficients
		(ParamFloat<px4::params::CA_MQ_DWG>)   _param_ca_mq_dwg,    ///< retained for parameter compatibility; paper uses unit null-space coefficients
		(ParamFloat<px4::params::CA_MQ_TMAX>)  _param_ca_mq_tmax,   ///< max rotor-system thrust [N]
		(ParamInt<px4::params::CA_MQ_TCURVE>)  _param_ca_mq_tcurve, ///< motor command/thrust mapping
		(ParamFloat<px4::params::CA_MQ_MMAX>)  _param_ca_mq_mmax,   ///< global normalized motor throttle limit
		(ParamFloat<px4::params::CA_MQ_SMAX>)  _param_ca_mq_smax,   ///< global servo angle limit [deg]
		(ParamFloat<px4::params::CA_MQ_LOW_THR>) _param_ca_mq_low_thr ///< throttle below which servo travel is reduced
	)

	float _alpha_prev[NUM_ROTORS] {};
	float _beta_prev[NUM_ROTORS] {};

	bool _motor_saturated{false};
	bool _servo_saturated{false};
	bool _allocation_saturated{false};
	float _unallocated_torque[3] {};
	float _unallocated_thrust[3] {};

	float commandFromThrust(float thrust, float thrust_max) const;
	float thrustFromCommand(float command, float thrust_max) const;

	/**
	 * Exact MorphQuad allocation structure from the paper:
	 *
	 *     w = M t,       M is 6x12
	 *     t0 = M^+ w
	 *     t  = t0 + N_M c
	 *     N_M is the paper's explicit null-space basis
	 *
	 * t contains the four 3-D thrust vectors in PX4 BODY FRD coordinates.
	 */
	bool computeThrustVectors(const matrix::Vector3f &f_des,
				  const matrix::Vector3f &tau_des,
				  float t_out[NUM_ROTORS][3]);

	/** Convert a thrust vector to continuous servo angles; returns true if angle limiting changed the direction. */
	bool recoverServoAngles(const float t[3], int idx,
				float &alpha_out, float &beta_out);

	void computeUnallocatedWrench(const matrix::Vector3f &f_des,
				      const matrix::Vector3f &tau_des,
				      const ActuatorVector &actuator_sp);

	/** Compute the paper's null-space coefficient vector c (Eq. 16). */
	matrix::Vector<float, WRENCH_AXES> computeNullspaceCoefficient(const matrix::Vector3f &f_des) const;
};
