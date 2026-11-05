#pragma once
#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include "VoliroTAllocation.hpp"
#include "VoliroTTailMotor.hpp"
#include "VoliroTActuatorDynamics.hpp"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vector_actuator_status.h>
#include <uORB/topics/voliro_tilt_status.h>

class ActuatorEffectivenessVoliroT : public ModuleParams, public ActuatorEffectiveness
{
public:
	explicit ActuatorEffectivenessVoliroT(ModuleParams *parent) : ModuleParams(parent)
	{
		_dynamics_enabled = _control_mode.get() == 1;
	}
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason) override;
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &minimum, const ActuatorVector &maximum) override;
	void updateSetpointAfterSlew(float dt, int matrix_index, ActuatorVector &actuator_sp, bool armed) override;
	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;
	const char *name() const override { return "Voliro T"; }
private:
	matrix::Vector3f _force_residual{}, _torque_residual{};
	matrix::Vector3f _requested_force{}, _requested_torque{};
	voliro_t::Geometry _geometry{};
	voliro_t::TailMotor _tail_motor;
	voliro_t::ReactionTorqueFilter _reaction_filter;
	voliro_t::ActuatorDynamics _actuator_prediction;
	uORB::Publication<vector_actuator_status_s> _status_pub{ORB_ID(vector_actuator_status)};
	uORB::Subscription _tilt_status_sub{ORB_ID(voliro_tilt_status)};
	bool _dynamics_enabled{false};
	bool _armed{false}, _fallback_latched{false};
	bool _fixed_tilt_active{false};
	bool _fallback_observation_fresh{false};
	float _fallback_tilt{0.f};
	uint64_t _last_disarmed{0};
	float _reaction_requested{0.f}, _reaction_filtered{0.f};
	uint64_t _last_status{0};
	float _tail_request{0.f}, _tail_minimum{-1.f}, _tail_maximum{1.f};
	float _tilt{0.f};
	uint64_t _last_update{0};
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VT_CTRL_MODE>) _control_mode,
		(ParamBool<px4::params::VT_TILT_EN>) _tilt_monitor_enable,
		(ParamFloat<px4::params::VT_PITCH_RATE>) _fallback_pitch_rate,
		(ParamFloat<px4::params::VT_REACT_FILT>) _reaction_cutoff,
		(ParamFloat<px4::params::VT_MOT_TAU>) _motor_tau,
		(ParamFloat<px4::params::VT_SRV_TAU>) _servo_tau,
		(ParamFloat<px4::params::VEC_TQ_R>) _tq_roll,
		(ParamFloat<px4::params::VEC_TQ_P>) _tq_pitch,
		(ParamFloat<px4::params::VEC_TQ_Y>) _tq_yaw,
		(ParamFloat<px4::params::CA_CT_X>) _x,
		(ParamFloat<px4::params::CA_CT_Y>) _y,
		(ParamFloat<px4::params::CA_CT_Z>) _z,
		(ParamFloat<px4::params::CA_CT_TAIL>) _tail,
		(ParamFloat<px4::params::CA_CT_TMAX>) _front_max,
		(ParamFloat<px4::params::CA_CT_T5MAX>) _tail_max,
		(ParamFloat<px4::params::CA_CT_T5RMAX>) _tail_reverse_max,
		(ParamFloat<px4::params::CA_CT_KM>) _km,
		(ParamFloat<px4::params::CA_CT_ANGLE>) _angle,
		(ParamFloat<px4::params::CA_CT_RATE>) _rate,
		(ParamFloat<px4::params::CA_R5_SLEW>) _tail_slew,
		(ParamFloat<px4::params::CA_CT_T5_DZ>) _tail_deadband,
		(ParamFloat<px4::params::CA_CT_T5_HYST>) _tail_hysteresis,
		(ParamFloat<px4::params::CA_CT_T5_HOLD>) _tail_confirm,
		(ParamFloat<px4::params::CA_CT_T5_WAIT>) _tail_wait
	)
};
