#pragma once
#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include "VoliroTAllocation.hpp"
#include "VoliroTTailMotor.hpp"

class ActuatorEffectivenessVoliroT : public ModuleParams, public ActuatorEffectiveness
{
public:
	explicit ActuatorEffectivenessVoliroT(ModuleParams *parent) : ModuleParams(parent) {}
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
	float _tail_request{0.f}, _tail_minimum{-1.f}, _tail_maximum{1.f};
	float _tilt{0.f};
	uint64_t _last_update{0};
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VEC_TQ_R>) _tq_roll,
		(ParamFloat<px4::params::VEC_TQ_P>) _tq_pitch,
		(ParamFloat<px4::params::VEC_TQ_Y>) _tq_yaw,
		(ParamFloat<px4::params::CA_CT_X>) _x,
		(ParamFloat<px4::params::CA_CT_Y>) _y,
		(ParamFloat<px4::params::CA_CT_Z>) _z,
		(ParamFloat<px4::params::CA_CT_TAIL>) _tail,
		(ParamFloat<px4::params::CA_CT_TMAX>) _front_max,
		(ParamFloat<px4::params::CA_CT_T5MAX>) _tail_max,
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
