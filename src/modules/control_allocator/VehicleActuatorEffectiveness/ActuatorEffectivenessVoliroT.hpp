#pragma once
#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include "VoliroTAllocation.hpp"

class ActuatorEffectivenessVoliroT : public ModuleParams, public ActuatorEffectiveness
{
public:
	explicit ActuatorEffectivenessVoliroT(ModuleParams *parent) : ModuleParams(parent) {}
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason) override;
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &minimum, const ActuatorVector &maximum) override;
	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;
	const char *name() const override { return "Voliro T"; }
private:
	matrix::Vector3f _force_residual{}, _torque_residual{};
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
		(ParamFloat<px4::params::CA_CT_RATE>) _rate
	)
};
