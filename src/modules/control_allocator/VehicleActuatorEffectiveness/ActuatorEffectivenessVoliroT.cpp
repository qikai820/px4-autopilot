#include "ActuatorEffectivenessVoliroT.hpp"
#include <drivers/drv_hrt.h>

using namespace matrix;
#include <lib/vector_control/VectorGeometry.hpp>
bool ActuatorEffectivenessVoliroT::getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason)
{
	(void)reason;

	for (int i = 0; i < 5; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f(0.f, 0.f, -1.f));
	}

	configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	return true;
}
void ActuatorEffectivenessVoliroT::updateSetpoint(const Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &minimum, const ActuatorVector &maximum)
{
	(void)matrix_index;
	const voliro_t::Geometry g{math::max(_x.get(), .01f), math::max(_y.get(), .01f), _z.get(),
				   math::max(_tail.get(), .01f), math::max(_front_max.get(), .01f),
				   math::max(_tail_max.get(), .01f), math::max(_km.get(), .0001f)};
	const float total_max = 4.f * g.front_max + g.tail_max;
	const Vector3f force(control_sp(3) * total_max, control_sp(4) * total_max, control_sp(5) * total_max);
	const Vector3f torque = Vector3f(control_sp(0), control_sp(1), control_sp(2)).emult(
					vector_control::torqueScale(_tq_roll.get(), _tq_pitch.get(), _tq_yaw.get()));
	const uint64_t now = hrt_absolute_time();
	const float dt = _last_update == 0 ? .0025f : math::constrain((now - _last_update) * 1e-6f, 0.f, .02f);
	_last_update = now;
	const float angle = math::radians(math::constrain(_angle.get(), 1.f, 179.f));
	voliro_t::Result r;

	if (force.isAllFinite() && torque.isAllFinite() && force.norm() + torque.norm() > 1e-6f) {
		r = voliro_t::allocate(g, force, torque, math::constrain(_tilt, -angle, angle), angle,
				       math::radians(math::max(_rate.get(), 1.f)) * dt);
	}

	actuator_sp(5) = math::constrain(r.tilt / angle, minimum(5), maximum(5));
	r.tilt = actuator_sp(5) * angle;
	_tilt = r.tilt;

	// A signed square-law model is used for M5; reverse thrust requires
	// CA_R_REV bit 4 and a calibrated reversible ESC/propeller combination.
	for (int i = 0; i < 5; ++i) {
		const float max_thrust = i == 4 ? g.tail_max : g.front_max;
		const float sign = r.thrust[i] < 0.f ? -1.f : 1.f;
		const float command = sign * sqrtf(fabsf(r.thrust[i]) / max_thrust);
		actuator_sp(i) = math::constrain(command, minimum(i), maximum(i));
		r.thrust[i] = actuator_sp(i) * fabsf(actuator_sp(i)) * max_thrust;
	}

	voliro_t::forward(g, r);
	_force_residual = force - r.force;
	_torque_residual = torque - r.torque;
}
void ActuatorEffectivenessVoliroT::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	(void)matrix_index;
	const float total_max = math::max(4.f * _front_max.get() + _tail_max.get(), .001f);
	(_force_residual / total_max).copyTo(status.unallocated_thrust);
	_torque_residual.edivide(vector_control::torqueScale(_tq_roll.get(), _tq_pitch.get(), _tq_yaw.get())).copyTo(status.unallocated_torque);
}
