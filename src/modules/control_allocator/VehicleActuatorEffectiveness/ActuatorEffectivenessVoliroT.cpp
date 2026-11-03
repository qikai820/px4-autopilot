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
	_geometry = g;
	_requested_force = force;
	_requested_torque = torque;
	const uint64_t now = hrt_absolute_time();
	const float dt = _last_update == 0 ? .0025f : math::constrain((now - _last_update) * 1e-6f, 0.f, .02f);
	_last_update = now;
	const float angle = math::radians(math::constrain(_angle.get(), 1.f, 179.f));
	voliro_t::Result r;

	if (force.isAllFinite() && torque.isAllFinite()) {
		r = voliro_t::allocate(g, force, torque, math::constrain(_tilt, -angle, angle), angle,
				       math::radians(math::max(_rate.get(), 1.f)) * dt);
	}

	actuator_sp(5) = math::constrain(r.tilt / angle, minimum(5), maximum(5));
	r.tilt = actuator_sp(5) * angle;

	// A signed square-law model is used for M5; reverse thrust requires
	// CA_R_REV bit 4 and a calibrated reversible ESC/propeller combination.
	for (int i = 0; i < 5; ++i) {
		const float max_thrust = i == 4 ? g.tail_max : g.front_max;
		const float sign = r.thrust[i] < 0.f ? -1.f : 1.f;
		const float command = sign * sqrtf(fabsf(r.thrust[i]) / max_thrust);
		actuator_sp(i) = math::constrain(command, minimum(i), maximum(i));
		r.thrust[i] = actuator_sp(i) * fabsf(actuator_sp(i)) * max_thrust;
	}

	_tail_request = actuator_sp(4);
	_tail_minimum = minimum(4);
	_tail_maximum = maximum(4);
}

void ActuatorEffectivenessVoliroT::updateSetpointAfterSlew(float dt, int matrix_index,
		ActuatorVector &actuator_sp, bool armed)
{
	(void)matrix_index;

	if (armed && _requested_force.isAllFinite() && _requested_torque.isAllFinite()) {
		// Use the original demand for direction/confirmation decisions. Using
		// the generic slew-limited value here could trap it below the start
		// threshold. The guard itself applies CA_R5_SLEW exactly once.
		const voliro_t::TailMotor::Settings settings{_tail_slew.get(), _tail_deadband.get(),
			_tail_hysteresis.get(), _tail_confirm.get(), _tail_wait.get()};
		actuator_sp(4) = math::constrain(_tail_motor.update(_tail_request, dt, settings),
					       _tail_minimum, _tail_maximum);

	} else {
		_tail_motor.reset();
		actuator_sp(4) = 0.f;
	}

	voliro_t::Result achieved;
	const float angle = math::radians(math::constrain(_angle.get(), 1.f, 179.f));
	achieved.tilt = actuator_sp(5) * angle;
	_tilt = achieved.tilt;

	for (int i = 0; i < 5; ++i) {
		const float max_thrust = i == 4 ? _geometry.tail_max : _geometry.front_max;
		achieved.thrust[i] = actuator_sp(i) * fabsf(actuator_sp(i)) * max_thrust;
	}

	// Include tail protection, all motor slew limits and actual servo command
	// in the residual used by the rate controller's anti-windup logic.
	voliro_t::forward(_geometry, achieved);
	_force_residual = _requested_force - achieved.force;
	_torque_residual = _requested_torque - achieved.torque;
}
void ActuatorEffectivenessVoliroT::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	(void)matrix_index;
	const float total_max = math::max(4.f * _front_max.get() + _tail_max.get(), .001f);
	(_force_residual / total_max).copyTo(status.unallocated_thrust);
	_torque_residual.edivide(vector_control::torqueScale(_tq_roll.get(), _tq_pitch.get(), _tq_yaw.get())).copyTo(status.unallocated_torque);
}
