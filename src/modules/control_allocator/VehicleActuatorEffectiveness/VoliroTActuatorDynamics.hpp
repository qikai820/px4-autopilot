#pragma once

#include "VoliroTAllocation.hpp"

namespace voliro_t
{

/** Optional prediction, NOT actuator measurement. Motor lag is on signed
 * normalized command before the square-law thrust map. Never substitute this
 * estimate for the allocator's final-command anti-windup residual. */
class ActuatorDynamics
{
public:
	void reset() { _tilt = 0.f; for (float &command : _commands) { command = 0.f; } }
	void initialize(const Result &state, const Geometry &g)
	{
		_tilt = state.tilt;

		for (int i = 0; i < 5; ++i) {
			const float normalized = state.thrust[i] / (i == 4 ? tailMax(g, state.thrust[i]) : g.front_max);
			_commands[i] = copysignf(sqrtf(fabsf(normalized)), normalized);
		}
	}
	Result update(const Geometry &g, const float commands[5], float tilt_command,
		      float motor_tau, float servo_tau, float servo_rate, float dt)
	{
		Result result;
		const float step = math::constrain(dt, 0.f, .02f);
		const float motor_alpha = motor_tau > 1e-4f ? 1.f - expf(-step / motor_tau) : 1.f;
		const float servo_alpha = servo_tau > 1e-4f ? 1.f - expf(-step / servo_tau) : 1.f;
		_tilt += math::constrain(servo_alpha * (tilt_command - _tilt), -servo_rate * step, servo_rate * step);
		result.tilt = _tilt;

		for (int i = 0; i < 5; ++i) {
			_commands[i] += motor_alpha * (commands[i] - _commands[i]);
			result.thrust[i] = i == 4 ? tailThrust(g, _commands[i]) : _commands[i] * fabsf(_commands[i]) * g.front_max;
		}

		forward(g, result);
		return result;
	}
private:
	float _commands[5] {};
	float _tilt{0.f};
};

} // namespace voliro_t
