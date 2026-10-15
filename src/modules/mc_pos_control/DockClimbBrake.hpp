// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>

// Vertical trajectory for Dock braking. The local position reference and elapsed
// time belong to one braking session, even if Commander transfers Dock to RTL.
class DockClimbBrake
{
public:
	struct Limits {
		float height{0.f};       // additional height above the first braking point [m]
		float speed_up{0.f};     // [m/s]
		float accel_up{0.f};     // [m/s^2]
		float accel_down{0.f};   // commanded vertical deceleration [m/s^2]
		float jerk{0.f};         // change of commanded vertical acceleration [m/s^3]
		float duration{0.f};     // [s]
		float tilt_rate{0.f};    // [rad/s]
	};

	struct Output {
		bool active{false};
		bool capture{false};
		bool complete{false};
		bool allow_floor{false};
		float velocity_z{0.f};
		float height_used{0.f};
	};

	void reset()
	{
		_active = false;
		_failed = false;
		_capture = false;
		_complete = false;
		_start_z = NAN;
		_velocity_z = 0.f;
		_accel_z = 0.f;
		_elapsed = 0.f;
		_stable = 0.f;
		_capture_elapsed = 0.f;
	}
	void abort()
	{
		reset();
		_failed = true;
	}

	bool active() const { return _active; }
	bool complete() const { return _complete; }
	bool failed() const { return _failed; }

	Output update(float z, float vz, float speed_xy, float tilt, uint8_t z_reset_counter,
		      float delta_z, float dt, bool attitude_settled, const Limits &limits)
	{
		if (_failed) { return {}; }
		if (!std::isfinite(z) || !std::isfinite(vz) || !std::isfinite(speed_xy)
		    || !std::isfinite(dt) || dt <= 0.f || dt > .2f
		    || !std::isfinite(limits.height) || limits.height <= 0.f
		    || !std::isfinite(limits.speed_up) || limits.speed_up <= 0.f
		    || !std::isfinite(limits.accel_up) || limits.accel_up <= 0.f
		    || !std::isfinite(limits.accel_down) || limits.accel_down <= 0.f
		    || !std::isfinite(limits.jerk) || limits.jerk <= 0.f
		    || !std::isfinite(limits.duration) || limits.duration <= 0.f
		    || !std::isfinite(limits.tilt_rate) || limits.tilt_rate <= 0.f) {
			abort();
			return {};
		}

		if (!_active) {
			_active = true;
			_start_z = z;
			_z_reset_counter = z_reset_counter;
			_velocity_z = fmaxf(-limits.speed_up, fminf(vz, 0.f));
		}

		if (z_reset_counter != _z_reset_counter) {
			if (static_cast<uint8_t>(z_reset_counter - _z_reset_counter) != 1 || !std::isfinite(delta_z)) {
				abort();
				return {};
			}
			_start_z += delta_z;
			_z_reset_counter = z_reset_counter;
		}

		const float used = fmaxf(0.f, _start_z - z);
		const float remaining = fmaxf(0.f, limits.height - used);
		const float speed_up = fmaxf(0.f, -vz);
		// Reserve a second for attitude/throttle response and one metre for model error.
		// The current commanded tilt adds its finite leveling time to the reserve.
		const float response = 1.f + limits.accel_down / limits.jerk
				     + (std::isfinite(tilt) ? fmaxf(0.f, tilt) / limits.tilt_rate : 1.f);
		const float stopping_height = speed_up * speed_up / (2.f * limits.accel_down)
					    + speed_up * response + 1.f;
		_elapsed += dt;

		if (!_capture && (speed_xy < 2.f || remaining <= stopping_height || _elapsed >= limits.duration)) {
			_capture = true;
		}
		if (_capture) {
			_capture_elapsed += dt;
			if (_capture_elapsed > 10.f) {
				abort();
				return {};
			}
		}

		const float target_up = _capture ? 0.f : fminf(limits.speed_up,
							 sqrtf(2.f * limits.accel_down * fmaxf(0.f, remaining - stopping_height)));
		const float target_z = -target_up;
		const float desired_accel = target_z < _velocity_z ? -limits.accel_up
					  : (target_z > _velocity_z ? limits.accel_down : 0.f);
		const float accel_step = limits.jerk * dt;
		_accel_z += fmaxf(-accel_step, fminf(desired_accel - _accel_z, accel_step));
		const float next_velocity = _velocity_z + _accel_z * dt;
		if ((target_z - _velocity_z) * (target_z - next_velocity) <= 0.f) {
			_velocity_z = target_z;
			_accel_z = 0.f;
		} else {
			_velocity_z = next_velocity;
		}
		if (_velocity_z < -limits.speed_up || _velocity_z > 0.f) {
			_velocity_z = fmaxf(-limits.speed_up, fminf(_velocity_z, 0.f));
			_accel_z = 0.f;
		}

		if (_capture && speed_xy < .5f && fabsf(vz) < .5f && attitude_settled) {
			_stable += dt;
		} else {
			_stable = 0.f;
		}
		_complete = _complete || _stable >= .5f;
		return {_active, _capture, _complete, !_capture, _complete ? 0.f : _velocity_z, used};
	}

private:
	float _start_z{NAN};
	float _velocity_z{0.f};
	float _accel_z{0.f};
	float _elapsed{0.f};
	float _stable{0.f};
	float _capture_elapsed{0.f};
	uint8_t _z_reset_counter{0};
	bool _active{false};
	bool _capture{false};
	bool _complete{false};
	bool _failed{false};
};
