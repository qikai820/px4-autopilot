#pragma once

#include <mathlib/mathlib.h>
#include <cmath>

namespace voliro_t
{
// Command-space protection for the reversible M5. Zero is an exact stop
// request; time at zero is not a measurement of rotor speed.
class TailMotor
{
public:
	struct Settings {
		float slew_time;       // seconds to traverse [-1, 1]; zero disables slew limiting
		float deadband;        // stop below this absolute command
		float hysteresis;      // start/reverse only above this absolute command
		float confirm_time;   // uninterrupted opposite request before reversal
		float zero_time;      // continuous zero-output dwell before reversal
	};

	void reset()
	{
		_output = 0.f;
		_direction = 0;
		_candidate = 0;
		_confirm_elapsed = 0.f;
		_zero_elapsed = 0.f;
	}

	float update(float request, float dt, const Settings &settings)
	{
		if (!std::isfinite(request) || !std::isfinite(dt) || dt <= 0.f) {
			// Preserve the last direction so a recovered opposite request still
			// needs a full dwell. Invalid input must never retain running output.
			_output = 0.f;
			_candidate = 0;
			_confirm_elapsed = 0.f;
			_zero_elapsed = 0.f;
			return _output;
		}

		request = math::constrain(request, -1.f, 1.f);
		const float off = math::constrain(settings.deadband, 0.f, 1.f);
		const float on = math::constrain(settings.hysteresis, off, 1.f);
		const float confirm_time = math::max(settings.confirm_time, 0.f);
		const float zero_time = math::max(settings.zero_time, 0.f);
		const bool was_zero = fabsf(_output) < 1e-7f;
		_zero_elapsed = was_zero ? math::min(_zero_elapsed + dt, zero_time + dt) : 0.f;
		const int requested_direction = fabsf(request) <= off ? 0 : (request > 0.f ? 1 : -1);
		float target = 0.f;

		if (requested_direction != 0 && requested_direction != _direction && _direction != 0
		    && fabsf(request) >= on) {
			if (_candidate != requested_direction) {
				_candidate = requested_direction;
				_confirm_elapsed = 0.f;

			} else {
				_confirm_elapsed = math::min(_confirm_elapsed + dt, confirm_time + dt);
			}

			// Brake to zero first. Never cross directly from one sign to the
			// other, even when slew limiting or the configured dwell is disabled.
			if (was_zero && _zero_elapsed >= zero_time && _confirm_elapsed >= confirm_time) {
				target = request;
				_direction = requested_direction;
			}

		} else {
			_candidate = 0;
			_confirm_elapsed = 0.f;

			// Schmitt trigger: keep a running direction down to 'off', but
			// require 'on' to start again from zero.
			if (requested_direction != 0 && (requested_direction == _direction || _direction == 0)
			    && (!was_zero || fabsf(request) >= on)) {
				target = request;
				_direction = requested_direction;
			}
		}

		if (settings.slew_time > 0.f) {
			const float step = 2.f * dt / settings.slew_time;
			_output += math::constrain(target - _output, -step, step);

		} else {
			_output = target;
		}

		// Snap only floating point roundoff, not a finite command (which
		// could violate the slew limit).
		if (fabsf(_output) < 1e-7f) {
			_output = 0.f;
		}

		if (!was_zero) {
			_zero_elapsed = 0.f;
		}

		return _output;
	}

private:
	float _output{0.f};
	int _direction{0};
	int _candidate{0};
	float _confirm_elapsed{0.f};
	float _zero_elapsed{0.f};
};
} // namespace voliro_t
