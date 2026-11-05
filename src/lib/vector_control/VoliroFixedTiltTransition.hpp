#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <stdint.h>

namespace vector_control
{

/** Hand over to fixed-tilt allocation only after vector control has returned
 * the body AND the rotor axis to near level. A fast centering command while
 * the body is still pitched removes vertical lift, even with a slewed attitude
 * reference. The RC request must keep targeting zero independent pitch while
 * pending, with vector force allocation and reference feedforward intact. */
class VoliroFixedTiltTransition
{
public:

	static bool ready(const matrix::Quatf &attitude, const matrix::Quatf &reference,
			  const matrix::Vector3f &rates, float tilt, float tilt_command)
	{
		if (!attitude.isAllFinite() || !reference.isAllFinite() || !rates.isAllFinite()
		    || fabsf(attitude.norm_squared() - 1.f) > .01f || fabsf(reference.norm_squared() - 1.f) > .01f
		    || !PX4_ISFINITE(tilt) || !PX4_ISFINITE(tilt_command)) { return false; }

		const matrix::Dcmf actual(attitude), desired(reference);
		const float limit = math::radians(5.f);
		// Rz*Rx*Ry pitch stays unambiguous beyond +/-90 degrees. Do not use
		// principal Euler pitch: an inverted attitude can appear near zero.
		return actual(2, 2) > 0.f && desired(2, 2) > 0.f
		       && fabsf(atan2f(-actual(2, 0), actual(2, 2))) <= limit
		       && fabsf(atan2f(-desired(2, 0), desired(2, 2))) <= limit
		       && fabsf(tilt) <= limit && fabsf(tilt_command) <= limit
		       && rates.norm() <= math::radians(10.f);
	}

	bool update(bool requested, bool ready, uint64_t now)
	{
		_requested = requested;

		if (!requested) {
			_active = false;
			_ready_since = 0;

		} else if (!_active) {
			if (!ready) {
				_ready_since = 0;

			} else if (_ready_since == 0 || now < _ready_since) {
				_ready_since = now;

			} else if (now - _ready_since >= 200000) {
				_active = true;
			}
		}

		return _active;
	}

	bool pending() const { return _requested && !_active; }

private:

	uint64_t _ready_since{0};
	bool _requested{false}, _active{false};
};

} // namespace vector_control
