#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace vector_control
{

/** Body-frame rate controller. Output and integrator are angular acceleration,
 * not normalized torque. Inertia/torque normalization belongs to the caller. */
class AngularAccelerationControl
{
public:
	void configure(const matrix::Vector3f &p, const matrix::Vector3f &i, const matrix::Vector3f &d,
		       float integral_limit, float anti_windup)
	{
		_p = p; _i = i; _d = d;
		_integral_limit = math::max(integral_limit, 0.f);
		_anti_windup = math::max(anti_windup, 0.f);
	}

	matrix::Vector3f update(const matrix::Vector3f &rate_error, const matrix::Vector3f &acceleration,
				const matrix::Vector3f &acceleration_ff, const matrix::Vector3f &unallocated_acceleration,
				float dt, bool landed)
	{
		if (!rate_error.isAllFinite() || !acceleration.isAllFinite() || !acceleration_ff.isAllFinite()
		    || !PX4_ISFINITE(dt) || dt <= 0.f || dt > .02f) {
			reset();
			return {};
		}

		if (landed) { reset(); }

		_feedback = _p.emult(rate_error) + _integral - _d.emult(acceleration - acceleration_ff);

		if (!landed) {
			for (int axis = 0; axis < 3; ++axis) {
				const float residual = PX4_ISFINITE(unallocated_acceleration(axis)) ? unallocated_acceleration(axis) : 0.f;
				float error = rate_error(axis);

				if (error * residual > 0.f) { error = 0.f; }

				_integral(axis) = math::constrain(_integral(axis)
								  + (_i(axis) * error - _anti_windup * residual) * dt,
								  -_integral_limit, _integral_limit);
			}
		}

		return _feedback;
	}

	void reset() { _integral.zero(); _feedback.zero(); }
	const matrix::Vector3f &integral() const { return _integral; }
	const matrix::Vector3f &feedback() const { return _feedback; }

private:
	matrix::Vector3f _p{}, _i{}, _d{}, _integral{}, _feedback{};
	float _integral_limit{0.f};
	float _anti_windup{0.f};
};

} // namespace vector_control
