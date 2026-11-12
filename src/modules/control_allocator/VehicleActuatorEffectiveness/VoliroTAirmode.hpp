#pragma once

#include "VoliroTAllocation.hpp"

namespace voliro_t
{
/** Keep pitch torque at the closest physically attainable value, then reserve
 * front-motor differential authority before fitting force. Used only for armed
 * manual direct-throttle AirMode; the original wrench remains the feedback target.
 * No servo motion is added by AirMode. All forces are N, moments are N*m. */
inline ForcePitchSolution allocateAirmodeAtTilt(const Geometry &g, const matrix::Vector3f &force,
		const matrix::Vector3f &torque, float tilt, bool yaw_airmode)
{
	const float sn = sinf(tilt), cs = cosf(tilt), h = g.z * sn + g.x * cs;
	const float max_front = 4.f * g.front_max;
	const float reverse = tailMax(g, -1.f);
	const float pitch = math::constrain(torque(1), math::min(0.f, h * max_front) - g.tail * g.tail_max,
					    math::max(0.f, h * max_front) + g.tail * reverse);
	float lower = 0.f, upper = max_front;

	if (fabsf(h) > 1e-5f) {
		const float a = (pitch - g.tail * reverse) / h;
		const float b = (pitch + g.tail * g.tail_max) / h;
		lower = math::constrain(math::min(a, b), 0.f, max_front);
		upper = math::constrain(math::max(a, b), lower, max_front);
	}

	// A front collective S can realize the projected roll differential d and
	// axial reaction r when max(|d|,|r|)<=S<=4*Tmax-max(|d|,|r|), provided
	// |d|+|r|<=2*Tmax. Roll keeps priority over an infeasible yaw request.
	const float differential = math::min(fabsf(cs * torque(0) + sn * torque(2)) / g.y, 2.f * g.front_max);
	const float reaction = yaw_airmode ? math::min(fabsf(reactionTorque(tilt, torque)) / g.km,
			       2.f * g.front_max - differential) : 0.f;
	const float reserve = math::max(differential, reaction);

	if (math::max(lower, reserve) <= math::min(upper, max_front - reserve)) {
		lower = math::max(lower, reserve);
		upper = math::min(upper, max_front - reserve);

	} else if (math::max(lower, differential) <= math::min(upper, max_front - differential)) {
		// Do not sacrifice feasible pitch/roll for yaw in full AirMode.
		lower = math::max(lower, differential);
		upper = math::min(upper, max_front - differential);
	}

	// Upright front lift and forward M5 thrust already provide opposite pitch
	// moments. Prefer that pair to an unnecessary M5 reversal. Reverse remains
	// available when the requested pitch/roll cannot otherwise be realized.
	if (fabsf(h) > 1e-5f) {
		const float a = pitch / h;
		const float b = (pitch + g.tail * g.tail_max) / h;
		const float forward_lower = math::max(lower, math::min(a, b));
		const float forward_upper = math::min(upper, math::max(a, b));

		if (forward_lower <= forward_upper) { lower = forward_lower; upper = forward_upper; }
	}

	const matrix::Vector2f axis(sn, -cs - h / g.tail);
	const matrix::Vector2f demand(force(0), force(2) - pitch / g.tail);
	const float sum = math::constrain(axis.dot(demand) / math::max(axis.norm_squared(), 1e-6f), lower, upper);
	return {tilt, sum, math::constrain((h * sum - pitch) / g.tail, -reverse, g.tail_max)};
}

// With tail thrust T fixed, |front_axis*S + tail_force| <= limit gives
// this upper root. T is bounded to +/-limit before this function is used.
inline float frontSumForForceLimit(float tilt, float tail, float limit)
{
	const float cs = cosf(tilt), sn = sinf(tilt);
	return math::max(0.f, -cs * tail + sqrtf(math::max(0.f, limit * limit - tail * tail * sn * sn)));
}

/** Constrain AirMode to the pilot's force request before pursuing torque.
 * A physical torque shortfall must remain a residual, not an unbounded lift
 * increase. The same calculation is repeated with the guarded M5 command. */
inline ForcePitchSolution boundAirmodeForce(const Geometry &g, const matrix::Vector3f &force,
		const ForcePitchSolution &wanted, float tail, float budget)
{
	const float limit = force.norm() + budget;
	tail = math::constrain(tail, -math::min(tailMax(g, -1.f), limit), math::min(g.tail_max, limit));
	const matrix::Vector2f axis(sinf(wanted.tilt), -cosf(wanted.tilt));
	const matrix::Vector2f front_force(force(0), force(2) + tail);
	const float center = axis.dot(front_force);
	const float perpendicular = math::max(0.f, front_force.norm_squared() - center * center);
	const float width = sqrtf(math::max(0.f, budget * budget - perpendicular));
	const float lower = math::constrain(center - width, 0.f, 4.f * g.front_max);
	const float upper = math::constrain(center + width, lower, 4.f * g.front_max);
	const float sum = math::min(math::constrain(wanted.front_sum, lower, upper),
				    frontSumForForceLimit(wanted.tilt, tail, limit));
	return {wanted.tilt, sum, tail};
}
} // namespace voliro_t
