#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

namespace vector_control
{
// Keep upright vertical thrust unchanged and blend weak-axis P between 10 and
// 30 degrees. Inverted thrust still has the same weak reaction-torque axis:
// do not restore high P as sin(tilt) falls again beyond 150 degrees.
inline float reactionProportionalScale(float tilt, float minimum_scale)
{
	const float exposure = cosf(tilt) >= 0.f ? fabsf(sinf(tilt)) : 1.f;
	const float blend = math::constrain((exposure - .17364818f) / (.5f - .17364818f), 0.f, 1.f);
	return 1.f + (math::constrain(minimum_scale, .1f, 1.f) - 1.f) * blend * blend * (3.f - 2.f * blend);
}

// Project physical torque, not normalized torque: VEC_TQ_R/P/Y can differ.
// Only the P contribution is shaped; D damping, integral trim authority and
// reference feedforward are retained. Allocation remains the authority on limits.
inline matrix::Vector3f reactionProportionalFeedback(const matrix::Vector3f &proportional,
		float tilt, float scale, const matrix::Vector3f &torque_scale)
{
	if (scale >= 1.f) { return proportional; }

	const matrix::Vector3f axis(sinf(tilt), 0.f, -cosf(tilt));
	const matrix::Vector3f torque = proportional.emult(torque_scale);
	return proportional - (axis * ((1.f - scale) * axis.dot(torque))).edivide(torque_scale);
}
}
