#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

namespace vector_control
{
// Independent pitch is atan2(-R20,R22), continuous through +/-90 deg.
// Preserve baseline bandwidth through 45 deg, smoothly schedule down by 75 deg,
// and restore it towards inverted level. This never limits integral authority.
inline float referenceFeedbackScale(const matrix::Quatf &q, float vertical_scale)
{
	const matrix::Dcmf rotation(q);
	const float projection = sqrtf(rotation(2, 0) * rotation(2, 0) + rotation(2, 2) * rotation(2, 2));
	const float pitch_cos = projection > 1e-4f ? fabsf(rotation(2, 2)) / projection : 0.f;
	const float blend = math::constrain((.70710678f - pitch_cos) / (.70710678f - .25881905f), 0.f, 1.f);
	return 1.f + (math::constrain(vertical_scale, .1f, 1.f) - 1.f) * blend * blend * (3.f - 2.f * blend);
}
}
