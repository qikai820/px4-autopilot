#pragma once
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace vector_control
{
constexpr int morphquad = 16;
constexpr int voliro_t = 17;

// Force is normalized by total installed thrust, in world NED. Body axes are FRD.
inline matrix::Quatf attitudeForForce(const matrix::Vector3f &force, float yaw, int airframe, float tilt_limit)
{
	float roll = 0.f;

	if (airframe == voliro_t &&force.norm() > 1e-5f) {
		const float lateral = -sinf(yaw) * force(0) + cosf(yaw) * force(1);
		roll = math::constrain(atan2f(lateral, math::max(-force(2), 1e-5f)), -tilt_limit, tilt_limit);
	}

	return matrix::Quatf(matrix::Eulerf(roll, 0.f, yaw));
}
inline matrix::Vector3f toBody(const matrix::Vector3f &force, const matrix::Quatf &q)
{
	return matrix::Dcmf(q).transpose() * force;
}
inline matrix::Vector3f torqueScale(float roll, float pitch, float yaw)
{
	return matrix::Vector3f(math::max(roll, .001f), math::max(pitch, .001f), math::max(yaw, .001f));
}
} // namespace vector_control
