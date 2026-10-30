#pragma once
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace vector_control
{
constexpr int morphquad = 16;
constexpr int voliro_t = 17;

// Force is normalized by total installed thrust, in world NED. Body axes are FRD.
inline matrix::Quatf attitudeForForce(const matrix::Vector3f &force, float yaw, int airframe, float tilt_limit,
				     float pitch_setpoint = 0.f)
{
	float roll = 0.f;
	float pitch = 0.f;

	if (airframe == voliro_t) {
		pitch = math::constrain(pitch_setpoint, -tilt_limit, tilt_limit);

		if (force.norm() > 1e-5f) {
			// The common front tilt cannot produce body-Y force. Choose roll using
			// the requested pitch so the position force remains in the XZ plane.
			const float forward = cosf(yaw) * force(0) + sinf(yaw) * force(1);
			const float lateral = -sinf(yaw) * force(0) + cosf(yaw) * force(1);
			const float vertical = sinf(pitch) * forward + cosf(pitch) * force(2);
			const float max_roll = acosf(math::constrain(cosf(tilt_limit) / math::max(cosf(pitch), 1e-5f), 0.f, 1.f));
			roll = math::constrain(atan2f(lateral, math::max(-vertical, 1e-5f)), -max_roll, max_roll);
		}
	}

	return matrix::Quatf(matrix::Eulerf(roll, pitch, yaw));
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
