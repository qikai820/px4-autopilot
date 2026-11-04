#pragma once
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace vector_control
{
constexpr int morphquad = 16;
constexpr int voliro_t = 17;

/** Heading of Rz(heading)*Rx(bank)*Ry(pitch), defined through pitch +/-90
 * provided force-plane bank stays away from +/-90. */
inline float forcePlaneHeading(const matrix::Quatf &q)
{
	if (!q.isAllFinite() || fabsf(q.norm_squared() - 1.f) > .01f) { return NAN; }

	const matrix::Vector3f y = matrix::Dcmf(q).col(1);
	return y.xy().norm_squared() > .01f ? atan2f(-y(0), y(1)) : NAN;
}

// Align body Z with the requested force in the banked force plane.
// The vector allocator can still use rotor tilt to track force during transients.
inline float pitchForForce(const matrix::Vector3f &force, float heading, float tilt_limit)
{
	if (!force.isAllFinite() || !PX4_ISFINITE(heading) || !PX4_ISFINITE(tilt_limit) || force(2) >= -1e-5f) {
		return 0.f;
	}

	const float longitudinal = cosf(heading) * force(0) + sinf(heading) * force(1);
	const float lateral = -sinf(heading) * force(0) + cosf(heading) * force(1);
	const float limit = math::constrain(tilt_limit, 0.f, math::radians(80.f));
	const float bank = math::constrain(atan2f(lateral, -force(2)), -limit, limit);
	const float banked_vertical = -lateral * sinf(bank) + force(2) * cosf(bank);
	return math::constrain(atan2f(-longitudinal, -banked_vertical), -limit, limit);
}

// Force is normalized by total installed thrust, in world NED. Body axes are FRD.
inline matrix::Quatf attitudeForForce(const matrix::Vector3f &force, float yaw, int airframe, float tilt_limit,
				      float pitch_setpoint = 0.f)
{
	if (airframe == voliro_t) {
		// Bank the force plane before applying the independent body pitch.
		// Rz(yaw)*Ry(pitch)*Rx(roll) becomes singular when the projected
		// force crosses body Z=0, even at moderate pitch. Rz*Rx*Ry keeps
		// body Y perpendicular to the force independently of pitch, including
		// +/-90 degrees. The position tilt limit bounds bank; VT_PITCH_MAX
		// separately bounds independent pitch at the caller.
		float bank = 0.f;

		if (force.norm() > 1e-5f) {
			const float lateral = -sinf(yaw) * force(0) + cosf(yaw) * force(1);
			bank = math::constrain(atan2f(lateral, math::max(-force(2), 1e-5f)), -tilt_limit, tilt_limit);
		}

		return matrix::Quatf(matrix::Eulerf(0.f, 0.f, yaw))
		       * matrix::Quatf(matrix::Eulerf(bank, 0.f, 0.f))
		       * matrix::Quatf(matrix::Eulerf(0.f, pitch_setpoint, 0.f));
	}

	return matrix::Quatf(matrix::Eulerf(0.f, 0.f, yaw));
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
