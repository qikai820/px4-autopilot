#pragma once

#include <lib/vector_control/VectorGeometry.hpp>
#include <cstdint>

namespace uorb_udp_bridge
{

/** Export angles in radians. Voliro uses Rz(heading)*Rx(bank)*Ry(pitch),
 * matching its controller and allowing independent pitch through +/-90.
 * Other airframes retain standard Euler angles. The source quaternion is
 * passed by value so its wire representation remains unchanged. */
inline matrix::Vector3f attitudeDisplayAngles(matrix::Quatf q, int32_t airframe)
{
	if (airframe != vector_control::voliro_t) { return matrix::Vector3f(matrix::Eulerf(q)); }

	if (!q.isAllFinite() || fabsf(q.norm_squared() - 1.f) > .01f) { return {NAN, NAN, NAN}; }

	q.normalize();
	const matrix::Dcmf rotation(q);
	const float heading = vector_control::forcePlaneHeading(q);
	const float bank = asinf(math::constrain(rotation(2, 1), -1.f, 1.f));
	// At a vertical body-Y axis the chart has no unique heading or pitch.
	// NAN is serialized as JSON null instead of a misleading zero angle.
	const float pitch = PX4_ISFINITE(heading) ? atan2f(-rotation(2, 0), rotation(2, 2)) : NAN;
	return {bank, pitch, heading};
}

} // namespace uorb_udp_bridge
