#pragma once

namespace vector_control
{
constexpr int morphquad = 16;
constexpr int voliro_t = 17;

/** Directional capability, before finite thrust/travel/rate constraints.
 * A plane vehicle must bank to supply lateral force; a spatial vehicle can
 * independently choose bank and pitch. Allocation remains vehicle-specific. */
enum class ForceSpace { FixedAxis, LongitudinalPlane, Spatial };

struct Capabilities {
	ForceSpace force_space;
	unsigned tilt_axes_per_unit;
	bool shared_tilt;
	bool reversible_tail;

	constexpr bool vectorThrust() const { return force_space != ForceSpace::FixedAxis; }
	constexpr bool independentRoll() const { return force_space == ForceSpace::Spatial; }
	constexpr bool independentPitch() const { return vectorThrust(); }
	constexpr bool forcePlane() const { return force_space == ForceSpace::LongitudinalPlane; }
};

constexpr Capabilities capabilities(int airframe)
{
	return airframe == morphquad ? Capabilities{ForceSpace::Spatial, 2, false, false}
	       : airframe == voliro_t ? Capabilities{ForceSpace::LongitudinalPlane, 1, true, true}
	       : Capabilities{ForceSpace::FixedAxis, 0, false, false};
}

/** Startup-only switches. Keep the saved Voliro mode authoritative for its
 * constrained model, independently of the spatial-reference setting. */
constexpr bool referenceEnabled(int airframe, int voliro_mode, bool spatial_reference)
{
	return capabilities(airframe).forcePlane() ? voliro_mode == 1
	       : capabilities(airframe).independentRoll() && spatial_reference;
}
} // namespace vector_control
