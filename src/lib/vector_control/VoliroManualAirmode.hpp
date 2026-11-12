#pragma once

#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace vector_control
{
inline bool voliroManualAirmode(const vehicle_control_mode_s &mode, uint64_t now, int setting)
{
	return (setting == 1 || setting == 2) && mode.timestamp > 0 && mode.timestamp <= now
	       && now - mode.timestamp <= 1000000 && mode.flag_armed && mode.flag_control_manual_enabled
	       && mode.flag_control_rates_enabled && !mode.flag_control_auto_enabled
	       && !mode.flag_control_offboard_enabled && !mode.flag_control_position_enabled
	       && !mode.flag_control_velocity_enabled && !mode.flag_control_altitude_enabled
	       && !mode.flag_control_climb_rate_enabled && !mode.flag_control_acceleration_enabled
	       && !mode.flag_multicopter_position_control_enabled;
}

// Low manual collective needs actuator lift compensation before a pitch moment
// is available. Slow integral accumulation in that regime; keep its state and
// full VC_PR_INT_LIM authority, and restore the configured I rate by hover.
inline float voliroAirmodeIntegralScale(float collective, float hover)
{
	if (!PX4_ISFINITE(collective) || !PX4_ISFINITE(hover) || hover < .01f) { return 1.f; }

	return math::constrain(collective / hover, .25f, 1.f);
}
// The pilot curve is anchored at zero; the airborne idle is a lower clamp,
// not a new lower interpolation endpoint that moves every fixed stick value.
inline float voliroManualThrust(float stick, int curve, float hover, float minimum, float maximum)
{
	const float raw = curve == 1 ? math::interpolate(stick, -1.f, 1.f, 0.f, maximum)
			  : math::interpolateNXY(stick, {-1.f, 0.f, 1.f}, {0.f, hover, maximum});
	return math::constrain(math::max(raw, minimum), 0.f, maximum);
}

// Bounded attitude assistance: no lift increase while on/near the ground.
// Even airborne it may add at most 2% total thrust and 25% of the pilot request.
inline float voliroAirmodeForceBudget(float requested_force, float total_max, bool airborne)
{
	return airborne ? math::min(.02f * total_max, .25f * math::max(requested_force, 0.f)) : 0.f;
}

}
