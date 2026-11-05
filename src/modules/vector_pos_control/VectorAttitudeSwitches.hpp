#pragma once

#include <cmath>
#include <cstdint>

namespace vector_control
{
struct AttitudeSwitches {
	bool valid;
	bool enabled;
	bool level;
};

inline AttitudeSwitches attitudeSwitches(int enable_channel, int level_channel, bool fresh,
		const float *channels, unsigned count, unsigned capacity)
{
	auto valid_channel = [&](int channel) {
		return channel == 0 || (fresh && channel > 0 && static_cast<unsigned>(channel) <= count
					&& static_cast<unsigned>(channel) <= capacity && std::isfinite(channels[channel - 1]));
	};
	const bool valid = valid_channel(enable_channel) && valid_channel(level_channel);
	return {valid, valid && (enable_channel == 0 || channels[enable_channel - 1] > 0.f),
		valid && level_channel > 0 && channels[level_channel - 1] > 0.f};
}

inline float independentAttitudeTarget(bool valid, bool enabled, bool level, float demand)
{
	return valid && enabled && !level ? demand : 0.f;
}

// Switch off selects fixed-tilt MC control; LEVEL only overrides independent
// pitch in vector mode and must not re-enable the servo in MC mode.
inline bool voliroFixedTiltRequested(int channel, const AttitudeSwitches &switches,
		bool armed, bool position_control, bool failsafe, bool tilt_fault)
{
	return channel > 0 && switches.valid && !switches.enabled
	       && armed && position_control && !failsafe && !tilt_fault;
}

// Switch on (or unmapped) permits AUX1; switch off selects fixed-tilt MC mode.
// pitch_limit is the caller's bounded independent pitch limit, in radians.
inline float voliroIndependentPitchTarget(bool input_valid, const AttitudeSwitches &switches,
		int vector_channel, float aux1, float pitch_limit)
{
	if (!input_valid || !std::isfinite(aux1) || !std::isfinite(pitch_limit) || pitch_limit < 0.f) { return 0.f; }

	const float bounded_aux1 = aux1 < -1.f ? -1.f : (aux1 > 1.f ? 1.f : aux1);
	const float demand = bounded_aux1 * pitch_limit;
	return independentAttitudeTarget(switches.valid, vector_channel == 0 || switches.enabled, switches.level, demand);
}
}
