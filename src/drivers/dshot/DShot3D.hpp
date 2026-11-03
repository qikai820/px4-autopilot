#pragma once

#include <cstdint>

namespace dshot3d
{
// Convert mixer-module units [0,1999] to the two 3D throttle ranges.
// The transport adds the DShot reserved-command offset to nonzero values.
inline uint16_t convertOutput(uint16_t output, float minimum_fraction, uint16_t dead_low, uint16_t dead_high)
{
	// Neutral must stop even when the configurable deadband is empty.
	if (output == 1000 || (output >= dead_low && output < dead_high)) {
		return 0;
	}

	const bool upper_range = output >= 1000;
	output = upper_range ? output - 1000 : 999 - output;
	const float minimum = 999.f * minimum_fraction;
	const float scaled = minimum + output * (999.f - minimum) / 999.f;
	output = static_cast<uint16_t>(scaled < 999.f ? scaled : 999.f);
	return upper_range ? output + 1000 : output;
}
} // namespace dshot3d
