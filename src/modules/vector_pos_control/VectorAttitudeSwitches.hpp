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
}
