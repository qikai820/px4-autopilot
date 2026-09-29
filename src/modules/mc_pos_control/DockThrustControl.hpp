/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>
#include <uORB/topics/dock_guidance_thrust.h>

// Validate a fixed collective or post-pass braking request for the active Dock mode.
// Braking geometry is handled by the position controller using measured horizontal speed.
class DockThrustControl
{
public:
	struct Request {
		float thrust{NAN};
		float slew_rate{0.f};
		bool braking{false};
		float brake_rate{0.f};
		float brake_speed{0.f};
	};

	Request update(const dock_guidance_thrust_s &command, uint64_t now, uint8_t source_id,
		       bool flight_allowed, float measured_speed) const
	{
		// Never consume another mode's command or retain a stopped publisher.
		if (command.source_id == 0 || command.source_id != source_id || command.timestamp == 0
		    || now < command.timestamp || now - command.timestamp > 100000
		    || command.activation_timestamp == 0 || command.activation_timestamp > command.timestamp) {
			return {};
		}

		if (!std::isfinite(measured_speed) || measured_speed < 0.f) {
			return {};
		}

		if (!flight_allowed || !command.valid || !std::isfinite(command.thrust)
		    || command.thrust < 0.f || (!command.braking && command.thrust <= 0.f)
		    || command.thrust > 1.f || !std::isfinite(command.slew_rate)
		    || command.slew_rate < .01f || command.slew_rate > 2.f) {
			return {};
		}

		if (command.braking && (!std::isfinite(command.brake_rate) || command.brake_rate < .08f
				       || command.brake_rate > 3.15f || !std::isfinite(command.brake_speed)
				       || command.brake_speed < 2.f || command.brake_speed > 30.f)) {
			return {};
		}

		return {command.thrust, command.slew_rate, command.braking, command.brake_rate, command.brake_speed};
	}
};
