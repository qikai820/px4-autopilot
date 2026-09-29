// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cstdint>
#include <uORB/topics/dock_guidance_thrust.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// Recognize the Commander fault handoff from the registered Dock mode to RTL.
// Status and control-mode publications may arrive in different controller cycles.
class DockFaultTransfer {
public:
	bool update(uint64_t now, const vehicle_control_mode_s &mode, const vehicle_status_s &status,
		    const dock_guidance_thrust_s &dock, bool airborne)
	{
		// Dock clears activation_timestamp as soon as its mode is deselected. Cache
		// the last fresh identity so subscriber ordering cannot hide the handoff.
		if (mode.source_id != 0 && dock.source_id == mode.source_id
		    && dock.activation_timestamp != 0 && dock.timestamp != 0
		    && now >= dock.timestamp && now - dock.timestamp <= 100000) {
			_last_dock_source_id = dock.source_id;
			_last_dock_seen = dock.timestamp;
		}

		if (_previous_source_id != 0 && mode.source_id != _previous_source_id
		    && _last_dock_source_id == _previous_source_id && _last_dock_seen != 0
		    && now >= _last_dock_seen && now - _last_dock_seen <= 100000) {
			_pending_source_id = _previous_source_id;
			_pending_since = now;
		}
		_previous_source_id = mode.source_id;

		if (_pending_since != 0 && (now < _pending_since || now - _pending_since > 1000000)) {
			_pending_since = 0;
		}

		const bool rtl = mode.source_id == 0 && mode.flag_armed
			 && mode.flag_multicopter_position_control_enabled
			 && !mode.flag_control_termination_enabled && airborne
			 && status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			 && status.timestamp != 0 && now >= status.timestamp && now - status.timestamp <= 1000000;

		if (_pending_since != 0 && rtl && status.failsafe
		    && status.nav_state_user_intention == _pending_source_id) {
			_active_source_id = _pending_source_id;
			_active = true;
			_pending_since = 0;
			return true;
		}

		if (_active && (!rtl || status.nav_state_user_intention != _active_source_id)) {
			_active = false;
		}
		return false;
	}

	bool active() const { return _active; }
	void cancel() { _active = false; _pending_since = 0; }

private:
	uint64_t _pending_since{0};
	uint64_t _last_dock_seen{0};
	uint8_t _previous_source_id{0};
	uint8_t _last_dock_source_id{0};
	uint8_t _pending_source_id{0};
	uint8_t _active_source_id{0};
	bool _active{false};
};
