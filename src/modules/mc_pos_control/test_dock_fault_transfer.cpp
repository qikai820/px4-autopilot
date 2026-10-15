// Standalone regression for the asynchronous Dock guidance-fault to RTL handoff.
#include "DockFaultTransfer.hpp"

#include <cassert>
#include <cstdio>

int main()
{
	DockFaultTransfer transfer;
	dock_guidance_thrust_s dock{};
	dock.timestamp = 1000000;
	dock.activation_timestamp = 900000;
	dock.source_id = 23;
	dock.valid = true;
	vehicle_control_mode_s mode{};
	mode.source_id = 23;
	mode.flag_armed = true;
	mode.flag_multicopter_position_control_enabled = true;
	vehicle_status_s status{};
	status.timestamp = dock.timestamp;
	status.nav_state_user_intention = 23;
	assert(!transfer.update(dock.timestamp, mode, status, dock, true, true));

	// Commander mode and Dock's invalidated request can arrive in either order.
	mode.source_id = 0;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	status.failsafe = true;
	status.timestamp += 10000;
	assert(!transfer.update(status.timestamp, mode, status, dock, true, true));
	assert(transfer.pending() && !transfer.active());
	dock.timestamp += 20000;
	dock.valid = false;
	dock.brake_fault_eligible = true;
	status.timestamp = dock.timestamp;
	assert(transfer.update(dock.timestamp, mode, status, dock, true, true));
	assert(transfer.active() && transfer.faultEligible());

	mode.flag_control_termination_enabled = true;
	assert(!transfer.update(dock.timestamp + 10000, mode, status, dock, true, true));
	assert(!transfer.active());

	DockFaultTransfer unrelated;
	mode.source_id = 23;
	mode.flag_control_termination_enabled = false;
	dock.timestamp = 2000000;
	dock.valid = true;
	dock.brake_fault_eligible = false;
	status.timestamp = dock.timestamp;
	assert(!unrelated.update(dock.timestamp, mode, status, dock, true, true));
	mode.source_id = 0;
	status.timestamp += 10000;
	assert(!unrelated.update(status.timestamp, mode, status, dock, true, true));
	assert(!unrelated.active());
	puts("Dock fault transfer: delayed eligible request and unrelated failsafe passed");
}
