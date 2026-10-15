// Standalone trajectory regression: g++ -std=c++17 -O2 -Wall -Wextra -pedantic
//   src/modules/mc_pos_control/test_dock_climb_brake.cpp -o /tmp/test_dock_climb_brake
#include "DockClimbBrake.hpp"
#include "../mc_rate_control/DockThrottleSlew.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
	DockClimbBrake brake;
	const DockClimbBrake::Limits limits{20.f, 5.f, 3.f, 3.f, 8.f, 10.f, .5f};
	auto output = brake.update(0.f, 0.f, 15.f, 0.f, 0, 0.f, .01f, false, limits);
	assert(output.active && !output.capture && output.allow_floor);
	assert(output.velocity_z < 0.f && output.velocity_z >= -.03001f);
	float previous_velocity = output.velocity_z;
	float previous_accel = output.velocity_z / .01f;
	for (int i = 0; i < 80; ++i) {
		output = brake.update(0.f, 0.f, 15.f, 0.f, 0, 0.f, .01f, false, limits);
		const float accel = (output.velocity_z - previous_velocity) / .01f;
		assert(fabsf(accel - previous_accel) <= .0801f);
		assert(output.velocity_z >= -limits.speed_up);
		previous_velocity = output.velocity_z;
		previous_accel = accel;
	}
	DockThrottleSlew throttle;
	const float pullup_floor = DockThrottleSlew::brakingFloor(output.allow_floor, 5.f, .5f, .7f);
	assert(throttle.update(.35f, .35f, .2f, .01f, .1f, .9f, pullup_floor) >= .7f);

	// A Dock-to-RTL handoff must keep the original height reference.
	for (int i = 0; i < 100; ++i) {
		output = brake.update(-.02f * i, -2.f, 15.f, .2f, 0, 0.f, .01f, false, limits);
	}
	assert(output.height_used > 1.9f && output.height_used < 2.1f);
	output = brake.update(3.f, -2.f, 15.f, .2f, 1, 5.f, .01f, false, limits);
	assert(output.active && output.height_used > 1.9f && output.height_used < 2.1f);

	// Closing on the relative height limit starts the vertical arrest and
	// releases the high collective floor while horizontal braking continues.
	output = brake.update(-15.f, -4.f, 12.f, .4f, 1, 5.f, .01f, false, limits);
	assert(output.capture && !output.allow_floor && !output.complete);
	assert(output.height_used > 19.9f && output.height_used < 20.1f);
	assert(DockThrottleSlew::brakingFloor(output.allow_floor, 12.f, .5f, .7f) == 0.f);
	assert(throttle.update(.4f, .4f, 0.f, .01f, .1f, .9f) == .4f);

	for (int i = 0; i < 60; ++i) {
		output = brake.update(-15.f, 0.f, 0.f, 0.f, 1, 5.f, .01f, false, limits);
	}
	assert(!output.complete); // low speed alone is insufficient without stable attitude
	for (int i = 0; i < 51; ++i) {
		output = brake.update(-15.f, 0.f, 0.f, 0.f, 1, 5.f, .01f, true, limits);
	}
	assert(output.complete && output.velocity_z == 0.f);

	brake.reset();
	output = brake.update(0.f, -3.f, 15.f, .1f, 0, 0.f, .01f, false, limits);
	assert(output.active);
	// A bad multi-count EKF reset invalidates the session. Fresh estimates do not
	// grant a second climb allowance until the original request has ended.
	output = brake.update(0.f, -3.f, 15.f, .1f, 2, 5.f, .01f, false, limits);
	assert(!output.active && brake.failed());
	output = brake.update(0.f, -3.f, 15.f, .1f, 2, 0.f, .01f, false, limits);
	assert(!output.active);
	brake.reset();
	assert(brake.update(0.f, 0.f, 15.f, 0.f, 2, 0.f, .01f, false, limits).active);

	brake.abort();
	assert(!brake.update(0.f, 0.f, 15.f, 0.f, 2, 0.f, .01f, false, limits).active);
	brake.reset();
	for (int i = 0; i < 1002; ++i) {
		output = brake.update(0.f, -1.f, 1.f, 0.f, 0, 0.f, .01f, false, limits);
	}
	assert(!output.active && brake.failed()); // capture cannot hold RTL indefinitely
	puts("Dock climb brake: budget, EKF reset, capture, stability and abort passed");
}
