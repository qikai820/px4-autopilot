// Standalone regression for Dock request validation, final scalar throttle,
// and the production position controller's integrator hold.
#include "DockThrustControl.hpp"
#include "DockFaultTransfer.hpp"
#include "DockFailsafeYaw.hpp"
#include "DockRtlVelocityRamp.hpp"
#include "../mc_rate_control/DockThrottleSlew.hpp"
#include "PositionControl/PositionControl.hpp"
#include "../dock_guidance/RecoveryHeading.hpp"
#include <geo/geo.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using matrix::Vector3f;

static void near(float actual, float expected, float tolerance = 2e-5f)
{
	assert(std::isfinite(actual) && fabsf(actual - expected) <= tolerance);
}

static dock_guidance_thrust_s command()
{
	dock_guidance_thrust_s result{};
	result.timestamp = 1000000;
	result.activation_timestamp = 900000;
	result.thrust = .8f;
	result.slew_rate = .2f;
	result.source_id = 23;
	result.valid = true;
	return result;
}

static void test_request_guards()
{
	DockThrustControl gate;
	auto cmd = command();
	auto disabled = [](const DockThrustControl::Request &request) {
		assert(std::isnan(request.thrust) && request.slew_rate == 0.f);
	};

	for (float speed : {0.f, 10.f, 50.f, 100.f, 0.f}) {
		near(gate.update(cmd, cmd.timestamp, 23, true, speed).thrust, .8f);
		cmd.timestamp += 5000000;
	}

	cmd = command();
	disabled(gate.update(cmd, cmd.timestamp, 24, true, 0.f));
	disabled(gate.update(cmd, cmd.timestamp, 23, false, 0.f));
	disabled(gate.update(cmd, cmd.timestamp, 23, true, NAN));
	disabled(gate.update(cmd, cmd.timestamp, 23, true, INFINITY));
	disabled(gate.update(cmd, cmd.timestamp - 1, 23, true, 0.f));
	disabled(gate.update(cmd, cmd.timestamp + 100001, 23, true, 0.f));
	cmd.valid = false;
	disabled(gate.update(cmd, cmd.timestamp, 23, true, 0.f));
	cmd.valid = true;
	cmd.activation_timestamp = 0;
	disabled(gate.update(cmd, cmd.timestamp, 23, true, 0.f));
	cmd.activation_timestamp = 900000;
	for (float bad : {0.f, -1.f, 1.01f, NAN, INFINITY}) {
		cmd.thrust = bad;
		disabled(gate.update(cmd, cmd.timestamp, 23, true, 0.f));
	}
	cmd = command();
	for (float bad : {0.f, -.1f, 2.01f, NAN, INFINITY}) {
		cmd.slew_rate = bad;
		disabled(gate.update(cmd, cmd.timestamp, 23, true, 0.f));
	}
	puts("Dock request: full-stage throttle and mode, freshness, flight gates passed");

	cmd = command();
	cmd.braking = true;
	cmd.thrust = 0.f; // Braking can use hover thrust without fixed Dock throttle.
	cmd.brake_rate = math::radians(45.f);
	cmd.brake_speed = 10.f;
	assert(gate.update(cmd, cmd.timestamp, 23, true, 0.f).braking);
	disabled(gate.update(cmd, cmd.timestamp + 100001, 23, true, 20.f));
	disabled(gate.update(cmd, cmd.timestamp, 24, true, 20.f));
	disabled(gate.update(cmd, cmd.timestamp, 23, false, 20.f));
	for (float bad : {0.f, -1.f, 3.2f, NAN, INFINITY}) {
		cmd.brake_rate = bad;
		disabled(gate.update(cmd, cmd.timestamp, 23, true, 20.f));
	}
	cmd.brake_rate = math::radians(45.f);
	for (float bad : {0.f, 1.9f, 31.f, NAN, INFINITY}) {
		cmd.brake_speed = bad;
		disabled(gate.update(cmd, cmd.timestamp, 23, true, 20.f));
	}
	puts("Braking request: mode, freshness, flight and configuration gates passed");
}

static void test_scalar_throttle()
{
	DockThrottleSlew throttle;
	float output = .35f;
	near(throttle.update(output, NAN, 0.f, .01f, .05f, 1.f), .35f);

	for (int i = 1; i <= 250; ++i) {
		output = throttle.update(i % 2 ? .1f : .95f, .8f, .2f, .01f, .05f, 1.f);
		near(output, fminf(.35f + i * .002f, .8f));
	}

	for (float automatic : {.1f, .4f, .9f, .2f, 1.f}) {
		near(throttle.update(automatic, .8f, .2f, .01f, .05f, 1.f), .8f);
	}

	for (int i = 1; i <= 150; ++i) {
		near(throttle.update(.1f, .5f, .2f, .01f, .05f, 1.f), fmaxf(.8f - i * .002f, .5f));
	}

	near(throttle.update(.4f, .9f, .2f, .01f, .05f, .45f), .45f); // MPC hard limit wins.
	near(throttle.update(.37f, NAN, 0.f, .01f, .05f, 1.f), .37f); // Exit releases immediately.
	near(throttle.update(.38f, .8f, .2f, .01f, .05f, 1.f), .372f); // Reentry starts from latest output.
	throttle.reset(); // Disabled rate control must not retain a previous Dock output.
	near(throttle.update(.25f, .8f, .2f, .01f, .05f, 1.f), .252f);
	throttle.reset();
	near(throttle.update(1.f, NAN, 0.f, .01f, .05f, 1.f), 1.f);
	near(throttle.update(.35f, .35f, .2f, .01f, .05f, 1.f), .998f);
	// Fault RTL must not reproduce the log's nearly instantaneous 1.0 to 0.35 drop.
	puts("Final collective throttle: constant scalar, bidirectional ramp, parameter change, limits and exit passed");
}

static void test_braking_collective_floor()
{
	near(DockThrottleSlew::brakingFloor(true, 35.f, 8.f, .7f), .7f);
	near(DockThrottleSlew::brakingFloor(true, 8.f, 8.f, .7f), .7f);
	near(DockThrottleSlew::brakingFloor(true, 7.999f, 8.f, .7f), 0.f);
	near(DockThrottleSlew::brakingFloor(false, 35.f, 8.f, .7f), 0.f);
	near(DockThrottleSlew::brakingFloor(true, NAN, 8.f, .7f), 0.f);

	DockThrottleSlew throttle;
	near(throttle.update(.35f, NAN, 0.f, .01f, .1f, 1.f), .35f);
	// The first recovery cycle must not spend 1.75 s ramping from .35 to .70.
	near(throttle.update(.35f, .35f, .2f, .01f, .1f, 1.f, .7f), .7f);
	near(throttle.update(.35f, .35f, .2f, .01f, .1f, 1.f, .7f), .7f);
	near(throttle.update(.35f, .35f, .2f, .01f, .1f, 1.f), .698f);
	near(throttle.update(.35f, .35f, .2f, .01f, .1f, .65f, .7f), .65f);
	near(throttle.update(.35f, NAN, 0.f, .01f, .1f, 1.f, .7f), .35f);
	puts("Braking collective floor: immediate 70%, strict 8 m/s gate, smooth release, MPC cap and safety exit passed");
}

static PositionControl make_controller(float integral_gain = 0.f)
{
	PositionControl control;
	control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	control.setVelocityGains(Vector3f(1.f, 1.f, 1.f), Vector3f(integral_gain, integral_gain, integral_gain), Vector3f{});
	control.setVelocityLimits(20.f, 20.f, 20.f);
	control.setThrustLimits(.1f, .9f);
	control.setHorizontalThrustMargin(.3f);
	control.setTiltLimit(1.f);
	control.setHoverThrust(.5f);
	control.setState({Vector3f{}, Vector3f{}, Vector3f{}, 0.f});
	return control;
}

static Vector3f step(PositionControl &control, const Vector3f &velocity = {}, float yaw = .2f)
{
	auto input = PositionControl::empty_trajectory_setpoint;
	velocity.copyTo(input.velocity);
	input.yaw = yaw;
	control.setInputSetpoint(input);
	assert(control.update(.01f));
	vehicle_local_position_setpoint_s output{};
	control.getLocalPositionSetpoint(output);
	vehicle_attitude_setpoint_s attitude{};
	control.getAttitudeSetpoint(attitude);
	near(-attitude.thrust_body[2], Vector3f(output.thrust).norm());
	return Vector3f(output.thrust);
}

static void test_position_direction_and_integral()
{
	auto control = make_controller(1.f);
	DockThrottleSlew throttle;
	near(step(control).norm(), .5f);
	control.setVelocityIntegralHold(true);
	Vector3f first_direction{};

	for (int i = 0; i < 3000; ++i) {
		const Vector3f wanted = i % 2 ? Vector3f(2.f, 1.f, 0.f) : Vector3f(-1.f, 2.f, 0.f);
		const Vector3f vector = step(control, wanted);
		if (i == 0) { first_direction = vector.normalized(); }
		const float scalar = throttle.update(vector.norm(), .8f, .2f, .01f, .05f, 1.f);
		if (i >= 150) { near(scalar, .8f); }
		if (i == 1) { assert(Vector3f(vector.normalized() - first_direction).norm() > .1f); }
	}

	control.setVelocityIntegralHold(false);
	const Vector3f settled = step(control);
	near(settled(0), 0.f);
	near(settled(1), 0.f);
	near(settled(2), -.5f); // No velocity-integral windup under fixed final throttle.
	near(throttle.update(settled.norm(), NAN, 0.f, .01f, .05f, 1.f), .5f);
	puts("Position controller: attitude direction remains free and velocity integral does not wind up");
}

static void test_post_pass_braking()
{
	for (const Vector3f &velocity : {Vector3f(20.f, 0.f, -15.f), Vector3f(-20.f, 0.f, -15.f),
					Vector3f(0.f, 20.f, -15.f), Vector3f(12.f, -16.f, -15.f)}) {
		auto control = make_controller(.3f);
		control.setHoverThrust(.35f);
		control.setTiltLimit(math::radians(60.f));
		// Establish a forward-tilted command before entering recovery.
		Vector3f previous_z = -step(control, Vector3f(15.f, 0.f, 0.f)).normalized();
		control.resetIntegral();
		control.setState({{}, velocity, {}, .2f});
		control.setDockBraking(true, .75f, math::radians(45.f), 10.f);
		dock_motion::RecoveryHeading heading;
		Vector3f thrust;

		for (int i = 0; i < 400; ++i) {
			const float held_yaw = heading.update(velocity, .2f, 1.7f, 2.f, math::radians(10.f), .01f, 0, 0.f);
			thrust = step(control, {}, held_yaw);
			const Vector3f body_z = -thrust.normalized();
			const float rotation = acosf(math::constrain(previous_z.dot(body_z), -1.f, 1.f));
			assert(rotation <= math::radians(45.f) * .01f + .0001f);
			assert(body_z(2) >= cosf(math::radians(60.f)) - 1e-5f);
			assert(thrust.norm() >= .75f - 1e-5f && thrust.norm() <= .9f + 1e-5f);
			previous_z = body_z;
		}

		assert(matrix::Vector2f(thrust).dot(matrix::Vector2f(velocity)) < 0.f);
		near(-matrix::Vector2f(thrust).normalized().dot(matrix::Vector2f(velocity).normalized()), 1.f);
		vehicle_attitude_setpoint_s attitude{};
		control.getAttitudeSetpoint(attitude);
		near(matrix::Eulerf(matrix::Quatf(attitude.q_d)).psi(), .2f);
		near(attitude.yaw_sp_move_rate, 0.f);
		if (velocity(0) > 19.f) {
			assert(matrix::Eulerf(matrix::Quatf(attitude.q_d)).theta() > 0.f);
		}

		control.setThrustLimits(.1f, .65f);
		near(step(control).norm(), .65f); // Hard thrust limit takes priority over the floor.
		control.setState({{}, {}, {}, .2f});
		for (int i = 0; i < 400; ++i) { thrust = step(control); }
		near(thrust(0), 0.f);
		near(thrust(1), 0.f);
		near(thrust(2), -.35f); // Low-speed return and no integral accumulated during braking.
		control.setDockBraking(false);
		near(step(control).norm(), .35f);
	}

	puts("Post-pass braking: opposite-motion thrust, nose-up pitch, yaw retention, tilt slew, floor, limits and hover return passed");
}

static void test_braking_after_saturated_approach()
{
	// A long 50 m/s request leaves a forward velocity integral. It must be
	// cleared on brake entry or it will drive the aircraft forward after stopping.
	auto control = make_controller();
	control.setVelocityGains(Vector3f(1.8f, 1.8f, 4.f), Vector3f(.4f, .4f, 2.f),
				 Vector3f(.2f, .2f, 0.f));
	control.setHoverThrust(.35f);
	control.setThrustLimits(.05f, 1.f);
	control.setHorizontalThrustMargin(.8f);
	control.setVelocityLimits(60.f, 20.f, 20.f);
	control.setTiltLimit(math::radians(89.f));
	control.setState({{}, Vector3f(42.f, 0.f, 0.f), {}, 0.f});
	for (int i = 0; i < 3000; ++i) { step(control, Vector3f(50.f, 0.f, 0.f), 0.f); }
	control.setDockBraking(true, .35f, math::radians(45.f), 10.f);
	control.setState({{}, Vector3f{}, {}, 0.f});
	Vector3f thrust{};
	for (int i = 0; i < 300; ++i) { thrust = step(control, {}, 0.f); }
	near(thrust(0), 0.f, .001f);
	puts("Dock braking: inherited approach integral clears at recovery entry");
}

static void test_braking_direction_with_forward_bias()
{
	// At low speed the normal velocity controller can still point forward after
	// a saturated approach. Blending must never turn that into a forward brake.
	auto control = make_controller();
	control.setHoverThrust(.35f);
	control.setTiltLimit(math::radians(89.f));
	control.setState({{}, Vector3f(4.7f, 0.f, -.8f), {}, 0.f});
	control.setDockBraking(true, .35f, math::radians(45.f), 10.f);
	auto input = PositionControl::empty_trajectory_setpoint;
	Vector3f{}.copyTo(input.velocity);
	input.acceleration[0] = 20.f;
	input.acceleration[1] = 0.f;
	input.yaw = 0.f;
	Vector3f thrust{};
	for (int i = 0; i < 300; ++i) {
		control.setInputSetpoint(input);
		assert(control.update(.01f));
		vehicle_local_position_setpoint_s output{};
		control.getLocalPositionSetpoint(output);
		thrust = Vector3f(output.thrust);
	}
	assert(thrust(0) <= 0.f);
	puts("Dock braking: forward-biased automatic control cannot propel the aircraft");
}

static void test_recovery_heading_gate()
{
	dock_motion::RecoveryHeading heading;
	const float rate = math::radians(10.f), dt = .01f, yaw_step = rate * dt;
	// The old target is different from measured heading: high speed must hold the latter.
	near(heading.update(Vector3f(30.f, 0.f, 0.f), .3f, 1.5f, 2.f, rate, dt, 0, 0.f), .3f);
	for (const Vector3f &velocity : {Vector3f(0.f, 20.f, 0.f), Vector3f(-5.f, 0.f, 0.f),
			Vector3f(2.f, 0.f, 0.f), Vector3f(0.f, 0.f, -3.f)}) {
		// A changing measured heading must not move an already captured heading lock.
		near(heading.update(velocity, .4f, -2.f, 2.f, rate, dt, 0, 0.f), .3f);
	}
	near(heading.update(Vector3f(1.999f, 0.f, 0.f), .3f, -2.f, 2.f, rate, dt, 0, 0.f), .3f + yaw_step);
	// Crossing back to the exact threshold immediately captures the current measured heading.
	near(heading.update(Vector3f(2.f, 0.f, 0.f), .31f, -2.f, 2.f, rate, dt, 0, 0.f), .31f);
	near(heading.update(Vector3f(1.f, 0.f, 0.f), .31f, -2.f, 2.f, rate, dt, 0, 0.f), .31f + yaw_step);
	float output = 0.f;
	for (int i = 0; i < 1000; ++i) {
		output = heading.update({}, .31f, -2.f, 2.f, rate, dt, 0, 0.f);
	}
	near(output, 1.5f); // Resume the original pending heading; no automatic turn-around target.

	heading.reset();
	near(heading.update(Vector3f(3.f, 0.f, 0.f), .3f, 1.5f, 4.f, rate, dt, 0, 0.f), .3f + yaw_step);
	near(heading.update(Vector3f(3.f, 0.f, 0.f), .32f, 1.5f, 2.f, rate, dt, 0, 0.f), .32f);
	// Estimator frame changes must preserve the same physical heading and pending target.
	near(heading.update(Vector3f(3.f, 0.f, 0.f), .52f, 1.5f, 2.f, rate, dt, 1, .2f), .52f);
	near(heading.update(Vector3f(3.f, 0.f, 0.f), .52f, 1.5f, 2.f, rate, dt, 1, .2f), .52f);
	for (int i = 0; i < 1000; ++i) {
		output = heading.update({}, .52f, 0.f, 2.f, rate, dt, 1, .2f);
	}
	near(output, 1.7f);
	assert(std::isnan(heading.update({}, .52f, 0.f, 2.f, rate, dt, 3, .2f)));

	heading.reset();
	const float from = math::radians(179.f), to = math::radians(-179.f);
	near(heading.update({}, from, to, 2.f, rate, dt, 255, 0.f), from + yaw_step);
	near(heading.update(Vector3f(3.f, 0.f, 0.f), from, to, 2.f, rate, dt, 0, 0.f), from);
	assert(std::isnan(heading.update(Vector3f(NAN, 0.f, 0.f), 0.f, 0.f, 2.f, rate, dt, 0, 0.f)));
	assert(std::isnan(heading.update({}, NAN, 0.f, 2.f, rate, dt, 0, 0.f)));
	assert(std::isnan(heading.update({}, 0.f, 0.f, NAN, rate, dt, 0, 0.f)));
	puts("Recovery heading: measured-heading lock, strict 2 m/s 3-D gate, re-lock, configurable threshold, yaw slew, wrap and EKF resets passed");
}

static void test_fault_rtl_transfer()
{
	DockFaultTransfer transfer;
	auto dock = command();
	dock.valid = false; // Fault withdrawal invalidates Dock's command before RTL takes ownership.
	vehicle_control_mode_s mode{};
	mode.source_id = 23;
	mode.flag_armed = true;
	mode.flag_multicopter_position_control_enabled = true;
	vehicle_status_s status{};
	status.timestamp = dock.timestamp;
	status.nav_state_user_intention = 23;
	assert(!transfer.update(dock.timestamp, mode, status, dock, true));
	mode.source_id = 0;
	dock.activation_timestamp = 0; // The logged Dock publisher clears this on mode deselection.
	dock.timestamp += 10000;
	assert(!transfer.update(dock.timestamp + 10000, mode, status, dock, true));
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	status.failsafe = true;
	status.timestamp = dock.timestamp + 30000;
	assert(transfer.update(dock.timestamp + 30000, mode, status, dock, true));
	assert(transfer.active());
	assert(!transfer.update(dock.timestamp + 40000, mode, status, dock, true));
	mode.flag_control_termination_enabled = true;
	assert(!transfer.update(dock.timestamp + 50000, mode, status, dock, true));
	assert(!transfer.active());

	// A manual RTL switch and a stale Dock message must not arm fault braking.
	for (bool stale : {false, true}) {
		DockFaultTransfer rejected;
		auto request = command();
		request.valid = false;
		auto next = mode;
		next.source_id = 23;
		next.flag_control_termination_enabled = false;
		status.nav_state = 23;
		assert(!rejected.update(request.timestamp, next, status, request, true));
		next.source_id = 0;
		const uint64_t next_time = request.timestamp + (stale ? 210000 : 10000);
		request.activation_timestamp = 0;
		status.nav_state_user_intention = stale ? 23 : 0;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		status.timestamp = next_time;
		assert(!rejected.update(next_time, next, status, request, true));
		assert(!rejected.active());
	}
	puts("Fault RTL transfer: asynchronous Commander handoff, invalidated Dock command, and safety exits passed");
}

static void test_fault_rtl_yaw()
{
	DockFailsafeYaw yaw;
	const float heading = math::radians(-1.9f);
	const float rate = math::radians(10.f);
	yaw.latch(heading, 0);
	for (float speed : {35.f, 28.f, 22.f, 2.f}) {
		const auto held = yaw.update(true, speed, heading + .1f, math::radians(90.f),
						   2.f, rate, .01f, 0, 0.f);
		assert(held.active && held.hold_translation && held.braking && !held.release_to_rtl);
		near(held.yaw, heading);
	}
	const auto turning = yaw.update(true, 1.99f, heading, math::radians(90.f),
					    2.f, rate, .01f, 0, 0.f);
	assert(turning.active && turning.hold_translation && !turning.braking);
	near(turning.yaw, heading + rate * .01f);
	const auto drift = yaw.update(true, 3.f, .2f, math::radians(90.f), 2.f, rate, .01f, 0, 0.f);
	assert(drift.active && drift.hold_translation && drift.braking);
	near(drift.yaw, .2f);
	const auto reset = yaw.update(true, 3.f, .4f, math::radians(90.f), 2.f, rate, .01f, 1, .2f);
	near(reset.yaw, .4f);
	// The RTL target can sweep past the held heading while the flight task turns.
	// Translation remains stopped until both the target settles and the aircraft follows it.
	float measured = .4f;
	for (int i = 0; i < 40; ++i) {
		const auto moving_target = yaw.update(true, .5f, measured, .4f + (i + 1) * math::radians(1.f),
							2.f, rate, .01f, 1, 0.f);
		assert(moving_target.active && moving_target.hold_translation && !moving_target.release_to_rtl);
		measured = moving_target.yaw;
	}
	bool released = false;
	for (int i = 0; i < 1000; ++i) {
		const auto aligned = yaw.update(true, .5f, measured, math::radians(90.f),
							2.f, rate, .01f, 1, 0.f);
		if (aligned.release_to_rtl) {
			assert(!aligned.active && !aligned.hold_translation && !aligned.braking && aligned.aligned);
			released = true;
			break;
		}
		assert(aligned.active && aligned.hold_translation && !aligned.braking);
		measured = aligned.yaw;
	}
	assert(released);
	assert(!yaw.update(true, 35.f, math::radians(90.f), math::radians(90.f),
					 2.f, rate, .01f, 1, 0.f).active);
	yaw.latch(.4f, 1);
	assert(!yaw.update(false, 3.f, .4f, 0.f, 2.f, rate, .01f, 1, 0.f).active);
	yaw.latch(0.f, 1);
	const auto no_target = yaw.update(true, .5f, 0.f, NAN, 2.f, rate, .01f, 1, 0.f);
	assert(no_target.release_to_rtl && !no_target.aligned);
	// A failed turn must eventually hand authority back to native RTL.
	yaw.latch(0.f, 1);
	bool timed_out = false;
	for (int i = 0; i < 900; ++i) {
		const auto command = yaw.update(true, .5f, 0.f, math::radians(90.f),
							2.f, math::radians(60.f), .01f, 1, 0.f);
		if (command.release_to_rtl) {
			assert(!command.aligned && !command.active && i >= 799);
			timed_out = true;
			break;
		}
		assert(command.active && command.hold_translation);
	}
	assert(timed_out);
	puts("Fault RTL yaw: brake hold, settle and measure return alignment before translation passed");
}

static void test_fault_rtl_velocity_ramp()
{
	DockRtlVelocityRamp ramp;
	ramp.start();
	const Vector3f target(-60.f, 0.f, 0.f);
	Vector3f previous{};
	for (int i = 0; i < 800 && ramp.active(); ++i) {
		const Vector3f output = ramp.update(target, 8.f, .01f);
		assert((output - previous).norm() <= .08001f);
		previous = output;
	}
	assert(!ramp.active());
	near(previous(0), -60.f);
	ramp.start();
	near(ramp.update(Vector3f{}, 8.f, .01f).norm(), 0.f);
	assert(!ramp.active());
	puts("Fault RTL velocity: bounded transition from yaw hold to planned return passed");
}

static void test_braking_point_mass()
{
	// Production controller + final throttle slew in an ideal thrust model with 150 ms attitude lag.
	// This checks transitions and stopping, not airframe aerodynamics or real motor authority.
	for (const matrix::Vector2f &scenario : {matrix::Vector2f(5.f, .35f), matrix::Vector2f(20.f, .35f),
			matrix::Vector2f(35.f, .35f), matrix::Vector2f(5.f, .7f), matrix::Vector2f(20.f, .7f), matrix::Vector2f(35.f, .7f)}) {
		const float initial_speed = scenario(0);
		const float initial_throttle = scenario(1);
		auto control = make_controller(.2f);
		control.setHoverThrust(.35f);
		control.setTiltLimit(math::radians(60.f));
		control.setVelocityGains(Vector3f(1.f, 1.f, 1.f), Vector3f(.2f, .2f, .2f), Vector3f(.2f, .2f, .2f));
		Vector3f body_z = -step(control, Vector3f(7.f, 0.f, 0.f)).normalized();
		control.setDockBraking(true, .35f, math::radians(45.f), 10.f);
		DockThrottleSlew throttle;
		float collective = throttle.update(initial_throttle, NAN, 0.f, .01f, .1f, .9f);
		Vector3f position{}, velocity(initial_speed, 0.f, -3.f), acceleration{};
		float stopped = NAN, min_forward_speed = initial_speed, max_height = 0.f;

		for (int i = 0; i < 3000; ++i) {
			control.setState({position, velocity, acceleration, .2f});
			const Vector3f desired_thrust = step(control);
			const float previous_collective = collective;
			const float floor = DockThrottleSlew::brakingFloor(true, velocity.xy().norm(), 8.f, .7f);
			collective = throttle.update(desired_thrust.norm(), desired_thrust.norm(), .2f, .01f, .1f, .9f, floor);
			assert(fabsf(collective - previous_collective) <= .00201f || (floor > 0.f && previous_collective < floor));
			body_z = (body_z + (-desired_thrust.normalized() - body_z) * (.01f / .15f)).normalized();
			acceleration = -body_z * (collective * CONSTANTS_ONE_G / .35f) + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
			velocity += acceleration * .01f;
			position += velocity * .01f;
			min_forward_speed = fminf(min_forward_speed, velocity(0));
			max_height = fmaxf(max_height, -position(2));
			if (!std::isfinite(stopped) && velocity.xy().norm() < .5f) { stopped = i * .01f; }
		}

		printf("Ideal braking model %.0f m/s, entry throttle %.2f: <0.5 m/s at %.2f s, minimum forward %.3f m/s, climb %.2f m, final speed %.3f m/s\n",
		       double(initial_speed), double(initial_throttle), double(stopped), double(min_forward_speed), double(max_height), double(velocity.norm()));
		assert(std::isfinite(stopped) && stopped < 20.f);
		assert(min_forward_speed > -1.f);
		assert(velocity.norm() < .5f);
	}
}

int main()
{
	test_request_guards();
	test_scalar_throttle();
	test_braking_collective_floor();
	test_position_direction_and_integral();
	test_post_pass_braking();
	test_braking_after_saturated_approach();
	test_braking_direction_with_forward_bias();
	test_recovery_heading_gate();
	test_fault_rtl_transfer();
	test_fault_rtl_yaw();
	test_fault_rtl_velocity_ramp();
	test_braking_point_mass();
}
