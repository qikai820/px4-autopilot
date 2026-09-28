// Standalone regression for Dock request validation, final scalar throttle,
// and the production position controller's integrator hold.
#include "DockThrustControl.hpp"
#include "../mc_rate_control/DockThrottleSlew.hpp"
#include "PositionControl/PositionControl.hpp"
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
	puts("Final collective throttle: constant scalar, bidirectional ramp, parameter change, limits and exit passed");
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

static Vector3f step(PositionControl &control, const Vector3f &velocity = {})
{
	auto input = PositionControl::empty_trajectory_setpoint;
	velocity.copyTo(input.velocity);
	input.yaw = .2f;
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

int main()
{
	test_request_guards();
	test_scalar_throttle();
	test_position_direction_and_integral();
}
