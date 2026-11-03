#include "VoliroTAllocation.hpp"
#include "VoliroTTailMotor.hpp"
#include <drivers/dshot/DShot3D.hpp>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace matrix;

static void check(bool condition, const char *message)
{
	if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(10); }
}

static void test_tail_protection()
{
	using Motor = voliro_t::TailMotor;
	const Motor::Settings settings{.4f, .03f, .10f, .04f, .08f};
	constexpr float dt = .01f;
	Motor motor;
	check(motor.update(.08f, dt, settings) == 0.f, "start threshold rejects zero-centred noise");
	float previous = 0.f;

	for (int i = 0; i < 30; ++i) {
		float output = motor.update(.6f, dt, settings);
		check(fabsf(output - previous) <= 2.f * dt / settings.slew_time + 1e-6f, "startup slew bound");
		previous = output;
	}

	check(fabsf(previous - .6f) < 1e-6f, "startup reaches demand");
	float first_zero = -1.f;
	bool reversed = false;

	for (int i = 0; i < 60; ++i) {
		float output = motor.update(-.6f, dt, settings);
		float time = (i + 1) * dt;
		check(fabsf(output - previous) <= 2.f * dt / settings.slew_time + 1e-6f, "reversal slew bound");
		check(!(previous > 0.f && output < 0.f), "no direct sign crossing");

		if (output == 0.f && first_zero < 0.f) { first_zero = time; }

		if (output < 0.f && !reversed) {
			check(first_zero >= 0.f && time - first_zero >= settings.zero_time - 1e-6f, "full dwell after reaching zero");
			reversed = true;
		}

		previous = output;
	}

	check(reversed && fabsf(previous + .6f) < 1e-6f, "sustained demand reverses after dwell");

// Running below the start threshold remains possible in the same direction.
	for (int i = 0; i < 30; ++i) { previous = motor.update(-.06f, dt, settings); }

	check(fabsf(previous + .06f) < 1e-6f, "Schmitt trigger holds running direction");

	for (int i = 0; i < 30; ++i) { previous = motor.update(-.02f, dt, settings); }

	check(previous == 0.f, "stop deadband ramps to exact zero");
	check(motor.update(-.06f, dt, settings) == 0.f, "restart requires high threshold");

	for (int i = 0; i < 200; ++i) {
		check(motor.update(i % 2 ? .08f : -.08f, dt, settings) == 0.f, "noise cannot restart or reverse");
	}

	Motor::Settings instant = settings;
	instant.slew_time = 0.f;
	motor.reset();
	check(motor.update(.5f, dt, instant) == .5f, "initial start without artificial reversal delay");

	for (int i = 0; i < 20; ++i) {
		check(motor.update(-.5f, dt, instant) == 0.f, "brief opposite request brakes");
		check(motor.update(-.5f, dt, instant) == 0.f, "brief opposite request cannot reverse");
		check(motor.update(.5f, dt, instant) == .5f, "cancelled reversal returns to old direction");
	}

// Confirmation must still be continuous after a long stopped interval.
	for (int i = 0; i < 30; ++i) { check(motor.update(0.f, dt, instant) == 0.f, "stop before confirmation test"); }

	for (int i = 0; i < 3; ++i) { check(motor.update(-.5f, dt, instant) == 0.f, "confirmation time enforced"); }

	check(motor.update(.05f, dt, instant) == 0.f, "subthreshold request cancels confirmation");

	for (int i = 0; i < 3; ++i) { check(motor.update(-.5f, dt, instant) == 0.f, "confirmation restarts after cancellation"); }

	for (int i = 0; i < 3; ++i) { previous = motor.update(-.5f, dt, instant); }

	check(previous < 0.f, "confirmed reversal after long zero interval");
	check(motor.update(std::numeric_limits<float>::quiet_NaN(), dt, settings) == 0.f, "nonfinite demand stops");
	check(motor.update(.5f, 0.f, settings) == 0.f, "invalid timestep stops");
	motor.reset();
	check(motor.update(.5f, dt, settings) > 0.f, "disarm reset clears prior direction");

// Variable scheduler intervals and repeated reversals: command bounds,
// slew bounds and a nonzero dwell must hold throughout a long sequence.
	motor.reset(); previous = 0.f;
	float time = 0.f, zero_since = 0.f;
	int last_direction = 0;

	for (int i = 0; i < 6000; ++i) {
		const float step = .001f + static_cast<float>(i % 13) * .001f;
		const float demand = (i / 173) % 2 ? -.9f : .9f;
		float output = motor.update(demand, step, settings);
		time += step;
		check(fabsf(output) <= 1.f, "variable-dt command bounds");
		check(fabsf(output - previous) <= 2.f * step / settings.slew_time + 1e-6f, "variable-dt slew bound");

		if (output == 0.f && previous != 0.f) { zero_since = time; }

		if (output != 0.f) {
			int direction = output > 0.f ? 1 : -1;

			if (last_direction && direction != last_direction) {
				check(previous == 0.f && time - zero_since >= settings.zero_time - 1e-4f, "variable-dt reversal dwell");
			}

			last_direction = direction;
		}

		previous = output;
	}

// The actual driver conversion must turn an exact neutral command into
// stop, even with DSHOT_MIN>0 and an empty default deadband.
	check(dshot3d::convertOutput(1000, .055f, 1000, 1000) == 0, "DShot exact neutral is stop");
	check(dshot3d::convertOutput(1000, 0.f, 1000, 1000) == 0, "DShot zero-min neutral is stop");
	check(dshot3d::convertOutput(990, .055f, 980, 1020) == 0, "DShot configured lower deadband");
	check(dshot3d::convertOutput(1010, .055f, 980, 1020) == 0, "DShot configured upper deadband");
	check(dshot3d::convertOutput(0, .055f, 1000, 1000) == 999, "DShot full lower direction");
	check(dshot3d::convertOutput(1999, .055f, 1000, 1000) == 1999, "DShot full upper direction");
	puts("PASS: M5 slew, hysteresis, reversal confirmation, zero dwell, reset, variable-dt and DShot neutral stop");
}

int main()
{
	test_tail_protection();
	const voliro_t::Geometry g{.14f, .45f, 0.f, .64f, 39.5f, 10.f, .02f};
	// A centered manual throttle requests zero force, but attitude/rate
	// feedback can still request either sign of pitch torque before takeoff.
	for (int pitch_sign = -1; pitch_sign <= 1; pitch_sign += 2) {
		const auto idle = voliro_t::allocate(g, Vector3f{}, Vector3f(.01f, pitch_sign * .2f, -.01f),
						   0.f, math::radians(120.f), math::radians(.9f));
		check(fabsf(idle.tilt) < 1e-6f, "zero force with pitch correction keeps servo neutral");
	}

	for (int tilt_sign = -1; tilt_sign <= 1; tilt_sign += 2) {
		float tilt = tilt_sign * math::radians(60.f);
		const float step = math::radians(.9f);

		for (int i = 0; i < 100; ++i) {
			const auto idle = voliro_t::allocate(g, Vector3f{}, Vector3f{}, tilt, math::radians(120.f), step);
			check(fabsf(idle.tilt - tilt) <= step + 1e-6f, "zero-force centering respects servo rate");
			check(fabsf(idle.tilt) <= fabsf(tilt) + 1e-6f, "zero force moves servo towards neutral");
			tilt = idle.tilt;
		}

		check(fabsf(tilt) < 1e-6f, "zero force returns a previously tilted servo to neutral");
	}

	const float weight = 5.04f * 9.80665f;
	auto hover = voliro_t::allocate(g, Vector3f(0.f, 0.f, -weight), Vector3f{}, 0.f, math::radians(120.f), .1f);
	check(fabsf(hover.thrust[4] - weight * .14f / .78f) < 1e-4f, "14 cm CG hover tail trim");
	check(hover.torque.norm() < 1e-4f && fabsf(hover.force(2) + weight) < 1e-4f, "14 cm CG equilibrium wrench");
// At zero tilt, each motor's yaw torque identifies its reaction sign.
	const float expected_yaw_sign[4] {1.f, -1.f, -1.f, 1.f};

	for (int i = 0; i < 4; ++i) {
		voliro_t::Result single;
		single.thrust[i] = 1.f;
		voliro_t::forward(g, single);

		if (fabsf(single.torque(2) - expected_yaw_sign[i]*g.km) > 1e-6f) { return 6; }
	}

// A small negative pitch correction during takeoff must not send the front
// rotors towards 180 degrees while upward thrust is requested.
	auto takeoff = voliro_t::allocate(g, Vector3f(.005f, 0.f, -.168f), Vector3f(0.f, -.18f, 0.f),
					  0.f, math::radians(120.f), math::radians(2.f));

	if (fabsf(takeoff.tilt) > 1e-5f || takeoff.force(2) > 0.f) { return 5; }

	for (int tail_sign = -1; tail_sign <= 1; tail_sign += 2) {
		for (int degree = -100; degree <= 100; degree += 5) {
			voliro_t::Result source; source.tilt = math::radians(static_cast<float>(degree));
			source.thrust[0] = 8.2f; source.thrust[1] = 7.8f; source.thrust[2] = 7.9f; source.thrust[3] = 8.1f;
			source.thrust[4] = 2.f * static_cast<float>(tail_sign);
			voliro_t::forward(g, source);
			auto r = voliro_t::allocate(g, source.force, source.torque, source.tilt, math::radians(120.f), .1f);

			if ((source.force - r.force).norm() > 1e-4f || (source.torque - r.torque).norm() > 1e-4f) {puts("FAIL feasible wrench"); return 1;}
		}
	}

	auto limited = voliro_t::allocate(g, Vector3f(80.f, 3.f, -300.f), Vector3f(10.f, 15.f, 20.f), 0.f, 1.f, .01f);

	if (fabsf(limited.tilt) > .01001f || fabsf(limited.force(1)) > 1e-5f) { return 2; }

	for (int i = 0; i < 4; ++i) if (limited.thrust[i] < 0.f || limited.thrust[i] > g.front_max) { return 3; }

	if (fabsf(limited.thrust[4]) > g.tail_max) { return 4; }

	puts("PASS: feasible common-tilt wrench -100..100 degrees; side force residual; actuator/angle/rate limits");
}
