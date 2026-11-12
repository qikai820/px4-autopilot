#include "VoliroTAllocation.hpp"
#include "VoliroTTailMotor.hpp"
#include "VoliroTAirmode.hpp"
#include <lib/vector_control/VoliroManualAirmode.hpp>
#include <drivers/dshot/DShot3D.hpp>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
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
	Motor limited;

	for (int i = 0; i < 30; ++i) { limited.update(.6f, dt, settings); }

	check(limited.limitMagnitude(.1f, 1.f) <= .10001f, "throttle reduction caps stored positive tail output");
	check(limited.limitMagnitude(0.f, 0.f) == 0.f, "zero pilot force stops the stored tail command");

	for (int i = 0; i < 7; ++i) {
		check(limited.update(-.5f, dt, settings) == 0.f, "force limiting cannot bypass reverse zero dwell");
	}

	puts("PASS: M5 slew, hysteresis, reversal confirmation, zero dwell, reset, variable-dt and DShot neutral stop");
}

static void test_fixed_tilt_pitch_saturation()
{
	const voliro_t::Geometry log_geometry{.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f};
	// HITL log at 49.24 s: the nose is rotating up while the controller asks
	// for more nose-down torque than the tail can supply. Force tracking must
	// not turn that request into a nose-up tail reversal.
	const Vector3f force(38.7072f, 0.f, 70.3584f), torque(0.f, -7.234f, 0.f);
	const auto recovery = voliro_t::allocateAtTilt(log_geometry, force, torque, math::radians(45.242f));
	const auto achieved = voliro_t::allocateRollYaw(log_geometry, recovery, {}, 0.f);
	check(fabsf(achieved.torque(1) + 6.9f) < 1e-4f, "infeasible fallback torque retains maximum nose-down authority");
	check(fabsf(recovery.tail - 10.f) < 1e-4f, "nose-down saturation must not request tail reversal");

	// Use independently reconstructed actuator corners as the attainable
	// torque envelope, including inverted rotors and asymmetric tail thrust.
	for (float degrees : {-160.f, -90.f, 0.f, 48.f, 90.f, 160.f}) {
		for (float z : {-.05f, 0.f, .05f}) {
			for (float reverse_max : {0.f, 6.f}) {
				const voliro_t::Geometry g{.09f, .45f, z, .69f, 39.5f, 10.f, .02f, reverse_max};
				const float tilt = math::radians(degrees);
				float min_pitch = INFINITY, max_pitch = -INFINITY;

				for (float front : {0.f, g.front_max}) {
					for (float tail : {-voliro_t::tailMax(g, -1.f), g.tail_max}) {
						voliro_t::Result corner;
						corner.tilt = tilt;

						for (int i = 0; i < 4; ++i) { corner.thrust[i] = front; }

						corner.thrust[4] = tail;
						voliro_t::forward(g, corner);
						min_pitch = math::min(min_pitch, corner.torque(1));
						max_pitch = math::max(max_pitch, corner.torque(1));
					}
				}

				for (const Vector3f &demand : {Vector3f(40.f, 0.f, 120.f), Vector3f(-80.f, 0.f, -50.f), Vector3f{}}) {
					for (float pitch : {
						     min_pitch - 1.f, min_pitch - .001f, min_pitch + .001f,
						     max_pitch - .001f, max_pitch + .001f, max_pitch + 1.f
					     }) {
						const auto solution = voliro_t::allocateAtTilt(g, demand, Vector3f(0.f, pitch, 0.f), tilt);
						const auto result = voliro_t::allocateRollYaw(g, solution, {}, 0.f);
						const float expected = math::constrain(pitch, min_pitch, max_pitch);
						check(PX4_ISFINITE(result.torque(1)) && fabsf(result.torque(1) - expected) < 1e-4f,
						      "fixed-tilt allocation stays at closest feasible pitch torque across saturation");
						check(solution.front_sum >= 0.f && solution.front_sum <= 4.f * g.front_max
						      && solution.tail >= -voliro_t::tailMax(g, -1.f) && solution.tail <= g.tail_max,
						      "pitch saturation respects asymmetric actuator bounds");
					}
				}
			}
		}
	}

	puts("PASS: fixed-tilt fallback pitch authority across infeasible demands, inverted rotors and asymmetric tail limits");
}


static void test_airmode_allocation()
{
	const voliro_t::Geometry g{.02f, .45f, 0.f, .74f, 39.5f, 10.f, .02f, .6f};
	check(fabsf(vector_control::voliroAirmodeIntegralScale(.04f, .2f) - .25f) < 1e-6f,
	      "manual low-throttle AirMode slows integral accumulation");
	check(vector_control::voliroAirmodeIntegralScale(.2f, .2f) == 1.f
	      && vector_control::voliroAirmodeIntegralScale(.8f, .2f) == 1.f,
	      "hover and higher thrust restore full configured integral rate");
	check(vector_control::voliroAirmodeIntegralScale(NAN, .2f) == 1.f,
	      "invalid scheduling input cannot produce a nonfinite integral scale");

	for (int curve : {0, 1, 2}) {
		for (int i = 0; i <= 200; ++i) {
			const float stick = -1.f + .01f * i;
			const float ground = vector_control::voliroManualThrust(stick, curve, .2f, 0.f, 1.f);
			const float air = vector_control::voliroManualThrust(stick, curve, .2f, .04f, 1.f);

			if (ground >= .04f) { check(fabsf(ground - air) < 1e-6f, "fixed-stick map cannot jump when landed status changes"); }

			if (i) { check(ground >= vector_control::voliroManualThrust(stick - .01f, curve, .2f, 0.f, 1.f) - 1e-6f, "manual throttle curve monotonicity"); }
		}
	}
	check(fabsf(vector_control::voliroManualThrust(-.748f, 0, .2f, .04f, 1.f) - .0504f) < 1e-6f,
	      "recorded fixed stick remains about 5 percent after takeoff, not 8 percent");
	const Vector3f low_force(0.f, 0.f, -6.72f);
	const Vector3f pitch(0.f, .5f, 0.f);
	const auto enabled = voliro_t::allocateAirmodeAtTilt(g, low_force, pitch, 0.f, false);
	const auto achieved = voliro_t::allocateRollYaw(g, enabled, pitch, 0.f);
	check(fabsf(achieved.torque(1) - .5f) < 1e-5f && enabled.tail >= 0.f,
	      "low-throttle AirMode raises front thrust for pitch without unnecessary M5 reversal");
	check(-achieved.force(2) > 6.72f, "AirMode may increase collective above manual throttle");
	const auto old = voliro_t::allocateForcePitch(g, low_force, pitch, 0.f, 2.8f, .01f, true);
	check(old.tail < 0.f, "recorded small-CG low-throttle case requires reversal in old allocation");

	for (float sign : {-1.f, 1.f}) {
		const Vector3f roll(sign * 2.f, 0.f, 0.f), yaw(0.f, 0.f, sign * .4f);
		const auto rp = voliro_t::allocateAirmodeAtTilt(g, {}, roll, 0.f, false);
		const auto rp_result = voliro_t::allocateRollYaw(g, rp, roll, 0.f);
		check((rp_result.torque - roll).norm() < 1e-4f, "RP AirMode retains roll at zero collective");
		const auto yaw_off = voliro_t::allocateAirmodeAtTilt(g, {}, yaw, 0.f, false);
		const auto yaw_on = voliro_t::allocateAirmodeAtTilt(g, {}, yaw, 0.f, true);
		const auto yaw_result = voliro_t::allocateRollYaw(g, yaw_on, yaw, voliro_t::reactionTorque(0.f, yaw));
		check(yaw_off.front_sum == 0.f && yaw_on.front_sum > 0.f,
		      "only full AirMode boosts collective for a pure yaw demand");
		check((yaw_result.torque - yaw).norm() < 1e-4f, "RPY AirMode retains yaw at zero collective");
	}

	// Independently reconstruct torque from actuator positions across normal,
	// tilted, inverted and zero-pitch-lever cases, including asymmetric M5.
	for (float degrees : {-160.f, -90.f, -45.f, 0.f, 45.f, 90.f, 160.f}) {
		const float tilt = math::radians(degrees);

		for (float fz : {0.f, -6.72f, -49.f, -200.f}) {
			for (float my : {-20.f, -1.f, 0.f, 1.f, 20.f}) {
				for (bool full : {false, true}) {
					const Vector3f torque(.5f, my, .1f);
					const auto solution = voliro_t::allocateAirmodeAtTilt(g, Vector3f(0.f, 0.f, fz), torque, tilt, full);
					const auto result = voliro_t::allocateRollYaw(g, solution, torque, voliro_t::reactionTorque(tilt, torque));
					check(solution.tilt == tilt && result.force.isAllFinite() && result.torque.isAllFinite(),
					      "AirMode keeps the supplied servo angle and finite outputs");
					float min_pitch = INFINITY, max_pitch = -INFINITY;

					for (float front : {0.f, g.front_max}) {
						for (float tail : {-.6f, 10.f}) {
							const float corner = 4.f * g.x * cosf(tilt) * front - g.tail * tail;
							min_pitch = math::min(min_pitch, corner); max_pitch = math::max(max_pitch, corner);
						}
					}
					check(fabsf(result.torque(1) - math::constrain(my, min_pitch, max_pitch)) < 2e-4f,
					      "AirMode preserves closest physically attainable pitch across actuator corners");

					for (int i = 0; i < 4; ++i) {
						check(result.thrust[i] >= 0.f && result.thrust[i] <= g.front_max, "AirMode front actuator bounds");
					}

					check(result.thrust[4] >= -.60001f && result.thrust[4] <= 10.00001f, "AirMode asymmetric tail bounds");
				}
			}
		}
	}

	for (float my : {-.4f, .4f}) {
		const auto wanted = voliro_t::allocateAirmodeAtTilt(g, {}, Vector3f(0.f, my, 0.f), 0.f, false);
		const auto held = voliro_t::boundAirmodeForce(g, {}, wanted, 0.f, 0.f);
		const auto result = voliro_t::allocateRollYaw(g, held, {}, 0.f);
		check(result.force.norm() < 1e-6f && result.torque.norm() < 1e-6f,
		      "zero manual force cannot be converted into lift by pitch feedback");
	}

	puts("PASS: AirMode low/zero throttle RP/RPY, forward-tail preference, torque envelopes, saturation and guarded-tail compensation");
}

int main()
{
	test_airmode_allocation();
	test_tail_protection();
	test_fixed_tilt_pitch_saturation();
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
	// A hard tilt stop must preserve feasible pitch damping even when force
	// tracking has to give way. This protects recovery near independent +/-160.
	const voliro_t::Geometry extreme{.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f, 6.f};
	const float stop = math::radians(160.f);
	const Vector3f extreme_force(weight * sinf(math::radians(165.f)), 0.f, -weight * cosf(math::radians(165.f)));
	const auto priority = voliro_t::allocateForcePitch(extreme, extreme_force, Vector3f(0.f, -.3f, 0.f), stop, stop, .01f, true);
	const auto priority_result = voliro_t::allocateRollYaw(extreme, priority, {}, 0.f);
	check(fabsf(priority_result.torque(1) + .3f) < 1e-5f, "tilt-stop allocation preserves feasible pitch torque");
	const auto wrapped = voliro_t::allocateForcePitch(extreme, Vector3f(-.1f, 0.f, 50.f), {}, stop, stop, .01f, true);
	check(fabsf(wrapped.tilt - stop) < 1e-5f, "atan2 boundary stays at the nearby servo stop");

	for (int j = 0; j < 1000; ++j) {
		const voliro_t::ForcePitchSolution solution{stop, .05f * j, -2.f};
		const auto saturated = voliro_t::allocateRollYaw(extreme, solution, Vector3f(.001f * j, 0.f, 0.f), 100.f);

		for (int i = 0; i < 4; ++i) { check(PX4_ISFINITE(sqrtf(saturated.thrust[i])), "saturated rotor thrust remains physically nonnegative"); }
	}

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
