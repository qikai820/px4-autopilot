#include <lib/vector_control/VoliroTiltMonitor.hpp>
#include "../VectorPosition/VectorPosition.hpp"
#include "../VectorPosition/ControlMath.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTActuatorDynamics.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTTailMotor.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include <lib/vector_control/VectorAngularAccelerationControl.hpp>
#include <geo/geo.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;
using Monitor = vector_control::VoliroTiltMonitor;

static const voliro_t::Geometry geometry{.14f, .45f, 0.f, .64f, 39.5f, 10.f, .02f};

static Monitor::Sample sample(uint64_t now, float actual, float command, const Monitor::Settings &settings,
			      float pitch_moment = .05f)
{
	voliro_t::Result plant;
	plant.tilt = actual;

	for (int i = 0; i < 4; ++i) { plant.thrust[i] = 10.f; }

	plant.thrust[4] = ((settings.x * cosf(actual) + settings.z * sinf(actual)) * 40.f - pitch_moment) / settings.tail;
	auto g = geometry; g.x = settings.x; g.z = settings.z; g.tail = settings.tail;
	voliro_t::forward(g, plant);
	Monitor::Sample result;
	result.acceleration_timestamp = result.angular_timestamp = result.command_timestamp = now;
	result.specific_force = plant.force / settings.mass;
	result.rates = Vector3f(.1f, -.05f, -.2f);
	result.angular_acceleration = (plant.torque - result.rates.cross(settings.inertia.emult(result.rates))).edivide(settings.inertia);
	result.command = command;
	return result;
}

static void test_observation_and_timing()
{
	Monitor::Settings settings;
	uint64_t now = 1000000;

	// Includes reverse tail thrust, nonzero rotor height, gyroscopic coupling,
	// vertical and inverted pitch: this is a BODY-frame angle, not Euler pitch.
	for (float z : {-.04f, 0.f, .04f}) {
		settings.z = z;

		for (float deg : {-160.f, -90.f, -30.f, 0.f, 30.f, 90.f, 160.f}) {
			Monitor monitor;
			const float angle = math::radians(deg);

			for (int i = 0; i < 200; ++i) {
				now += 10000;
				monitor.update(settings, sample(now, angle, angle, settings), now, true, true, true);
			}

			assert(monitor.valid() && !monitor.fault());
			assert(fabsf(monitor.observed() - angle) < 1e-5f);
		}
	}

	settings.z = 0.f;
	Monitor monitor;
	const float stuck = math::radians(25.f);

	for (int i = 0; i < 80; ++i) {
		now += 10000;
		monitor.update(settings, sample(now, stuck, 0.f, settings), now, true, true, true);
	}

	assert(!monitor.fault()); // warmup plus confirmation have not completed

	for (int i = 0; i < 30; ++i) {
		now += 10000;
		monitor.update(settings, sample(now, stuck, 0.f, settings), now, true, true, true);
	}

	assert(monitor.fault() && monitor.valid());
	const float held = monitor.observed();
	now += 1000000;
	monitor.update(settings, {}, now, false, true, false);
	assert(monitor.fault() && !monitor.valid() && monitor.observed() == held);
	monitor.update(settings, {}, now, false, false, false);
	assert(!monitor.fault() && !monitor.valid());

	// Disabled, grounded, stale, future and misaligned samples never trigger.
	for (int kind = 0; kind < 6; ++kind) {
		Monitor invalid;

		for (int i = 0; i < 250; ++i) {
			now += 10000;
			auto s = sample(now, stuck, 0.f, settings);

			if (kind == 2) { s.acceleration_timestamp -= 200000; }

			if (kind == 3) { s.angular_timestamp += 10000; }

			if (kind == 4) { s.angular_timestamp -= 40000; }

			if (kind == 5) { s.specific_force.zero(); s.angular_acceleration.zero(); }

			invalid.update(settings, s, now, kind != 0, true, kind != 1);
		}

		assert(!invalid.fault() && !invalid.valid());
	}

	Monitor repeated;
	auto frozen = sample(now, stuck, 0.f, settings);

	for (int i = 0; i < 500; ++i) {
		repeated.update(settings, frozen, now + i * 1000, true, true, true);
	}

	assert(!repeated.fault());

	Monitor transient;

	for (int i = 0; i < 300; ++i) {
		now += 10000;
		const float actual = i >= 100 && i < 110 ? stuck : 0.f;
		transient.update(settings, sample(now, actual, 0.f, settings), now, true, true, true);
	}

	assert(transient.valid() && !transient.fault());
	Monitor below_threshold;

	for (int i = 0; i < 200; ++i) {
		now += 10000;
		below_threshold.update(settings, sample(now, math::radians(5.f), 0.f, settings), now, true, true, true);
	}

	assert(below_threshold.valid() && !below_threshold.fault());
	std::puts("PASS: tilt observation +/-160 deg, pitch/tail elimination, gating, persistence, stale data and disarm latch");
}

static void test_healthy_servo_lag()
{
	Monitor::Settings settings;
	settings.servo_tau = .2f;
	settings.servo_rate = math::radians(45.f);
	Monitor monitor;
	float actual = 0.f;
	uint64_t now = 1000000;

	for (int i = 0; i < 1500; ++i) {
		const float command = i < 100 ? 0.f : i < 600 ? math::radians(80.f) : math::radians(-50.f);
		actual += math::constrain((1.f - expf(-.01f / settings.servo_tau)) * (command - actual),
					  -settings.servo_rate * .01f, settings.servo_rate * .01f);
		now += 10000;
		monitor.update(settings, sample(now, actual, command, settings), now, true, true, true);
		assert(monitor.valid() && !monitor.fault());
		assert(fabsf(monitor.error()) < 1e-5f);
	}

	std::puts("PASS: healthy rate/lag-limited servo avoids false faults during direction changes");
}

static void configure(VectorPosition &position)
{
	position.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	position.setVelocityGains(Vector3f(2.f, 2.f, 2.f), {}, {});
	position.setVelocityLimits(5.f, 2.f, 2.f); position.setThrustLimits(.05f, .9f);
	position.setHorizontalThrustMargin(.3f); position.setHoverThrust(.3f); position.setTiltLimit(.6f);
}

static void test_fallback_geometry()
{
	VectorPosition fallback, ordinary;
	configure(fallback); configure(ordinary);
	fallback.setVectorAirframe(17); fallback.setMulticopterFallback(true);
	fallback.setAllocationAntiWindupGain(1.f); fallback.setAllocationShortfall(Vector3f(.2f, .1f, .05f));
	auto target = VectorPosition::empty_trajectory_setpoint;
	Vector3f(2.f, 1.f, -2.f).copyTo(target.position); target.yaw = .3f;

	for (int i = 0; i < 100; ++i) {
		for (auto *controller : {&fallback, &ordinary}) {
			controller->setState(VectorPositionStates{Vector3f(0.f, 0.f, -2.f), {}, {}, 0.f});
			controller->setInputSetpoint(target); assert(controller->update(.01f));
		}
	}

	vehicle_attitude_setpoint_s mc{}, degraded{};
	ordinary.getAttitudeSetpoint(mc); fallback.getAttitudeSetpoint(degraded, 1.2f);
	assert((Quatf(mc.q_d).inversed() * Quatf(degraded.q_d)).canonical().imag().norm() < 1e-6f);
	vehicle_local_position_setpoint_s force_mc{}, force_fallback{};
	ordinary.getLocalPositionSetpoint(force_mc); fallback.getLocalPositionSetpoint(force_fallback);
	assert((Vector3f(force_mc.thrust) - Vector3f(force_fallback.thrust)).norm() < 1e-6f);

	for (float degrees : {-60.f, -25.f, 0.f, 25.f, 60.f}) {
		const float tilt = math::radians(degrees);
		const float beta = vector_control::voliroFallbackThrustAngle(tilt, geometry.x, geometry.z, geometry.tail);
		const Quatf q = Quatf(mc.q_d) * Quatf(Eulerf(0.f, beta, 0.f));
		const Vector3f world_force = Vector3f(force_mc.thrust) * 168.f;
		const auto pitch = voliro_t::allocateAtTilt(geometry, Dcmf(q).transpose() * world_force, {}, tilt);
		const auto result = voliro_t::allocateRollYaw(geometry, pitch, {}, 0.f);
		assert(Vector3f(Dcmf(q) * result.force - world_force).norm() < 1e-4f);
		assert(result.torque.norm() < 1e-4f);
		const auto correction = voliro_t::allocateAtTilt(geometry, Dcmf(q).transpose() * world_force, Vector3f(0.f, .1f, 0.f), tilt);
		const auto corrected = voliro_t::allocateRollYaw(geometry, correction, {}, 0.f);
		assert(fabsf(corrected.torque(1) - .1f) < 1e-4f);
		const Quatf previous(Eulerf(0.f, -.7f, .3f));
		const Quatf step = vector_control::slewFallbackAttitude(previous, q, .005f);
		assert(AxisAnglef((previous.inversed() * step).canonical()).angle() <= .00501f);
	}
	std::puts("PASS: ordinary MC position/velocity mapping, nonzero stuck-tilt force alignment, pitch priority and bounded transition");
}

// Close the real position/attitude/rate/allocator cores through rigid-body
// motion. This tests fault recovery, not HIL or real mechanical performance.
static void test_closed_loop_fault(bool stuck)
{
	constexpr float dt = .0025f, mass = 5.04f, total = 168.f;
	const voliro_t::Geometry g{.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f};
	const Vector3f inertia(.18f, .12f, .43f);
	VectorPosition position; configure(position);
	position.setVectorAirframe(17); position.setHoverThrust(mass * CONSTANTS_ONE_G / total);
	position.setAllocationAntiWindupGain(1.f);
	position.setVelocityGains(Vector3f(1.2f, 1.2f, 4.f), Vector3f(.4f, .4f, .5f), Vector3f(.2f, .2f, 0.f));
	VectorAttitude attitude; attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(4.f, 4.f, 2.f), 1.f);
	attitude.setRateLimit(Vector3f(2.f, 2.f, 2.f));
	vector_control::AngularAccelerationControl rate;
	rate.configure(Vector3f(8.f, 12.f, 8.f), Vector3f(1.f, .3f, 1.f), Vector3f(.3f, .3f, .3f), 2.f, 2.f);
	Monitor monitor; Monitor::Settings settings;
	settings.x = g.x; settings.z = g.z; settings.tail = g.tail; settings.mass = mass; settings.inertia = inertia;
	Vector3f p(0.f, 0.f, -2.f), velocity{}, acceleration{}, omega{}, angular_acceleration{}, shortfall{}, torque_shortfall{};
	Vector3f force(0.f, 0.f, -mass * CONSTANTS_ONE_G), specific_force(0.f, 0.f, -CONSTANTS_ONE_G);
	Quatf q, target;
	float servo_command = 0.f, actual_tilt = 0.f, held_tilt = 0.f, fault_time = NAN, peak_position = 0.f;
	voliro_t::ActuatorDynamics motors;
	const auto trim = voliro_t::allocate(g, force, {}, 0.f, math::radians(160.f), 1.f);
	motors.initialize(trim, g);
	voliro_t::TailMotor tail;
	const voliro_t::TailMotor::Settings tail_settings{.4f, .03f, .1f, .04f, .08f};

	for (int i = 0; i < 400; ++i) { tail.update(voliro_t::tailCommand(g, trim.thrust[4]), dt, tail_settings); }

	for (int k = 0; k < 20000; ++k) {
		const float time = k * dt;
		const uint64_t now = 1000000 + k * 2500;

		if (k % 4 == 0) {
			Monitor::Sample s;
			s.acceleration_timestamp = s.angular_timestamp = s.command_timestamp = now;
			s.specific_force = specific_force; s.rates = omega; s.angular_acceleration = angular_acceleration;
			s.command = servo_command;
			const bool previous_fault = monitor.fault();
			monitor.update(settings, s, now, true, true, true);

			if (!previous_fault && monitor.fault()) { fault_time = time; position.resetIntegralXY(); }

			position.setMulticopterFallback(monitor.fault());
			position.setState(VectorPositionStates{p, velocity, acceleration, 0.f});
			position.setAllocationShortfall(Vector3f(Dcmf(q) * shortfall) / total);
			auto sp = VectorPosition::empty_trajectory_setpoint;

			if (time >= 12.f && time < 17.f) {
				Vector3f(.5f, .2f, 0.f).copyTo(sp.velocity);

			} else {
				Vector3f(0.f, 0.f, -2.f).copyTo(sp.position);
			}

			sp.yaw = 0.f;
			position.setInputSetpoint(sp); assert(position.update(4.f * dt));
			vehicle_local_position_setpoint_s out{}; position.getLocalPositionSetpoint(out);
			force = Vector3f(out.thrust) * total;
			vehicle_attitude_setpoint_s attitude_sp{};
			position.getAttitudeSetpoint(attitude_sp, stuck && time >= 4.f && !monitor.fault() ? math::radians(30.f) : 0.f);
			Quatf next(attitude_sp.q_d);

			if (monitor.fault()) {
				const float beta = vector_control::voliroFallbackThrustAngle(monitor.observed(), g.x, g.z, g.tail);
				next = next * Quatf(Eulerf(0.f, beta, 0.f));
			}

			if (stuck || monitor.fault()) {
				next = vector_control::slewFallbackAttitude(target, next, math::radians(monitor.fault() ? 80.f : 20.f) * 4.f * dt);
			}

			target = next;
		}

		attitude.setAttitudeSetpoint(target, 0.f);
		const Vector3f torque = inertia.emult(rate.update(attitude.update(q) - omega, angular_acceleration, {},
						      torque_shortfall.edivide(inertia), dt, false)) + omega.cross(inertia.emult(omega));
		const Vector3f body_force = Dcmf(q).transpose() * force;
		auto solution = monitor.fault() ? voliro_t::allocateAtTilt(g, body_force, torque, monitor.observed())
				: voliro_t::allocateForcePitch(g, body_force, torque, servo_command, math::radians(160.f), settings.servo_rate * dt, true);
		const float tail_request = voliro_t::tailCommand(g, solution.tail);
		auto preview = tail;
		const float available_tail = voliro_t::tailThrust(g, preview.update(tail_request, dt, tail_settings));

		if (fabsf(available_tail - solution.tail) > 1e-4f) {
			solution = monitor.fault() ? voliro_t::refitAtTiltForTail(g, body_force, torque, monitor.observed(), available_tail)
				   : voliro_t::refitForTail(g, body_force, torque, available_tail, servo_command, math::radians(160.f), settings.servo_rate * dt);
		}

		auto allocation = voliro_t::allocateRollYaw(g, solution, torque, voliro_t::reactionTorque(solution.tilt, torque));
		servo_command = monitor.fault() ? servo_command + math::constrain(-servo_command,
				-math::radians(10.f) * dt, math::radians(10.f) * dt) : allocation.tilt;

		if (time < 4.f) { actual_tilt = servo_command; held_tilt = actual_tilt; }

		else { actual_tilt = stuck ? held_tilt : servo_command + math::radians(20.f); }

		float commands[5];

		for (int i = 0; i < 5; ++i) { commands[i] = i == 4 ? voliro_t::tailCommand(g, allocation.thrust[i]) : sqrtf(allocation.thrust[i] / g.front_max); }

		commands[4] = tail.update(tail_request, dt, tail_settings);
		allocation.thrust[4] = voliro_t::tailThrust(g, commands[4]);
		// Match production anti-windup: final-command reconstruction, with an
		// observed angle only after fallback; never use motor-lag prediction.
		auto available = allocation;
		available.tilt = monitor.fault() ? monitor.observed() : servo_command;
		voliro_t::forward(g, available);
		shortfall = body_force - available.force; torque_shortfall = torque - available.torque;
		auto actual = motors.update(g, commands, actual_tilt, .04f, 0.f, math::radians(10000.f), dt);
		specific_force = actual.force / mass;
		acceleration = Dcmf(q) * specific_force + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
		angular_acceleration = (actual.torque - omega.cross(inertia.emult(omega))).edivide(inertia);
		velocity += acceleration * dt; p += velocity * dt; omega += angular_acceleration * dt;
		q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
		peak_position = math::max(peak_position, (p - Vector3f(0.f, 0.f, -2.f)).norm());
		assert(p.isAllFinite() && q.isAllFinite() && omega.norm() < 5.f);
	}

	const float final_error = (p - Vector3f(0.f, 0.f, -2.f)).norm();
	std::fprintf(stderr, "Tilt fault recovery stuck=%d trigger=%.3fs peak_pos=%.3fm final_pos=%.3fm speed=%.3fm/s tilt=%.2f pitch=%.2f\n",
		     stuck, (double)fault_time, (double)peak_position, (double)final_error, (double)velocity.norm(),
		     (double)math::degrees(monitor.observed()), (double)math::degrees(Eulerf(q).theta()));
	assert(monitor.fault() && fault_time > 4.f && fault_time < 7.f);
	assert(final_error < .15f && velocity.norm() < .1f && peak_position < 10.f);
}

int main()
{
	test_observation_and_timing();
	test_healthy_servo_lag();
	test_fallback_geometry();
	test_closed_loop_fault(false);
	test_closed_loop_fault(true);
}
