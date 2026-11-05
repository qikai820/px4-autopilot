#include "../VectorPosition/VectorPosition.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTActuatorDynamics.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTTailMotor.hpp"
#include <lib/vector_control/VoliroFixedTiltTransition.hpp>
#include <lib/vector_control/VoliroTiltMonitor.hpp>
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <lib/vector_control/VectorAngularAccelerationControl.hpp>
#include <geo/geo.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;
using Handover = vector_control::VoliroFixedTiltTransition;

static void test_handover_gates()
{
	const Quatf level, pitched(Eulerf(0.f, math::radians(73.f), 0.f));
	assert(Handover::ready(level, level, {}, 0.f, 0.f));
	assert(Handover::ready(-level, -level, {}, 0.f, 0.f));
	assert(!Handover::ready(pitched, level, {}, 0.f, 0.f));
	assert(!Handover::ready(level, pitched, {}, 0.f, 0.f));
	assert(!Handover::ready(Quatf(Eulerf(0.f, math::radians(179.f), 0.f)), level, {}, 0.f, 0.f));
	assert(!Handover::ready(level, level, {}, math::radians(20.f), 0.f));
	assert(!Handover::ready(level, level, {}, 0.f, math::radians(20.f)));
	assert(!Handover::ready(level, level, Vector3f(0.f, math::radians(30.f), 0.f), 0.f, 0.f));
	assert(!Handover::ready(level, level, {}, NAN, 0.f));
	Handover handover;
	assert(!handover.update(true, false, 1000000) && handover.pending());
	assert(!handover.update(true, true, 1010000));
	assert(!handover.update(true, true, 1200000));
	assert(!handover.update(true, false, 1205000)); // a bad sample restarts confirmation
	assert(!handover.update(true, true, 1210000));
	assert(handover.update(true, true, 1410000) && !handover.pending());
	assert(handover.update(true, false, 1420000)); // normal MC maneuvers do not undo handover
	assert(!handover.update(false, true, 1430000) && !handover.pending());
	assert(!handover.update(true, true, 1440000)); // a new request needs confirmation again
	assert(!handover.update(false, false, 1450000)); // pending cancellation/disarm/fault
	assert(!handover.pending());
	puts("PASS: fixed-tilt attitude/rotor/rate gates, confirmation, cancellation and reentry");
}

struct Metrics {
	float height_loss{0.f}, max_speed{0.f}, entry_time{-1.f}, entry_pitch{0.f}, entry_tilt{0.f}, final_error{0.f};
};

// The log's 200 deg/s servo, 80 deg/s fallback quaternion slew and 30 deg/s
// independent pitch reference. Compare immediate centering with staged handover
// using actual controller/reference/allocation cores and a 400 Hz rigid body.
// This plant has no ground contact: crossing z=0 means the 1 m clearance is lost.
static Metrics simulate(float degrees, bool staged, float motor_tau = 0.f, float servo_tau = 0.f)
{
	constexpr float dt = .0025f, mass = 5.04f, total = 168.f;
	const voliro_t::Geometry g{.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f};
	const Vector3f inertia(.1f, .1f, .2f);
	const float pitch = math::radians(degrees), servo_rate = math::radians(200.f);
	VectorPosition position;
	position.setVectorAirframe(17); position.setHoverThrust(mass * CONSTANTS_ONE_G / total);
	position.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	position.setVelocityGains(Vector3f(1.2f, 1.2f, 4.f), Vector3f(.4f, .4f, .5f), Vector3f(.2f, .2f, 0.f));
	position.setVelocityLimits(3.f, 2.f, 2.f); position.setThrustLimits(.05f, .9f);
	position.setHorizontalThrustMargin(.3f); position.setTiltLimit(math::radians(25.f));
	position.setAllocationAntiWindupGain(1.f);
	VectorAttitude attitude; attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(4.f, 4.f, 2.f), 1.f);
	attitude.setRateLimit(Vector3f(math::radians(120.f), math::radians(160.f), math::radians(80.f)));
	vector_control::AngularAccelerationControl rate;
	rate.configure(Vector3f(8.f, 12.f, 8.f), Vector3f(1.f, .3f, 1.f), Vector3f(.3f, .3f, .3f), 2.f, 2.f);
	vector_control::AttitudeReference reference;
	vector_control::AttitudeReference::Settings settings;
	settings.pitch_rate = math::radians(30.f);
	vector_control::AttitudeReference::Output ref;
	vector_control::VoliroTiltMonitor monitor;
	vector_control::VoliroTiltMonitor::Settings monitor_settings;
	monitor_settings.x = g.x; monitor_settings.z = g.z; monitor_settings.tail = g.tail;
	monitor_settings.mass = mass; monitor_settings.inertia = inertia;
	monitor_settings.servo_rate = servo_rate; monitor_settings.servo_tau = servo_tau;
	Handover handover;
	voliro_t::TailMotor tail;
	const voliro_t::TailMotor::Settings tail_settings{.4f, .03f, .1f, .04f, .08f};
	Vector3f p(0.f, 0.f, -1.f), v{}, acceleration{}, omega{}, alpha{}, shortfall{}, torque_shortfall{}, ref_shortfall{}, rate_sp{};
	Vector3f force(0.f, 0.f, -mass * CONSTANTS_ONE_G);
	Quatf q(Eulerf(0.f, pitch, .7f)), target = q;
	assert(reference.initialize(q, settings.bank_limit));
	const auto trim = voliro_t::allocate(g, Dcmf(q).transpose() * force, {}, pitch, math::radians(160.f), 10.f);
	float servo = trim.tilt;
	voliro_t::ActuatorDynamics plant;
	plant.initialize(trim, g);
	auto actual = trim;
	for (int i = 0; i < 1000; ++i) { tail.update(voliro_t::tailCommand(g, trim.thrust[4]), dt, tail_settings); }
	bool fixed = false, ref_valid = true;
	Metrics metrics;
	for (int k = 0; k < 8000; ++k) {
		const float time = k * dt;
		if (k % 4 == 0) {
			const uint64_t now = 1000000 + uint64_t(k) * 2500;
			vector_control::VoliroTiltMonitor::Sample sample;
			sample.acceleration_timestamp = sample.angular_timestamp = sample.command_timestamp = now;
			sample.specific_force = actual.force / mass; sample.rates = omega;
			sample.angular_acceleration = alpha; sample.command = servo;
			monitor.update(monitor_settings, sample, now, true, true, true);
			const float observed = monitor.valid() ? monitor.observed() : actual.tilt;
			const bool requested = time >= 1.f;
			const bool next_fixed = staged ? handover.update(requested,
				Handover::ready(q, target, omega, observed, servo), now) : requested;
			if (next_fixed && !fixed) {
				position.resetIntegralXY(); reference.reset(); ref_shortfall.zero();
				metrics.entry_time = time;
				metrics.entry_pitch = atan2f(-Dcmf(q)(2, 0), Dcmf(q)(2, 2)); metrics.entry_tilt = actual.tilt;
			}
			fixed = next_fixed;
			position.setMulticopterFallback(fixed);
			position.setState(VectorPositionStates{p, v, acceleration, .7f});
			position.setAllocationShortfall(Dcmf(q) * shortfall / total + ref_shortfall);
			auto sp = VectorPosition::empty_trajectory_setpoint;
			Vector3f(0.f, 0.f, -1.f).copyTo(sp.position); sp.yaw = .7f;
			position.setInputSetpoint(sp); assert(position.update(4.f * dt));
			vehicle_local_position_setpoint_s out{}; position.getLocalPositionSetpoint(out);
			const Vector3f raw_force(out.thrust);
			ref_valid = !fixed;
			if (ref_valid) {
				assert(reference.update(raw_force, .7f, 0.f, requested ? 0.f : pitch,
						       reference.pitchAuthority(q), 4.f * dt, settings, ref));
				target = ref.attitude; force = ref.force * total; ref_shortfall = raw_force - ref.force;
			} else {
				vehicle_attitude_setpoint_s asp{}; position.getAttitudeSetpoint(asp, 0.f);
				Quatf next = Quatf(asp.q_d) * Quatf(Eulerf(0.f,
					vector_control::voliroFallbackThrustAngle(observed, g.x, g.z, g.tail), 0.f));
				target = vector_control::slewFallbackAttitude(target, next, math::radians(80.f) * 4.f * dt);
				force = raw_force * total; ref_shortfall.zero();
			}
		}
		if (k % 2 == 0) {
			attitude.setAttitudeSetpoint(target, 0.f);
			attitude.setTaskReference(Vector3f(2.f, 2.f, 1.f), .7f, ref_valid ? ref.omega_ned : Vector3f{});
			rate_sp = attitude.update(q);
		}
		const Vector3f omega_ff = ref_valid ? Vector3f(Dcmf(q).transpose() * ref.omega_ned) : Vector3f{};
		const Vector3f alpha_ff = ref_valid ? Vector3f(Dcmf(q).transpose() * ref.alpha_ned - omega.cross(omega_ff)) : Vector3f{};
		const Vector3f torque = inertia.emult(alpha_ff + rate.update(rate_sp - omega, alpha, alpha_ff,
					       torque_shortfall.edivide(inertia), dt, false)) + omega.cross(inertia.emult(omega));
		const Vector3f body_force = Dcmf(q).transpose() * force;
		const float center = servo + math::constrain(-servo, -servo_rate * dt, servo_rate * dt);
		auto solution = fixed ? voliro_t::allocateAtTilt(g, body_force, torque, center)
			: voliro_t::allocateForcePitch(g, body_force, torque, servo, math::radians(160.f), servo_rate * dt, true);
		const float tail_request = voliro_t::tailCommand(g, solution.tail);
		auto preview = tail;
		const float available_tail = voliro_t::tailThrust(g, preview.update(tail_request, dt, tail_settings));
		if (fabsf(available_tail - solution.tail) > 1e-4f) {
			solution = fixed ? voliro_t::refitAtTiltForTail(g, body_force, torque, center, available_tail)
				: voliro_t::refitForTail(g, body_force, torque, available_tail, servo, math::radians(160.f), servo_rate * dt);
		}
		auto output = voliro_t::allocateRollYaw(g, solution, torque, voliro_t::reactionTorque(solution.tilt, torque));
		servo = fixed ? center : output.tilt;
		float commands[5];
		for (int i = 0; i < 5; ++i) { commands[i] = i == 4 ? voliro_t::tailCommand(g, output.thrust[i]) : sqrtf(output.thrust[i] / g.front_max); }
		commands[4] = tail.update(tail_request, dt, tail_settings);
		output.thrust[4] = voliro_t::tailThrust(g, commands[4]); output.tilt = servo; voliro_t::forward(g, output);
		shortfall = body_force - output.force; torque_shortfall = torque - output.torque;
		actual = plant.update(g, commands, servo, motor_tau, servo_tau, servo_rate, dt);
		alpha = (actual.torque - omega.cross(inertia.emult(omega))).edivide(inertia);
		omega += alpha * dt; q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
		acceleration = Dcmf(q) * actual.force / mass + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
		v += acceleration * dt; p += v * dt;
		assert(p.isAllFinite() && q.isAllFinite());
		metrics.height_loss = math::max(metrics.height_loss, p(2) + 1.f);
		metrics.max_speed = math::max(metrics.max_speed, v.norm());
		if (!staged && metrics.height_loss > 1.f) { break; }
		assert(omega.norm() < 20.f && p.norm() < 50.f);
	}
	metrics.final_error = (p - Vector3f(0.f, 0.f, -1.f)).norm();
	if (staged) { assert(fixed && fabsf(servo) < 1e-6f); }
	printf("%s initial=%+.0fdeg motor/servo_tau=%.2f/%.2fs: entry=%.2fs pitch/tilt=%+.2f/%+.2fdeg, height loss=%.4fm peak speed=%.3fm/s final error=%.4fm\n",
		staged ? "STAGED" : "IMMEDIATE", (double)degrees, (double)motor_tau, (double)servo_tau,
		(double)metrics.entry_time, (double)math::degrees(metrics.entry_pitch), (double)math::degrees(metrics.entry_tilt),
		(double)metrics.height_loss, (double)metrics.max_speed, (double)metrics.final_error);
	fflush(stdout);
	return metrics;
}

int main()
{
	test_handover_gates();
	const auto unsafe = simulate(-73.f, false);
	assert(unsafe.height_loss > 1.f); // reproduces loss of the log's 1 m clearance
	for (float degrees : {-120.f, -73.f, 73.f, 120.f}) {
		for (bool lag : {false, true}) {
			const auto safe = simulate(degrees, true, lag ? .04f : 0.f, lag ? .1f : 0.f);
			assert(safe.entry_time > 2.f && safe.entry_time < 15.f);
			assert(fabsf(safe.entry_pitch) < math::radians(5.f) && fabsf(safe.entry_tilt) < math::radians(5.f));
			assert(safe.height_loss < .2f && safe.final_error < .2f && safe.max_speed < 2.f);
		}
	}
}
