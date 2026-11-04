#include "../VectorPosition/VectorPosition.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../vector_rate_control/RateControl/rate_control.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTTailMotor.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTActuatorDynamics.hpp"
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <lib/vector_control/VectorAngularAccelerationControl.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <geo/geo.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <cstring>
#ifdef VECTOR_TEST_JSBSIM
#include "NativeVoliroPlant.hpp"
#include <memory>
static const char *native_model = nullptr, *native_systems = nullptr;
static matrix::Vector3f native_inertia{};
static voliro_t::Geometry native_geometry{};
#endif

using namespace matrix;
struct Tuning { float py{12.f}, iy{.3f}, dy{.3f}, pitch_p{2.f}; };
static Tuning tuning;

struct Metrics {
	float peak_error{0.f}, final_error{0.f}, final_position{0.f};
	float early_bank{0.f}, late_bank{0.f};
	bool finite{true};
};

// Deterministic controller/allocator integration test, not a JSBSim/HIL claim.
// Position 100 Hz, attitude 200 Hz, gyro/allocation 400 Hz. The actuator model
// has motor command lag, a physical servo rate and the real M5 reversal guard.
static Metrics simulate(bool dynamics, float degrees, float motor_tau, float inertia_scale, bool transition, FILE *csv = nullptr,
			float predicted_motor_tau = 0.f, bool asymmetric_tail = false, bool feedforward_loss = false, bool reverse_pitch = false)
{
	constexpr float dt = .0025f, mass = 5.04f, total_thrust = 168.f;
	constexpr float gravity = CONSTANTS_ONE_G;
	const float hover = mass * gravity / total_thrust;
	Vector3f inertia(.18f, .12f, .43f);
	voliro_t::Geometry geometry{.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f, asymmetric_tail ? 6.f : 0.f};
#ifdef VECTOR_TEST_JSBSIM

	if (native_model) { inertia = native_inertia; geometry = native_geometry; }

#endif
	const Vector3f plant_inertia = inertia * inertia_scale;
	const float target_pitch = math::radians(degrees);
	VectorPosition position;
	position.setVectorAirframe(17);
	position.setAllocationAntiWindupGain(dynamics ? 1.f : 0.f);
	position.setPositionGains(Vector3f(.95f, .95f, 1.f));
	position.setVelocityGains(Vector3f(1.2f, 1.2f, 4.f), Vector3f(.4f, .4f, .5f), Vector3f(.2f, .2f, 0.f));
	position.setVelocityLimits(3.f, 2.f, 2.f);
	position.setThrustLimits(.05f, 1.f);
	position.setHorizontalThrustMargin(.3f);
	position.setHoverThrust(hover);
	position.setTiltLimit(math::radians(20.f));
	VectorAttitude attitude;
	attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(5.5f, 3.f, 1.5f), 1.f);
	attitude.setRateLimit(Vector3f(math::radians(120.f), math::radians(160.f), math::radians(80.f)));
	VectorRateCore legacy_rate;
	legacy_rate.setPidGains(Vector3f(.26f, .11f, .1f), Vector3f(.2f, .2f, .1f), Vector3f(.002f, .001f, 0.f));
	legacy_rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
	vector_control::AngularAccelerationControl rate;
	rate.configure(Vector3f(8.f, tuning.py, 8.f), Vector3f(1.f, tuning.iy, 1.f), Vector3f(.3f, tuning.dy, .3f), 2.f, 2.f);
	vector_control::AttitudeReference reference;
	vector_control::AttitudeReference::Settings reference_settings;
	reference_settings.pitch_rate = math::radians(30.f);
	vector_control::AttitudeReference::Output ref;
	AlphaFilter<float> legacy_yaw;
	legacy_yaw.setCutoffFreq(2.f);
	voliro_t::ReactionTorqueFilter reaction_filter;
	voliro_t::TailMotor tail;
	const voliro_t::TailMotor::Settings tail_settings{.4f, .03f, .1f, .04f, .08f};
	Vector3f p(0.f, 0.f, -2.f), v{}, acceleration{}, omega{}, angular_acceleration{};
	Vector3f force(0.f, 0.f, -hover), force_shortfall{}, reference_shortfall{}, torque_shortfall{}, rate_sp{};
	float pitch_sp = transition ? 0.f : target_pitch;
	Quatf q = Quatf(Eulerf(.02f, 0.f, 0.f)) * Quatf(Eulerf(0.f, pitch_sp, 0.f));
	Quatf qd(Eulerf(0.f, pitch_sp, 0.f));
	assert(reference.initialize(q, reference_settings.bank_limit));
	float servo = pitch_sp;
	const auto trim = voliro_t::allocate(geometry, Dcmf(qd).transpose() * force * total_thrust, {}, servo,
					     math::radians(160.f), 10.f);
	voliro_t::ActuatorDynamics plant;
	voliro_t::ActuatorDynamics prediction;
	plant.initialize(trim, geometry);
	prediction.initialize(trim, geometry);
	const float tail_trim = voliro_t::tailCommand(geometry, trim.thrust[4]);
#ifdef VECTOR_TEST_JSBSIM
	std::unique_ptr<NativeVoliroPlant> native;

	if (native_model) {
		float trim_commands[5] {};

		for (int i = 0; i < 4; ++i) { trim_commands[i] = sqrtf(trim.thrust[i] / geometry.front_max); }

		trim_commands[4] = tail_trim;
		native.reset(new NativeVoliroPlant(native_model, native_systems, q, trim_commands, trim.tilt, dt));
	}

#endif

	for (int i = 0; i < 400; ++i) { tail.update(tail_trim, dt, tail_settings); }

	Metrics metrics;
	float position_energy = 0.f, attitude_energy = 0.f;
	unsigned final_samples = 0;

	for (int k = 0; k < 30000; ++k) {
		const float time = k * dt;
		const float demanded_pitch = transition ? (time < 5.f || time > 50.f ? 0.f : reverse_pitch
					     && time >= 25.f ? -target_pitch : target_pitch) : target_pitch;
		const float heading = transition && time >= 25.f && time <= 45.f ? math::radians(20.f) : 0.f;
		const bool feedforward_available = !(feedforward_loss && time >= 20.f && time < 45.f);

		if (k % 4 == 0) {
			position.setState(VectorPositionStates{p, v, acceleration, vector_control::forcePlaneHeading(q)});
			position.setAllocationShortfall(Dcmf(q) * force_shortfall / total_thrust + reference_shortfall);
			auto setpoint = VectorPosition::empty_trajectory_setpoint;
			Vector3f(0.f, 0.f, -2.f).copyTo(setpoint.position);
			setpoint.yaw = heading;
			position.setInputSetpoint(setpoint);

			if (!position.update(4.f * dt)) { metrics.finite = false; break; }

			vehicle_local_position_setpoint_s out{};
			position.getLocalPositionSetpoint(out);
			const Vector3f raw_force(out.thrust);

			if (dynamics) {
				assert(reference.update(raw_force, heading, 0.f, demanded_pitch, reference.pitchAuthority(q), 4.f * dt, reference_settings, ref));
				qd = ref.attitude; force = ref.force;
				reference_shortfall = raw_force - force;

			} else {
				pitch_sp += math::constrain(demanded_pitch - pitch_sp, -reference_settings.pitch_rate * 4.f * dt,
							    reference_settings.pitch_rate * 4.f * dt);
				force = raw_force;
				qd = vector_control::attitudeForForce(force, heading, 17, reference_settings.bank_limit, pitch_sp);
			}
		}

		if (k % 2 == 0) {
			attitude.setAttitudeSetpoint(qd, 0.f);

			if (dynamics) {
				attitude.setTaskReference(Vector3f(2.f, tuning.pitch_p, 1.f), ref.heading,
							  feedforward_available ? ref.omega_ned : Vector3f{});
			}

			rate_sp = attitude.update(q);
		}

		Vector3f torque;

		if (dynamics) {
			const Vector3f omega_ff = feedforward_available ? Vector3f(Dcmf(q).transpose() * ref.omega_ned) : Vector3f{};
			const Vector3f alpha_ff = feedforward_available ? Vector3f(Dcmf(q).transpose() * ref.alpha_ned - omega.cross(omega_ff)) : Vector3f{};
			torque = inertia.emult(alpha_ff + rate.update(rate_sp - omega, angular_acceleration, alpha_ff,
					       torque_shortfall.edivide(inertia), dt, false)) + omega.cross(inertia.emult(omega));

		} else {
			legacy_rate.setSaturationStatus(Vector3<bool>(torque_shortfall(0) > 1e-5f, torque_shortfall(1) > 1e-5f, torque_shortfall(2) > 1e-5f),
							Vector3<bool>(torque_shortfall(0) < -1e-5f, torque_shortfall(1) < -1e-5f, torque_shortfall(2) < -1e-5f));
			torque = 10.f * legacy_rate.update(omega, rate_sp, angular_acceleration, dt, false);
			torque(2) = legacy_yaw.update(torque(2), dt);
		}

		const Vector3f body_force = Dcmf(q).transpose() * force * total_thrust;
		auto solution = voliro_t::allocateForcePitch(geometry, body_force, torque, servo, math::radians(160.f), math::radians(90.f) * dt, dynamics);
		const float tail_request = voliro_t::tailCommand(geometry, solution.tail);

		if (dynamics) {
			auto preview = tail;
			const float command = preview.update(tail_request, dt, tail_settings);
			const float available_tail = voliro_t::tailThrust(geometry, command);

			if (fabsf(available_tail - solution.tail) > 1e-4f) {
				solution = voliro_t::refitForTail(geometry, body_force, torque, available_tail, servo,
								  math::radians(160.f), math::radians(90.f) * dt);
			}
		}

		float reaction = voliro_t::reactionTorque(solution.tilt, torque);

		if (dynamics) { reaction = reaction_filter.update(reaction, 2.f, dt); }

		auto commanded = voliro_t::allocateRollYaw(geometry, solution, torque, reaction);
		servo = commanded.tilt;
		float commands[5] {};

		for (int i = 0; i < 5; ++i) {
			commands[i] = i == 4 ? voliro_t::tailCommand(geometry, commanded.thrust[i])
				      : sqrtf(commanded.thrust[i] / geometry.front_max);
		}

		commands[4] = tail.update(tail_request, dt, tail_settings);
		commanded.thrust[4] = voliro_t::tailThrust(geometry, commands[4]);
		voliro_t::forward(geometry, commanded);
		force_shortfall = body_force - commanded.force;
		torque_shortfall = torque - commanded.torque;
		const auto achieved = plant.update(geometry, commands, servo, motor_tau, 0.f, 3.f, dt);
		prediction.update(geometry, commands, servo, predicted_motor_tau, 0.f, math::radians(90.f), dt);
		const Vector3f disturbance = time >= 10.f && time < 10.25f ? Vector3f(.03f, .03f, .03f) : Vector3f{};
#ifdef VECTOR_TEST_JSBSIM

		if (native) {
			native->step(commands, servo, q, omega, angular_acceleration, p, v, acceleration);

		} else
#endif
		{
			angular_acceleration = (achieved.torque + disturbance - omega.cross(plant_inertia.emult(omega))).edivide(plant_inertia);
			omega += angular_acceleration * dt;
			q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
			acceleration = Dcmf(q) * achieved.force / mass + Vector3f(0.f, 0.f, gravity);
			v += acceleration * dt; p += v * dt;
		}

		const float error = 2.f * asinf(math::min((q.inversed() * qd).canonical().imag().norm(), 1.f));
		const float bank = asinf(math::constrain(Dcmf(q)(2, 1), -1.f, 1.f));
		metrics.peak_error = math::max(metrics.peak_error, error);

		if (time >= 10.f && time < 20.f) { metrics.early_bank = math::max(metrics.early_bank, fabsf(bank)); }

		if (time >= 60.f) {
			metrics.late_bank = math::max(metrics.late_bank, fabsf(bank));
			position_energy += (p - Vector3f(0.f, 0.f, -2.f)).norm_squared();
			attitude_energy += error * error;
			++final_samples;
		}

		if (csv && k % 4 == 0) {
			const Dcmf actual_rotation(q), desired_rotation(qd);
			fprintf(csv, "%.3f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n", (double)time,
				(double)bank, (double)error, (double)p(1), (double)omega(2), (double)rate_sp(2), (double)servo,
				(double)atan2f(-actual_rotation(2, 0), actual_rotation(2, 2)),
				(double)atan2f(-desired_rotation(2, 0), desired_rotation(2, 2)), (double)omega(1),
				(double)rate_sp(1), (double)commands[4], (double)torque_shortfall(1), (double)rate.integral()(1));
		}

		if (!q.isAllFinite() || !p.isAllFinite() || p.norm() > 100.f || omega.norm() > 30.f) { metrics.finite = false; break; }
	}

	metrics.final_position = final_samples ? sqrtf(position_energy / final_samples) : INFINITY;
	metrics.final_error = final_samples ? sqrtf(attitude_energy / final_samples) : INFINITY;
	printf("%s pitch=%+.0f tau=%.3f J=%.2f transition=%d ff_loss=%d reverse=%d finite=%d peak=%.2fdeg final=%.3fdeg pos=%.3fm bank_early/late=%.2f/%.2fdeg\n",
	       dynamics ? "dynamic" : "legacy", (double)degrees, (double)motor_tau, (double)inertia_scale, transition, feedforward_loss, reverse_pitch,
	       metrics.finite,
	       (double)math::degrees(metrics.peak_error), (double)math::degrees(metrics.final_error), (double)metrics.final_position,
	       (double)math::degrees(metrics.early_bank), (double)math::degrees(metrics.late_bank));
	return metrics;
}

static void requireStable(const Metrics &metrics, bool transition, bool feedforward_lost = false)
{
	// Physical attitude error, not conventional Euler yaw at its singularity.
	assert(metrics.finite);
	// Loss of FF permits lag during the 20-degree heading command, but must
	// still settle to the SAME final bounds without increasing bank oscillation.
	assert(metrics.peak_error < math::radians(feedforward_lost ? 20.f : transition ? 10.f : 3.f));
	assert(metrics.final_error < math::radians(1.f));
	assert(metrics.final_position < .1f);
	assert(metrics.late_bank < math::radians(.5f));
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, nullptr, _IOLBF, 0);
#ifdef VECTOR_TEST_JSBSIM

	if (argc == 11 && strcmp(argv[1], "--jsbsim") == 0) {
		native_model = argv[2]; native_systems = argv[3];
		native_inertia = Vector3f(strtof(argv[4], nullptr), strtof(argv[5], nullptr), strtof(argv[6], nullptr));
		native_geometry = {strtof(argv[7], nullptr), strtof(argv[8], nullptr), strtof(argv[9], nullptr), strtof(argv[10], nullptr),
				   39.5f, 10.f, .02f, 6.f
				  };
		puts("Native JSBSim plant; current XML inertia/geometry, 6 N reverse tail; no network I/O");
		simulate(false, 90.f, 0.f, 1.f, false);

		for (float angle : {0.f, -160.f, -140.f, -100.f, -90.f, -85.f, 85.f, 90.f, 100.f, 140.f, 160.f}) {
			requireStable(simulate(true, angle, 0.f, 1.f, false), false);
		}
		requireStable(simulate(true, 100.f, 0.f, 1.f, true), true);
		requireStable(simulate(true, -100.f, 0.f, 1.f, true), true);
		requireStable(simulate(true, 100.f, 0.f, 1.f, true, nullptr, 0.f, true, true), true, true);
		requireStable(simulate(true, -100.f, 0.f, 1.f, true, nullptr, 0.f, true, true), true, true);
		requireStable(simulate(true, 160.f, 0.f, 1.f, true), true);
		requireStable(simulate(true, -160.f, 0.f, 1.f, true), true);
		requireStable(simulate(true, 160.f, 0.f, 1.f, true, nullptr, 0.f, true, false, true), true);
		puts("PASS: native JSBSim holding and bidirectional transitions through +/-160");
		return 0;
	}

#endif

	if (argc == 6 && strcmp(argv[1], "--tune") == 0) {
		tuning = {strtof(argv[2], nullptr), strtof(argv[3], nullptr), strtof(argv[4], nullptr), strtof(argv[5], nullptr)};
		simulate(true, -90.f, .15f, 1.2f, false, nullptr, 0.f, true);
		simulate(true, -100.f, .1f, 1.f, true, nullptr, 0.f, true);
		simulate(true, 0.f, .15f, 1.2f, false, nullptr, 0.f, true);
		return 0;
	}

	simulate(false, 90.f, .1f, 1.f, false);
	simulate(false, 90.f, .15f, 1.f, false);

	for (float angle : {0.f, -160.f, -140.f, -100.f, -90.f, -85.f, -60.f, 60.f, 85.f, 90.f, 100.f, 140.f, 160.f}) {
		requireStable(simulate(true, angle, .1f, 1.f, false), false);
	}
	requireStable(simulate(true, 100.f, .1f, 1.f, true), true);
	requireStable(simulate(true, -100.f, .1f, 1.f, true), true);

	for (float tau : {.05f, .15f}) {
		for (float inertia : {.8f, 1.2f}) {
			requireStable(simulate(true, 90.f, tau, inertia, false), false);
			requireStable(simulate(true, -90.f, tau, inertia, false), false);
		}
	}

	if (argc > 1) {
		FILE *csv = fopen(argv[1], "w");
		assert(csv);
		fprintf(csv, "t,bank,error,y,r,r_sp,servo,pitch,pitch_sp,q,q_sp,tail_cmd,my_shortfall,iy\n");
		simulate(true, -90.f, .15f, 1.2f, false, csv, .1f, true);
		fclose(csv);
	}

	requireStable(simulate(true, -90.f, .15f, 1.2f, false, nullptr, .1f, true), false);
	requireStable(simulate(true, -100.f, .1f, 1.f, true, nullptr, .1f, true), true);
	requireStable(simulate(true, 100.f, .1f, 1.f, true, nullptr, 0.f, true, true), true, true);
	requireStable(simulate(true, -100.f, .1f, 1.f, true, nullptr, 0.f, true, true), true, true);
	requireStable(simulate(true, 160.f, .1f, 1.f, true, nullptr, 0.f, true), true);
	requireStable(simulate(true, -160.f, .1f, 1.f, true, nullptr, 0.f, true), true);
	requireStable(simulate(true, 160.f, .1f, 1.f, true, nullptr, 0.f, true, false, true), true);
	puts("PASS: multirate dynamics, motor lag, inertia mismatch, tail guard and asymmetric reverse thrust through +/-160");
}
