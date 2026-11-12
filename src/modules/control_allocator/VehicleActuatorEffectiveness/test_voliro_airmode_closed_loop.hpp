#pragma once

#include <modules/vector_rate_control/RateControl/rate_control.hpp>
#include <modules/vector_att_control/VectorAttitude/VectorAttitude.hpp>
#include <lib/vector_control/VectorReferenceGains.hpp>
#include <cmath>
#ifdef VOLIRO_AIRMODE_JSBSIM
#include <modules/vector_pos_control/tests/NativeVoliroPlant.hpp>
#include <memory>
#endif

struct AirmodeMetrics {
	float peak_deg{0.f}, final_rms_deg{0.f}, peak_rate_deg_s{0.f}, peak_front_n{0.f};
	int reversals{0};
	bool finite{true};
};

// Manual attitude -> production rate PID -> real effectiveness wrapper ->
// first-order motor + rigid-body pitch plant. The native variant instead runs
// the unchanged repository JSBSim motor/propeller/airframe model without IO.
static AirmodeMetrics run_manual_airmode_loop(int setting, bool dynamic, float motor_tau,
		float inertia_scale = 1.f, FILE *trace = nullptr, const char *native_model = nullptr,
		const char *native_systems = nullptr, float low_collective = .04f)
{
	using namespace matrix;
	constexpr float dt = .0025f;
	clock_step = 0;
	log_geometry = true; dynamics_mode = dynamic; airmode_setting = setting;
	control_mode_available = refresh_control_mode = true; land_available = true; refresh_land = true; input_land = {}; tilt_status_available =
		false; monitor_mode = false;
	input_control_mode = {};
	input_control_mode.flag_armed = input_control_mode.flag_control_manual_enabled = input_control_mode.flag_control_rates_enabled = true;
	ActuatorEffectivenessVoliroT effectiveness(nullptr);
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = minimum(5) = -1.f;
	effectiveness.updateSetpointAfterSlew(dt, 0, output, true);
	VectorRateCore rate;
	rate.setPidGains(Vector3f(.26f, .1f, .3f), Vector3f(.2f, .4f, .1f), Vector3f(.003f, .001f, 0.f));
	rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f)); rate.setFeedForwardGain({});
	VectorAttitude attitude;
	attitude.setFullAttitude(true); attitude.setProportionalGain(Vector3f(5.5f, 3.f, 2.8f), 1.f);
	attitude.setRateLimit(Vector3f(math::radians(120.f), math::radians(60.f), math::radians(80.f)));
	const voliro_t::Geometry g{.02f, .45f, 0.f, .74f, 39.5f, 10.f, .02f, .6f};
	voliro_t::ActuatorDynamics plant;
	const auto trim = voliro_t::allocate(g, Vector3f(0.f, 0.f, -33.6f), {}, 0.f, 2.8f, .01f);
	plant.initialize(trim, g);
	Quatf q;
	Vector3f omega{}, alpha{}, rate_sp{}, residual{};
	Vector3f position{}, velocity{}, acceleration{};
#ifdef VOLIRO_AIRMODE_JSBSIM
	std::unique_ptr<NativeVoliroPlant> native;

	if (native_model) {
		float commands[5];

		for (int i = 0; i < 4; ++i) { commands[i] = sqrtf(trim.thrust[i] / g.front_max); }

		commands[4] = voliro_t::tailCommand(g, trim.thrust[4]);
		native.reset(new NativeVoliroPlant(native_model, native_systems, q, commands, 0.f, dt));
	}

#endif
	AirmodeMetrics metrics;
	double energy = 0.; unsigned final_samples = 0;
	int last_direction = 0;

	for (int k = 0; k < 10000; ++k) {
		test_time += 2500;
		const float time = k * dt;
		const float target = time >= 5.f && time < 8.f ? 15.f : time >= 11.f && time < 14.f ? -15.f : 0.f;
		const Quatf qd(Eulerf(0.f, math::radians(target), 0.f));

		if (k % 2 == 0) {
			attitude.setAttitudeSetpoint(qd, 0.f);

			if (dynamic) { attitude.setTaskReference(Vector3f(5.5f, 3.f, 2.8f), 0.f, {}, false, 1.f / 3.f); }

			rate_sp = attitude.update(q);
		}

		Vector<bool, 3> positive, negative;

		for (int i = 0; i < 3; ++i) { positive(i) = residual(i) > FLT_EPSILON; negative(i) = residual(i) < -FLT_EPSILON; }

		rate.setSaturationStatus(positive, negative);
		Vector3f integral_scale(1.f, dynamic ? vector_control::referenceFeedbackScale(q, 1.f / 3.f) : 1.f, 1.f);

		if (setting) {
			integral_scale(1) = math::min(integral_scale(1), vector_control::voliroAirmodeIntegralScale(time < 3.f ? .2f : low_collective, .2f));
		}

		const Vector3f torque = 10.f * rate.update(omega, rate_sp, alpha, dt, false, {}, {}, integral_scale);
		matrix::Vector<float, 6> demand;

		for (int i = 0; i < 3; ++i) { demand(i) = torque(i) / 10.f; }

		demand(5) = time < 3.f ? -.2f : -low_collective;
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(dt, 0, output, true);
		control_allocator_status_s status{}; effectiveness.getUnallocatedControl(0, status);
		residual = Vector3f(status.unallocated_torque) * 10.f;
		float commands[5];

		for (int i = 0; i < 5; ++i) { commands[i] = output(i); }

		const auto actual = plant.update(g, commands, 0.f, motor_tau, 0.f, 3.f, dt);
#ifdef VOLIRO_AIRMODE_JSBSIM

		if (native) { native->step(commands, 0.f, q, omega, alpha, position, velocity, acceleration); }

		else
#endif
		{
			const float disturbance = time >= 3.5f && time < 3.65f ? .12f : 0.f;
			alpha = Vector3f(0.f, (actual.torque(1) + disturbance) / (.12f * inertia_scale), 0.f);
			omega += alpha * dt; q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
		}

		const Dcmf rotation(q);
		const float pitch = atan2f(-rotation(2, 0), rotation(2, 2));
		const float error = matrix::wrap_pi(pitch - math::radians(target));
		float front_command = 0.f;

		for (int i = 0; i < 4; ++i) { front_command += commands[i] * commands[i] * g.front_max; }

		if (setting) {
			const float collective = time < 3.f ? .2f : low_collective;
			const float command_force = front_command + voliro_t::tailThrust(g, commands[4]);
			check(command_force <= 168.f * (collective + math::min(.02f, .25f * collective)) + 1e-3f,
			      "attitude closed loop must satisfy lift authority as well as angle convergence");
		}

		if (time >= 3.f) {
			metrics.peak_deg = math::max(metrics.peak_deg, fabsf(math::degrees(error)));
			metrics.peak_rate_deg_s = math::max(metrics.peak_rate_deg_s, fabsf(math::degrees(omega(1))));
			metrics.peak_front_n = math::max(metrics.peak_front_n, front_command);
			const int direction = commands[4] > .03f ? 1 : commands[4] < -.03f ? -1 : 0;

			if (last_direction && direction && last_direction != direction) { ++metrics.reversals; }

			if (direction) { last_direction = direction; }
		}

		if (time >= 20.f) { energy += error * error; ++final_samples; }

		if (trace && k % 4 == 0) {
			fprintf(trace, "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", (double)time,
				(double)target, (double)math::degrees(pitch), (double)math::degrees(omega(1)), (double)commands[4],
				(double)torque(1), (double)residual(1), (double) - demand(5), (double)front_command);
		}

		if (!q.isAllFinite() || !omega.isAllFinite() || omega.norm() > 100.f) { metrics.finite = false; break; }
	}

	metrics.final_rms_deg = final_samples ? math::degrees(sqrtf(energy / final_samples)) : INFINITY;
	printf("manual AirMode=%d VT=%d tau=%.3f inertia=%.2f native=%d finite=%d peak=%.3fdeg final=%.3fdeg rate=%.3fdeg/s reversals=%d front=%.3fN low_collective=%.3f\n",
	       setting, dynamic, (double)motor_tau, (double)inertia_scale, native_model != nullptr, metrics.finite,
	       (double)metrics.peak_deg, (double)metrics.final_rms_deg, (double)metrics.peak_rate_deg_s, metrics.reversals, (double)metrics.peak_front_n,
	       (double)low_collective);
	return metrics;
}

static void test_manual_airmode_closed_loop()
{
	for (bool dynamic : {false, true}) {
		for (float tau : {.05f, .1f, .15f}) {
			for (float inertia : {.75f, 1.f, 1.5f}) {
				const auto before = run_manual_airmode_loop(0, dynamic, tau, inertia);
				const auto after = run_manual_airmode_loop(1, dynamic, tau, inertia);
				(void)before;
				check(after.finite && after.final_rms_deg < 1.f && after.peak_deg < 25.f,
				      "AirMode manual low-throttle loop converges with motor lag/inertia uncertainty");
			}
		}
	}
	clock_step = 10000; airmode_setting = 0; log_geometry = false; control_mode_available = false; dynamics_mode = false;
}
