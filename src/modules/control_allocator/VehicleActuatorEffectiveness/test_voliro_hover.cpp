// Reuse transport-only stubs and the real effectiveness wrapper's parameter fixture.
#define main voliro_existing_regressions
#include "test_voliro_t_effectiveness.cpp"
#undef main
#include <lib/vector_control/VectorGeometry.hpp>
#include <lib/vector_control/VectorReferenceGains.hpp>
#include "VoliroTActuatorDynamics.hpp"
#include <modules/vector_pos_control/tests/NativeVoliroPlant.hpp>

#ifndef VOLIRO_HOVER_TAIL_RMAX
#define VOLIRO_HOVER_TAIL_RMAX .6f
#endif
#ifndef VOLIRO_HOVER_REACTION_SCALE
#define VOLIRO_HOVER_REACTION_SCALE .5f
#endif

static bool hover_passed = true;
static void check_hover(bool ok, float pitch, const char *message)
{
	if (!ok) {
		printf("FAIL pitch=%+.0f: %s\n", double(pitch), message);
		hover_passed = false;
	}
}

static float simulate_hover(float pitch_deg, float gain, const char *model, const char *systems)
{
	using namespace matrix;
	dynamics_mode = true; airmode_setting = 0; log_geometry = true; tail_reverse_limit = VOLIRO_HOVER_TAIL_RMAX; clock_step = 0; servo_rate = 200.f;
	servo_angle = 160.f;
	const voliro_t::Geometry g{.02f, .45f, 0.f, .74f, 39.5f, 10.f, .02f, tail_reverse_limit};
	Vector3f force(0.f, 0.f, -33.6f);
	const Quatf target(Eulerf(0.f, math::radians(pitch_deg), 0.f));
	ActuatorEffectivenessVoliroT allocator(nullptr);
	using AV = ActuatorEffectiveness::ActuatorVector; AV lo, hi, output;

	for (int i = 0; i < 16; ++i) {lo(i) = i == 4 || i >= 5 ? -1.f : 0.f; hi(i) = 1.f;}

	Vector<float, 6> request;
	auto set_force = [&](const Quatf & q) {const Vector3f f = vector_control::toBody(force, q) / 168.f; for (int i = 0; i < 3; ++i)request(i + 3) = f(i);};
	set_force(target);
	constexpr float dt = .0025f;

	for (int k = 0; k < 1200; ++k) {test_time += 2500; allocator.updateSetpoint(request, 0, output, lo, hi); allocator.updateSetpointAfterSlew(dt, 0, output, true);}

	voliro_t::Result trim; trim.tilt = output(5) * math::radians(160.f);

	for (int i = 0; i < 5; ++i) { trim.thrust[i] = i == 4 ? voliro_t::tailThrust(g, output(i)) : output(i) * output(i) * g.front_max; }

	VectorAttitude att; att.setFullAttitude(true); att.setProportionalGain(Vector3f(5.5f, 3.f, 2.8f), 1.f);
	att.setRateLimit(Vector3f(2.0944f, 1.0472f, 1.3963f));
	VectorRateCore rate; rate.setPidGains(Vector3f(.26f, .1f, .3f), Vector3f(.2f, .4f, .1f), Vector3f(.003f, .001f, 0.f));
	rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
	Quatf q = target * Quatf(Eulerf(.005f, 0.f, .005f)); Vector3f w{}, acc{}, pos{}, vel{}, vacc{};
	float initial[5];

	for (int i = 0; i < 5; ++i) { initial[i] = output(i); }

	NativeVoliroPlant native(model, systems, q, initial, trim.tilt, dt); float peak = 0.f, energy = 0.f, rate_energy = 0.f;
	int samples = 0, saturated = 0;

	for (int k = 0; k < 12000; ++k) {
		test_time += 2500;
		att.setAttitudeSetpoint(target, 0.f); att.setTaskReference(Vector3f(5.5f, 3.f, 2.8f), 0.f, {}, true, 1.f / 3.f);
		const Vector3f scale(1.f, vector_control::referenceFeedbackScale(q, 1.f / 3.f), 1.f);
		rate.setReactionFeedback(output(5)*math::radians(160.f), gain, Vector3f(10.f, 10.f, 10.f));
		const Vector3f tq = rate.update(w, att.update(q), acc, dt, false, {}, {}, scale);

		for (int i = 0; i < 3; ++i) { request(i) = tq(i); } force(0) = math::constrain(-5.f * vel(0) - pos(0), -15.f, 15.f);
		force(2) = math::constrain(-33.6f - 10.f * vel(2) - 2.f * pos(2), -80.f, -10.f); set_force(q);

		allocator.updateSetpoint(request, 0, output, lo, hi); allocator.updateSetpointAfterSlew(dt, 0, output, true);
		control_allocator_status_s status; allocator.getUnallocatedControl(0, status);
		rate.setSaturationStatus(Vector3<bool>(status.unallocated_torque[0] > 1e-5f, status.unallocated_torque[1] > 1e-5f,
						       status.unallocated_torque[2] > 1e-5f), Vector3<bool>(status.unallocated_torque[0] < -1e-5f, status.unallocated_torque[1] < -1e-5f,
								       status.unallocated_torque[2] < -1e-5f));
		float commands[5];

		for (int i = 0; i < 5; ++i) { commands[i] = output(i); }

		native.step(commands, output(5)*math::radians(160.f), q, w, acc, pos, vel, vacc);
		const float err = AxisAnglef((q.inversed() * target).canonical()).angle(); peak = math::max(peak, err);

		if (k > 10000) {energy += err * err; rate_energy += w.norm_squared(); ++samples; for (int j = 0; j < 4; ++j)if (commands[j] < .001f) {++saturated; break;}}
	}

	printf("gain=%.3f pitch=%+.0f peak=%.3f rms=%.3f rate_rms=%.3f\n", double(gain), double(pitch_deg), double(math::degrees(peak)),
	       double(math::degrees(sqrtf(energy / samples))), double(math::degrees(sqrtf(rate_energy / samples))));

	if (gain < 1.f) {
		check_hover(q.isAllFinite() && w.isAllFinite() && peak < math::radians(10.f), pitch_deg, "finite bounded attitude transient");
		check_hover(math::degrees(sqrtf(rate_energy / samples)) < 1.f, pitch_deg, "late angular rates decay below 1 deg/s RMS");
		check_hover(saturated == 0, pitch_deg, "no repeated front motor zero commands in final five seconds");
	}

	return math::degrees(sqrtf(energy / samples));
}
int main(int argc, char **argv)
{
	check(argc == 3 || (argc == 4 && strcmp(argv[3], "--extended") == 0), "model and systems paths, optionally --extended, required");
	setvbuf(stdout, nullptr, _IOLBF, 0);
	const float normal_angles[] = {-45.f, -30.f, -15.f, 0.f, 15.f, 30.f, 45.f};
	const float extended_angles[] = {-160.f, -150.f, -135.f, -120.f, -90.f, -60.f, -45.f, -30.f, -15.f,
					0.f, 15.f, 30.f, 45.f, 60.f, 90.f, 120.f, 135.f, 150.f, 160.f};
	const bool extended = argc == 4;
	const float *angles = extended ? extended_angles : normal_angles;
	const unsigned count = extended ? sizeof(extended_angles) / sizeof(float) : sizeof(normal_angles) / sizeof(float);

	for (unsigned i = 0; i < count; ++i) {
		const float pitch = angles[i];
		const float before = simulate_hover(pitch, 1.f, argv[1], argv[2]);
		const float after = simulate_hover(pitch, VOLIRO_HOVER_REACTION_SCALE, argv[1], argv[2]);
		check_hover(PX4_ISFINITE(after) && after < .2f, pitch, "shaped hover must settle below 0.2 deg RMS");

		if (pitch == -30.f || pitch == -45.f) { check_hover(before > 1.f && after < before * .1f, pitch, "same native model must reproduce and suppress baseline oscillation"); }
	}
	check(hover_passed, "one or more native hover cases failed; all angles were evaluated");
	printf("PASS native JSBSim hover: real allocator, 2 Hz reaction filter, logged gains, +/-%d degrees\n", extended ? 160 : 45);
	return 0;
}
