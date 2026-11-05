#include "../VectorPosition/VectorPosition.hpp"
#include "../VectorAttitudeSwitches.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTActuatorDynamics.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTTailMotor.hpp"
#include <lib/vector_control/VectorAngularAccelerationControl.hpp>
#include <lib/vector_control/VoliroTiltMonitor.hpp>
#include <lib/vector_control/VoliroFixedTiltTransition.hpp>
#include <geo/geo.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#ifdef VECTOR_TEST_JSBSIM
#include "NativeVoliroPlant.hpp"
#include <memory>
#endif
using namespace matrix;

// Same real control cores and allocation functions as flight code, with a
// multirate rigid body (or network-free native JSBSim) plant. No hardware I/O.
static void simulate(const char *model = nullptr, const char *systems = nullptr,
		     Vector3f inertia = Vector3f(.18f, .12f, .43f),
		     voliro_t::Geometry g = {.09f, .45f, 0.f, .69f, 39.5f, 10.f, .02f})
{
	constexpr float dt = .0025f, mass = 5.04f;
	const float total = 4.f * g.front_max + g.tail_max;
	VectorPosition position;
	position.setVectorAirframe(17); position.setHoverThrust(mass * CONSTANTS_ONE_G / total);
	position.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	position.setVelocityGains(Vector3f(1.2f, 1.2f, 4.f), Vector3f(.4f, .4f, .5f), Vector3f(.2f, .2f, 0.f));
	position.setVelocityLimits(3.f, 2.f, 2.f); position.setThrustLimits(.05f, .9f);
	position.setHorizontalThrustMargin(.3f); position.setTiltLimit(math::radians(25.f));
	position.setAllocationAntiWindupGain(1.f);
	VectorAttitude attitude; attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(4.f, 4.f, 2.f), 1.f); attitude.setRateLimit(Vector3f(2.f, 2.f, 2.f));
	vector_control::AngularAccelerationControl rate;
	rate.configure(Vector3f(8.f, 12.f, 8.f), Vector3f(1.f, .3f, 1.f), Vector3f(.3f, .3f, .3f), 2.f, 2.f);
	voliro_t::TailMotor tail;
	const voliro_t::TailMotor::Settings tail_settings{.4f, .03f, .1f, .04f, .08f};
	voliro_t::ActuatorDynamics plant;
	Vector3f p(0.f, 0.f, -2.f), v{}, a{}, omega{}, alpha{}, shortfall{}, torque_shortfall{};
	Vector3f force(0.f, 0.f, -mass * CONSTANTS_ONE_G);
	Quatf q, target;
	const auto trim = voliro_t::allocateAtTilt(g, force, {}, 0.f);
	const auto trim_output = voliro_t::allocateRollYaw(g, trim, {}, 0.f);
	plant.initialize(trim_output, g);
	float commands[5];
	for (int i = 0; i < 5; ++i) { commands[i] = i == 4 ? voliro_t::tailCommand(g, trim_output.thrust[i]) : sqrtf(trim_output.thrust[i] / g.front_max); }
	for (int i = 0; i < 400; ++i) { tail.update(commands[4], dt, tail_settings); }
#ifdef VECTOR_TEST_JSBSIM
	std::unique_ptr<NativeVoliroPlant> native;
	if (model) { native.reset(new NativeVoliroPlant(model, systems, q, commands, 0.f, dt)); }
#else
	(void)model; (void)systems;
#endif
	float servo = 0.f, servo_actual = 0.f, max_fixed_servo = 0.f, max_vector_servo = 0.f;
	float forward_error = NAN, backward_error = NAN, hover_error = NAN, min_pitch = 0.f, max_pitch = 0.f;
	float max_target_step = 0.f, reentry_tilt = 0.f, max_fixed_actual = 0.f;
	bool fixed = false, transitioning = false;
	vector_control::VoliroFixedTiltTransition handover;
	float aux_pitch = 0.f, aux_before = NAN, aux_after = NAN;
	float desired_x = 0.f;
	for (int k = 0; k < 36000; ++k) {
		const float time = k * dt;
		if (k % 4 == 0) {
			const bool mc_requested = (time >= 3.f && time < 38.f) || (time >= 40.5f && time < 55.f);
			float channels[2] = {mc_requested ? -1.f : 1.f, -1.f};
			const auto switches = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
			const bool requested = vector_control::voliroFixedTiltRequested(1, switches, true, true, false, false);
			assert(requested == mc_requested);
			const bool next_fixed = handover.update(requested,
				vector_control::VoliroFixedTiltTransition::ready(q, target, omega, servo_actual, servo),
				1000000 + uint64_t(k) * 2500);
			if (fixed != next_fixed) { position.resetIntegralXY(); transitioning = true; aux_pitch = 0.f; }
			fixed = next_fixed;
			position.setMulticopterFallback(fixed);
			position.setState(VectorPositionStates{p, v, a, 0.f});
			position.setAllocationShortfall(Vector3f(Dcmf(q) * shortfall) / total);
			desired_x = time < 5.f ? 0.f : time < 15.f ? 2.f : time < 27.f ? -2.f : time < 40.f ? 0.f
				    : time < 57.f ? 2.f : time < 70.f ? -2.f : 0.f;
			auto sp = VectorPosition::empty_trajectory_setpoint;
			Vector3f(desired_x, 0.f, -2.f).copyTo(sp.position); sp.yaw = 0.f;
			position.setInputSetpoint(sp); assert(position.update(4.f * dt));
			vehicle_local_position_setpoint_s out{}; position.getLocalPositionSetpoint(out);
			force = Vector3f(out.thrust) * total;
			const float aux = time >= 1.f && time < 65.f ? .5f : 0.f;
			const float pitch_target = vector_control::voliroIndependentPitchTarget(!fixed, switches, 1, aux, math::radians(20.f));
			aux_pitch += math::constrain(pitch_target - aux_pitch, -math::radians(20.f) * 4.f * dt, math::radians(20.f) * 4.f * dt);
			vehicle_attitude_setpoint_s asp{}; position.getAttitudeSetpoint(asp, aux_pitch);
			Quatf next(asp.q_d);
			if (fixed || transitioning) {
				const float beta = fixed ? vector_control::voliroFallbackThrustAngle(servo_actual, g.x, g.z, g.tail) : 0.f;
				next = next * Quatf(Eulerf(0.f, beta, 0.f));
				const Quatf step = vector_control::slewFallbackAttitude(target, next, math::radians(80.f) * 4.f * dt);
				transitioning = AxisAnglef((step.inversed() * next).canonical()).angle() > 1e-4f;
				next = step;
			}
			max_target_step = math::max(max_target_step, AxisAnglef((target.inversed() * next).canonical()).angle());
			target = next;
			if (fabsf(time - 14.f) < .001f) { forward_error = fabsf(p(0) - 2.f); }
			if (fabsf(time - 26.f) < .001f) { backward_error = fabsf(p(0) + 2.f); }
			if (fabsf(time - 37.f) < .001f) { hover_error = (p - Vector3f(0.f, 0.f, -2.f)).norm(); }
			if (fabsf(time - 40.5f) < .001f) { reentry_tilt = servo; }
			if (fabsf(time - 2.9f) < .001f) { aux_before = Eulerf(q).theta(); }
			if (fabsf(time - 39.8f) < .001f) { aux_after = Eulerf(q).theta(); }
		}
		attitude.setAttitudeSetpoint(target, 0.f);
		const Vector3f torque = inertia.emult(rate.update(attitude.update(q) - omega, alpha, {}, torque_shortfall.edivide(inertia), dt, false))
				       + omega.cross(inertia.emult(omega));
		const Vector3f body_force = Dcmf(q).transpose() * force;
		const float centered = servo + math::constrain(-servo, -math::radians(90.f) * dt, math::radians(90.f) * dt);
		auto allocation = fixed ? voliro_t::allocateAtTilt(g, body_force, torque, centered)
				: voliro_t::allocateForcePitch(g, body_force, torque, servo, math::radians(160.f), math::radians(90.f) * dt, true);
		const float tail_request = voliro_t::tailCommand(g, allocation.tail);
		auto preview = tail;
		const float available_tail = voliro_t::tailThrust(g, preview.update(tail_request, dt, tail_settings));
		if (fabsf(available_tail - allocation.tail) > 1e-4f) {
			allocation = fixed ? voliro_t::refitAtTiltForTail(g, body_force, torque, centered, available_tail)
				     : voliro_t::refitForTail(g, body_force, torque, available_tail, servo, math::radians(160.f), math::radians(90.f) * dt);
		}
		auto result = voliro_t::allocateRollYaw(g, allocation, torque, voliro_t::reactionTorque(allocation.tilt, torque));
		servo = fixed ? centered : result.tilt;
		for (int i = 0; i < 5; ++i) { commands[i] = i == 4 ? voliro_t::tailCommand(g, result.thrust[i]) : sqrtf(result.thrust[i] / g.front_max); }
		commands[4] = tail.update(tail_request, dt, tail_settings);
		result.thrust[4] = voliro_t::tailThrust(g, commands[4]); result.tilt = servo;
		voliro_t::forward(g, result);
		shortfall = body_force - result.force; torque_shortfall = torque - result.torque;
		const auto actual = plant.update(g, commands, servo, .04f, .1f, math::radians(90.f), dt);
		servo_actual = actual.tilt;
#ifdef VECTOR_TEST_JSBSIM
		if (native) { native->step(commands, servo, q, omega, alpha, p, v, a); }
		else
#endif
		{
			alpha = (actual.torque - omega.cross(inertia.emult(omega))).edivide(inertia);
			a = Dcmf(q) * actual.force / mass + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
			omega += alpha * dt; v += a * dt; p += v * dt;
			q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
		}
		assert(p.isAllFinite() && q.isAllFinite() && omega.norm() < 5.f && p.norm() < 30.f);
		if (fixed && ((time > 5.f && time < 38.f) || time > 43.f)) {
			max_fixed_servo = math::max(max_fixed_servo, fabsf(servo));
			float actual_angle = servo_actual;
#ifdef VECTOR_TEST_JSBSIM
			if (native) { actual_angle = native->servoAngle(); }
#endif
			max_fixed_actual = math::max(max_fixed_actual, fabsf(actual_angle));
		}
		if (!fixed && time > 40.f) { max_vector_servo = math::max(max_vector_servo, fabsf(servo)); }
		if (fixed) { min_pitch = math::min(min_pitch, Eulerf(q).theta()); max_pitch = math::max(max_pitch, Eulerf(q).theta()); }
	}
	const float final_error = (p - Vector3f(0.f, 0.f, -2.f)).norm();
	std::printf("Fixed tilt %s: forward/back/hold/final error=%.4f/%.4f/%.4f/%.4fm, fixed command/actual=%.6f/%.6fdeg, vector servo=%.2fdeg, reentry tilt=%.2fdeg, pitch=%.2f..%.2fdeg, max target step=%.3fdeg\n",
		model ? "JSBSim" : "rigid-body", (double)forward_error, (double)backward_error, (double)hover_error, (double)final_error,
		(double)math::degrees(max_fixed_servo), (double)math::degrees(max_fixed_actual), (double)math::degrees(max_vector_servo), (double)math::degrees(reentry_tilt),
		(double)math::degrees(min_pitch), (double)math::degrees(max_pitch), (double)math::degrees(max_target_step));
	std::fflush(stdout);
	std::printf("AUX before/after fixed mode: %.3f/%.3fdeg (target +10deg, no parameter changes)\n", (double)math::degrees(aux_before), (double)math::degrees(aux_after));
	std::fflush(stdout);
	assert(fabsf(aux_before - math::radians(10.f)) < math::radians(1.f));
	assert(fabsf(aux_after - math::radians(10.f)) < math::radians(1.f));
	assert(forward_error < .2f && backward_error < .2f && hover_error < .2f && final_error < .2f);
	assert(max_fixed_actual < math::radians(.01f));
	assert(max_fixed_servo < 1e-6f && max_vector_servo > math::radians(.5f) && fabsf(reentry_tilt) > math::radians(.5f));
	assert(min_pitch < math::radians(-1.f) && max_pitch > math::radians(1.f));
	assert(max_target_step < math::radians(2.f));
}
int main(int argc, char **argv)
{
	if (argc == 11 && !std::strcmp(argv[1], "--jsbsim")) {
		const Vector3f inertia(atof(argv[4]), atof(argv[5]), atof(argv[6]));
		const voliro_t::Geometry g{float(atof(argv[7])), float(atof(argv[8])), float(atof(argv[9])), float(atof(argv[10])), 39.5f, 10.f, .02f};
		simulate(argv[2], argv[3], inertia, g);
	} else { simulate(); }
}
