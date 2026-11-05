#include "../VectorPosition/VectorPosition.hpp"
#include "../VectorAttitudeSwitches.hpp"
#include "../../mc_pos_control/PositionControl/PositionControl.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../mc_att_control/AttitudeControl/AttitudeControl.hpp"
#include "../../vector_rate_control/RateControl/rate_control.hpp"
#include <lib/rate_control/rate_control.hpp>
#include <lib/vector_control/VectorGeometry.hpp>
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <geo/geo.h>
using namespace matrix;
static void near(const Vector3f &a, const Vector3f &b, float eps = 1e-5f) { assert(a.isAllFinite() && b.isAllFinite()); assert((a - b).norm() < eps); }
template<class C> void setup(C &c)
{
	c.setPositionGains(Vector3f(1.f, 1.f, 1.f)); c.setVelocityGains(Vector3f(2.f, 2.f, 2.f), Vector3f(.4f, .4f, .4f), Vector3f(.1f, .1f, .1f));
	c.setVelocityLimits(5.f, 2.f, 2.f); c.setThrustLimits(.05f, .9f); c.setHorizontalThrustMargin(.3f); c.setHoverThrust(.35f);
	c.setTiltLimit(.6f);
}

static void test_voliro_aux_enable_and_level()
{
	// Independent AUX helper and reference regression; fixed mode disables this path at the caller.
	float channels[18] {};
	channels[7] = -1.f;
	channels[8] = 1.f;
	const float limit = math::radians(120.f);
	auto switches = vector_control::attitudeSwitches(9, 8, true, channels, 18, 18);
	for (float aux : {-.812f, -.5f, 0.f, .5f, .868f}) {
		assert(fabsf(vector_control::voliroIndependentPitchTarget(true, switches, 9, aux, limit) - aux * limit) < 1e-6f);
	}
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, 2.f, limit) == limit);
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, -2.f, limit) == -limit);
	assert(vector_control::voliroIndependentPitchTarget(false, switches, 9, .5f, limit) == 0.f);
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, NAN, limit) == 0.f);
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, .5f, NAN) == 0.f);

	for (float enable : {-1.f, 0.f, 1.f}) {
		for (float level : {-1.f, 0.f, 1.f}) {
			channels[8] = enable; channels[7] = level;
			switches = vector_control::attitudeSwitches(9, 8, true, channels, 18, 18);
			const float expected = enable > 0.f && level <= 0.f ? .5f * limit : 0.f;
			assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, .5f, limit) == expected);
		}
	}
	channels[8] = 1.f; channels[7] = -1.f;
	for (auto invalid : {vector_control::attitudeSwitches(9, 8, false, channels, 18, 18),
			     vector_control::attitudeSwitches(9, 8, true, channels, 8, 18)}) {
		assert(vector_control::voliroIndependentPitchTarget(true, invalid, 9, .5f, limit) == 0.f);
	}
	channels[7] = NAN;
	switches = vector_control::attitudeSwitches(9, 8, true, channels, 18, 18);
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 9, .5f, limit) == 0.f);
	channels[7] = -1.f;
	switches = vector_control::attitudeSwitches(0, 8, true, channels, 18, 18);
	assert(vector_control::voliroIndependentPitchTarget(true, switches, 0, .5f, limit) == .5f * limit);

	// Both reference paths must slew through AUX -> fixed -> AUX ->
	// level override -> release -> RC loss, with AUX1 still away from center.
	for (bool dynamics : {false, true}) {
		vector_control::AttitudeReference reference;
		vector_control::AttitudeReference::Settings settings;
		settings.pitch_limit = limit; settings.pitch_rate = math::radians(30.f);
		assert(reference.initialize(Quatf(), settings.bank_limit));
		float pitch = 0.f;
		for (int k = 0; k < 2400; ++k) {
			const int stage = k / 400;
			const float aux = stage < 2 ? .5f : -.5f;
			channels[8] = stage == 1 ? -1.f : 1.f;
			channels[7] = stage == 3 ? 1.f : -1.f;
			switches = vector_control::attitudeSwitches(9, 8, stage != 5, channels, 18, 18);
			const float target = vector_control::voliroIndependentPitchTarget(true, switches, 9, aux, limit);
			const float previous = pitch;
			if (dynamics) {
				vector_control::AttitudeReference::Output out;
				assert(reference.update(Vector3f(0.f, 0.f, -.3f), 0.f, 0.f, target, 1.f, .01f, settings, out));
				pitch = out.pitch;
			} else {
				pitch += math::constrain(target - pitch, -settings.pitch_rate * .01f, settings.pitch_rate * .01f);
			}
			assert(fabsf(pitch - previous) <= settings.pitch_rate * .01f + 1e-5f);
			if (k % 400 == 399) { assert(fabsf(pitch - target) < math::radians(1.f)); }
		}
	}
	puts("PASS: switch-on vector AUX / switch-off fixed selection, AUX limits, level priority/release, unassigned switch, RC loss and both reference paths");
}

static void test_fixed_tilt_selection()
{
	float channels[2] = {-1.f, -1.f};
	auto switches = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(vector_control::voliroFixedTiltRequested(1, switches, true, true, false, false));
	assert(!vector_control::voliroFixedTiltRequested(0, switches, true, true, false, false));
	assert(!vector_control::voliroFixedTiltRequested(1, switches, false, true, false, false));
	assert(!vector_control::voliroFixedTiltRequested(1, switches, true, false, false, false));
	assert(!vector_control::voliroFixedTiltRequested(1, switches, true, true, true, false));
	assert(!vector_control::voliroFixedTiltRequested(1, switches, true, true, false, true));
	channels[1] = 1.f;
	switches = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(vector_control::voliroFixedTiltRequested(1, switches, true, true, false, false)); // LEVEL cannot re-enable the servo
	channels[1] = -1.f; channels[0] = 1.f;
	switches = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(!vector_control::voliroFixedTiltRequested(1, switches, true, true, false, false));
	channels[0] = 1.f;
	switches = vector_control::attitudeSwitches(1, 2, false, channels, 2, 2);
	assert(!vector_control::voliroFixedTiltRequested(1, switches, true, true, false, false));
	puts("PASS: manual fixed-servo selection, unmapped/RC loss/level/armed/control/failsafe/fault gating");
}

int main()
{
	test_fixed_tilt_selection();
	test_voliro_aux_enable_and_level();
// Verify mapping, channel bounds, leveling priority and loss behavior.
	float channels[] = {1.f, -1.f};
	auto sw = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(sw.valid && sw.enabled && !sw.level);
	assert(vector_control::independentAttitudeTarget(sw.valid, sw.enabled, sw.level, .4f) == .4f);
	channels[0] = -1.f;
	sw = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(sw.valid && !sw.enabled && !sw.level);
	assert(vector_control::independentAttitudeTarget(sw.valid, sw.enabled, sw.level, .4f) == 0.f);
	channels[1] = 1.f;
	sw = vector_control::attitudeSwitches(1, 2, true, channels, 2, 2);
	assert(sw.level && vector_control::independentAttitudeTarget(sw.valid, sw.enabled, sw.level, .4f) == 0.f);
	assert(!vector_control::attitudeSwitches(1, 2, false, channels, 2, 2).valid);
	assert(!vector_control::attitudeSwitches(3, 0, true, channels, 3, 2).valid);
	assert(vector_control::attitudeSwitches(0, 0, false, channels, 0, 2).enabled);
// Before geometry adaptation, the copied controllers preserve the complete MC numerical behavior.
	PositionControl original; VectorPosition copy; setup(original); setup(copy);

	for (int i = 0; i < 1000; i++) {
		const float a = .01f * i; PositionControlStates state{Vector3f(sinf(a), cosf(a), -.2f * a), Vector3f(.2f, .1f, -.2f), Vector3f(.1f, 0.f, 0.f), .3f};
		VectorPositionStates vstate{state.position, state.velocity, state.acceleration, state.yaw};
		trajectory_setpoint_s sp = PositionControl::empty_trajectory_setpoint;
		sp.position[0] = 1.f; sp.position[1] = 2.f; sp.position[2] = -3.f; sp.yaw = .4f; sp.yawspeed = .1f;
		original.setState(state); copy.setState(vstate); original.setInputSetpoint(sp); copy.setInputSetpoint(sp);
		assert(original.update(.01f)); assert(copy.update(.01f));
		vehicle_local_position_setpoint_s a_sp{}, b_sp{}; original.getLocalPositionSetpoint(a_sp); copy.getLocalPositionSetpoint(b_sp);
		near(Vector3f(a_sp.thrust), Vector3f(b_sp.thrust)); near(Vector3f(a_sp.acceleration), Vector3f(b_sp.acceleration));
	}

	AttitudeControl attitude; VectorAttitude vatt;
	attitude.setProportionalGain(Vector3f(4.f, 4.f, 2.f), .4f); vatt.setProportionalGain(Vector3f(4.f, 4.f, 2.f), .4f);
	attitude.setRateLimit(Vector3f(4.f, 4.f, 3.f)); vatt.setRateLimit(Vector3f(4.f, 4.f, 3.f));

	for (int i = 0; i < 200; i++) {
		const Quatf q(Eulerf(.1f * i, .05f * i, .02f * i)), qd(Eulerf(.02f * i, -.03f * i, .1f * i));
		attitude.setAttitudeSetpoint(qd, .3f); vatt.setAttitudeSetpoint(qd, .3f); near(attitude.update(q), vatt.update(q));
	}

	vatt.setFullAttitude(true); vatt.setProportionalGain(Vector3f(4.f, 4.f, 2.f), 1.f);
	const Quatf inverted(Eulerf(0.f, M_PI_F, 0.f)); vatt.setAttitudeSetpoint(inverted, 0.f); assert(vatt.update(Quatf()).norm() > .1f);
	const Vector3f r = vatt.update(Quatf()); vatt.setAttitudeSetpoint(-inverted, 0.f); near(r, vatt.update(Quatf()));
	RateControl rate; VectorRateCore vrate;
	rate.setPidGains(Vector3f(.2f, .2f, .2f), Vector3f(.1f, .1f, .1f), Vector3f(.01f, .01f, .01f));
	vrate.setPidGains(Vector3f(.2f, .2f, .2f), Vector3f(.1f, .1f, .1f), Vector3f(.01f, .01f, .01f));
	rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f)); vrate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));

	for (int i = 0; i < 1000; i++) {const Vector3f w(.01f * i, -.002f * i, .003f * i), sp(.3f, -.4f, .1f), acc(.1f, .2f, .3f); near(rate.update(w, sp, acc, .0025f, i < 20), vrate.update(w, sp, acc, .0025f, i < 20));}

// Same NED force remains invariant while the aircraft rotates through large angles.
	const Vector3f force(.12f, .08f, -.35f);

	for (int i = 0; i < 360; i++) {const Quatf q(Eulerf(.01f * i, .02f * i, .03f * i)); near(Dcmf(q)*vector_control::toBody(force, q), force);}

	auto mq = vector_control::attitudeForForce(force, .3f, 16, .6f); assert(fabsf(Eulerf(mq).phi()) < 1e-5f);
	assert(fabsf(Eulerf(mq).theta()) < 1e-5f);
	// Force-derived pitch points body -Z along the requested force at any heading.
	for (float heading : {-.8f, 0.f, .8f}) {
		for (float forward : {-.12f, 0.f, .12f}) {
			const Vector3f request(cosf(heading) * forward - sinf(heading) * .08f,
					       sinf(heading) * forward + cosf(heading) * .08f, -.35f);
			const float pitch = vector_control::pitchForForce(request, heading, .6f);
			const Quatf attitude = vector_control::attitudeForForce(request, heading, 17, .6f, pitch);
			const Vector3f body_force = vector_control::toBody(request, attitude);
			assert(fabsf(body_force(0)) < 1e-5f && fabsf(body_force(1)) < 1e-5f);
			if (forward > 0.f) { assert(pitch < 0.f); }
			if (forward < 0.f) { assert(pitch > 0.f); }
		}
	}
	assert(fabsf(vector_control::pitchForForce(Vector3f(1.f, 0.f, -.1f), 0.f, .2f) + .2f) < 1e-5f);
	assert(vector_control::pitchForForce(Vector3f(1.f, 0.f, 0.f), 0.f, .2f) == 0.f);
	auto vt = vector_control::attitudeForForce(force, .3f, 17, .6f); assert(fabsf(vector_control::toBody(force, vt)(1)) < 1e-5f);
	for (float yaw : {-.8f, 0.f, .8f}) {
		for (float pitch : {math::radians(-10.f), 0.f, math::radians(10.f)}) {
			const Quatf q = vector_control::attitudeForForce(force, yaw, 17, .6f, pitch);
			assert(fabsf(vector_control::toBody(force, q)(1)) < 1e-5f);
		}
	}
// Sweep independent pitch through both vertical orientations with lateral force.
// Compare rotations, not Euler angles (Euler yaw/roll are undefined at 90 deg).
	for (float yaw : {-.8f, 0.f, .8f}) {
		Quatf previous;
		for (int degrees = -160; degrees <= 160; ++degrees) {
			const Quatf q = vector_control::attitudeForForce(force, yaw, 17, .6f, math::radians(float(degrees)));
			assert(q.isAllFinite());
			assert(fabsf(vector_control::toBody(force, q)(1)) < 1e-5f);
			if (degrees > -160) { assert((previous.inversed() * q).canonical().imag().norm() < .01f); }
			previous = q;
		}
	}
// Reproduce the old force-projection zero crossing from the negative-pitch log.
	Quatf previous_force;
	for (int i = 0; i <= 100; ++i) {
		const Vector3f crossing_force(-.15f - .0003f * i, -.0002f, -.25f + .0003f * i);
		const Quatf q = vector_control::attitudeForForce(crossing_force, 0.f, 17, 1.55f, math::radians(-55.7f));
		assert(fabsf(vector_control::toBody(crossing_force, q)(1)) < 1e-5f);
		if (i > 0) { assert((previous_force.inversed() * q).canonical().imag().norm() < .001f); }
		previous_force = q;
	}
	const Vector3f bank_force(0.f, .3f, -.3f);
	const Quatf bank_limited = vector_control::attitudeForForce(bank_force, 0.f, 17, math::radians(15.f));
	assert(fabsf(Eulerf(bank_limited).phi() - math::radians(15.f)) < 1e-5f);
// Voliro T allocation respects force/torque signs, bounds and the reversible tail.
	const voliro_t::Geometry g{.03f, .45f, 0.f, .75f, 39.5f, 10.f, .02f};
	const auto result = voliro_t::allocate(g, Vector3f(0.f, 0.f, -49.05f), Vector3f{}, 0.f, 2.f, 2.f);
	assert((result.force - Vector3f(0.f, 0.f, -49.05f)).norm() < .05f); assert(result.torque.norm() < .05f);
	for (float pitch : {math::radians(-10.f), math::radians(10.f)}) {
		const Quatf pitched(Eulerf(0.f, pitch, 0.f));
		const Vector3f hover_body = vector_control::toBody(Vector3f(0.f, 0.f, -49.05f), pitched);
		const auto pitched_hover = voliro_t::allocate(g, hover_body, Vector3f{}, 0.f, 2.f, 2.f);
		near(pitched_hover.force, hover_body, 1e-4f);
		near(pitched_hover.torque, Vector3f{}, 1e-4f);
	}

// Downstream actuator shortfall must stop position-integral growth in the unavailable direction.
	VectorPosition limited_position; setup(limited_position); limited_position.setVectorAirframe(16);
	limited_position.setAllocationShortfall(Vector3f(.1f, 0.f, 0.f));
	auto speed_sp = VectorPosition::empty_trajectory_setpoint; Vector3f(1.f, 0.f, 0.f).copyTo(speed_sp.velocity); speed_sp.yaw = 0.f;

	for (int k = 0; k < 1000; k++) {
		limited_position.setState(VectorPositionStates{Vector3f{}, Vector3f{}, Vector3f{}, 0.f});
		limited_position.setInputSetpoint(speed_sp); assert(limited_position.update(.01f));
	}

	vehicle_local_position_setpoint_s limited_out{}; limited_position.getLocalPositionSetpoint(limited_out);
	assert(fabsf(limited_out.acceleration[0] - 2.f) < .01f);
// Close the copied attitude/rate loops through the actual Voliro T allocator and rigid-body dynamics.
	VectorAttitude closed_att; closed_att.setFullAttitude(true);
	closed_att.setProportionalGain(Vector3f(4.f, 4.f, 2.8f), 1.f);
	closed_att.setRateLimit(Vector3f(3.f, 3.f, 2.f)); closed_att.setAttitudeSetpoint(Quatf(), 0.f);
	VectorRateCore closed_rate;
	closed_rate.setPidGains(Vector3f(.15f, .15f, .2f), Vector3f(.2f, .2f, .1f), Vector3f(.003f, .003f, 0.f));
	closed_rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
	const Vector3f inertia(.1f, .1f, .2f);
	Quatf orientation(Eulerf(.15f, -.1f, .2f)); Vector3f omega{}, derivative{}; float tilt = 0.f;

	for (int k = 0; k < 8000; k++) {
		const Vector3f torque = closed_rate.update(omega, closed_att.update(orientation), derivative, .0025f, false) * 10.f;
		auto allocation = voliro_t::allocate(g, Vector3f(0.f, 0.f, -49.05f), torque, tilt, 2.f, .003927f); tilt = allocation.tilt;
		const Vector3f residual = torque - allocation.torque;
		closed_rate.setSaturationStatus(Vector3<bool>(residual(0) > 1e-4f, residual(1) > 1e-4f, residual(2) > 1e-4f),
						Vector3<bool>(residual(0) < -1e-4f, residual(1) < -1e-4f, residual(2) < -1e-4f));
		derivative = (allocation.torque - omega.cross(inertia.emult(omega))).edivide(inertia);
		omega += derivative * .0025f; orientation = orientation * Quatf(AxisAnglef(omega * .0025f)); orientation.normalize();
		assert(omega.isAllFinite() && omega.norm() < 20.f);
	}

	assert(orientation.canonical().imag().norm() < .02f); assert(omega.norm() < .05f);

// Small attitude/rate disturbances at and across vertical pitch. The actual
// allocator transports roll/yaw demand between thrust difference and reaction
// torque; include finite servo rate and its residual in rate anti-windup.
	for (float degrees : {-100.f, -90.f, -80.f, 80.f, 90.f, 100.f}) {
		VectorAttitude att; att.setFullAttitude(true);
		att.setProportionalGain(Vector3f(4.f, 4.f, 2.8f), 1.f);
		att.setRateLimit(Vector3f(2.f, 2.f, 2.f));
		VectorRateCore rate90;
		rate90.setPidGains(Vector3f(.15f, .15f, .2f), Vector3f(.2f, .2f, .1f), Vector3f(.003f, .003f, 0.f));
		rate90.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
		const Vector3f world_force(0.f, 0.f, -49.05f);
		const Quatf target = vector_control::attitudeForForce(world_force, .3f, 17, .6f, math::radians(degrees));
		att.setAttitudeSetpoint(target, 0.f);
		Quatf q = target * Quatf(Eulerf(.03f, -.02f, .03f));
		Vector3f w(.02f, -.02f, .02f), acc{};
		float servo = math::radians(degrees);
		for (int k = 0; k < 8000; ++k) {
			const Vector3f demand = rate90.update(w, att.update(q), acc, .0025f, false) * 10.f;
			const auto a = voliro_t::allocate(g, vector_control::toBody(world_force, q), demand, servo, 2.0944f, .003927f);
			servo = a.tilt;
			const Vector3f residual = demand - a.torque;
			rate90.setSaturationStatus(Vector3<bool>(residual(0) > 1e-4f, residual(1) > 1e-4f, residual(2) > 1e-4f),
				Vector3<bool>(residual(0) < -1e-4f, residual(1) < -1e-4f, residual(2) < -1e-4f));
			acc = (a.torque - w.cross(inertia.emult(w))).edivide(inertia);
			w += acc * .0025f; q = q * Quatf(AxisAnglef(w * .0025f)); q.normalize();
			assert(w.isAllFinite() && w.norm() < 5.f);
		}
		assert((q.inversed() * target).canonical().imag().norm() < .01f);
		assert(w.norm() < .05f);
	}

// Transition from level to each vertical orientation, dwell, and return.
	for (float direction : {-1.f, 1.f}) {
		VectorAttitude att; att.setFullAttitude(true);
		att.setProportionalGain(Vector3f(4.f, 4.f, 2.8f), 1.f);
		att.setRateLimit(Vector3f(2.f, 2.f, 2.f));
		VectorRateCore rate90;
		rate90.setPidGains(Vector3f(.15f, .15f, .2f), Vector3f(.2f, .2f, .1f), Vector3f(.003f, .003f, 0.f));
		rate90.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
		Quatf q; Vector3f w{}, acc{}; float servo = 0.f;
		for (int k = 0; k < 8000; ++k) {
			const float t = k * .0025f;
			const float pitch = direction * math::radians(15.f) * math::constrain(math::min(t, 16.f - t), 0.f, 6.f);
			att.setAttitudeSetpoint(vector_control::attitudeForForce(Vector3f(0.f, 0.f, -49.05f), 0.f, 17, .6f, pitch), 0.f);
			const Vector3f demand = rate90.update(w, att.update(q), acc, .0025f, false) * 10.f;
			const auto a = voliro_t::allocate(g, vector_control::toBody(Vector3f(0.f, 0.f, -49.05f), q), demand, servo, 2.0944f, .003927f);
			servo = a.tilt;
			const Vector3f residual = demand - a.torque;
			rate90.setSaturationStatus(Vector3<bool>(residual(0) > 1e-4f, residual(1) > 1e-4f, residual(2) > 1e-4f),
				Vector3<bool>(residual(0) < -1e-4f, residual(1) < -1e-4f, residual(2) < -1e-4f));
			acc = (a.torque - w.cross(inertia.emult(w))).edivide(inertia);
			w += acc * .0025f; q = q * Quatf(AxisAnglef(w * .0025f)); q.normalize();
			assert(w.isAllFinite() && w.norm() < 2.f);
			const Quatf target = vector_control::attitudeForForce(Vector3f(0.f, 0.f, -49.05f), 0.f, 17, .6f, pitch);
			assert((q.inversed() * target).canonical().imag().norm() < .08f);
		}
		assert(q.canonical().imag().norm() < .01f && w.norm() < .05f);
	}

// Execute a simple point-mass waypoint sequence using the actual copied position core for both models.
	for (int model : {16, 17}) {
		VectorPosition controller; setup(controller); controller.setVectorAirframe(model);
		Vector3f pos{}, vel{}, acc{};

		for (const Vector3f target : {Vector3f(0.f, 0.f, -2.f), Vector3f(3.f, 2.f, -2.f), Vector3f(0.f, 0.f, -1.f)}) {
			for (int k = 0; k < 2000; k++) {
				controller.setState(VectorPositionStates{pos, vel, acc, 0.f});
				auto sp = VectorPosition::empty_trajectory_setpoint; target.copyTo(sp.position); sp.yaw = 0.f; controller.setInputSetpoint(sp);
				assert(controller.update(.01f));
				vehicle_local_position_setpoint_s out{}; controller.getLocalPositionSetpoint(out);
				acc = Vector3f(out.thrust) * (CONSTANTS_ONE_G / .35f) + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
				vel += acc * .01f; pos += vel * .01f;
			}

			assert((pos - target).norm() < .1f);
		}
	}
	puts("PASS: MC equivalence, force-plane continuity, +/-90-degree ideal rigid-body recovery/transitions, allocation and point-mass waypoints");
}
