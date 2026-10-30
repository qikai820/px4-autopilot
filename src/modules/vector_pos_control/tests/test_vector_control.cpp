#include "../VectorPosition/VectorPosition.hpp"
#include "../../mc_pos_control/PositionControl/PositionControl.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../mc_att_control/AttitudeControl/AttitudeControl.hpp"
#include "../../vector_rate_control/RateControl/rate_control.hpp"
#include <lib/rate_control/rate_control.hpp>
#include <lib/vector_control/VectorGeometry.hpp>
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
int main()
{
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
	auto vt = vector_control::attitudeForForce(force, .3f, 17, .6f); assert(fabsf(vector_control::toBody(force, vt)(1)) < 1e-5f);
	for (float yaw : {-.8f, 0.f, .8f}) {
		for (float pitch : {math::radians(-10.f), 0.f, math::radians(10.f)}) {
			const Quatf q = vector_control::attitudeForForce(force, yaw, 17, .6f, pitch);
			assert(fabsf(Eulerf(q).theta() - pitch) < 1e-5f);
			assert(fabsf(vector_control::toBody(force, q)(1)) < 1e-5f);
		}
	}
	const Quatf limited_pitch = vector_control::attitudeForForce(force, 0.f, 17,
								 math::radians(15.f), math::radians(10.f));
	assert(acosf(Dcmf(limited_pitch)(2, 2)) < math::radians(15.f) + 1e-5f);
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
	puts("PASS: MC numerical equivalence, quaternion continuity, force frames, Voliro T allocation/rigid-body loop and waypoint tracking (point-mass model)");
}
