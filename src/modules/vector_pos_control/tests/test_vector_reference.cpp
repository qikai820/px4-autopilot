#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <lib/vector_control/VectorAngularAccelerationControl.hpp>
#include <lib/vector_control/VectorReferenceValidity.hpp>
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include "../../control_allocator/VehicleActuatorEffectiveness/VoliroTAllocation.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;
using namespace vector_control;

int main()
{
	// Logical heading does not flip when independent pitch crosses vertical.
	for (float heading : {-3.13f, -.3f, .8f, 3.13f}) {
		for (int degrees = -120; degrees <= 120; ++degrees) {
			const Quatf q = attitudeForForce(Vector3f(0.f, .03f, -.3f), heading, vector_control::voliro_t, .35f, math::radians(float(degrees)));
			assert(fabsf(wrap_pi(forcePlaneHeading(q) - heading)) < 1e-5f);
			assert(fabsf(wrap_pi(forcePlaneHeading(-q) - heading)) < 1e-5f);
		}
	}
	assert(!PX4_ISFINITE(forcePlaneHeading(Quatf(0.f, 0.f, 0.f, 0.f))));
	assert(!PX4_ISFINITE(forcePlaneHeading(Quatf(Eulerf(M_PI_2_F, 0.f, 0.f)))));

	// A bank correction becomes +body Z, and a world heading correction -body X
	// at +90 pitch. Full quaternion feedback remains invariant under q -> -q.
	VectorAttitude attitude;
	attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(5.f, 3.f, 1.f), 1.f);
	attitude.setRateLimit(Vector3f(3.f, 3.f, 3.f));
	const Quatf vertical(Eulerf(0.f, M_PI_2_F, 0.f));
	attitude.setAttitudeSetpoint(Quatf(Eulerf(.01f, 0.f, 0.f)) * vertical, 0.f);
	attitude.setTaskReference(Vector3f(2.f, 2.f, 1.f), 0.f, {});
	assert(fabsf(attitude.update(vertical)(2) - .02f) < 1e-5f);
	assert((attitude.update(vertical) - attitude.update(-vertical)).norm() < 1e-6f);
	attitude.setAttitudeSetpoint(Quatf(Eulerf(0.f, 0.f, .01f)) * vertical, 0.f);
	attitude.setTaskReference(Vector3f(2.f, 2.f, 1.f), 0.f, {});
	assert(fabsf(attitude.update(vertical)(0) + .01f) < 1e-5f);

	AttitudeReference reference;
	AttitudeReference::Settings settings;
	settings.pitch_rate = math::radians(30.f);
	AttitudeReference::Output output;
	const Vector3f force(.03f, .02f, -.3f);
	assert(reference.initialize(Quatf(Eulerf(0.f, M_PI_2_F, 3.13f)), settings.bank_limit));
	Quatf previous = reference.attitude();
	Vector3f previous_omega{};
	constexpr float dt = .001f;
	float max_kinematic_error = 0.f;

	for (int k = 0; k < 10000; ++k) {
		const float pitch = k < 3000 ? math::radians(100.f) : math::radians(-100.f);
		const float authority = k < 3500 ? 1.f : .2f;
		assert(reference.update(force, NAN, .3f, pitch, authority, dt, settings, output));
		assert(fabsf(Vector3f(Dcmf(output.attitude).transpose() * output.force)(1)) < 1e-6f);
		assert(output.force.norm() <= settings.thrust_limit + 1e-6f);
		assert(fabsf(output.bank) <= settings.bank_limit + 1e-6f);
		assert(fabsf(output.pitch) <= settings.pitch_limit + 1e-6f);
		const Vector3f omega_from_rotation = Dcmf(previous) * Vector3f(AxisAnglef((previous.inversed() * output.attitude).canonical())) / dt;
		max_kinematic_error = math::max(max_kinematic_error, (omega_from_rotation - .5f * (previous_omega + output.omega_ned)).norm());
		assert(output.alpha_ned.norm() < 12.f); // includes transport cross terms
		previous = output.attitude;
		previous_omega = output.omega_ned;
	}

	printf("Reference kinematic error: %.6f rad/s\n", (double)max_kinematic_error);
	fflush(stdout);
	assert(max_kinematic_error < .003f);
	// Abrupt command reversals at the angle boundaries must retain derivative
	// consistency, including the deceleration before a bank/pitch limit.
	AttitudeReference bounded;
	assert(bounded.initialize(Quatf{}, settings.bank_limit));
	Vector3f bounded_previous_omega{};
	float previous_pitch_rate = 0.f;

	for (int k = 0; k < 12000; ++k) {
		const float sign = (k / 1000) % 2 == 0 ? 1.f : -1.f;
		const Vector3f saturated_force(.03f, sign * .3f, -.3f);
		assert(bounded.update(saturated_force, 0.f, 0.f, sign * 3.f, 1.f, .01f, settings, output));
		const float observed_acceleration = (output.omega_ned - bounded_previous_omega).norm() / .01f;

		if (observed_acceleration >= 10.f) {
			fprintf(stderr, "Reference boundary k=%d bank=%.6f pitch=%.6f acceleration=%.3f\n", k,
				(double)output.bank, (double)output.pitch, (double)observed_acceleration);
		}

		assert(observed_acceleration < 10.f);
		const Vector3f pitch_axis(0.f, cosf(output.bank), sinf(output.bank));
		const float pitch_rate = pitch_axis.dot(output.omega_ned);
		assert(fabsf(pitch_rate - previous_pitch_rate) <= settings.pitch_acceleration * .01f + 1e-5f);
		assert(fabsf(output.omega_ned(0) - bounded_previous_omega(0)) <= settings.bank_acceleration * .01f + 1e-5f);
		previous_pitch_rate = pitch_rate;
		bounded_previous_omega = output.omega_ned;
	}

	// A frame reset rotates reference attitude once and clears its derivatives.
	const Quatf delta(Eulerf(0.f, 0.f, .4f));
	reference.adaptFrame(delta, settings.bank_limit);
	assert(((delta * previous).inversed() * reference.attitude()).canonical().imag().norm() < 1e-5f);
	assert(!reference.update(force, 0.f, 0.f, 0.f, 1.f, 0.f, settings, output));
	reference.reset();
	assert(!reference.update(force, 0.f, 0.f, 0.f, 1.f, .01f, settings, output));

	// Saturation must unwind rather than accumulate, and stale residuals can be
	// replaced with zero by the wrapper without preserving a latched saturation.
	AngularAccelerationControl rate;
	rate.configure(Vector3f(8.f, 8.f, 8.f), Vector3f(1.f, 1.f, 1.f), Vector3f(.3f, .3f, .3f), 2.f, 2.f);

	for (int k = 0; k < 1000; ++k) { rate.update(Vector3f(1.f, 0.f, 0.f), {}, {}, Vector3f(2.f, 0.f, 0.f), .0025f, false); }

	assert(rate.integral()(0) < -1.9f);

	for (int k = 0; k < 1000; ++k) { rate.update(Vector3f(1.f, 0.f, 0.f), {}, {}, {}, .0025f, false); }

	assert(rate.integral()(0) > .4f);
	rate.update({}, {}, {}, {}, .0025f, true);
	assert(rate.integral().norm() < 1e-7f);
	assert(rate.update(Vector3f(NAN, 0.f, 0.f), {}, {}, {}, .0025f, false).isAllFinite());

	// Wrong pair, loss, time reversal and estimator epoch changes suppress FF.
	assert(referenceMatches(1200000, 1190000, 1190000, 1180000, 3, 3));
	assert(!referenceMatches(1200000, 1190001, 1190000, 1180000, 3, 3));
	assert(!referenceMatches(1300000, 1190000, 1190000, 1180000, 3, 3));
	assert(!referenceMatches(1180000, 1190000, 1190000, 1180000, 3, 3));
	assert(!referenceMatches(1200000, 1190000, 1190000, 0, 3, 3));
	assert(!referenceMatches(1200000, 1190000, 1190000, 1180000, 3, 4));

	// Independently calibrated reverse tail maps must round-trip in force units.
	const ::voliro_t::Geometry geometry{.14f, .45f, 0.f, .64f, 39.5f, 10.f, .02f, 6.f};
	assert(fabsf(::voliro_t::tailThrust(geometry, -1.f) + 6.f) < 1e-6f);

	for (float command : {-1.f, -.3f, 0.f, .3f, 1.f}) {
		assert(fabsf(::voliro_t::tailCommand(geometry, ::voliro_t::tailThrust(geometry, command)) - command) < 1e-6f);
	}
	// With M5 held at zero, refit front thrust to retain pitch damping while
	// bounding the additional force correction. Actual servo limits still apply.
	const Vector3f horizontal_force(-49.f, 0.f, 0.f);

	for (float pitch_torque : {.2f, 20.f}) {
		const auto solution = ::voliro_t::refitForTail(geometry, horizontal_force, Vector3f(0.f, pitch_torque, 0.f),
				      0.f, 0.f, math::radians(120.f), M_PI_F);
		const auto achieved = ::voliro_t::allocateRollYaw(geometry, solution, {}, 0.f);
		assert((achieved.force - horizontal_force).norm() <= .15f * horizontal_force.norm() + 1e-4f);
		assert(achieved.torque(1) > 0.f && achieved.torque(1) <= pitch_torque);
		const auto constrained = ::voliro_t::refitForTail(geometry, horizontal_force, Vector3f(0.f, pitch_torque, 0.f),
					 0.f, 0.f, math::radians(120.f), .01f);
		assert(fabsf(constrained.tilt) <= .01f && constrained.front_sum >= 0.f);
	}
	const auto upright = ::voliro_t::refitForTail(geometry, Vector3f(0.f, 0.f, -1.f), {}, 5.f, 0.f,
			     math::radians(120.f), M_PI_F);
	assert(fabsf(upright.tilt) < 1e-6f && upright.front_sum < 1e-6f);
	printf("PASS: heading through +/-120, task axes, reference derivatives (max %.6f rad/s), force-plane consistency, reset, anti-windup and message epochs\n",
	       (double)max_kinematic_error);
}
