#include <lib/vector_control/VectorAttitudeReference.hpp>
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;
using namespace vector_control;

int main()
{
	static_assert(capabilities(morphquad).independentRoll(), "dual-axis thrust");
	static_assert(capabilities(voliro_t).independentPitch() && !capabilities(voliro_t).independentRoll(), "plane constraint");
	static_assert(capabilities(voliro_t).shared_tilt && capabilities(voliro_t).reversible_tail, "Voliro topology");
	static_assert(!capabilities(0).vectorThrust(), "conventional MC");

	for (int mode : {0, 1}) {
		for (bool enabled : {false, true}) {
			assert(referenceEnabled(voliro_t, mode, enabled) == (mode == 1));
			assert(referenceEnabled(morphquad, mode, enabled) == enabled);
			assert(!referenceEnabled(0, mode, enabled));
		}
	}

	AttitudeReference reference;
	AttitudeReference::Settings settings;
	settings.force_space = capabilities(morphquad).force_space;
	settings.bank_limit = settings.pitch_limit = math::radians(20.f);
	settings.bank_rate = settings.pitch_rate = math::radians(20.f);
	settings.bank_acceleration = settings.pitch_acceleration = math::radians(60.f);
	assert(reference.initialize(Quatf{}, settings.bank_limit));
	AttitudeReference::Output out;
	Quatf previous{};
	Vector3f previous_omega{};
	float previous_roll_rate = 0.f, previous_pitch_rate = 0.f, max_error = 0.f;
	constexpr float dt = .0025f;
	const Vector3f force(.08f, -.07f, -.35f);

	// Independent roll/pitch, command reversals, reduced tracking authority,
	// leveling, and heading across +/-pi share one derivative-consistent model.
	for (int k = 0; k < 16000; ++k) {
		const float target = k < 6000 ? 1.f : k < 12000 ? -1.f : 0.f;
		assert(reference.update(force, NAN, .3f, -target, k < 9000 ? 1.f : .2f, dt, settings, out, target));
		assert((out.force - force).norm() < 1e-7f);
		assert(fabsf(out.bank) <= settings.bank_limit + 1e-6f);
		assert(fabsf(out.pitch) <= settings.pitch_limit + 1e-6f);
		const Vector3f x(cosf(out.heading), sinf(out.heading), 0.f);
		const float roll_rate = x.dot(out.omega_ned);
		const float pitch_rate = Vector3f(-sinf(out.heading), cosf(out.heading), 0.f).dot(out.omega_ned) / cosf(out.bank);
		assert(fabsf(roll_rate) <= settings.bank_rate + 1e-5f);
		assert(fabsf(pitch_rate) <= settings.pitch_rate + 1e-5f);
		assert(fabsf(roll_rate - previous_roll_rate) <= settings.bank_acceleration * dt + 2e-5f);
		assert(fabsf(pitch_rate - previous_pitch_rate) <= settings.pitch_acceleration * dt + 2e-5f);
		const Vector3f observed = Dcmf(previous) * Vector3f(AxisAnglef((previous.inversed() * out.attitude).canonical())) / dt;
		max_error = math::max(max_error, (observed - .5f * (previous_omega + out.omega_ned)).norm());
		assert(out.alpha_ned.isAllFinite());
		const float reconstructed_heading_rate = out.omega_ned(2) - sinf(out.bank) * pitch_rate;
		assert(fabsf(out.heading_rate - reconstructed_heading_rate) < 1e-6f);
		previous = out.attitude; previous_omega = out.omega_ned;
		previous_roll_rate = roll_rate; previous_pitch_rate = pitch_rate;
	}

	assert(max_error < .003f);
	assert(fabsf(out.bank) < 1e-4f && fabsf(out.pitch) < .01f);
	assert(!reference.update(force, 0.f, 0.f, 0.f, 1.f, dt, settings, out, NAN));
	const Quatf delta(Eulerf(0.f, 0.f, .3f));
	reference.adaptFrame(delta, settings.bank_limit);
	assert(((delta * previous).inversed() * reference.attitude()).canonical().imag().norm() < 1e-5f);
	printf("PASS: spatial force decoupling, two-axis limits/derivatives/leveling/frame reset (kinematic error %.6f rad/s)\n",
	       double(max_error));

	// Beyond vertical, the principal asin bank branch would introduce a
	// 180-degree heading/pitch change. Reinitialize and reset the estimator
	// frame at both vertical and inverted-bank poses using the same chart.
	settings.bank_limit = settings.pitch_limit = spatialAttitudeLimit(120.f);
	for (float bank : {-120.f, -90.f, -89.f, 0.f, 89.f, 90.f, 120.f}) {
		for (float pitch : {-120.f, 0.f, 120.f}) {
			const float heading = 3.1f;
			const Quatf pose = Quatf(Eulerf(0.f, 0.f, heading)) * Quatf(Eulerf(math::radians(bank), 0.f, 0.f))
					   * Quatf(Eulerf(0.f, math::radians(pitch), 0.f));
			assert(fabsf(wrap_pi(spatialHeading(pose, heading) - heading)) < 1e-5f);
			assert(reference.initialize(pose, settings.bank_limit, heading));
			assert(AxisAnglef((pose.inversed() * reference.attitude()).canonical()).angle() < 1e-5f);
			reference.adaptFrame(delta, settings.bank_limit);
			assert(AxisAnglef(((delta * pose).inversed() * reference.attitude()).canonical()).angle() < 1e-5f);
			assert(reference.update(force, wrap_pi(heading + .3f), 0.f, math::radians(pitch), 1.f, dt,
						settings, out, math::radians(bank)));
			assert(fabsf(out.bank - math::radians(bank)) < 1e-5f);
			assert(fabsf(out.pitch - math::radians(pitch)) < 1e-5f);
		}
	}
	assert(!PX4_ISFINITE(spatialHeading(Quatf(NAN, 0.f, 0.f, 0.f), 0.f)));
	puts("PASS: spatial +/-120-degree initialization and heading/frame continuity through +/-90 degrees");

	VectorAttitude attitude;
	attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(5.f, 3.f, 2.f), 1.f);
	attitude.setRateLimit(Vector3f(5.f, 5.f, 5.f));
	const Quatf q(Eulerf(.3f, -.4f, .7f));
	const Vector3f omega(.2f, -.1f, .4f);
	attitude.setAttitudeSetpoint(q, .5f);
	attitude.setAngularReference(omega, true);
	assert((attitude.update(q) - Dcmf(q).transpose() * omega).norm() < 1e-6f);
	attitude.setAngularReference(omega, false);
	assert((attitude.update(q) - q.inversed().dcm_z() * .5f).norm() < 1e-6f);
	attitude.setAngularReference(omega, true);
	attitude.adaptAttitudeSetpoint(delta);
	assert((attitude.update(delta * q) - Dcmf(q).transpose() * omega).norm() < 1e-6f);
	attitude.setAttitudeSetpoint(q, .5f); // a different source clears old full FF
	assert((attitude.update(q) - q.inversed().dcm_z() * .5f).norm() < 1e-6f);
	puts("PASS: spatial feedforward, dropout yaw fallback, source switch and estimator-frame transport");
}
