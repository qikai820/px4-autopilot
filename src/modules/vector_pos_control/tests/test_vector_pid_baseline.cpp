#include "VoliroRateTest.hpp"
#include "../../vector_att_control/VectorAttitude/VectorAttitude.hpp"
#include <lib/rate_control/rate_control.hpp>
#include <lib/vector_control/VectorReferenceValidity.hpp>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;

static void near(const Vector3f &a, const Vector3f &b, float tolerance = 2e-6f)
{
	assert((a - b).norm() < tolerance);
}

int main()
{
	VectorRateCore baseline, enhanced;
	configureBaselineRate(baseline); configureBaselineRate(enhanced);
	const Vector3f inertia(.1f, .1f, .2f), scale(10.f, 10.f, 10.f);
	const Vector3f measured(.1f, -.2f, .05f), requested(.3f, .2f, -.1f), accel(.4f, -.5f, .2f);

	// Absent/stale references must retain every PID gain, D term, integral limit
	// and saturation-direction decision. Changing feedforward inertia has no effect.
	for (int k = 0; k < 12000; ++k) {
		const Vector3<bool> positive(k > 2000 && k < 4000, false, false);
		const Vector3<bool> negative(false, false, k > 5000 && k < 7000);
		baseline.setSaturationStatus(positive, negative);
		enhanced.setSaturationStatus(positive, negative);
		const Vector3f old = baseline.update(measured, requested, accel, .0025f, false);
		const Vector3f hybrid = enhanced.update(measured, requested, accel, .0025f, false,
						     {}, Vector3f(k % 2 ? 100.f : .0001f, 2.f, 3.f));
		near(old, hybrid); near(baseline.integral(), enhanced.integral());
	}
	assert(enhanced.integral()(1) > .299f); // not capped at the old mode-1 .020
	const Vector3f held_integral = enhanced.integral();
	enhanced.update({}, {}, {}, .0025f, true);
	near(enhanced.integral(), held_integral); // same landed behavior as mode 0
	enhanced.resetIntegral(); near(enhanced.integral(), {});
	puts("PASS: exact mode-0 PID/integral parity, changed inertia, landed hold, reset and saturation gating");

	// A known feedforward acceleration is not resisted by the baseline D term;
	// neither P/I feedback nor its normalized limits are rescaled by inertia.
	configureBaselineRate(enhanced);
	const Vector3f alpha_ff(.7f, -.4f, .2f);
	near(enhanced.update(measured, measured, alpha_ff, .0025f, false,
			    alpha_ff, inertia.edivide(scale)), inertia.emult(alpha_ff).edivide(scale));
	assert(vector_control::referenceMatches(1200000, 1190000, 1190000, 1180000, 3, 3));
	assert(!vector_control::referenceMatches(1400000, 1190000, 1190000, 1180000, 3, 3));
	puts("PASS: additive physical acceleration feedforward and D reference compensation");

	VectorAttitude old_attitude, mapped;
	const Vector3f gains(5.5f, 3.f, 2.8f);
	for (VectorAttitude *controller : {&old_attitude, &mapped}) {
		controller->setFullAttitude(true);
		controller->setProportionalGain(gains, 1.f);
		controller->setRateLimit(Vector3f(3.f, 3.f, 3.f));
	}
	const Quatf q(Eulerf(0.f, 0.f, .7f)), desired = q * Quatf(Eulerf(.02f, -.03f, .01f));
	old_attitude.setAttitudeSetpoint(desired, .6f); mapped.setAttitudeSetpoint(desired, .6f);
	mapped.setTaskReference(gains, .7f, {}, false);
	near(old_attitude.update(q), mapped.update(q));

	// Reproduce the two logged command-loss events through the actual attitude
	// core, with mode-0 gains and unavailable full angular references.
	for (float yaw_rate : {.79543996f, -2.61799335f}) {
		mapped.setAttitudeSetpoint(q, yaw_rate);
		mapped.setTaskReference(gains, .7f, Vector3f(9.f, 9.f, 9.f), false);
		near(mapped.update(q), q.inversed().dcm_z() * yaw_rate);
	}
	for (float pitch : {0.f, 90.f, -90.f, 160.f, -160.f}) {
		const Quatf tilted(Eulerf(.1f, math::radians(pitch), .7f));
		mapped.setAttitudeSetpoint(tilted, .6f);
		mapped.setTaskReference(gains, .7f, {}, false);
		near(mapped.update(tilted), tilted.inversed().dcm_z() * .6f);
		mapped.setTaskReference(gains, .7f, Vector3f(0.f, 0.f, .6f), true);
		near(mapped.update(tilted), tilted.inversed().dcm_z() * .6f); // no double count
	}
	puts("PASS: mode-0 attitude strength, recorded yaw commands, +/-90/160 deg projection and no duplicate feedforward");

	mapped.setRateLimit(Vector3f(math::radians(120.f), math::radians(160.f), math::radians(80.f)));
	const Quatf recorded_actual[] = {
		Quatf(.6174243689f, .0001467204f, -.0516346619f, .7849338055f),
		Quatf(.9782647491f, -.1355001777f, -.1236290485f, -.0967144817f)
	};
	const Quatf recorded_target[] = {
		Quatf(.6189844012f, -.0128750848f, -.0275960453f, .7848126888f),
		Quatf(.9732201099f, -.1432579458f, -.1292892396f, -.1249169260f)
	};
	for (int i = 0; i < 2; ++i) {
		const Dcmf rotation(recorded_target[i]);
		mapped.setAttitudeSetpoint(recorded_target[i], i == 0 ? .79543996f : -2.61799335f);
		mapped.setTaskReference(gains, atan2f(-rotation(0, 1), rotation(1, 1)), {}, false, 1.f / 3.f);
		const Vector3f rates = mapped.update(recorded_actual[i]);
		if (i == 0) { assert(rates(2) > .78f && rates(2) < .85f); }
		else { assert(fabsf(rates(2) + math::radians(80.f)) < 1e-6f); }
	}
	puts("PASS: exact real/HIL logged quaternion replay restores yaw and retains body rate limits");

	// The near-vertical schedule preserves level/inverted baseline gains and
	// full I authority, even though pitch integration runs on a slower clock.
	for (float pitch : {-180.f, -45.f, 0.f, 45.f, 180.f}) {
		assert(fabsf(vector_control::referenceFeedbackScale(Quatf(Eulerf(0.f, math::radians(pitch), 0.f)), 1.f / 3.f) - 1.f) < 1e-6f);
	}
	for (float pitch : {-90.f, 90.f}) {
		assert(fabsf(vector_control::referenceFeedbackScale(Quatf(Eulerf(0.f, math::radians(pitch), 0.f)), 1.f / 3.f) - 1.f / 3.f) < 1e-6f);
	}
	enhanced.resetIntegral();
	for (int k = 0; k < 16000; ++k) {
		enhanced.update({}, Vector3f(0.f, .4f, 0.f), {}, .0025f, false, {}, {}, Vector3f(1.f, 1.f / 3.f, 1.f));
	}
	assert(enhanced.integral()(1) > .299f);
	puts("PASS: smooth near-vertical bandwidth schedule preserves full per-axis integral range");
}
