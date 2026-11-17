#include <modules/vector_rate_control/RateControl/rate_control.hpp>
#include <lib/vector_control/VoliroReactionFeedback.hpp>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;

int main()
{
	const Vector3f units(8.f, 10.f, 15.f), proportional(.12f, -.3f, .2f);

	for (float tilt : {-2.8f, -1.57f, -.785f, -.524f, 0.f, .524f, .785f, 1.57f, 2.8f}) {
		const Vector3f axis(sinf(tilt), 0.f, -cosf(tilt));
		const float scale = vector_control::reactionProportionalScale(tilt, .5f);
		const Vector3f before = proportional.emult(units);
		const Vector3f after = vector_control::reactionProportionalFeedback(proportional, tilt, scale, units).emult(units);
		assert(fabsf(axis.dot(after) - scale * axis.dot(before)) < 1e-6f);
		assert((axis.cross(after - before)).norm() < 1e-6f);
		assert(after(1) == before(1));
		assert((vector_control::reactionProportionalFeedback(proportional, tilt, 1.f, units) - proportional).norm() == 0.f);
	}
	assert(vector_control::reactionProportionalScale(0.f, .5f) == 1.f);
	assert(vector_control::reactionProportionalScale(math::radians(9.f), .5f) == 1.f);
	assert(fabsf(vector_control::reactionProportionalScale(math::radians(31.f), .5f) - .5f) < 1e-6f);
	// Crossing upright-to-inverted flight must not reopen the high-P path.
	// HIL reproduced motor saturation when the old schedule rose at +/-160 deg.
	for (float degrees : {-180.f, -179.f, -160.f, -150.f, -90.f, -30.f,
			      30.f, 90.f, 150.f, 160.f, 179.f, 180.f}) {
		assert(fabsf(vector_control::reactionProportionalScale(math::radians(degrees), .4f) - .4f) < 1e-6f);
		assert(vector_control::reactionProportionalScale(math::radians(degrees), 1.f) == 1.f);
	}
	for (int degrees = 0; degrees <= 180; ++degrees) {
		const float scale = vector_control::reactionProportionalScale(math::radians(float(degrees)), .4f);
		assert(scale >= .4f - 1e-6f && scale <= 1.f);
		assert(fabsf(scale - vector_control::reactionProportionalScale(math::radians(float(-degrees)), .4f)) < 1e-6f);
		if (degrees > 0) {
			assert(scale <= vector_control::reactionProportionalScale(math::radians(float(degrees - 1)), .4f) + 1e-6f);
		}
	}
	VectorRateCore original, shaped;

	for (auto *core : {&original, &shaped}) {
		core->setPidGains(Vector3f(.26f, .1f, .3f), Vector3f(.2f, .4f, .1f), Vector3f(.003f, .001f, .002f));
		core->setIntegratorLimit(Vector3f(.3f, .3f, .3f));
	}
	const float tilt = .7f;
	shaped.setReactionFeedback(tilt, .5f, units);
	const Vector3f rate(.2f, .1f, -.2f), sp(.1f, .2f, .3f), accel(.4f, .3f, .2f), ff(.1f, -.2f, .2f);
	const Vector3f p = Vector3f(.26f, .1f, .3f).emult(sp - rate);
	const auto correction = vector_control::reactionProportionalFeedback(p, tilt, shaped.reactionFeedbackScale(), units) - p;

	for (int i = 0; i < 100; ++i) {
		original.setSaturationStatus(Vector3<bool>(false, true, false), Vector3<bool>(true, false, false));
		shaped.setSaturationStatus(Vector3<bool>(false, true, false), Vector3<bool>(true, false, false));
		const auto a = original.update(rate, sp, accel, .0025f, false, ff, Vector3f(.01f, .02f, .03f));
		const auto b = shaped.update(rate, sp, accel, .0025f, false, ff, Vector3f(.01f, .02f, .03f));
		assert((b - a - correction).norm() < 1e-6f); // D, integral, FF unchanged
		assert((original.integral() - shaped.integral()).norm() == 0.f);
	}

	shaped.setReactionFeedback(NAN, .5f, units);
	assert(shaped.reactionFeedbackTilt() == tilt); // invalid update retains valid state
	shaped.setReactionFeedback(0.f, 1.f, units);
	assert((original.update(rate, sp, accel, .0025f, false) - shaped.update(rate, sp, accel, .0025f, false)).norm() == 0.f);
	puts("PASS reaction P projection: unequal torque units, both tilt signs, unchanged D/I/FF, opt-out parity");
}
