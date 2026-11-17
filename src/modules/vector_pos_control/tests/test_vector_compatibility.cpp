// This replay intentionally uses only the pre-refactor public API. Compile it
// against both source trees to compare every emitted float, not just end states.
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <lib/vector_control/VoliroFixedTiltTransition.hpp>
#include <VectorAttitude.hpp>
#include <RateControl/rate_control.hpp>
#include <VoliroTAllocation.hpp>
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;
static FILE *trace_file;
static void emit(const Vector3f &v)
{
	assert(v.isAllFinite());

	if (trace_file) { for (int i = 0; i < 3; ++i) { const float x = v(i); assert(fwrite(&x, sizeof(x), 1, trace_file) == 1); } }
}
int main(int argc, char **argv)
{
	if (argc == 2) { trace_file = fopen(argv[1], "wb"); assert(trace_file); }

	const ::voliro_t::Geometry geometry{.14f, .45f, -.04f, .64f, 39.5f, 10.f, .02f, 6.f};

	for (bool dynamic : {false, true}) {
		vector_control::AttitudeReference reference;
		vector_control::AttitudeReference::Settings settings;
		assert(reference.initialize(Quatf{}, settings.bank_limit));
		VectorAttitude attitude;
		attitude.setFullAttitude(true);
		attitude.setProportionalGain(Vector3f(5.5f, 3.f, 2.8f), 1.f);
		attitude.setRateLimit(Vector3f(3.f, 3.f, 2.f));
		VectorRateCore rate;
		rate.setPidGains(Vector3f(.26f, .1f, .3f), Vector3f(.2f, .4f, .1f), Vector3f(.003f, .001f, 0.f));
		rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
		float previous_tilt = 0.f;

		for (int k = 0; k < 6000; ++k) {
			const float t = k * .01f;
			const Vector3f force(.08f * sinf(t), .05f * cosf(t), -.3f);
			const float target = (k / 1000) % 2 ? -2.f : 2.f;
			vector_control::AttitudeReference::Output out;
			assert(reference.update(force, .2f * t, .2f, target, k % 900 < 500 ? 1.f : .2f, .01f, settings, out));
			const Quatf q = Quatf(Eulerf(.1f * sinf(t), 2.5f * sinf(.2f * t), .2f * t));
			const Quatf desired = dynamic ? out.attitude : vector_control::attitudeForForce(force, .2f * t, 17, .35f, target);
			attitude.setAttitudeSetpoint(desired, .2f);

			if (dynamic) { attitude.setTaskReference(Vector3f(5.5f, 3.f, 2.8f), out.heading, out.omega_ned, k % 37 != 0, 1.f / 3.f); }

			const Vector3f rates_sp = attitude.update(q);
			const Vector3f rates(.1f * cosf(t), -.2f * sinf(t), .05f);
			rate.setSaturationStatus(Vector3<bool>(k % 11 == 0, false, false), Vector3<bool>(false, k % 13 == 0, false));
			const Vector3f ff = dynamic ? Dcmf(q).transpose() * out.alpha_ned - rates.cross(Dcmf(q).transpose() * out.omega_ned) : Vector3f{};
			const Vector3f torque = rate.update(rates, rates_sp, {}, .01f, k < 20, ff, dynamic ? Vector3f(.01f, .02f, .03f) : Vector3f{});
			const auto allocation = ::voliro_t::allocate(geometry, Dcmf(q).transpose() * force * 168.f, torque * 10.f,
						previous_tilt, math::radians(160.f), .01f);
			previous_tilt = allocation.tilt;
			emit(out.force); emit(out.omega_ned); emit(out.alpha_ned);
			emit(Vector3f(out.heading, out.bank, out.pitch));
			emit(rates_sp); emit(torque); emit(rate.integral()); emit(allocation.force); emit(allocation.torque);
			emit(Vector3f(allocation.thrust[0], allocation.thrust[1], allocation.thrust[2]));
			emit(Vector3f(allocation.thrust[3], allocation.thrust[4], allocation.tilt));

			if (k == 3000) { reference.adaptFrame(Quatf(Eulerf(0.f, 0.f, .1f)), settings.bank_limit); }
		}
	}

	if (trace_file) { assert(fclose(trace_file) == 0); }

	puts("PASS: 12000-sample Voliro legacy/dynamic compatibility replay");
}
