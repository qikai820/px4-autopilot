// Real MorphQuad allocation with the common reference and controller cores.
// Parameter/registration stubs replace services only; allocation is not mocked.
#include "ActuatorEffectivenessMorphQuad.hpp"
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <geo/geo.h>
#include <VectorAttitude.hpp>
#include <RateControl/rate_control.hpp>
#include <cassert>
#include <cstdio>
#include <cstdarg>
#include <initializer_list>

using namespace matrix;
extern "C" void px4_log_modulename(int, const char *, const char *fmt, ...)
{
	va_list args; va_start(args, fmt); vfprintf(stderr, fmt, args); va_end(args);
	assert(false && "allocator model validation failed");
}
#undef param_get
extern "C" void param_set_used(param_t) {}
extern "C" int param_get(param_t p, void *value)
{
	const auto parameter = static_cast<px4::params>(p);

	if (parameter == px4::params::CA_MQ_TCURVE) { *static_cast<int32_t *>(value) = 0; return 0; }

	float v = 0.f;

	switch (parameter) {
	case px4::params::VEC_TQ_R: case px4::params::VEC_TQ_P: case px4::params::VEC_TQ_Y: v = 10.f; break;

	case px4::params::CA_MQ_L: v = .25f; break;

	case px4::params::CA_MQ_B: v = .275f; break;

	case px4::params::CA_MQ_Z: v = -.04f; break;

	case px4::params::CA_MQ_TMAX: v = 47.459f; break;

	case px4::params::CA_MQ_ALPM: case px4::params::CA_MQ_BETM: case px4::params::CA_MQ_SMAX: v = 180.f; break;

	case px4::params::CA_MQ_MMAX: v = 1.f; break;

	case px4::params::CA_MQ_LOW_THR: v = .2f; break;

	case px4::params::CA_MQ_SSTEP: v = .05f; break;

	default: break;
	}

	*static_cast<float *>(value) = v;
	return 0;
}
int ActuatorEffectiveness::Configuration::addActuator(ActuatorType, const Vector3f &, const Vector3f &) { return 0; }
void ActuatorEffectiveness::stopMaskedMotorsWithZeroThrust(uint32_t, ActuatorVector &) {}

static void forward(const ActuatorEffectiveness::ActuatorVector &output, Vector3f &force, Vector3f &torque)
{
	force.zero(); torque.zero();

	for (int i = 0; i < 4; ++i) {
		const float a = output(4 + 2 * i) * M_PI_F, b = output(5 + 2 * i) * M_PI_F;
		const Vector3f f = Vector3f(sinf(a) * cosf(b), sinf(b), -cosf(a) * cosf(b)) * (output(i) * output(i) * 47.459f);
		const Vector3f r(i < 2 ? .25f : -.25f, i == 0 || i == 3 ? .275f : -.275f, -.04f);
		force += f; torque += r.cross(f);
	}
}
int main()
{
	setvbuf(stdout, nullptr, _IONBF, 0);
	ActuatorEffectivenessMorphQuad allocation(nullptr);
	ActuatorEffectiveness::ActuatorVector minimum{}, maximum{}, output{};

	for (int i = 0; i < 16; ++i) { minimum(i) = i < 4 ? 0.f : -1.f; maximum(i) = 1.f; }

	// Exercise all six requested axes through the original allocator and
	// independently reconstruct force/torque from its final twelve outputs.
	for (int axis = 0; axis < 6; ++axis) {
		for (float sign : {-1.f, 1.f}) {
			Vector<float, 6> request{}; request(5) = -.35f; request(axis) += sign * .02f;

			for (int k = 0; k < 100; ++k) { allocation.updateSetpoint(request, 0, output, minimum, maximum); }

			Vector3f f, torque; forward(output, f, torque);
			assert(Vector3f(f / (4.f * 47.459f) - Vector3f(request(3), request(4), request(5))).norm() < 1e-5f);
			assert(Vector3f(torque / 10.f - Vector3f(request(0), request(1), request(2))).norm() < 1e-5f);
		}
	}

	Vector<float, 6> zero{};
	allocation.updateSetpoint(zero, 0, output, minimum, maximum);

	for (int i = 0; i < 12; ++i) { assert(fabsf(output(i)) < 1e-6f); }

	puts("PASS: original MorphQuad allocator six-axis signs/units/output mapping and zero reset");

	vector_control::AttitudeReference reference;
	vector_control::AttitudeReference::Settings settings;
	settings.force_space = vector_control::capabilities(vector_control::morphquad).force_space;
	settings.bank_limit = settings.pitch_limit = vector_control::spatialAttitudeLimit(120.f);
	settings.pitch_rate = settings.bank_rate = math::radians(20.f);
	settings.pitch_acceleration = settings.bank_acceleration = math::radians(60.f);
	assert(reference.initialize(Quatf{}, settings.bank_limit));
	VectorAttitude attitude;
	attitude.setFullAttitude(true);
	attitude.setProportionalGain(Vector3f(4.f, 4.f, 2.8f), 1.f);
	attitude.setRateLimit(Vector3f(3.f, 3.f, 2.f));
	VectorRateCore rate;
	rate.setPidGains(Vector3f(.15f, .15f, .2f), Vector3f(.2f, .2f, .1f), Vector3f(.003f, .003f, 0.f));
	rate.setIntegratorLimit(Vector3f(.3f, .3f, .3f));
	const Vector3f inertia(.182f, .260f, .384f);
	const Vector3f force_ned(0.f, 0.f, -6.8f * CONSTANTS_ONE_G / (4.f * 47.459f));
	Quatf q{}; Vector3f omega{}, acceleration{};
	float max_force_error = 0.f;
	constexpr float dt = .005f;

	// Cross +/-90 on each independent axis and return to level between
	// large-axis maneuvers. Combined +/-120-degree transitions are outside
	// this ideal rigid-body regression's validated envelope.
	for (int stage = 0; stage < 14; ++stage) {
		const float roll = stage == 0 ? .17f : stage == 1 ? -.17f : stage == 4 ? .12f
				   : stage == 6 ? math::radians(120.f) : stage == 8 ? math::radians(-120.f) : 0.f;
		const float pitch = stage == 2 ? .22f : stage == 3 ? -.22f : stage == 4 ? -.15f
				    : stage == 10 ? math::radians(120.f) : stage == 12 ? math::radians(-120.f) : 0.f;
		const float heading = stage == 4 ? .3f : 0.f;

		for (int k = 0; k < (stage < 6 ? 1600 : 2400); ++k) {
			vector_control::AttitudeReference::Output ref;
			assert(reference.update(force_ned, heading, 0.f, pitch, reference.pitchAuthority(q), dt, settings, ref, roll));
			attitude.setAttitudeSetpoint(ref.attitude, 0.f);
			attitude.setAngularReference(ref.omega_ned, true);
			const Dcmf world_to_body = Dcmf(q).transpose();
			const Vector3f alpha_ff = world_to_body * ref.alpha_ned - omega.cross(world_to_body * ref.omega_ned);
			const Vector3f torque = rate.update(omega, attitude.update(q), acceleration, dt, false, alpha_ff, inertia / 10.f);
			Vector<float, 6> request{};

			for (int i = 0; i < 3; ++i) { request(i) = torque(i); request(i + 3) = Vector3f(world_to_body * ref.force)(i); }

			allocation.updateSetpoint(request, 0, output, minimum, maximum);
			Vector3f achieved_force, achieved_torque; forward(output, achieved_force, achieved_torque);
			control_allocator_status_s status{}; allocation.getUnallocatedControl(0, status);
			rate.setSaturationStatus(Vector3<bool>(status.unallocated_torque[0] > 1e-5f, status.unallocated_torque[1] > 1e-5f,
							       status.unallocated_torque[2] > 1e-5f),
						 Vector3<bool>(status.unallocated_torque[0] < -1e-5f, status.unallocated_torque[1] < -1e-5f, status.unallocated_torque[2] < -1e-5f));
			max_force_error = math::max(max_force_error, Vector3f(Dcmf(q) * achieved_force / (4.f * 47.459f) - force_ned).norm());
			acceleration = (achieved_torque - omega.cross(inertia.emult(omega))).edivide(inertia);
			omega += acceleration * dt;
			q = q * Quatf(AxisAnglef(omega * dt)); q.normalize();
			assert(q.isAllFinite() && omega.isAllFinite() && omega.norm() < 5.f);

			for (int i = 0; i < 12; ++i) { assert(output(i) >= minimum(i) && output(i) <= maximum(i)); }
		}

		const Quatf expected = Quatf(Eulerf(0.f, 0.f, heading)) * Quatf(Eulerf(roll, 0.f, 0.f)) * Quatf(Eulerf(0.f, pitch, 0.f));
		const float error = AxisAnglef((expected.inversed() * q).canonical()).angle();
		printf("MorphQuad stage %d attitude error=%.4f deg\n", stage, double(math::degrees(error)));
		assert(error < math::radians(1.f));
	}

	printf("MorphQuad peak normalized force residual=%.7f\n", double(max_force_error));
	assert(max_force_error < .005f);
	printf("PASS: common spatial reference/PID + real MorphQuad allocator rigid-body closed loop; peak normalized force error %.7f\n",
	       double(max_force_error));
}
