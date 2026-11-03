#include "ActuatorEffectivenessVoliroT.hpp"
#include <cstdio>
#include <cstdlib>
static bool dynamics_mode = false;
static vector_actuator_status_s published_status{};
ORB_DEFINE(vector_actuator_status, struct vector_actuator_status_s, sizeof(vector_actuator_status_s), 0,
	   static_cast<orb_id_size_t>(ORB_ID::vector_actuator_status), 1);
// Transport-only stubs: exercise the real effectiveness implementation and
// capture its optional diagnostic publication without starting a uORB service.
const orb_metadata *get_orb_meta(ORB_ID) { return nullptr; }
extern "C" orb_advert_t orb_advertise(const orb_metadata *, const void *) { return &published_status; }
int uORB::Manager::orb_unadvertise(orb_advert_t) { return 0; }
uint8_t uORB::Manager::orb_get_queue_size(const void *) { return 1; }
int uORB::Manager::orb_publish(const orb_metadata *, orb_advert_t, const void *data)
{
	published_status = *static_cast<const vector_actuator_status_s *>(data);
	return 0;
}
#undef param_get
extern "C" void param_set_used(param_t) {}
extern "C" int param_get(param_t p, void *value)
{
	if (static_cast<px4::params>(p) == px4::params::VT_CTRL_MODE) {
		*static_cast<int32_t *>(value) = dynamics_mode ? 1 : 0;
		return 0;
	}

	float v = 0.f;

	switch (static_cast<px4::params>(p)) {
	case px4::params::VT_REACT_FILT: v = 2.f; break;

	case px4::params::VT_MOT_TAU: v = .1f; break;

	case px4::params::CA_CT_X: v = .14f; break;

	case px4::params::CA_CT_Y: v = .45f; break;

	case px4::params::CA_CT_TAIL: v = .64f; break;

	case px4::params::CA_CT_TMAX: v = 39.5f; break;

	case px4::params::CA_CT_T5MAX: v = 10.f; break;

	case px4::params::CA_CT_KM: v = .02f; break;

	case px4::params::CA_CT_ANGLE: v = 120.f; break;

	case px4::params::CA_CT_RATE: v = 90.f; break;

	case px4::params::CA_R5_SLEW: v = .4f; break;

	case px4::params::CA_CT_T5_DZ: v = .03f; break;

	case px4::params::CA_CT_T5_HYST: v = .10f; break;

	case px4::params::CA_CT_T5_HOLD: v = .04f; break;

	case px4::params::CA_CT_T5_WAIT: v = .08f; break;

	case px4::params::VEC_TQ_R: case px4::params::VEC_TQ_P: case px4::params::VEC_TQ_Y: v = 10.f; break;

	default: break;
	}

	*static_cast<float *>(value) = v; return 0;
}
extern "C" uint64_t hrt_absolute_time() { static uint64_t t = 100000; return t += 10000; }
int ActuatorEffectiveness::Configuration::addActuator(ActuatorType, const matrix::Vector3f &, const matrix::Vector3f &) {return 0;}
// Registration and stoppable-motor policy are not exercised by this host test.
void ActuatorEffectiveness::stopMaskedMotorsWithZeroThrust(uint32_t, ActuatorVector &) {}
static void check(bool ok, const char *message)
{
	if (!ok) {std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1);}
}
static void test_zero_force_servo()
{
	ActuatorEffectivenessVoliroT effectiveness(nullptr);
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = -1.f; minimum(5) = -1.f;
	matrix::Vector<float, 6> demand;

	for (int i = 0; i < 300; ++i) {
		demand(1) = i % 2 ? -.02f : .02f;
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		check(fabsf(output(5)) < 1e-6f, "armed zero thrust with pitch feedback keeps Servo1 neutral");
	}

	// Establish a nonzero common tilt, then cut thrust and torque. The
	// wrapper must retain the allocator's configured slew back to neutral.
	demand.zero(); demand(3) = .10f; demand(5) = -.25f;

	for (int i = 0; i < 100; ++i) {
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	}

	check(output(5) > .1f, "nonzero forward thrust can still tilt Servo1");
	float previous = output(5);
	demand.zero();

	for (int i = 0; i < 100; ++i) {
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		check(fabsf(output(5) - previous) <= 90.f / 120.f * .01f + 1e-6f,
		      "zero-demand wrapper respects common-tilt rate when centering");
		check(fabsf(output(5)) <= fabsf(previous) + 1e-6f, "zero demand centers Servo1 monotonically");
		previous = output(5);
	}

	check(fabsf(output(5)) < 1e-6f, "zero demand finishes centering Servo1");
	std::puts("PASS: armed zero-force pitch feedback, forward tilt and rate-limited servo centering");
}
int main()
{
	test_zero_force_servo();
	ActuatorEffectivenessVoliroT effectiveness(nullptr);
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) {minimum(i) = 0.f; maximum(i) = 1.f;}

	minimum(4) = -1.f; minimum(5) = -1.f;
	matrix::Vector<float, 6> demand;
	demand(5) = -5.04f * 9.80665f / 168.f;
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
// Emulate common motor/servo slew constraints and a near-zero generic
// M5 slew result. The original demand must still pass the start threshold.
	output(0) *= .5f; output(5) = .02f; output(4) = 0.f;
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	check(fabsf(output(4) - .05f) < 1e-6f, "M5 guard uses original demand and applies a single slew limit");
	voliro_t::Geometry geometry{.14f, .45f, 0.f, .64f, 39.5f, 10.f, .02f};
	voliro_t::Result achieved;
	achieved.tilt = output(5) * math::radians(120.f);

	for (int i = 0; i < 5; ++i) { achieved.thrust[i] = output(i) * fabsf(output(i)) * (i == 4 ? 10.f : 39.5f); }

	voliro_t::forward(geometry, achieved);
	control_allocator_status_s status{};
	effectiveness.getUnallocatedControl(0, status);

	for (int i = 0; i < 3; ++i) {
		check(fabsf(status.unallocated_thrust[i] - (demand(i + 3) * 168.f - achieved.force(i)) / 168.f) < 1e-6f,
		      "force residual includes final motor/servo outputs");
		check(fabsf(status.unallocated_torque[i] - (demand(i) * 10.f - achieved.torque(i)) / 10.f) < 1e-6f,
		      "torque residual includes final motor/servo outputs");
	}

// With front motors held at zero, the guarded tail's pitch moment must
// be reported as unallocated correction rather than falsely achieved.
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);

	for (int i = 0; i < 4; ++i) { output(i) = 0.f; }

	output(5) = 0.f;
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	effectiveness.getUnallocatedControl(0, status);
	check(status.unallocated_torque[1] > 0.f, "pitch anti-windup sees protected output");
	check(fabsf(output(4) - .10f) < 1e-6f, "M5 protection advances from last final command");
	effectiveness.updateSetpointAfterSlew(.001f, 0, output, false);
	check(fabsf(output(4)) < 1e-7f, "disarm stops immediately without slew delay");
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	check(fabsf(output(4) - .05f) < 1e-6f, "rearm starts from zero state");
	std::puts("PASS: actual Voliro T effectiveness guard, single slew stage, final-command residuals and disarm reset");

	dynamics_mode = true;
	ActuatorEffectivenessVoliroT dynamic_effectiveness(nullptr);
	demand.zero(); demand(3) = 5.04f * 9.80665f / 168.f;

	for (int i = 0; i < 150; ++i) {
		dynamic_effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		dynamic_effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	}

	demand(2) = .03f; // body-Z bank torque at vertical common tilt
	dynamic_effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	dynamic_effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	dynamic_effectiveness.getUnallocatedControl(0, status);
	check(fabsf(status.unallocated_torque[2]) < 1e-5f, "vertical bank torque bypasses reaction filtering");
	demand(0) = .03f; // body-X reaction torque at vertical common tilt
	dynamic_effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	dynamic_effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	dynamic_effectiveness.getUnallocatedControl(0, status);
	check(status.unallocated_torque[0] > .01f, "vertical reaction torque is filtered and reported as shortfall");
	check(published_status.prediction_enabled && published_status.armed, "actuator prediction is explicitly labeled");
	dynamic_effectiveness.updateSetpointAfterSlew(.01f, 0, output, false);
	dynamic_effectiveness.updateSetpointAfterSlew(.01f, 0, output, false);
	check(!published_status.armed && !published_status.prediction_enabled, "disarm invalidates dynamic prediction");
	std::puts("PASS: vertical reaction/bank filtering, residuals, prediction labeling and disarm reset");
}
