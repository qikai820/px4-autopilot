#include "ActuatorEffectivenessVoliroT.hpp"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <cstring>
static bool dynamics_mode = false;
static int32_t airmode_setting = 0;
static bool log_geometry = false;
static float tail_reverse_limit = .6f;
static bool control_mode_available = false;
static bool refresh_control_mode = true;
static vehicle_control_mode_s input_control_mode{};
static bool land_available = false;
static bool refresh_land = true;
static vehicle_land_detected_s input_land{};
static bool monitor_mode = false;
static bool tilt_status_available = false;
static float servo_rate = 90.f;
static uint64_t test_time = 100000;
static uint64_t clock_step = 10000;
static voliro_tilt_status_s input_tilt_status{};
static vector_actuator_status_s published_status{};
ORB_DEFINE(vector_actuator_status, struct vector_actuator_status_s, sizeof(vector_actuator_status_s), 0,
	   static_cast<orb_id_size_t>(ORB_ID::vector_actuator_status), 1);
ORB_DEFINE(voliro_tilt_status, struct voliro_tilt_status_s, sizeof(voliro_tilt_status_s), 0,
	   static_cast<orb_id_size_t>(ORB_ID::voliro_tilt_status), 1);
ORB_DEFINE(vehicle_control_mode, struct vehicle_control_mode_s, sizeof(vehicle_control_mode_s), 0,
	   static_cast<orb_id_size_t>(ORB_ID::vehicle_control_mode), 1);
ORB_DEFINE(vehicle_land_detected, struct vehicle_land_detected_s, sizeof(vehicle_land_detected_s), 0,
	   static_cast<orb_id_size_t>(ORB_ID::vehicle_land_detected), 1);
// Transport-only stubs: exercise the real effectiveness implementation and
// capture its optional diagnostic publication without starting a uORB service.
const orb_metadata *get_orb_meta(ORB_ID) { return nullptr; }
bool uORB::Subscription::subscribe()
{
	_node = _orb_id == ORB_ID::vehicle_control_mode ? static_cast<void *>(&input_control_mode)
		: _orb_id == ORB_ID::vehicle_land_detected ? static_cast<void *>(&input_land) : static_cast<void *>(&input_tilt_status);
	return true;
}
void uORB::Subscription::unsubscribe() { _node = nullptr; }
bool uORB::Manager::orb_data_copy(void *node, void *dst, unsigned &, bool)
{
	if (node == &input_land) {
		if (!land_available) { return false; }

		if (refresh_land) { input_land.timestamp = test_time; }

		*static_cast<vehicle_land_detected_s *>(dst) = input_land;
		return true;
	}

	if (node == &input_control_mode) {
		if (!control_mode_available) { return false; }

		if (refresh_control_mode) { input_control_mode.timestamp = test_time; }

		*static_cast<vehicle_control_mode_s *>(dst) = input_control_mode;
		return true;
	}

	if (!tilt_status_available) { return false; }

	*static_cast<voliro_tilt_status_s *>(dst) = input_tilt_status;
	return true;
}
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

	if (static_cast<px4::params>(p) == px4::params::VT_TILT_EN) {
		*static_cast<int32_t *>(value) = monitor_mode ? 1 : 0;
		return 0;
	}

	if (static_cast<px4::params>(p) == px4::params::VC_AIRMODE) {
		*static_cast<int32_t *>(value) = airmode_setting;
		return 0;
	}

	float v = 0.f;

	switch (static_cast<px4::params>(p)) {
	case px4::params::VT_REACT_FILT: v = 2.f; break;

	case px4::params::VT_MOT_TAU: v = .1f; break;

	case px4::params::VT_PITCH_RATE: v = 20.f; break;

	case px4::params::CA_CT_X: v = log_geometry ? .02f : .14f; break;

	case px4::params::CA_CT_Y: v = .45f; break;

	case px4::params::CA_CT_TAIL: v = log_geometry ? .74f : .64f; break;

	case px4::params::CA_CT_TMAX: v = 39.5f; break;

	case px4::params::CA_CT_T5MAX: v = 10.f; break;

	case px4::params::CA_CT_T5RMAX: v = log_geometry ? tail_reverse_limit : 0.f; break;

	case px4::params::CA_CT_KM: v = .02f; break;

	case px4::params::CA_CT_ANGLE: v = 120.f; break;

	case px4::params::CA_CT_RATE: v = servo_rate; break;

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
extern "C" uint64_t hrt_absolute_time() { return test_time += clock_step; }
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
static void test_tilt_fallback()
{
	dynamics_mode = false; monitor_mode = true;
	ActuatorEffectivenessVoliroT effectiveness(nullptr);
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = minimum(5) = -1.f;
	matrix::Vector<float, 6> demand;
	demand(3) = .1f; demand(5) = -.25f;

	for (int i = 0; i < 200; ++i) {
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	}

	check(published_status.armed, "monitor receives final servo command in legacy mode");
	input_tilt_status = {};
	input_tilt_status.timestamp = test_time + 1;
	input_tilt_status.timestamp_sample = input_tilt_status.timestamp;
	input_tilt_status.armed = input_tilt_status.fallback = input_tilt_status.observation_valid = true;
	input_tilt_status.tilt_observed = math::radians(-15.f);
	tilt_status_available = true;
	const float previous = output(5);
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	check(output(5) < previous && output(5) > 0.f, "fallback slews servo toward neutral without a step");
	check(previous - output(5) <= 10.f / 120.f * .02f + 1e-6f, "fallback centering reserves attitude tracking rate");
	voliro_t::Geometry g{.14f, .45f, 0.f, .64f, 39.5f, 10.f, .02f};
	voliro_t::Result actual;
	actual.tilt = input_tilt_status.tilt_observed;

	for (int i = 0; i < 5; ++i) { actual.thrust[i] = output(i) * fabsf(output(i)) * (i == 4 ? 10.f : 39.5f); }

	voliro_t::forward(g, actual);
	control_allocator_status_s status{};
	effectiveness.getUnallocatedControl(0, status);

	for (int i = 0; i < 3; ++i) {
		check(fabsf(status.unallocated_thrust[i] - (demand(i + 3) * 168.f - actual.force(i)) / 168.f) < 1e-6f,
		      "fallback residual uses held observed tilt rather than centering command");
	}

	tilt_status_available = false;
	const float held_command = output(5);

	for (int i = 0; i < 300; ++i) {
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	}

	check(fabsf(output(5) - held_command) < 1e-6f, "status loss retains fallback and holds unobserved servo motion");
	tilt_status_available = true;

	for (int i = 0; i < 300; ++i) {
		input_tilt_status.timestamp = input_tilt_status.timestamp_sample = test_time + 1;
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	}

	check(fabsf(output(5)) < 1e-6f, "fresh observations allow commanded centering to finish");
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, false);
	// A stale previous-flight latch must not be accepted after rearming.
	tilt_status_available = true;
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
	effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
	check(output(5) > .001f, "disarm clears allocator latch and rejects previous-flight messages");
	tilt_status_available = false;
	std::puts("PASS: monitor command publication, fallback centering, observed-angle residual, dropout latch and disarm epoch");
}

static void test_fixed_tilt_switch()
{
	for (bool dynamic : {false, true}) {
		dynamics_mode = dynamic; monitor_mode = false; tilt_status_available = false;
		ActuatorEffectivenessVoliroT effectiveness(nullptr);
		using Vec = ActuatorEffectiveness::ActuatorVector;
		Vec minimum, maximum, output;

		for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

		minimum(4) = minimum(5) = -1.f;
		matrix::Vector<float, 6> demand;
		demand(3) = .1f; demand(5) = -.25f;

		for (int i = 0; i < 200; ++i) {
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		}

		check(output(5) > .1f, "vector mode establishes nonzero servo tilt");
		tilt_status_available = true; input_tilt_status = {};
		input_tilt_status.armed = input_tilt_status.fixed_tilt = true;
		float previous = output(5);

		for (int i = 0; i < 300; ++i) {
			input_tilt_status.timestamp = test_time + 1;
			demand(3) = i % 2 ? -.1f : .1f;
			demand(1) = i % 3 ? -.02f : .02f;
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
			check(fabsf(output(5) - previous) <= 90.f / 120.f * .02f + 1e-6f,
			      "fixed mode centers at configured rate (clock stub advances on status publication)");
			check(fabsf(output(5)) <= fabsf(previous) + 1e-6f, "fixed mode centers monotonically despite wrench demand");
			previous = output(5);
		}

		check(fabsf(output(5)) < 1e-6f, "fixed mode keeps servo at zero under forward/reverse and pitch demands");
		input_tilt_status.fixed_tilt = false;
		demand.zero(); demand(3) = .1f; demand(5) = -.25f;

		for (int i = 0; i < 100; ++i) {
			input_tilt_status.timestamp = test_time + 1;
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		}

		check(output(5) > .1f, "switch off restores vector allocation");
		input_tilt_status.fixed_tilt = true;
		input_tilt_status.timestamp = test_time - 200000;
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		check(output(5) > .1f, "stale fixed-mode request cannot freeze vector allocation");
		input_tilt_status.timestamp = test_time + 1000000;
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		check(output(5) > .1f, "future fixed-mode request is rejected");
		input_tilt_status.timestamp = test_time + 1;
		input_tilt_status.fallback = input_tilt_status.observation_valid = true;
		input_tilt_status.timestamp_sample = input_tilt_status.timestamp;
		input_tilt_status.tilt_observed = math::radians(15.f);
		effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		const float latched_servo = output(5);
		input_tilt_status.fixed_tilt = input_tilt_status.fallback = false;

		for (int i = 0; i < 600; ++i) {
			input_tilt_status.timestamp = test_time + 1;
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		}

		check(fabsf(output(5) - latched_servo) < 1e-6f, "fault latch survives switch off and pauses centering without fresh fault observation");
	}
	tilt_status_available = false;
	std::puts("PASS: real allocator fixed-servo mode, centering, switch off, stale/future commands and fault priority (legacy/dynamic)");
}

static void test_configured_servo_speed()
{
	for (bool dynamic : {false, true}) {
		for (float rate : {5.f, 30.f, 90.f}) {
			dynamics_mode = dynamic; monitor_mode = false; tilt_status_available = false; servo_rate = rate;
			ActuatorEffectivenessVoliroT effectiveness(nullptr);
			using Vec = ActuatorEffectiveness::ActuatorVector;
			Vec minimum, maximum, output;

			for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

			minimum(4) = minimum(5) = -1.f;
			matrix::Vector<float, 6> demand;
			float previous = 0.f;

			for (float direction : {1.f, -1.f, 0.f}) {
				float peak_step = 0.f;
				demand(3) = direction * .25f; demand(5) = direction == 0.f ? 0.f : -.1f;

				for (int i = 0; i < 3000; ++i) {
					effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
					effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
					// The clock stub advances 10 ms per read; dynamic status
					// publication adds a read, giving a 20 ms allocation interval.
					const float step = fabsf(output(5) - previous) * 120.f;
					check(step <= rate * (dynamic ? .02f : .01f) + 1e-4f,
					      "configured speed bounds Servo1 in both directions and while centering");
					peak_step = math::max(peak_step, step);
					previous = output(5);
				}

				check(peak_step > rate * .009f, "servo command can reach configured speed");
				check(direction == 0.f ? fabsf(output(5)) < 1e-6f : output(5) * direction > .1f,
				      "limited servo reaches forward/reverse tilt and center");
			}
		}
	}

	dynamics_mode = false; servo_rate = 90.f;
	std::puts("PASS: real allocator CA_CT_RATE=5/30/90 deg/s in legacy/dynamic modes, reversal and centering");
}


static void test_manual_airmode()
{
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = minimum(5) = -1.f;
	log_geometry = true; control_mode_available = true; refresh_control_mode = true; land_available = true; input_land = {};
	monitor_mode = false; tilt_status_available = false;
	const voliro_t::Geometry g{.02f, .45f, 0.f, .74f, 39.5f, 10.f, .02f, .6f};

	for (bool dynamic : {false, true}) {
		for (int setting : {0, 1, 2}) {
			dynamics_mode = dynamic; airmode_setting = setting;
			input_control_mode = {};
			input_control_mode.flag_armed = input_control_mode.flag_control_manual_enabled = input_control_mode.flag_control_rates_enabled = true;
			ActuatorEffectivenessVoliroT effectiveness(nullptr);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
			matrix::Vector<float, 6> demand;
			demand(5) = -.04f; demand(1) = .05f;

			for (int i = 0; i < 150; ++i) {
				effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
				effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
			}

			voliro_t::Result actual;

			for (int i = 0; i < 5; ++i) { actual.thrust[i] = i == 4 ? voliro_t::tailThrust(g, output(i)) : output(i) * output(i) * g.front_max; }

			voliro_t::forward(g, actual);
			control_allocator_status_s status{}; effectiveness.getUnallocatedControl(0, status);
			check(fabsf(status.unallocated_thrust[2] - (-6.72f - actual.force(2)) / 168.f) < 1e-6f,
			      "AirMode force residual reports boost relative to ORIGINAL manual demand");
			check(fabsf(status.unallocated_torque[1] - (.5f - actual.torque(1)) / 10.f) < 1e-6f,
			      "AirMode anti-windup residual uses FINAL protected outputs");

			if (setting) {
				check(output(4) >= 0.f && -actual.force(2) <= 8.4001f && status.unallocated_torque[1] > .02f,
				      "production AirMode caps lift increase and reports infeasible pitch instead of uncontrolled thrust");

			} else {
				check(output(4) < 0.f && -actual.force(2) < 10.f, "AirMode disabled retains original low-throttle allocation");
			}
		}
	}

	airmode_setting = 1; dynamics_mode = true;

	// Missing, stale/future, disarmed and climb/position/automatic control do
	// not acquire the manual AirMode boost. These cases exercise the wrapper.
	for (int invalid = 0; invalid < 9; ++invalid) {
		input_control_mode = {};
		input_control_mode.flag_armed = input_control_mode.flag_control_manual_enabled = input_control_mode.flag_control_rates_enabled = true;
		control_mode_available = invalid != 0; refresh_control_mode = invalid < 1 || invalid > 2;

		if (invalid == 1) { input_control_mode.timestamp = test_time - 2000000; }

		if (invalid == 2) { input_control_mode.timestamp = test_time + 10000000; }

		if (invalid == 3) { input_control_mode.flag_armed = false; }

		if (invalid == 4) { input_control_mode.flag_control_manual_enabled = false; }

		if (invalid == 5) { input_control_mode.flag_control_climb_rate_enabled = true; }

		if (invalid == 6) { input_control_mode.flag_control_position_enabled = true; }

		if (invalid == 7) { input_control_mode.flag_control_auto_enabled = true; }

		if (invalid == 8) { input_control_mode.flag_control_offboard_enabled = true; }

		ActuatorEffectivenessVoliroT effectiveness(nullptr);
		effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		matrix::Vector<float, 6> demand; demand(5) = -.04f; demand(1) = .05f;

		for (int i = 0; i < 100; ++i) {
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(.01f, 0, output, true);
		}

		check(output(4) < 0.f && output(0) < .25f, "non-manual, stale and disarmed requests retain non-AirMode allocation");
	}

	airmode_setting = 0; log_geometry = false; control_mode_available = false; refresh_control_mode = true;
	std::puts("PASS: production VC_AIRMODE=0/1/2, both VT modes, manual gating, stale/future/disarmed modes and final-output residuals");
}


static void test_throttle_force_authority()
{
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = minimum(5) = -1.f;
	log_geometry = true; dynamics_mode = true; airmode_setting = 1;
	control_mode_available = refresh_control_mode = true; land_available = refresh_land = true; clock_step = 0;
	input_control_mode = {};
	input_control_mode.flag_armed = input_control_mode.flag_control_manual_enabled = input_control_mode.flag_control_rates_enabled = true;
	const voliro_t::Geometry g{.02f, .45f, 0.f, .74f, 39.5f, 10.f, .02f, .6f};

	for (int phase = 0; phase < 6; ++phase) {
		input_land = {};

		if (phase == 0) { input_land.landed = input_land.ground_contact = true; }

		if (phase == 1) { input_land.ground_contact = true; }

		land_available = phase != 3;
		refresh_land = phase != 4 && phase != 5;

		if (phase == 4) { input_land.timestamp = test_time - 2000000; }

		if (phase == 5) { input_land.timestamp = test_time + 10000000; }

		ActuatorEffectivenessVoliroT effectiveness(nullptr);
		effectiveness.updateSetpointAfterSlew(.0025f, 0, output, true);
		matrix::Vector<float, 6> demand;

		for (float collective : {.04f, .3f, .04f, 0.f}) {
			demand(5) = -collective; demand(1) = collective == .3f ? 0.f : .05f;

			for (int i = 0; i < 300; ++i) {
				test_time += 2500;
				effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);

				// Emulate an excessive retained generic motor-slew output on
				// throttle reduction; final command limiting must still apply.
				if (i == 0 && collective < .1f) { for (int j = 0; j < 4; ++j) { output(j) = .9f; } }

				effectiveness.updateSetpointAfterSlew(.0025f, 0, output, true);
				float lift = voliro_t::tailThrust(g, output(4));

				for (int j = 0; j < 4; ++j) { lift += output(j) * output(j) * g.front_max; }

				const float budget = phase == 2 ? math::min(.02f, .25f * collective) : 0.f;
				check(lift <= 168.f * (collective + budget) + 1e-3f,
				      "ground/stale states prohibit boost and airborne final outputs obey pilot-relative force ceiling");

				if (collective == 0.f) { check(fabsf(lift) < 1e-5f, "ground/zero-force feedback cannot spin motors into lift"); }

				if (i == 299 && collective == .3f) { check(fabsf(lift - 50.4f) < .02f, "raising pilot throttle does not add a remembered boost"); }
			}
		}
	}

	// Hold the pilot throttle fixed and sweep all attitude torque axes.
	// A yaw or roll request must not become a large collective increase.
	land_available = refresh_land = true; input_land = {};

	for (int mode : {1, 2}) {
		airmode_setting = mode;
		ActuatorEffectivenessVoliroT effectiveness(nullptr);
		effectiveness.updateSetpointAfterSlew(.0025f, 0, output, true);

		for (int axis = 0; axis < 3; ++axis) {
			for (float moment : {-5.f, -.5f, 0.f, .5f, 5.f}) {
				matrix::Vector<float, 6> demand; demand(5) = -.16f; demand(axis) = moment / 10.f;

				for (int i = 0; i < 100; ++i) {
					test_time += 2500;
					effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
					effectiveness.updateSetpointAfterSlew(.0025f, 0, output, true);
					float lift = voliro_t::tailThrust(g, output(4));

					for (int j = 0; j < 4; ++j) { lift += output(j) * output(j) * g.front_max; }

					check(lift <= .18001f * 168.f, "fixed-throttle Roll/Pitch/Yaw correction obeys the same collective ceiling");
				}
			}
		}
	}
	land_available = false; refresh_land = true; clock_step = 10000; control_mode_available = false;
	log_geometry = false; dynamics_mode = false; airmode_setting = 0;
	puts("PASS: ground/airborne/stale lift budgets, throttle up/down, zero force, final slew override and no remembered lift floor");
}

static void test_real_takeoff_replay(const char *path)
{
	FILE *input = fopen(path, "r"); check(input != nullptr, "takeoff replay fixture opens");
	char line[512]; check(fgets(line, sizeof(line), input) != nullptr, "takeoff replay header");
	using Vec = ActuatorEffectiveness::ActuatorVector;
	Vec minimum, maximum, output;

	for (int i = 0; i < 16; ++i) { minimum(i) = 0.f; maximum(i) = 1.f; }

	minimum(4) = minimum(5) = -1.f;

	for (int corrected_curve = 0; corrected_curve < 2; ++corrected_curve) {
		rewind(input); fgets(line, sizeof(line), input);
		log_geometry = true; tail_reverse_limit = 0.f; dynamics_mode = true; airmode_setting = 1;
		control_mode_available = refresh_control_mode = true; land_available = refresh_land = true; clock_step = 0;
		input_control_mode = {};
		input_control_mode.flag_armed = input_control_mode.flag_control_manual_enabled = input_control_mode.flag_control_rates_enabled = true;
		input_land = {};
		ActuatorEffectivenessVoliroT effectiveness(nullptr);
		effectiveness.updateSetpointAfterSlew(.0025f, 0, output, true);
		FILE *trace = nullptr;

		if (const char *dir = getenv("VOLIRO_REPLAY_OUTPUT")) {
			char output_path[512]; snprintf(output_path, sizeof(output_path), "%s/takeoff_replay_%d.csv", dir, corrected_curve);
			trace = fopen(output_path, "w"); check(trace != nullptr, "replay trace output opens");
			fputs("time_s,stick,logged_request,new_request,logged_allocated,new_allocated,force_limit\n", trace);
		}

		float previous = .23f, max_before = 0.f, max_after = 0.f, max_excess = 0.f;
		unsigned samples = 0;

		while (fgets(line, sizeof(line), input)) {
			float t, stick, collective, tx, ty, tz, landed, maybe, ground, before;
			check(sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &stick, &collective, &tx, &ty, &tz, &landed, &maybe, &ground, &before) == 10,
			      "takeoff replay row schema");
			const float dt = math::constrain(t - previous, .001f, .02f); previous = t;
			test_time += static_cast<uint64_t>(dt * 1e6f);
			input_land.landed = landed > .5f; input_land.maybe_landed = maybe > .5f; input_land.ground_contact = ground > .5f;
			const float requested = corrected_curve ? vector_control::voliroManualThrust(stick, 0, .2f, landed > .5f ? 0.f : .04f, 1.f) : collective;
			matrix::Vector<float, 6> demand; demand(0) = tx / 10.f; demand(1) = ty / 10.f; demand(2) = tz / 10.f; demand(5) = -requested;
			effectiveness.updateSetpoint(demand, 0, output, minimum, maximum);
			effectiveness.updateSetpointAfterSlew(dt, 0, output, true);
			float allocated = output(4) * fabsf(output(4)) * 10.f;

			for (int i = 0; i < 4; ++i) { allocated += output(i) * output(i) * 39.5f; }

			allocated /= 168.f;
			const float budget = !input_land.landed && !input_land.maybe_landed && !input_land.ground_contact ? math::min(.02f, .25f * requested) : 0.f;
			check(allocated <= requested + budget + 1e-5f, "recorded takeoff torques cannot override the bounded pilot collective");
			max_before = math::max(max_before, before); max_after = math::max(max_after, allocated);
			max_excess = math::max(max_excess, allocated - requested); ++samples;

			if (trace) { fprintf(trace, "%.6f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n", (double)t, (double)stick, (double)collective, (double)requested, (double)before, (double)allocated, (double)(requested + budget)); }
		}

		if (trace) { fclose(trace); }

		check(samples > 300 && max_before > .3f
		      && max_after < .13f, "actual-log fixture reproduces old runaway command and bounds the corrected command");
		printf("PASS real takeoff INPUT replay: curve=%d rows=%u old_peak=%.6f new_peak=%.6f max_excess=%.6f\n", corrected_curve, samples,
		       (double)max_before, (double)max_after, (double)max_excess);
	}

	fclose(input); tail_reverse_limit = .6f; clock_step = 10000; land_available = false; control_mode_available = false;
	log_geometry = false; dynamics_mode = false; airmode_setting = 0;
}

#include "test_voliro_airmode_closed_loop.hpp"

int main(int argc, char **argv)
{
	if (argc == 3 && strcmp(argv[1], "--replay") == 0) { test_real_takeoff_replay(argv[2]); return 0; }

#ifdef VOLIRO_AIRMODE_JSBSIM

	if (argc == 5 && strcmp(argv[1], "--native") == 0) {
		for (bool dynamic : {false, true}) {
			for (int setting : {0, 1, 2}) {
				char path[512]; snprintf(path, sizeof(path), "%s/native_%d_%d.csv", argv[4], dynamic, setting);
				FILE *trace = fopen(path, "w");
				check(trace != nullptr, "native trace output opens");
				fputs("time_s,target_pitch_deg,pitch_deg,pitch_rate_deg_s,tail_command,requested_pitch_nm,residual_pitch_nm,collective_command,front_thrust_n\n",
				      trace);
				const auto metrics = run_manual_airmode_loop(setting, dynamic, .05f, 1.f, trace, argv[2], argv[3]);
				fclose(trace);

				if (!setting) { check(metrics.finite && metrics.final_rms_deg > 20.f, "native baseline reproduces low-throttle pitch oscillation"); }

				if (setting) { check(metrics.finite && metrics.final_rms_deg < 1.f && metrics.peak_deg < 25.f, "native JSBSim manual AirMode converges"); }
			}
		}

		// Hold a near-hover pilot collective while commanding attitude steps.
		// This separately covers the reported "touch pitch and climb" case.
		for (bool dynamic : {false, true}) {
			for (int setting : {1, 2}) {
				char path[512]; snprintf(path, sizeof(path), "%s/attitude_fixed_throttle_%d_%d.csv", argv[4], dynamic, setting);
				FILE *trace = fopen(path, "w"); check(trace != nullptr, "fixed-throttle attitude trace opens");
				fputs("time_s,target_pitch_deg,pitch_deg,pitch_rate_deg_s,tail_command,requested_pitch_nm,residual_pitch_nm,collective_command,front_thrust_n\n",
				      trace);
				const auto metrics = run_manual_airmode_loop(setting, dynamic, .05f, 1.f, trace, argv[2], argv[3], .16f);
				fclose(trace);
				check(metrics.finite && metrics.final_rms_deg < 1.f
				      && metrics.peak_deg < 25.f, "fixed-throttle attitude steps converge while force remains bounded");
			}
		}
		return 0;
	}

#endif
	(void)argc; (void)argv;
	test_throttle_force_authority();
	test_manual_airmode_closed_loop();
	test_manual_airmode();
	test_configured_servo_speed();
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
	test_tilt_fallback();
	test_fixed_tilt_switch();
}
