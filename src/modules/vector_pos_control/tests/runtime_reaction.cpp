// Actual rate module/uORB wiring, isolated from hardware and MAVLink.
#include "RuntimeTestTransport.hpp"
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vector_actuator_status.h>
#include <uORB/topics/vector_rate_control_status.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <cmath>
#include <cstdio>
#include <unistd.h>

extern "C" __EXPORT int px4_module_main(int, char **)
{
	int32_t airframe = 0;
	param_get(param_find("CA_AIRFRAME"), &airframe);
	Publisher<vehicle_control_mode_s> mode_pub{ORB_ID(vehicle_control_mode)};
	Publisher<vehicle_status_s> vehicle_pub{ORB_ID(vehicle_status)};
	Publisher<vehicle_land_detected_s> land_pub{ORB_ID(vehicle_land_detected)};
	Publisher<vehicle_rates_setpoint_s> rates_pub{ORB_ID(vehicle_rates_setpoint)};
	Publisher<vehicle_angular_velocity_s> gyro_pub{ORB_ID(vehicle_angular_velocity)};
	Publisher<vector_actuator_status_s> actuator_pub{ORB_ID(vector_actuator_status)};
	Subscription result_sub{ORB_ID(vector_rate_control_status)};
	unsigned checked = 0, failed = 0;

	for (unsigned k = 0; k < 350; ++k) {
		timespec ts{}; abstime_to_ts(&ts, 1000000 + k * 10000);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
		const auto now = hrt_absolute_time();
		vehicle_control_mode_s mode{}; mode.timestamp = now;
		mode.flag_armed = mode.flag_control_rates_enabled = true; mode_pub.publish(mode);
		vehicle_status_s vehicle{}; vehicle.timestamp = now;
		vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED; vehicle_pub.publish(vehicle);
		vehicle_land_detected_s land{}; land.timestamp = now; land_pub.publish(land);
		vehicle_rates_setpoint_s rates{}; rates.timestamp = now;
		rates.roll = .2f; rates.yaw = .3f; rates.thrust_body[2] = -.2f; rates_pub.publish(rates);
		vector_actuator_status_s actuator{}; actuator.timestamp = now; actuator.armed = true;
		actuator.tilt_command = k < 50 || k >= 300 ? 0.f
				       : k >= 250 ? -2.7925268f : k >= 200 ? 2.7925268f : .78539816f;

		// Stop publications long enough to become stale; retain previous shaping.
		if (k < 100 || k >= 170) { actuator_pub.publish(actuator); }

		vehicle_angular_velocity_s gyro{}; gyro.timestamp = gyro.timestamp_sample = now; gyro_pub.publish(gyro);
		usleep(10000);

		if (k % 50 > 10) {
			vector_rate_control_status_s result{};
			const float expected = airframe == 17 && k >= 50 && k < 300 ? .5f : 1.f;
			++checked;

			if (!result_sub.copy(&result) || result.timestamp < now - 50000
			    || fabsf(result.reaction_p_scale - expected) > 1e-5f
			    || (airframe == 17 && k >= 200 && k < 300
				&& fabsf(result.reaction_tilt - actuator.tilt_command) > 1e-5f)) { ++failed; }
		}
	}

	printf("Reaction runtime CA=%ld: checked=%u failed=%u (fixed/tilted/stale/inverted/restored)\n", long(airframe), checked, failed);
	puts(failed == 0 ? "REACTION_RUNTIME_OK" : "REACTION_RUNTIME_FAILED");
	return failed == 0 ? 0 : 1;
}
