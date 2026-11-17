// Real control modules/uORB; synthetic motion isolates the monitor/mode gates.
#include "RuntimeTestTransport.hpp"
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vector_actuator_status.h>
#include <uORB/topics/vector_attitude_reference.h>
#include <uORB/topics/voliro_tilt_status.h>
#include <uORB/topics/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <matrix/matrix/math.hpp>
#include <cstdio>
#include <cstring>
#include <unistd.h>

extern "C" __EXPORT int px4_module_main(int, char **)
{
	Publisher<vehicle_attitude_s> attitude_pub{ORB_ID(vehicle_attitude)};
	Publisher<vehicle_angular_velocity_s> gyro_pub{ORB_ID(vehicle_angular_velocity)};
	Publisher<vehicle_acceleration_s> acceleration_pub{ORB_ID(vehicle_acceleration)};
	Publisher<vehicle_control_mode_s> mode_pub{ORB_ID(vehicle_control_mode)};
	Publisher<vehicle_constraints_s> constraints_pub{ORB_ID(vehicle_constraints)};
	Publisher<vehicle_status_s> status_pub{ORB_ID(vehicle_status)};
	Publisher<vehicle_land_detected_s> land_pub{ORB_ID(vehicle_land_detected)};
	Publisher<vehicle_local_position_s> position_pub{ORB_ID(vehicle_local_position)};
	Publisher<trajectory_setpoint_s> trajectory_pub{ORB_ID(trajectory_setpoint)};
	Publisher<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
	Publisher<rc_channels_s> rc_pub{ORB_ID(rc_channels)};
	Publisher<vector_actuator_status_s> actuator_pub{ORB_ID(vector_actuator_status)};
	Subscription tilt_sub{ORB_ID(voliro_tilt_status)};
	Subscription reference_sub{ORB_ID(vector_attitude_reference)};
	Subscription log_sub{ORB_ID(mavlink_log)};
	bool normal = false, inhibited = false, vector = false, fault = false, blocked_notice = false, reset = false, reentered = false;
	bool unexpected_fault = false;

	for (int k = 0; k < 950; ++k) {
		const uint64_t simulation_time = 1000000 + uint64_t(k) * 10000;
		timespec ts{}; abstime_to_ts(&ts, simulation_time); px4_clock_settime(CLOCK_MONOTONIC, &ts);
		const uint64_t now = hrt_absolute_time();
		const bool fast = k >= 100 && k < 250;
		const bool mismatch = k >= 500 && k < 700;
		const bool armed = !(k >= 750 && k < 800);
		const bool vector_requested = k >= 350 && !(k >= 700 && k < 725);
		vehicle_control_mode_s mode{}; mode.timestamp = now; mode.flag_armed = armed;
		mode.flag_control_manual_enabled = true;
		mode.flag_control_position_enabled = mode.flag_control_velocity_enabled = true;
		mode.flag_control_altitude_enabled = mode.flag_control_climb_rate_enabled = true;
		mode.flag_control_attitude_enabled = mode.flag_control_rates_enabled = true;
		mode.flag_multicopter_position_control_enabled = true; mode_pub.publish(mode);
		vehicle_status_s status{}; status.timestamp = now;
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		status_pub.publish(status);
		vehicle_land_detected_s land{}; land.timestamp = now; land.landed = land.ground_contact = !armed; land_pub.publish(land);
		vehicle_constraints_s constraints{}; constraints.timestamp = now; constraints.want_takeoff = armed;
		constraints.speed_up = constraints.speed_down = 1.f; constraints_pub.publish(constraints);
		manual_control_setpoint_s manual{}; manual.timestamp = manual.timestamp_sample = now;
		manual.valid = true; manual.data_source = manual_control_setpoint_s::SOURCE_RC; manual.aux1 = .5f; manual_pub.publish(manual);
		rc_channels_s rc{}; rc.timestamp = now; rc.channel_count = 18; rc.channels[7] = vector_requested ? 1.f : -1.f; rc_pub.publish(rc);
		trajectory_setpoint_s trajectory{}; trajectory.timestamp = now; trajectory.position[2] = -5.f; trajectory_pub.publish(trajectory);
		vehicle_attitude_s attitude{}; attitude.timestamp = attitude.timestamp_sample = now;
		matrix::Quatf{}.copyTo(attitude.q); attitude_pub.publish(attitude);
		vehicle_angular_velocity_s gyro{}; gyro.timestamp = gyro.timestamp_sample = now; gyro_pub.publish(gyro);
		vehicle_acceleration_s acceleration{}; acceleration.timestamp = acceleration.timestamp_sample = now;
		acceleration.xyz[0] = fast || mismatch ? -3.5f : 0.f; acceleration.xyz[2] = -9.81f; acceleration_pub.publish(acceleration);
		vector_actuator_status_s actuator{}; actuator.timestamp = now; actuator.armed = armed;
		actuator_pub.publish(actuator);
		vehicle_local_position_s position{}; position.timestamp = position.timestamp_sample = now;
		position.z = -5.f; position.vx = fast ? 17.f : 0.f;
		position.xy_valid = position.z_valid = position.v_xy_valid = position.v_z_valid = true;
		position_pub.publish(position);
		usleep(10000);
		voliro_tilt_status_s tilt{}; vector_attitude_reference_s reference{}; mavlink_log_s log{};

		if (tilt_sub.copy(&tilt)) {
			if (k < 500 && tilt.fallback) { unexpected_fault = true; }

			if (k > 60 && k < 100 && tilt.fixed_tilt) { normal = true; }

			if (fast && !tilt.motion_valid && !tilt.observation_valid && !tilt.fallback) { inhibited = true; }

			if (k >= 600 && k < 750 && tilt.fallback) { fault = true; }

			if (!armed && !tilt.fallback) { reset = true; }

			if (reference_sub.copy(&reference) && reference.valid && !tilt.fallback && !tilt.fixed_tilt) {
				if (k > 400 && k < 500 && reference.pitch > .03f) { vector = true; }

				if (k > 850 && reference.pitch > .03f) { reentered = true; }
			}
		}

		if (k >= 700 && log_sub.copy(&log) && std::strstr(reinterpret_cast<const char *>(log.text), "tilt fault latched")) {
			blocked_notice = true;
		}
	}

	printf("Voliro modes: normal=%d high-speed-inhibited=%d unexpected-fault=%d vector=%d real-fault=%d notice=%d reset=%d reentry=%d\n",
	       normal, inhibited, unexpected_fault, vector, fault, blocked_notice, reset, reentered);
	const bool ok = normal && inhibited && !unexpected_fault && vector && fault && blocked_notice && reset && reentered;
	puts(ok ? "VOLIRO_MODES_RUNTIME_OK" : "VOLIRO_MODES_RUNTIME_FAILED");
	return ok ? 0 : 1;
}
