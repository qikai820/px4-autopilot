// Loaded only by runtime_reference.py into an isolated SITL process. There are
// no sensors, commander, land detector, MAVLink links or hardware drivers.
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vector_attitude_reference.h>
#include <uORB/topics/vector_rate_reference.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <cstdio>
#include <unistd.h>

#include "RuntimeTestTransport.hpp"

extern "C" __EXPORT int px4_module_main(int, char **)
{
	Publisher<vehicle_attitude_s> attitude_pub{ORB_ID(vehicle_attitude)};
	Publisher<vehicle_angular_velocity_s> gyro_pub{ORB_ID(vehicle_angular_velocity)};
	Publisher<vehicle_control_mode_s> mode_pub{ORB_ID(vehicle_control_mode)};
	Publisher<vehicle_constraints_s> constraints_pub{ORB_ID(vehicle_constraints)};
	Publisher<vehicle_status_s> status_pub{ORB_ID(vehicle_status)};
	Publisher<vehicle_land_detected_s> land_pub{ORB_ID(vehicle_land_detected)};
	Publisher<vehicle_local_position_s> position_pub{ORB_ID(vehicle_local_position)};
	Publisher<trajectory_setpoint_s> trajectory_pub{ORB_ID(trajectory_setpoint)};
	Publisher<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
	Publisher<rc_channels_s> rc_pub{ORB_ID(rc_channels)};
	Subscription reference_sub{ORB_ID(vector_attitude_reference)};
	Subscription rate_reference_sub{ORB_ID(vector_rate_reference)};
	Subscription local_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	Subscription attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	float peak_bank = 0.f, peak_pitch = 0.f, final_bank = 1.f, final_pitch = 1.f;
	bool valid_before_reset = false, valid_after_reset = false, valid_rate_reference = false;
#ifdef VECTOR_TEST_SPATIAL_LARGE
	constexpr int steps = 1200, reset_step = 300, reverse_step = 400, loss_step = 850;
	float min_bank = 0.f, max_pitch = 0.f;
	matrix::Quatf actual{};
#else
	constexpr int steps = 700, reset_step = 300, loss_step = 450;
#endif

	for (int k = 0; k < steps; ++k) {
		// Supply the simulator clock as well as topics in lockstep SITL.
		const uint64_t simulation_time = 1000000 + uint64_t(k) * 10000;
		timespec ts{}; abstime_to_ts(&ts, simulation_time);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
		const uint64_t now = hrt_absolute_time();
		const bool reset = k >= reset_step;
		vehicle_control_mode_s mode{};
		mode.timestamp = now;
		mode.flag_armed = true; mode.flag_control_manual_enabled = true;
		mode.flag_control_position_enabled = mode.flag_control_velocity_enabled = true;
		mode.flag_control_altitude_enabled = mode.flag_control_climb_rate_enabled = true;
		mode.flag_control_attitude_enabled = mode.flag_control_rates_enabled = true;
		mode.flag_multicopter_position_control_enabled = true;
		mode.flag_control_allocation_enabled = true;
		mode_pub.publish(mode);
		vehicle_constraints_s constraints{}; constraints.timestamp = now; constraints.want_takeoff = true;
		constraints.speed_up = constraints.speed_down = 1.f; constraints_pub.publish(constraints);
		vehicle_status_s status{};
		status.timestamp = now; status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		status_pub.publish(status);
		vehicle_land_detected_s land{}; land.timestamp = now; land_pub.publish(land);
		manual_control_setpoint_s manual{};
		manual.timestamp = manual.timestamp_sample = now; manual.valid = k < loss_step;
		manual.data_source = manual_control_setpoint_s::SOURCE_RC;
		manual.aux1 = .5f; manual.aux2 = -.5f;
#ifdef VECTOR_TEST_SPATIAL_LARGE
		manual.aux1 = k < reverse_step ? 1.f : -1.f;
		manual.aux2 = -manual.aux1;
#endif
		manual_pub.publish(manual);
		rc_channels_s rc{}; rc.timestamp = now; rc.channel_count = 18; rc_pub.publish(rc);
		trajectory_setpoint_s trajectory{};
		trajectory.timestamp = now; trajectory.position[2] = -5.f; trajectory.yaw = reset ? .2f : 0.f;
		trajectory_pub.publish(trajectory);
		vehicle_local_position_s position{};
		position.timestamp = position.timestamp_sample = now; position.z = -5.f;
		position.xy_valid = position.z_valid = position.v_xy_valid = position.v_z_valid = true;
		position.heading = position.unaided_heading = reset ? .2f : 0.f;
		position_pub.publish(position);
		vehicle_attitude_s attitude{};
		attitude.timestamp = attitude.timestamp_sample = now;
		matrix::Quatf(matrix::Eulerf(0.f, 0.f, reset ? .2f : 0.f)).copyTo(attitude.q);
#ifdef VECTOR_TEST_SPATIAL_LARGE
		if (k == reset_step) { actual = matrix::Quatf(matrix::Eulerf(0.f, 0.f, .2f)) * actual; }

		actual.copyTo(attitude.q);
#endif
		matrix::Quatf(matrix::Eulerf(0.f, 0.f, .2f)).copyTo(attitude.delta_q_reset);
		attitude.quat_reset_counter = reset ? 1 : 0;
		attitude_pub.publish(attitude);
		vehicle_angular_velocity_s gyro{}; gyro.timestamp = gyro.timestamp_sample = now; gyro_pub.publish(gyro);
		usleep(10000);
		vector_attitude_reference_s reference{};

		if (reference_sub.copy(&reference) && reference.valid && reference.timestamp > now - 100000) {
			valid_before_reset |= !reset;
			valid_after_reset |= reset && reference.quat_reset_counter == 1;

			if (k < loss_step) { peak_bank = fmaxf(peak_bank, reference.bank); peak_pitch = fminf(peak_pitch, reference.pitch); }

#ifdef VECTOR_TEST_SPATIAL_LARGE
			min_bank = fminf(min_bank, reference.bank); max_pitch = fmaxf(max_pitch, reference.pitch);
			const matrix::Quatf next(reference.q_d);
			const float jump = matrix::AxisAnglef((actual.inversed() * next).canonical()).angle();

			if (jump > .05f || fabsf(matrix::wrap_pi(reference.heading - (reset ? .2f : 0.f))) > .03f) {
				printf("SPATIAL_REFERENCE_DISCONTINUITY k=%d jump=%f heading=%f\n", k, double(jump), double(reference.heading));
				return 1;
			}

			actual = next; // ideal tracking supplies authority; the allocator test supplies a dynamic plant.
#endif

			final_bank = reference.bank; final_pitch = reference.pitch;
		}

		vector_rate_reference_s rate_reference{};

		if (rate_reference_sub.copy(&rate_reference) && rate_reference.valid && rate_reference.quat_reset_counter == 1) {
			valid_rate_reference = true;
		}
	}

	vector_attitude_reference_s last{}; reference_sub.copy(&last);
	vehicle_local_position_setpoint_s local_sp{}; local_sp_sub.copy(&local_sp);
	vehicle_attitude_setpoint_s att_sp{}; attitude_sp_sub.copy(&att_sp);
	printf("Runtime topics: reference=%llu valid=%d, local_sp=%llu thrust=%f/%f/%f, att=%llu q=%f/%f/%f/%f\n",
	       (unsigned long long)last.timestamp, last.valid, (unsigned long long)local_sp.timestamp,
	       double(local_sp.thrust[0]), double(local_sp.thrust[1]), double(local_sp.thrust[2]),
	       (unsigned long long)att_sp.timestamp, double(att_sp.q_d[0]), double(att_sp.q_d[1]), double(att_sp.q_d[2]), double(att_sp.q_d[3]));
	printf("Runtime reference: before/after reset=%d/%d, rate=%d, roll/pitch peak=%.4f/%.4f final=%.4f/%.4f rad\n",
	       valid_before_reset, valid_after_reset, valid_rate_reference, double(peak_bank), double(peak_pitch), double(final_bank),
	       double(final_pitch));
	const bool ok = valid_before_reset && valid_after_reset && valid_rate_reference && peak_bank > .10f && peak_pitch < -.10f
			&& fabsf(final_bank) < .03f && fabsf(final_pitch) < .03f
#ifdef VECTOR_TEST_SPATIAL_LARGE
			&& peak_bank > math::radians(119.f) && peak_pitch < math::radians(-119.f)
			&& min_bank < math::radians(-119.f) && max_pitch > math::radians(119.f)
#endif
			;
#ifdef VECTOR_TEST_SPATIAL_LARGE
	printf("Spatial reference extrema: roll=%.2f/%.2f pitch=%.2f/%.2f deg\n", double(math::degrees(min_bank)),
	       double(math::degrees(peak_bank)), double(math::degrees(peak_pitch)), double(math::degrees(max_pitch)));
#endif
	puts(ok ? "VECTOR_REFERENCE_RUNTIME_OK" : "VECTOR_REFERENCE_RUNTIME_FAILED");
	return ok ? 0 : 1;
}
