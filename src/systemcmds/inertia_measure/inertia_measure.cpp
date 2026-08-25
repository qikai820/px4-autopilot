/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file inertia_measure.cpp
 * @brief Bifilar-pendulum inertia measurement using EKF2 attitude.
 */

#include <drivers/drv_hrt.h>
#include <lib/systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

#include <cmath>
#include <cstring>

using namespace matrix;

extern "C" __EXPORT int inertia_measure_main(int argc, char *argv[]);

namespace
{
constexpr float kGravity = 9.80665f;
constexpr float kVerticalAxisAlignment = 0.94f;
constexpr float kStartAmplitudeRad = 0.05236f; // 3 degrees
constexpr float kSettledAmplitudeRad = 0.01745f; // 1 degree
constexpr int kRequiredHalfPeriods = 8;
constexpr uint64_t kTimeoutUs = 120000000ULL;
constexpr uint64_t kSettledHoldUs = 500000ULL;

const char *axis_name(unsigned axis)
{
	static constexpr const char *names[] = {"X (roll)", "Y (pitch)", "Z (yaw)"};
	return names[axis];
}

param_t result_param(unsigned axis)
{
	static const char *names[] = {"INR_IXX", "INR_IYY", "INR_IZZ"};
	return param_find(names[axis]);
}

void usage(const char *reason = nullptr)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Measures a principal moment of inertia with a two-rope (bifilar) pendulum.
The vehicle must be disarmed, props removed, gimbals locked, and suspended
from two equal, parallel ropes. It uses EKF2 vehicle_attitude to identify the
body axis that is vertical and estimates the torsional period from zero
crossings after the vehicle is twisted and released.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("inertia_measure", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("measure", "Capture INR_CNT releases, print each result, and save their mean");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show setup and saved IXX/IYY/IZZ results");
	PRINT_MODULE_USAGE_COMMAND_DESCR("apply", "Copy saved results into GC_INERTIA_XX/YY/ZZ");
}

bool get_float_parameter(const char *name, float &value)
{
	const param_t param = param_find(name);
	return param != PARAM_INVALID && param_get(param, &value) == PX4_OK;
}

int show_status()
{
	float mass{}, distance{}, length{}, ixx{}, iyy{}, izz{};
	int32_t count{};
	if (!get_float_parameter("INR_MASS", mass) || !get_float_parameter("INR_DIST", distance)
	    || !get_float_parameter("INR_LEN", length) || !get_float_parameter("INR_IXX", ixx)
	    || !get_float_parameter("INR_IYY", iyy) || !get_float_parameter("INR_IZZ", izz)
	    || param_get(param_find("INR_CNT"), &count) != PX4_OK) {
		PX4_ERR("inertia measurement parameters are unavailable");
		return PX4_ERROR;
	}

	PX4_INFO("setup: mass=%.3f kg, rope distance=%.3f m, rope length=%.3f m, count=%d", (double)mass,
		 (double)distance, (double)length, (int)count);
	PX4_INFO("saved: IXX=%.6f, IYY=%.6f, IZZ=%.6f kg m^2", (double)ixx, (double)iyy, (double)izz);
	return PX4_OK;
}

int apply_results()
{
	static const char *source_names[] = {"INR_IXX", "INR_IYY", "INR_IZZ"};
	static const char *destination_names[] = {"GC_INERTIA_XX", "GC_INERTIA_YY", "GC_INERTIA_ZZ"};

	for (unsigned i = 0; i < 3; ++i) {
		float value{};
		const param_t source = param_find(source_names[i]);
		const param_t destination = param_find(destination_names[i]);

		if (source == PARAM_INVALID || destination == PARAM_INVALID || param_get(source, &value) != PX4_OK || value <= 0.f) {
			PX4_ERR("valid saved result required for %s", source_names[i]);
			return PX4_ERROR;
		}

		if (param_set(destination, &value) != PX4_OK) {
			PX4_ERR("could not set %s", destination_names[i]);
			return PX4_ERROR;
		}
	}

	PX4_INFO("copied saved inertia results to geometric-controller parameters");
	return PX4_OK;
}

int measure()
{
	orb_advert_t mavlink_log_pub = nullptr;
	vehicle_status_s vehicle_status{};
	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};

	if (vehicle_status_sub.copy(&vehicle_status) && vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		PX4_ERR("disarm the vehicle and remove props before measuring");
		return PX4_ERROR;
	}

	float mass{}, distance{}, length{};
	int32_t measurement_count{};
	if (!get_float_parameter("INR_MASS", mass) || !get_float_parameter("INR_DIST", distance)
	    || !get_float_parameter("INR_LEN", length) || param_get(param_find("INR_CNT"), &measurement_count) != PX4_OK
	    || mass < 0.05f || distance < 0.02f || length < 0.10f || measurement_count < 1 || measurement_count > 20) {
		PX4_ERR("set INR_MASS, INR_DIST, INR_LEN, and INR_CNT to valid values first");
		return PX4_ERROR;
	}

	uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)};
	vehicle_attitude_s attitude{};
	const uint64_t wait_until = hrt_absolute_time() + 3000000ULL;

	while (!attitude_sub.update(&attitude) && hrt_absolute_time() < wait_until) {
		px4_usleep(10000);
	}

	if (attitude.timestamp == 0) {
		PX4_ERR("no EKF2 vehicle_attitude received");
		return PX4_ERROR;
	}

	const Quatf q_reference{attitude.q};
	const Dcmf rotation{q_reference}; // body to NED
	unsigned axis = 0;
	float best_alignment = fabsf(rotation(2, 0));

	for (unsigned i = 1; i < 3; ++i) {
		const float alignment = fabsf(rotation(2, i));

		if (alignment > best_alignment) {
			best_alignment = alignment;
			axis = i;
		}
	}

	if (best_alignment < kVerticalAxisAlignment) {
		PX4_ERR("no body axis is vertical (best alignment %.2f); align a body axis with gravity", (double)best_alignment);
		return PX4_ERROR;
	}

	mavlink_log_info(&mavlink_log_pub, "IM axis %s, count %d", axis_name(axis), (int)measurement_count);
	mavlink_log_info(&mavlink_log_pub, "IM 1/%d: twist >3 deg, release", (int)measurement_count);

	bool departed_from_centre = false;
	bool have_previous = false;
	bool have_crossing = false;
	float previous_angle = 0.f;
	uint64_t previous_timestamp = 0;
	float previous_crossing_s = 0.f;
	float half_period_sum_s = 0.f;
	int half_period_count = 0;
	uint64_t timeout = hrt_absolute_time() + kTimeoutUs;
	uint64_t settled_since = 0;
	bool waiting_for_settle = false;
	int completed_measurements = 0;
	float inertia_mean = 0.f;
	float inertia_m2 = 0.f;

	while (completed_measurements < measurement_count) {
		if (hrt_absolute_time() >= timeout) {
			PX4_ERR("measurement %d/%d timed out", completed_measurements + 1, (int)measurement_count);
			return PX4_ERROR;
		}

		if (!attitude_sub.update(&attitude)) {
			px4_usleep(5000);
			continue;
		}

		Quatf q_delta = q_reference.inversed() * Quatf{attitude.q};

		if (q_delta(0) < 0.f) {
			q_delta = Quatf{-q_delta(0), -q_delta(1), -q_delta(2), -q_delta(3)};
		}

		const float angle = 2.f * atan2f(q_delta(axis + 1), q_delta(0));

		if (waiting_for_settle) {
			if (fabsf(angle) <= kSettledAmplitudeRad) {
				if (settled_since == 0) {
					settled_since = attitude.timestamp;

				} else if (attitude.timestamp - settled_since >= kSettledHoldUs) {
					waiting_for_settle = false;
					settled_since = 0;
					timeout = hrt_absolute_time() + kTimeoutUs;
					have_previous = false;
					have_crossing = false;
					departed_from_centre = false;
					half_period_count = 0;
					half_period_sum_s = 0.f;
					mavlink_log_info(&mavlink_log_pub, "IM %d/%d ready: twist >3 deg, release",
						 completed_measurements + 1, (int)measurement_count);
				}

			} else {
				settled_since = 0;
			}

			continue;
		}

		if (fabsf(angle) >= kStartAmplitudeRad) {
			departed_from_centre = true;
		}

		if (have_previous && departed_from_centre && ((previous_angle <= 0.f && angle > 0.f)
				|| (previous_angle >= 0.f && angle < 0.f))) {
			const float denominator = angle - previous_angle;
			const float fraction = fabsf(denominator) > 1e-6f ? -previous_angle / denominator : 0.f;
			const float crossing_s = (float)previous_timestamp * 1e-6f
						 + fraction * (float)(attitude.timestamp - previous_timestamp) * 1e-6f;

			if (have_crossing) {
				const float half_period_s = crossing_s - previous_crossing_s;

				if (half_period_s > 0.05f && half_period_s < 20.f) {
					half_period_sum_s += half_period_s;
					half_period_count++;
					PX4_INFO("half period %d/%d: %.3f s", half_period_count, kRequiredHalfPeriods, (double)half_period_s);

					if (half_period_count == kRequiredHalfPeriods) {
						const float period_s = 2.f * half_period_sum_s / (float)half_period_count;
						const float half_rope_distance = distance * 0.5f;
						const float inertia = mass * kGravity * half_rope_distance * half_rope_distance * period_s * period_s
								      / (4.f * M_PI_F * M_PI_F * length);

						if (!PX4_ISFINITE(inertia) || inertia <= 0.f) {
							PX4_ERR("invalid inertia calculation");
							return PX4_ERROR;
						}

						completed_measurements++;
						const float delta = inertia - inertia_mean;
						inertia_mean += delta / (float)completed_measurements;
						inertia_m2 += delta * (inertia - inertia_mean);
						const float standard_deviation = completed_measurements > 1
									 ? sqrtf(inertia_m2 / (float)(completed_measurements - 1)) : 0.f;

						mavlink_log_info(&mavlink_log_pub, "IM %d/%d T %.3f I %.5f avg %.5f", completed_measurements,
							 (int)measurement_count, (double)period_s, (double)inertia, (double)inertia_mean);
						PX4_INFO("IM %d/%d std %.6f kg m^2", completed_measurements, (int)measurement_count,
							 (double)standard_deviation);

						if (completed_measurements == measurement_count) {
							const param_t param = result_param(axis);

							if (param == PARAM_INVALID || param_set(param, &inertia_mean) != PX4_OK) {
								PX4_ERR("could not save inertia result");
								return PX4_ERROR;
							}

							mavlink_log_info(&mavlink_log_pub, "IM complete: %s = %.6f", axis_name(axis), (double)inertia_mean);
							PX4_INFO("All %d measurements complete; saved to %s", (int)measurement_count,
								 axis == 0 ? "INR_IXX" : axis == 1 ? "INR_IYY" : "INR_IZZ");

						} else {
							waiting_for_settle = true;
							settled_since = 0;
							mavlink_log_info(&mavlink_log_pub, "IM %d/%d done: wait until settled", completed_measurements,
								 (int)measurement_count);
							PX4_INFO("Next prompt will request measurement %d/%d", completed_measurements + 1,
								 (int)measurement_count);
						}
					}
				}
			}

			previous_crossing_s = crossing_s;
			have_crossing = true;
			departed_from_centre = false;
		}

		previous_angle = angle;
		previous_timestamp = attitude.timestamp;
		have_previous = true;
	}

	return PX4_OK;
}
} // namespace

int inertia_measure_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return PX4_ERROR;
	}

	if (!strcmp(argv[1], "measure")) {
		return measure();
	}

	if (!strcmp(argv[1], "status")) {
		return show_status();
	}

	if (!strcmp(argv[1], "apply")) {
		return apply_results();
	}

	usage("unknown command");
	return PX4_ERROR;
}
