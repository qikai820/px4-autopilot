/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class GeoControl : public ModuleBase<GeoControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GeoControl();
	~GeoControl() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static GeoControl *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class ControlMode {
		MANUAL = 0,    ///< RC sticks: roll/pitch/yaw → rate, aux1/2/3 → velocity
		OFFBOARD = 1,  ///< trajectory_setpoint from companion computer
		AUTO = 2       ///< flight_mode_manager setpoints (posctl, loiter, mission, ...)
	};

	void Run() override;
	void parameters_update(bool force = false);
	void control_step();

	/**
	 * Manual mode: RC sticks → angular rate + velocity references
	 * roll/pitch/yaw → body-frame angular rate [rad/s]
	 * aux1/aux2/aux3 → body-frame velocity [m/s]
	 */
	void control_step_manual(const matrix::Quatf &R_q, const matrix::Vector3f &vel,
				 const matrix::Vector3f &omega, uint64_t timestamp_sample);

	/**
	 * Offboard mode: trajectory_setpoint → geometric control law
	 */
	void control_step_offboard(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
				   const matrix::Vector3f &vel, const matrix::Vector3f &omega,
				   uint64_t timestamp_sample);

	/**
	 * Auto mode: flight_mode_manager setpoints → geometric control law
	 */
	void control_step_auto(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
			       const matrix::Vector3f &vel, const matrix::Vector3f &omega,
			       uint64_t timestamp_sample);

	/**
	 * Publish thrust and torque setpoints.
	 */
	void publish_wrench(const matrix::Vector3f &f_des_body, const matrix::Vector3f &tau_des,
			    uint64_t timestamp_sample);

	// uORB Subscriptions
	uORB::SubscriptionCallbackWorkItem _sub_vehicle_local_position{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};
	uORB::Subscription _sub_vehicle_attitude_setpoint{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _sub_vehicle_angular_velocity{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _sub_local_pos_sp{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::Subscription _sub_manual_control_sp{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _sub_trajectory_sp{ORB_ID(trajectory_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// uORB Publications
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};

	// Parameters (GC_ prefix)
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GC_MASS>) _param_gc_mass,
		(ParamFloat<px4::params::GC_XY_P>) _param_gc_xy_p,
		(ParamFloat<px4::params::GC_XY_VEL_P>) _param_gc_xy_vel_p,
		(ParamFloat<px4::params::GC_Z_P>) _param_gc_z_p,
		(ParamFloat<px4::params::GC_Z_VEL_P>) _param_gc_z_vel_p,
		(ParamFloat<px4::params::GC_RP_P>) _param_gc_rp_p,
		(ParamFloat<px4::params::GC_YAW_P>) _param_gc_yaw_p,
		(ParamFloat<px4::params::GC_RP_RATE_P>) _param_gc_rp_rate_p,
		(ParamFloat<px4::params::GC_YAW_RATE_P>) _param_gc_yaw_rate_p,
		(ParamFloat<px4::params::GC_INERTIA_XX>) _param_gc_inertia_xx,
		(ParamFloat<px4::params::GC_INERTIA_YY>) _param_gc_inertia_yy,
		(ParamFloat<px4::params::GC_INERTIA_ZZ>) _param_gc_inertia_zz,
		(ParamFloat<px4::params::GC_MAN_RATE_MAX>) _param_gc_man_rate_max,
		(ParamFloat<px4::params::GC_MAN_VEL_MAX>) _param_gc_man_vel_max,
		(ParamFloat<px4::params::GC_MAN_DZ>) _param_gc_man_dz
	)

	// Manual mode: integrated attitude reference from rate commands
	matrix::Quatf _q_att_ref{1.f, 0.f, 0.f, 0.f};

	hrt_abstime _last_run{0};
	perf_counter_t _loop_perf;
};
