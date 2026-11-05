/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Multicopter position controller.
 */

#pragma once

#include "VectorPosition/VectorPosition.hpp"
#include "VectorTakeoff/VectorTakeoff.hpp"
#include "VectorGoto/VectorGoto.hpp"
#include "DockThrustControl.hpp"
#include "DockFailsafeYaw.hpp"
#include "DockRtlVelocityRamp.hpp"
#include "DockFaultTransfer.hpp"
#include "DockClimbBrake.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/dock_brake_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vector_attitude_reference.h>
#include <uORB/topics/vector_actuator_status.h>
#include <lib/vector_control/VectorAttitudeReference.hpp>
#include <lib/vector_control/VoliroTiltMonitor.hpp>
#include <lib/vector_control/VoliroFixedTiltTransition.hpp>
#include <uORB/topics/voliro_tilt_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vector_force_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class VectorPositionControl : public ModuleBase<VectorPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	VectorPositionControl(bool vtol = false);
	~VectorPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void updateVoliroTiltMonitor(hrt_abstime now);

	VectorTakeoffHandling _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */
	uORB::Publication<dock_brake_status_s> _dock_brake_status_pub{ORB_ID(dock_brake_status)};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _dock_thrust_sub{ORB_ID(dock_guidance_thrust)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};

	trajectory_setpoint_s _setpoint{VectorPosition::empty_trajectory_setpoint};
	trajectory_setpoint_s _last_valid_setpoint{VectorPosition::empty_trajectory_setpoint};
	vehicle_control_mode_s _vehicle_control_mode{};
	dock_guidance_thrust_s _dock_thrust{};
	DockThrustControl _dock_thrust_control;
	vehicle_status_s _vehicle_status{};
	DockFailsafeYaw _dock_failsafe_yaw;
	DockFaultTransfer _dock_fault_transfer;
	DockRtlVelocityRamp _dock_rtl_velocity_ramp;
	DockClimbBrake _dock_climb_brake;
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	control_allocator_status_s _control_allocator_status{};
	float _voliro_pitch_setpoint{0.f};
	bool _dynamics_enabled{false};
	vector_control::AttitudeReference _attitude_reference;
	matrix::Vector3f _reference_shortfall{};
	uint8_t _reference_reset_counter{0};
	uORB::Publication<vector_attitude_reference_s> _reference_pub{ORB_ID(vector_attitude_reference)};
	uORB::Subscription _actuator_prediction_sub{ORB_ID(vector_actuator_status)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Publication<voliro_tilt_status_s> _tilt_status_pub{ORB_ID(voliro_tilt_status)};
	vector_control::VoliroTiltMonitor _tilt_monitor;
	matrix::Quatf _last_attitude_target{};
	bool _last_attitude_target_valid{false};
	bool _fixed_tilt_mode{false}, _fixed_tilt_transition{false};
	vector_control::VoliroFixedTiltTransition _fixed_tilt_handover;
	float _fixed_tilt_angle{0.f};
	uint8_t _fallback_reset_counter{0};
	float _morph_roll_setpoint{0.f};
	float _morph_pitch_setpoint{0.f};
	hrt_abstime _dock_climb_allocation_bad_since{0};
	hrt_abstime _dock_climb_attitude_bad_since{0};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	uORB::Subscription _vector_manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vector_rc_sub{ORB_ID(rc_channels)};
	uORB::Publication<vector_force_setpoint_s> _force_pub{ORB_ID(vector_force_setpoint)};
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VT_CTRL_MODE>) _param_control_mode,
		(ParamFloat<px4::params::VT_REF_FREQ>) _param_reference_frequency,
		(ParamFloat<px4::params::VT_BANK_RATE>) _param_bank_rate,
		(ParamFloat<px4::params::VT_BANK_ACC>) _param_bank_acc,
		(ParamFloat<px4::params::VT_PITCH_ACC>) _param_pitch_acc,
		(ParamFloat<px4::params::VT_HEAD_RATE>) _param_heading_rate,
		(ParamFloat<px4::params::VT_HEAD_ACC>) _param_heading_acc,
		(ParamFloat<px4::params::VT_POS_AW>) _param_position_aw,
		(ParamBool<px4::params::VT_TILT_EN>) _param_tilt_monitor_enable,
		(ParamFloat<px4::params::VT_TILT_ERR>) _param_tilt_error,
		(ParamFloat<px4::params::VT_TILT_T>) _param_tilt_error_time,
		(ParamFloat<px4::params::VT_SRV_TAU>) _param_servo_tau,
		(ParamFloat<px4::params::CA_CT_RATE>) _param_servo_rate,
		(ParamFloat<px4::params::CA_CT_X>) _param_front_x,
		(ParamFloat<px4::params::CA_CT_Z>) _param_front_z,
		(ParamFloat<px4::params::CA_CT_TAIL>) _param_tail_arm,
		(ParamFloat<px4::params::CA_CT_TMAX>) _param_front_max,
		(ParamFloat<px4::params::CA_CT_T5MAX>) _param_tail_max,
		(ParamFloat<px4::params::VEC_IXX>) _param_inertia_x,
		(ParamFloat<px4::params::VEC_IYY>) _param_inertia_y,
		(ParamFloat<px4::params::VEC_IZZ>) _param_inertia_z,
		(ParamInt<px4::params::CA_AIRFRAME>) _param_vector_airframe,
		(ParamBool<px4::params::VEC_AUX_ATT>) _param_vector_aux_att,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_vector_aux_max,
		(ParamFloat<px4::params::VT_PITCH_MAX>) _param_vt_pitch_max,
		(ParamFloat<px4::params::VT_PITCH_RATE>) _param_vt_pitch_rate,
		(ParamInt<px4::params::RC_MAP_AUX1>) _param_rc_map_aux1,
		(ParamInt<px4::params::RC_MAP_VECTOR_SW>) _param_rc_map_vector_sw,
		(ParamInt<px4::params::RC_MAP_LEVEL_SW>) _param_rc_map_level_sw,
		// Position Control
		(ParamFloat<px4::params::VPC_XY_P>)         _param_mpc_xy_p,
		(ParamFloat<px4::params::VPC_Z_P>)          _param_mpc_z_p,
		(ParamFloat<px4::params::VPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::VPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::VPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::VPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::VPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::VPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>)  _param_mpc_acc_decouple,
		(ParamFloat<px4::params::DOCK_REC_YAW_V>)   _param_dock_recovery_yaw_speed,
		(ParamInt<px4::params::DOCK_EXIT_MODE>)    _param_dock_exit_mode,
		(ParamBool<px4::params::DOCK_BRK_EN>)       _param_dock_brake_enable,
		(ParamInt<px4::params::DOCK_BRK_MODE>)      _param_dock_brake_mode,
		(ParamFloat<px4::params::DOCK_BRK_THR>)    _param_dock_brake_thrust,
		(ParamFloat<px4::params::DOCK_BRK_RATE>)   _param_dock_brake_rate,
		(ParamFloat<px4::params::DOCK_BRK_VEL>)    _param_dock_brake_speed,
		(ParamFloat<px4::params::DOCK_CLB_P_RATE>) _param_dock_climb_pitch_rate,
		(ParamFloat<px4::params::DOCK_CLB_TILT>)   _param_dock_climb_tilt,
		(ParamFloat<px4::params::DOCK_CLB_DH_MAX>) _param_dock_climb_height,
		(ParamFloat<px4::params::DOCK_CLB_VMAX>)   _param_dock_climb_speed,
		(ParamFloat<px4::params::DOCK_CLB_A_UP>)   _param_dock_climb_accel_up,
		(ParamFloat<px4::params::DOCK_CLB_A_DN>)   _param_dock_climb_accel_down,
		(ParamFloat<px4::params::DOCK_DYN_JERK>)   _param_dock_dynamic_jerk,
		(ParamFloat<px4::params::DOCK_CLB_TMAX>)   _param_dock_climb_duration,

		(ParamFloat<px4::params::MPC_VEL_LP>)       _param_mpc_vel_lp,
		(ParamFloat<px4::params::MPC_VEL_NF_FRQ>)   _param_mpc_vel_nf_frq,
		(ParamFloat<px4::params::MPC_VEL_NF_BW>)    _param_mpc_vel_nf_bw,
		(ParamFloat<px4::params::MPC_VELD_LP>)      _param_mpc_veld_lp,

		// VectorTakeoff / Land
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time, /**< time to let motors spool up after arming */
		(ParamBool<px4::params::COM_THROW_EN>)      _param_com_throw_en, /**< throw launch enabled  */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,
		(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,    /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,

		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all,

		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc
	);

	math::WelfordMean<float> _sample_interval_s{};

	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
	AlphaFilter<float> _vel_z_lp_filter{};

	math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{};
	math::NotchFilter<float> _vel_z_notch_filter{};

	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
	AlphaFilter<float> _vel_deriv_z_lp_filter{};

	VectorGoto _goto_control; ///< class for handling smooth goto position setpoints
	VectorPosition _control; ///< class for core PID position control

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	SlewRate<float> _tilt_limit_slew_rate;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	VectorPositionStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const VectorPositionStates &states, bool warn);

	/**
	 * @brief adjust existing (or older) setpoint with any EKF reset deltas and update the local counters
	 *
	 * @param[in] vehicle_local_position struct containing EKF reset deltas and counters
	 * @param[out] setpoint trajectory setpoint struct to be adjusted
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
};
