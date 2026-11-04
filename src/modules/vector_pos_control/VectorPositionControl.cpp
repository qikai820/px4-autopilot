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

#include "VectorPositionControl.hpp"
#include "VectorAttitudeSwitches.hpp"
#include <lib/vector_control/VectorGeometry.hpp>

#include <float.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "VectorPosition/ControlMath.hpp"

using namespace matrix;

VectorPositionControl::VectorPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	_sample_interval_s.update(0.01f); // 100 Hz default
	parameters_update(true);
	_dynamics_enabled = _param_control_mode.get() == 1 && _param_vector_airframe.get() == vector_control::voliro_t;
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

VectorPositionControl::~VectorPositionControl()
{
	perf_free(_cycle_perf);
}

int VectorPositionControl::print_status()
{
	PX4_INFO("Voliro dynamics: %s (startup setting)", _dynamics_enabled ? "enabled" : "disabled");
	PX4_INFO("Tilt monitor: %s, observation: %s, fallback: %s, tilt: %.1f deg, error: %.1f deg",
		 _param_tilt_monitor_enable.get() ? "enabled" : "disabled", _tilt_monitor.valid() ? "valid" : "unknown",
		 _tilt_monitor.fault() ? "latched" : "inactive", (double)math::degrees(_tilt_monitor.observed()),
		 (double)math::degrees(_tilt_monitor.error()));
	return 0;
}

void VectorPositionControl::updateVoliroTiltMonitor(hrt_abstime now)
{
	if (_param_vector_airframe.get() != vector_control::voliro_t) { return; }

	vector_control::VoliroTiltMonitor::Settings settings;
	settings.x = _param_front_x.get(); settings.z = _param_front_z.get(); settings.tail = _param_tail_arm.get();
	// Same normalized-thrust/mass relationship as VectorPosition. Do not learn
	// mass from an actuator fault or let a changing HTE hide that fault.
	settings.mass = _param_mpc_thr_hover.get() * (4.f * _param_front_max.get() + _param_tail_max.get()) / CONSTANTS_ONE_G;
	settings.inertia = Vector3f(_param_inertia_x.get(), _param_inertia_y.get(), _param_inertia_z.get());
	settings.servo_rate = math::radians(_param_servo_rate.get()); settings.servo_tau = _param_servo_tau.get();
	settings.error_limit = math::radians(_param_tilt_error.get()); settings.error_time = _param_tilt_error_time.get();
	vector_control::VoliroTiltMonitor::Sample sample;
	vehicle_acceleration_s acceleration{};
	vector_actuator_status_s actuator{};

	if (_vehicle_acceleration_sub.copy(&acceleration)) {
		sample.acceleration_timestamp = acceleration.timestamp_sample;
		sample.specific_force = Vector3f(acceleration.xyz);
	}

	sample.angular_timestamp = _vehicle_angular_velocity.timestamp_sample;
	sample.rates = Vector3f(_vehicle_angular_velocity.xyz);
	sample.angular_acceleration = Vector3f(_vehicle_angular_velocity.xyz_derivative);

	if (_actuator_prediction_sub.copy(&actuator) && actuator.armed) {
		sample.command_timestamp = actuator.timestamp;
		sample.command = actuator.tilt_command;
	}

	const bool previous_fault = _tilt_monitor.fault();
	const bool airborne = _vehicle_control_mode.flag_multicopter_position_control_enabled
			      && !_vehicle_land_detected.landed && !_vehicle_land_detected.ground_contact
			      && !_vehicle_land_detected.maybe_landed;
	_tilt_monitor.update(settings, sample, now, _param_tilt_monitor_enable.get(), _vehicle_control_mode.flag_armed, airborne);

	if (_tilt_monitor.fault() && !previous_fault) {
		_control.resetIntegralXY();
		_attitude_reference.reset();
		_reference_shortfall.zero();
		const float tilt_error_degrees = math::degrees(fabsf(_tilt_monitor.error()));
		// Send plain STATUSTEXT: QGC suppresses text ending in a tab while
		// waiting for event metadata. Keep the event internal to avoid a second
		// notification when that metadata is available.
		mavlink_log_critical(&_mavlink_log_pub, "Tilt fault: MC pos/vel fallback (err %.1f deg)",
				    (double)tilt_error_degrees);
		/* EVENT
		 * @description A persistent motion-based common-tilt mismatch was detected.
		 * Position/velocity control now uses multicopter attitude mapping with
		 * remaining tilt compensation. The fallback clears only after disarm.
		 */
		events::send<float>(events::ID("voliro_tilt_fallback"), {events::Log::Disabled, events::LogInternal::Critical},
				    "Tilt fault: MC pos/vel fallback (err {1:.1} deg)", tilt_error_degrees);
	}

	_control.setMulticopterFallback(_tilt_monitor.fault());
	voliro_tilt_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = sample.acceleration_timestamp;
	status.tilt_command = sample.command;
	status.tilt_expected = _tilt_monitor.expected();
	status.tilt_observed = _tilt_monitor.observed();
	status.tilt_error = _tilt_monitor.error();
	status.error_duration = _tilt_monitor.errorDuration();
	status.enabled = _param_tilt_monitor_enable.get();
	status.armed = _vehicle_control_mode.flag_armed;
	status.observation_valid = _tilt_monitor.valid();
	status.fallback = _tilt_monitor.fault();
	_tilt_status_pub.publish(status);
}

bool VectorPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void VectorPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();

		float sample_freq_hz = 1.f / _sample_interval_s.mean();

		// velocity notch filter
		if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
			_vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());
			_vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());

		} else {
			_vel_xy_notch_filter.disable();
			_vel_z_notch_filter.disable();
		}

		// velocity xy/z low pass filter
		if (_param_mpc_vel_lp.get() > 0.f) {
			_vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());
			_vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());

		} else {
			// disable filtering
			_vel_xy_lp_filter.setAlpha(1.f);
			_vel_z_lp_filter.setAlpha(1.f);
		}

		// velocity derivative xy/z low pass filter
		if (_param_mpc_veld_lp.get() > 0.f) {
			_vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());
			_vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());

		} else {
			// disable filtering
			_vel_deriv_xy_lp_filter.setAlpha(1.f);
			_vel_deriv_z_lp_filter.setAlpha(1.f);
		}



		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		_control.setVectorAirframe(_param_vector_airframe.get());
		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());
		_goto_control.setParamMpcAccHor(_param_mpc_acc_hor.get());
		_goto_control.setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
		_goto_control.setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
		_goto_control.setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
		_goto_control.setParamMpcXyCruise(_param_mpc_xy_cruise.get());
		_goto_control.setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
		_goto_control.setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
		_goto_control.setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
		_goto_control.setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
		_goto_control.setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("vector_mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

VectorPositionStates VectorPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const float dt_s)
{
	VectorPositionStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();

		// vel xy notch filter, then low pass filter
		states.velocity.xy() = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));

		// vel xy derivative low pass filter
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt_s);

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_xy_lp_filter.reset({});
		_vel_xy_notch_filter.reset();
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {

		const float vel_z_prev = _vel_z_lp_filter.getState();

		// vel z notch filter, then low pass filter
		states.velocity(2) = _vel_z_lp_filter.update(_vel_z_notch_filter.apply(vehicle_local_position.vz));

		// vel z derivative low pass filter
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((_vel_z_lp_filter.getState() - vel_z_prev) / dt_s);

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_z_lp_filter.reset({});
		_vel_z_notch_filter.reset();
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void VectorPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		_sample_interval_s.update(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = VectorPosition::empty_trajectory_setpoint;
					_voliro_pitch_setpoint = 0.f;
					_attitude_reference.reset();
					_reference_shortfall.zero();
					_morph_roll_setpoint = 0.f;
					_morph_pitch_setpoint = 0.f;
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_dock_thrust_sub.update(&_dock_thrust);
		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
		_control_allocator_status_sub.update(&_control_allocator_status);
		_vehicle_status_sub.update(&_vehicle_status);
		updateVoliroTiltMonitor(hrt_absolute_time());

		if (_fallback_reset_counter != _vehicle_attitude.quat_reset_counter) {
			if (_last_attitude_target_valid) {
				_last_attitude_target = Quatf(_vehicle_attitude.delta_q_reset) * _last_attitude_target;
				_last_attitude_target.normalize();
			}

			_fallback_reset_counter = _vehicle_attitude.quat_reset_counter;
		}

		if (!_vehicle_control_mode.flag_armed) { _last_attitude_target_valid = false; }

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		VectorPositionStates states{set_vehicle_states(vehicle_local_position, dt)};

		if (_dynamics_enabled) {
			const float heading = vector_control::forcePlaneHeading(Quatf(_vehicle_attitude.q));

			if (PX4_ISFINITE(heading)) { states.yaw = heading; }

			if (_reference_reset_counter != _vehicle_attitude.quat_reset_counter) {
				// The attitude epoch owns reference-frame adaptation. The separate
				// local-position reset handler adapts the INPUT trajectory only.
				_attitude_reference.adaptFrame(Quatf(_vehicle_attitude.delta_q_reset), math::radians(_param_mpc_tiltmax_air.get()));
				_reference_reset_counter = _vehicle_attitude.quat_reset_counter;
				_reference_shortfall.zero();
			}
		}

		const hrt_abstime control_now = hrt_absolute_time();

		if (_dock_fault_transfer.update(control_now, _vehicle_control_mode, _vehicle_status, _dock_thrust,
						!_vehicle_land_detected.landed && !_vehicle_land_detected.ground_contact,
						_param_dock_brake_enable.get() && _param_dock_brake_mode.get() == 1)) {
			_dock_failsafe_yaw.latch(states.yaw, vehicle_local_position.heading_reset_counter);
			_dock_rtl_velocity_ramp.reset();
			PX4_WARN("dock fault RTL: brake, then align yaw before return at %.1f m/s",
				 (double)_param_dock_recovery_yaw_speed.get());
		}

		if (_dock_rtl_velocity_ramp.active() && (!_vehicle_control_mode.flag_armed
				|| _vehicle_control_mode.source_id != 0
				|| _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)) {
			_dock_rtl_velocity_ramp.reset();
		}

		// If a goto setpoint is available this publishes a trajectory setpoint to go there
		// If trajectory_setpoint is published elsewhere, do not use the goto setpoint
		const bool goto_setpoint_enable = _vehicle_control_mode.flag_multicopter_position_control_enabled
						  && !_trajectory_setpoint_sub.updated();

		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample, goto_setpoint_enable)) {
			_goto_control.update(dt, states.position, states.yaw);
		}

		_trajectory_setpoint_sub.update(&_setpoint);

		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			bool skip_takeoff = _param_com_throw_en.get();
			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = VectorPosition::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
			}

			// limit tilt during takeoff ramupup
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			float max_speed_xy = _param_mpc_xy_vel_max.get();

			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}

			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			trajectory_setpoint_s control_setpoint = _setpoint;
			const bool valid_velocity = vehicle_local_position.v_xy_valid && vehicle_local_position.v_z_valid
						    && vehicle_local_position.timestamp > 0 && control_now >= vehicle_local_position.timestamp
						    && control_now - vehicle_local_position.timestamp <= 100_ms;
			const float speed = valid_velocity ? Vector3f(vehicle_local_position.vx, vehicle_local_position.vy,
					    vehicle_local_position.vz).norm() : NAN;
			const float speed_xy = valid_velocity ? Vector2f(vehicle_local_position.vx, vehicle_local_position.vy).norm() : NAN;
			const bool thrust_override_allowed = _vehicle_control_mode.flag_armed
							     && !_vehicle_control_mode.flag_control_termination_enabled && flying
							     && !_vehicle_land_detected.landed && !_vehicle_land_detected.ground_contact;
			const auto thrust_request = _dock_thrust_control.update(_dock_thrust, control_now,
						    _vehicle_control_mode.source_id, thrust_override_allowed, speed);
			const bool climb_selected = _param_dock_brake_enable.get() && _param_dock_brake_mode.get() == 1;
			const bool climb_requested = climb_selected && (thrust_request.braking || _dock_fault_transfer.active());
			const bool attitude_fresh = _vehicle_attitude.timestamp != 0
						    && control_now >= _vehicle_attitude.timestamp
						    && control_now - _vehicle_attitude.timestamp <= 100_ms
						    && Quatf(_vehicle_attitude.q).isAllFinite();
			const bool gyro_fresh = _vehicle_angular_velocity.timestamp != 0
						&& control_now >= _vehicle_angular_velocity.timestamp
						&& control_now - _vehicle_angular_velocity.timestamp <= 100_ms
						&& Vector3f(_vehicle_angular_velocity.xyz).isAllFinite();
			const float actual_tilt = attitude_fresh
						  ? acosf(math::constrain(Dcmf(Quatf(_vehicle_attitude.q))(2, 2), -1.f, 1.f)) : NAN;
			const bool attitude_settled = attitude_fresh && gyro_fresh
						      && actual_tilt < math::radians(5.f)
						      && hypotf(_vehicle_angular_velocity.xyz[0], _vehicle_angular_velocity.xyz[1])
						      < math::radians(10.f);
			const bool climb_eligible = thrust_override_allowed && valid_velocity
						    && vehicle_local_position.xy_valid && vehicle_local_position.z_valid
						    && PX4_ISFINITE(vehicle_local_position.z) && PX4_ISFINITE(states.yaw)
						    && attitude_fresh && gyro_fresh;
			DockClimbBrake::Output climb{};

			if (climb_requested && climb_eligible) {
				const float configured_up = _param_dock_climb_speed.get() > 0.f
							    ? _param_dock_climb_speed.get() : _param_mpc_z_vel_max_up.get();
				const float configured_acc_up = _param_dock_climb_accel_up.get() > 0.f
								? _param_dock_climb_accel_up.get() : _param_mpc_acc_up_max.get();
				const float configured_acc_down = _param_dock_climb_accel_down.get() > 0.f
								  ? _param_dock_climb_accel_down.get() : _param_mpc_acc_down_max.get();
				const DockClimbBrake::Limits limits{
					_param_dock_climb_height.get(),
					math::min(configured_up, math::min(speed_up, _param_mpc_z_vel_max_up.get())),
					math::min(configured_acc_up, _param_mpc_acc_up_max.get()),
					math::min(configured_acc_down, _param_mpc_acc_down_max.get()),
					_param_dock_dynamic_jerk.get(),
					_param_dock_climb_duration.get(),
					math::radians(_param_dock_climb_pitch_rate.get())};
				climb = _dock_climb_brake.update(vehicle_local_position.z, vehicle_local_position.vz,
								 speed_xy, _control.dockBrakeTilt(), vehicle_local_position.z_reset_counter,
								 vehicle_local_position.delta_z, dt, attitude_settled, limits);

				if (climb.active && !climb.complete) {
					const bool allocator_fresh = _control_allocator_status.timestamp != 0
								     && control_now >= _control_allocator_status.timestamp
								     && control_now - _control_allocator_status.timestamp <= 100_ms;
					const bool bad_allocation = allocator_fresh
								    && (_control_allocator_status.handled_motor_failure_mask != 0
									|| (!_control_allocator_status.torque_setpoint_achieved
									    && hypotf(_control_allocator_status.unallocated_torque[0],
											    _control_allocator_status.unallocated_torque[1]) > .05f));
					const bool bad_attitude = fabsf(actual_tilt - _control.dockBrakeTilt())
								  > math::radians(15.f);
					_dock_climb_allocation_bad_since = bad_allocation
									   ? (_dock_climb_allocation_bad_since == 0 ? control_now : _dock_climb_allocation_bad_since) : 0;
					_dock_climb_attitude_bad_since = bad_attitude
									 ? (_dock_climb_attitude_bad_since == 0 ? control_now : _dock_climb_attitude_bad_since) : 0;

					if ((_dock_climb_allocation_bad_since != 0
					     && control_now - _dock_climb_allocation_bad_since > 200_ms)
					    || (_dock_climb_attitude_bad_since != 0
						&& control_now - _dock_climb_attitude_bad_since > 300_ms)) {
						_dock_climb_brake.abort();
						climb = {};
					}
				}

				if (!climb.active && _dock_fault_transfer.active()) {
					_dock_fault_transfer.cancel();
				}

			} else if (climb_requested) {
				_dock_climb_brake.abort();

				if (_dock_fault_transfer.active()) { _dock_fault_transfer.cancel(); }

			} else if (!(_dock_fault_transfer.pending() && climb_selected
				     && (_dock_climb_brake.active() || _dock_climb_brake.failed()))) {
				_dock_climb_brake.reset();
				_dock_climb_allocation_bad_since = 0;
				_dock_climb_attitude_bad_since = 0;
			}

			const bool dock_fault_to_hold = _param_dock_exit_mode.get() == 1
							&& _dock_fault_transfer.active() && PX4_ISFINITE(speed)
							&& vehicle_local_position.xy_valid && vehicle_local_position.z_valid
							&& PX4_ISFINITE(states.yaw);

			if (_param_dock_exit_mode.get() == 1 && _dock_fault_transfer.active() && !dock_fault_to_hold) {
				// Without a fresh position, velocity and heading, leave the native RTL failsafe in charge.
				_dock_fault_transfer.cancel();
			}

			const float yaw_release_speed = dock_fault_to_hold ? .1f
							: math::constrain(_param_dock_recovery_yaw_speed.get(), .1f, 20.f);
			const float yaw_guard_speed = climb.active && !climb.complete && PX4_ISFINITE(speed)
						      ? math::max(speed, yaw_release_speed) : speed;
			const auto yaw_guard = _dock_failsafe_yaw.update(_dock_fault_transfer.active(),
					       yaw_guard_speed, states.yaw, _setpoint.yaw, yaw_release_speed,
					       math::radians(math::constrain(_param_mpc_yawrauto_max.get(), 1.f, 90.f)), dt,
					       vehicle_local_position.heading_reset_counter, vehicle_local_position.delta_heading);

			if (yaw_guard.release_to_rtl && !dock_fault_to_hold) {
				_dock_fault_transfer.cancel();
				_dock_rtl_velocity_ramp.start();

				if (yaw_guard.aligned) {
					PX4_INFO("dock fault RTL: yaw aligned; resume return with velocity ramp");

				} else {
					PX4_WARN("dock fault RTL: yaw unavailable or alignment timed out; return to native RTL");
				}
			}

			if (yaw_guard.active || dock_fault_to_hold) {
				control_setpoint.yaw = dock_fault_to_hold && speed < .5f ? states.yaw
						       : (yaw_guard.active ? yaw_guard.yaw : states.yaw);
				control_setpoint.yawspeed = 0.f;
			}

			if (yaw_guard.hold_translation || dock_fault_to_hold) {
				// Complete braking and measured yaw alignment before RTL can accelerate away.
				for (int axis = 0; axis < 3; ++axis) {
					control_setpoint.position[axis] = NAN;
					control_setpoint.velocity[axis] = 0.f;
					control_setpoint.acceleration[axis] = NAN;
					control_setpoint.jerk[axis] = NAN;
				}

				if (climb.active) { control_setpoint.velocity[2] = climb.velocity_z; }

			} else if (_dock_rtl_velocity_ramp.active()) {
				const Vector3f target_velocity(control_setpoint.velocity);
				const Vector3f ramped_velocity = _dock_rtl_velocity_ramp.update(target_velocity,
								 math::constrain(_param_mpc_acc_hor.get(), .1f, 20.f), dt);

				if (_dock_rtl_velocity_ramp.active()) {
					for (int axis = 0; axis < 3; ++axis) {
						control_setpoint.position[axis] = NAN;
						control_setpoint.velocity[axis] = ramped_velocity(axis);
						control_setpoint.acceleration[axis] = NAN;
						control_setpoint.jerk[axis] = NAN;
					}
				}
			}

			if (climb.active && !yaw_guard.hold_translation && !dock_fault_to_hold) {
				// Normal predicted pass: use the same vertical braking trajectory.
				control_setpoint.position[2] = NAN;
				control_setpoint.velocity[2] = climb.velocity_z;
				control_setpoint.acceleration[2] = NAN;
				control_setpoint.jerk[2] = NAN;
			}

			_control.setInputSetpoint(control_setpoint);

			// update states
			if (!PX4_ISFINITE(control_setpoint.position[2])
			    && PX4_ISFINITE(control_setpoint.velocity[2]) && (fabsf(control_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(control_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}

			if ((!PX4_ISFINITE(control_setpoint.velocity[0]) || !PX4_ISFINITE(control_setpoint.velocity[1]))
			    && (!PX4_ISFINITE(control_setpoint.position[0]) || !PX4_ISFINITE(control_setpoint.position[1]))) {
				// Horizontal velocity is not controlled, reset the integrators to avoid
				// over-corrections when starting again.
				_control.resetIntegralXY();
			}

			_control.setState(states);

			_control.setVelocityIntegralHold(thrust_request.slew_rate > 0.f && !thrust_request.braking);
			const bool fault_brake = (yaw_guard.braking || (dock_fault_to_hold && speed >= .5f))
						 && _param_dock_brake_enable.get();
			const bool brake_enabled = climb_selected ? climb.active && !climb.complete
						   : fault_brake || thrust_request.braking;

			if (_dock_fault_transfer.pending() && climb_selected && _dock_climb_brake.active()) {
				_control.suspendDockBraking();

			} else {
				_control.setDockBraking(brake_enabled,
							fault_brake ? _param_dock_brake_thrust.get() : thrust_request.thrust,
							climb.active ? math::radians(_param_dock_climb_pitch_rate.get())
							: (fault_brake ? math::radians(_param_dock_brake_rate.get()) : thrust_request.brake_rate),
							fault_brake ? _param_dock_brake_speed.get() : thrust_request.brake_speed);
			}

			_control.setDockClimb(climb.active && !climb.complete, climb.capture,
					      math::radians(_param_dock_climb_tilt.get()));

			// Allocation feedback is normalized; rotate the body residual into NED.
			_vehicle_attitude_sub.copy(&_vehicle_attitude);
			control_allocator_status_s allocation{};
			Vector3f shortfall{};

			if (_control_allocator_status_sub.copy(&allocation) && hrt_elapsed_time(&allocation.timestamp) < 200_ms) {
				shortfall = Dcmf(Quatf(_vehicle_attitude.q)) * Vector3f(allocation.unallocated_thrust);
			}

			_control.setAllocationShortfall(shortfall + _reference_shortfall);
			_control.setAllocationAntiWindupGain(_dynamics_enabled ? _param_position_aw.get() : 0.f);
			// Run position control
			const bool control_update_success = _control.update(dt);

			if (control_update_success) {

				// Valid control update - store for fallback
				_last_valid_setpoint = control_setpoint;

			} else {

				// Fallback/failsafe must be able to change thrust immediately.
				_control.setVelocityIntegralHold(false);
				_control.setDockBraking(false);
				_dock_climb_brake.abort();

				if (_dock_fault_transfer.active()) { _dock_fault_transfer.cancel(); }

				// Initial update failed - Try fallback if within timeout
				if (control_now < _last_valid_setpoint.timestamp + 200_ms) {
					// Use last valid setpoint
					adjustSetpointForEKFResets(vehicle_local_position, _last_valid_setpoint);
					_control.setInputSetpoint(_last_valid_setpoint);
				}

				// Still failing / not within timeout - Go to failsafe
				if (!_control.update(dt)) {

					_vehicle_constraints = {0, NAN, NAN, false, {}}; // reset constraints

					_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
					_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());

					_control.update(dt);
				}
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			manual_control_setpoint_s manual{};
			const bool manual_fresh = _vector_manual_sub.copy(&manual) && manual.valid
						  && hrt_elapsed_time(&manual.timestamp) < 500_ms;
			rc_channels_s rc{};
			const bool rc_fresh = _vector_rc_sub.copy(&rc) && !rc.signal_lost && rc.timestamp != 0
					      && hrt_elapsed_time(&rc.timestamp) < 500_ms;
			const auto switches = vector_control::attitudeSwitches(_param_rc_map_vector_sw.get(),
					      _param_rc_map_level_sw.get(), rc_fresh, rc.channels, rc.channel_count,
					      sizeof(rc.channels) / sizeof(rc.channels[0]));
			const bool switches_valid = switches.valid;
			const bool vector_enabled = switches.enabled;
			const bool level_requested = switches.level;
			const bool voliro_airframe = _param_vector_airframe.get() == vector_control::voliro_t;
			const bool user_hold = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			const bool manual_position = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
						     || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW;
			const bool voliro_position_allowed = voliro_airframe && !_tilt_monitor.fault() && control_update_success
							     && (manual_position || user_hold) && !_vehicle_status.failsafe
							     && _vehicle_control_mode.flag_armed && !_vehicle_land_detected.landed
							     && !_vehicle_land_detected.ground_contact;
			const bool rc_input_valid = manual_fresh && manual.data_source == manual_control_setpoint_s::SOURCE_RC;
			const bool voliro_pitch_allowed = voliro_position_allowed && _param_vector_aux_att.get();
			const bool voliro_input_valid = voliro_pitch_allowed && rc_input_valid
							&& _param_rc_map_aux1.get() > 0 && PX4_ISFINITE(manual.aux1);
			const float pitch_target = vector_control::voliroIndependentPitchTarget(voliro_input_valid, switches,
						   manual.aux1, math::radians(math::constrain(_param_vt_pitch_max.get(), 0.f, 160.f)));

			if (!voliro_airframe || _tilt_monitor.fault() || !_vehicle_control_mode.flag_armed || _vehicle_land_detected.landed) {
				_voliro_pitch_setpoint = 0.f;

			} else {
				const float max_step = math::radians(_param_vt_pitch_rate.get()) * dt;
				_voliro_pitch_setpoint += math::constrain(pitch_target - _voliro_pitch_setpoint, -max_step, max_step);
			}

			_control.getAttitudeSetpoint(attitude_setpoint, _voliro_pitch_setpoint);
			vector_attitude_reference_s reference_message{};

			if (_dynamics_enabled && voliro_pitch_allowed) {
				vector_control::AttitudeReference::Settings settings;
				settings.bank_limit = math::constrain(_tilt_limit_slew_rate.getState(), 0.f, math::radians(80.f));
				settings.pitch_limit = math::radians(math::constrain(_param_vt_pitch_max.get(), 0.f, 160.f));
				settings.pitch_rate = math::radians(_param_vt_pitch_rate.get());
				settings.pitch_acceleration = math::radians(_param_pitch_acc.get());
				settings.bank_rate = math::radians(_param_bank_rate.get());
				settings.bank_acceleration = math::radians(_param_bank_acc.get());
				settings.heading_rate = math::radians(_param_heading_rate.get());
				settings.heading_acceleration = math::radians(_param_heading_acc.get());
				settings.tracking_frequency = _param_reference_frequency.get();
				settings.thrust_limit = _param_mpc_thr_max.get();

				if (!_attitude_reference.initialized()) {
					_attitude_reference.initialize(Quatf(_vehicle_attitude.q), settings.bank_limit);
				}

				float pitch_authority = _attitude_reference.pitchAuthority(Quatf(_vehicle_attitude.q));
				vector_actuator_status_s prediction{};

				if (_actuator_prediction_sub.copy(&prediction) && prediction.armed && prediction.prediction_enabled
				    && hrt_elapsed_time(&prediction.timestamp) < 100_ms
				    && PX4_ISFINITE(prediction.tilt_command) && PX4_ISFINITE(prediction.tilt_estimate)) {
					// Slow independent body pitching when the calibrated servo model
					// falls behind. Do not slow the stabilizing bank correction.
					pitch_authority = math::min(pitch_authority,
								    math::constrain(1.f - fabsf(prediction.tilt_command - prediction.tilt_estimate)
										    / math::radians(20.f), .2f, 1.f));
				}

				vector_control::AttitudeReference::Output reference;
				const Vector3f raw_force(local_pos_sp.thrust);
				const bool valid = _attitude_reference.update(raw_force, control_setpoint.yaw, control_setpoint.yawspeed,
						   pitch_target, pitch_authority, dt, settings, reference);

				if (valid) {
					reference.attitude.copyTo(attitude_setpoint.q_d);
					attitude_setpoint.yaw_sp_move_rate = 0.f; // full angular feedforward is in the paired reference
					reference.force.copyTo(local_pos_sp.thrust);
					_reference_shortfall = raw_force - reference.force;
					_voliro_pitch_setpoint = reference.pitch;
					reference_message.valid = true;
					reference.attitude.copyTo(reference_message.q_d);
					reference.omega_ned.copyTo(reference_message.omega_ned);
					reference.alpha_ned.copyTo(reference_message.alpha_ned);
					raw_force.copyTo(reference_message.force_raw);
					reference.force.copyTo(reference_message.force_limited);
					reference_message.heading = reference.heading;
					reference_message.bank = reference.bank;
					reference_message.pitch = reference.pitch;
					reference_message.pitch_authority = pitch_authority;

				} else {
					_attitude_reference.reset();
					_reference_shortfall.zero();
				}

			} else {
				_attitude_reference.reset();
				_reference_shortfall.zero();
			}

			if (voliro_airframe && _tilt_monitor.fault()) {
				const float thrust_angle = vector_control::voliroFallbackThrustAngle(_tilt_monitor.observed(),
							   _param_front_x.get(), _param_front_z.get(), _param_tail_arm.get());
				const Quatf target = Quatf(attitude_setpoint.q_d) * Quatf(Eulerf(0.f, thrust_angle, 0.f));
				const Quatf previous = _last_attitude_target_valid ? _last_attitude_target : Quatf(_vehicle_attitude.q);
				vector_control::slewFallbackAttitude(previous, target,
						math::radians(_param_bank_rate.get()) * dt).copyTo(attitude_setpoint.q_d);
			}

			// Optional MorphQuad independent attitude inputs; standard flight-task stick mapping remains active.

			if (_param_vector_aux_att.get() && _param_vector_airframe.get() == vector_control::morphquad
			    && _vehicle_control_mode.flag_control_manual_enabled && manual_fresh) {
				const float limit = math::radians(_param_vector_aux_max.get());
				const bool configured = _param_rc_map_vector_sw.get() > 0 || _param_rc_map_level_sw.get() > 0;
				float roll_target = 0.f;
				float morph_pitch_target = 0.f;

				if (switches_valid && !level_requested) {
					if (vector_enabled && PX4_ISFINITE(manual.aux1) && PX4_ISFINITE(manual.aux2)) {
						roll_target = math::constrain(manual.aux1, -1.f, 1.f) * limit;
						morph_pitch_target = math::constrain(manual.aux2, -1.f, 1.f) * limit;
					}
				}

				const float max_step = configured ? math::radians(_param_vt_pitch_rate.get()) * dt : 2.f * limit;
				_morph_roll_setpoint += math::constrain(roll_target - _morph_roll_setpoint, -max_step, max_step);
				_morph_pitch_setpoint += math::constrain(morph_pitch_target - _morph_pitch_setpoint, -max_step, max_step);
				Quatf(Eulerf(_morph_roll_setpoint, _morph_pitch_setpoint, local_pos_sp.yaw)).copyTo(attitude_setpoint.q_d);

			} else {
				_morph_roll_setpoint = 0.f;
				_morph_pitch_setpoint = 0.f;
			}

			const Vector3f force_ned(local_pos_sp.thrust);
			_last_attitude_target = Quatf(attitude_setpoint.q_d);
			_last_attitude_target_valid = _last_attitude_target.isAllFinite();
			_local_pos_sp_pub.publish(local_pos_sp);
			vector_control::toBody(force_ned, Quatf(_vehicle_attitude.q)).copyTo(attitude_setpoint.thrust_body);
			vector_force_setpoint_s force_sp{};
			force_sp.timestamp = hrt_absolute_time();
			force_sp.timestamp_sample = vehicle_local_position.timestamp_sample;
			force_ned.copyTo(force_sp.xyz);
			_force_pub.publish(force_sp);
			attitude_setpoint.timestamp = hrt_absolute_time();

			if (_dynamics_enabled) {
				reference_message.timestamp = attitude_setpoint.timestamp;
				reference_message.timestamp_sample = vehicle_local_position.timestamp_sample;
				reference_message.quat_reset_counter = _vehicle_attitude.quat_reset_counter;
				_reference_pub.publish(reference_message);
			}

			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
			dock_brake_status_s brake_status{};
			brake_status.timestamp = hrt_absolute_time();
			brake_status.active = control_update_success && climb.active;
			brake_status.climb = brake_status.active;
			brake_status.capture = brake_status.active && climb.capture;
			brake_status.complete = brake_status.active && climb.complete;
			brake_status.allow_floor = brake_status.active && climb.allow_floor;
			brake_status.capture_floor = brake_status.capture
						     && math::max(actual_tilt, _control.dockBrakeTilt()) > math::radians(5.f)
						     ? _param_dock_brake_thrust.get() : 0.f;
			brake_status.height_used = brake_status.active ? climb.height_used : NAN;
			_dock_brake_status_pub.publish(brake_status);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			_dock_climb_brake.reset();
			_control.setDockBraking(false);
			dock_brake_status_s brake_status{};
			brake_status.timestamp = hrt_absolute_time();
			_dock_brake_status_pub.publish(brake_status);
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
			_control.resetIntegral();
			_control.setVelocityIntegralHold(false);
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
}

trajectory_setpoint_s VectorPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const VectorPositionStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = VectorPosition::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// don't move along xy
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// descend with land speed since we can't stop
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// don't move along z if we can stop in all dimensions
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		// emergency descend with a bit below hover thrust
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

void VectorPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}

	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + Vector2f(vehicle_local_position.delta_vxy));
		_vel_xy_notch_filter.reset();
	}

	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + vehicle_local_position.delta_vz);
		_vel_z_notch_filter.reset();
	}

	// save latest reset counters
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int VectorPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	VectorPositionControl *instance = new VectorPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VectorPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VectorPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vector_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vector_pos_control_main(int argc, char *argv[])
{
	return VectorPositionControl::main(argc, argv);
}
