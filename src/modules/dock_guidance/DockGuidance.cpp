/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "DockGuidance.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>

#include <cstring>

using namespace matrix;

DockGuidance::DockGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

DockGuidance::~DockGuidance()
{
}

bool
DockGuidance::init()
{
	update_parameters();
	_mode_request_id = hrt_absolute_time();

	// 新视觉观测会立即触发运行；固定周期同时负责状态历史采样和断流看门狗。
	if (!_viztra_sub.registerCallback()) {
		PX4_ERR("viztra callback registration failed");
		return false;
	}

	// 100 Hz 周期同时维护状态历史并充当视觉断流看门狗。
	// 视觉回调仍会立即唤醒本模块，所以可接收最高 120 Hz 的输入而不增加固定等待。
	ScheduleOnInterval(WATCHDOG_INTERVAL_US);

	return true;
}

void
DockGuidance::Run()
{
	if (should_exit()) {
		if (!_unregistration_sent) {
			if (_dock_mode_active) {
				publish_safe_rates_setpoint(hrt_absolute_time());
			}

			unregister_dock_mode();
			_unregistration_sent = true;
			// 给 Commander 留出一个更新周期消费注销消息，再销毁 uORB 发布者。
			ScheduleDelayed(200_ms);
			return;
		}

		_viztra_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s p;
		_parameter_update_sub.copy(&p);
		update_parameters();
	}

	const uint64_t now_us = hrt_absolute_time();

	if (_mode_id < 0 || _arming_check_id < 0) {
		if ((_last_registration_request_us == 0) || (now_us - _last_registration_request_us > 1_s)) {
			register_dock_mode(now_us);
		}

		check_mode_registration();

	} else if ((_last_mode_config_us == 0) || (now_us - _last_mode_config_us > 1_s)) {
		configure_dock_mode(now_us);
	}

	arming_check_request_s arming_check_request{};

	if (_arming_check_request_sub.update(&arming_check_request)) {
		reply_to_arming_check(arming_check_request);
	}

	update_mode_state();
	push_delay_history(now_us);

	viztra_observation_s obs;

	if (!_viztra_sub.update(&obs)) {
		const uint64_t timeout_us = static_cast<uint64_t>(math::constrain(_param_dock_obs_tout.get(), 20, 5000)) * 1000ULL;

		if (_setpoint_valid && (now_us > _last_observation_us) && ((now_us - _last_observation_us) > timeout_us)) {
			invalidate_setpoint(now_us, "vision timeout");
		}

		return;
	}

	// timestamp_sample 是完整视觉帧接收时刻。旧驱动未填写时退化为当前时刻；
	// 明显位于未来的时间戳视为非法，避免无符号减法和延迟补偿越界。
	const uint64_t sample_us = (obs.timestamp_sample != 0) ? obs.timestamp_sample : now_us;

	if (sample_us > now_us + 5000ULL) {
		invalidate_setpoint(now_us, "future timestamp");
		return;
	}

	const uint64_t timeout_us = static_cast<uint64_t>(math::constrain(_param_dock_obs_tout.get(), 20, 5000)) * 1000ULL;

	if ((now_us > sample_us) && ((now_us - sample_us) > timeout_us)) {
		invalidate_setpoint(now_us, "stale observation");
		return;
	}

	// 丢弃重复或乱序帧；保留上一条有效输出，随后由看门狗统一执行超时退出。
	if ((_last_sample_us != 0) && (sample_us <= _last_sample_us)) {
		return;
	}

	const float run_dt_s = (_last_sample_us == 0) ? LOOP_DT :
			       math::constrain((sample_us - _last_sample_us) * 1e-6f, 0.002f, 0.1f);
	_last_sample_us = sample_us;
	_last_observation_us = sample_us;

	// 无目标或输入非法时，清空内部状态并明确发布 invalid，
	// 使下游控制器可以立即放弃视觉引导。
	if (!obs.valid || !std::isfinite(obs.offset_x) || !std::isfinite(obs.offset_y)) {
		invalidate_setpoint(now_us, "invalid observation");
		return;
	}

	// 取与相机画面时间对应的姿态/角速度，减小图像传输和识别延迟带来的误差。
	Vector3f att_deg;
	Vector3f gyro_dps;
	if (!get_delayed_state(sample_us, att_deg, gyro_dps)) {
		invalidate_setpoint(now_us, "state history unavailable");
		return;
	}

	// 将视觉板给出的目标偏移转换为相机视线角（LOS）。
	// 若视觉板已经输出角度则直接使用；否则根据图像分辨率和 FOV 将像素偏移换算为角度。
	float los_yaw_deg, los_pitch_deg;

	if (obs.offsets_are_angles) {
		los_yaw_deg = obs.offset_x;
		los_pitch_deg = obs.offset_y;

	} else {
		const float res_x = _param_dock_res_x.get();
		const float res_y = _param_dock_res_y.get();
		const float fov_x_rad = math::radians(_param_dock_fov_x.get());
		const float fov_y_rad = math::radians(_param_dock_fov_y.get());

		auto cal_angle = [](float pixel, float angle, float x_in) -> float {
			pixel = math::constrain(pixel, 100.0f, 8000.0f);
			angle = math::constrain(angle, math::radians(10.0f), math::radians(80.0f));
			x_in = math::constrain(x_in, -pixel, pixel);
			return math::degrees(atanf(2.0f * x_in / pixel * tanf(angle * 0.5f)));
		};

		const float theta1 = cal_angle(res_x, fov_x_rad, obs.offset_x);
		const float theta2 = cal_angle(res_y, fov_y_rad, obs.offset_y);

		// 在相机坐标系中构造指向目标的单位方向之前的向量。
		const Vector3f los_cam(1.0f, tanf(math::radians(theta1)), -tanf(math::radians(theta2)));
		los_yaw_deg = math::degrees(atanf(los_cam(1) / los_cam(0)));
		los_pitch_deg = -math::degrees(atanf(los_cam(2) / Vector2f(los_cam(0), los_cam(1)).norm()));
	}

	if (!std::isfinite(los_yaw_deg) || !std::isfinite(los_pitch_deg)) {
		invalidate_setpoint(now_us, "invalid LOS");
		return;
	}

	const float los_limit = math::constrain(_param_dock_los_max.get(), 10.0f, 89.9f);

	if ((fabsf(los_yaw_deg) > los_limit) || (fabsf(los_pitch_deg) > los_limit)) {
		invalidate_setpoint(now_us, "LOS outside configured range");
		return;
	}

	process_observation(los_yaw_deg, los_pitch_deg, att_deg, gyro_dps, run_dt_s, sample_us);
}

// ---------- 姿态/角速度延时缓冲 ----------

void
DockGuidance::push_delay_history(hrt_abstime timestamp)
{
	// 将 PX4 四元数姿态转换为欧拉角，统一以度保存，便于后续与视觉角度相加。
	vehicle_attitude_s att{};
	vehicle_angular_velocity_s av{};

	if (!_att_sub.copy(&att) || !_gyro_sub.copy(&av)) {
		return;
	}

	const Quatf q(att.q);

	if (!q.isAllFinite() || (q.norm() < 0.5f) || !Vector3f(av.xyz).isAllFinite()) {
		return;
	}

	Vector3f gyro_dps(math::degrees(av.xyz[0]),
			  math::degrees(av.xyz[1]),
			  math::degrees(av.xyz[2]));

	_att_delay_buf[_delay_idx] = q.normalized();
	_gyro_delay_buf[_delay_idx] = gyro_dps;
	_delay_time_buf[_delay_idx] = timestamp;
	_delay_idx = (_delay_idx + 1) % DELAY_BUF_SIZE;
	_delay_count = math::min(_delay_count + 1, DELAY_BUF_SIZE);
}

bool
DockGuidance::get_delayed_state(hrt_abstime timestamp_sample, Vector3f &att_deg, Vector3f &gyro_dps) const
{
	if (_delay_count <= 0) {
		return false;
	}

	const uint64_t delay_us = static_cast<uint64_t>(math::constrain(_param_dock_cam_dly.get(), 0, 5000)) * 1000ULL;
	const hrt_abstime target_us = (timestamp_sample > delay_us) ? timestamp_sample - delay_us : 0;
	const int oldest = (_delay_idx - _delay_count + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;
	const int newest = (_delay_idx - 1 + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;

	if (target_us <= _delay_time_buf[oldest]) {
		// 历史尚未覆盖相机采样时刻时不使用错误姿态；等缓冲区积累足够数据后再输出有效指令。
		if (target_us == _delay_time_buf[oldest]) {
			const Eulerf euler(_att_delay_buf[oldest]);
			att_deg = Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()), math::degrees(euler.psi()));
			gyro_dps = _gyro_delay_buf[oldest];
			return true;
		}

		return false;
	}

	if (target_us >= _delay_time_buf[newest]) {
		const Eulerf euler(_att_delay_buf[newest]);
		att_deg = Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()), math::degrees(euler.psi()));
		gyro_dps = _gyro_delay_buf[newest];
		return true;
	}

	for (int n = 1; n < _delay_count; ++n) {
		const int previous = (oldest + n - 1) % DELAY_BUF_SIZE;
		const int next = (oldest + n) % DELAY_BUF_SIZE;

		if (target_us <= _delay_time_buf[next]) {
			const uint64_t span_us = _delay_time_buf[next] - _delay_time_buf[previous];
			const float alpha = (span_us > 0) ?
					    static_cast<float>(target_us - _delay_time_buf[previous]) / static_cast<float>(span_us) : 0.0f;

			Quatf q_previous = _att_delay_buf[previous];
			Quatf q_next = _att_delay_buf[next];

			// q 与 -q 表示同一姿态，选择短弧一侧后再做归一化线性插值。
			if (q_previous.dot(q_next) < 0.0f) {
				q_next *= -1.0f;
			}

			Quatf q_interpolated(q_previous(0) + alpha * (q_next(0) - q_previous(0)),
					      q_previous(1) + alpha * (q_next(1) - q_previous(1)),
					      q_previous(2) + alpha * (q_next(2) - q_previous(2)),
					      q_previous(3) + alpha * (q_next(3) - q_previous(3)));
			q_interpolated.normalize();
			const Eulerf euler(q_interpolated);
			att_deg = Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()), math::degrees(euler.psi()));

			for (int axis = 0; axis < 3; ++axis) {
				gyro_dps(axis) = _gyro_delay_buf[previous](axis) + alpha *
						 (_gyro_delay_buf[next](axis) - _gyro_delay_buf[previous](axis));
			}

			return att_deg.isAllFinite() && gyro_dps.isAllFinite();
		}
	}

	return false;
}

void
DockGuidance::reset_tracking_state()
{
	_los_c_deg.zero();
	_los_e_deg.zero();
	_los_e_unit.zero();
	_los_e_dot.zero();
	_los_e_rate_dps.zero();
	_camera_e_deg.zero();
	_gyro_c_dps.zero();
	_los_c_rate_dps.zero();
	_target_rate_c_dps.zero();
	_target_rate_b_dps.zero();
	_stage = Stage::NONE;
	_have_last_target = false;
	_los_rate_control_active = false;
	_los_yaw_rate_pid_I = 0.0f;
	_los_pitch_rate_pid_I = 0.0f;
	_filt_initialized = false;
	_deriv_count = 0;
	_deriv_idx = 0;
}

void
DockGuidance::invalidate_setpoint(hrt_abstime timestamp, const char *reason)
{
	if (_setpoint_valid) {
		PX4_WARN("dock: %s", reason);
	}

	reset_tracking_state();
	_setpoint_valid = false;

	dock_guidance_setpoint_s sp{};
	sp.timestamp = timestamp;
	sp.valid = false;
	sp.stage = static_cast<uint8_t>(Stage::NONE);
	_setpoint_pub.publish(sp);

	if (_dock_mode_active) {
		publish_safe_rates_setpoint(timestamp);
	}
}

void
DockGuidance::register_dock_mode(hrt_abstime now)
{
	register_ext_component_request_s request{};
	request.timestamp = now;
	request.request_id = _mode_request_id;
	request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
	request.register_arming_check = true;
	request.register_mode = true;
	strncpy(request.name, "Dock Guidance", sizeof(request.name) - 1);
	_register_ext_component_request_pub.publish(request);
	_last_registration_request_us = now;
}

void
DockGuidance::check_mode_registration()
{
	register_ext_component_reply_s reply{};
	int remaining = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

	while ((_mode_id < 0 || _arming_check_id < 0) && remaining-- > 0
	       && _register_ext_component_reply_sub.update(&reply)) {
		if (reply.request_id != _mode_request_id) {
			continue;
		}

		if (reply.success) {
			_mode_id = reply.mode_id;
			_arming_check_id = reply.arming_check_id;
			PX4_INFO("Dock mode registered: mode=%d check=%d", _mode_id, _arming_check_id);
			configure_dock_mode(hrt_absolute_time());

		} else {
			PX4_WARN("Dock mode registration rejected");
		}
	}
}

void
DockGuidance::configure_dock_mode(hrt_abstime now)
{
	if (_mode_id < 0) {
		return;
	}

	vehicle_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.source_id = static_cast<uint8_t>(_mode_id);
	control_mode.flag_control_rates_enabled = true;
	control_mode.flag_control_allocation_enabled = true;
	_config_control_setpoints_pub.publish(control_mode);
	_last_mode_config_us = now;
}

void
DockGuidance::unregister_dock_mode()
{
	if (_mode_id < 0 && _arming_check_id < 0) {
		return;
	}

	unregister_ext_component_s request{};
	request.timestamp = hrt_absolute_time();
	request.mode_id = _mode_id;
	request.arming_check_id = _arming_check_id;
	request.mode_executor_id = -1;
	strncpy(request.name, "Dock Guidance", sizeof(request.name) - 1);
	_unregister_ext_component_pub.publish(request);
	_mode_id = -1;
	_arming_check_id = -1;
	_dock_mode_active = false;
}

void
DockGuidance::reply_to_arming_check(const arming_check_request_s &request)
{
	if (_arming_check_id < 0) {
		return;
	}

	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request.request_id;
	reply.registration_id = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.can_arm_and_run = _setpoint_valid;
	reply.mode_req_angular_velocity = true;
	reply.mode_req_attitude = true;
	reply.mode_req_manual_control = true;
	_arming_check_reply_pub.publish(reply);
}

void
DockGuidance::update_mode_state()
{
	vehicle_status_s status{};

	if (!_vehicle_status_sub.copy(&status)) {
		return;
	}

	const bool active = (_mode_id >= 0) && (status.nav_state == static_cast<uint8_t>(_mode_id));

	if (active && !_dock_mode_active) {
		PX4_INFO("Dock mode active");
		_reset_rate_integral = true;

		if (!_setpoint_valid) {
			// 正常情况下已解锁时 Commander 会拒绝无视觉的模式切换；这里仍清除旧模式遗留指令作为最后防线。
			publish_safe_rates_setpoint(hrt_absolute_time());
		}

	} else if (!active && _dock_mode_active) {
		PX4_INFO("Dock mode inactive");
	}

	_dock_mode_active = active;
}

void
DockGuidance::publish_rates_setpoint(const dock_guidance_setpoint_s &setpoint, bool reset_integral)
{
	vehicle_rates_setpoint_s rates{};
	rates.timestamp = setpoint.timestamp;
	rates.roll = setpoint.roll_rate_rad_s;
	rates.pitch = setpoint.pitch_rate_rad_s;
	rates.yaw = setpoint.yaw_rate_rad_s;
	rates.thrust_body[0] = 0.0f;
	rates.thrust_body[1] = 0.0f;
	rates.thrust_body[2] = -math::constrain(setpoint.throttle, 0.0f, 1.0f);
	rates.reset_integral = reset_integral;
	_rates_setpoint_pub.publish(rates);
}

void
DockGuidance::publish_safe_rates_setpoint(hrt_abstime timestamp)
{
	dock_guidance_setpoint_s safe{};
	safe.timestamp = timestamp;
	safe.throttle = math::constrain(_param_dock_thr.get(), 0.0f, 1.0f);
	publish_rates_setpoint(safe, true);
}

// ---------- 视觉观测处理 ----------

void
DockGuidance::process_observation(float los_yaw_deg, float los_pitch_deg,
				  const Vector3f &att_deg,
				  const Vector3f &gyro_dps,
				  float dt_s, hrt_abstime timestamp_sample)
{
	// los_c：相机坐标系中的目标方位角，分别为水平偏差（yaw）与垂直偏差（pitch）。
	_los_c_deg(0) = los_yaw_deg;
	_los_c_deg(1) = los_pitch_deg;

	const float yaw_rad = math::radians(los_yaw_deg);
	const float pitch_rad = math::radians(los_pitch_deg);
	const Vector3f los_c_unit(cosf(pitch_rad) * cosf(yaw_rad),
				  cosf(pitch_rad) * sinf(yaw_rad),
				  -sinf(pitch_rad));

	// 将相机视线从相机系旋转到 NED 地理系；这样可在地理系估计 LOS 变化率。
	const Dcmf R_ned_body(Eulerf(math::radians(att_deg(0)),
				     math::radians(att_deg(1)),
				     math::radians(att_deg(2))));
	const Dcmf R_ned_cam = R_ned_body * _rot_cam_to_body;
	_los_e_unit = R_ned_cam * los_c_unit;
	_los_e_unit.normalize();

	_los_e_deg(0) = math::degrees(matrix::wrap_pi(atan2f(_los_e_unit(1), _los_e_unit(0))));
	_los_e_deg(1) = math::degrees(atan2f(-_los_e_unit(2), Vector2f(_los_e_unit(0), _los_e_unit(1)).norm()));

	// 对单位视线向量低通滤波，而不是直接滤波欧拉角，避免角度跨 ±180° 时出现跳变。
	const float filt_hz = math::constrain(_param_dock_filt_hz.get(), 0.1f, 20.0f);

	if (!_filt_initialized) {
		_los_x_filt = _los_e_unit(0);
		_los_y_filt = _los_e_unit(1);
		_los_z_filt = _los_e_unit(2);
		_filt_initialized = true;

	} else {
		_los_x_filt = low_pass_filter(_los_x_filt, _los_e_unit(0), filt_hz, dt_s);
		_los_y_filt = low_pass_filter(_los_y_filt, _los_e_unit(1), filt_hz, dt_s);
		_los_z_filt = low_pass_filter(_los_z_filt, _los_e_unit(2), filt_hz, dt_s);
	}

	Vector3f los_e_filtered(_los_x_filt, _los_y_filt, _los_z_filt);

	if (!los_e_filtered.isAllFinite() || (los_e_filtered.norm() < 0.5f)) {
		invalidate_setpoint(hrt_absolute_time(), "filter failure");
		return;
	}

	los_e_filtered.normalize();
	_los_e_unit = los_e_filtered;
	_los_x_filt = _los_e_unit(0);
	_los_y_filt = _los_e_unit(1);
	_los_z_filt = _los_e_unit(2);

	// 带 uint64 时间戳的环形窗口，避免微秒时间存入 float 后随运行时间丢失精度。
	_los_x_deriv_buf[_deriv_idx] = _los_x_filt;
	_los_y_deriv_buf[_deriv_idx] = _los_y_filt;
	_los_z_deriv_buf[_deriv_idx] = _los_z_filt;
	_los_deriv_time_buf[_deriv_idx] = timestamp_sample;
	_deriv_idx = (_deriv_idx + 1) % DERIV_SIZE;
	_deriv_count = math::min(_deriv_count + 1, DERIV_SIZE);

	if (_deriv_count >= DERIV_SIZE) {
		_los_e_dot = Vector3f(derivative_filter(_los_x_deriv_buf), derivative_filter(_los_y_deriv_buf),
					 derivative_filter(_los_z_deriv_buf));
		_los_e_dot -= _los_e_unit * (_los_e_unit * _los_e_dot);
	}

	// 单位向量 l 的角速度满足 omega = l × l_dot；先在地理系求得，再转换到相机系。
	const Vector3f los_e_rate_rads = _los_e_unit % _los_e_dot;
	_los_e_rate_dps = Vector3f(math::degrees(los_e_rate_rads(0)),
				   math::degrees(los_e_rate_rads(1)),
				   math::degrees(los_e_rate_rads(2)));

	const Dcmf R_cam_ned = R_ned_cam;
	Dcmf R_ned_from_cam_T = R_cam_ned;
	R_ned_from_cam_T.transpose();
	_los_c_rate_dps = R_ned_from_cam_T * _los_e_rate_dps;
	_gyro_c_dps = _rot_body_to_cam * gyro_dps;

	// 读取地速并转换为地理系航向/俯仰角。
	// 当前版本仅保存该信息，尚未将其用于控制律。
	vehicle_local_position_s lpos{};
	_velocity_valid = _local_pos_sub.copy(&lpos) && lpos.v_xy_valid && lpos.v_z_valid;
	const Vector3f vel_ned(lpos.vx, lpos.vy, lpos.vz);
	_velocity_speed_ms = vel_ned.norm();
	_velocity_valid = _velocity_valid && (_velocity_speed_ms > 5.0f);

	if (_velocity_valid) {
		_velocity_e_deg(0) = math::degrees(atan2f(vel_ned(1), vel_ned(0)));
		_velocity_e_deg(1) = math::degrees(atan2f(-vel_ned(2), Vector2f(vel_ned(0), vel_ned(1)).norm()));

	} else {
		_velocity_e_deg.zero();
	}

	compute_control(att_deg, dt_s);
}

// ---------- 三阶段视觉引导控制器 ----------

void
DockGuidance::compute_control(const Vector3f &att_deg, float obs_dt_s)
{
	// 计算相机在 NED 中的姿态，用于水平滚转约束与前进俯仰目标。
	const Dcmf R_ned_body(Eulerf(math::radians(att_deg(0)),
				     math::radians(att_deg(1)),
				     math::radians(att_deg(2))));
	const Dcmf R_ned_cam = R_ned_body * _rot_cam_to_body;

	Eulerf cam_euler(R_ned_cam);
	_camera_e_deg(0) = math::degrees(cam_euler.phi());
	_camera_e_deg(1) = math::degrees(cam_euler.theta());
	_camera_e_deg(2) = math::degrees(cam_euler.psi());

	const float pitch_err_abs = fabsf(_los_c_deg(1));
	const float yaw_err_abs = fabsf(_los_c_deg(0));
	const float rate_limit = math::constrain(_param_dock_rate_max.get(), 1.0f, 360.0f);
	const float vis_limit = math::max(_param_dock_vis_lim.get(), 0.0f);
	const float vis_hysteresis = math::constrain(_param_dock_vis_hyst.get(), 0.0f, vis_limit);
	const float pitch_capture_enter = math::constrain(_param_dock_pcap_enter.get(), 10.0f, 89.0f);
	const float pitch_capture_exit = math::constrain(_param_dock_pcap_exit.get(), 5.0f, pitch_capture_enter);

	const float yaw_p_rate = _param_dock_yaw_p.get() * _los_c_deg(0);
	const float pitch_p_rate = _param_dock_pit_p.get() * _los_c_deg(1);
	const float level_rate = (fabsf(_camera_e_deg(1)) < 80.0f) ?
				 -_param_dock_rll_p.get() * _camera_e_deg(0) : 0.0f;

	const float active_vis_limit = (_stage == Stage::ANGLE_CAPTURE) ? vis_limit - vis_hysteresis : vis_limit;
	const bool outside_vis = (pitch_err_abs > active_vis_limit) || (yaw_err_abs > active_vis_limit);
	const bool pitch_capture = (_stage == Stage::PITCH_CAPTURE) ?
				   (pitch_err_abs > pitch_capture_exit) : (pitch_err_abs > pitch_capture_enter);

	if (pitch_capture) {
		// 阶段 1：目标接近相机上下边界时，优先仅调整俯仰，先将目标拉回可视区域。
		if (_stage != Stage::PITCH_CAPTURE) {
			PX4_INFO("dock stage PITCH_CAPTURE");
		}

		_stage = Stage::PITCH_CAPTURE;

		if (_los_rate_control_active) {
			_los_yaw_rate_pid_I = 0.0f;
			_los_pitch_rate_pid_I = 0.0f;
			_los_rate_control_active = false;
		}

		_target_rate_c_dps(0) = 0.0f;
		_target_rate_c_dps(1) = math::constrain(pitch_p_rate, -rate_limit, rate_limit);
		_target_rate_c_dps(2) = 0.0f;

	} else if (outside_vis) {
		// 阶段 2：目标仍在可视中心范围外，同时调整滚转水平、俯仰和偏航进行捕获。
		if (_stage != Stage::ANGLE_CAPTURE) {
			PX4_INFO("dock stage ANGLE_CAPTURE");
		}

		_stage = Stage::ANGLE_CAPTURE;

		if (_los_rate_control_active) {
			_los_yaw_rate_pid_I = 0.0f;
			_los_pitch_rate_pid_I = 0.0f;
			_los_rate_control_active = false;
		}

		_target_rate_c_dps(0) = math::constrain(level_rate, -rate_limit, rate_limit);
		_target_rate_c_dps(1) = math::constrain(pitch_p_rate, -rate_limit, rate_limit);
		_target_rate_c_dps(2) = math::constrain(yaw_p_rate, -rate_limit, rate_limit);

	} else {
		// 阶段 3：目标已在中心可视范围内。
		// 使用 LOS 角速度抑制目标漂移，同时叠加前进俯仰和残余角度修正以执行跟踪/对接。
		if (_stage != Stage::LOS_RATE) {
			PX4_INFO("dock stage LOS_RATE");
		}

		_stage = Stage::LOS_RATE;

		const float fwd_pitch_rate = math::constrain(
			_param_dock_fwd_pit.get() - _camera_e_deg(1), 0.0f, 5.0f);

		if (!_los_rate_control_active) {
			_los_yaw_rate_pid_I = 0.0f;
			_los_pitch_rate_pid_I = 0.0f;
			_los_rate_control_active = true;
		}

		const float nav_y_p = _param_dock_nav_y_p.get();
		const float nav_p_p = _param_dock_nav_p_p.get();

		_target_rate_c_dps(0) = math::constrain(level_rate, -rate_limit, rate_limit);
		_target_rate_c_dps(1) = math::constrain(
			-nav_p_p * _los_c_rate_dps(1) + fwd_pitch_rate + _param_dock_trc_p.get() * _los_c_deg(1),
			-rate_limit, rate_limit);
		_target_rate_c_dps(2) = math::constrain(
			-nav_y_p * _los_c_rate_dps(2) + _param_dock_trc_y.get() * _los_c_deg(0),
			-rate_limit, rate_limit);
	}

	apply_visibility_limit();
	// 控制律先在相机系生成；转换到机体系后，下游可直接作为飞行器角速度指令使用。
	const Vector3f requested_rate_b_dps = _rot_cam_to_body * _target_rate_c_dps;
	const float max_delta = math::constrain(_param_dock_rate_slew.get(), 1.0f, 2000.0f) *
				math::constrain(obs_dt_s, 0.002f, 0.1f);

	for (int axis = 0; axis < 3; ++axis) {
		_target_rate_b_dps(axis) += math::constrain(requested_rate_b_dps(axis) - _target_rate_b_dps(axis),
						     -max_delta, max_delta);
		_target_rate_b_dps(axis) = math::constrain(_target_rate_b_dps(axis), -rate_limit, rate_limit);
	}

	if (!_target_rate_b_dps.isAllFinite()) {
		invalidate_setpoint(hrt_absolute_time(), "non-finite output");
		return;
	}

	const float throttle = _param_dock_thr.get();
	const float throttle_rate = _param_dock_thr_rate.get();

	if (!std::isfinite(throttle) || !std::isfinite(throttle_rate)) {
		invalidate_setpoint(hrt_absolute_time(), "non-finite throttle");
		return;
	}

	// 发布独立的 Dock 引导设定值。当前模块不直接接管 PX4 的 vehicle_rates_setpoint。
	dock_guidance_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();
	sp.roll_rate_rad_s = math::radians(_target_rate_b_dps(0));
	sp.pitch_rate_rad_s = math::radians(_target_rate_b_dps(1));
	sp.yaw_rate_rad_s = math::radians(_target_rate_b_dps(2));
	sp.throttle = math::constrain(throttle, 0.0f, 1.0f);
	sp.throttle_rate = math::max(throttle_rate, 0.0f);
	sp.camera_yaw_deg = _los_c_deg(0);
	sp.camera_pitch_deg = _los_c_deg(1);
	sp.stage = static_cast<uint8_t>(_stage);
	sp.valid = true;
	_setpoint_pub.publish(sp);
	_setpoint_valid = true;

	if (_dock_mode_active) {
		publish_rates_setpoint(sp, _reset_rate_integral);
		_reset_rate_integral = false;
	}
}

void
DockGuidance::apply_visibility_limit()
{
	// 当目标已经靠近视场边界时，禁止会把目标进一步推离画面的指令方向。
	// 这不是对输出幅值限幅，而是基于目标偏差符号的方向保护。
	const float limit = math::max(_param_dock_vis_lim.get(), 0.0f);
	const float yaw_err = _los_c_deg(0);
	const float pitch_err = _los_c_deg(1);

	if ((yaw_err > limit) && (_target_rate_c_dps(2) < 0.0f)) {
		_target_rate_c_dps(2) = 0.0f;

	} else if ((yaw_err < -limit) && (_target_rate_c_dps(2) > 0.0f)) {
		_target_rate_c_dps(2) = 0.0f;
	}

	if ((pitch_err > limit) && (_target_rate_c_dps(1) < 0.0f)) {
		_target_rate_c_dps(1) = 0.0f;

	} else if ((pitch_err < -limit) && (_target_rate_c_dps(1) > 0.0f)) {
		_target_rate_c_dps(1) = 0.0f;
	}
}

// ---------- 数值计算辅助函数 ----------

float
DockGuidance::low_pass_filter(float &state, float input, float cutoff_hz, float dt)
{
	// 一阶 RC 低通：截止频率越低，视觉噪声抑制越强，但跟踪响应越慢。
	const float rc = 1.0f / (2.0f * M_PI_F * math::max(cutoff_hz, 0.01f));
	const float alpha = dt / (rc + dt);
	state = state + alpha * (input - state);
	return state;
}

float
DockGuidance::derivative_filter(const float *buf) const
{
	if (_deriv_count < 2) {
		return 0.0f;
	}

	// 按真实采样时刻做最小二乘直线拟合，在帧周期抖动和少量丢帧时仍可稳定估计导数。
	const int oldest = (_deriv_idx - _deriv_count + DERIV_SIZE) % DERIV_SIZE;
	const hrt_abstime time_origin = _los_deriv_time_buf[oldest];
	float sum_t = 0.0f;
	float sum_y = 0.0f;
	float sum_tt = 0.0f;
	float sum_ty = 0.0f;

	for (int n = 0; n < _deriv_count; ++n) {
		const int idx = (oldest + n) % DERIV_SIZE;
		const float t = static_cast<float>(_los_deriv_time_buf[idx] - time_origin) * 1e-6f;
		const float y = buf[idx];
		sum_t += t;
		sum_y += y;
		sum_tt += t * t;
		sum_ty += t * y;
	}

	const float denominator = _deriv_count * sum_tt - sum_t * sum_t;
	return (denominator > 1e-9f) ? (_deriv_count * sum_ty - sum_t * sum_y) / denominator : 0.0f;
}

void
DockGuidance::update_parameters()
{
	updateParams();
	update_camera_rotation();
}

void
DockGuidance::update_camera_rotation()
{
	// 三个参数定义从相机坐标系到 PX4 FRD 机体系的 3-2-1 旋转。
	// 默认 roll=-90 deg 与优化前的固定矩阵完全一致，因此升级后不改变当前安装行为。
	_rot_cam_to_body = Dcmf(Eulerf(math::radians(_param_dock_cam_roll.get()),
					 math::radians(_param_dock_cam_pitch.get()),
					 math::radians(_param_dock_cam_yaw.get())));
	_rot_body_to_cam = _rot_cam_to_body;
	_rot_body_to_cam.transpose();
}

int
DockGuidance::task_spawn(int argc, char *argv[])
{
	DockGuidance *instance = new DockGuidance();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
	}

	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int
DockGuidance::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
DockGuidance::print_status()
{
	PX4_INFO("Dock mode: %s", (_mode_id >= 0) ? "registered" : "not registered");
	PX4_INFO("mode id: %d, arming check id: %d", _mode_id, _arming_check_id);
	PX4_INFO("active: %s, vision: %s", _dock_mode_active ? "yes" : "no", _setpoint_valid ? "valid" : "invalid");
	return 0;
}

int
DockGuidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Visual docking guidance controller.  Subscribes to viztra_observation for
camera line-of-sight data and produces body-frame rate setpoints via a
three-stage controller (pitch capture → angle capture → LOS-rate tracking).
It registers a selectable external flight mode named "Dock Guidance". In that
mode only the PX4 rate controller and control allocation remain enabled.

### Usage

$ dock_guidance start
$ dock_guidance status
$ dock_guidance stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dock_guidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}
