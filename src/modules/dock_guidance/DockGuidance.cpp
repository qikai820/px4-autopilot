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

using namespace matrix;

DockGuidance::DockGuidance() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 相机坐标系约定：X 向前、Y 向右、Z 向下。
	// 机体系遵循 PX4 FRD：X 向前、Y 向右、Z 向下。
	// 本安装矩阵用于把相机视线/角速度指令转换到机体系；安装方向改变时应在此处修改。
	_rot_cam_to_body(0, 0) = 1.0f;  _rot_cam_to_body(0, 1) = 0.0f; _rot_cam_to_body(0, 2) = 0.0f;
	_rot_cam_to_body(1, 0) = 0.0f;  _rot_cam_to_body(1, 1) = 0.0f; _rot_cam_to_body(1, 2) = 1.0f;
	_rot_cam_to_body(2, 0) = 0.0f;  _rot_cam_to_body(2, 1) = -1.0f; _rot_cam_to_body(2, 2) = 0.0f;

	_rot_body_to_cam = _rot_cam_to_body;
	_rot_body_to_cam.transpose();
}

DockGuidance::~DockGuidance()
{
}

bool
DockGuidance::init()
{
	// 模块由视觉观测话题触发，而非固定周期定时器。
	// 每收到一帧 viztra_observation，Run() 执行一次。
	if (!_viztra_sub.registerCallback()) {
		PX4_ERR("viztra callback registration failed");
		return false;
	}

	return true;
}

void
DockGuidance::Run()
{
	if (should_exit()) {
		_viztra_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s p;
		_parameter_update_sub.copy(&p);
		update_parameters();
	}

	viztra_observation_s obs;

	if (!_viztra_sub.update(&obs)) {
		return;
	}

	// 使用两帧视觉观测之间的时间间隔作为滤波和控制计算的 dt。
	// 这里限幅可避免首帧、丢帧或突发帧导致数值不稳定。
	const uint64_t now_us = hrt_absolute_time();
	const float run_dt_s = (_last_run_us == 0) ? LOOP_DT :
			       math::constrain((now_us - _last_run_us) * 1e-6f, 0.002f, 0.05f);
	_last_run_us = now_us;

	// 保存当前姿态与角速度历史，供相机延时补偿读取。
	push_delay_history();

	// 无目标或输入非法时，清空内部状态并明确发布 invalid，
	// 使下游控制器可以立即放弃视觉引导。
	if (!obs.valid || !std::isfinite(obs.offset_x) || !std::isfinite(obs.offset_y)) {
		if (_stage != Stage::NONE) {
			PX4_INFO("dock: no target");
		}

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
		_last_observation_us = 0;
		_los_yaw_rate_pid_I = 0.0f;
		_los_pitch_rate_pid_I = 0.0f;
		_filt_initialized = false;

		dock_guidance_setpoint_s sp{};
		sp.timestamp = now_us;
		sp.valid = false;
		sp.stage = 0;
		_setpoint_pub.publish(sp);
		return;
	}

	// 读取飞行器当前状态。延时补偿实际使用的是下方缓冲区中的历史状态。
	vehicle_attitude_s att;
	_att_sub.copy(&att);
	const Quatf q(att.q);
	const Dcmf R_ned_body(q);

	vehicle_angular_velocity_s av;
	_gyro_sub.copy(&av);
	const Vector3f gyro_b_rads(av.xyz[0], av.xyz[1], av.xyz[2]);

	// 取与相机画面时间对应的姿态/角速度，减小图像传输和识别延迟带来的误差。
	Vector3f att_deg;
	Vector3f gyro_dps;
	get_delayed_state(att_deg, gyro_dps);

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

	process_observation(los_yaw_deg, los_pitch_deg, att_deg, gyro_dps, run_dt_s);
}

// ---------- 姿态/角速度延时缓冲 ----------

void
DockGuidance::push_delay_history()
{
	// 将 PX4 四元数姿态转换为欧拉角，统一以度保存，便于后续与视觉角度相加。
	vehicle_attitude_s att;
	_att_sub.copy(&att);
	const Quatf q(att.q);
	const Dcmf R(q);
	Vector3f euler_deg;
	Eulerf euler(R);
	euler_deg(0) = math::degrees(euler.phi());
	euler_deg(1) = math::degrees(euler.theta());
	euler_deg(2) = math::degrees(euler.psi());

	vehicle_angular_velocity_s av;
	_gyro_sub.copy(&av);
	Vector3f gyro_dps(math::degrees(av.xyz[0]),
			  math::degrees(av.xyz[1]),
			  math::degrees(av.xyz[2]));

	_att_delay_buf[_delay_idx] = euler_deg;
	_gyro_delay_buf[_delay_idx] = gyro_dps;
	_delay_idx = (_delay_idx + 1) % DELAY_BUF_SIZE;
}

bool
DockGuidance::get_delayed_state(Vector3f &att_deg, Vector3f &gyro_dps) const
{
	// 根据相机延时参数从环形缓冲区回溯。
	// 注意：当前实现假设历史样本约为 100 Hz，且仅在视觉帧到达时写入历史。
	const int delay_steps = math::constrain(
		(int)(_param_dock_cam_dly.get() * 100), 0, DELAY_BUF_SIZE - 1);

	int idx = _delay_idx - 1 - delay_steps;

	if (idx < 0) {
		idx += DELAY_BUF_SIZE;
	}

	att_deg = _att_delay_buf[idx];
	gyro_dps = _gyro_delay_buf[idx];
	return true;
}

// ---------- 视觉观测处理 ----------

void
DockGuidance::process_observation(float los_yaw_deg, float los_pitch_deg,
				  const Vector3f &att_deg,
				  const Vector3f &gyro_dps,
				  float dt_s)
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

	_los_e_deg(0) = matrix::wrap_pi(math::degrees(atan2f(_los_e_unit(1), _los_e_unit(0))));
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

	// 保存最近 7 个滤波后的视线向量，用首尾差分估计 LOS 变化率。
	if (_deriv_count < DERIV_SIZE) {
		_los_x_deriv_buf[_deriv_count] = _los_x_filt;
		_los_y_deriv_buf[_deriv_count] = _los_y_filt;
		_los_z_deriv_buf[_deriv_count] = _los_z_filt;
		_los_deriv_time_buf[_deriv_count] = (float)hrt_absolute_time();
		_deriv_count++;

	} else {
		for (int i = 0; i < DERIV_SIZE - 1; i++) {
			_los_x_deriv_buf[i] = _los_x_deriv_buf[i + 1];
			_los_y_deriv_buf[i] = _los_y_deriv_buf[i + 1];
			_los_z_deriv_buf[i] = _los_z_deriv_buf[i + 1];
			_los_deriv_time_buf[i] = _los_deriv_time_buf[i + 1];
		}

		_los_x_deriv_buf[DERIV_SIZE - 1] = _los_x_filt;
		_los_y_deriv_buf[DERIV_SIZE - 1] = _los_y_filt;
		_los_z_deriv_buf[DERIV_SIZE - 1] = _los_z_filt;
		_los_deriv_time_buf[DERIV_SIZE - 1] = (float)hrt_absolute_time();
	}

	if (_deriv_count >= DERIV_SIZE) {
		const float total_dt = (_los_deriv_time_buf[DERIV_SIZE - 1] - _los_deriv_time_buf[0]) * 1e-6f;

		if (total_dt > 0.001f) {
			const float slope_x = derivative_filter(_los_x_deriv_buf, DERIV_SIZE, total_dt);
			const float slope_y = derivative_filter(_los_y_deriv_buf, DERIV_SIZE, total_dt);
			const float slope_z = derivative_filter(_los_z_deriv_buf, DERIV_SIZE, total_dt);

			_los_e_dot = Vector3f(slope_x, slope_y, slope_z);
			_los_e_dot -= _los_e_unit * (_los_e_unit * _los_e_dot);
		}
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
	vehicle_local_position_s lpos;
	_local_pos_sub.copy(&lpos);
	_velocity_valid = lpos.v_xy_valid && lpos.v_z_valid;
	const Vector3f vel_ned(lpos.vx, lpos.vy, lpos.vz);
	_velocity_speed_ms = vel_ned.norm();
	_velocity_valid = _velocity_valid && (_velocity_speed_ms > 5.0f);

	if (_velocity_valid) {
		_velocity_e_deg(0) = math::degrees(atan2f(vel_ned(1), vel_ned(0)));
		_velocity_e_deg(1) = math::degrees(atan2f(-vel_ned(2), Vector2f(vel_ned(0), vel_ned(1)).norm()));

	} else {
		_velocity_e_deg.zero();
	}

	_last_observation_us = hrt_absolute_time();

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
	const float rate_limit = math::max(_param_dock_rate_max.get(), 30.0f);
	const float vis_limit = math::max(_param_dock_vis_lim.get(), 0.0f);

	const float yaw_p_rate = _param_dock_yaw_p.get() * _los_c_deg(0);
	const float pitch_p_rate = _param_dock_pit_p.get() * _los_c_deg(1);
	const float level_rate = (fabsf(_camera_e_deg(1)) < 80.0f) ?
				 -_param_dock_rll_p.get() * _camera_e_deg(0) : 0.0f;

	const bool outside_vis = (pitch_err_abs > vis_limit) || (yaw_err_abs > vis_limit);

	if (pitch_err_abs > 60.0f) {
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
	_target_rate_b_dps = _rot_cam_to_body * _target_rate_c_dps;

	// 发布独立的 Dock 引导设定值。当前模块不直接接管 PX4 的 vehicle_rates_setpoint。
	dock_guidance_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();
	sp.roll_rate_rad_s = math::radians(_target_rate_b_dps(0));
	sp.pitch_rate_rad_s = math::radians(_target_rate_b_dps(1));
	sp.yaw_rate_rad_s = math::radians(_target_rate_b_dps(2));
	sp.throttle = _param_dock_thr.get();
	sp.throttle_rate = _param_dock_thr_rate.get();
	sp.camera_yaw_deg = _los_c_deg(0);
	sp.camera_pitch_deg = _los_c_deg(1);
	sp.stage = static_cast<uint8_t>(_stage);
	sp.valid = true;
	_setpoint_pub.publish(sp);
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
DockGuidance::derivative_filter(float *buf, int size, float dt_s)
{
	// 使用窗口首尾样本做差分，避免直接差分单帧视觉数据带来的高频噪声。
	if (size < 2 || dt_s <= 0.0f) {
		return 0.0f;
	}

	return (buf[size - 1] - buf[0]) / (dt_s * (size - 1));
}

void
DockGuidance::update_parameters()
{
	updateParams();
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
