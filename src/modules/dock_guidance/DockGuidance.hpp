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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/viztra_observation.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/dock_guidance_setpoint.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/matrix/math.hpp>

#include <cmath>

using namespace time_literals;

class DockGuidance : public ModuleBase<DockGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DockGuidance();
	~DockGuidance() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	// 延时补偿历史长度、LOS 导数窗口长度，以及首帧使用的默认周期。
	static constexpr int DELAY_BUF_SIZE = 601;
	static constexpr int DERIV_SIZE = 7;
	static constexpr float LOOP_DT = 0.01f;
	static constexpr uint32_t WATCHDOG_INTERVAL_US = 10_ms;

	enum class Stage : uint8_t {
		NONE = 0,
		ALIGN = 1,
		ACCELERATE = 2,
		MAX_SPEED_APPROACH = 3,
		TERMINAL_BRAKE = 4,
		RECOVERY = 5
	};

	void Run() override;
	void update_parameters();
	void update_camera_rotation();
	void register_dock_mode(hrt_abstime now);
	void check_mode_registration();
	void configure_dock_mode(hrt_abstime now);
	void unregister_dock_mode();
	void reply_to_arming_check(const arming_check_request_s &request);
	void update_mode_state();
	void publish_velocity_setpoint(hrt_abstime timestamp, const matrix::Vector3f &velocity_ned,
				       float yaw_setpoint_rad);
	void publish_safe_velocity_setpoint(hrt_abstime timestamp);
	void handle_visual_loss(hrt_abstime timestamp, const char *reason);
	void publish_degraded_setpoint(hrt_abstime timestamp);
	void publish_guidance_status(hrt_abstime timestamp, bool valid, bool degraded);
	void set_stage(Stage stage);

	// 保存并读取与相机画面时间对齐的飞行器状态。
	void push_delay_history(hrt_abstime timestamp);
	bool get_delayed_state(hrt_abstime timestamp_sample, matrix::Vector3f &att_deg,
			       matrix::Vector3f &gyro_dps) const;
	void invalidate_setpoint(hrt_abstime timestamp, const char *reason);
	void reset_tracking_state();

	void process_observation(float los_yaw_deg, float los_pitch_deg,
				 const matrix::Vector3f &relative_position_ned,
				 bool relative_position_valid,
				 const matrix::Vector3f &att_deg,
				 const matrix::Vector3f &gyro_dps,
				 float dt_s, hrt_abstime timestamp_sample);
	void compute_control(const matrix::Vector3f &att_deg, float obs_dt_s, hrt_abstime now);

	float low_pass_filter(float &state, float input, float cutoff_hz, float dt);
	float derivative_filter(const float *buf) const;

	// 视觉观测为本模块的执行触发源；其余话题在每次视觉帧到达时读取最新值。
	uORB::SubscriptionCallbackWorkItem _viztra_sub{this, ORB_ID(viztra_observation)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _gyro_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// 对接引导输出，供未来的 Dock 模式仲裁器或控制器订阅。
	uORB::Publication<dock_guidance_setpoint_s> _setpoint_pub{ORB_ID(dock_guidance_setpoint)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};

	// 相机系与 PX4 机体系（FRD）之间的可配置安装旋转矩阵。
	matrix::Matrix3f _rot_body_to_cam;
	matrix::Matrix3f _rot_cam_to_body;

	// LOS 分别在相机系（c）和地理 NED 系（e）中的方向、导数与角速度。
	matrix::Vector3f _los_c_deg{0.0f, 0.0f, 0.0f};
	matrix::Vector2f _los_e_deg{0.0f, 0.0f};
	matrix::Vector3f _los_e_unit{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _los_e_dot{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _los_e_rate_dps{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _camera_e_deg{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _gyro_c_dps{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _los_c_rate_dps{0.0f, 0.0f, 0.0f};

	// 低通后的 LOS 分量，以及用于窗口差分的历史数据。
	float _los_x_filt{0.0f};
	float _los_y_filt{0.0f};
	float _los_z_filt{0.0f};
	bool _filt_initialized{false};

	float _los_x_deriv_buf[DERIV_SIZE] {};
	float _los_y_deriv_buf[DERIV_SIZE] {};
	float _los_z_deriv_buf[DERIV_SIZE] {};
	hrt_abstime _los_deriv_time_buf[DERIV_SIZE] {};
	int _deriv_count{0};
	int _deriv_idx{0};

	// 当前视觉捕获阶段与视觉目标状态。
	Stage _stage{Stage::NONE};
	uint64_t _last_observation_us{0};
	uint64_t _last_sample_us{0};
	uint64_t _last_valid_observation_us{0};
	bool _setpoint_valid{false};
	bool _dock_mode_active{false};
	bool _vision_degraded{false};
	bool _unregistration_sent{false};
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};
	uint64_t _mode_request_id{0};
	hrt_abstime _last_registration_request_us{0};
	hrt_abstime _last_mode_config_us{0};

	// 环形历史缓冲：以四元数保存姿态，避免欧拉角跨界和奇异点插值错误。
	matrix::Quatf _att_delay_buf[DELAY_BUF_SIZE] {};
	matrix::Vector3f _gyro_delay_buf[DELAY_BUF_SIZE] {};
	hrt_abstime _delay_time_buf[DELAY_BUF_SIZE] {};
	int _delay_idx{0};
	int _delay_count{0};

	matrix::Vector3f _velocity_ned{0.0f, 0.0f, 0.0f};
	float _velocity_speed_ms{0.0f};
	float _closing_speed_ms{0.0f};
	bool _velocity_valid{false};
	matrix::Vector2f _velocity_e_deg{0.0f, 0.0f};
	matrix::Vector3f _relative_position_ned{0.0f, 0.0f, 0.0f};
	bool _relative_position_valid{false};
	float _relative_distance_m{NAN};
	matrix::Vector3f _velocity_setpoint_ned{0.0f, 0.0f, 0.0f};
	float _yaw_setpoint_rad{NAN};
	float _commanded_speed_ms{0.0f};
	float _fov_ratio{0.0f};
	hrt_abstime _aligned_since_us{0};
	hrt_abstime _last_control_update_us{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DOCK_CAM_DLY>)   _param_dock_cam_dly,
		(ParamInt<px4::params::DOCK_OBS_TOUT>)  _param_dock_obs_tout,
		(ParamFloat<px4::params::DOCK_CAM_ROLL>) _param_dock_cam_roll,
		(ParamFloat<px4::params::DOCK_CAM_PITCH>) _param_dock_cam_pitch,
		(ParamFloat<px4::params::DOCK_CAM_YAW>) _param_dock_cam_yaw,
		(ParamFloat<px4::params::DOCK_FILT_HZ>) _param_dock_filt_hz,
		(ParamFloat<px4::params::DOCK_RES_X>)   _param_dock_res_x,
		(ParamFloat<px4::params::DOCK_RES_Y>)   _param_dock_res_y,
		(ParamFloat<px4::params::DOCK_FOV_X>)   _param_dock_fov_x,
		(ParamFloat<px4::params::DOCK_FOV_Y>)   _param_dock_fov_y,
		(ParamFloat<px4::params::DOCK_LOS_MAX>) _param_dock_los_max,
		(ParamFloat<px4::params::DOCK_VEL_MAX>) _param_dock_vel_max,
		(ParamFloat<px4::params::DOCK_ACC_MAX>) _param_dock_acc_max,
		(ParamFloat<px4::params::DOCK_DEC_MAX>) _param_dock_dec_max,
		(ParamFloat<px4::params::DOCK_STOP_DIST>) _param_dock_stop_dist,
		(ParamFloat<px4::params::DOCK_ALIGN_T>) _param_dock_align_t,
		(ParamFloat<px4::params::DOCK_FOV_ALIGN>) _param_dock_fov_align,
		(ParamFloat<px4::params::DOCK_FOV_SOFT>) _param_dock_fov_soft,
		(ParamFloat<px4::params::DOCK_FOV_HARD>) _param_dock_fov_hard,
		(ParamFloat<px4::params::DOCK_ALIGN_SPD>) _param_dock_align_speed,
		(ParamInt<px4::params::DOCK_VIS_GRACE>) _param_dock_vis_grace,
		(ParamInt<px4::params::DOCK_REL_TOUT>) _param_dock_rel_tout,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_down
	)
};
