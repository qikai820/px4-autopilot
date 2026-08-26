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
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/geo_control_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
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
	int print_status() override;

	/** 注入固定 6D 力/力矩，从真实 ControlAllocator 读回各旋翼电机/舵机输出，
	 *  验证 MorphQuad 控制分配。 */
	int verify_allocation();

	/** Acro 模式: 发布固定力/力矩并读回实际力/力矩用于验证。 */
	void control_step_acro(uint64_t timestamp_sample);
	int acro_command(int argc, char *argv[]);

	/** 悬停测试: 注入 Fz=-mass*g，监控速度收敛。 */
	int hover_command(int argc, char *argv[]);

	/** Offboard 测试: 内部注入 trajectory_setpoint 实现闭环位置控制。 */
	void control_step_offboard_test(uint64_t timestamp_sample);
	int offboard_command(int argc, char *argv[]);
	bool _offboard_test_active{false};
	uint64_t _offboard_test_dur_us{0};
	hrt_abstime _offboard_test_start{0};
	matrix::Vector3f _offboard_pos_sp{};
	float _offboard_yaw_sp{0.f};

	// Rate- and attitude-only MAVLink commands do not carry a position setpoint.
	// Hold their entry altitude so a closed-loop test remains in a safe envelope.
	bool _offboard_external_alt_hold_active{false};
	float _offboard_external_alt_sp{0.f};

	/** 姿态测试: 跟踪期望欧拉角 (roll, pitch, yaw)。 */
	void control_step_att_test(uint64_t timestamp_sample);
	int att_test_command(int argc, char *argv[]);
	bool _att_test_active{false};
	uint64_t _att_test_duration_us{0};
	hrt_abstime _att_test_start{0};
	float _att_test_roll{0.f};
	float _att_test_pitch{0.f};
	float _att_test_yaw{0.f};
	matrix::Vector3f _att_test_error_last{};
	matrix::Vector3f _att_test_omega_last{};
	bool _att_test_feedback_valid{false};

	/** 自动姿态闭环验证: 依次施加 roll/pitch/yaw 正负阶跃并评分。 */
	void control_step_att_verify(uint64_t timestamp_sample);
	int att_verify_command(int argc, char *argv[]);
	bool _att_verify_active{false};
	int _att_verify_step{0};
	hrt_abstime _att_verify_step_start{0};
	matrix::Vector3f _att_verify_error_sum{};
	float _att_verify_rate_sum{0.f};
	int _att_verify_score_count{0};
	int _att_verify_pass_count{0};

	/** 速度测试: 跟踪期望机体速度 (含姿态内环)。 */
	void control_step_vel_test(uint64_t timestamp_sample);
	int vel_test_command(int argc, char *argv[]);
	bool _vel_test_active{false};
	matrix::Vector3f _vel_test_sp{};

	/** Sweep 模式: 自动 6D 力/力矩逐轴扫频。 */
	void control_step_sweep(uint64_t timestamp_sample);
	int sweep_command(int argc, char *argv[]);

	bool init();

private:
	enum class ControlMode {
		MANUAL = 0,
		OFFBOARD = 1,
		AUTO = 2,
		POSITION = 3,
		ACRO = 4,
		ALTITUDE = 5
	};

	void Run() override;
	void parameters_update(bool force = false);
	void control_step(const vehicle_local_position_s &local_pos);
	void update_manual_level_hold(const matrix::Quatf &R_q, bool enabled);
	void limit_manual_attitude_reference();

	void control_step_manual(const matrix::Quatf &R_q, const matrix::Vector3f &omega,
				 uint64_t timestamp_sample);
	void control_step_altitude(const matrix::Quatf &R_q, const matrix::Vector3f &vel,
				   const matrix::Vector3f &omega, uint64_t timestamp_sample);

	void control_step_position(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
				   const matrix::Vector3f &vel, const matrix::Vector3f &omega,
				   uint64_t timestamp_sample);

	void control_step_offboard(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
				   const matrix::Vector3f &vel, const matrix::Vector3f &omega,
				   uint64_t timestamp_sample);

	/** Offboard body-rate command path (MAVLink SET_ATTITUDE_TARGET body rates). */
	void control_step_offboard_rates(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
					 const matrix::Vector3f &vel,
					 const matrix::Vector3f &omega,
					 const vehicle_rates_setpoint_s &rate_sp,
					 uint64_t timestamp_sample);

	void control_step_auto(const matrix::Quatf &R_q, const matrix::Vector3f &pos,
			       const matrix::Vector3f &vel, const matrix::Vector3f &omega,
			       uint64_t timestamp_sample);

	/**
	 * Lee SO(3) 姿态力矩，含陀螺耦合补偿:
	 *   τ = -kR·e_R - kΩ·e_Ω + Ω×JΩ - J(Ω×RᵀRdΩd)
	 * 不对外部速率设定值作有限差分。Offboard 指令频率低于控制
	 * 周期时，该差分会形成不可实现的力矩脉冲。
	 * e_Ω 必须先经过积分 (见 updateIntegrators)，使 _rate_int 在调用时有效。
	 * @param R            当前姿态 (world-from-body)
	 * @param R_sp         期望姿态 (world-from-body)
	 * @param omega        当前机体角速度
	 * @param omega_sp     期望机体角速度 (速率参考)
	 * @param omega_d_in_B RᵀRd·Ωd, 在当前机体坐标系中表示的期望角速率
	 * @return 期望力矩 (机体坐标系)
	 */
	matrix::Vector3f computeAttitudeTorque(const matrix::Dcmf &R, const matrix::Dcmf &R_sp,
					       const matrix::Vector3f &omega, const matrix::Vector3f &omega_sp,
					       const matrix::Vector3f &omega_d_in_B, float dt);

	void publish_wrench(const matrix::Vector3f &f_des_body, const matrix::Vector3f &tau_des,
			    uint64_t timestamp_sample);

	/**
	 * 发布调试指令 (local-position / angular-acceleration)，
	 * 便于用 `listener`/ulog 检查期望力/力矩。归一化油门估计
	 * (总力占比, 悬停≈0.41, 满油≈1.0) 填入
	 * vehicle_local_position_setpoint.thrust[3]。
	 * 注意这是总力占比 ‖f‖/(N_rotors*TMAX)，不同于单路电机指令
	 * sqrt(T_i/TMAX) ≈ 0.64 (悬停时)。
	 */
	void publish_setpoints(const matrix::Vector3f &f_des_world, const matrix::Vector3f &f_des_body,
			       const matrix::Vector3f &tau_des, const matrix::Quatf &q_att_ref,
			       const matrix::Vector3f &vel_sp_world, const matrix::Vector3f &accel_sp_world,
			       const matrix::Vector3f &omega,
			       const matrix::Vector3f &omega_sp,
			       uint64_t timestamp_sample);

	/**
	 * 更新积分器，含执行器饱和反馈抗饱和。
	 * 遵循 mc_rate_control 的 RateControl 模式。
	 * @param e_omega    角速度误差 (机体坐标系)
	 * @param e_v_world  速度误差 (NED 世界坐标系)
	 * @param dt         时间步长 [s]
	 */
	void updateIntegrators(const matrix::Vector3f &e_omega, const matrix::Vector3f &e_v_world,
			       float dt);

	/**
	 * 清零所有积分器。
	 */
	void resetIntegrators();

	// uORB 订阅
	// Use attitude setpoint publication as the control-loop wake-up source.
	uORB::SubscriptionCallbackWorkItem _sub_vehicle_attitude_setpoint_event{this, ORB_ID(vehicle_attitude_setpoint)};
	// vehicle_attitude is published at the IMU/control rate in configurations
	// that do not publish a separate attitude setpoint topic.
	uORB::SubscriptionCallbackWorkItem _sub_vehicle_attitude_event{this, ORB_ID(vehicle_attitude)};
	// Fallback for modes/configurations that do not publish vehicle_attitude_setpoint.
	uORB::SubscriptionCallbackWorkItem _sub_vehicle_local_position_event{this, ORB_ID(vehicle_local_position)};
	// Position remains a regular subscription and is copied at each control tick.
	uORB::Subscription _sub_vehicle_local_position{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};
	uORB::Subscription _sub_vehicle_attitude_setpoint{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _sub_vehicle_angular_velocity{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _sub_vehicle_rates_setpoint{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _sub_local_pos_sp{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::Subscription _sub_vehicle_land_detected{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _sub_manual_control_sp{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _sub_trajectory_sp{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _sub_control_allocator_status{ORB_ID(control_allocator_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// uORB 发布
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_angular_acceleration_setpoint_s> _ang_accel_sp_pub{ORB_ID(vehicle_angular_acceleration_setpoint)};
	uORB::Publication<geo_control_attitude_setpoint_s> _attitude_sp_debug_pub{ORB_ID(geo_control_attitude_setpoint)};

	// 参数 (GC_ 前缀)
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GC_MASS>) _param_gc_mass,
		(ParamFloat<px4::params::GC_HOVER_THR>) _param_gc_hover_thr,
		(ParamFloat<px4::params::GC_RATE_HZ>) _param_gc_rate_hz,
		(ParamFloat<px4::params::GC_XY_P>) _param_gc_xy_p,
		(ParamFloat<px4::params::GC_XY_VEL_P>) _param_gc_xy_vel_p,
		(ParamFloat<px4::params::GC_Z_P>) _param_gc_z_p,
		(ParamFloat<px4::params::GC_Z_VEL_P>) _param_gc_z_vel_p,
		(ParamFloat<px4::params::GC_RP_P>) _param_gc_rp_p,
		(ParamFloat<px4::params::GC_ROLL_P>) _param_gc_roll_p,
		(ParamFloat<px4::params::GC_PITCH_P>) _param_gc_pitch_p,
		(ParamFloat<px4::params::GC_YAW_P>) _param_gc_yaw_p,
		(ParamFloat<px4::params::GC_RP_RATE_P>) _param_gc_rp_rate_p,
		(ParamFloat<px4::params::GC_ROLL_RATE_P>) _param_gc_roll_rate_p,
		(ParamFloat<px4::params::GC_PITCH_RATE_P>) _param_gc_pitch_rate_p,
		(ParamFloat<px4::params::GC_YAW_RATE_P>) _param_gc_yaw_rate_p,
		(ParamFloat<px4::params::GC_INERTIA_XX>) _param_gc_inertia_xx,
		(ParamFloat<px4::params::GC_INERTIA_YY>) _param_gc_inertia_yy,
		(ParamFloat<px4::params::GC_INERTIA_ZZ>) _param_gc_inertia_zz,
		(ParamFloat<px4::params::GC_MAN_RATE_MAX>) _param_gc_man_rate_max,
		(ParamFloat<px4::params::GC_MAN_VEL_MAX>) _param_gc_man_vel_max,
		(ParamFloat<px4::params::GC_VX_MAX>) _param_gc_vx_max,
		(ParamFloat<px4::params::GC_VY_MAX>) _param_gc_vy_max,
		(ParamFloat<px4::params::GC_VZ_MAX>) _param_gc_vz_max,
		(ParamFloat<px4::params::GC_AX_MAX>) _param_gc_ax_max,
		(ParamFloat<px4::params::GC_AY_MAX>) _param_gc_ay_max,
		(ParamFloat<px4::params::GC_AZ_MAX>) _param_gc_az_max,
		(ParamFloat<px4::params::GC_ROLL_MAX>) _param_gc_roll_max,
		(ParamFloat<px4::params::GC_PITCH_MAX>) _param_gc_pitch_max,
		(ParamFloat<px4::params::GC_MAN_DZ>) _param_gc_man_dz,
		(ParamFloat<px4::params::GC_RATE_I>) _param_gc_rate_i,
		(ParamFloat<px4::params::GC_ROLL_RATE_I>) _param_gc_roll_rate_i,
		(ParamFloat<px4::params::GC_PITCH_RATE_I>) _param_gc_pitch_rate_i,
		(ParamFloat<px4::params::GC_YAW_RATE_I>) _param_gc_yaw_rate_i,
		(ParamFloat<px4::params::GC_ATT_I>) _param_gc_att_i,
		(ParamFloat<px4::params::GC_RATE_INT_LIM>) _param_gc_rate_int_lim,
		(ParamFloat<px4::params::GC_VEL_I>) _param_gc_vel_i,
		(ParamFloat<px4::params::GC_VEL_INT_LIM>) _param_gc_vel_int_lim,
		(ParamFloat<px4::params::GC_THRUST_MAX>) _param_gc_thrust_max,
		(ParamFloat<px4::params::GC_MAN_FMAX>) _param_gc_man_fmax,
		// MorphQuad 分配几何参数 (从 CA_MQ_* 参数读取，用于重构)
		(ParamFloat<px4::params::CA_MQ_L>) _param_ca_mq_l,
		(ParamFloat<px4::params::CA_MQ_B>) _param_ca_mq_b,
		(ParamFloat<px4::params::CA_MQ_Z>) _param_ca_mq_z,
		(ParamFloat<px4::params::CA_MQ_TMAX>) _param_ca_mq_tmax
	)

	// 积分器状态
	matrix::Vector3f _rate_int{};   ///< 姿态/角速率误差积分器 (力矩环)
	matrix::Vector3f _att_int{};    ///< 姿态误差积分器，用于消除稳态角度误差
	matrix::Vector3f _vel_int{};    ///< 水平速度误差积分器 (力环)
	float _vel_int_z_accel{0.f};    ///< 垂向速度误差积分器 (加速度环)

	// 最新的外部 body-rate 指令。速率环在每个控制周期运行，直到该
	// 指令超时，不能仅在 50 Hz 的 MAVLink 新消息到达时运行。
	vehicle_rates_setpoint_s _offboard_rate_sp{};
	hrt_abstime _offboard_rate_sp_timestamp{0};

	// 控制分配器饱和反馈
	matrix::Vector<bool, 3> _torque_sat_pos{};
	matrix::Vector<bool, 3> _torque_sat_neg{};
	matrix::Vector<bool, 3> _thrust_sat_pos{};
	matrix::Vector<bool, 3> _thrust_sat_neg{};

	// 状态标志
	bool _landed{true};
	bool _armed{false};
	uint8_t _nav_state_last{0};   ///< 最近一次 vehicle_status.nav_state, 供 `status` 诊断

	// 分配验证 (`verify` 命令): 固定力/力矩注入+读回。
	// 激活时 control_step() 每周期注入 _verify_wrench (绕过正常
	// 模式/解锁调度)，使 ControlAllocator 在已知力/力矩上运行。
	bool _verify_active{false};
	matrix::Vector<float, 6> _verify_wrench{};   ///< [fx,fy,fz,tx,ty,tz] 机体 FRD, N / N·m

	// Acro 模式: 开环固定力/力矩注入，用于控制分配测试。
	bool _acro_active{false};
	matrix::Vector<float, 6> _acro_wrench{};      ///< [fx,fy,fz,tx,ty,tz] 机体 FRD
	uint64_t _acro_duration_us{0};                 ///< 超时后自动关闭 (0=持续)
	hrt_abstime _acro_start_time{0};               ///< acro 模式激活时刻
	int _acro_reset_hold{0};                       ///< 短暂零力/力矩，复位舵机
	static constexpr int ACRO_RESET_HOLD = 125;    ///< ~500ms 零力/力矩保持，舵机复位

	// Sweep 模式: 自动 6D 力/力矩逐维扫频。
	bool _sweep_active{false};
	int _sweep_axis{0};           ///< 当前轴 (0=Fx,1=Fy,2=Fz,3=Tx,4=Ty,5=Tz)
	int _sweep_step{0};           ///< 当前轴内步数 (0.._sweep_samples-1)
	int _sweep_samples{10};       ///< 每轴采样点数 (0 → max)
	float _sweep_max[6]{};        ///< 各轴最大值
	int _sweep_hold_count{0};     ///< 当前采样点剩余保持步数
	int _sweep_reset_hold{0};     ///< 扫频前短暂零力/力矩复位舵机
	static constexpr int SWEEP_HOLD = 60;  ///< 每采样点保持 ~240ms (足够分配器收敛)
	static constexpr int SWEEP_RESET_HOLD = 125;  ///< ~500ms 零力/力矩保持，舵机复位

	// 手动模式: 角速率指令积分得到的姿态参考
	matrix::Quatf _q_att_ref{1.f, 0.f, 0.f, 0.f};
	bool _manual_level_hold_active{false};
	orb_advert_t _mavlink_log_pub{nullptr};

	// 位置模式: 位置保持用积分位置指令 ((重新)解锁时初始化)
	matrix::Vector3f _pos_sp{};
	bool _pos_sp_init{false};
	matrix::Vector3f _vel_sp_limited{};
	bool _vel_sp_limited_init{false};

	hrt_abstime _last_run{0};
	perf_counter_t _loop_perf;

	// 最近一次期望力/力矩 (机体坐标系), 通过 `info` 命令查看。
	// publish_wrench() 中更新; custom_command 读取。
	matrix::Vector3f _tau_des_last{};   ///< 最近期望力矩 [N·m], 机体坐标系
	matrix::Vector3f _f_des_last{};     ///< 最近期望力 [N], 机体坐标系
	hrt_abstime _wrench_timestamp{0};   ///< 最近力/力矩更新时间戳
};
