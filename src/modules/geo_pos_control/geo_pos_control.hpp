#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

// 几何位置/速度外环。
//
// 本模块只负责计算世界 NED 坐标系中的期望力，并生成姿态、角速度和
// 位置期望；实际姿态力矩由 geo_att_control 在更高频率下计算，执行器
// 分配由 ControlAllocator 完成。这样位置环和姿态环的执行频率可以独立调整。
class GeoPosControl final : public ModuleBase<GeoPosControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	GeoPosControl();
	~GeoPosControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static GeoPosControl *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	void Run() override;
	void resetState(const matrix::Quatf &q, const matrix::Vector3f &pos,
			const matrix::Vector3f &vel);
	void publishSetpoints(const matrix::Vector3f &force_body,
			      const matrix::Vector3f &vel_sp,
			      const matrix::Vector3f &accel_sp,
			      const matrix::Vector3f &rates_sp,
			      uint64_t timestamp_sample, bool publish_attitude_rates);
	// 根据位置/速度误差计算世界 NED 坐标系期望力，单位为 N。
	matrix::Vector3f positionForce(const matrix::Vector3f &pos, const matrix::Vector3f &vel,
				       const matrix::Vector3f &vel_stick, float dt);
	void limitAttitudeReference();

	uORB::SubscriptionCallbackWorkItem _local_position_event{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _trajectory_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s> _rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_position_sp_pub{ORB_ID(vehicle_local_position_setpoint)};

	// 位置外环维护的姿态期望。manual/position 模式下由摇杆速率积分得到，
	// 并由 limitAttitudeReference() 限制滚转和俯仰范围；航向由 yaw 摇杆控制。
	matrix::Quatf _attitude_reference{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _position_reference{};
	matrix::Vector3f _velocity_limited{};
	matrix::Vector3f _velocity_integral{};
	bool _reference_initialized{false};
	bool _landed{true};
	bool _armed{false};
	bool _level_hold_active{false};
	bool _hold_active{false};
	uint8_t _last_nav_state{vehicle_status_s::NAVIGATION_STATE_MAX};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GPC_RATE_HZ>) _p_rate_hz,
		(ParamFloat<px4::params::GPC_MASS>) _p_mass,
		// 与 PX4 多旋翼位置控制和着陆检测共用同一个悬停油门参数。
		(ParamFloat<px4::params::MPC_THR_HOVER>) _p_hover_thrust,
		// 直接复用 PX4 原生多旋翼的推力输出上下限。
		(ParamFloat<px4::params::MPC_THR_MIN>) _p_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _p_mpc_thr_max,
		(ParamFloat<px4::params::GPC_THR_MAX>) _p_thrust_max,
		(ParamFloat<px4::params::GPC_XY_P>) _p_xy_p,
		(ParamFloat<px4::params::GPC_Z_P>) _p_z_p,
		(ParamFloat<px4::params::GPC_XY_VP>) _p_xy_vel_p,
		(ParamFloat<px4::params::GPC_Z_VP>) _p_z_vel_p,
		(ParamFloat<px4::params::GPC_VEL_I>) _p_vel_i,
		(ParamFloat<px4::params::GPC_VEL_ILIM>) _p_vel_ilim,
		(ParamFloat<px4::params::GPC_VX_MAX>) _p_vx_max,
		(ParamFloat<px4::params::GPC_VY_MAX>) _p_vy_max,
		(ParamFloat<px4::params::GPC_VZ_MAX>) _p_vz_max,
		(ParamFloat<px4::params::GPC_AX_MAX>) _p_ax_max,
		(ParamFloat<px4::params::GPC_AY_MAX>) _p_ay_max,
		(ParamFloat<px4::params::GPC_AZ_MAX>) _p_az_max,
		(ParamFloat<px4::params::GPC_MAN_DZ>) _p_manual_deadzone,
		(ParamFloat<px4::params::GPC_MAN_FMAX>) _p_manual_force_max,
		(ParamFloat<px4::params::GPC_RATE_MAX>) _p_manual_rate_max,
		(ParamFloat<px4::params::GPC_ROLL_MAX>) _p_roll_max,
		(ParamFloat<px4::params::GPC_PITCH_MAX>) _p_pitch_max
	)
};
