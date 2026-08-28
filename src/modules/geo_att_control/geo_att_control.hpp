#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/geo_control_attitude_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

// 几何姿态/角速度内环。
// 以 vehicle_angular_velocity 为唤醒源，默认 400 Hz；根据姿态四元数和
// 机体角速度期望计算 Lee SO(3) 姿态控制力矩，并交给 ControlAllocator。
class GeoAttControl final : public ModuleBase<GeoAttControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	GeoAttControl();
	~GeoAttControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static GeoAttControl *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	void Run() override;
	void resetIntegrators();
	matrix::Vector3f computeTorque(const matrix::Dcmf &R, const matrix::Dcmf &R_sp,
				       const matrix::Vector3f &omega,
				       const matrix::Vector3f &omega_sp_body, float dt);

	uORB::SubscriptionCallbackWorkItem _angular_velocity_event{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_angular_acceleration_setpoint_s> _angular_acceleration_pub{
		ORB_ID(vehicle_angular_acceleration_setpoint)};
	uORB::Publication<geo_control_attitude_setpoint_s> _debug_pub{ORB_ID(geo_control_attitude_setpoint)};

	// 角速度环和姿态环积分项。未解锁、未离地、模式切换或收到
	// reset_integral 时清零，避免旧模式误差带入新模式。
	matrix::Vector3f _rate_integral{};
	matrix::Vector3f _attitude_integral{};
	matrix::Vector3<bool> _saturation_positive{false, false, false};
	matrix::Vector3<bool> _saturation_negative{false, false, false};
	bool _landed{true};
	bool _armed{false};
	uint8_t _last_nav_state{vehicle_status_s::NAVIGATION_STATE_MAX};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GAC_RATE_HZ>) _p_rate_hz,
		(ParamFloat<px4::params::GAC_ROLL_P>) _p_roll_p,
		(ParamFloat<px4::params::GAC_PITCH_P>) _p_pitch_p,
		(ParamFloat<px4::params::GAC_YAW_P>) _p_yaw_p,
		(ParamFloat<px4::params::GAC_R_RATE_P>) _p_roll_rate_p,
		(ParamFloat<px4::params::GAC_P_RATE_P>) _p_pitch_rate_p,
		(ParamFloat<px4::params::GAC_Y_RATE_P>) _p_yaw_rate_p,
		(ParamFloat<px4::params::GAC_R_RATE_I>) _p_roll_rate_i,
		(ParamFloat<px4::params::GAC_P_RATE_I>) _p_pitch_rate_i,
		(ParamFloat<px4::params::GAC_Y_RATE_I>) _p_yaw_rate_i,
		(ParamFloat<px4::params::GAC_ATT_I>) _p_attitude_i,
		(ParamFloat<px4::params::GAC_INT_LIM>) _p_integral_limit,
		(ParamFloat<px4::params::GAC_ROLL_MAX>) _p_roll_max,
		(ParamFloat<px4::params::GAC_PITCH_MAX>) _p_pitch_max,
		(ParamFloat<px4::params::GAC_YAW_THR_MIN>) _p_yaw_throttle_min,
		(ParamFloat<px4::params::GAC_IXX>) _p_ixx,
		(ParamFloat<px4::params::GAC_IYY>) _p_iyy,
		(ParamFloat<px4::params::GAC_IZZ>) _p_izz
	)
};
