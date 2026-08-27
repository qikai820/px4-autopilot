#include "geo_pos_control.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;

GeoPosControl::GeoPosControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool GeoPosControl::init()
{
	if (!_local_position_event.registerCallback()) {
		PX4_ERR("local position callback failed");
		return false;
	}

	return true;
}

void GeoPosControl::resetState(const Quatf &q, const Vector3f &pos, const Vector3f &vel)
{
	_attitude_reference = q;
	_position_reference = pos;
	_velocity_limited = vel;
	_velocity_integral.zero();
	_reference_initialized = true;
}

void GeoPosControl::limitAttitudeReference()
{
	const Eulerf euler(_attitude_reference);
	const float roll_limit = math::radians(math::constrain(_p_roll_max.get(), 1.f, 85.f));
	const float pitch_limit = math::radians(math::constrain(_p_pitch_max.get(), 1.f, 85.f));
	_attitude_reference = Quatf(Eulerf(math::constrain(euler.phi(), -roll_limit, roll_limit),
					      math::constrain(euler.theta(), -pitch_limit, pitch_limit), euler.psi()));
	_attitude_reference.normalize();
}

Vector3f GeoPosControl::positionForce(const Vector3f &pos, const Vector3f &vel,
				      const Vector3f &vel_stick, float dt)
{
	// 位置环结构：位置误差经过 P 环生成速度期望，速度期望经过加速度
	// 限幅后进入速度 P/I 环。积分器在地面清零，避免起飞时带着地面误差积分。
	const Vector3f velocity_limit(math::max(_p_vx_max.get(), 0.1f),
				      math::max(_p_vy_max.get(), 0.1f),
				      math::max(_p_vz_max.get(), 0.1f));
	const Vector3f acceleration_limit(math::max(_p_ax_max.get(), 0.1f),
					  math::max(_p_ay_max.get(), 0.1f),
					  math::max(_p_az_max.get(), 0.1f));

	if (vel_stick.norm() > 0.05f) {
		_position_reference += vel_stick * dt;
	}

	Vector3f velocity_sp = vel_stick + Vector3f(_p_xy_p.get(), _p_xy_p.get(), _p_z_p.get())
				       .emult(_position_reference - pos);

	for (int i = 0; i < 3; ++i) {
		velocity_sp(i) = math::constrain(velocity_sp(i), -velocity_limit(i), velocity_limit(i));
		const float delta = math::constrain(velocity_sp(i) - _velocity_limited(i),
						   -acceleration_limit(i) * dt, acceleration_limit(i) * dt);
		_velocity_limited(i) += delta;
	}

	const Vector3f velocity_error = vel - _velocity_limited;
	const float integral_limit = math::max(_p_vel_ilim.get(), 0.f);

	if (!_landed) {
		for (int i = 0; i < 3; ++i) {
			_velocity_integral(i) = math::constrain(_velocity_integral(i)
							 + _p_vel_i.get() * velocity_error(i) * dt,
							 -integral_limit, integral_limit);
		}
	}

	Vector3f acceleration_sp(-_p_xy_vel_p.get() * velocity_error(0) - _velocity_integral(0),
				 -_p_xy_vel_p.get() * velocity_error(1) - _velocity_integral(1),
				 -_p_z_vel_p.get() * velocity_error(2) - _velocity_integral(2));

	for (int i = 0; i < 3; ++i) {
		acceleration_sp(i) = math::constrain(acceleration_sp(i), -acceleration_limit(i), acceleration_limit(i));
	}

	// 水平力使用质量乘以期望加速度；垂向力使用标定悬停推力换算，
	// 这样即使质量参数存在小误差，也不会直接改变垂向速度环的增益。
	const float mass = math::max(_p_mass.get(), 0.1f);
	const float thrust_max = math::max(_p_thrust_max.get(), 1.f);
	const float hover_force = math::constrain(_p_hover_thrust.get(), 0.05f, 0.95f) * thrust_max;
	Vector3f force_world(mass * acceleration_sp(0), mass * acceleration_sp(1),
			     hover_force * (acceleration_sp(2) / 9.81f - 1.f));
	return force_world;
}

void GeoPosControl::publishSetpoints(const Vector3f &force_body, const Vector3f &vel_sp,
				     const Vector3f &accel_sp, const Vector3f &rates_sp,
				     uint64_t timestamp_sample, bool publish_attitude_rates)
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = now;
	thrust.timestamp_sample = timestamp_sample;
	for (int i = 0; i < 3; ++i) { thrust.xyz[i] = force_body(i); }
	_thrust_pub.publish(thrust);

	if (publish_attitude_rates) {
		vehicle_attitude_setpoint_s attitude_sp{};
		attitude_sp.timestamp = now;
		for (int i = 0; i < 4; ++i) { attitude_sp.q_d[i] = _attitude_reference(i); }
		attitude_sp.yaw_sp_move_rate = rates_sp(2);
		_attitude_sp_pub.publish(attitude_sp);

		vehicle_rates_setpoint_s rate_sp{};
		rate_sp.timestamp = now;
		rate_sp.roll = rates_sp(0);
		rate_sp.pitch = rates_sp(1);
		rate_sp.yaw = rates_sp(2);
		rate_sp.reset_integral = _landed || !_armed;
		_rates_sp_pub.publish(rate_sp);
	}

	vehicle_local_position_setpoint_s local_sp{};
	local_sp.timestamp = now;
	local_sp.x = _position_reference(0);
	local_sp.y = _position_reference(1);
	local_sp.z = _position_reference(2);
	local_sp.vx = vel_sp(0);
	local_sp.vy = vel_sp(1);
	local_sp.vz = vel_sp(2);
	for (int i = 0; i < 3; ++i) { local_sp.acceleration[i] = accel_sp(i); }
	local_sp.yaw = Eulerf(_attitude_reference).psi();
	local_sp.yawspeed = rates_sp(2);
	_local_position_sp_pub.publish(local_sp);
}

void GeoPosControl::Run()
{
	if (should_exit()) {
		_local_position_event.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	vehicle_local_position_s local{};
	vehicle_attitude_s attitude{};
	vehicle_status_s status{};
	vehicle_land_detected_s land{};
	vehicle_local_position_s event{};

	if (!_local_position_event.update(&event) || !_local_position_sub.copy(&local)
	    || !_attitude_sub.copy(&attitude) || !_status_sub.copy(&status)) {
		return;
	}

	_land_sub.copy(&land);
	_landed = land.landed;
	_armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const hrt_abstime now = hrt_absolute_time();
	// 回调由 vehicle_local_position 唤醒，但实际执行频率由 GPC_RATE_HZ
	// 再次限频。默认 50 Hz，避免位置环以 IMU 频率重复计算。
	const hrt_abstime period = static_cast<hrt_abstime>(1e6f /
		math::constrain(_p_rate_hz.get(), 10.f, 100.f));

	if (_last_run != 0 && now - _last_run < period) { return; }
	const float dt = (_last_run == 0) ? 0.02f : math::constrain((now - _last_run) * 1e-6f, 0.005f, 0.1f);
	_last_run = now;

	const Quatf q(attitude.q);
	const Dcmf R(q);
	const Vector3f pos(local.x, local.y, local.z);
	const Vector3f vel(local.vx, local.vy, local.vz);
	const bool mode_changed = status.nav_state != _last_nav_state;

	if (!_reference_initialized || !_armed || mode_changed) {
		resetState(q, pos, vel);
	}

	_last_nav_state = status.nav_state;
	manual_control_setpoint_s manual{};
	const bool manual_valid = _manual_sub.copy(&manual) && manual.valid
				  && hrt_elapsed_time(&manual.timestamp) < 500_ms;
	const float dz = math::constrain(_p_manual_deadzone.get(), 0.f, 0.5f);
	Vector3f rates_sp{};
	Vector3f velocity_sp{};
	Vector3f acceleration_sp{};
	Vector3f force_body{};
	bool publish_attitude_rates = status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	if (!_armed) {
		publishSetpoints(force_body, velocity_sp, acceleration_sp, rates_sp, local.timestamp_sample, true);
		return;
	}

	if (manual_valid) {
		rates_sp = Vector3f(math::deadzone(manual.roll, dz), -math::deadzone(manual.pitch, dz),
				    _landed ? 0.f : math::deadzone(manual.yaw, dz)) * _p_manual_rate_max.get();
	}

	const bool rate_only = status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO;
	const bool position_mode = status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
				   || status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const bool altitude_mode = status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	if (!rate_only) {
		const float rate_norm = rates_sp.norm();
		if (rate_norm > 1e-6f) {
			const float angle = rate_norm * dt;
			const Vector3f axis = rates_sp / rate_norm;
			const float half_angle = angle * 0.5f;
			const float sine = sinf(half_angle);
			_attitude_reference = _attitude_reference
					      * Quatf(cosf(half_angle), axis(0) * sine, axis(1) * sine, axis(2) * sine);
			_attitude_reference.normalize();
		}

		limitAttitudeReference();
	}

	if (position_mode) {
		// aux1/aux2 是机体系水平速度指令，先按当前航向旋转到世界 NED；
		// throttle 是垂向速度指令。位置期望随速度指令积分更新。
		const float yaw = Eulerf(q).psi();
		const float vx_body = manual_valid ? math::deadzone(manual.aux1, dz) * _p_vx_max.get() : 0.f;
		const float vy_body = manual_valid ? math::deadzone(manual.aux2, dz) * _p_vy_max.get() : 0.f;
		const float vz = manual_valid ? -math::deadzone(manual.throttle, dz) * _p_vz_max.get() : 0.f;
		velocity_sp = Vector3f(cosf(yaw) * vx_body - sinf(yaw) * vy_body,
				       sinf(yaw) * vx_body + cosf(yaw) * vy_body, vz);
		const Vector3f force_world = positionForce(pos, vel, velocity_sp, dt);
		force_body = R.transpose() * force_world;

	} else if (altitude_mode) {
		// 定高模式只闭合垂向速度，水平位置冻结在当前点；roll/pitch 摇杆
		// 直接产生手动水平力，故该模式不是水平位置闭环。
		velocity_sp(2) = manual_valid ? -math::deadzone(manual.throttle, dz) * _p_vz_max.get() : 0.f;
		_position_reference(0) = pos(0);
		_position_reference(1) = pos(1);
		const Vector3f force_world = positionForce(pos, vel, velocity_sp, dt);
		force_body = R.transpose() * Vector3f(0.f, 0.f, force_world(2));
		if (manual_valid) {
			force_body(0) += math::deadzone(manual.pitch, dz) * _p_manual_force_max.get();
			force_body(1) += math::deadzone(manual.roll, dz) * _p_manual_force_max.get();
		}

	} else if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		// Offboard 位置/速度指令使用轨迹期望；收到有效轨迹时由本模块发布
		// 姿态期望。外部姿态或角速度模式下不覆盖对应的 uORB 期望。
		trajectory_setpoint_s trajectory{};
		if (_trajectory_sub.copy(&trajectory) && hrt_elapsed_time(&trajectory.timestamp) < 500_ms) {
			// Offboard 位置/速度控制由本外环生成姿态和角速度期望；
			// Offboard 姿态或角速度控制则保留外部发布的期望。
			publish_attitude_rates = true;
			for (int i = 0; i < 3; ++i) {
				if (PX4_ISFINITE(trajectory.position[i])) { _position_reference(i) = trajectory.position[i]; }
				velocity_sp(i) = PX4_ISFINITE(trajectory.velocity[i]) ? trajectory.velocity[i] : 0.f;
			}

			// 位置控制只使用外部的航向期望，滚转和俯仰由位置环保持水平。
			const float yaw_reference = PX4_ISFINITE(trajectory.yaw)
						    ? trajectory.yaw : Eulerf(_attitude_reference).psi();
			_attitude_reference = Quatf(Eulerf(0.f, 0.f, yaw_reference));
			_attitude_reference.normalize();
			rates_sp(2) = PX4_ISFINITE(trajectory.yawspeed) ? trajectory.yawspeed : 0.f;
		}
		force_body = R.transpose() * positionForce(pos, vel, velocity_sp, dt);

	} else {
		const float throttle = manual_valid ? math::constrain((manual.throttle + 1.f) * 0.5f, 0.f, 1.f) : 0.f;
		const float fz = -_p_thrust_max.get() * throttle * throttle;
		force_body = R.transpose() * Vector3f(0.f, 0.f, fz);

		if (manual_valid && !rate_only) {
			force_body(0) += math::deadzone(manual.pitch, dz) * _p_manual_force_max.get();
			force_body(1) += math::deadzone(manual.roll, dz) * _p_manual_force_max.get();
		}
	}

	publishSetpoints(force_body, _velocity_limited, acceleration_sp, rates_sp,
			 local.timestamp_sample, publish_attitude_rates);
}

int GeoPosControl::task_spawn(int argc, char *argv[])
{
	GeoPosControl *instance = new GeoPosControl();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
	}
	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

GeoPosControl *GeoPosControl::instantiate(int argc, char *argv[]) { return new GeoPosControl(); }
int GeoPosControl::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
int GeoPosControl::print_status() { PX4_INFO("50 Hz geometric position/force controller"); return 0; }

int GeoPosControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION("MorphQuad geometric position, velocity and force controller.");
	PRINT_MODULE_USAGE_NAME("geo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int geo_pos_control_main(int argc, char *argv[])
{
	return GeoPosControl::main(argc, argv);
}
