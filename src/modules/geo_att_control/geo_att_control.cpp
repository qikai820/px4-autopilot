#include "geo_att_control.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;

GeoAttControl::GeoAttControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool GeoAttControl::init()
{
	if (!_angular_velocity_event.registerCallback()) {
		PX4_ERR("angular velocity callback failed");
		return false;
	}

	return true;
}

void GeoAttControl::resetIntegrators()
{
	_rate_integral.zero();
	_attitude_integral.zero();
}

Vector3f GeoAttControl::computeTorque(const Dcmf &R, const Dcmf &R_sp,
				     const Vector3f &omega, const Vector3f &omega_sp_body, float dt)
{
	// R 和 R_sp 均为机体到世界的方向余弦矩阵。反对称误差矩阵提取
	// 得到姿态误差向量；角速度期望先转换到当前机体系再参与速率环。
	const Dcmf error_matrix = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
	const Vector3f attitude_error(error_matrix(2, 1), error_matrix(0, 2), error_matrix(1, 0));
	const Vector3f rate_error = omega - omega_sp_body;
	const Vector3f attitude_gain(_p_roll_p.get(), _p_pitch_p.get(), _p_yaw_p.get());
	const Vector3f rate_gain(_p_roll_rate_p.get(), _p_pitch_rate_p.get(), _p_yaw_rate_p.get());
	const Vector3f rate_integral_gain(_p_roll_rate_i.get(), _p_pitch_rate_i.get(), _p_yaw_rate_i.get());
	const float integral_limit = math::max(_p_integral_limit.get(), 0.f);

	if (_armed && !_landed && dt > 0.f && dt <= 0.05f) {
		for (int i = 0; i < 3; ++i) {
			const float rate_delta = -rate_integral_gain(i) * rate_error(i) * dt;
			const bool blocked = (rate_delta > 0.f && _saturation_positive(i))
					     || (rate_delta < 0.f && _saturation_negative(i));
			if (!blocked) {
				_rate_integral(i) = math::constrain(_rate_integral(i) + rate_delta,
							   -integral_limit, integral_limit);
			}
			_attitude_integral(i) = math::constrain(_attitude_integral(i)
								 - _p_attitude_i.get() * attitude_error(i) * dt,
								 -integral_limit, integral_limit);
		}
	}

	// 刚体补偿项 omega × (J omega) 和参考系运输项提高快速姿态变化时
	// 的跟踪能力。惯量参数单位为 kg·m²，必须与实际机体坐标轴一致。
	const Matrix3f inertia = matrix::diag(Vector3f(math::max(_p_ixx.get(), 0.001f),
							 math::max(_p_iyy.get(), 0.001f),
							 math::max(_p_izz.get(), 0.001f)));
	const Vector3f rigid_body_compensation = omega.cross(inertia * omega);
	const Vector3f reference_transport = -inertia * omega.cross(omega_sp_body);

	return -attitude_gain.emult(attitude_error) - rate_gain.emult(rate_error)
	       + _rate_integral + _attitude_integral + rigid_body_compensation + reference_transport;
}

void GeoAttControl::Run()
{
	if (should_exit()) {
		_angular_velocity_event.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	vehicle_angular_velocity_s angular_event{};
	vehicle_angular_velocity_s angular_velocity{};
	vehicle_attitude_s attitude{};
	vehicle_status_s status{};
	vehicle_land_detected_s land{};

	if (!_angular_velocity_event.update(&angular_event) || !_angular_velocity_sub.copy(&angular_velocity)
	    || !_attitude_sub.copy(&attitude) || !_status_sub.copy(&status)) {
		return;
	}

	_land_sub.copy(&land);
	_landed = land.landed;
	_armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const hrt_abstime now = hrt_absolute_time();
	// 姿态内环由角速度消息唤醒，并由 GAC_RATE_HZ 限频；默认 400 Hz。
	const hrt_abstime period = static_cast<hrt_abstime>(1e6f /
		math::constrain(_p_rate_hz.get(), 50.f, 400.f));

	if (_last_run != 0 && now - _last_run < period) { return; }
	const float dt = (_last_run == 0) ? 0.0025f : math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.05f);
	_last_run = now;

	vehicle_rates_setpoint_s rates_sp{};
	vehicle_attitude_setpoint_s attitude_sp{};
	offboard_control_mode_s offboard_control_mode{};
	const bool rates_valid = _rates_sp_sub.copy(&rates_sp)
				 && hrt_elapsed_time(&rates_sp.timestamp) < 200_ms;
	const bool attitude_valid = _attitude_sp_sub.copy(&attitude_sp)
				    && hrt_elapsed_time(&attitude_sp.timestamp) < 200_ms;
	const bool offboard_mode_valid = _offboard_control_mode_sub.copy(&offboard_control_mode)
					 && hrt_elapsed_time(&offboard_control_mode.timestamp) < 500_ms;
	const bool mode_changed = status.nav_state != _last_nav_state;

	if (!_armed || _landed || mode_changed || (rates_valid && rates_sp.reset_integral)) {
		resetIntegrators();
	}

	_last_nav_state = status.nav_state;
	control_allocator_status_s allocator_status{};
	if (_allocator_status_sub.update(&allocator_status)) {
		_saturation_positive = Vector3<bool>(false, false, false);
		_saturation_negative = Vector3<bool>(false, false, false);
		if (!allocator_status.torque_setpoint_achieved) {
			for (int i = 0; i < 3; ++i) {
				_saturation_positive(i) = allocator_status.unallocated_torque[i] > FLT_EPSILON;
				_saturation_negative(i) = allocator_status.unallocated_torque[i] < -FLT_EPSILON;
			}
		}
	}

	const Quatf q(attitude.q);
	const Dcmf R(q);
	const Vector3f omega(angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]);
	Vector3f rates_reference{};
	if (rates_valid) { rates_reference = Vector3f(rates_sp.roll, rates_sp.pitch, rates_sp.yaw); }

	Quatf q_reference = q;
	if (attitude_valid) {
		q_reference = Quatf(attitude_sp.q_d);
		if (!PX4_ISFINITE(q_reference.norm()) || fabsf(q_reference.norm() - 1.f) > 0.05f) { q_reference = q; }
		q_reference.normalize();
	}

	// Offboard body_rate 模式必须使用纯角速度误差，不能被残留的姿态
	// 四元数期望重新引入姿态误差项。
	const bool offboard_rate_only = status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
					&& offboard_mode_valid && offboard_control_mode.body_rate;
	const bool rate_only = status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
			       || offboard_rate_only || (!attitude_valid && rates_valid);
	const Dcmf R_sp(rate_only ? q : q_reference);
	Vector3f omega_sp_body = rate_only ? rates_reference : R.transpose() * R_sp * rates_reference;

	// 姿态硬限幅：到达边界时只禁止继续向外的角速度指令，
	// 向内的指令仍然保留，用于把飞机拉回限制范围内。
	const Eulerf current_euler(q);
	const float roll_limit = math::radians(math::constrain(_p_roll_max.get(), 1.f, 85.f));
	const float pitch_limit = math::radians(math::constrain(_p_pitch_max.get(), 1.f, 85.f));
	if (!rate_only) {
		if (current_euler.phi() >= roll_limit && omega_sp_body(0) > 0.f) { omega_sp_body(0) = 0.f; }
		if (current_euler.phi() <= -roll_limit && omega_sp_body(0) < 0.f) { omega_sp_body(0) = 0.f; }
		if (current_euler.theta() >= pitch_limit && omega_sp_body(1) > 0.f) { omega_sp_body(1) = 0.f; }
		if (current_euler.theta() <= -pitch_limit && omega_sp_body(1) < 0.f) { omega_sp_body(1) = 0.f; }
	}
	Vector3f torque{};

	if (_armed && rates_valid) {
		torque = computeTorque(R, R_sp, omega, omega_sp_body, dt);
	}

	// 飞机未离地时不控制航向，避免解锁、起飞建立阶段的残留或外部
	// yaw 期望产生航向力矩，导致机体在地面旋转。
	if (_landed) {
		torque(2) = 0.f;
	}

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = now;
	torque_sp.timestamp_sample = angular_velocity.timestamp_sample;
	for (int i = 0; i < 3; ++i) { torque_sp.xyz[i] = torque(i); }
	_torque_pub.publish(torque_sp);

	const Matrix3f inertia = matrix::diag(Vector3f(math::max(_p_ixx.get(), 0.001f),
							 math::max(_p_iyy.get(), 0.001f),
							 math::max(_p_izz.get(), 0.001f)));
	const Vector3f angular_acceleration = inertia.I() * (torque - omega.cross(inertia * omega));
	vehicle_angular_acceleration_setpoint_s angular_acceleration_sp{};
	angular_acceleration_sp.timestamp = now;
	angular_acceleration_sp.timestamp_sample = angular_velocity.timestamp_sample;
	for (int i = 0; i < 3; ++i) { angular_acceleration_sp.xyz[i] = angular_acceleration(i); }
	_angular_acceleration_pub.publish(angular_acceleration_sp);

	geo_control_attitude_setpoint_s debug{};
	debug.timestamp = now;
	debug.timestamp_sample = angular_velocity.timestamp_sample;
	for (int i = 0; i < 4; ++i) { debug.q[i] = q_reference(i); }
	const Eulerf euler(q_reference);
	debug.roll = euler.phi();
	debug.pitch = euler.theta();
	debug.yaw = euler.psi();
	for (int i = 0; i < 3; ++i) { debug.rates[i] = rates_reference(i); }
	_debug_pub.publish(debug);
}

int GeoAttControl::task_spawn(int argc, char *argv[])
{
	GeoAttControl *instance = new GeoAttControl();
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

GeoAttControl *GeoAttControl::instantiate(int argc, char *argv[]) { return new GeoAttControl(); }
int GeoAttControl::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
int GeoAttControl::print_status() { PX4_INFO("400 Hz geometric attitude/rate controller"); return 0; }

int GeoAttControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION("MorphQuad geometric attitude and angular-rate controller.");
	PRINT_MODULE_USAGE_NAME("geo_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int geo_att_control_main(int argc, char *argv[])
{
	return GeoAttControl::main(argc, argv);
}
