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
	_position_adaptive_velocity.zero();
	_velocity_disturbance_acceleration.zero();
	_previous_acceleration_command.zero();
	_acceleration_observation.zero();
	_velocity_adaptation_filter.reset(Vector3f{});
	_reference_initialized = true;
	_adaptive_previous_valid = false;
	_adaptive_valid = false;
	_previous_collective_valid = false;
	_hover_thrust_learning = false;
	_vertical_capture_active = false;
	_vertical_capture_velocity = 0.f;
	_vertical_capture_direction = 0.f;
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

void GeoPosControl::updateVerticalCapture(bool mode_changed, bool vertical_control_enabled,
					  const Vector3f &pos, const Vector3f &vel,
					  float dt, float &vertical_velocity_command)
{
	const float command_deadzone = 0.05f;
	const float stop_threshold = math::constrain(_p_vertical_capture_velocity_threshold.get(), 0.02f, 1.f);
	const bool command_quiet = fabsf(vertical_velocity_command) < command_deadzone;

	if (!vertical_control_enabled || !command_quiet || !_armed || _landed) {
		_vertical_capture_active = false;
		_vertical_capture_velocity = 0.f;
		_vertical_capture_direction = 0.f;
		return;
	}

	if (mode_changed && fabsf(vel(2)) > stop_threshold) {
		_vertical_capture_active = true;
		_vertical_capture_velocity = vel(2);
		_vertical_capture_direction = vel(2) > 0.f ? 1.f : -1.f;
	}

	if (!_vertical_capture_active) {
		return;
	}

	const float brake_acceleration = math::constrain(_p_vertical_capture_acceleration.get(), 0.1f, 10.f);
	const float velocity_delta = math::constrain(-_vertical_capture_velocity,
						     -brake_acceleration * dt, brake_acceleration * dt);
	_vertical_capture_velocity += velocity_delta;
	vertical_velocity_command = _vertical_capture_velocity;

	// Latch the altitude where the vehicle actually stops.  Waiting for both
	// the reference and measured velocity to become small avoids returning to
	// the altitude captured at mode entry.  A velocity sign change means the
	// brake has already crossed zero, so latch immediately before a descent can
	// develop.
	const bool velocity_crossed_zero = _vertical_capture_direction * vel(2) <= 0.f;
	const bool stopped = fabsf(_vertical_capture_velocity) <= stop_threshold
			     && fabsf(vel(2)) <= stop_threshold;

	if (velocity_crossed_zero || stopped) {
		_vertical_capture_active = false;
		_vertical_capture_velocity = 0.f;
		_vertical_capture_direction = 0.f;
		vertical_velocity_command = 0.f;
		_position_reference(2) = pos(2);
		_velocity_limited(2) = 0.f;
	}
}

Vector3f GeoPosControl::positionForce(const Vector3f &pos, const Vector3f &vel,
				      const Vector3f &vel_stick, const Vector3f &acceleration_measured,
				      const Dcmf &attitude, float dt,
				      Vector3f &acceleration_sp)
{
	// 位置环结构：位置误差经过 P 环生成速度期望，速度期望经过加速度
	// 限幅后进入速度 P/I 环。积分器在地面清零，避免起飞时带着地面误差积分。
	const Vector3f velocity_limit(math::max(_p_vx_max.get(), 0.1f),
				      math::max(_p_vy_max.get(), 0.1f),
				      math::max(_p_vz_max.get(), 0.1f));
	const float vertical_acceleration_limit = _vertical_capture_active
						? math::max(_p_az_max.get(), _p_vertical_capture_acceleration.get())
						: _p_az_max.get();
	const Vector3f acceleration_limit(math::max(_p_ax_max.get(), 0.1f),
				      math::max(_p_ay_max.get(), 0.1f),
				      math::max(vertical_acceleration_limit, 0.1f));

	if (vel_stick.norm() > 0.05f) {
		_position_reference += vel_stick * dt;
	}

	const Vector3f position_error = _position_reference - pos;
	Vector3f velocity_sp = vel_stick + Vector3f(_p_xy_p.get(), _p_xy_p.get(), _p_z_p.get())
				       .emult(position_error);

	const int32_t adaptive_mode = math::constrain(_p_adaptive_mode.get(), int32_t{0}, int32_t{2});
	const float adaptive_position_limit = math::max(_p_adaptive_position_limit.get(), 0.f);
	const float adaptive_position_i = math::max(_p_adaptive_position_i.get(), 0.f);
	// Only learn while the requested velocity is quiet. During forward/backward
	// acceleration the position error is intentional and must not be learned as
	// a persistent disturbance.
	const bool quiet_command = vel_stick.norm() < 0.05f && velocity_sp.norm() < 0.5f;
	const bool adaptation_window = adaptive_mode > 0 && !_landed && _armed
				       && quiet_command && dt > 0.f && dt <= 0.1f
				       && position_error.isAllFinite();

	if (adaptation_window) {
		for (int i = 0; i < 3; ++i) {
			_position_adaptive_velocity(i) = math::constrain(
				_position_adaptive_velocity(i) + adaptive_position_i * position_error(i) * dt,
				-adaptive_position_limit, adaptive_position_limit);
		}
	}

	if (adaptive_mode == 2) {
		velocity_sp += _position_adaptive_velocity;
	}

	for (int i = 0; i < 3; ++i) {
		velocity_sp(i) = math::constrain(velocity_sp(i), -velocity_limit(i), velocity_limit(i));
		const float delta = math::constrain(velocity_sp(i) - _velocity_limited(i),
						   -acceleration_limit(i) * dt, acceleration_limit(i) * dt);
		_velocity_limited(i) += delta;
	}

	// 水平力使用质量乘以期望加速度；垂向力使用标定悬停推力换算，
	// 这样即使质量参数存在小误差，也不会直接改变垂向速度环的增益。
	const float mass = math::max(_p_mass.get(), 0.1f);
	const float thrust_max = math::max(_p_thrust_max.get(), 1.f);
	const float min_thrust = math::constrain(_p_mpc_thr_min.get(), 0.f, 1.f);
	const float max_thrust = math::constrain(_p_mpc_thr_max.get(), min_thrust, 1.f);
	const float hover_ratio = math::constrain(_hover_thrust_estimate, math::max(min_thrust, 0.01f), max_thrust);
	const float hover_force = hover_ratio * thrust_max;

	const Vector3f velocity_error = vel - _velocity_limited;
	const float integral_limit = math::max(_p_vel_ilim.get(), 0.f);
	const Vector3f velocity_gain(_p_xy_vel_p.get(), _p_xy_vel_p.get(), _p_z_vel_p.get());

	// 垂向加速度除了受 GPC_AZ_MAX 限制，还受实际可用集体推力限制：
	// collective = hover_ratio * (1 - az/g)。把该关系反解为 az 的
	// 上下界后用于抗积分饱和，防止油门已经到顶/到底时积分仍继续累积。
	Vector3f acceleration_min = -acceleration_limit;
	Vector3f acceleration_max = acceleration_limit;
	acceleration_min(2) = math::max(acceleration_min(2), 9.81f * (1.f - max_thrust / hover_ratio));
	acceleration_max(2) = math::min(acceleration_max(2), 9.81f * (1.f - min_thrust / hover_ratio));

	if (!_landed) {
		for (int i = 0; i < 3; ++i) {
			const float integral_delta = _p_vel_i.get() * velocity_error(i) * dt;
			const float candidate_integral = math::constrain(_velocity_integral(i) + integral_delta,
									-integral_limit, integral_limit);
			const float candidate_acceleration = -velocity_gain(i) * velocity_error(i) - candidate_integral;

			// 只禁止把输出进一步推向饱和方向；允许反向积分立即卸载已有积分。
			const bool drives_high_saturation = candidate_acceleration > acceleration_max(i) && integral_delta < 0.f;
			const bool drives_low_saturation = candidate_acceleration < acceleration_min(i) && integral_delta > 0.f;

			if (!drives_high_saturation && !drives_low_saturation) {
				_velocity_integral(i) = candidate_integral;
			}
		}
	}

	acceleration_sp = -velocity_gain.emult(velocity_error) - _velocity_integral;

	// Estimate an acceleration disturbance from the measured NED acceleration
	// and the previous raw acceleration command. This is a velocity-loop
	// disturbance observer; filtering prevents sensor noise from becoming a
	// high-frequency force command.
	const float acceleration_limit_adaptive = math::max(_p_adaptive_acceleration_limit.get(), 0.f);
	_adaptive_valid = false;
	if (adaptation_window && _adaptive_previous_valid && acceleration_measured.isAllFinite()
	    && acceleration_measured.norm() < 50.f) {
		_acceleration_observation = acceleration_measured - _previous_acceleration_command;

		for (int i = 0; i < 3; ++i) {
			_acceleration_observation(i) = math::constrain(_acceleration_observation(i),
									 -acceleration_limit_adaptive, acceleration_limit_adaptive);
		}

		const float cutoff = math::constrain(_p_adaptive_cutoff.get(), 0.05f, 20.f);
		_velocity_adaptation_filter.setParameters(dt, 1.f / cutoff);
		_velocity_disturbance_acceleration = _velocity_adaptation_filter.update(_acceleration_observation);
		_adaptive_valid = true;
	}

	if (adaptive_mode == 2) {
		acceleration_sp -= _velocity_disturbance_acceleration;
	}

	for (int i = 0; i < 3; ++i) {
		acceleration_sp(i) = math::constrain(acceleration_sp(i), acceleration_min(i), acceleration_max(i));
	}

	// Estimate the collective required for zero vertical acceleration separately
	// from the generic disturbance observer.  For a near-level vehicle in NED:
	//   a_z = g * (1 - collective / hover_thrust)
	// hence hover_thrust = collective / (1 - a_z/g).  The previous collective
	// is paired with the current measured acceleration to respect the one-cycle
	// sensor/control delay.  Learning is deliberately gated to quiet commands,
	// near-level attitude and available actuator authority.
	const float collective_command = math::constrain(
		hover_ratio * (1.f - acceleration_sp(2) / 9.81f), min_thrust, max_thrust);
	const float collective_margin = 0.01f;
	const bool collective_unsaturated = collective_command > min_thrust + collective_margin
					    && collective_command < max_thrust - collective_margin;
	const float tilt_limit = math::radians(math::constrain(_p_hover_estimator_tilt.get(), 1.f, 45.f));
	const bool attitude_level = attitude(2, 2) > cosf(tilt_limit);
	const bool hover_quiet_command = vel_stick.norm() < 0.05f;
	const bool acceleration_valid = PX4_ISFINITE(acceleration_measured(2))
					&& fabsf(acceleration_measured(2)) < 5.f;
	_hover_thrust_learning = _p_hover_estimator_enable.get() != 0 && _armed && !_landed
				 && hover_quiet_command && attitude_level && collective_unsaturated
				 && _previous_collective_valid && acceleration_valid
				 && dt > 0.f && dt <= 0.1f;

	if (_hover_thrust_learning) {
		const float denominator = 1.f - acceleration_measured(2) / 9.81f;

		if (denominator > 0.25f && denominator < 1.75f) {
			const float estimate_min = math::constrain(_p_hover_estimator_min.get(),
								   math::max(min_thrust, 0.05f), max_thrust);
			const float estimate_max = math::constrain(_p_hover_estimator_max.get(), estimate_min, max_thrust);
			_hover_thrust_observation = math::constrain(_previous_collective_command / denominator,
								   estimate_min, estimate_max);
			const float gain = math::constrain(_p_hover_estimator_gain.get(), 0.f, 5.f);
			const float alpha = math::constrain(gain * dt, 0.f, 1.f);
			_hover_thrust_estimate = math::constrain(
				_hover_thrust_estimate + alpha * (_hover_thrust_observation - _hover_thrust_estimate),
				estimate_min, estimate_max);
		}
	}

	_previous_collective_command = collective_command;
	_previous_collective_valid = collective_unsaturated;

	// Store the acceleration command that is actually sent after limiting. If
	// the command becomes non-quiet (for example, a forward stick input), the
	// next sample must not compare against an old maneuver command.
	_previous_acceleration_command = acceleration_sp;
	_adaptive_previous_valid = adaptation_window && acceleration_measured.isAllFinite();

	Vector3f force_world(mass * acceleration_sp(0), mass * acceleration_sp(1),
			     hover_force * (acceleration_sp(2) / 9.81f - 1.f));
	return force_world;
}

void GeoPosControl::publishAdaptive(uint64_t timestamp_sample)
{
	geo_control_position_adaptive_s adaptive{};
	adaptive.timestamp = hrt_absolute_time();
	adaptive.timestamp_sample = timestamp_sample;
	for (int i = 0; i < 3; ++i) {
		adaptive.position_bias_velocity[i] = _position_adaptive_velocity(i);
		adaptive.velocity_disturbance_acceleration[i] = _velocity_disturbance_acceleration(i);
		adaptive.acceleration_observation[i] = _acceleration_observation(i);
		adaptive.velocity_integral[i] = _velocity_integral(i);
	}
	adaptive.hover_thrust_estimate = _hover_thrust_estimate;
	adaptive.hover_thrust_observation = _hover_thrust_observation;
	adaptive.hover_thrust_learning = _hover_thrust_learning;
	adaptive.vertical_capture_active = _vertical_capture_active;
	adaptive.vertical_capture_velocity = _vertical_capture_velocity;
	adaptive.valid = _adaptive_valid;
	adaptive.mode = (uint8_t)math::constrain(_p_adaptive_mode.get(), int32_t{0}, int32_t{2});
	_adaptive_pub.publish(adaptive);
}

void GeoPosControl::publishSetpoints(const Vector3f &force_body, const Vector3f &vel_sp,
				     const Vector3f &accel_sp, const Vector3f &rates_sp,
				     uint64_t timestamp_sample, bool publish_attitude_rates)
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = now;
	thrust.timestamp_sample = timestamp_sample;

	// vehicle_thrust_setpoint 是 PX4 系统级的归一化推力接口，着陆检测会直接
	// 用 -xyz[2] 与 MPC_THR_HOVER/MPC_THR_MIN 比较。几何控制器内部仍以 N
	// 计算，发布前除以总最大推力；MorphQuad 分配器入口再恢复为物理力。
	const float thrust_max = math::max(_p_thrust_max.get(), 1.f);
	const float min_collective = math::constrain(_p_mpc_thr_min.get(), 0.f, 1.f);
	const float max_collective = math::constrain(_p_mpc_thr_max.get(), min_collective, 1.f);
	for (int i = 0; i < 3; ++i) {
		thrust.xyz[i] = math::constrain(force_body(i) / thrust_max, -max_collective, max_collective);
	}

	// 原生多旋翼位置控制在已解锁时总会保持 MPC_THR_MIN 以上的向上
	// 集体推力。采用同一规则，才能使起飞和着陆检测使用同一个尺度。
	if (_armed) {
		thrust.xyz[2] = -math::constrain(-thrust.xyz[2], min_collective, max_collective);
	}
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
	publishAdaptive(timestamp_sample);
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
	const Vector3f acceleration_measured(local.ax, local.ay, local.az);
	const float output_min_collective = math::constrain(_p_mpc_thr_min.get(), 0.f, 1.f);
	const float output_max_collective = math::constrain(_p_mpc_thr_max.get(), output_min_collective, 1.f);
	const float configured_hover = math::constrain(_p_hover_thrust.get(), math::max(output_min_collective, 0.01f),
						       output_max_collective);
	// A live MPC_THR_HOVER change starts a fresh estimate from the requested
	// baseline.  Disarming also discards the previous flight's learned state.
	if (!PX4_ISFINITE(_hover_thrust_parameter_last)
	    || fabsf(configured_hover - _hover_thrust_parameter_last) > 1e-4f || !_armed) {
		_hover_thrust_estimate = configured_hover;
		_hover_thrust_observation = configured_hover;
		_hover_thrust_parameter_last = configured_hover;
		_previous_collective_valid = false;
		_hover_thrust_learning = false;
	}
	const bool mode_changed = status.nav_state != _last_nav_state;

	if (!_reference_initialized || !_armed || mode_changed) {
		resetState(q, pos, vel);
	}

	// 飞机仍在地面时不能保留速度积分和旧的位置参考，否则在已解锁等待、
	// 降落检测抖动或再次起飞时会把地面阶段累积的状态突然带入控制。
	// 每周期跟随当前位置，也能保证离地瞬间从当前状态平滑接管。
	if (_landed) {
		_position_reference = pos;
		_velocity_limited = vel;
		_velocity_integral.zero();
	}

	_last_nav_state = status.nav_state;
	manual_control_setpoint_s manual{};
	const bool hold_mode = status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const bool manual_valid = !hold_mode && _manual_sub.copy(&manual) && manual.valid
				  && hrt_elapsed_time(&manual.timestamp) < 500_ms;
	const float dz = math::constrain(_p_manual_deadzone.get(), 0.f, 0.5f);
	Vector3f rates_sp{};
	Vector3f velocity_sp{};
	Vector3f acceleration_sp{};
	Vector3f force_body{};
	bool publish_attitude_rates = status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	const bool level_hold_requested = manual_valid && manual.aux3 > 0.5f;

	// AUX3 触发水平保持：只在开关沿触发时重置滚转/俯仰，航向保持当前值。
	// 这样不会因为每个控制周期重复构造四元数而产生跳变。
	if (level_hold_requested != _level_hold_active) {
		if (level_hold_requested) {
			_attitude_reference = Quatf(Eulerf(0.f, 0.f, Eulerf(q).psi()));
			_attitude_reference.normalize();
		}
		_level_hold_active = level_hold_requested;
	}

	// AUTO_LOITER 是无摇杆的定点保持模式。进入时 resetState() 已将位置参考
	// 置为当前位置，因此这里必须屏蔽全部摇杆输入，不能沿用 POSCTL 逻辑。
	if (hold_mode && !_hold_active) {
		_attitude_reference = Quatf(Eulerf(0.f, 0.f, Eulerf(q).psi()));
		_attitude_reference.normalize();
		_level_hold_active = false;
	}
	_hold_active = hold_mode;

	if (!_armed) {
		publishSetpoints(force_body, velocity_sp, acceleration_sp, rates_sp, local.timestamp_sample, true);
		return;
	}

	const bool rate_only = status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO;
	const bool position_mode = status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
				   || status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const bool altitude_mode = status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	// 保持重构前的遥控器映射：
	//   Position：roll/pitch 控制姿态，AUX1/AUX2 控制水平速度；
	//   Manual/Altitude/Acro：AUX1/AUX2 控制姿态角速度，roll/pitch 控制水平力。
	// yaw 和 throttle 在所有模式中仍使用原来的通道。
	if (manual_valid) {
		if (position_mode) {
			rates_sp = Vector3f(math::deadzone(manual.roll, dz),
					    -math::deadzone(manual.pitch, dz),
					    _landed ? 0.f : math::deadzone(manual.yaw, dz)) * _p_manual_rate_max.get();

		} else {
			rates_sp = Vector3f(math::deadzone(manual.aux1, dz),
					    math::deadzone(manual.aux2, dz),
					    _landed ? 0.f : math::deadzone(manual.yaw, dz)) * _p_manual_rate_max.get();
		}

		if (level_hold_requested) {
			// 水平保持时忽略姿态摇杆，油门和水平速度/力仍按模式处理。
			rates_sp(0) = 0.f;
			rates_sp(1) = 0.f;
		}
	}

	if (!rate_only) {
		// 手动姿态参考采用独立的 Euler 轴积分。用一个合成旋转轴更新
		// 四元数时，已有的 pitch 会把 pitch 角速度投影到 roll，导致两个
		// 摇杆相互影响；独立积分可以保证每个轴严格受自己的角度限制。
		Eulerf euler_ref(_attitude_reference);
		const float roll_limit = math::radians(math::constrain(_p_roll_max.get(), 1.f, 85.f));
		const float pitch_limit = math::radians(math::constrain(_p_pitch_max.get(), 1.f, 85.f));
		float roll_rate = rates_sp(0);
		float pitch_rate = rates_sp(1);

		// 到达边界后只阻止继续向外运动，允许反向摇杆立即脱离边界。
		if ((euler_ref.phi() >= roll_limit && roll_rate > 0.f)
		    || (euler_ref.phi() <= -roll_limit && roll_rate < 0.f)) {
			roll_rate = 0.f;
		}
		if ((euler_ref.theta() >= pitch_limit && pitch_rate > 0.f)
		    || (euler_ref.theta() <= -pitch_limit && pitch_rate < 0.f)) {
			pitch_rate = 0.f;
		}

		const float roll = math::constrain(euler_ref.phi() + roll_rate * dt,
						   -roll_limit, roll_limit);
		const float pitch = math::constrain(euler_ref.theta() + pitch_rate * dt,
						    -pitch_limit, pitch_limit);
		_attitude_reference = Quatf(Eulerf(roll, pitch, euler_ref.psi() + rates_sp(2) * dt));
		_attitude_reference.normalize();
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
		updateVerticalCapture(mode_changed, true, pos, vel, dt, velocity_sp(2));
		const Vector3f force_world = positionForce(pos, vel, velocity_sp, acceleration_measured, R, dt, acceleration_sp);
		force_body = R.transpose() * force_world;

	} else if (altitude_mode) {
		// 定高模式只闭合垂向速度，水平位置冻结在当前点；roll/pitch 摇杆
		// 直接产生手动水平力，故该模式不是水平位置闭环。
		velocity_sp(2) = manual_valid ? -math::deadzone(manual.throttle, dz) * _p_vz_max.get() : 0.f;
		updateVerticalCapture(mode_changed, true, pos, vel, dt, velocity_sp(2));
		_position_reference(0) = pos(0);
		_position_reference(1) = pos(1);
		const Vector3f force_world = positionForce(pos, vel, velocity_sp, acceleration_measured, R, dt, acceleration_sp);
		force_body = R.transpose() * Vector3f(0.f, 0.f, force_world(2));
		if (manual_valid && !level_hold_requested) {
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
		force_body = R.transpose() * positionForce(pos, vel, velocity_sp, acceleration_measured, R, dt, acceleration_sp);

	} else {
		const float throttle = manual_valid ? math::constrain((manual.throttle + 1.f) * 0.5f, 0.f, 1.f) : 0.f;
		const float min_collective = math::constrain(_p_mpc_thr_min.get(), 0.f, 1.f);
		const float max_collective = math::constrain(_p_mpc_thr_max.get(), min_collective, 1.f);
		// 保留原来的平方油门曲线，但其最低/最高值与 PX4 多旋翼共用。
		const float collective = min_collective + (max_collective - min_collective) * throttle * throttle;
		const float fz = -_p_thrust_max.get() * collective;
		force_body = R.transpose() * Vector3f(0.f, 0.f, fz);

		if (manual_valid && !rate_only && !level_hold_requested) {
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
