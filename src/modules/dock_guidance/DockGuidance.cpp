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
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/log.h>

#include <cmath>
#include <cstring>

using namespace matrix;

DockGuidance::DockGuidance()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        px4::wq_configurations::nav_and_controllers) {}

DockGuidance::~DockGuidance() {}

bool DockGuidance::init() {
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

void DockGuidance::Run() {
  if (should_exit()) {
    if (!_unregistration_sent) {
      if (_dock_mode_active) {
        publish_safe_velocity_setpoint(hrt_absolute_time());
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
    if ((_last_registration_request_us == 0) ||
        (now_us - _last_registration_request_us > 1_s)) {
      register_dock_mode(now_us);
    }

    check_mode_registration();

  } else if ((_last_mode_config_us == 0) ||
             (now_us - _last_mode_config_us > 1_s)) {
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
    const uint64_t timeout_us =
        static_cast<uint64_t>(math::constrain(_param_dock_obs_tout.get(),
                                              int32_t{20}, int32_t{5000})) *
        1000ULL;

    if (_setpoint_valid && (now_us > _last_observation_us) &&
        ((now_us - _last_observation_us) > timeout_us)) {
      handle_visual_loss(now_us, "vision timeout");

    } else if (_dock_mode_active && !_setpoint_valid) {
      // Commander may need more than one work-queue cycle to leave the external
      // mode. Keep refreshing a zero-velocity request until control ownership
      // changes.
      publish_safe_velocity_setpoint(now_us);

    } else if (_dock_mode_active && _setpoint_valid) {
      // Keep mc_pos_control supplied between camera frames. The visual watchdog
      // above prevents a stale direction from being held after the configured
      // timeout.
      publish_velocity_setpoint(now_us, _velocity_setpoint_ned,
                                _yaw_setpoint_rad);
    }

    return;
  }

  // timestamp_sample 是完整视觉帧接收时刻。旧驱动未填写时退化为当前时刻；
  // 明显位于未来的时间戳视为非法，避免无符号减法和延迟补偿越界。
  const uint64_t sample_us =
      (obs.timestamp_sample != 0) ? obs.timestamp_sample : now_us;

  if (sample_us > now_us + 5000ULL) {
    invalidate_setpoint(now_us, "future timestamp");
    return;
  }

  const uint64_t timeout_us =
      static_cast<uint64_t>(math::constrain(_param_dock_obs_tout.get(),
                                            int32_t{20}, int32_t{5000})) *
      1000ULL;

  if ((now_us > sample_us) && ((now_us - sample_us) > timeout_us)) {
    handle_visual_loss(now_us, "stale observation");
    return;
  }

  // 丢弃重复或乱序帧；保留上一条有效输出，随后由看门狗统一执行超时退出。
  if ((_last_sample_us != 0) && (sample_us <= _last_sample_us)) {
    return;
  }

  const float run_dt_s =
      (_last_sample_us == 0)
          ? LOOP_DT
          : math::constrain((sample_us - _last_sample_us) * 1e-6f, 0.002f,
                            0.1f);
  _last_sample_us = sample_us;
  _last_observation_us = sample_us;

  // 无目标或输入非法时，清空内部状态并明确发布 invalid，
  // 使下游控制器可以立即放弃视觉引导。
  if (!obs.valid || !std::isfinite(obs.offset_x) ||
      !std::isfinite(obs.offset_y)) {
    handle_visual_loss(now_us, "invalid observation");
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
  // 若视觉板已经输出角度则直接使用；否则根据图像分辨率和 FOV
  // 将像素偏移换算为角度。
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
      angle =
          math::constrain(angle, math::radians(10.0f), math::radians(80.0f));
      x_in = math::constrain(x_in, -pixel, pixel);
      return math::degrees(atanf(2.0f * x_in / pixel * tanf(angle * 0.5f)));
    };

    const float theta1 = cal_angle(res_x, fov_x_rad, obs.offset_x);
    const float theta2 = cal_angle(res_y, fov_y_rad, obs.offset_y);

    // 在相机坐标系中构造指向目标的单位方向之前的向量。
    const Vector3f los_cam(1.0f, tanf(math::radians(theta1)),
                           -tanf(math::radians(theta2)));
    los_yaw_deg = math::degrees(atanf(los_cam(1) / los_cam(0)));
    los_pitch_deg = -math::degrees(
        atanf(los_cam(2) / Vector2f(los_cam(0), los_cam(1)).norm()));
  }

  if (!std::isfinite(los_yaw_deg) || !std::isfinite(los_pitch_deg)) {
    handle_visual_loss(now_us, "invalid LOS");
    return;
  }

  const float los_limit =
      math::constrain(_param_dock_los_max.get(), 10.0f, 89.9f);

  if ((fabsf(los_yaw_deg) > los_limit) || (fabsf(los_pitch_deg) > los_limit)) {
    handle_visual_loss(now_us, "LOS outside configured range");
    return;
  }

  _last_valid_observation_us = sample_us;
  _vision_degraded = false;
  const float image_width = math::max(_param_dock_res_x.get(), 1.0f);
  const float image_height = math::max(_param_dock_res_y.get(), 1.0f);
  _target_size_ratio = math::max(
      static_cast<float>(obs.target_width_px) / image_width,
      static_cast<float>(obs.target_height_px) / image_height);
  const Vector3f relative_position_ned(obs.relative_n_m, obs.relative_e_m,
                                       obs.relative_d_m);
  const uint64_t relative_timeout_us =
      static_cast<uint64_t>(math::constrain(_param_dock_rel_tout.get(),
                                            int32_t{20}, int32_t{2000})) *
      1000ULL;
  const bool relative_position_fresh =
      (now_us >= sample_us) && ((now_us - sample_us) <= relative_timeout_us);
  const bool relative_position_valid =
      obs.relative_position_valid && relative_position_ned.isAllFinite() &&
      (relative_position_ned.norm() > 0.05f) && relative_position_fresh;
  process_observation(los_yaw_deg, los_pitch_deg, relative_position_ned,
                      relative_position_valid, att_deg, gyro_dps, run_dt_s,
                      sample_us);
}

// ---------- 姿态/角速度延时缓冲 ----------

void DockGuidance::push_delay_history(hrt_abstime timestamp) {
  // 将 PX4 四元数姿态转换为欧拉角，统一以度保存，便于后续与视觉角度相加。
  vehicle_attitude_s att{};
  vehicle_angular_velocity_s av{};

  if (!_att_sub.copy(&att) || !_gyro_sub.copy(&av)) {
    return;
  }

  const Quatf q(att.q);

  if (!q.isAllFinite() || (q.norm() < 0.5f) ||
      !Vector3f(av.xyz).isAllFinite()) {
    return;
  }

  Vector3f gyro_dps(math::degrees(av.xyz[0]), math::degrees(av.xyz[1]),
                    math::degrees(av.xyz[2]));

  _att_delay_buf[_delay_idx] = q.normalized();
  _gyro_delay_buf[_delay_idx] = gyro_dps;
  _delay_time_buf[_delay_idx] = timestamp;
  _delay_idx = (_delay_idx + 1) % DELAY_BUF_SIZE;
  _delay_count = math::min(_delay_count + 1, DELAY_BUF_SIZE);
}

bool DockGuidance::get_delayed_state(hrt_abstime timestamp_sample,
                                     Vector3f &att_deg,
                                     Vector3f &gyro_dps) const {
  if (_delay_count <= 0) {
    return false;
  }

  const uint64_t delay_us =
      static_cast<uint64_t>(math::constrain(_param_dock_cam_dly.get(),
                                            int32_t{0}, int32_t{5000})) *
      1000ULL;
  const hrt_abstime target_us =
      (timestamp_sample > delay_us) ? timestamp_sample - delay_us : 0;
  const int oldest =
      (_delay_idx - _delay_count + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;
  const int newest = (_delay_idx - 1 + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;

  if (target_us <= _delay_time_buf[oldest]) {
    // 历史尚未覆盖相机采样时刻时不使用错误姿态；等缓冲区积累足够数据后再输出有效指令。
    if (target_us == _delay_time_buf[oldest]) {
      const Eulerf euler(_att_delay_buf[oldest]);
      att_deg =
          Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()),
                   math::degrees(euler.psi()));
      gyro_dps = _gyro_delay_buf[oldest];
      return true;
    }

    return false;
  }

  if (target_us >= _delay_time_buf[newest]) {
    const Eulerf euler(_att_delay_buf[newest]);
    att_deg = Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()),
                       math::degrees(euler.psi()));
    gyro_dps = _gyro_delay_buf[newest];
    return true;
  }

  for (int n = 1; n < _delay_count; ++n) {
    const int previous = (oldest + n - 1) % DELAY_BUF_SIZE;
    const int next = (oldest + n) % DELAY_BUF_SIZE;

    if (target_us <= _delay_time_buf[next]) {
      const uint64_t span_us =
          _delay_time_buf[next] - _delay_time_buf[previous];
      const float alpha =
          (span_us > 0)
              ? static_cast<float>(target_us - _delay_time_buf[previous]) /
                    static_cast<float>(span_us)
              : 0.0f;

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
      att_deg =
          Vector3f(math::degrees(euler.phi()), math::degrees(euler.theta()),
                   math::degrees(euler.psi()));

      for (int axis = 0; axis < 3; ++axis) {
        gyro_dps(axis) = _gyro_delay_buf[previous](axis) +
                         alpha * (_gyro_delay_buf[next](axis) -
                                  _gyro_delay_buf[previous](axis));
      }

      return att_deg.isAllFinite() && gyro_dps.isAllFinite();
    }
  }

  return false;
}

void DockGuidance::reset_tracking_state() {
  _los_c_deg.zero();
  _los_e_deg.zero();
  _los_e_unit.zero();
  _los_e_dot.zero();
  _los_e_rate_dps.zero();
  _camera_e_deg.zero();
  _gyro_c_dps.zero();
  _los_c_rate_dps.zero();
  _relative_position_ned.zero();
  _relative_position_valid = false;
  _relative_distance_m = NAN;
  _velocity_setpoint_ned.zero();
  _tracking_velocity_ned.zero();
  _yaw_setpoint_rad = NAN;
  _commanded_speed_ms = 0.0f;
  _fov_ratio = 0.0f;
  _fov_yaw_ratio = 0.0f;
  _fov_pitch_ratio = 0.0f;
  _target_size_ratio = 0.0f;
  _stage = Stage::NONE;
  _aligned_since_us = 0;
  _hard_fov_since_us = 0;
  _recovery_stable_since_us = 0;
  _terminal_candidate_since_us = 0;
  _terminal_started_us = 0;
  _terminal_blind_since_us = 0;
  _last_control_update_us = 0;
  _terminal_velocity_ned.zero();
  _last_valid_observation_us = 0;
  _vision_degraded = false;
  _filt_initialized = false;
  _deriv_count = 0;
  _deriv_idx = 0;
}

void DockGuidance::invalidate_setpoint(hrt_abstime timestamp,
                                       const char *reason) {
  const bool was_active = _dock_mode_active;
  const float last_yaw_setpoint_rad = _yaw_setpoint_rad;

  if (_setpoint_valid) {
    PX4_WARN("dock: %s", reason);
  }

  if (was_active) {
    _mode_fault_latched = true;
  }

  reset_tracking_state();

  // Hold the last finite heading while publishing the final zero-velocity
  // setpoint. Releasing yaw at the same instant as a mode failure creates an
  // unnecessary heading step before Commander completes the fallback.
  _yaw_setpoint_rad = last_yaw_setpoint_rad;
  _setpoint_valid = false;

  dock_guidance_setpoint_s sp{};
  sp.timestamp = timestamp;
  sp.relative_distance_m = NAN;
  sp.camera_mount_pitch_deg = _camera_mount_pitch_deg;
  sp.camera_servo_enabled = _param_dock_cam_pitch_variable.get() != 0;
  sp.yaw_setpoint_deg = std::isfinite(_yaw_setpoint_rad)
                            ? math::degrees(_yaw_setpoint_rad)
                            : NAN;
  sp.fault_latched = _mode_fault_latched;
  sp.valid = false;
  sp.stage = static_cast<uint8_t>(Stage::NONE);
  _setpoint_pub.publish(sp);

  if (_dock_mode_active) {
    publish_safe_velocity_setpoint(timestamp);
  }
}

void DockGuidance::handle_visual_loss(hrt_abstime timestamp,
                                      const char *reason) {
  // Grace is only for an already-running Dock mode. While inactive, one bad
  // observation must immediately block a new mode entry.
  if (!_dock_mode_active || !_setpoint_valid ||
      (_last_valid_observation_us == 0)) {
    invalidate_setpoint(timestamp, reason);
    return;
  }

  if (_stage == Stage::TERMINAL_DASH && _terminal_started_us != 0) {
    const uint64_t maximum_terminal_time_us = static_cast<uint64_t>(
        math::constrain(_param_dock_terminal_max_time.get(), 0.1f, 10.0f) *
        1e6f);

    if (timestamp > _terminal_started_us &&
        (timestamp - _terminal_started_us) > maximum_terminal_time_us) {
      invalidate_setpoint(timestamp, "terminal dash time limit");
      return;
    }

    if (_terminal_blind_since_us == 0) {
      _terminal_blind_since_us = timestamp;
      PX4_WARN("dock: %s, terminal blind hold", reason);
    }

    const uint64_t blind_limit_us = static_cast<uint64_t>(
        math::constrain(_param_dock_terminal_blind.get(), 0.05f, 3.0f) *
        1e6f);

    if (timestamp >= _terminal_blind_since_us &&
        (timestamp - _terminal_blind_since_us) <= blind_limit_us) {
      _vision_degraded = true;
      publish_terminal_dash_setpoint(timestamp);
      return;
    }

    invalidate_setpoint(timestamp, "terminal blind time limit");
    return;
  }

  const uint64_t observation_timeout_us =
      static_cast<uint64_t>(math::constrain(_param_dock_obs_tout.get(),
                                            int32_t{20}, int32_t{5000})) *
      1000ULL;
  const uint64_t configured_grace_us =
      static_cast<uint64_t>(math::constrain(_param_dock_vis_grace.get(),
                                            int32_t{20}, int32_t{2000})) *
      1000ULL;
  const uint64_t grace_us =
      math::max(observation_timeout_us, configured_grace_us);
  const uint64_t age_us = (timestamp > _last_valid_observation_us)
                              ? timestamp - _last_valid_observation_us
                              : 0;

  if (age_us <= grace_us) {
    if (!_vision_degraded) {
      PX4_WARN("dock: %s, grace period", reason);
    }

    _vision_degraded = true;

    if (_dock_mode_active) {
      publish_degraded_setpoint(timestamp);
    }

  } else {
    invalidate_setpoint(timestamp, reason);
  }
}

void DockGuidance::register_dock_mode(hrt_abstime now) {
  register_ext_component_request_s request{};
  request.timestamp = now;
  request.request_id = _mode_request_id;
  request.px4_ros2_api_version =
      register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
  request.register_arming_check = true;
  request.register_mode = true;
  strncpy(request.name, "Dock Guidance", sizeof(request.name) - 1);
  _register_ext_component_request_pub.publish(request);
  _last_registration_request_us = now;
}

void DockGuidance::check_mode_registration() {
  register_ext_component_reply_s reply{};
  int remaining = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

  while ((_mode_id < 0 || _arming_check_id < 0) && remaining-- > 0 &&
         _register_ext_component_reply_sub.update(&reply)) {
    if (reply.request_id != _mode_request_id) {
      continue;
    }

    if (reply.success) {
      _mode_id = reply.mode_id;
      _arming_check_id = reply.arming_check_id;
      PX4_INFO("Dock mode registered: mode=%d check=%d", _mode_id,
               _arming_check_id);
      configure_dock_mode(hrt_absolute_time());

    } else {
      PX4_WARN("Dock mode registration rejected");
    }
  }
}

void DockGuidance::configure_dock_mode(hrt_abstime now) {
  if (_mode_id < 0) {
    return;
  }

  vehicle_control_mode_s control_mode{};
  control_mode.timestamp = now;
  control_mode.source_id = static_cast<uint8_t>(_mode_id);
  control_mode.flag_control_velocity_enabled = true;
  control_mode.flag_control_altitude_enabled = true;
  control_mode.flag_control_climb_rate_enabled = true;
  control_mode.flag_control_attitude_enabled = true;
  control_mode.flag_control_rates_enabled = true;
  control_mode.flag_control_allocation_enabled = true;
  _config_control_setpoints_pub.publish(control_mode);
  _last_mode_config_us = now;
}

void DockGuidance::unregister_dock_mode() {
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

void DockGuidance::reply_to_arming_check(
    const arming_check_request_s &request) {
  if (_arming_check_id < 0) {
    return;
  }

  arming_check_reply_s reply{};
  reply.timestamp = hrt_absolute_time();
  reply.request_id = request.request_id;
  reply.registration_id = _arming_check_id;
  reply.health_component_index =
      arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
  reply.can_arm_and_run = _setpoint_valid && !_mode_fault_latched;
  reply.mode_req_angular_velocity = true;
  reply.mode_req_attitude = true;
  reply.mode_req_local_alt = true;
  reply.mode_req_local_position = true;
  reply.mode_req_manual_control = true;
  _arming_check_reply_pub.publish(reply);
}

void DockGuidance::update_mode_state() {
  vehicle_status_s status{};

  if (!_vehicle_status_sub.copy(&status)) {
    return;
  }

  const bool active =
      (_mode_id >= 0) && (status.nav_state == static_cast<uint8_t>(_mode_id));

  // A real visual failure is latched so a still-selected Dock switch cannot
  // bounce between Dock and RTL. The pilot clears the latch by deliberately
  // selecting another mode before requesting Dock again.
  if (_mode_fault_latched && (_mode_id >= 0) &&
      (status.nav_state_user_intention != static_cast<uint8_t>(_mode_id))) {
    _mode_fault_latched = false;
    PX4_INFO("dock fault latch cleared by mode deselection");
  }

  if (active && !_dock_mode_active) {
    PX4_INFO("Dock mode active");
    set_stage(Stage::ALIGN);
    _commanded_speed_ms = 0.0f;
    _aligned_since_us = 0;
    _last_control_update_us = 0;
    _velocity_setpoint_ned.zero();
    _terminal_candidate_since_us = 0;
    _terminal_started_us = 0;
    _terminal_blind_since_us = 0;
    _terminal_velocity_ned.zero();

    // mc_pos_control requires a trajectory setpoint no older than the instant
    // position control becomes active. Start with a zero-velocity request.
    publish_safe_velocity_setpoint(hrt_absolute_time());

  } else if (!active && _dock_mode_active) {
    PX4_INFO("Dock mode inactive");
    _commanded_speed_ms = 0.0f;
    _aligned_since_us = 0;
    _last_control_update_us = 0;
    _velocity_setpoint_ned.zero();
    _terminal_candidate_since_us = 0;
    _terminal_started_us = 0;
    _terminal_blind_since_us = 0;
    _terminal_velocity_ned.zero();
    set_stage(Stage::ALIGN);
  }

  _dock_mode_active = active;
}

void DockGuidance::publish_velocity_setpoint(hrt_abstime timestamp,
                                             const Vector3f &velocity_ned,
                                             float yaw_setpoint_rad) {
  trajectory_setpoint_s trajectory{};
  trajectory.timestamp = timestamp;

  for (int axis = 0; axis < 3; ++axis) {
    trajectory.position[axis] = NAN;
    trajectory.velocity[axis] = velocity_ned(axis);
    trajectory.acceleration[axis] = NAN;
    trajectory.jerk[axis] = NAN;
  }

  trajectory.yaw = yaw_setpoint_rad;
  trajectory.yawspeed = NAN;
  _trajectory_setpoint_pub.publish(trajectory);
}

void DockGuidance::publish_safe_velocity_setpoint(hrt_abstime timestamp) {
  _velocity_setpoint_ned.zero();
  publish_velocity_setpoint(timestamp, _velocity_setpoint_ned,
                            _yaw_setpoint_rad);
}

void DockGuidance::publish_degraded_setpoint(hrt_abstime timestamp) {
  const float dt_s =
      (_last_control_update_us > 0 && timestamp > _last_control_update_us)
          ? math::constrain((timestamp - _last_control_update_us) * 1e-6f,
                            0.002f, 0.1f)
          : LOOP_DT;
  const float deceleration =
      math::constrain(_param_dock_dec_max.get(), 0.1f, 20.0f);
  const float speed = _velocity_setpoint_ned.norm();

  if (speed > 0.01f) {
    const float next_speed = math::max(speed - deceleration * dt_s, 0.0f);
    _velocity_setpoint_ned *= next_speed / speed;

  } else {
    _velocity_setpoint_ned.zero();
  }

  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  _last_control_update_us = timestamp;
  set_stage(Stage::RECOVERY);
  publish_guidance_status(timestamp, true, true);
  publish_velocity_setpoint(timestamp, _velocity_setpoint_ned,
                            _yaw_setpoint_rad);
}

void DockGuidance::publish_terminal_dash_setpoint(hrt_abstime timestamp) {
  vehicle_local_position_s lpos{};
  const bool have_local_position = _local_pos_sub.copy(&lpos);
  const bool velocity_fresh =
      have_local_position && (lpos.timestamp > 0) &&
      (timestamp >= lpos.timestamp) && ((timestamp - lpos.timestamp) < 500_ms);
  const Vector3f velocity_ned(lpos.vx, lpos.vy, lpos.vz);

  if (!velocity_fresh || !lpos.v_xy_valid || !lpos.v_z_valid ||
      !velocity_ned.isAllFinite() ||
      !_terminal_velocity_ned.isAllFinite() ||
      (_terminal_velocity_ned.norm() < 0.1f)) {
    invalidate_setpoint(timestamp, "terminal local velocity invalid");
    return;
  }

  _velocity_ned = velocity_ned;
  _velocity_valid = true;
  _velocity_speed_ms = velocity_ned.norm();
  _velocity_setpoint_ned = _terminal_velocity_ned;
  _commanded_speed_ms = _terminal_velocity_ned.norm();
  const Vector3f terminal_direction =
      _terminal_velocity_ned / _commanded_speed_ms;
  _closing_speed_ms = _velocity_ned * terminal_direction;
  _last_control_update_us = timestamp;
  publish_guidance_status(timestamp, true, true);
  publish_velocity_setpoint(timestamp, _terminal_velocity_ned,
                            _yaw_setpoint_rad);
}

void DockGuidance::publish_guidance_status(hrt_abstime timestamp, bool valid,
                                           bool degraded) {
  dock_guidance_setpoint_s status{};
  status.timestamp = timestamp;
  status.closing_speed_m_s = _closing_speed_ms;
  status.target_speed_m_s = _commanded_speed_ms;
  _velocity_setpoint_ned.copyTo(status.velocity_setpoint_ned_m_s);
  _tracking_velocity_ned.copyTo(status.tracking_velocity_ned_m_s);
  _relative_position_ned.copyTo(status.relative_position_ned_m);
  status.relative_distance_m = _relative_distance_m;
  status.fov_ratio = _fov_ratio;
  status.fov_yaw_ratio = _fov_yaw_ratio;
  status.fov_pitch_ratio = _fov_pitch_ratio;
  status.target_size_ratio = _target_size_ratio;
  status.terminal_time_s =
      (_terminal_started_us != 0 && timestamp >= _terminal_started_us)
          ? (timestamp - _terminal_started_us) * 1e-6f
          : 0.0f;
  status.terminal_blind_time_s =
      (_terminal_blind_since_us != 0 && timestamp >= _terminal_blind_since_us)
          ? (timestamp - _terminal_blind_since_us) * 1e-6f
          : 0.0f;
  status.camera_yaw_deg = _los_c_deg(0);
  status.camera_pitch_deg = _los_c_deg(1);
  status.camera_mount_pitch_deg = _camera_mount_pitch_deg;
  status.yaw_setpoint_deg = std::isfinite(_yaw_setpoint_rad)
                                ? math::degrees(_yaw_setpoint_rad)
                                : NAN;
  status.stage = static_cast<uint8_t>(_stage);
  status.velocity_valid = _velocity_valid;
  status.relative_position_valid = _relative_position_valid;
  status.camera_servo_enabled = _param_dock_cam_pitch_variable.get() != 0;
  status.vision_degraded = degraded;
  status.fault_latched = _mode_fault_latched;
  status.valid = valid;
  _setpoint_pub.publish(status);
}

void DockGuidance::update_camera_pitch_servo(float camera_pitch_error_deg,
                                             float dt_s,
                                             hrt_abstime timestamp_sample) {
  if (_param_dock_cam_pitch_variable.get() == 0) {
    return;
  }

  const float minimum_pitch = math::min(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float maximum_pitch = math::max(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float deadband =
      math::constrain(_param_dock_cam_pitch_deadband.get(), 0.0f, 10.0f);
  const float time_constant =
      math::constrain(_param_dock_cam_pitch_tc.get(), 0.2f, 20.0f);
  const float maximum_rate =
      math::constrain(_param_dock_cam_pitch_rate.get(), 0.1f, 30.0f);
  float effective_error = 0.0f;

  if (camera_pitch_error_deg > deadband) {
    effective_error = camera_pitch_error_deg - deadband;

  } else if (camera_pitch_error_deg < -deadband) {
    effective_error = camera_pitch_error_deg + deadband;
  }

  // This is intentionally a slow body-relative pointing adjustment. It uses
  // only sustained image error; vehicle attitude and angular rate are not fed
  // back to the servo, so the position servo is never used as a stabilizer.
  const float pitch_rate_deg_s = math::constrain(
      effective_error / time_constant, -maximum_rate, maximum_rate);
  _camera_mount_pitch_deg =
      math::constrain(_camera_mount_pitch_deg + pitch_rate_deg_s * dt_s,
                      minimum_pitch, maximum_pitch);
  update_camera_rotation();
  publish_camera_pitch_servo(timestamp_sample);
}

void DockGuidance::publish_camera_pitch_servo(hrt_abstime timestamp_sample) {
  const float minimum_pitch = math::min(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float maximum_pitch = math::max(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float pitch_range = maximum_pitch - minimum_pitch;

  if (!std::isfinite(_camera_mount_pitch_deg) || pitch_range < 1.0f) {
    return;
  }

  gimbal_controls_s controls{};
  controls.timestamp = hrt_absolute_time();
  controls.timestamp_sample = timestamp_sample;
  controls.control[gimbal_controls_s::INDEX_ROLL] = NAN;
  controls.control[gimbal_controls_s::INDEX_PITCH] = math::constrain(
      2.0f * (_camera_mount_pitch_deg - minimum_pitch) / pitch_range - 1.0f,
      -1.0f, 1.0f);
  controls.control[gimbal_controls_s::INDEX_YAW] = NAN;
  _gimbal_controls_pub.publish(controls);
}

void DockGuidance::set_stage(Stage stage) {
  if (_stage == stage) {
    return;
  }

  _stage = stage;

  switch (stage) {
  case Stage::NONE:
    PX4_INFO("dock stage NONE");
    break;

  case Stage::ALIGN:
    PX4_INFO("dock stage ALIGN");
    break;

  case Stage::ACCELERATE:
    PX4_INFO("dock stage ACCELERATE");
    break;

  case Stage::MAX_SPEED_APPROACH:
    PX4_INFO("dock stage MAX_SPEED");
    break;

  case Stage::TERMINAL_BRAKE:
    PX4_INFO("dock stage BRAKE");
    break;

  case Stage::RECOVERY:
    PX4_INFO("dock stage RECOVERY");
    break;

  case Stage::EMERGENCY_RECOVERY:
    PX4_WARN("dock stage EMERGENCY_RECOVERY");
    break;

  case Stage::TERMINAL_DASH:
    PX4_INFO("dock stage TERMINAL_DASH");
    break;
  }
}

Vector3f DockGuidance::compute_image_tracking_velocity(
    const Vector3f &approach_direction,
    const Vector3f &camera_forward_ned) const {
  // Moving exactly along LOS preserves the earth-frame LOS, but mc_pos_control
  // must tilt the fixed camera while accelerating. Translate perpendicular to
  // LOS so the target bearing follows that camera motion instead of walking out
  // of the image. Body yaw already aligns the horizontal camera bearing, so
  // remove the error component that yaw can correct and leave translation to
  // handle the remaining (normally vertical) image error.
  Vector3f pointing_error =
      camera_forward_ned -
      approach_direction * (approach_direction * camera_forward_ned);
  const Vector3f ned_down(0.0f, 0.0f, 1.0f);
  Vector3f yaw_correctable = ned_down % camera_forward_ned;
  yaw_correctable -=
      approach_direction * (approach_direction * yaw_correctable);
  const float yaw_correctable_norm = yaw_correctable.norm();

  if (yaw_correctable_norm > 0.01f) {
    yaw_correctable /= yaw_correctable_norm;
    pointing_error -= yaw_correctable * (yaw_correctable * pointing_error);
  }

  const float gain = math::constrain(_param_dock_img_p.get(), 0.05f, 5.0f);
  Vector3f tracking_velocity{};

  if (_relative_position_valid && std::isfinite(_relative_distance_m)) {
    tracking_velocity =
        -math::max(_relative_distance_m, 0.0f) * gain * pointing_error;

  } else {
    // Legacy 21-byte Viztra frames contain a valid image angle but no target
    // range. Keep image tracking available so Dock mode can be entered and the
    // target can be centered, but use a bounded angular-only correction.
    const float pointing_error_norm = pointing_error.norm();

    if (pointing_error_norm > 0.001f) {
      const float remaining_angle =
          asinf(math::constrain(pointing_error_norm, 0.0f, 1.0f));
      const float half_vertical_fov =
          math::radians(math::max(0.5f * _param_dock_fov_y.get(), 5.0f));
      const float align_ratio =
          math::constrain(_param_dock_fov_align.get(), 0.05f, 0.8f);
      const float correction_scale = math::constrain(
          remaining_angle / (half_vertical_fov * align_ratio), 0.0f, 1.0f);
      const bool approach_active = _dock_mode_active &&
                                   (_stage != Stage::NONE) &&
                                   (_stage != Stage::ALIGN);
      const float image_limit =
          math::constrain(_param_dock_img_vmax.get(), 0.1f, 20.0f);
      const float fallback_limit =
          approach_active
              ? image_limit
              : math::min(
                    math::constrain(_param_dock_align_speed.get(), 0.1f, 5.0f),
                    image_limit);
      tracking_velocity = -pointing_error * (fallback_limit * correction_scale /
                                             pointing_error_norm);
    }
  }

  tracking_velocity -=
      approach_direction * (approach_direction * tracking_velocity);
  const float tracking_speed = tracking_velocity.norm();
  const float tracking_limit =
      math::constrain(_param_dock_img_vmax.get(), 0.1f, 20.0f);

  if (tracking_speed > tracking_limit) {
    tracking_velocity *= tracking_limit / tracking_speed;
  }

  return tracking_velocity.isAllFinite() ? tracking_velocity : Vector3f{};
}

Vector3f
DockGuidance::compose_tracked_velocity(const Vector3f &approach_direction,
                                       const Vector3f &tracking_velocity,
                                       float speed_budget) const {
  speed_budget = math::max(speed_budget, 0.0f);

  if (speed_budget < 0.01f) {
    return Vector3f{};
  }

  // The correction is perpendicular to LOS. Reserve part of the existing
  // speed budget for image tracking, so MAX_SPEED still means the magnitude of
  // the final 3-D velocity vector and not LOS speed plus an unbounded addition.
  Vector3f correction =
      tracking_velocity -
      approach_direction * (approach_direction * tracking_velocity);
  const float correction_speed = correction.norm();
  const float correction_fraction =
      math::constrain(_param_dock_img_fraction.get(), 0.05f, 0.8f);
  const float correction_limit = correction_fraction * speed_budget;

  if (correction_speed > correction_limit) {
    correction *= correction_limit / correction_speed;
  }

  const float limited_correction_speed = correction.norm();
  const float closing_speed =
      sqrtf(math::max(speed_budget * speed_budget -
                          limited_correction_speed * limited_correction_speed,
                      0.0f));
  Vector3f result = approach_direction * closing_speed + correction;

  // A transverse correction around a steep LOS can otherwise reverse the
  // horizontal component even though the 3-D dot product remains positive.
  // Losing horizontal closing direction caused the large +/- pitch commands
  // seen in flight. Retain cross-track correction but never command motion
  // backwards along the horizontal target bearing.
  Vector2f horizontal_approach(approach_direction(0), approach_direction(1));
  const float horizontal_norm = horizontal_approach.norm();

  if (horizontal_norm > 0.05f) {
    horizontal_approach /= horizontal_norm;
    Vector2f horizontal_result(result(0), result(1));
    const float along_horizontal = horizontal_result * horizontal_approach;

    if (along_horizontal < 0.0f) {
      horizontal_result -= horizontal_approach * along_horizontal;
      result(0) = horizontal_result(0);
      result(1) = horizontal_result(1);
    }
  }

  return result;
}

float DockGuidance::maximum_approach_speed(
    const Vector3f &approach_direction,
    bool legacy_approach_enabled) const {
  float controller_limited_speed = 100.0f;
  const float horizontal_direction =
      Vector2f(approach_direction(0), approach_direction(1)).norm();

  if (horizontal_direction > 0.01f) {
    controller_limited_speed = math::min(
        controller_limited_speed,
        math::max(_param_mpc_xy_vel_max.get(), 0.0f) / horizontal_direction);
  }

  if (approach_direction(2) < -0.01f) {
    controller_limited_speed =
        math::min(controller_limited_speed,
                  math::max(_param_mpc_z_vel_max_up.get(), 0.0f) /
                      -approach_direction(2));

  } else if (approach_direction(2) > 0.01f) {
    controller_limited_speed =
        math::min(controller_limited_speed,
                  math::max(_param_mpc_z_vel_max_down.get(), 0.0f) /
                      approach_direction(2));
  }

  float maximum_speed =
      math::min(math::constrain(_param_dock_vel_max.get(), 0.0f, 100.0f),
                controller_limited_speed);

  if (!_relative_position_valid && legacy_approach_enabled) {
    maximum_speed =
        math::min(maximum_speed,
                  math::constrain(_param_dock_legacy_vel_max.get(), 0.1f,
                                  20.0f));
  }

  return maximum_speed;
}

void DockGuidance::constrain_velocity_setpoint(Vector3f &velocity_ned) const {
  // Apply all mc_pos_control component constraints with one common scale. This
  // preserves the image-tracking direction instead of clipping axes separately.
  float scale = 1.0f;
  const float horizontal_speed =
      Vector2f(velocity_ned(0), velocity_ned(1)).norm();
  const float horizontal_limit = math::max(_param_mpc_xy_vel_max.get(), 0.0f);

  if ((horizontal_speed > 0.01f) && (horizontal_speed > horizontal_limit)) {
    scale = math::min(scale, horizontal_limit / horizontal_speed);
  }

  if (velocity_ned(2) < -0.01f) {
    const float up_limit = math::max(_param_mpc_z_vel_max_up.get(), 0.0f);
    scale = math::min(scale, up_limit / -velocity_ned(2));

  } else if (velocity_ned(2) > 0.01f) {
    const float down_limit = math::max(_param_mpc_z_vel_max_down.get(), 0.0f);
    scale = math::min(scale, down_limit / velocity_ned(2));
  }

  velocity_ned *= math::constrain(scale, 0.0f, 1.0f);
}

void DockGuidance::limit_velocity_setpoint_slew(
    const Vector3f &previous_velocity_ned, float dt_s, float acceleration,
    float deceleration, bool emergency_recovery) {
  if (!previous_velocity_ned.isAllFinite() ||
      !_velocity_setpoint_ned.isAllFinite()) {
    _velocity_setpoint_ned.zero();
    return;
  }

  Vector3f velocity_delta = _velocity_setpoint_ned - previous_velocity_ned;
  const float delta_norm = velocity_delta.norm();

  if (delta_norm < 0.001f) {
    return;
  }

  const bool reducing_speed =
      (_velocity_setpoint_ned.norm() < previous_velocity_ned.norm()) ||
      ((previous_velocity_ned * _velocity_setpoint_ned) < 0.0f);
  float slew_rate = reducing_speed ? deceleration : acceleration;

  if (emergency_recovery) {
    slew_rate = math::max(2.0f * deceleration, acceleration);
  }

  const float maximum_delta = math::max(slew_rate, 0.1f) * dt_s;

  if (delta_norm > maximum_delta) {
    _velocity_setpoint_ned =
        previous_velocity_ned + velocity_delta * (maximum_delta / delta_norm);
  }
}

// ---------- 视觉观测处理 ----------

void DockGuidance::process_observation(float los_yaw_deg, float los_pitch_deg,
                                       const Vector3f &relative_position_ned,
                                       bool relative_position_valid,
                                       const Vector3f &att_deg,
                                       const Vector3f &gyro_dps, float dt_s,
                                       hrt_abstime timestamp_sample) {
  // los_c：相机坐标系中的目标方位角，分别为水平偏差（yaw）与垂直偏差（pitch）。
  _los_c_deg(0) = los_yaw_deg;
  _los_c_deg(1) = los_pitch_deg;

  const float yaw_rad = math::radians(los_yaw_deg);
  const float pitch_rad = math::radians(los_pitch_deg);
  const Vector3f los_c_unit(cosf(pitch_rad) * cosf(yaw_rad),
                            cosf(pitch_rad) * sinf(yaw_rad), -sinf(pitch_rad));

  // 将相机视线从相机系旋转到 NED 地理系；这样可在地理系估计 LOS 变化率。
  const Dcmf R_ned_body(Eulerf(math::radians(att_deg(0)),
                               math::radians(att_deg(1)),
                               math::radians(att_deg(2))));
  const Dcmf R_ned_cam = R_ned_body * _rot_cam_to_body;
  _los_e_unit = R_ned_cam * los_c_unit;
  _los_e_unit.normalize();

  _los_e_deg(0) =
      math::degrees(matrix::wrap_pi(atan2f(_los_e_unit(1), _los_e_unit(0))));
  _los_e_deg(1) = math::degrees(
      atan2f(-_los_e_unit(2), Vector2f(_los_e_unit(0), _los_e_unit(1)).norm()));

  // 对单位视线向量低通滤波，而不是直接滤波欧拉角，避免角度跨 ±180° 时出现跳变。
  // 目标进入末段后提高带宽，减小近距离高速 LOS 变化造成的方向滞后。
  float filt_hz = math::constrain(_param_dock_filt_hz.get(), 0.1f, 20.0f);
  const bool terminal_filter_active =
      (_param_dock_terminal_enable.get() != 0) &&
      ((_stage == Stage::TERMINAL_DASH) ||
       (_target_size_ratio >=
        math::constrain(_param_dock_terminal_size.get(), 0.01f, 1.0f)) ||
       (_relative_position_valid &&
        (_param_dock_terminal_distance.get() > 0.0f) &&
        (_relative_distance_m <= _param_dock_terminal_distance.get())));

  if (terminal_filter_active) {
    filt_hz = math::max(
        filt_hz,
        math::constrain(_param_dock_terminal_filter.get(), 1.0f, 30.0f));
  }

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
    _los_e_dot = Vector3f(derivative_filter(_los_x_deriv_buf),
                          derivative_filter(_los_y_deriv_buf),
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
  _relative_position_valid = relative_position_valid;

  if (_relative_position_valid) {
    _relative_position_ned = relative_position_ned;
    _relative_distance_m = relative_position_ned.norm();

  } else {
    _relative_position_ned.zero();
    _relative_distance_m = NAN;
  }

  // 使用 EKF 融合后的本机速度，不直接使用原始 GPS 速度。将速度投影到
  // 地理系 LOS 后得到朝目标运动的接近速度，用于状态监视和制动验证。
  vehicle_local_position_s lpos{};
  const bool have_local_position = _local_pos_sub.copy(&lpos);
  const hrt_abstime now = hrt_absolute_time();
  const bool velocity_fresh = have_local_position && (lpos.timestamp > 0) &&
                              (now >= lpos.timestamp) &&
                              ((now - lpos.timestamp) < 500_ms);
  _velocity_ned = Vector3f(lpos.vx, lpos.vy, lpos.vz);
  _velocity_valid = velocity_fresh && lpos.v_xy_valid && lpos.v_z_valid &&
                    _velocity_ned.isAllFinite();

  if (_velocity_valid) {
    _velocity_speed_ms = _velocity_ned.norm();
    _closing_speed_ms = _velocity_ned * _los_e_unit;
    _velocity_e_deg(0) =
        math::degrees(atan2f(_velocity_ned(1), _velocity_ned(0)));
    _velocity_e_deg(1) = math::degrees(
        atan2f(-_velocity_ned(2),
               Vector2f(_velocity_ned(0), _velocity_ned(1)).norm()));

  } else {
    _velocity_ned.zero();
    _velocity_speed_ms = 0.0f;
    _closing_speed_ms = 0.0f;
    _velocity_e_deg.zero();
  }

  if (!_velocity_valid) {
    invalidate_setpoint(now, "local velocity invalid");
    return;
  }

  compute_control(att_deg, dt_s, now);
  update_camera_pitch_servo(los_pitch_deg, dt_s, timestamp_sample);
}

// ---------- 视觉三维速度引导状态机 ----------

void DockGuidance::compute_control(const Vector3f &att_deg, float obs_dt_s,
                                   hrt_abstime now) {
  const Vector3f previous_velocity_setpoint_ned = _velocity_setpoint_ned;

  // The camera-frame errors define image safety, while the filtered NED LOS
  // defines the direction of the velocity vector sent to mc_pos_control.
  const Dcmf R_ned_body(Eulerf(math::radians(att_deg(0)),
                               math::radians(att_deg(1)),
                               math::radians(att_deg(2))));
  const Dcmf R_ned_cam = R_ned_body * _rot_cam_to_body;

  Eulerf cam_euler(R_ned_cam);
  _camera_e_deg(0) = math::degrees(cam_euler.phi());
  _camera_e_deg(1) = math::degrees(cam_euler.theta());
  _camera_e_deg(2) = math::degrees(cam_euler.psi());

  const float half_fov_x = math::max(0.5f * _param_dock_fov_x.get(), 5.0f);
  const float half_fov_y = math::max(0.5f * _param_dock_fov_y.get(), 5.0f);
  _fov_yaw_ratio = fabsf(_los_c_deg(0)) / half_fov_x;
  _fov_pitch_ratio = fabsf(_los_c_deg(1)) / half_fov_y;
  _fov_ratio = math::max(_fov_yaw_ratio, _fov_pitch_ratio);

  const float hard_ratio =
      math::constrain(_param_dock_fov_hard.get(), 0.30f, 0.99f);
  const float soft_ratio =
      math::constrain(_param_dock_fov_soft.get(), 0.15f, hard_ratio - 0.05f);
  const float align_ratio =
      math::constrain(_param_dock_fov_align.get(), 0.05f, soft_ratio - 0.05f);
  const float dt_s =
      (_last_control_update_us > 0 && now > _last_control_update_us)
          ? math::constrain((now - _last_control_update_us) * 1e-6f, 0.002f,
                            0.1f)
          : math::constrain(obs_dt_s, 0.002f, 0.1f);
  _last_control_update_us = now;

  if (_fov_ratio >= hard_ratio) {
    if (_hard_fov_since_us == 0) {
      _hard_fov_since_us = now;
    }

  } else {
    _hard_fov_since_us = 0;
  }

  const uint64_t hard_fov_time_us = static_cast<uint64_t>(
      math::constrain(_param_dock_fov_hard_time.get(), 0.0f, 1.0f) * 1e6f);
  const bool hard_fov_confirmed =
      (_hard_fov_since_us != 0) &&
      ((now - _hard_fov_since_us) >= hard_fov_time_us);

  if (_mode_fault_latched) {
    _setpoint_valid = false;
    _commanded_speed_ms = 0.0f;
    _velocity_setpoint_ned.zero();
    _tracking_velocity_ned.zero();
    publish_guidance_status(now, false, false);

    if (_dock_mode_active) {
      publish_safe_velocity_setpoint(now);
    }

    return;
  }

  Vector3f approach_direction = _los_e_unit;

  if (!approach_direction.isAllFinite() || approach_direction.norm() < 0.5f) {
    invalidate_setpoint(now, "invalid approach direction");
    return;
  }

  approach_direction.normalize();
  const Vector3f camera_forward_ned = R_ned_cam * Vector3f(1.0f, 0.0f, 0.0f);

  if (_param_dock_cam_pitch_variable.get() != 0) {
    // Pitch centering belongs to the slow mount servo in this mode. Do not let
    // the aircraft and servo fight over the same image-axis error. Horizontal
    // centering is still handled by the vehicle yaw setpoint below.
    _tracking_velocity_ned.zero();

  } else {
    _tracking_velocity_ned =
        compute_image_tracking_velocity(approach_direction, camera_forward_ned);
  }
  const float horizontal_los =
      Vector2f(approach_direction(0), approach_direction(1)).norm();

  const float current_body_yaw = math::radians(att_deg(2));
  float raw_yaw_setpoint = std::isfinite(_yaw_setpoint_rad)
                               ? _yaw_setpoint_rad
                               : current_body_yaw;
  const float yaw_deadband =
      math::constrain(_param_dock_yaw_deadband.get(), 0.0f, 10.0f);
  const bool recovery_heading_hold =
      (_stage == Stage::RECOVERY) ||
      (_stage == Stage::EMERGENCY_RECOVERY);

  if (!recovery_heading_hold &&
      (fabsf(_los_c_deg(0)) > yaw_deadband) && (horizontal_los > 0.25f)) {
    const float target_yaw =
        atan2f(approach_direction(1), approach_direction(0));
    const float camera_yaw =
        atan2f(camera_forward_ned(1), camera_forward_ned(0));
    const float yaw_authority =
        math::constrain((horizontal_los - 0.25f) / (0.60f - 0.25f), 0.0f,
                        1.0f);
    const float yaw_correction =
        matrix::wrap_pi(target_yaw - camera_yaw) * yaw_authority;
    raw_yaw_setpoint = matrix::wrap_pi(current_body_yaw + yaw_correction);
  }

  const float maximum_yaw_delta = math::radians(
      math::constrain(_param_dock_yaw_rate.get(), 1.0f, 90.0f) * dt_s);
  const float previous_yaw_setpoint = std::isfinite(_yaw_setpoint_rad)
                                          ? _yaw_setpoint_rad
                                          : current_body_yaw;
  const float yaw_delta = math::constrain(
      matrix::wrap_pi(raw_yaw_setpoint - previous_yaw_setpoint),
      -maximum_yaw_delta, maximum_yaw_delta);
  _yaw_setpoint_rad = matrix::wrap_pi(previous_yaw_setpoint + yaw_delta);

  // Before the external mode owns control, only validate and publish status.
  // Starting the acceleration timer while another mode is active would create
  // an immediate high-speed setpoint on mode entry.
  if (!_dock_mode_active) {
    set_stage(Stage::ALIGN);
    _commanded_speed_ms = 0.0f;
    _velocity_setpoint_ned.zero();
    _tracking_velocity_ned.zero();
    _yaw_setpoint_rad = current_body_yaw;
    _aligned_since_us = 0;
    _hard_fov_since_us = 0;
    _recovery_stable_since_us = 0;
    _setpoint_valid = true;
    publish_guidance_status(now, true, false);
    return;
  }

  if (_stage == Stage::NONE) {
    set_stage(Stage::ALIGN);
  }

  const bool terminal_enabled = _param_dock_terminal_enable.get() != 0;

  if (!terminal_enabled && _stage == Stage::TERMINAL_DASH) {
    _terminal_candidate_since_us = 0;
    _terminal_started_us = 0;
    _terminal_blind_since_us = 0;
    _terminal_velocity_ned.zero();
    set_stage(Stage::RECOVERY);
  }

  if (terminal_enabled && _stage == Stage::TERMINAL_DASH) {
    const uint64_t maximum_terminal_time_us = static_cast<uint64_t>(
        math::constrain(_param_dock_terminal_max_time.get(), 0.1f, 10.0f) *
        1e6f);

    if (_terminal_started_us == 0 || now < _terminal_started_us ||
        (now - _terminal_started_us) > maximum_terminal_time_us) {
      invalidate_setpoint(now, "terminal dash time limit");
      return;
    }

  } else if (terminal_enabled) {
    const float terminal_distance = math::constrain(
        _param_dock_terminal_distance.get(), 0.0f, 100.0f);
    const float terminal_size = math::constrain(
        _param_dock_terminal_size.get(), 0.01f, 1.0f);
    const bool distance_candidate =
        (terminal_distance > 0.0f) && _relative_position_valid &&
        std::isfinite(_relative_distance_m) &&
        (_relative_distance_m <= terminal_distance);
    const bool size_candidate = _target_size_ratio >= terminal_size;
    const bool approach_stage =
        (_stage == Stage::ACCELERATE) ||
        (_stage == Stage::MAX_SPEED_APPROACH) ||
        (_stage == Stage::TERMINAL_BRAKE);

    if (approach_stage && (distance_candidate || size_candidate)) {
      if (_terminal_candidate_since_us == 0) {
        _terminal_candidate_since_us = now;
      }

      const uint64_t confirmation_us = static_cast<uint64_t>(
          math::constrain(_param_dock_terminal_confirm.get(), 0.02f, 2.0f) *
          1e6f);

      if ((now - _terminal_candidate_since_us) >= confirmation_us) {
        _terminal_started_us = now;
        _terminal_candidate_since_us = 0;
        _terminal_blind_since_us = 0;
        _terminal_velocity_ned = previous_velocity_setpoint_ned;
        set_stage(Stage::TERMINAL_DASH);
      }

    } else {
      _terminal_candidate_since_us = 0;
    }

  } else {
    _terminal_candidate_since_us = 0;
  }

  const bool camera_servo_enabled =
      _param_dock_cam_pitch_variable.get() != 0;
  const float servo_minimum_pitch =
      math::min(_param_dock_cam_pitch_min.get(),
                _param_dock_cam_pitch_max.get());
  const float servo_maximum_pitch =
      math::max(_param_dock_cam_pitch_min.get(),
                _param_dock_cam_pitch_max.get());
  const float servo_deadband =
      math::constrain(_param_dock_cam_pitch_deadband.get(), 0.0f, 10.0f);
  const bool servo_at_outward_limit =
      camera_servo_enabled &&
      (((_camera_mount_pitch_deg <= servo_minimum_pitch + 0.5f) &&
        (_los_c_deg(1) < -servo_deadband)) ||
       ((_camera_mount_pitch_deg >= servo_maximum_pitch - 0.5f) &&
        (_los_c_deg(1) > servo_deadband)));

  if ((_stage != Stage::TERMINAL_DASH) && hard_fov_confirmed &&
      (_stage != Stage::EMERGENCY_RECOVERY)) {
    set_stage(Stage::EMERGENCY_RECOVERY);
    _aligned_since_us = 0;
    _recovery_stable_since_us = 0;

  } else if ((_stage == Stage::EMERGENCY_RECOVERY) &&
             (_fov_ratio < soft_ratio)) {
    set_stage(Stage::RECOVERY);
    _recovery_stable_since_us = 0;

  } else if (((_fov_ratio >= soft_ratio) || servo_at_outward_limit) &&
             (_stage != Stage::TERMINAL_DASH) &&
             (_stage != Stage::ALIGN) && (_stage != Stage::RECOVERY) &&
             (_stage != Stage::EMERGENCY_RECOVERY)) {
    set_stage(Stage::RECOVERY);
    _aligned_since_us = 0;
    _recovery_stable_since_us = 0;
  }

  const bool legacy_approach_enabled = _param_dock_legacy_approach.get() != 0;

  // When legacy approach is disabled, loss of range during an approach returns
  // to ALIGN. If it is enabled, angle tracking and FOV protection continue but
  // terminal distance braking remains unavailable until range returns.
  if ((_stage != Stage::TERMINAL_DASH) && !legacy_approach_enabled &&
      !_relative_position_valid &&
      (_stage != Stage::ALIGN) && (_stage != Stage::RECOVERY) &&
      (_stage != Stage::EMERGENCY_RECOVERY)) {
    set_stage(Stage::RECOVERY);
    _aligned_since_us = 0;
    _recovery_stable_since_us = 0;
  }

  const float acceleration =
      math::constrain(_param_dock_acc_max.get(), 0.1f, 20.0f);
  const float deceleration =
      math::constrain(_param_dock_dec_max.get(), 0.1f, 20.0f);
  const bool camera_servo_ready =
      !camera_servo_enabled ||
      ((!servo_at_outward_limit) &&
       (fabsf(_los_c_deg(1)) <= math::max(servo_deadband, 1.0f)));

  if (_stage == Stage::ALIGN) {
    _commanded_speed_ms = 0.0f;
    _velocity_setpoint_ned = _tracking_velocity_ned;
    const float alignment_speed =
        math::constrain(_param_dock_align_speed.get(), 0.1f, 5.0f);
    const float alignment_tracking_speed = _velocity_setpoint_ned.norm();

    if (alignment_tracking_speed > alignment_speed) {
      _velocity_setpoint_ned *= alignment_speed / alignment_tracking_speed;
    }

    constrain_velocity_setpoint(_velocity_setpoint_ned);

    if ((_relative_position_valid || legacy_approach_enabled) &&
        (_fov_ratio <= align_ratio) && camera_servo_ready) {
      if (_aligned_since_us == 0) {
        _aligned_since_us = now;
      }

      const uint64_t align_time_us = static_cast<uint64_t>(
          math::constrain(_param_dock_align_t.get(), 0.05f, 5.0f) * 1e6f);

      if ((now - _aligned_since_us) >= align_time_us) {
        set_stage(Stage::ACCELERATE);
        _aligned_since_us = 0;
        _recovery_stable_since_us = 0;
      }

    } else {
      _aligned_since_us = 0;
    }

  } else if (_stage == Stage::TERMINAL_DASH) {
    _recovery_stable_since_us = 0;
    _terminal_blind_since_us = 0;
    const float maximum_speed =
        maximum_approach_speed(approach_direction, legacy_approach_enabled);
    _commanded_speed_ms = maximum_speed;
    _velocity_setpoint_ned = compose_tracked_velocity(
        approach_direction, _tracking_velocity_ned, maximum_speed);
    constrain_velocity_setpoint(_velocity_setpoint_ned);

  } else if (_stage == Stage::RECOVERY) {
    _commanded_speed_ms =
        math::max(_commanded_speed_ms - deceleration * dt_s, 0.0f);

    if (_commanded_speed_ms > 0.05f) {
      _velocity_setpoint_ned = compose_tracked_velocity(
          approach_direction, _tracking_velocity_ned, _commanded_speed_ms);

    } else {
      _commanded_speed_ms = 0.0f;
      _velocity_setpoint_ned = _tracking_velocity_ned;
      const float alignment_speed =
          math::constrain(_param_dock_align_speed.get(), 0.1f, 5.0f);

      if (_velocity_setpoint_ned.norm() > alignment_speed) {
        _velocity_setpoint_ned *=
            alignment_speed / _velocity_setpoint_ned.norm();
      }
    }

    constrain_velocity_setpoint(_velocity_setpoint_ned);

    if ((_commanded_speed_ms <= 0.05f) && (_fov_ratio <= align_ratio) &&
        camera_servo_ready) {
      if (_recovery_stable_since_us == 0) {
        _recovery_stable_since_us = now;
      }

      const uint64_t recovery_time_us = static_cast<uint64_t>(
          math::constrain(_param_dock_recovery_time.get(), 0.1f, 5.0f) *
          1e6f);

      if ((now - _recovery_stable_since_us) >= recovery_time_us) {
        set_stage(Stage::ALIGN);
        _aligned_since_us = 0;
        _recovery_stable_since_us = 0;
      }

    } else {
      _recovery_stable_since_us = 0;
    }

  } else if (_stage == Stage::EMERGENCY_RECOVERY) {
    const float emergency_deceleration = 2.0f * deceleration;
    _commanded_speed_ms = math::max(
        _commanded_speed_ms - emergency_deceleration * dt_s, 0.0f);

    if (_commanded_speed_ms > 0.05f) {
      _velocity_setpoint_ned = compose_tracked_velocity(
          approach_direction, _tracking_velocity_ned, _commanded_speed_ms);

    } else {
      _commanded_speed_ms = 0.0f;
      _velocity_setpoint_ned = _tracking_velocity_ned;
      const float alignment_speed =
          math::constrain(_param_dock_align_speed.get(), 0.1f, 5.0f);

      if (_velocity_setpoint_ned.norm() > alignment_speed) {
        _velocity_setpoint_ned *=
            alignment_speed / _velocity_setpoint_ned.norm();
      }
    }

    constrain_velocity_setpoint(_velocity_setpoint_ned);
    _recovery_stable_since_us = 0;

  } else {
    _recovery_stable_since_us = 0;
    // Apply one common speed limit so downstream per-axis clipping cannot
    // rotate the approach direction.
    const float maximum_speed =
        maximum_approach_speed(approach_direction, legacy_approach_enabled);

    float desired_speed = maximum_speed;

    // Reduce approach speed continuously as the target consumes the remaining
    // image margin. This prevents the old accelerate/recovery limit cycle.
    if (_fov_ratio > align_ratio) {
      const float fov_speed_scale = math::constrain(
          (soft_ratio - _fov_ratio) / (soft_ratio - align_ratio), 0.0f, 1.0f);
      desired_speed *= fov_speed_scale;
    }

    if (_relative_position_valid) {
      const float stop_distance =
          math::constrain(_param_dock_stop_dist.get(), 0.0f, 100.0f);
      const float remaining_distance =
          math::max(_relative_distance_m - stop_distance, 0.0f);
      const float braking_speed =
          sqrtf(2.0f * deceleration * remaining_distance);
      desired_speed = math::min(desired_speed, braking_speed);
    }
    const float speed_delta =
        math::constrain(desired_speed - _commanded_speed_ms,
                        -deceleration * dt_s, acceleration * dt_s);
    _commanded_speed_ms = math::constrain(
        _commanded_speed_ms + speed_delta, 0.0f, maximum_speed);
    _velocity_setpoint_ned = compose_tracked_velocity(
        approach_direction, _tracking_velocity_ned, _commanded_speed_ms);
    constrain_velocity_setpoint(_velocity_setpoint_ned);

    if (desired_speed < maximum_speed - 0.05f) {
      set_stage(Stage::TERMINAL_BRAKE);

    } else if (_commanded_speed_ms >= maximum_speed - 0.05f) {
      set_stage(Stage::MAX_SPEED_APPROACH);

    } else {
      set_stage(Stage::ACCELERATE);
    }
  }

  limit_velocity_setpoint_slew(
      previous_velocity_setpoint_ned, dt_s, acceleration, deceleration,
      _stage == Stage::EMERGENCY_RECOVERY);
  constrain_velocity_setpoint(_velocity_setpoint_ned);

  if (_stage == Stage::TERMINAL_DASH) {
    _terminal_velocity_ned = _velocity_setpoint_ned;
  }

  _closing_speed_ms = _velocity_ned * approach_direction;
  _setpoint_valid = true;
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
}

// ---------- 数值计算辅助函数 ----------

float DockGuidance::low_pass_filter(float &state, float input, float cutoff_hz,
                                    float dt) {
  // 一阶 RC 低通：截止频率越低，视觉噪声抑制越强，但跟踪响应越慢。
  const float rc = 1.0f / (2.0f * M_PI_F * math::max(cutoff_hz, 0.01f));
  const float alpha = dt / (rc + dt);
  state = state + alpha * (input - state);
  return state;
}

float DockGuidance::derivative_filter(const float *buf) const {
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
    const float t =
        static_cast<float>(_los_deriv_time_buf[idx] - time_origin) * 1e-6f;
    const float y = buf[idx];
    sum_t += t;
    sum_y += y;
    sum_tt += t * t;
    sum_ty += t * y;
  }

  const float denominator = _deriv_count * sum_tt - sum_t * sum_t;
  return (denominator > 1e-9f)
             ? (_deriv_count * sum_ty - sum_t * sum_y) / denominator
             : 0.0f;
}

void DockGuidance::update_parameters() {
  const bool camera_pitch_was_variable =
      _param_dock_cam_pitch_variable.get() != 0;
  updateParams();
  const bool camera_pitch_is_variable =
      _param_dock_cam_pitch_variable.get() != 0;
  const float minimum_pitch = math::min(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float maximum_pitch = math::max(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());

  if (!camera_pitch_is_variable) {
    // Preserve the original fixed-camera behavior exactly, including mounts
    // whose static pitch lies outside the servo-only limits.
    _camera_mount_pitch_deg = _param_dock_cam_pitch.get();
    _camera_pitch_initialized = true;

  } else if (!_camera_pitch_initialized || !camera_pitch_was_variable) {
    _camera_mount_pitch_deg = math::constrain(_param_dock_cam_pitch.get(),
                                              minimum_pitch, maximum_pitch);
    _camera_pitch_initialized = true;

  } else {
    _camera_mount_pitch_deg =
        math::constrain(_camera_mount_pitch_deg, minimum_pitch, maximum_pitch);
  }

  update_camera_rotation();

  if (camera_pitch_is_variable || camera_pitch_was_variable) {
    // When disabling at runtime, send the configured fixed angle once so the
    // physical camera and the fixed-camera transform remain consistent.
    publish_camera_pitch_servo(hrt_absolute_time());
  }
}

void DockGuidance::update_camera_rotation() {
  // 三个参数定义从相机坐标系到 PX4 FRD 机体系的 3-2-1 旋转。
  // 固定版本使用 DOCK_CAM_PITCH；可变版本使用慢速舵机的当前指令角。
  _rot_cam_to_body = Dcmf(Eulerf(math::radians(_param_dock_cam_roll.get()),
                                 math::radians(_camera_mount_pitch_deg),
                                 math::radians(_param_dock_cam_yaw.get())));
  _rot_body_to_cam = _rot_cam_to_body;
  _rot_body_to_cam.transpose();
}

int DockGuidance::task_spawn(int argc, char *argv[]) {
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

int DockGuidance::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int DockGuidance::print_status() {
  PX4_INFO("Dock mode: %s", (_mode_id >= 0) ? "registered" : "not registered");
  PX4_INFO("mode id: %d, arming check id: %d", _mode_id, _arming_check_id);
  PX4_INFO("active: %s, vision: %s%s, fault latch: %s",
           _dock_mode_active ? "yes" : "no",
           _setpoint_valid ? "valid" : "invalid",
           _vision_degraded ? " (degraded)" : "",
           _mode_fault_latched ? "set" : "clear");
  PX4_INFO("velocity: %s, closing: %.2f m/s, command: %.2f m/s, image "
           "correction: %.2f m/s",
           _velocity_valid ? "valid" : "invalid", (double)_closing_speed_ms,
           (double)_commanded_speed_ms, (double)_tracking_velocity_ned.norm());
  PX4_INFO(
      "relative position: %s, distance: %.2f m, FOV ratio: %.2f "
      "(yaw %.2f, pitch %.2f), stage: %u",
      _relative_position_valid ? "valid" : "invalid",
      (double)_relative_distance_m, (double)_fov_ratio,
      (double)_fov_yaw_ratio, (double)_fov_pitch_ratio,
      static_cast<unsigned>(_stage));
  PX4_INFO("camera: %s, mount pitch: %.1f deg, yaw setpoint: %.1f deg",
           _param_dock_cam_pitch_variable.get() != 0 ? "pitch servo" : "fixed",
           (double)_camera_mount_pitch_deg,
           (double)(std::isfinite(_yaw_setpoint_rad)
                        ? math::degrees(_yaw_setpoint_rad)
                        : NAN));

  if (_setpoint_valid && !_relative_position_valid) {
    PX4_INFO("range unavailable: %s",
             _param_dock_legacy_approach.get() != 0
                 ? "legacy approach enabled, no terminal braking"
                 : "alignment only, high-speed approach inhibited");
  }
  return 0;
}

int DockGuidance::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description

Visual docking guidance controller.  Subscribes to viztra_observation for
camera line-of-sight and target-relative NED position. It produces a 3D NED
velocity setpoint through alignment, acceleration, maximum-speed approach,
terminal braking and recovery stages. With DOCK_CAM_SERVO=0, a mounting-aware
image servo reserves transverse velocity inside the same speed budget to keep
the target in a fixed camera. With DOCK_CAM_SERVO=1, a slow body-relative pitch
servo changes camera framing without attitude stabilization. The selectable
"Dock Guidance" mode uses PX4 multicopter position, attitude and rate control
for coupled thrust and altitude control.

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
