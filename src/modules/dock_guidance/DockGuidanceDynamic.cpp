// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#include "DockGuidance.hpp"
#include <mathlib/mathlib.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Vector3f;

bool DockGuidance::delayed_position(hrt_abstime target, Vector3f &position, Vector3f *velocity) const {
  // 在曝光时刻插值本机 NED 位置；目标运动估计必须使用与图像同一时刻的机位。
  // 历史未覆盖、定位无效或相邻采样间隔过大时拒绝插值。
  if (!target || _state_history.count() == 0) {
    return false;
  }
  const int oldest = _state_history.oldest();
  const int newest = _state_history.newest();
  if (target < _delay_time_buf[oldest]) {
    return false;
  }
  if (target >= _delay_time_buf[newest]) {
    position = _position_delay_buf[newest];
    if (velocity) { *velocity = _velocity_delay_buf[newest]; }
    return _position_delay_valid[newest] && target - _delay_time_buf[newest] < 20_ms;
  }
  for (int n = 1; n < _state_history.count(); ++n) {
    const int a = (oldest + n - 1) % DELAY_BUF_SIZE, b = (oldest + n) % DELAY_BUF_SIZE;
    if (target <= _delay_time_buf[b]) {
      if (!_position_delay_valid[a] || !_position_delay_valid[b] ||
          _delay_time_buf[b] - _delay_time_buf[a] > 100_ms) {
        return false;
      }
      const float weight =
          float(target - _delay_time_buf[a]) / float(_delay_time_buf[b] - _delay_time_buf[a]);
      position = _position_delay_buf[a] * (1.f - weight) + _position_delay_buf[b] * weight;
      if (velocity) { *velocity = _velocity_delay_buf[a] * (1.f - weight) + _velocity_delay_buf[b] * weight; }
      return position.isAllFinite();
    }
  }
  return false;
}

void DockGuidance::update_target_motion(const viztra_observation_s &obs, const Vector3f &att_deg,
                                        float yaw_deg, float pitch_deg, hrt_abstime exposure,
                                        const Dcmf &sample_camera_to_body) {
  // 动态估计先把相机视线转到 NED，再选取可用的米制量测：
  // 优先采用帧内相对位置；否则仅用已标定实际尺寸且未被画面裁剪的目标框推算距离。
  // 只有角度时可更新已初始化滤波器的横向信息，但不能凭角度初始化距离。
  if (!_param_dock_dyn_enable.get()) {
    return;
  }
  _target_motion_valid = false;
  Vector3f own, own_velocity;
  vehicle_local_position_s local{};
  const hrt_abstime now = hrt_absolute_time();
  uint64_t position_time = 0;
  if (!delayed_position(exposure, own, &own_velocity) || !_local_pos_sub.copy(&local) || !local.xy_valid ||
      !local.z_valid || !local.v_xy_valid || !local.v_z_valid ||
      !dock_motion::state_sample_time(local.timestamp_sample, local.timestamp, now, 100_ms, position_time)) {
    return;
  }
  const Vector3f own_now =
      Vector3f(local.x, local.y, local.z) +
      Vector3f(local.vx, local.vy, local.vz) * ((now - position_time) * 1e-6f);
  if (!own_now.isAllFinite()) {
    return;
  }
  const float yaw = math::radians(yaw_deg), pitch = math::radians(pitch_deg);
  const Vector3f camera_los(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
  const Dcmf rotation(
      Eulerf(math::radians(att_deg(0)), math::radians(att_deg(1)), math::radians(att_deg(2))));
  Vector3f los = rotation * sample_camera_to_body * camera_los;
  float range = NAN, sigma = NAN;
  const Vector3f measured_relative(obs.relative_n_m, obs.relative_e_m, obs.relative_d_m);
  bool metric = obs.relative_position_valid && measured_relative.isAllFinite() &&
                measured_relative.norm() > .2f;
  const float rx = math::constrain(_param_dock_res_x.get(), 100.f, 8000.f);
  const float ry = math::constrain(_param_dock_res_y.get(), 100.f, 8000.f);
  const float fx =
      rx / (2.f * tanf(math::radians(math::constrain(_param_dock_fov_x.get(), 10.f, 80.f)) * .5f));
  const float fy =
      ry / (2.f * tanf(math::radians(math::constrain(_param_dock_fov_y.get(), 10.f, 80.f)) * .5f));
  if (metric) {
    // 相对位置为真正的米制观测；其长度与方向同时覆盖图像视线近似。
    range = measured_relative.norm();
    los = measured_relative / range;
    sigma = math::max(.2f, range * .02f);
  } else {
    // 框碰到图像边缘会低估可见尺寸并高估距离，故必须检查四边均未裁剪。
    const bool uncropped =
        camera_los(0) > .2f &&
        fabsf(fx * camera_los(1) / camera_los(0)) + .5f * obs.target_width_px < .5f * rx - 2.f &&
        fabsf(fy * camera_los(2) / camera_los(0)) + .5f * obs.target_height_px < .5f * ry - 2.f;
    metric = uncropped && dock_motion::box_range(
                              obs.target_width_px, obs.target_height_px,
                              _param_dock_target_width.get(), _param_dock_target_height.get(), fx,
                              fy, camera_los(0), _param_dock_size_error.get(), range, sigma);
  }
  if (metric && exposure) {
    record_closest_target_distance(range);
  }
  if (exposure) {
    if (metric && !_relative_terminal_active) {
      _relative_motion.update(exposure, own, own_velocity, los * range, sigma,
          math::max(.0005f, 2.f / math::min(fx, fy)),
          math::constrain(_param_dock_est_accel.get(), .1f, 20.f));
    }
    // 本机位置噪声会直接进入目标位置差分；量测方差必须纳入 EKF 位置误差，
    // 避免把本机定位抖动识别为目标运动。
    const float own_error = std::isfinite(local.eph) && std::isfinite(local.epv)
                                ? math::max(.1f, math::max(local.eph, local.epv))
                                : 5.f;
    const uint32_t resets_before = _target_motion.resets();
    const float bearing_range = metric ? range : math::max(5.f, _estimated_relative.norm());
    _target_motion.update(exposure, own, los, metric ? range : NAN,
                          metric ? hypotf(sigma, own_error) : NAN,
                          math::max(2.f / math::min(fx, fy), own_error / bearing_range),
                          _param_dock_est_accel.get(),
                          math::constrain(_param_dock_est_max_speed.get(), 1.f, 100.f));
    if (_target_motion.resets() != resets_before) {
      _estimator_reset_reason = 3; // 观测间断或估计状态不合理。
    }
  }
  if (_target_motion.initialized()) {
    _target_velocity = _target_motion.velocity();
    _estimated_relative = _target_motion.relative(now, own_now);
    _target_position_error = _target_motion.position_sigma(_target_motion.age(now));
    _target_motion_valid = _target_motion.valid(now, _param_dock_est_velocity_error.get()) &&
                           _estimated_relative.norm() > .2f;
    if (_target_motion_valid) {
      record_closest_target_distance(_estimated_relative.norm());
    }
  }
}

dock_motion::Limits DockGuidance::dynamic_limits() const {
  // 同时传入总速、水平、上升和下降上限，后续按最终三维速度组合检查可行性。
  return {math::constrain(_param_dock_vel_max.get(), 0.f, 100.f),
          math::max(_param_mpc_xy_vel_max.get(), 0.f),
          math::max(_param_mpc_z_vel_max_up.get(), 0.f),
          math::max(_param_mpc_z_vel_max_down.get(), 0.f)};
}

void DockGuidance::compute_dynamic_control(const Vector3f &direction, const Vector3f &previous,
                                           const Dcmf &camera_to_ned, float dt, float align, float soft, bool hard,
                                           hrt_abstime now) {
  _relative_guidance_active = false;
  _course_yaw_active = false;
  // 动态路径将目标速度跟随、图像纠偏、沿目标视线的闭合速度叠加，
  // 最后统一经过速度/加速度/加加速度和预测视场约束。
  // trusted 需要新鲜的米制目标状态；部分速度跟随可更早启用，但不能提前放行高速闭合。
  if (_stage == Stage::TERMINAL_DASH) {
    if (!dynamic_terminal(now, false)) {
      return;
    }
  }
  const auto limits = dynamic_limits();
  const float pursuit_limit = dock_motion::approach_limit(
      _param_dock_terminal_enable.get(), limits.total, _param_dock_relative_speed.get());
  const float acceleration = math::constrain(_param_dock_acc_max.get(), .1f, 20.f);
  const float deceleration = math::constrain(_param_dock_dec_max.get(), .1f, 20.f);
  _vertical_accel_up = math::min(acceleration, math::constrain(_param_mpc_acc_up.get(), .1f, 20.f));
  _vertical_accel_down = math::min(deceleration, math::constrain(_param_mpc_acc_down.get(), .1f, 20.f));
  // Either vector budget can be selected after the desired velocity changes.
  // The stopping envelope must not assume more braking than either permits.
  const float height_deceleration = math::min(acceleration, _vertical_accel_down);
  const bool trusted = _airborne && _target_motion_valid &&
                       _target_motion.valid(now, _param_dock_est_velocity_error.get());
  Vector3f correction = _tracking_velocity_ned;
  if (_stage == Stage::ALIGN || !trusted) {
    const float cap = math::constrain(_param_dock_align_speed.get(), .1f, 5.f);
    if (correction.norm() > cap) {
      correction *= cap / correction.norm();
    }
  }
  const float correction_cap =
      math::constrain(_param_dock_img_fraction.get(), .05f, .8f) * limits.total;
  if (correction.norm() > correction_cap && correction.norm() > .001f) {
    correction *= correction_cap / correction.norm();
  }
  // 米制速度尚在收敛时允许有限跟随：不确定分量逐渐衰减；
  // 主动向目标闭合仍必须等待完整目标状态可信。
  Vector3f following =
      _airborne ? _target_motion.following_velocity(now, _param_dock_est_velocity_error.get())
                : Vector3f{};
  if (_startup_climb) {
    following(2) = 0.f;
    correction(2) = 0.f;
  }
  const Vector3f base = following + correction;
  // 以完整的 base + 闭合速度求交可行域；跟随的竖直分量可能被闭合分量抵消，
  // 不能先单独裁剪 base 再算剩余预算。
  _dynamic_max_closure = trusted ? dock_motion::closing_budget(
                                       base, direction, limits,
                                       pursuit_limit)
                                 : 0.f;
  _dynamic_reachable = trusted && _dynamic_max_closure > .1f;
  float startup_vz = 0.f;
  vehicle_local_position_s height_local{};
  if (_startup_height_hold && _airborne) {
    vehicle_local_position_s local{};
    if (!_local_pos_sub.copy(&local) || !local.z_valid || !local.v_z_valid ||
        now < local.timestamp || now - local.timestamp > 100_ms ||
        !std::isfinite(local.z) || !std::isfinite(local.vz) || !std::isfinite(_startup_target_z)) {
      invalidate_setpoint(now, "startup climb local height unavailable");
      return;
    }
    height_local = local;
    const float remaining = local.z - _startup_target_z;
    if (remaining <= .3f) {
      _startup_height_reached = true;
    }
    const float fov_scale = hard ? 0.f
                                 : dock_motion::clamp((soft - _fov_ratio) / (soft - align), 0.f, 1.f);
    startup_vz = dock_motion::height_capture_velocity(remaining, local.vz, limits.up,
        height_deceleration, _param_dock_dynamic_jerk.get(), fov_scale);
    // 起始高度完成只取决于高度和爬升速度；目标可观测性与图像对准另行判定。
    // 完成爬升后仍可能先进入受限捕获，而不是立即高速追踪。
    if (_startup_climb) {
      if (_startup_height_reached && fabsf(local.vz) <= .5f) {
        if (!_startup_settled_since) { _startup_settled_since = now; }
        if (now - _startup_settled_since >= 500_ms) {
          _startup_climb = false;
          _aligned_since_us = 0;
          set_stage(Stage::ALIGN);
        }
      } else {
        _startup_settled_since = 0;
      }
    }
  }
  float wanted_closure = 0.f;
  if (!_airborne) {
    _dynamic_recovery.reset();
    set_stage(Stage::ALIGN);
    _aligned_since_us = 0;
  } else if (_startup_climb) {
    _dynamic_recovery.reset();
    set_stage(Stage::STARTUP_CLIMB);
    _aligned_since_us = _terminal_candidate_since_us = 0;
    _dynamic_closure = _dynamic_max_closure = 0.f;
    _dynamic_reachable = false;
  } else if (_stage != Stage::TERMINAL_DASH) {
    // 视场越界优先恢复；目标速度不可信时保留 TRACK_CAPTURE，
    // 只有目标跟随误差、LOS 角速度和对准时间共同满足条件才放行加速。
    const bool recovering = _dynamic_recovery.update(now,
        hard || _fov_ratio >= soft || (trusted && !_dynamic_reachable),
        _fov_ratio <= align, _param_dock_recovery_time.get());
    if (hard) {
      set_stage(Stage::EMERGENCY_RECOVERY);
      _aligned_since_us = 0;
    } else if (recovering) {
      set_stage(Stage::RECOVERY);
      _aligned_since_us = 0;
    } else if (!trusted) {
      set_stage(Stage::TRACK_CAPTURE);
      _aligned_since_us = 0;
    } else if (_stage == Stage::TRACK_CAPTURE) {
      const Vector3f relative_tracking = _velocity_ned - base;
      const float along = relative_tracking * direction;
      const float across = (relative_tracking - direction * along).norm();
      // 捕获阶段若已经稳定沿 LOS 闭合，目标速度刚达到可信阈值时无需先刹停再加速。
      if (_capture_schedule.ready() && _fov_ratio <= align && along > .1f &&
          across <= _param_dock_match_error.get() &&
          _los_e_dot.norm() < math::radians(5.f)) {
        _dynamic_closure = math::constrain((previous - base) * direction, 0.f, _dynamic_max_closure);
        set_stage(Stage::ACCELERATE);
      } else {
        set_stage(Stage::ALIGN);
      }
    } else if (_stage == Stage::RECOVERY || _stage == Stage::EMERGENCY_RECOVERY) {
      if (_dynamic_closure < .1f && _fov_ratio <= align) {
        set_stage(Stage::ALIGN);
      }
    }
    if (_stage == Stage::ALIGN) {
      if (trusted && _dynamic_reachable && _fov_ratio <= align &&
          (_velocity_ned - base).norm() <= _param_dock_match_error.get() &&
          _los_e_dot.norm() < math::radians(5.f)) {
        if (!_aligned_since_us) {
          _aligned_since_us = now;
        }
        if (now - _aligned_since_us >=
            static_cast<uint64_t>(math::max(.1f, _param_dock_align_t.get()) * 1e6f)) {
          set_stage(Stage::ACCELERATE);
        }
      } else {
        _aligned_since_us = 0;
      }
    }
    if (trusted && (_stage == Stage::ACCELERATE || _stage == Stage::MAX_SPEED_APPROACH ||
                    _stage == Stage::TERMINAL_BRAKE)) {
      wanted_closure =
          _dynamic_max_closure * dock_motion::clamp((soft - _fov_ratio) / (soft - align), 0.f, 1.f);
      wanted_closure = dock_motion::distance_limited_approach_speed(_param_dock_terminal_enable.get(),
          wanted_closure, _estimated_relative.norm(), _param_dock_stop_dist.get(), deceleration);
      const Vector3f relative_velocity = _velocity_ned - _target_velocity;
      const float actual_closure = relative_velocity * direction;
      const auto closest = dock_motion::predict_closest_approach(
          _estimated_relative, _velocity_ned, _target_velocity, previous,
          math::constrain(_param_dock_pred_response.get(), .05f, 2.f),
          math::constrain(_param_dock_pred_time.get(), .2f, 10.f));
      const auto point_evidence = dock_motion::assess_closest_approach(_target_motion, now, closest);
      const float pass_time = point_evidence.time_upper_s +
          _param_dock_pass_distance.get() / math::max(actual_closure, .5f);
      // The target accepts entry from any direction: require a bounded 3-D
      // closest approach rather than only a small error in a chosen plane.
      const bool candidate =
          _param_dock_terminal_enable.get() && _fov_ratio <= align &&
          actual_closure > .5f && point_evidence.valid &&
          pass_time < .8f * _param_dock_terminal_max_time.get() &&
          point_evidence.miss_bound_m < _param_dock_pass_error.get() &&
          (_param_dock_terminal_distance.get() <= 0.f ||
           _estimated_relative.norm() <= _param_dock_terminal_distance.get());
      if (candidate) {
        if (!_terminal_candidate_since_us) {
          _terminal_candidate_since_us = now;
        }
        if (now - _terminal_candidate_since_us >=
            static_cast<uint64_t>(math::constrain(_param_dock_terminal_confirm.get(), .02f, 2.f) *
                                  1e6f)) {
          _terminal_direction = direction;
          _terminal_started_us = now;
          _terminal_blind_since_us = 0;
          _terminal_velocity_ned = previous;
          set_stage(Stage::TERMINAL_DASH);
        }
      } else {
        _terminal_candidate_since_us = 0;
      }
      if (_stage != Stage::TERMINAL_DASH) {
        set_stage(wanted_closure < _dynamic_max_closure - .05f
                      ? Stage::TERMINAL_BRAKE
                      : (_dynamic_closure >= wanted_closure - .05f ? Stage::MAX_SPEED_APPROACH
                                                                   : Stage::ACCELERATE));
      }
    } else {
      _terminal_candidate_since_us = 0;
    }
  }
  if (_stage == Stage::TERMINAL_DASH) {
    wanted_closure = _dynamic_max_closure;
    _terminal_blind_since_us = 0;
  }
  const float rate = hard ? 2.f * deceleration : deceleration;
  _dynamic_closure +=
      dock_motion::clamp(wanted_closure - _dynamic_closure, -rate * dt, acceleration * dt);
  Vector3f desired =
      _airborne ? base + direction * _dynamic_closure
                : Vector3f{};
  _following_command = following;
  _capture_command.zero();
  _capture_speed_limit = 0.f;
  _capture_evidence_valid = false;
  _capture_acceleration_ready = false;
  if (_airborne && (_startup_climb || _stage == Stage::TRACK_CAPTURE)) {
    Vector3f capture_direction = direction;
    if (_startup_climb) {
      capture_direction(2) = 0.f;
    }
    if (capture_direction.norm() > .05f) {
      capture_direction.normalize();
      const float range_lower = _estimated_relative.norm() - 2.f * _target_position_error;
      const bool pass_through_latched = _param_dock_terminal_enable.get() &&
                                        _capture_schedule.ready();
      const float range_required = math::max(10.f, _param_dock_stop_dist.get());
      const float los_rate = _los_e_dot.norm();
      const float acquisition_rate = math::radians(5.f);
      _capture_gate.evaluated = true;
      _capture_gate.range_lower_m = range_lower;
      _capture_gate.range_required_m = range_required;
      _capture_gate.fov_limit = pass_through_latched ? soft : align;
      _capture_gate.los_rate_deg_s = math::degrees(los_rate);
      _capture_gate.los_rate_limit_deg_s = math::degrees(acquisition_rate);
      _capture_gate.confirmation_required_s = math::constrain(_param_dock_align_t.get(), .1f, 5.f);
      _capture_gate.pass_through_latched = pass_through_latched;
      _capture_gate.range_fresh = _target_motion.fresh_range(now);
      _capture_gate.not_hard = !hard;
      _capture_gate.fov_ok = std::isfinite(_fov_ratio + align + soft) &&
                             (pass_through_latched ? _fov_ratio < soft : _fov_ratio <= align);
      _capture_gate.los_rate_ok = std::isfinite(los_rate) &&
                                  (pass_through_latched || los_rate < acquisition_rate);
      _capture_gate.range_ok = std::isfinite(range_lower) &&
                                (pass_through_latched || range_lower > range_required);
      // 末段穿越属于追击动作：米制目标速度尚未完全收敛时，可用已确认的
      // 几何和图像证据争取可用闭合速度；发布前仍会检查视场、分轴速度、加速度和加加速度。
      const float initial_capture_limit = _param_dock_terminal_enable.get()
                                          ? pursuit_limit
                                          : _param_dock_capture_speed.get();
      _capture_gate.limit_enabled = initial_capture_limit > 0.f;
      const bool common_capture_evidence = _capture_gate.range_fresh &&
          std::isfinite(range_lower) && dock_motion::capture_image_evidence(
              hard, _fov_ratio, align, soft, los_rate,
              acquisition_rate, pass_through_latched);
      const bool capture_evidence = dock_motion::capture_evidence_allowed(
          common_capture_evidence, _param_dock_terminal_enable.get(),
          pass_through_latched, range_lower,
          range_required);
      const float closing_bias = (base - _target_velocity) * capture_direction +
          2.f * _target_motion.directional_velocity_sigma(capture_direction);
      const float capture_limit = _capture_schedule.update(now, capture_evidence,
          _param_dock_align_t.get(), initial_capture_limit,
          pursuit_limit, _param_dock_terminal_enable.get(), range_lower, closing_bias,
          _param_dock_stop_dist.get(), deceleration, _param_dock_dynamic_jerk.get());
      _capture_evidence_valid = capture_evidence;
      _capture_acceleration_ready = _capture_schedule.ready();
      _capture_gate.evidence_time_s = _capture_schedule.evidence_duration(now);
      _capture_speed_limit = capture_limit;
      const float capture_scale = hard ? 0.f : dock_motion::clamp(
          (soft - _fov_ratio) / (soft - align), 0.f, 1.f);
      // 在完整目标速度和纠偏合成向量上分配闭合预算，再统一缩放速度；
      // 爬升捕获可能抵消部分向下的目标跟随分量。
      const float budget = dock_motion::closing_budget(desired, capture_direction, limits,
          capture_limit);
      _capture_command = capture_direction * (budget * capture_scale);
      desired += _capture_command;
      // 起始高度只是初次捕获目标，不是永久的竖直速度限制。
      // 高度捕获且几何/图像证据持续新鲜后解除高度保持；目标速度可信度仍单独
      // 决定速度前馈和末段穿越能否启用。
      if (!_startup_climb && _capture_schedule.ready()) {
        _startup_height_hold = false;
      }
    } else {
      _capture_schedule.reset();
    }
  } else {
    _capture_schedule.reset();
  }
  if (_startup_climb && _airborne) {
    desired(2) = startup_vz;
  } else if (_startup_height_hold && _airborne) {
    // 起始爬升结束后，有限的竖直速度前馈逐步移动高度参考点。
    // 当前馈为零时仍能抵消缓慢位置漂移；参考点与当前位置距离限幅以防积分积累。
    const float vertical_feedforward = math::constrain(desired(2), -1.f, 1.f);
    _startup_target_z = math::constrain(_startup_target_z + vertical_feedforward * dt,
        height_local.z - 2.f, height_local.z + 2.f);
    desired(2) = math::constrain(vertical_feedforward + .8f * (_startup_target_z - height_local.z),
        -math::min(3.f, limits.up), math::min(1.f, limits.down));
    if (trusted && (_stage == Stage::ACCELERATE || _stage == Stage::MAX_SPEED_APPROACH ||
                    _stage == Stage::TERMINAL_DASH)) {
      _startup_height_hold = false;
    }
  }
  _height_brake_velocity = NAN;
  const bool height_brake_allowed = _airborne && !_startup_climb && !_startup_height_hold &&
      _stage != Stage::TERMINAL_DASH && _target_motion.recent_metric(now) &&
      _estimated_relative.isAllFinite() && std::isfinite(_target_position_error);
  if (height_brake_allowed) {
    // The target's vertical speed is used only after the full estimate is
    // trusted. Otherwise a recent camera-derived height still prevents the
    // aircraft from continuing a fast climb through the target's altitude.
    const float target_vz = trusted ? _target_velocity(2) : 0.f;
    desired(2) = dock_motion::limit_target_climb_velocity(
        desired(2), _estimated_relative(2), _velocity_ned(2), target_vz,
        _target_position_error, height_deceleration, _param_dock_dynamic_jerk.get());
    _height_brake_velocity = desired(2);
  }
  desired = dock_motion::constrained(desired, limits);
  // 捕获阶段的完整目标速度还不可信时，三维穿越修正常因距离方差过大被拒绝。
  // 持续同号的垂直错失仍可先给出小幅预修正，为加速度/jerk 规划器争取时间；
  // 不使用这条软修正放行终端盲飞，也不改变已经收敛的东西向控制。
  const bool early_vertical_eligible = _param_dock_pred_enable.get() &&
      _stage == Stage::TRACK_CAPTURE && _capture_schedule.ready() &&
      _capture_evidence_valid && !_startup_height_hold && !hard &&
      _fov_ratio < align && _camera_time_aligned &&
      _target_motion.motion_state() && _target_motion.fresh_range(now) &&
      std::isfinite(_target_position_error);
  if (early_vertical_eligible) {
    const float prediction_time = math::constrain(_param_dock_capture_predict_time.get(), .2f, 5.f);
    const auto preview = dock_motion::correct_predicted_miss(_estimated_relative,
        _velocity_ned, _target_velocity, desired, direction,
        math::constrain(_param_dock_pred_response.get(), .05f, 2.f), prediction_time,
        math::radians(math::constrain(_param_dock_pred_angle.get(), 1.f, 30.f)), acceleration);
    const float vertical_delta = dock_motion::soft_capture_vertical_delta(
        preview, _target_position_error, acceleration);
    const auto evidence = preview.valid ? dock_motion::assess_prediction(_target_motion, now,
        direction, _velocity_ned, _target_velocity, preview, prediction_time,
        math::max(.2f, _param_dock_pass_error.get()),
        math::max(.2f, _param_dock_est_velocity_error.get())) : dock_motion::PredictionEvidence{};
    const float confirmed_delta = _capture_vertical_latch.update(now,
        _target_motion.accepted_updates(), evidence.usable ? 0.f : vertical_delta);
    if (fabsf(confirmed_delta) > .01f) {
      const float before_vertical = desired(2);
      desired(2) += confirmed_delta;
      desired = dock_motion::constrained(desired, limits);
      _early_vertical_bias = desired(2) - before_vertical;
    }
  } else {
    _capture_vertical_latch.reset();
  }
  // 在最终速度规划前修正预测的三维穿越误差；视场/居中约束仍具有优先权。
  // 完整三维速度仍按原门槛标记；捕获时仅允许短时、低横向方差的穿越修正。
  const bool approach_stage = _stage == Stage::ACCELERATE ||
      _stage == Stage::MAX_SPEED_APPROACH || _stage == Stage::TERMINAL_BRAKE ||
      _stage == Stage::TERMINAL_DASH;
  const bool capture_prediction = _stage == Stage::TRACK_CAPTURE &&
      _capture_schedule.ready() && _capture_evidence_valid;
  _prediction_ready = false;
  _prediction_cross_sigma = _prediction_cross_velocity_sigma = INFINITY;
  _prediction_reject = !_param_dock_pred_enable.get()
      ? dock_motion::PredictionReject::Disabled : dock_motion::PredictionReject::NoScale;
  if (_param_dock_pred_enable.get() && _airborne &&
      (capture_prediction || (trusted && approach_stage)) &&
      !_startup_height_hold && !hard && _fov_ratio < soft && _camera_time_aligned) {
    _miss_correction = dock_motion::correct_predicted_miss(_estimated_relative,
        _velocity_ned, _target_velocity, desired, direction,
        math::constrain(_param_dock_pred_response.get(), .05f, 2.f),
        capture_prediction ? math::constrain(_param_dock_capture_predict_time.get(), .2f, 5.f)
                           : math::constrain(_param_dock_pred_time.get(), .2f, 10.f),
        math::radians(math::constrain(_param_dock_pred_angle.get(), 1.f, 30.f)), acceleration);
    if (_miss_correction.valid) {
      const auto evidence = dock_motion::assess_prediction(_target_motion, now, direction,
          _velocity_ned, _target_velocity, _miss_correction,
          capture_prediction ? math::constrain(_param_dock_capture_predict_time.get(), .2f, 5.f)
                             : math::constrain(_param_dock_pred_time.get(), .2f, 10.f),
          math::max(.2f, _param_dock_pass_error.get()),
          math::max(.2f, _param_dock_est_velocity_error.get()));
      _prediction_cross_sigma = evidence.transverse_sigma;
      _prediction_cross_velocity_sigma = evidence.transverse_velocity_sigma;
      _prediction_reject = evidence.reject;
      if (evidence.usable) {
        const Vector3f nominal = desired;
        const Vector3f limited = dock_motion::constrained(_miss_correction.velocity, limits);
        if ((limited - _target_velocity) * direction > .5f) {
          desired = limited;
          _miss_correction.velocity = desired;
          _miss_correction.correction = desired - nominal;
          _prediction_ready = true;
        } else {
          _prediction_reject = dock_motion::PredictionReject::Unreachable;
        }
      }
    }
    if (!_prediction_ready) { _miss_correction.valid = false; }
    if (!_miss_correction.valid && _prediction_reject == dock_motion::PredictionReject::NoScale) {
      _prediction_reject = dock_motion::PredictionReject::NoCrossing;
    }
  } else if (_param_dock_pred_enable.get()) {
    _prediction_reject = !_target_motion.initialized()
        ? dock_motion::PredictionReject::NoScale
        : (!_target_motion.fresh_range(now)
            ? dock_motion::PredictionReject::Stale
            : (!_target_motion.motion_state()
                ? dock_motion::PredictionReject::NoMotionEvidence
                : ((hard || _fov_ratio >= soft)
                    ? dock_motion::PredictionReject::ImageEdge
                    : dock_motion::PredictionReject::NotEligible)));
  }
  if (height_brake_allowed) {
    // A subsequent image/prediction correction must not undo the vertical
    // stopping envelope. If it requested more climb, its unmodified miss
    // prediction no longer describes the velocity sent to the planner.
    const float target_vz = trusted ? _target_velocity(2) : 0.f;
    const float braked_vz = dock_motion::limit_target_climb_velocity(
        desired(2), _estimated_relative(2), _velocity_ned(2), target_vz,
        _target_position_error, height_deceleration, _param_dock_dynamic_jerk.get());
    if (braked_vz > desired(2) + .001f && _prediction_ready) {
      _prediction_ready = false;
      _miss_correction.valid = false;
      _prediction_reject = dock_motion::PredictionReject::PlannerLimited;
    }
    desired(2) = braked_vz;
    _height_brake_velocity = braked_vz;
  }
  _desired_velocity = desired;
  const float max_accel = desired.norm() < previous.norm() ? rate : acceleration;
  const float vector_accel_limit = _startup_height_hold ? acceleration : max_accel;
  {
    const float yaw = math::radians(_los_c_deg(0)), pitch = math::radians(_los_c_deg(1));
    const Vector3f ray(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
    if (_startup_ray_time && now > _startup_ray_time && now - _startup_ray_time < 100_ms) {
      const float elapsed = (now - _startup_ray_time) * 1e-6f;
      const Vector3f measured_ray_rate = (ray - _startup_previous_ray) / elapsed;
      _startup_ray_rate +=
          (measured_ray_rate - _startup_ray_rate) * (elapsed / (.08f + elapsed));
    } else {
      _startup_ray_rate.zero();
    }
    _startup_previous_ray = ray;
    _startup_ray_time = now;
    // 保守距离只影响短时平移预测；图像漂移和机体倾斜预测不依赖可信米制距离。
    const float range = _target_motion.initialized() && _estimated_relative.isAllFinite()
                          && std::isfinite(_target_position_error)
                        ? math::max(5.f, _estimated_relative.norm() - 2.f * _target_position_error) : 5.f;
    const auto plan = dock_motion::plan_velocity(desired, previous, _dynamic_acceleration,
        limits, dt, vector_accel_limit, math::constrain(_param_dock_dynamic_jerk.get(), .1f, 100.f),
        ray, _startup_ray_rate, camera_to_ned, range,
        math::radians(.5f * _param_dock_fov_x.get()), math::radians(.5f * _param_dock_fov_y.get()),
        soft, _startup_height_hold, _startup_climb, _vertical_accel_up, _vertical_accel_down);
    _velocity_setpoint_ned = plan.velocity;
    _dynamic_acceleration = plan.acceleration;
    _velocity_limits_override = plan.limits_override;
    _predicted_fov_ratio = plan.predicted_fov;
  }
  _closest_approach = {};
  _point_error_bound = INFINITY;
  _point_time_upper = INFINITY;
  if (_target_motion.recent_metric(now) && _estimated_relative.isAllFinite() &&
      _velocity_ned.isAllFinite() && _target_velocity.isAllFinite()) {
    _closest_approach = dock_motion::predict_closest_approach(
        _estimated_relative, _velocity_ned, _target_velocity, _velocity_setpoint_ned,
        math::constrain(_param_dock_pred_response.get(), .05f, 2.f),
        math::constrain(_param_dock_pred_time.get(), .2f, 10.f));
    const auto point_evidence = dock_motion::assess_closest_approach(
        _target_motion, now, _closest_approach);
    if (point_evidence.valid) {
      _point_error_bound = point_evidence.miss_bound_m;
      _point_time_upper = point_evidence.time_upper_s;
    }
  }
  // Planner 的视场/动态限幅可能挡住修正；只有实际发布的速度朝修正方向变化
  // 才把 prediction_valid 计为真，且需要新观测持续确认。
  if (_prediction_ready) {
    const float applied = (_velocity_setpoint_ned - previous) * _miss_correction.correction;
    if (applied <= .001f || _predicted_fov_ratio >= soft) {
      _prediction_ready = false;
      _miss_correction.valid = false;
      _prediction_reject = _predicted_fov_ratio >= soft
          ? dock_motion::PredictionReject::ImageEdge
          : dock_motion::PredictionReject::PlannerLimited;
    }
  }
  if (_prediction_ready) {
    if (!_prediction_candidate_since) {
      _prediction_candidate_since = now;
      _prediction_start_updates = _target_motion.accepted_updates();
    }
    if (now - _prediction_candidate_since <
            static_cast<uint64_t>(math::constrain(_param_dock_prediction_confirm.get(), .1f, 1.f) * 1e6f) ||
        _target_motion.accepted_updates() < _prediction_start_updates + 2) {
      _prediction_reject = dock_motion::PredictionReject::Confirming;
    }
  } else {
    _prediction_candidate_since = 0;
  }
  const float pass_error = math::max(.2f, _param_dock_pass_error.get());
  const float predicted_miss = _miss_correction.valid
      ? (_miss_correction.miss - direction * (_miss_correction.miss * direction)).norm()
      : INFINITY;
  const bool commit_capture_plan = capture_prediction && _prediction_ready &&
      _prediction_reject == dock_motion::PredictionReject::Ready &&
      _param_dock_terminal_enable.get() && !_startup_height_hold &&
      _closest_approach.valid &&
      _point_time_upper <= math::constrain(_param_dock_blind_tgo.get(), .1f, .5f) &&
      _point_error_bound < pass_error &&
      predicted_miss + 2.f * _prediction_cross_sigma < pass_error &&
      _bbox_edge_ratio < soft && _predicted_fov_ratio < soft &&
      (_param_dock_terminal_distance.get() <= 0.f ||
       _estimated_relative.norm() <= _param_dock_terminal_distance.get());
  if (commit_capture_plan) {
    if (!_capture_terminal_candidate_since) { _capture_terminal_candidate_since = now; }
    if (now - _capture_terminal_candidate_since >=
        static_cast<uint64_t>(math::constrain(_param_dock_terminal_confirm.get(), .02f, 2.f) * 1e6f)) {
      _capture_terminal_plan = _terminal_ready = true;
      _terminal_direction = direction;
      _terminal_started_us = now;
      _terminal_blind_since_us = 0;
      _terminal_velocity_ned = _velocity_setpoint_ned;
      const float duration = math::min(_param_dock_terminal_blind.get(),
          _point_time_upper + .1f);
      _terminal_plan_deadline = now + static_cast<uint64_t>(math::max(.05f, duration) * 1e6f);
      set_stage(Stage::TERMINAL_DASH);
    }
  } else {
    _capture_terminal_candidate_since = 0;
  }
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  // 地速与相对目标的闭合速度分别记录，避免把两者混作同一个性能指标。
  _closing_speed_ms = _velocity_ned * direction;
  if (_stage == Stage::TERMINAL_DASH) {
    _terminal_velocity_ned = _velocity_setpoint_ned;
  }
  _setpoint_valid = true;
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
}

bool DockGuidance::dynamic_terminal(hrt_abstime now, bool blind) {
  if (_relative_terminal_active) { return relative_terminal(now, blind); }
  // 动态末段每次都重验本机定位、目标估计、穿越平面和总时长。
  // blind 为真时只允许在观测年龄限制内沿最后的末段速度继续，不能凭旧目标状态无限追踪。
  if (blind) { _miss_correction = {}; }
  vehicle_local_position_s local{};
  if (!_airborne || !_target_motion.initialized() || !_local_pos_sub.copy(&local) ||
      !local.xy_valid || !local.z_valid || !local.v_xy_valid || !local.v_z_valid ||
      now < local.timestamp || now - local.timestamp > 100_ms || !_terminal_started_us ||
      now < _terminal_started_us ||
      now - _terminal_started_us >
          static_cast<uint64_t>(math::constrain(_param_dock_terminal_max_time.get(), .1f, 10.f) *
                                1e6f)) {
    invalidate_setpoint(now, "dynamic terminal state/time limit");
    return false;
  }
  _velocity_ned = Vector3f(local.vx, local.vy, local.vz);
  const Vector3f own =
      Vector3f(local.x, local.y, local.z) + _velocity_ned * ((now - local.timestamp) * 1e-6f);
  if (!own.isAllFinite() || !_velocity_ned.isAllFinite()) {
    invalidate_setpoint(now, "dynamic terminal nonfinite local state");
    return false;
  }
  _estimated_relative = _target_motion.relative(now, own);
  _target_position_error = _target_motion.position_sigma(_target_motion.age(now));
  if (_capture_terminal_plan) {
    if (!_param_dock_pred_enable.get() || !_param_dock_terminal_enable.get()) {
      invalidate_setpoint(now, "capture terminal disabled after commit");
      return false;
    }
    // 仅凭入末段前锁定的相机估计延拓；不把仿真目标真值接进控制。
    // 穿越判定分别检查平面内误差和沿程误差，均有有限截止时间。
    const float age = _target_motion.age(now);
    const float along = _estimated_relative * _terminal_direction;
    const float across = (_estimated_relative - _terminal_direction * along).norm();
    const float cross_error = _target_motion.transverse_position_sigma(_terminal_direction, age);
    if (now > _terminal_plan_deadline ||
        age > math::constrain(_param_dock_terminal_blind.get(), .05f, 3.f) ||
        !std::isfinite(along + across + cross_error + _target_position_error) ||
        across + 2.f * cross_error > _param_dock_pass_error.get()) {
      invalidate_setpoint(now, "capture terminal plan expired/uncertain");
      return false;
    }
    record_closest_target_distance(_estimated_relative.norm());
    if (along + 2.f * _target_position_error < -_param_dock_pass_distance.get()) {
      _capture_terminal_plan = false;
      _terminal_ready = false;
      set_stage(Stage::PASS_PREDICTED);
      publish_predicted_pass(now);
      return false;
    }
    _prediction_ready = false;
    _prediction_reject = blind ? dock_motion::PredictionReject::Lost
                               : dock_motion::PredictionReject::Ready;
    _miss_correction = {};
    _vision_degraded = blind;
    if (blind && !_terminal_blind_since_us) { _terminal_blind_since_us = now; }
    _velocity_setpoint_ned = dock_motion::constrained(_terminal_velocity_ned, dynamic_limits());
    _last_control_update_us = now;
    publish_guidance_status(now, true, blind);
    publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
    return false;
  }
  const auto pass =
      dock_motion::classify_pass(_estimated_relative, _terminal_direction, _target_position_error,
                                 _param_dock_pass_distance.get(), _param_dock_pass_error.get());
  if (pass == dock_motion::PassState::Uncertain) {
    invalidate_setpoint(now, "dynamic terminal prediction uncertain");
    return false;
  }
  if (blind) {
    if (!_terminal_blind_since_us) {
      _terminal_blind_since_us = now;
    }
    // 限制支撑估计的真实观测年龄，而不是只计算看门狗第一次触发后的经过时间。
    if (_target_motion.age(now) > math::constrain(_param_dock_terminal_blind.get(), .05f, 3.f)) {
      invalidate_setpoint(now, "dynamic terminal observation age limit");
      return false;
    }
  } else if (!_target_motion_valid) {
    invalidate_setpoint(now, "dynamic terminal estimate invalid");
    return false;
  }
  record_closest_target_distance(_estimated_relative.norm());
  if (pass == dock_motion::PassState::Predicted) {
    // 已预测越过锁定平面时转入减速保持；这只是几何判定，并非实体接触确认。
    set_stage(Stage::PASS_PREDICTED);
    publish_predicted_pass(now);
    return false;
  }
  if (blind) {
    _target_motion_valid = _target_motion.valid(now, _param_dock_est_velocity_error.get());
    _vision_degraded = true;
    _velocity_setpoint_ned = dock_motion::constrained(_terminal_velocity_ned, dynamic_limits());
    _last_control_update_us = now;
    publish_guidance_status(now, true, true);
    publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
    return false;
  }
  return true;
}

void DockGuidance::publish_predicted_pass(hrt_abstime now) {
  // 推力制动以零速度为闭环目标，由下层对实际速度反向倾斜并平滑改变姿态/油门。
  // 高速期间允许高度暂时偏移，避免为了立即消除爬升而大幅收油门。
  vehicle_local_position_s local{};
  if (!_local_pos_sub.copy(&local) || !local.v_xy_valid || !local.v_z_valid ||
      !local.timestamp || now < local.timestamp || now - local.timestamp > 500_ms ||
      !Vector3f(local.vx, local.vy, local.vz).isAllFinite() || !std::isfinite(local.heading)) {
    invalidate_setpoint(now, "post-pass local velocity/heading invalid");
    return;
  }
  const float dt = _last_control_update_us && now > _last_control_update_us
                       ? math::constrain((now - _last_control_update_us) * 1e-6f, .002f, .1f)
                       : LOOP_DT;
  _velocity_ned = Vector3f(local.vx, local.vy, local.vz);
  _velocity_valid = true;
  _velocity_speed_ms = _velocity_ned.norm();
  // 高速锁定实际机头，不继续追赶通过前尚未到达的 yaw 目标。
  // 三维实速严格低于阈值后才限速恢复目标；俯仰/横滚制动不受此门控影响。
  const float recovery_yaw = _recovery_heading.update(_velocity_ned, local.heading, _yaw_setpoint_rad,
      math::constrain(_param_dock_recovery_yaw_speed.get(), .1f, 20.f),
      math::radians(math::constrain(_param_dock_yaw_rate.get(), 1.f, 90.f)), dt,
      local.heading_reset_counter, local.delta_heading);
  if (!std::isfinite(recovery_yaw)) {
    invalidate_setpoint(now, "post-pass heading gate invalid");
    return;
  }
  _yaw_setpoint_rad = recovery_yaw;
  const float speed = _velocity_setpoint_ned.norm();
  if (_param_dock_brake_enable.get()) {
    _velocity_setpoint_ned.zero();
  } else if (speed > .001f) {
    _velocity_setpoint_ned *=
        math::max(0.f, speed - math::max(.1f, _param_dock_dec_max.get()) * dt) / speed;
  } else {
    _velocity_setpoint_ned.zero();
  }
  _dynamic_closure = _dynamic_max_closure = 0.f;
  _dynamic_reachable = false;
  _target_motion_valid = false;
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  _last_control_update_us = now;
  _setpoint_valid = true;
  // PASS_PREDICTED 已经不依赖视觉；不能让盲穿阶段遗留的退化标志关闭制动推力。
  _vision_degraded = false;
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
  report_closest_target_distance();
}
