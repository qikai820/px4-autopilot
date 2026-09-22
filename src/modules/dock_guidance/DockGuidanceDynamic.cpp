// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#include "DockGuidance.hpp"
#include <mathlib/mathlib.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Vector3f;

bool DockGuidance::delayed_position(hrt_abstime sample, Vector3f &position) const {
  const uint64_t delay =
      static_cast<uint64_t>(math::constrain(_param_dock_cam_dly.get(), int32_t{0}, int32_t{5000})) *
      1000;
  if (sample < delay || _delay_count == 0) {
    return false;
  }
  const uint64_t target = sample - delay;
  const int oldest = (_delay_idx - _delay_count + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;
  const int newest = (_delay_idx - 1 + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;
  if (target < _delay_time_buf[oldest]) {
    return false;
  }
  if (target >= _delay_time_buf[newest]) {
    position = _position_delay_buf[newest];
    return _position_delay_valid[newest] && target - _delay_time_buf[newest] < 20_ms;
  }
  for (int n = 1; n < _delay_count; ++n) {
    const int a = (oldest + n - 1) % DELAY_BUF_SIZE, b = (oldest + n) % DELAY_BUF_SIZE;
    if (target <= _delay_time_buf[b]) {
      if (!_position_delay_valid[a] || !_position_delay_valid[b] ||
          _delay_time_buf[b] - _delay_time_buf[a] > 100_ms) {
        return false;
      }
      const float weight =
          float(target - _delay_time_buf[a]) / float(_delay_time_buf[b] - _delay_time_buf[a]);
      position = _position_delay_buf[a] * (1.f - weight) + _position_delay_buf[b] * weight;
      return position.isAllFinite();
    }
  }
  return false;
}

void DockGuidance::update_target_motion(const viztra_observation_s &obs, const Vector3f &att_deg,
                                        float yaw_deg, float pitch_deg, hrt_abstime sample) {
  if (!_param_dock_dyn_enable.get()) {
    return;
  }
  _target_motion_valid = false;
  Vector3f own;
  vehicle_local_position_s local{};
  const hrt_abstime now = hrt_absolute_time();
  if (!delayed_position(sample, own) || !_local_pos_sub.copy(&local) || !local.xy_valid ||
      !local.z_valid || !local.v_xy_valid || !local.v_z_valid ||
      now < local.timestamp || now - local.timestamp > 100_ms) {
    return;
  }
  const Vector3f own_now =
      Vector3f(local.x, local.y, local.z) +
      Vector3f(local.vx, local.vy, local.vz) * ((now - local.timestamp) * 1e-6f);
  if (!own_now.isAllFinite()) {
    return;
  }
  const float yaw = math::radians(yaw_deg), pitch = math::radians(pitch_deg);
  const Vector3f camera_los(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
  const Dcmf rotation(
      Eulerf(math::radians(att_deg(0)), math::radians(att_deg(1)), math::radians(att_deg(2))));
  Vector3f los = rotation * _rot_cam_to_body * camera_los;
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
    range = measured_relative.norm();
    los = measured_relative / range;
    sigma = math::max(.2f, range * .02f);
  } else {
    const bool uncropped =
        camera_los(0) > .2f &&
        fabsf(fx * camera_los(1) / camera_los(0)) + .5f * obs.target_width_px < .5f * rx - 2.f &&
        fabsf(fy * camera_los(2) / camera_los(0)) + .5f * obs.target_height_px < .5f * ry - 2.f;
    metric = uncropped && dock_motion::box_range(
                              obs.target_width_px, obs.target_height_px,
                              _param_dock_target_width.get(), _param_dock_target_height.get(), fx,
                              fy, camera_los(0), _param_dock_size_error.get(), range, sigma);
  }
  const uint64_t delay =
      static_cast<uint64_t>(math::constrain(_param_dock_cam_dly.get(), int32_t{0}, int32_t{5000})) *
      1000;
  if (sample > delay) {
    // Local-position noise must not masquerade as target motion.
    const float own_error = std::isfinite(local.eph) && std::isfinite(local.epv)
                                ? math::max(.1f, math::max(local.eph, local.epv))
                                : 5.f;
    const uint32_t resets_before = _target_motion.resets();
    const float bearing_range = metric ? range : math::max(5.f, _estimated_relative.norm());
    _target_motion.update(sample - delay, own, los, metric ? range : NAN,
                          metric ? hypotf(sigma, own_error) : NAN,
                          math::max(2.f / math::min(fx, fy), own_error / bearing_range),
                          _param_dock_est_accel.get(),
                          math::constrain(_param_dock_est_max_speed.get(), 1.f, 100.f));
    if (_target_motion.resets() != resets_before) {
      _estimator_reset_reason = 3; // observation gap or implausible state
    }
  }
  if (_target_motion.initialized()) {
    _target_velocity = _target_motion.velocity();
    _estimated_relative = _target_motion.relative(now, own_now);
    _target_position_error = _target_motion.position_sigma(_target_motion.age(now));
    _target_motion_valid = _target_motion.valid(now, _param_dock_est_velocity_error.get()) &&
                           _estimated_relative.norm() > .2f;
  }
}

dock_motion::Limits DockGuidance::dynamic_limits() const {
  return {math::constrain(_param_dock_vel_max.get(), 0.f, 100.f),
          math::max(_param_mpc_xy_vel_max.get(), 0.f),
          math::max(_param_mpc_z_vel_max_up.get(), 0.f),
          math::max(_param_mpc_z_vel_max_down.get(), 0.f)};
}

void DockGuidance::compute_dynamic_control(const Vector3f &direction, const Vector3f &previous,
                                           const Dcmf &camera_to_ned, float dt, float align, float soft, bool hard,
                                           hrt_abstime now) {
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
  // Partial following is allowed while metric velocity converges. Uncertain
  // components fade to zero; active closure still needs the full estimate.
  Vector3f following =
      _airborne ? _target_motion.following_velocity(now, _param_dock_est_velocity_error.get())
                : Vector3f{};
  if (_startup_climb) {
    following(2) = 0.f;
    correction(2) = 0.f;
  }
  const Vector3f base = following + correction;
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
        deceleration, _param_dock_dynamic_jerk.get(), fov_scale);
    // Height completion is independent of target observability and image
    // alignment. Once complete, the next stage may still be bounded capture.
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
    set_stage(Stage::ALIGN);
    _aligned_since_us = 0;
  } else if (_startup_climb) {
    set_stage(Stage::STARTUP_CLIMB);
    _aligned_since_us = _terminal_candidate_since_us = 0;
    _dynamic_closure = _dynamic_max_closure = 0.f;
    _dynamic_reachable = false;
  } else if (_stage != Stage::TERMINAL_DASH) {
    if (hard) {
      set_stage(Stage::EMERGENCY_RECOVERY);
      _aligned_since_us = 0;
    } else if (_fov_ratio >= soft || (trusted && !_dynamic_reachable)) {
      set_stage(Stage::RECOVERY);
      _aligned_since_us = 0;
    } else if (!trusted) {
      set_stage(Stage::TRACK_CAPTURE);
      _aligned_since_us = 0;
    } else if (_stage == Stage::TRACK_CAPTURE) {
      const Vector3f relative_tracking = _velocity_ned - base;
      const float along = relative_tracking * direction;
      const float across = (relative_tracking - direction * along).norm();
      // A capture already closing along LOS need not brake to zero merely
      // because metric velocity confidence has just become available.
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
      if (!_param_dock_terminal_enable.get()) {
        const float remaining =
            math::max(0.f, _estimated_relative.norm() - _param_dock_stop_dist.get());
        wanted_closure = math::min(wanted_closure, sqrtf(2.f * deceleration * remaining));
      }
      const Vector3f relative_velocity = _velocity_ned - _target_velocity;
      const float actual_closure = relative_velocity * direction;
      const float ttc =
          actual_closure > .5f ? _estimated_relative.norm() / actual_closure : INFINITY;
      const float pass_time =
          ttc + _param_dock_pass_distance.get() / math::max(actual_closure, .5f);
      const float miss = (_estimated_relative - relative_velocity * ttc).norm();
      const bool candidate =
          _param_dock_terminal_enable.get() && _fov_ratio <= align && std::isfinite(ttc) &&
          pass_time < .8f * _param_dock_terminal_max_time.get() &&
          miss + 2.f * _target_motion.position_sigma(_target_motion.age(now) + pass_time) <
              _param_dock_pass_error.get() &&
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
      _airborne ? dock_motion::constrained(base + direction * _dynamic_closure, limits)
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
      const bool common_capture_evidence = _target_motion.fresh_range(now) &&
          std::isfinite(range_lower) && dock_motion::capture_image_evidence(
              hard, _fov_ratio, align, soft, _los_e_dot.norm(),
              math::radians(5.f), pass_through_latched);
      const bool capture_evidence = dock_motion::capture_evidence_allowed(
          common_capture_evidence, _param_dock_terminal_enable.get(),
          pass_through_latched, range_lower,
          math::max(10.f, _param_dock_stop_dist.get()));
      const float closing_bias = (base - _target_velocity) * capture_direction +
          2.f * _target_motion.directional_velocity_sigma(capture_direction);
      const float capture_limit = _capture_schedule.update(now, capture_evidence,
          _param_dock_align_t.get(), _param_dock_capture_speed.get(),
          pursuit_limit, _param_dock_terminal_enable.get(), range_lower, closing_bias,
          _param_dock_stop_dist.get(), deceleration, _param_dock_dynamic_jerk.get());
      _capture_evidence_valid = capture_evidence;
      _capture_acceleration_ready = _capture_schedule.ready();
      _capture_speed_limit = capture_limit;
      const float capture_scale = hard ? 0.f : dock_motion::clamp(
          (soft - _fov_ratio) / (soft - align), 0.f, 1.f);
      const float budget = dock_motion::closing_budget(base, capture_direction, limits,
          capture_limit);
      _capture_command = capture_direction * (budget * capture_scale);
      desired += _capture_command;
      // The 10 m startup height is an initial acquisition target, not a
      // permanent 1 m/s vertical approach limit. Release only after altitude
      // capture and sustained, fresh geometric/image evidence. Velocity
      // confidence still independently gates target feedforward/terminal entry.
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
    // After startup, bounded vertical tracking advances a position reference.
    // A zero requested vertical speed now actually holds altitude, including
    // slow local-position drift. Limit accumulated lag to avoid windup.
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
  desired = dock_motion::constrained(desired, limits);
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
    // Conservative depth affects only the short-horizon translation prediction;
    // image drift and camera-tilt prediction do not require metric target validity.
    const float range = _target_motion.initialized() && _estimated_relative.isAllFinite()
                          && std::isfinite(_target_position_error)
                        ? math::max(5.f, _estimated_relative.norm() - 2.f * _target_position_error) : 5.f;
    const auto plan = dock_motion::plan_velocity(desired, previous, _dynamic_acceleration,
        limits, dt, vector_accel_limit, math::constrain(_param_dock_dynamic_jerk.get(), .1f, 100.f),
        ray, _startup_ray_rate, camera_to_ned, range,
        math::radians(.5f * _param_dock_fov_x.get()), math::radians(.5f * _param_dock_fov_y.get()),
        soft, _startup_height_hold, _startup_climb);
    _velocity_setpoint_ned = plan.velocity;
    _dynamic_acceleration = plan.acceleration;
    _velocity_limits_override = plan.limits_override;
    _predicted_fov_ratio = plan.predicted_fov;
  }
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  // Preserve the existing ground-speed diagnostic; relative speed has its own field.
  _closing_speed_ms = _velocity_ned * direction;
  if (_stage == Stage::TERMINAL_DASH) {
    _terminal_velocity_ned = _velocity_setpoint_ned;
  }
  _setpoint_valid = true;
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
}

bool DockGuidance::dynamic_terminal(hrt_abstime now, bool blind) {
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
    // Bound the actual age of supporting data, not only time since watchdog firing.
    if (_target_motion.age(now) > math::constrain(_param_dock_terminal_blind.get(), .05f, 3.f)) {
      invalidate_setpoint(now, "dynamic terminal observation age limit");
      return false;
    }
  } else if (!_target_motion_valid) {
    invalidate_setpoint(now, "dynamic terminal estimate invalid");
    return false;
  }
  if (pass == dock_motion::PassState::Predicted) {
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
  vehicle_local_position_s local{};
  if (!_local_pos_sub.copy(&local) || !local.v_xy_valid || !local.v_z_valid ||
      now < local.timestamp || now - local.timestamp > 500_ms ||
      !Vector3f(local.vx, local.vy, local.vz).isAllFinite()) {
    invalidate_setpoint(now, "post-pass local velocity invalid");
    return;
  }
  const float dt = _last_control_update_us && now > _last_control_update_us
                       ? math::constrain((now - _last_control_update_us) * 1e-6f, .002f, .1f)
                       : LOOP_DT;
  const float speed = _velocity_setpoint_ned.norm();
  if (speed > .001f) {
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
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
}
