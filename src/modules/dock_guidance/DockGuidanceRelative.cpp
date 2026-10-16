// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#include "DockGuidance.hpp"
#include <mathlib/mathlib.h>

using matrix::Vector3f;
using matrix::Dcmf;

bool DockGuidance::relative_guidance_enabled() const {
  return _param_dock_dyn_enable.get() && _param_dock_terminal_enable.get() &&
      _param_dock_pred_enable.get();
}

void DockGuidance::compute_relative_control(const Vector3f &direction, const Vector3f &previous,
    const Dcmf &camera_to_ned, float dt, float align, float soft, bool hard, hrt_abstime now) {
  _relative_guidance_active = true;
  if (_relative_terminal_active) {
    _course_yaw_active = _preposition_enabled_for_activation && _airborne;
    relative_terminal(now, false);
    return;
  }
  _course_yaw_active = false;
  vehicle_local_position_s local{};
  uint64_t sample = 0;
  if (!_local_pos_sub.copy(&local) || !local.xy_valid || !local.z_valid ||
      !local.v_xy_valid || !local.v_z_valid ||
      !dock_motion::state_sample_time(local.timestamp_sample, local.timestamp, now, 100_ms, sample)) {
    invalidate_setpoint(now, "relative guidance local state unavailable", false);
    return;
  }
  _velocity_ned = Vector3f(local.vx, local.vy, local.vz);
  const Vector3f own = Vector3f(local.x, local.y, local.z) + _velocity_ned * ((now - sample) * 1e-6f);
  if (!own.isAllFinite() || !_velocity_ned.isAllFinite()) {
    invalidate_setpoint(now, "relative guidance local state nonfinite", false);
    return;
  }
  auto &c = _relative_context;
  c = {};
  c.relative = _relative_motion.fresh(now) ? _relative_motion.relative(now, own) : direction * 5.f;
  c.relative_rate = _relative_motion.fresh(now) ? _relative_motion.rate(_velocity_ned) : -_velocity_ned;
  c.own_velocity = _velocity_ned;
  c.previous = previous;
  c.previous_acceleration = _dynamic_acceleration;
  c.limits = dynamic_limits();
  c.dt = dt;
  c.acceleration = math::constrain(_param_dock_acc_max.get(), .1f, 20.f);
  c.deceleration = math::constrain(_param_dock_dec_max.get(), .1f, 20.f);
  c.jerk = math::constrain(_param_dock_dynamic_jerk.get(), .1f, 100.f);
  c.up = math::min(c.acceleration, math::constrain(_param_mpc_acc_up.get(), .1f, 20.f));
  c.down = math::min(c.deceleration, math::constrain(_param_mpc_acc_down.get(), .1f, 20.f));
  c.response = math::constrain(_param_dock_pred_response.get(), .05f, 2.f);
  vehicle_thrust_setpoint_s thrust{};
  vehicle_attitude_s attitude{};
  hover_thrust_estimate_s hover_estimate{};
  float hover = _param_mpc_hover.get();
  bool hover_valid = !_param_mpc_use_hte.get();
  if (_param_mpc_use_hte.get() && _hover_thrust_sub.copy(&hover_estimate) && hover_estimate.valid &&
      hover_estimate.timestamp && now >= hover_estimate.timestamp && now - hover_estimate.timestamp <= 500_ms) {
    hover = hover_estimate.hover_thrust;
    hover_valid = true;
  }
  if (hover_valid && _vehicle_thrust_sub.copy(&thrust) && _att_sub.copy(&attitude) &&
      thrust.timestamp && now >= thrust.timestamp && now - thrust.timestamp <= 100_ms &&
      attitude.timestamp && now >= attitude.timestamp && now - attitude.timestamp <= 100_ms) {
    const matrix::Quatf q(attitude.q);
    if (q.isAllFinite() && fabsf(q.norm() - 1.f) < .01f) {
      _relative_response.update(sample, _velocity_ned, Vector3f(local.ax, local.ay, local.az), previous,
          Dcmf(q) * Vector3f(thrust.xyz), hover, c.response);
    }
  }
  if (_relative_response.fresh(now)) {
    c.response_valid = true;
    hover = _relative_response.hover();
    c.response_bias = _relative_response.bias();
    c.drag_coefficient = _relative_response.drag();
    c.linear_drag = _relative_response.linear_drag();
    c.response_uncertainty = _relative_response.uncertainty();
    const float thrust_limit = _param_dock_thrust.get() > 0.f
        ? math::min(_param_mpc_thrust_max.get(), _param_dock_thrust.get()) : _param_mpc_thrust_max.get();
    c.thrust_acceleration = 9.80665f / hover * math::constrain(thrust_limit, .05f, 1.f);
    c.horizontal_force_margin = 9.80665f / hover * math::constrain(
        _param_mpc_thrust_xy_margin.get(), 0.f, thrust_limit);
  }
  c.horizon = math::constrain(_param_dock_pred_time.get(), .2f, 10.f);
  c.angle = math::radians(math::constrain(_param_dock_pred_angle.get(), 1.f, 30.f));
  c.tolerance = math::min(.25f, .25f * math::max(.2f, _param_dock_pass_error.get()));
  c.half_x = math::radians(.5f * _param_dock_fov_x.get());
  c.half_y = math::radians(.5f * _param_dock_fov_y.get());
  c.soft = soft;
  c.image_velocity = _tracking_velocity_ned;
  const float image_cap = math::constrain(_param_dock_img_fraction.get(), .05f, .8f) * c.limits.total;
  if (c.image_velocity.norm() > image_cap && c.image_velocity.norm() > .001f) {
    c.image_velocity *= image_cap / c.image_velocity.norm();
  }
  c.camera_to_ned = camera_to_ned;
  const float yaw = math::radians(_los_c_deg(0)), pitch = math::radians(_los_c_deg(1));
  c.ray = Vector3f(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
  if (_startup_ray_time && now > _startup_ray_time && now - _startup_ray_time < 100_ms) {
    const float elapsed = (now - _startup_ray_time) * 1e-6f;
    _startup_ray_rate += ((c.ray - _startup_previous_ray) / elapsed - _startup_ray_rate) *
        (elapsed / (.08f + elapsed));
  } else { _startup_ray_rate.zero(); }
  _startup_previous_ray = c.ray;
  _startup_ray_time = now;
  c.ray_rate = _startup_ray_rate;
  _vertical_accel_up = c.up;
  _vertical_accel_down = c.down;
  _height_brake_velocity = NAN;
  _following_command.zero();
  _capture_command.zero();
  _miss_correction = {};
  _prediction_ready = _terminal_ready = false;
  _closest_approach = {};
  _point_error_bound = _point_time_upper = INFINITY;
  _prediction_cross_sigma = _prediction_cross_velocity_sigma = INFINITY;

  const bool ready = _relative_motion.ready(now);
  if (ready) {
    record_closest_target_distance(c.relative.norm());
  }
  const float capture_speed = math::constrain(_param_dock_capture_speed.get(), 0.f, 10.f);
  float far_speed_limit = capture_speed;
  Vector3f far_direction{};
  Vector3f far_target_velocity{};
  bool far_reference_valid = false;
  // An unavailable external fix cannot prove that an older QGC-derived
  // command is still outside the lock gate.
  bool qgc_far_guidance_blocked = _preposition_enabled_for_activation &&
      _param_dock_pre_lock_distance.get() > 0.f;
  if (_preposition_enabled_for_activation && !ready &&
      update_rear_reference(now, local) && _pre_reference_valid) {
    const Vector3f target_line = _pre_target_position - own;
    const float target_range = target_line.norm();
    qgc_far_guidance_blocked = dock_motion::preposition_lock_distance_reached(target_line,
        _param_dock_pre_lock_distance.get());
    // The ground station position and the image must refer to the same target.
    // The absolute fix is only a far-field range source; it never makes the
    // relative observer ready or permits a terminal dash.
    if (_relative_motion.updates() < 4 && target_line.isAllFinite() && target_range > 1.f &&
        !qgc_far_guidance_blocked &&
        target_line * direction > 0.82f * target_range &&
        dock_motion::far_lookahead_direction(own, _pre_reference, _pre_target_position,
            far_direction)) {
      far_target_velocity = Vector3f(_pre_target.vn, _pre_target.ve, _pre_target.vd);
      c.relative = target_line;
      c.relative_rate = far_target_velocity - _velocity_ned;
      // The rear point shapes the approach path; it is not a stopping point
      // after visual handoff. Look ahead to the target across that point so
      // an early quality-based handoff keeps accelerating without a speed dip.
      far_speed_limit = c.limits.total;
      far_reference_valid = true;
    }
  }
  // Momentary soft-FOV excursions inhibit further acceleration but do not
  // restart ALIGN or wait for target-speed matching. Sustained hard-FOV
  // excursions retain explicit recovery and the existing yaw protection.
  const bool recovering = _dynamic_recovery.update(now, hard,
      _fov_ratio <= align, _param_dock_recovery_time.get());
  // Once visual scale is established, keep course-aligned yaw at speed.
  // Dropping this flag at metric handoff makes the camera-centering yaw turn
  // the fuselage sideways even while the requested ground track stays straight.
  _course_yaw_active = _preposition_enabled_for_activation && _airborne && !recovering &&
      (far_reference_valid || _relative_motion.updates() >= 4);
  Vector3f desired{};
  if (!_airborne) {
    _relative_plan_time = 0;
    set_stage(Stage::ALIGN);
  } else if (recovering) {
    _relative_plan_time = 0;
    desired = _tracking_velocity_ned;
    set_stage(hard ? Stage::EMERGENCY_RECOVERY : Stage::RECOVERY);
  } else if (ready) {
    // Once scale and relative rate are available, climb and forward closure
    // are one encounter trajectory. There is no settle-at-start-height phase.
    _startup_climb = _startup_height_hold = false;
    if (!_relative_plan_time || now - _relative_plan_time >= 50_ms) {
      const hrt_abstime started = hrt_absolute_time();
      _relative_trajectory = dock_motion::plan_relative_intercept(c);
      _relative_plan_runtime_us = hrt_absolute_time() - started;
      _relative_plan_time = now;
    }
    desired = _relative_trajectory.valid ? _relative_trajectory.desired : previous;
    if (_fov_ratio >= soft && desired.norm() > previous.norm() && desired.norm() > .001f) {
      desired *= previous.norm() / desired.norm();
    }
    set_stage((desired - previous).norm() < .5f ? Stage::MAX_SPEED_APPROACH : Stage::ACCELERATE);
  } else {
    _relative_plan_time = 0;
    _relative_trajectory = {};
    set_stage(far_reference_valid ? Stage::ACCELERATE : Stage::TRACK_CAPTURE);
    if (_relative_motion.updates() >= 4) {
      // A short loss of metric scale cannot authorize new acceleration or a
      // blind terminal. Inside the lock range do not retain a command that
      // may have originated from the external target position.
      desired = qgc_far_guidance_blocked
          ? direction * capture_speed + _tracking_velocity_ned : previous;
      if (_relative_motion.age(now) > .5f) {
        invalidate_setpoint(now, "relative scale observation expired");
        return;
      }
    } else {
      if (far_reference_valid) {
        desired = far_target_velocity + far_direction * far_speed_limit +
            _tracking_velocity_ned;
        if (_fov_ratio >= soft && desired.norm() > previous.norm() && desired.norm() > .001f) {
          desired *= previous.norm() / desired.norm();
        }
        _startup_climb = _startup_height_hold = false;
      } else {
        // Without a fresh absolute fix, brake back to visual capture speed.
        // Without rear guidance, retain the original fast-entry continuity.
        const float speed = _preposition_enabled_for_activation ? capture_speed :
            math::max(previous.norm(), capture_speed);
        desired = direction * speed + _tracking_velocity_ned;
      }
      if (!far_reference_valid && _startup_height_hold && std::isfinite(_startup_target_z)) {
        desired(2) = dock_motion::height_capture_velocity(local.z - _startup_target_z, local.vz,
            c.limits.up, c.down, c.jerk, hard ? 0.f : 1.f);
      }
    }
  }
  const auto plan = dock_motion::relative_step(c, desired, previous, _dynamic_acceleration, dt, true);
  _desired_velocity = desired;
  _velocity_setpoint_ned = plan.velocity;
  _dynamic_acceleration = plan.acceleration;
  _velocity_limits_override = plan.limits_override;
  _predicted_fov_ratio = plan.predicted_fov;
  _capture_evidence_valid = _capture_acceleration_ready = ready && !recovering && _airborne;
  _capture_speed_limit = ready ? c.limits.total : far_reference_valid ? far_speed_limit : capture_speed;
  _dynamic_reachable = ready && _relative_trajectory.valid;
  _dynamic_closure = (plan.velocity - _velocity_ned - c.relative_rate) * direction;
  _dynamic_max_closure = ready ? c.limits.total : far_reference_valid ? far_speed_limit : 0.f;
  _prediction_reject = ready ? dock_motion::PredictionReject::NoCrossing : dock_motion::PredictionReject::NoScale;
  if (ready && _airborne && !recovering && !plan.limits_override) {
    _closest_approach = dock_motion::relative_frozen_approach(c, plan.velocity);
    const auto evidence = dock_motion::relative_point_evidence(_relative_motion, now, _closest_approach,
        c.response_uncertainty);
    _point_error_bound = evidence.miss_bound_m;
    _point_time_upper = evidence.time_upper_s;
    _prediction_cross_sigma = _closest_approach.valid ? _relative_motion.sigma(
        _closest_approach.approach_axis, _relative_motion.age(now) + _closest_approach.time_s, true) : INFINITY;
    _prediction_ready = _relative_trajectory.valid && _relative_trajectory.closest.valid &&
        _fov_ratio < soft && plan.predicted_fov < soft;
    if (_prediction_ready) {
      _miss_correction.valid = true;
      _miss_correction.miss = _relative_trajectory.closest.miss;
      _miss_correction.time_to_plane = _relative_trajectory.closest.time_s;
      _miss_correction.correction = desired - previous;
      _prediction_reject = dock_motion::PredictionReject::Ready;
    }
    // Include clearance of the pass plane in the blind budget, not just time
    // to minimum separation. Lock the command and crossing direction; safe
    // visual measurements may still update the pass estimate after entry.
    const bool stable_terminal_evidence = _relative_response.fresh(now) &&
        _relative_trajectory.valid && _relative_trajectory.closest.valid && evidence.valid &&
        std::isfinite(evidence.miss_bound_m + evidence.time_upper_s) &&
        evidence.miss_bound_m < _param_dock_pass_error.get() &&
        evidence.time_upper_s > 0.f &&
        evidence.time_upper_s <= math::constrain(_param_dock_terminal_max_time.get(), .1f, 10.f) &&
        _closest_approach.relative_speed_m_s > .5f &&
        _fov_ratio < soft && _bbox_edge_ratio < soft && plan.predicted_fov < soft &&
        (_param_dock_terminal_distance.get() <= 0.f ||
         c.relative.norm() <= _param_dock_terminal_distance.get());
    const bool blind_window = dock_motion::relative_terminal_window(evidence, _relative_motion.age(now),
        _closest_approach.relative_speed_m_s, _param_dock_pass_distance.get(), _param_dock_pass_error.get(),
        math::constrain(_param_dock_blind_tgo.get(), .1f, .5f),
        math::constrain(_param_dock_terminal_blind.get(), .05f, 3.f), _param_dock_terminal_max_time.get());
    if (dock_motion::relative_terminal_confirmed(now, _terminal_candidate_since_us,
        stable_terminal_evidence, blind_window, _param_dock_terminal_confirm.get())) {
      _relative_terminal_active = _terminal_ready = true;
      _terminal_started_us = now;
      _terminal_blind_since_us = 0;
      _terminal_direction = _closest_approach.approach_axis;
      _terminal_velocity_ned = plan.velocity;
      _terminal_plan_deadline = dock_motion::relative_terminal_deadline(
          _terminal_started_us, now, _relative_motion.age(now),
          _param_dock_terminal_blind.get(), _param_dock_terminal_max_time.get());
      set_stage(Stage::TERMINAL_DASH);
    }
  } else { _terminal_candidate_since_us = 0; }
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  _closing_speed_ms = _velocity_ned * direction;
  _setpoint_valid = true;
  publish_guidance_status(now, true, false);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
}

bool DockGuidance::relative_terminal(hrt_abstime now, bool blind) {
  vehicle_local_position_s local{};
  uint64_t sample = 0;
  if (!relative_guidance_enabled() || !_airborne || !_terminal_started_us ||
      now < _terminal_started_us) {
    invalidate_setpoint(now, "relative terminal inactive/invalid state", false);
    return false;
  }
  if (now > _terminal_plan_deadline) {
    invalidate_setpoint(now, "relative terminal deadline", false);
    return false;
  }
  if (_relative_motion.age(now) > math::constrain(_param_dock_terminal_blind.get(), .05f, 3.f)) {
    invalidate_setpoint(now, "relative terminal observation age limit", false);
    return false;
  }
  if (!_local_pos_sub.copy(&local) || !local.xy_valid || !local.z_valid ||
      !local.v_xy_valid || !local.v_z_valid ||
      local.xy_reset_counter != _xy_reset_counter || local.z_reset_counter != _z_reset_counter ||
      local.vxy_reset_counter != _vxy_reset_counter || local.vz_reset_counter != _vz_reset_counter ||
      !dock_motion::state_sample_time(local.timestamp_sample, local.timestamp, now, 100_ms, sample)) {
    invalidate_setpoint(now, "relative terminal local state unavailable", false);
    return false;
  }
  _velocity_ned = Vector3f(local.vx, local.vy, local.vz);
  const Vector3f own = Vector3f(local.x, local.y, local.z) + _velocity_ned * ((now - sample) * 1e-6f);
  const Vector3f relative = _relative_motion.relative(now, own);
  const float along = relative * _terminal_direction;
  const float across = (relative - _terminal_direction * along).norm();
  const float age = _relative_motion.age(now);
  const float cross_sigma = _relative_motion.sigma(_terminal_direction, age, true);
  const float along_sigma = _relative_motion.sigma(_terminal_direction, age, false);
  if (!own.isAllFinite() || !_velocity_ned.isAllFinite() || !relative.isAllFinite() ||
      !std::isfinite(cross_sigma + along_sigma) || across + 2.f * cross_sigma > _param_dock_pass_error.get() ||
      !dock_motion::feasible(_terminal_velocity_ned, dynamic_limits())) {
    invalidate_setpoint(now, "relative terminal corridor/limits changed");
    return false;
  }
  record_closest_target_distance(relative.norm());
  _relative_context.relative = relative;
  _relative_context.relative_rate = _relative_motion.rate(_velocity_ned);
  if (along + 2.f * along_sigma < -_param_dock_pass_distance.get()) {
    _relative_terminal_active = _terminal_ready = false;
    set_stage(Stage::PASS_PREDICTED);
    publish_predicted_pass(now);
    return false;
  }
  if (blind && !_terminal_blind_since_us) { _terminal_blind_since_us = now; }
  if (!blind) { _terminal_blind_since_us = 0; }
  _vision_degraded = blind;
  _prediction_ready = false;
  _miss_correction = {};
  _velocity_setpoint_ned = _terminal_velocity_ned;
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  _last_control_update_us = now;
  publish_guidance_status(now, true, blind);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);
  return false;
}

void DockGuidance::publish_relative_status(hrt_abstime now, bool valid) {
  dock_guidance_relative_s status{};
  status.timestamp = now;
  status.active = _relative_guidance_active;
  status.rate_ready = valid && _relative_motion.ready(now);
  status.trajectory_valid = valid && _prediction_ready && _relative_plan_time && now >= _relative_plan_time &&
      now - _relative_plan_time <= 100_ms && _relative_trajectory.closest.valid;
  status.terminal_committed = _relative_terminal_active;
  _relative_context.relative.copyTo(status.relative_position);
  _relative_context.relative_rate.copyTo(status.relative_rate);
  status.trajectory_miss = status.trajectory_valid ? _relative_trajectory.closest.miss.norm() : NAN;
  status.trajectory_tgo = status.trajectory_valid ? _relative_trajectory.closest.time_s : NAN;
  status.frozen_miss_bound = valid ? _point_error_bound : NAN;
  status.frozen_time_upper = valid ? _point_time_upper : NAN;
  status.observation_age = _relative_motion.age(now);
  status.plan_age = _relative_plan_time && now >= _relative_plan_time ? (now - _relative_plan_time) * 1e-6f : NAN;
  status.updates = _relative_motion.updates();
  status.planner_runtime_us = _relative_plan_runtime_us;
  _relative_context.response_bias.copyTo(status.response_bias);
  _relative_context.drag_coefficient.copyTo(status.drag_coefficient);
  _relative_context.linear_drag.copyTo(status.linear_drag);
  status.response_uncertainty = _relative_context.response_uncertainty;
  status.thrust_acceleration = _relative_context.thrust_acceleration;
  status.response_valid = valid && _relative_response.fresh(now);
  _relative_pub.publish(status);
}
