#include "DockGuidance.hpp"

#include <mathlib/math/Functions.hpp>

using namespace matrix;

namespace {
constexpr uint8_t kDockTargetComponent = 25; // MAV_COMP_ID_USER1, reserved for QGC's simulation target stream.
constexpr float kPathSampleDistance = 1.f;
constexpr float kPositionGain = 0.35f;
}

bool DockGuidance::preposition_entry_ready(hrt_abstime now) {
  viztra_target_s target{};
  vehicle_local_position_s local{};
  const uint64_t timeout_us = static_cast<uint64_t>(math::constrain(
      _param_dock_pre_timeout.get(), int32_t{100}, int32_t{5000})) * 1000ULL;
  return _viztra_target_sub.copy(&target) &&
      target.target_id == 1 &&
      target.source_component == kDockTargetComponent &&
      (target.valid_flags & 3U) == 3U && target.timestamp > 0 &&
      now >= target.timestamp && now - target.timestamp <= timeout_us &&
      std::isfinite(target.lat) && std::isfinite(target.lon) &&
      fabs(target.lat) <= 90.0 && fabs(target.lon) <= 180.0 &&
      std::isfinite(target.alt_msl) &&
      Vector3f(target.vn, target.ve, target.vd).isAllFinite() &&
      _local_pos_sub.copy(&local) && local.xy_valid && local.z_valid &&
      local.v_xy_valid && local.v_z_valid && local.xy_global && local.z_global &&
      local.timestamp > 0 && now >= local.timestamp && now - local.timestamp < 200_ms &&
      std::isfinite(local.ref_lat) && std::isfinite(local.ref_lon) &&
      std::isfinite(local.ref_alt);
}

void DockGuidance::publish_preposition_status(hrt_abstime now, uint8_t state, bool target_valid) {
  dock_preposition_status_s status{};
  status.timestamp = now;
  status.state = state;
  status.target_valid = target_valid;
  status.reference_valid = target_valid && _pre_reference_valid;
  status.target_source_time_boot_ms = _pre_target.source_time_boot_ms;
  status.target_sequence = _pre_target.sequence;
  status.visual_tracking_s = _pre_visual_since > 0 && now >= _pre_visual_since
      ? (now - _pre_visual_since) * 1e-6f : 0.f;
  status.visual_stable = status.visual_tracking_s >= _param_dock_pre_visual_time.get();
  status.target_age_s = _pre_target.timestamp > 0 && now >= _pre_target.timestamp
      ? (now - _pre_target.timestamp) * 1e-6f : NAN;
  status.prediction_horizon_s = _pre_prediction_horizon_s;
  status.reference_distance_m = status.reference_valid ? (_pre_reference - _pre_target_position).norm() : NAN;
  status.commanded_yaw_rad = _yaw_setpoint_rad;
  _pre_target_position.copyTo(status.target_position_ned);
  if (_pre_target.timestamp > 0) {
    Vector3f(_pre_target.vn, _pre_target.ve, _pre_target.vd).copyTo(status.target_velocity_ned);
  } else {
    Vector3f(NAN, NAN, NAN).copyTo(status.target_velocity_ned);
  }
  _pre_reference.copyTo(status.reference_position_ned);
  vehicle_local_position_s local{};
  if (_local_pos_sub.copy(&local) && local.timestamp > 0 && now >= local.timestamp) {
    status.vehicle_age_s = (now - local.timestamp) * 1e-6f;
  } else {
    status.vehicle_age_s = NAN;
  }
  if (local.timestamp > 0 && local.xy_valid && local.z_valid &&
      local.v_xy_valid && local.v_z_valid &&
      Vector3f(local.x, local.y, local.z).isAllFinite() &&
      Vector3f(local.vx, local.vy, local.vz).isAllFinite()) {
    const Vector3f vehicle_position(local.x, local.y, local.z);
    vehicle_position.copyTo(status.vehicle_position_ned);
    Vector3f(local.vx, local.vy, local.vz).copyTo(status.vehicle_velocity_ned);
    if (status.reference_valid) {
      (_pre_reference - vehicle_position).copyTo(status.reference_error_ned);
    } else {
      Vector3f(NAN, NAN, NAN).copyTo(status.reference_error_ned);
    }
  } else {
    Vector3f(NAN, NAN, NAN).copyTo(status.vehicle_position_ned);
    Vector3f(NAN, NAN, NAN).copyTo(status.vehicle_velocity_ned);
    Vector3f(NAN, NAN, NAN).copyTo(status.reference_error_ned);
  }
  _pre_requested_velocity.copyTo(status.requested_velocity_ned);
  _velocity_setpoint_ned.copyTo(status.commanded_velocity_ned);
  _preposition_pub.publish(status);
}

bool DockGuidance::update_rear_reference(hrt_abstime now, const vehicle_local_position_s &local) {
  _pre_reference_valid = false;
  _pre_prediction_horizon_s = NAN;
  if (!local.xy_global || !local.z_global || !std::isfinite(local.ref_lat) ||
      !std::isfinite(local.ref_lon) || !std::isfinite(local.ref_alt)) {
    return false;
  }
  if (_pre_reference_timestamp != local.ref_timestamp) {
    _pre_reference_timestamp = local.ref_timestamp;
    _pre_path_count = 0;
    _pre_last_source_ms = 0;
    _pre_last_direction.zero();
    _pre_target = {};
  }

  MapProjection projection(local.ref_lat, local.ref_lon, local.ref_timestamp);
  viztra_target_s incoming{};
  if (_viztra_target_sub.update(&incoming) &&
      incoming.target_id == 1 && incoming.source_component == kDockTargetComponent &&
      (incoming.valid_flags & 3U) == 3U && incoming.timestamp <= now &&
      std::isfinite(incoming.lat) && std::isfinite(incoming.lon) &&
      fabs(incoming.lat) <= 90.0 && fabs(incoming.lon) <= 180.0 &&
      std::isfinite(incoming.alt_msl) &&
      Vector3f(incoming.vn, incoming.ve, incoming.vd).isAllFinite() &&
      (_pre_last_source_ms == 0 || incoming.source_time_boot_ms > _pre_last_source_ms)) {
    Vector3f target_position{};
    projection.project(incoming.lat, incoming.lon, target_position(0), target_position(1));
    target_position(2) = local.ref_alt - incoming.alt_msl;
    const bool discontinuity = _pre_path_count > 0 && target_position.isAllFinite() &&
        (target_position - _pre_path[_pre_path_count - 1]).norm() >= 200.f;
    if (discontinuity) {
      // Do not bridge a target teleport with the old path or keep rejecting the new path forever.
      _pre_path_count = 0;
      _pre_last_source_ms = 0;
      _pre_last_direction.zero();
      _pre_target = {};
    }
    if (target_position.isAllFinite() && !discontinuity) {
      _pre_target = incoming;
      _pre_last_source_ms = incoming.source_time_boot_ms;
      const Vector3f velocity(incoming.vn, incoming.ve, incoming.vd);
      if (velocity.norm() > 0.5f) { _pre_last_direction = velocity.normalized(); }
      if (_pre_path_count == 0 ||
          (target_position - _pre_path[_pre_path_count - 1]).norm() >= kPathSampleDistance) {
        if (_pre_path_count == PRE_PATH_CAPACITY) {
          for (int i = 1; i < PRE_PATH_CAPACITY; ++i) { _pre_path[i - 1] = _pre_path[i]; }
          --_pre_path_count;
        }
        _pre_path[_pre_path_count++] = target_position;
      }
    }
  }

  const uint64_t timeout_us = static_cast<uint64_t>(math::constrain(
      _param_dock_pre_timeout.get(), int32_t{100}, int32_t{5000})) * 1000ULL;
  const bool target_fresh = _pre_target.timestamp > 0 && now >= _pre_target.timestamp &&
      now - _pre_target.timestamp <= timeout_us && _pre_path_count > 0;
  if (!target_fresh) { return false; }

  const float age = (now - _pre_target.timestamp) * 1e-6f;
  const float horizon = math::min(age + 0.25f,
      math::constrain(_param_dock_pre_predict_time.get(), 0.f, 3.f));
  _pre_prediction_horizon_s = horizon;
  const Vector3f velocity(_pre_target.vn, _pre_target.ve, _pre_target.vd);
  _pre_target_position = _pre_path[_pre_path_count - 1] + velocity * horizon;

  const float rear_distance = math::constrain(_param_dock_pre_distance.get(), 1.f, 1000.f);
  float remaining = rear_distance;
  Vector3f cursor = _pre_target_position;
  bool reference_ready = false;
  for (int i = _pre_path_count - 1; i >= 0; --i) {
    const Vector3f segment = cursor - _pre_path[i];
    const float length = segment.norm();
    if (length > 0.001f) {
      if (length >= remaining) {
        _pre_reference = cursor - segment * (remaining / length);
        reference_ready = true;
        break;
      }
      remaining -= length;
      cursor = _pre_path[i];
    }
  }
  if (!reference_ready && _pre_last_direction.norm() > 0.5f) {
    _pre_reference = cursor - _pre_last_direction * remaining;
    reference_ready = true;
  }
  if (!reference_ready) {
    // A stationary target has no track; use the approach line rather than its nose yaw.
    const Vector2f approach(_pre_target_position(0) - local.x,
                            _pre_target_position(1) - local.y);
    const float approach_distance = approach.norm();
    if (approach_distance > 1.f) {
      _pre_reference = _pre_target_position - Vector3f(approach(0), approach(1), 0.f) *
          (rear_distance / approach_distance);
      reference_ready = true;
    }
  }
  if (reference_ready) {
    _pre_reference_valid = true;
    _pre_reference(2) -= math::constrain(_param_dock_pre_z_offset.get(), -100.f, 100.f);
  }
  return true;
}

void DockGuidance::point_camera_to_preposition_target(
    hrt_abstime now, const vehicle_local_position_s &local, float dt_s) {
  if (!_camera_servo_armed || _param_dock_cam_pitch_variable.get() == 0 ||
      _pre_visual_since != 0) {
    return;
  }

  vehicle_attitude_s attitude{};
  if (!_att_sub.copy(&attitude) || attitude.timestamp == 0 ||
      now < attitude.timestamp || now - attitude.timestamp > 200_ms) {
    return;
  }

  const Vector3f target_ned = _pre_target_position - Vector3f(local.x, local.y, local.z);
  const Quatf q(attitude.q);
  if (!target_ned.isAllFinite() || target_ned.norm() < 1.f || !q.isAllFinite()) {
    return;
  }

  // The external target stream may point the gimbal before Viztra acquires it.
  // This only changes the camera direction; visual handoff still requires a
  // fresh valid Viztra frame inside the sensor's own range and FOV.
  const Vector3f target_body = Dcmf(q).transpose() * target_ned;
  const float yaw = math::radians(_param_dock_cam_yaw.get());
  const float forward = cosf(yaw) * target_body(0) + sinf(yaw) * target_body(1);
  if (forward <= 1.f) {
    return;
  }

  const float desired_pitch = math::degrees(atan2f(-target_body(2), forward));
  if (!std::isfinite(desired_pitch)) {
    return;
  }

  const float minimum_pitch = math::min(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float maximum_pitch = math::max(_param_dock_cam_pitch_min.get(),
                                        _param_dock_cam_pitch_max.get());
  const float max_step = math::constrain(_param_dock_cam_pitch_rate.get(), 0.1f, 30.f) * dt_s;
  _camera_mount_pitch_deg = math::constrain(_camera_mount_pitch_deg + math::constrain(
      desired_pitch - _camera_mount_pitch_deg, -max_step, max_step), minimum_pitch, maximum_pitch);
  publish_camera_pitch_servo(now);
}

void DockGuidance::run_preposition(hrt_abstime now) {
  _course_yaw_active = false;
  _pre_requested_velocity.zero();
  _pre_prediction_horizon_s = NAN;
  _pre_reference_valid = false;
  if (_mode_fault_latched) {
    publish_safe_velocity_setpoint(now);
    publish_preposition_status(now, 6, false);
    return;
  }

  const auto finish_visual_handoff = [&](hrt_abstime sample, bool target_fresh) {
    _preposition_active = false;
    _preposition_handoff_pending = true;
    _last_sample_us = 0;
    _last_observation_us = sample;
    _vision_degraded = false;
    set_stage(Stage::ALIGN);
    publish_preposition_status(now, 4, target_fresh);
  };
  hrt_abstime visual_handoff_sample = 0;
  const uint64_t observation_timeout_us = static_cast<uint64_t>(math::constrain(
      _param_dock_obs_tout.get(), int32_t{20}, int32_t{5000})) * 1000ULL;
  viztra_observation_s observation{};
  if (_viztra_sub.update(&observation)) {
    const hrt_abstime sample = observation.timestamp_sample > 0
        ? observation.timestamp_sample : observation.timestamp;
    const bool fresh = sample > _pre_last_visual_sample && sample <= now &&
        now - sample <= observation_timeout_us;
    _pre_last_visual_sample = math::max(_pre_last_visual_sample, sample);
    const bool good = fresh && observation.valid &&
        std::isfinite(observation.offset_x) && std::isfinite(observation.offset_y);

    if (good) {
      if (_pre_visual_since == 0 || sample - _last_valid_observation_us > observation_timeout_us) {
        _pre_visual_since = sample;
      }
      const float dt = _last_valid_observation_us > 0 && sample > _last_valid_observation_us
          ? math::constrain((sample - _last_valid_observation_us) * 1e-6f, 0.002f, 0.1f)
          : LOOP_DT;
      _last_valid_observation_us = sample;
      float pitch_error = observation.offset_y;
      if (!observation.offsets_are_angles) {
        const auto angle_from_pixel = [](float pixel, float resolution, float fov_deg) {
          const float pixels = math::constrain(resolution, 100.f, 8000.f);
          const float fov = math::radians(math::constrain(fov_deg, 10.f, 80.f));
          return math::degrees(atanf(2.f * math::constrain(pixel, -pixels, pixels) /
                                     pixels * tanf(fov * 0.5f)));
        };
        const float yaw = angle_from_pixel(observation.offset_x,
            _param_dock_res_x.get(), _param_dock_fov_x.get());
        const float pitch = angle_from_pixel(observation.offset_y,
            _param_dock_res_y.get(), _param_dock_fov_y.get());
        const Vector3f los_camera(1.f, tanf(math::radians(yaw)), -tanf(math::radians(pitch)));
        pitch_error = -math::degrees(atanf(los_camera(2) /
            Vector2f(los_camera(0), los_camera(1)).norm()));
      }
      if (std::isfinite(pitch_error)) {
        update_camera_pitch_servo(pitch_error, dt, sample);
      }
      if (sample >= _pre_visual_since &&
          sample - _pre_visual_since >= static_cast<uint64_t>(
              math::constrain(_param_dock_pre_visual_time.get(), 0.05f, 5.f) * 1e6f)) {
        if (!_param_dock_pre_rear.get()) {
          const uint64_t target_timeout_us = static_cast<uint64_t>(math::constrain(
              _param_dock_pre_timeout.get(), int32_t{100}, int32_t{5000})) * 1000ULL;
          const bool target_fresh = _pre_target.timestamp > 0 && now >= _pre_target.timestamp &&
              now - _pre_target.timestamp <= target_timeout_us && _pre_path_count > 0;
          finish_visual_handoff(sample, target_fresh);
          return;
        }
        visual_handoff_sample = sample;
      }
    } else {
      _pre_visual_since = 0;
    }
  } else if (_last_valid_observation_us > 0 && now - _last_valid_observation_us > observation_timeout_us) {
    _pre_visual_since = 0;
  }

  vehicle_local_position_s local{};
  if (!_local_pos_sub.copy(&local) || !local.xy_valid || !local.z_valid ||
      !local.v_xy_valid || !local.v_z_valid || !local.xy_global || !local.z_global ||
      local.timestamp == 0 || now < local.timestamp || now - local.timestamp > 200_ms ||
      !Vector3f(local.x, local.y, local.z).isAllFinite() ||
      !Vector3f(local.vx, local.vy, local.vz).isAllFinite() ||
      !std::isfinite(local.ref_lat) || !std::isfinite(local.ref_lon) ||
      !std::isfinite(local.ref_alt)) {
    invalidate_setpoint(now, "preposition local position invalid");
    publish_preposition_status(now, 5, false);
    return;
  }

  const bool target_fresh = update_rear_reference(now, local);

  if (visual_handoff_sample && target_fresh && _pre_reference_valid && !_startup_climb) {
    const float rear_distance = math::constrain(_param_dock_pre_distance.get(), 1.f, 1000.f);
    const float handoff_radius = math::constrain(0.25f * rear_distance, 2.f, 25.f);
    const Vector3f reference_error = _pre_reference - Vector3f(local.x, local.y, local.z);
    if (reference_error.norm() <= handoff_radius) {
      finish_visual_handoff(visual_handoff_sample, true);
      return;
    }
  }

  const float dt = _pre_last_control_us > 0 && now > _pre_last_control_us
      ? math::constrain((now - _pre_last_control_us) * 1e-6f, 0.002f, 0.1f) : LOOP_DT;
  _pre_last_control_us = now;
  const auto limits = dynamic_limits();
  Vector3f desired_velocity{};
  uint8_t state = 1;

  if (target_fresh) {
    _pre_stale_since = 0;
    point_camera_to_preposition_target(now, local, dt);
    if (_pre_reference_valid) {
      const Vector3f velocity(_pre_target.vn, _pre_target.ve, _pre_target.vd);
      if (_startup_climb && std::isfinite(_startup_target_z)) {
        if (local.z <= _startup_target_z + 0.5f) {
          _startup_climb = false;
          _startup_height_hold = false;
          _startup_height_reached = true;
        } else {
          _pre_reference(2) = math::min(_pre_reference(2), _startup_target_z);
        }
      }
      const Vector3f error = _pre_reference - Vector3f(local.x, local.y, local.z);
      const float distance = error.norm();
      desired_velocity = velocity;
      if (distance > .01f) {
        dock_motion::RelativeContext c{};
        c.limits = limits;
        c.deceleration = math::constrain(_param_dock_dec_max.get(), .1f, 20.f);
        c.up = math::min(math::constrain(_param_dock_acc_max.get(), .1f, 20.f),
            math::constrain(_param_mpc_acc_up.get(), .1f, 20.f));
        c.down = math::min(c.deceleration,
            math::constrain(_param_mpc_acc_down.get(), .1f, 20.f));
        c.response = math::constrain(_param_dock_pred_response.get(), .05f, 2.f);
        const float speed = math::min(kPositionGain * distance,
            dock_motion::rear_stop_speed(distance,
                (Vector3f(local.vx, local.vy, local.vz) - velocity).norm(), c));
        desired_velocity += error * (speed / distance);
      }
      state = 2;
      const Vector2f horizontal_to_target(_pre_target_position(0) - local.x,
                                          _pre_target_position(1) - local.y);
      if (horizontal_to_target.norm() > 1.f) {
        const float target_yaw = atan2f(horizontal_to_target(1), horizontal_to_target(0));
        if (!std::isfinite(_yaw_setpoint_rad)) {
          vehicle_attitude_s attitude{};
          _yaw_setpoint_rad = _att_sub.copy(&attitude)
              ? Eulerf(Quatf(attitude.q)).psi() : target_yaw;
        }
        const float yaw_step = math::radians(_param_dock_yaw_rate.get()) * dt;
        _yaw_setpoint_rad = matrix::wrap_pi(_yaw_setpoint_rad + math::constrain(
            matrix::wrap_pi(target_yaw - _yaw_setpoint_rad), -yaw_step, yaw_step));
      }
    }
  } else {
    if (_pre_stale_since == 0) {
      _pre_stale_since = now;
    }
    state = 3;
  }

  _pre_requested_velocity = desired_velocity;
  const Vector3f previous_velocity = _velocity_setpoint_ned;
  _velocity_setpoint_ned = desired_velocity;
  constrain_velocity_setpoint(_velocity_setpoint_ned);
  if (_velocity_setpoint_ned.norm() > limits.total && limits.total > 0.f) {
    _velocity_setpoint_ned *= limits.total / _velocity_setpoint_ned.norm();
  }
  const Vector3f requested_acceleration = (_velocity_setpoint_ned - previous_velocity) / dt;
  const bool braking = _velocity_setpoint_ned.norm() < previous_velocity.norm();
  const float acceleration_limit = math::constrain(braking ? _param_dock_dec_max.get()
      : _param_dock_acc_max.get(), 0.1f, 20.f);
  Vector3f limited_acceleration = requested_acceleration;
  if (limited_acceleration.norm() > acceleration_limit) {
    limited_acceleration *= acceleration_limit / limited_acceleration.norm();
  }
  const float turn_radius = math::constrain(_param_dock_pre_radius.get(), 0.f, 2000.f);
  limited_acceleration = dock_motion::limit_preposition_turn_acceleration(
      previous_velocity, limited_acceleration, turn_radius);
  const float max_acceleration_change = math::constrain(_param_dock_dynamic_jerk.get(), 0.1f, 100.f) * dt;
  const Vector3f acceleration_change = limited_acceleration - _pre_command_acceleration;
  _pre_command_acceleration += acceleration_change.norm() > max_acceleration_change
      ? acceleration_change * (max_acceleration_change / acceleration_change.norm())
      : acceleration_change;
  // A speed reduction can lower the allowed lateral acceleration before the
  // jerk ramp reaches it. Keep the actual command inside the curvature bound.
  _pre_command_acceleration = dock_motion::limit_preposition_turn_acceleration(
      previous_velocity, _pre_command_acceleration, turn_radius);
  _velocity_setpoint_ned = previous_velocity + _pre_command_acceleration * dt;
  constrain_velocity_setpoint(_velocity_setpoint_ned);
  if (_velocity_setpoint_ned.norm() > limits.total && limits.total > 0.f) {
    _velocity_setpoint_ned *= limits.total / _velocity_setpoint_ned.norm();
  }
  _commanded_speed_ms = _velocity_setpoint_ned.norm();
  _setpoint_valid = true;
  _vision_degraded = false;
  publish_guidance_status(now, true, state == 3);
  publish_preposition_status(now, state, target_fresh);
  publish_velocity_setpoint(now, _velocity_setpoint_ned, _yaw_setpoint_rad);

  if (state == 3 && _pre_stale_since > 0 && _velocity_setpoint_ned.norm() < 0.3f &&
      now - _pre_stale_since > 500_ms) {
    invalidate_setpoint(now, "preposition target stream lost");
  }
}
