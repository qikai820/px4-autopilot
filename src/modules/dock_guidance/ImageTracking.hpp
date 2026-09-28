// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <matrix/matrix/math.hpp>

namespace dock_motion {

// Camera yaw can rotate the image horizontally, but that rotation does not
// remove a lateral position error relative to a moving target.
inline matrix::Vector3f image_yaw_tangent(const matrix::Vector3f &direction,
                                         const matrix::Vector3f &camera_forward) {
  const matrix::Vector3f down(0.f, 0.f, 1.f);
  matrix::Vector3f tangent = down % camera_forward;
  tangent -= direction * (direction * tangent);
  const float norm = tangent.norm();
  return norm > .01f && tangent.isAllFinite() ? tangent / norm : matrix::Vector3f{};
}

inline matrix::Vector3f image_pointing_error(const matrix::Vector3f &direction,
                                              const matrix::Vector3f &camera_forward,
                                              bool keep_horizontal) {
  matrix::Vector3f error = camera_forward - direction * (direction * camera_forward);
  if (!keep_horizontal) {
    const matrix::Vector3f yaw = image_yaw_tangent(direction, camera_forward);
    error -= yaw * (yaw * error);
  }
  return error;
}

inline matrix::Vector3f image_pitch_assist(const matrix::Vector3f &tracking,
                                            const matrix::Vector3f &direction,
                                            const matrix::Vector3f &camera_right,
                                            float assist) {
  // Use the calibrated camera image-right axis. World-down x camera-forward
  // becomes ill-conditioned when an upward camera points near the zenith.
  matrix::Vector3f horizontal_axis = camera_right - direction * (direction * camera_right);
  const float norm = horizontal_axis.norm();
  if (norm < .01f || !horizontal_axis.isAllFinite()) {
    return tracking * assist;
  }
  horizontal_axis /= norm;
  const matrix::Vector3f horizontal = horizontal_axis * (horizontal_axis * tracking);
  return horizontal + (tracking - horizontal) * assist;
}

inline float pitch_rate_assist(float fov_assist, float image_pitch_rate_deg_s,
                               float servo_rate_deg_s) {
  if (!std::isfinite(image_pitch_rate_deg_s) || !std::isfinite(servo_rate_deg_s) ||
      servo_rate_deg_s <= 0.f) {
    return fov_assist;
  }
  const float shortfall = fabsf(image_pitch_rate_deg_s) - servo_rate_deg_s;
  const float rate_assist = fminf(fmaxf(shortfall / fmaxf(2.f * servo_rate_deg_s, 5.f), 0.f), 1.f);
  return fmaxf(fov_assist, rate_assist);
}

inline matrix::Vector3f limit_image_velocity(const matrix::Vector3f &tracking, float limit) {
  const float speed = tracking.norm();
  return tracking.isAllFinite() && speed > limit ? tracking * (limit / speed)
                                                : (tracking.isAllFinite() ? tracking : matrix::Vector3f{});
}

} // namespace dock_motion
