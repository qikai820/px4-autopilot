// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include "TargetMotion.hpp"

namespace dock_motion {
struct MissCorrection {
  Vector3f velocity{};               // desired command, before the final planner
  Vector3f miss{};                   // NED residual predicted with nominal command
  Vector3f correction{};             // change in desired velocity, not acceleration
  float time_to_plane{NAN};
  bool valid{false};
};

// Constant target velocity and first-order own velocity response to a held
// command. Solve the transverse residual before reaching the approach plane.
// The caller supplies fresh, trusted state and retains FOV/accel/jerk planning.
inline MissCorrection correct_predicted_miss(const Vector3f &relative,
    const Vector3f &own_velocity, const Vector3f &target_velocity,
    const Vector3f &nominal, const Vector3f &approach_axis, float response_time,
    float max_time, float max_angle_rad, float acceleration) {
  MissCorrection result;
  result.velocity = nominal;
  if (!relative.isAllFinite() || !own_velocity.isAllFinite() ||
      !target_velocity.isAllFinite() || !nominal.isAllFinite() ||
      !approach_axis.isAllFinite() || fabsf(approach_axis.norm() - 1.f) > .01f ||
      !std::isfinite(response_time + max_time + max_angle_rad + acceleration) ||
      response_time < .05f || max_time <= 0.f || max_angle_rad <= 0.f || acceleration <= 0.f) {
    return result;
  }
  const Vector3f relative_velocity = own_velocity - target_velocity;
  const float remaining = relative * approach_axis;
  const float closing = relative_velocity * approach_axis;
  const float commanded_closing = (nominal - target_velocity) * approach_axis;
  const float speed = nominal.norm();
  if (remaining <= 0.f || closing <= .5f || speed < .5f ||
      commanded_closing <= .5f) {
    return result; // no turn-back after crossing or correction while receding
  }
  const auto response_integral = [response_time](float time) {
    return clamp(time + response_time * expm1f(-time / response_time), 0.f, time);
  };
  const auto distance = [&](float time) {
    return closing * time + (commanded_closing - closing) * response_integral(time);
  };
  // Estimate arrival with the SAME response model as the residual. A time based
  // only on current speed would invent radial misses during speed changes and
  // could delay correction even when the requested speed reaches the plane soon.
  if (distance(max_time) < remaining) { return result; }
  float time = remaining / closing;
  if (fabsf(commanded_closing - closing) > 1e-5f) {
    float lo = 0.f, hi = max_time;
    // Both endpoint closing speeds are positive, so distance is monotonic.
    for (int i = 0; i < 18; ++i) {
      const float mid = .5f * (lo + hi);
      if (distance(mid) < remaining) { lo = mid; } else { hi = mid; }
    }
    time = .5f * (lo + hi);
  }
  if (!std::isfinite(time) || time <= 0.f) { return result; }
  // Integral of a first-order velocity step, in seconds. The denominator floor
  // and reachable lateral-speed cap avoid the near-zero time-to-plane singularity.
  const float response = response_integral(time);
  result.time_to_plane = time;
  result.miss = relative - relative_velocity * time - (nominal - own_velocity) * response;
  const Vector3f forward = nominal / speed;
  Vector3f correction = result.miss - forward * (result.miss * forward);
  correction /= max(response, .05f);
  const float cap = min(speed * tanf(clamp(max_angle_rad, 0.f, .523599f)), acceleration * time);
  if (correction.norm() > cap && correction.norm() > .001f) {
    correction *= cap / correction.norm();
  }
  // Rotate the requested ground-speed vector without adding terminal braking.
  // Final speed/axis constraints and the existing image planner still apply.
  const Vector3f candidate = (nominal + correction).normalized() * speed;
  if (!candidate.isAllFinite() || (candidate - target_velocity) * approach_axis <= .5f) {
    return result;
  }
  result.velocity = candidate;
  result.correction = candidate - nominal;
  result.valid = true;
  return result;
}
} // namespace dock_motion
