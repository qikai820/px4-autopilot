// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include "PredictiveCorrection.hpp"
#include "RelativeMotion.hpp"
#include "RelativeResponse.hpp"

namespace dock_motion {
struct RelativeContext {
  Vector3f relative{}, relative_rate{}, own_velocity{}, previous{}, previous_acceleration{};
  Vector3f response_bias{};
  Vector3f drag_coefficient{};
  Vector3f linear_drag{};
  float thrust_acceleration{0.f}, horizontal_force_margin{0.f};
  float response_uncertainty{0.f};
  bool response_valid{false};
  Limits limits{};
  Vector3f ray{}, ray_rate{}, image_velocity{};
  matrix::Dcmf camera_to_ned{};
  float dt{.02f}, acceleration{8.f}, deceleration{3.f}, jerk{8.f};
  float up{4.f}, down{3.f}, response{.5f}, horizon{5.f};
  float half_x{.5f}, half_y{.4f}, soft{.65f}, angle{.261799f};
  float tolerance{.5f};
};

struct RelativeTrajectory {
  Vector3f desired{}, end_error{};
  VelocityPlan first{};
  ClosestApproach closest{};
  bool valid{false};
};

inline VelocityPlan relative_step(const RelativeContext &c, const Vector3f &desired,
    const Vector3f &previous, const Vector3f &acceleration, float dt, bool fov) {
  // Lowering the acceleration budget must not itself make jerk infeasible.
  const float requested = desired.norm() < previous.norm() ? c.deceleration : c.acceleration;
  const float budget = max(requested, min(c.acceleration, acceleration.norm() - c.jerk * dt));
  return plan_velocity(desired, previous, acceleration, c.limits, dt, budget, c.jerk,
      c.ray, c.ray_rate, c.camera_to_ned, max(.2f, c.relative.norm()),
      c.half_x, c.half_y, c.soft, false, false, c.up, c.down, fov);
}

// Predict the aircraft response to the actual velocity governor, including its
// ramp and jerk. Relative rate is measured at the start; subtract only the
// additional ego displacement caused by changing our velocity. No target
// absolute velocity input, motion classification or velocity matching phase.
inline RelativeTrajectory relative_rollout(const RelativeContext &c, const Vector3f &desired, bool frozen = false) {
  RelativeTrajectory result;
  if (!c.relative.isAllFinite() || !c.relative_rate.isAllFinite() ||
      !c.own_velocity.isAllFinite() || !desired.isAllFinite() ||
      !std::isfinite(c.response + c.horizon + c.dt) || c.response < .05f ||
      c.horizon < .2f || c.horizon > 10.f || c.dt < .002f || c.dt > .1f) { return result; }
  if (c.response_valid && (!c.response_bias.isAllFinite() || !c.drag_coefficient.isAllFinite() || !c.linear_drag.isAllFinite() ||
      !std::isfinite(c.thrust_acceleration + c.horizontal_force_margin) ||
      c.thrust_acceleration <= .1f || c.horizontal_force_margin < 0.f ||
      c.horizontal_force_margin > c.thrust_acceleration || c.drag_coefficient.min() < 0.f || c.linear_drag.min() < 0.f)) { return result; }
  result.desired = desired;
  if (frozen) { result.first.velocity = desired; }
  else { result.first = relative_step(c, desired, c.previous, c.previous_acceleration, c.dt, true); }
  if (!result.first.velocity.isAllFinite() || !result.first.acceleration.isAllFinite()) { return result; }
  Vector3f relative = c.relative, actual = c.own_velocity;
  Vector3f command = result.first.velocity, acceleration = result.first.acceleration;
  float best = relative.norm_squared(), elapsed = 0.f;
  // 100 ms maximum future step; bounded 101 segments, no allocations.
  const int steps = static_cast<int>(ceilf((c.horizon - c.dt) / .1f));
  const float step = (c.horizon - c.dt) / max(1, steps);
  const float decay = expf(-step / c.response);
  for (int i = 0; i <= steps; ++i) {
    const float dt = i == 0 ? c.dt : step;
    const float e = i == 0 ? expf(-dt / c.response) : decay;
    if (i && !frozen) {
      const auto plan = relative_step(c, desired, command, acceleration, dt, false);
      command = plan.velocity;
      acceleration = plan.acceleration;
    }
    const Vector3f response_command = command + c.response_bias;
    Vector3f displacement = (response_command - c.own_velocity - c.relative_rate) * dt +
        (actual - response_command) * (c.response * (1.f - e));
    const Vector3f response_next = response_command + (actual - response_command) * e;
    const Vector3f response_accel = (response_next - actual) / dt;
    Vector3f limited_accel = c.response_valid
        ? constrain_acceleration(response_accel, c.acceleration, c.up, c.down) : response_accel;
    // Reproduce the position controller's vertical-first thrust allocation,
    // including the configured horizontal reservation. Clipping command jerk
    // alone does not make these future aircraft accelerations achievable.
    if (c.response_valid && c.thrust_acceleration > .1f) {
      const Vector3f drag = actual.emult(c.linear_drag) + Vector3f(c.drag_coefficient(0) * actual(0) * fabsf(actual(0)),
          c.drag_coefficient(1) * actual(1) * fabsf(actual(1)),
          c.drag_coefficient(2) * actual(2) * fabsf(actual(2)));
      Vector3f force = limited_accel + drag - Vector3f(0.f, 0.f, 9.80665f);
      const float xy = hypotf(force(0), force(1));
      const float reserved = min(xy, c.horizontal_force_margin);
      force(2) = clamp(force(2), -sqrtf(max(0.f, c.thrust_acceleration * c.thrust_acceleration - reserved * reserved)), 0.f);
      const float xy_limit = sqrtf(max(0.f, c.thrust_acceleration * c.thrust_acceleration - force(2) * force(2)));
      if (xy > xy_limit && xy > .001f) { force(0) *= xy_limit / xy; force(1) *= xy_limit / xy; }
      limited_accel = force - drag + Vector3f(0.f, 0.f, 9.80665f);
    }
    if ((response_accel - limited_accel).norm() > .001f) {
      displacement = (actual - c.own_velocity - c.relative_rate) * dt + limited_accel * (.5f * dt * dt);
    }
    const float fraction = clamp((relative * displacement) / max(1e-12f, displacement.norm_squared()), 0.f, 1.f);
    const Vector3f miss = relative - displacement * fraction;
    if (miss.norm_squared() < best) {
      best = miss.norm_squared();
      result.closest.miss = miss;
      result.closest.time_s = elapsed + fraction * dt;
      result.closest.relative_speed_m_s = displacement.norm() / dt;
      if (displacement.norm() > .0001f) { result.closest.approach_axis = displacement.normalized(); }
    }
    relative -= displacement;
    actual += limited_accel * dt;
    elapsed += dt;
  }
  result.end_error = relative;
  result.valid = relative.isAllFinite();
  result.closest.valid = result.valid && std::isfinite(result.closest.time_s) &&
      result.closest.time_s > .01f && result.closest.time_s < c.horizon - .01f &&
      result.closest.relative_speed_m_s > .5f;
  return result;
}

inline bool better_relative_trajectory(const RelativeTrajectory &candidate,
    const RelativeTrajectory &best, float tolerance, float soft) {
  if (!candidate.valid) { return false; }
  if (!best.valid) { return true; }
  const float risk = max(0.f, candidate.first.predicted_fov - soft);
  const float best_risk = max(0.f, best.first.predicted_fov - soft);
  if (fabsf(risk - best_risk) > .001f) { return risk < best_risk; }
  const bool hit = candidate.closest.valid && candidate.closest.miss.norm() <= tolerance;
  const bool best_hit = best.closest.valid && best.closest.miss.norm() <= tolerance;
  if (hit != best_hit) { return hit; }
  // Once inside the design tolerance prefer the earliest feasible arrival.
  if (hit) { return candidate.closest.time_s < best.closest.time_s; }
  const float miss = candidate.closest.valid ? candidate.closest.miss.norm_squared() : candidate.end_error.norm_squared();
  const float best_miss = best.closest.valid ? best.closest.miss.norm_squared() : best.end_error.norm_squared();
  return miss + .0001f < best_miss;
}

inline RelativeTrajectory plan_relative_intercept(const RelativeContext &c) {
  const Vector3f ray = c.relative.normalized();
  // Use the simultaneous 3-D velocity envelope already on the aircraft.
  // Vertical speed is directed through the encounter, not towards a stop at
  // target altitude. There is no independent sqrt(2*a*height) brake here.
  const float travel = max(.2f, c.relative.norm() / max(.5f, constrained(ray * c.limits.total, c.limits).norm()));
  const float lookahead = min(travel, c.horizon);
  const Vector3f lead = c.own_velocity +
      (c.relative + c.relative_rate * lookahead) / max(.1f, lookahead - c.response);
  Vector3f desired = constrained(lead, c.limits);
  if (!desired.isAllFinite()) { return {}; }
  RelativeTrajectory best = relative_rollout(c, desired);
  const auto evaluate = [&](const Vector3f &value) {
    const auto candidate = relative_rollout(c, constrained(value, c.limits));
    if (better_relative_trajectory(candidate, best, c.tolerance, c.soft)) { best = candidate; }
  };
  evaluate(desired + c.image_velocity);
  evaluate(ray * c.limits.total);
  evaluate(c.previous); // maintaining speed can free acceleration for steering
  Vector3f vertical = Vector3f(0.f, 0.f, 1.f) - ray * ray(2);
  if (vertical.norm() < .1f) { vertical = Vector3f(1.f, 0.f, 0.f) - ray * ray(0); }
  vertical.normalize();
  const Vector3f lateral = ray.cross(vertical);
  const float radius = min(c.limits.total * tanf(clamp(c.angle, .017453f, .523599f)),
      c.acceleration * c.horizon);
  // Bounded coordinate refinement. Every candidate is rolled through the
  // governor. Receding-horizon replanning handles target maneuvers.
  for (int level = 0; level < 3; ++level) {
    const Vector3f center = best.desired;
    const float scale = radius / (1 << level);
    evaluate(center + vertical * scale);
    evaluate(center - vertical * scale);
    evaluate(center + lateral * scale);
    evaluate(center - lateral * scale);
  }
  return best;
}

// Freeze only a command that is already being published. Steering rollout
// success is never sufficient evidence to permit blind flight.
inline ClosestApproach relative_frozen_approach(const RelativeContext &c, const Vector3f &command) {
  if (c.response_valid) { return relative_rollout(c, command, true).closest; }
  return predict_closest_approach(c.relative, -c.relative_rate, Vector3f{},
      command + c.response_bias - c.own_velocity - c.relative_rate, c.response, c.horizon);
}

inline PointEvidence relative_point_evidence(const RelativeMotion &motion, uint64_t now,
    const ClosestApproach &point, float response_uncertainty = 0.f) {
  PointEvidence result;
  if (!motion.ready(now) || !point.valid || point.relative_speed_m_s <= .5f ||
      !std::isfinite(response_uncertainty) || response_uncertainty < 0.f) { return result; }
  const float horizon = motion.age(now) + point.time_s;
  const float response_error = response_uncertainty * point.time_s;
  result.miss_bound_m = point.miss.norm() + 2.f * motion.sigma(point.approach_axis, horizon, true) + response_error;
  result.time_upper_s = point.time_s + 2.f * motion.sigma(point.approach_axis, horizon, false) /
      point.relative_speed_m_s + response_error / point.relative_speed_m_s;
  result.valid = std::isfinite(result.miss_bound_m + result.time_upper_s);
  return result;
}

inline bool relative_terminal_window(const PointEvidence &point, float observation_age,
    float relative_speed, float pass_distance, float allowed_miss, float tgo_limit,
    float blind_limit, float total_limit) {
  if (!point.valid || !std::isfinite(point.miss_bound_m + point.time_upper_s + observation_age +
      relative_speed + pass_distance + allowed_miss + tgo_limit + blind_limit + total_limit) ||
      observation_age < 0.f || relative_speed <= .5f || point.time_upper_s <= 0.f) { return false; }
  const float clearance = max(0.f, pass_distance) / relative_speed;
  return point.miss_bound_m < allowed_miss && point.time_upper_s <= tgo_limit &&
      point.time_upper_s + clearance + .03f < min(blind_limit - observation_age, total_limit);
}
} // namespace dock_motion
