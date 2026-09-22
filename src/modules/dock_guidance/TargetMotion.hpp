// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace dock_motion {
using matrix::Vector3f;
template <typename T> inline T min(T a, T b) { return a < b ? a : b; }
template <typename T> inline T max(T a, T b) { return a > b ? a : b; }
inline float clamp(float v, float lo, float hi) { return max(lo, min(v, hi)); }

// LandDetector publishes changes immediately, but its unchanged-state heartbeat
// is only 1 Hz plus scheduling jitter. Do not reject a normal 1.01 s interval.
inline bool airborne_state(bool armed, bool landed, uint64_t stamp, uint64_t now) {
  return armed && !landed && stamp != 0 && now >= stamp && now - stamp <= 2000000;
}

inline bool capture_evidence_allowed(bool common_evidence, bool pass_through,
                                     bool already_ready, float range_lower,
                                     float initial_distance) {
  return common_evidence && std::isfinite(range_lower) &&
         ((pass_through && already_ready) || range_lower > initial_distance);
}

inline bool capture_image_evidence(bool hard, float fov, float align, float soft,
                                   float los_rate, float acquisition_rate,
                                   bool pass_through_ready) {
  if (hard || !std::isfinite(fov + align + soft + los_rate + acquisition_rate)) {
    return false;
  }
  // ALIGN is the acquisition threshold. Once pass-through is confirmed, keep
  // the permission until SOFT so small camera/vehicle lag cannot repeatedly
  // switch the target speed between initial and maximum capture. The final
  // planner still predicts FOV from measured ray rate and can brake smoothly.
  return pass_through_ready ? fov < soft
                            : fov <= align && los_rate < acquisition_rate;
}

// A reliable bearing/range can authorize bounded LOS capture without claiming
// that target velocity is known. Keep the initial low-speed limit until image
// alignment and geometric evidence have remained continuous for confirmation.
class CaptureSchedule {
public:
  void reset() { _since = 0; _ready = false; }
  bool ready() const { return _ready; }
  float update(uint64_t now, bool evidence, float confirmation, float initial_limit,
               float pursuit_limit, bool pass_through, float range_lower, float closing_bias,
               float stop_distance, float deceleration, float jerk) {
    const float low = clamp(initial_limit, 0.f, 10.f);
    if (!evidence || low <= 0.f || !std::isfinite(range_lower + closing_bias)) {
      reset();
      return low;
    }
    if (!_since || now < _since) { _since = now; _ready = false; }
    _ready = now - _since >= uint64_t(clamp(confirmation, .1f, 5.f) * 1e6f);
    if (!_ready) { return low; }
    if (pass_through) {
      // A terminal pass-through intentionally does not reserve stopping
      // distance before the target plane. The caller still enforces vehicle
      // speed/axis, acceleration, jerk and predicted-FOV constraints.
      return max(low, clamp(pursuit_limit, 0.f, 60.f));
    }
    const float decel = max(.1f, deceleration);
    const float reaction = .5f + decel / max(.1f, jerk);
    const float distance = max(0.f, range_lower - max(2.f, stop_distance));
    // Reserve reaction/jerk reversal distance and uncertainty in radial motion.
    // This is a conservative capture envelope, not terminal-pass authorization.
    const float speed = sqrtf(decel * decel * reaction * reaction + 2.f * decel * distance)
                        - decel * reaction - max(0.f, closing_bias);
    return max(low, min(clamp(pursuit_limit, 0.f, 60.f), max(0.f, speed)));
  }
private:
  uint64_t _since{0};
  bool _ready{false};
};

// NED: positive remaining height requires negative vertical velocity. Never
// request descent while acquiring altitude. FOV may pause, not reverse, climb.
inline float startup_climb_velocity(float remaining, float up_limit, float decel, float fov_scale) {
  if (!std::isfinite(remaining + up_limit + decel + fov_scale)) {
    return 0.f;
  }
  const float height = max(0.f, remaining);
  return -min(min(max(0.f, up_limit), 3.f),
              min(height, sqrtf(2.f * max(.1f, decel) * height))) * clamp(fov_scale, 0.f, 1.f);
}

inline float height_capture_velocity(float remaining, float actual_vz, float up_limit,
                                     float decel, float jerk, float fov_scale) {
  // Reserve distance for measured climb speed, controller delay and acceleration
  // reversal before applying the stopping envelope.
  const float reaction = .25f + max(.1f, decel) / max(.1f, jerk);
  const float available = remaining - max(0.f, -actual_vz) * reaction;
  return startup_climb_velocity(available, up_limit, decel, fov_scale);
}

// A bounding box measures optical depth, not slant range. This model requires
// calibrated projected physical dimensions; unknown size must stay disabled.
inline bool box_range(float width_px, float height_px, float width_m, float height_m, float fx,
                      float fy, float forward_cos, float size_error, float &range, float &sigma) {
  if (!std::isfinite(width_px + height_px + width_m + height_m + fx + fy + forward_cos +
                     size_error) ||
      fx <= 0.f || fy <= 0.f || forward_cos < .2f) {
    return false;
  }
  // Retain small boxes across 5/6-pixel quantization transitions. Their large
  // range uncertainty is retained below; this does not authorize radial closure.
  const bool w = width_m > 0.f && width_px >= 4.f && width_px < 65535.f;
  const bool h = height_m > 0.f && height_px >= 4.f && height_px < 65535.f;
  if (!w && !h) {
    return false;
  }
  const float rw = w ? fx * width_m / (width_px * forward_cos) : 0.f;
  const float rh = h ? fy * height_m / (height_px * forward_cos) : 0.f;
  if (w && h && fabsf(rw - rh) > .25f * max(rw, rh)) {
    return false;
  }
  range = w && h ? .5f * (rw + rh) : (w ? rw : rh);
  const float pixels = w && h ? min(width_px, height_px) : (w ? width_px : height_px);
  sigma = range * max(clamp(size_error, .02f, .5f), 2.f / pixels);
  return std::isfinite(range) && range > .2f && range < 2000.f;
}

// Constant-velocity target filter in the local NED frame. Own position is at
// exposure time. Full covariance retains position/velocity correlations.
// No metric initialization from bearing alone: a range prior is not evidence.
class TargetMotion {
public:
  void reset() { const uint32_t count = _resets + 1; *this = TargetMotion{}; _resets = count; }
  uint32_t resets() const { return _resets; }
  int accepted_updates() const { return _count; }
  uint8_t motion_state() const { return _motion_state; } // 0 unknown, 1 stationary, 2 moving
  bool initialized() const { return _time != 0; }
  Vector3f velocity() const { return Vector3f(_x(3), _x(4), _x(5)); }
  float age(uint64_t now) const {
    return initialized() && now >= _accepted ? (now - _accepted) * 1e-6f : INFINITY;
  }
  float velocity_sigma() const { return sqrtf(max(0.f, _p(3, 3) + _p(4, 4) + _p(5, 5))); }
  bool mature(uint64_t now) const {
    return initialized() && _count >= 8 && now >= _start && now - _start >= 1000000 &&
           age(now) <= .25f;
  }
  float directional_velocity_sigma(const Vector3f &axis) const {
    float variance = 0.f;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        variance += axis(i) * _p(i + 3, j + 3) * axis(j);
      }
    }
    return sqrtf(max(0.f, variance));
  }
  Vector3f following_velocity(uint64_t now, float max_error) const {
    Vector3f result;
    // Evaluate uncertainty in LOS/tangent space, not fixed NED components:
    // uncertain depth otherwise suppresses well-observed lateral motion.
    if (_motion_state != 2 || !initialized() || now < _start || now - _start < 400000 || _count < 5 ||
        age(now) > .25f) {
      return result;
    }
    const Vector3f radial = _last_los * (velocity() * _last_los);
    const Vector3f lateral = velocity() - radial;
    float total_variance = 0.f;
    for (int i = 0; i < 3; ++i) {
      total_variance += _p(i + 3, i + 3);
    }
    const float radial_error = directional_velocity_sigma(_last_los);
    // Shared tangent-plane weight is invariant to NED/camera orientation.
    const float lateral_error = sqrtf(max(0.f, total_variance - radial_error * radial_error));
    const auto weight = [max_error](float error) {
      return clamp((2.f * max_error - error) / max(max_error, .1f), 0.f, 1.f);
    };
    // This is bounded partial following, not a declaration of metric validity.
    // Retain a scale-dependent attenuation without treating uncertain depth as
    // unobservable lateral direction. Full closure still uses valid().
    result = lateral * (weight(lateral_error) / (1.f + _scale_fraction * _scale_fraction));
    if (mature(now)) {
      result += radial * weight(radial_error);
    }
    return result;
  }
  float position_sigma(float horizon = 0.f) const {
    float variance = 0.f;
    for (int i = 0; i < 3; ++i) {
      variance += _p(i, i) + 2.f * horizon * _p(i, i + 3) + horizon * horizon * _p(i + 3, i + 3);
    }
    return max(_scale_floor, sqrtf(max(0.f, variance))) + .5f * _accel * horizon * horizon;
  }
  bool valid(uint64_t now, float max_error) const {
    return mature(now) && now >= _metric_time && now - _metric_time <= 1000000 &&
           velocity_sigma() <= max_error;
  }
  bool fresh_range(uint64_t now) const {
    return mature(now) && now >= _metric_time && now - _metric_time <= 250000;
  }
  Vector3f relative(uint64_t now, const Vector3f &own) const {
    const float dt = now >= _time ? (now - _time) * 1e-6f : 0.f;
    return Vector3f(_x(0), _x(1), _x(2)) + velocity() * dt - own;
  }

  bool update(uint64_t time, const Vector3f &own, const Vector3f &los, float range,
              float range_sigma, float angle_sigma, float accel, float max_speed) {
    const bool metric = std::isfinite(range + range_sigma) && range > .2f && range_sigma > 0.f;
    if (!own.isAllFinite() || !los.isAllFinite() || fabsf(los.norm() - 1.f) > .01f ||
        !std::isfinite(angle_sigma) || (!metric && !initialized()) ||
        angle_sigma <= 0.f || time == 0) {
      return false;
    }
    if (_time && time <= _time) {
      return false;
    }
    if (_time && time - _accepted > 500000) {
      reset();
    }
    if (!metric && (!_time || time < _metric_time || time - _metric_time > 1000000)) {
      return false;
    }
    _accel = clamp(accel, .1f, 30.f);
    // Tangential bearing observations do not need a new box-derived range.
    const float observation_range = metric ? range : relative(time, own).norm();
    const Vector3f measurement = own + los * observation_range;
    if (!_time) {
      for (int i = 0; i < 3; ++i) {
        _x(i) = measurement(i);
        _p(i, i) = range_sigma * range_sigma;
        _p(i + 3, i + 3) = max_speed * max_speed;
      }
      _time = _accepted = _start = time;
      _scale_floor = range_sigma;
      _scale_fraction = range_sigma / range;
      _last_los = los;
      _metric_time = time;
      _count = 1;
      return true;
    }
    // At most 10 Hz: consecutive quantized boxes are strongly correlated.
    if (time - _time < 100000) {
      return false;
    }
    const float dt = (time - _time) * 1e-6f;
    matrix::SquareMatrix<float, 6> f;
    f.setIdentity();
    for (int i = 0; i < 3; ++i) {
      f(i, i + 3) = dt;
    }
    _x = f * _x;
    _p = f * _p * f.transpose();
    for (int i = 0; i < 3; ++i) {
      const float q = _accel * _accel;
      _p(i, i) += q * dt * dt * dt / 3.f;
      _p(i, i + 3) += q * dt * dt / 2.f;
      _p(i + 3, i) += q * dt * dt / 2.f;
      _p(i + 3, i + 3) += q * dt;
    }
    _time = time;
    Vector3f tangent = los % (fabsf(los(2)) < .9f ? Vector3f(0, 0, 1) : Vector3f(0, 1, 0));
    tangent.normalize();
    const Vector3f axes[3] = {los, tangent, los % tangent};
    const float lateral_sigma = max(.1f, observation_range * angle_sigma);
    const float noise[3] = {metric ? range_sigma * range_sigma : 1.f, lateral_sigma * lateral_sigma,
                            lateral_sigma * lateral_sigma};
    // Gate the entire observation before changing any state component.
    for (int j = metric ? 0 : 1; j < 3; ++j) {
      matrix::Vector<float, 6> h;
      for (int i = 0; i < 3; ++i) {
        h(i) = axes[j](i);
      }
      const float residual = axes[j] * measurement - h.dot(_x);
      const float variance = h.dot(_p * h) + noise[j];
      if (residual * residual > 16.f * variance) {
        _count = 0;
        _motion_state = _evidence_candidate = 0;
        _evidence_since = 0;
        return false;
      }
    }
    for (int j = metric ? 0 : 1; j < 3; ++j) {
      matrix::Vector<float, 6> h;
      for (int i = 0; i < 3; ++i) {
        h(i) = axes[j](i);
      }
      const matrix::Vector<float, 6> ph = _p * h;
      const float variance = h.dot(ph) + noise[j];
      const matrix::Vector<float, 6> k = ph / variance;
      _x += k * (axes[j] * measurement - h.dot(_x));
      matrix::SquareMatrix<float, 6> identity;
      identity.setIdentity();
      const matrix::Matrix<float, 6, 1> km(k);
      const matrix::SquareMatrix<float, 6> a = identity - km * h.transpose();
      _p = a * _p * a.transpose() + km * k.transpose() * noise[j]; // Joseph form
    }
    if (!_x.isAllFinite() || !_p.isAllFinite() || velocity().norm() > max_speed) {
      reset();
      return false;
    }
    _accepted = time;
    if (metric) {
      _metric_time = time;
      _scale_floor = range_sigma; // systematic size error cannot be averaged away
      _scale_fraction = range_sigma / range;
    }
    _last_los = los;
    _count = min(_count + 1, 1000);
    const float speed = velocity().norm(), error = velocity_sigma();
    const float motion_error = speed > .1f ? directional_velocity_sigma(velocity() / speed) : error;
    const bool consistent = (velocity() - _evidence_velocity).norm() <= max(2.f, 3.f * _accel * dt);
    const uint8_t candidate = speed + 2.f * error < 3.f ? 1 :
                              (speed - 2.f * motion_error > 2.f && consistent ? 2 : 0);
    _evidence_velocity = velocity();
    if (candidate != _evidence_candidate || !candidate) {
      _evidence_candidate = candidate;
      _evidence_since = time;
      _motion_state = 0;
    } else if (time - _evidence_since >= 1500000) {
      _motion_state = candidate;
    }
    return true;
  }

private:
  matrix::Vector<float, 6> _x{};
  matrix::SquareMatrix<float, 6> _p{};
  uint64_t _time{0}, _accepted{0}, _start{0};
  uint64_t _metric_time{0}, _evidence_since{0};
  uint32_t _resets{0};
  uint8_t _motion_state{0}, _evidence_candidate{0};
  Vector3f _evidence_velocity{};
  int _count{0};
  float _scale_floor{INFINITY}, _accel{8.f};
  float _scale_fraction{1.f};
  Vector3f _last_los{1.f, 0.f, 0.f};
};

enum class PassState { Approaching, Predicted, Uncertain };
// Short-horizon camera retention heuristic, not a guarantee against target
// maneuvers. Current image drift includes actual camera rotation. Only predict
// the additional tilt/translation caused by a change in commanded acceleration.
inline float startup_fov_cost(const Vector3f &ray, const Vector3f &ray_rate,
                             const matrix::Dcmf &camera_to_ned, const Vector3f &delta_accel,
                             float range, float half_x, float half_y) {
  constexpr float horizon = .35f;
  const Vector3f earth_ray = camera_to_ned * ray;
  const Vector3f tilt(delta_accel(1) / 9.81f, -delta_accel(0) / 9.81f, 0.f);
  Vector3f predicted = ray + ray_rate * horizon;
  predicted -= camera_to_ned.transpose() * (tilt % earth_ray) * .5f;
  predicted -= camera_to_ned.transpose() *
               (delta_accel - earth_ray * (earth_ray * delta_accel)) *
               (.5f * horizon * horizon / max(range, 5.f));
  if (!predicted.isAllFinite() || predicted(0) <= .05f) {
    return 100.f;
  }
  return max(fabsf(atan2f(predicted(1), predicted(0))) / max(half_x, .05f),
             fabsf(atan2f(-predicted(2), hypotf(predicted(0), predicted(1)))) / max(half_y, .05f));
}

inline PassState classify_pass(const Vector3f &relative, const Vector3f &direction, float error,
                               float margin, float allowed_miss) {
  if (!relative.isAllFinite() || !direction.isAllFinite() || !std::isfinite(error) || error < 0.f ||
      fabsf(direction.norm() - 1.f) > .01f) {
    return PassState::Uncertain;
  }
  const float along = relative * direction;
  const float across = (relative - direction * along).norm();
  if (across + 2.f * error > allowed_miss) {
    return PassState::Uncertain;
  }
  return along + 2.f * error < -margin ? PassState::Predicted : PassState::Approaching;
}

struct Limits {
  float total, xy, up, down;
};
inline float approach_limit(bool terminal_enabled, float total_limit, float relative_limit) {
  return terminal_enabled ? max(0.f, total_limit) : clamp(relative_limit, 0.f, 60.f);
}
inline bool feasible(const Vector3f &v, const Limits &l) {
  return v.isAllFinite() && v.norm() <= l.total + 1e-4f && hypotf(v(0), v(1)) <= l.xy + 1e-4f &&
         -v(2) <= l.up + 1e-4f && v(2) <= l.down + 1e-4f;
}
inline Vector3f constrained(Vector3f v, const Limits &l) {
  if (!v.isAllFinite()) {
    return Vector3f{};
  }
  float scale = 1.f;
  if (v.norm() > .001f) {
    scale = min(scale, l.total / v.norm());
  }
  const float xy = hypotf(v(0), v(1));
  if (xy > .001f) {
    scale = min(scale, l.xy / xy);
  }
  if (v(2) < -.001f) {
    scale = min(scale, l.up / -v(2));
  }
  if (v(2) > .001f) {
    scale = min(scale, l.down / v(2));
  }
  return v * clamp(scale, 0.f, 1.f);
}
// Convex velocity constraints: reserve the base velocity before adding closure.
inline float closing_budget(const Vector3f &base, const Vector3f &los, const Limits &l,
                            float requested) {
  if (!feasible(base, l) || !los.isAllFinite() || los.norm() < .5f) {
    return 0.f;
  }
  float lo = 0.f, hi = max(requested, 0.f);
  for (int i = 0; i < 24; ++i) {
    const float mid = .5f * (lo + hi);
    if (feasible(base + los * mid, l)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}
struct VelocityPlan {
  Vector3f velocity{}, acceleration{};
  float predicted_fov{0.f};
  bool limits_override{false};
};

// Evaluate exactly the velocity that will be published. Height feedback is
// included before the single jerk step; there is no post-selection Z override.
inline VelocityPlan plan_velocity(const Vector3f &desired_velocity, const Vector3f &previous_velocity,
    const Vector3f &previous_accel, const Limits &limits, float dt, float accel_limit, float jerk,
    const Vector3f &ray, const Vector3f &ray_rate, const matrix::Dcmf &camera_to_ned,
    float range, float half_x, float half_y, float soft, bool height_priority, bool no_descent) {
  VelocityPlan result;
  dt = clamp(dt, .002f, .1f);
  const float jerk_step = max(.1f, jerk) * dt;
  Vector3f desired = (constrained(desired_velocity, limits) - previous_velocity) / .5f;
  if (desired.norm() > accel_limit) { desired *= accel_limit / desired.norm(); }
  const float vertical_step = clamp(desired(2) - previous_accel(2), -.6f * jerk_step, .6f * jerk_step);
  const float horizontal_step = sqrtf(max(0.f, jerk_step * jerk_step - vertical_step * vertical_step));
  float best_score = INFINITY;
  for (int mode = 0; mode < 2; ++mode) {
    Vector3f target = mode == 0 ? desired : -previous_velocity * 2.f;
    if (target.norm() > accel_limit) { target *= accel_limit / target.norm(); }
    Vector3f delta = target - previous_accel;
    if (height_priority) {
      delta(2) = 0.f;
      if (delta.norm() > horizontal_step) { delta *= horizontal_step / delta.norm(); }
      delta(2) = vertical_step;
    } else if (delta.norm() > jerk_step) {
      delta *= jerk_step / delta.norm();
    }
    for (int x = 0; x <= 2; ++x) {
      for (int y = 0; y <= 2; ++y) {
        const Vector3f proposal = previous_accel + Vector3f(.5f * x * delta(0), .5f * y * delta(1), delta(2));
        Vector3f velocity = constrained(previous_velocity + proposal * dt, limits);
        if (no_descent) { velocity(2) = min(velocity(2), 0.f); }
        const Vector3f actual = (velocity - previous_velocity) / dt;
        if (actual.norm() > accel_limit + .002f || (actual - previous_accel).norm() > jerk_step + .002f) {
          continue;
        }
        const float risk = startup_fov_cost(ray, ray_rate, camera_to_ned,
            actual - previous_accel, range, half_x, half_y);
        const float violation = max(0.f, risk - soft);
        const float score = 10000.f * violation * violation +
            (actual - desired).norm_squared() / max(1.f, accel_limit * accel_limit);
        if (score < best_score) {
          best_score = score;
          result.velocity = velocity;
          result.acceleration = actual;
          result.predicted_fov = risk;
        }
      }
    }
  }
  if (!std::isfinite(best_score)) {
    // Newly reduced speed limits or an imminent velocity boundary may make
    // speed and jerk constraints incompatible. Report that override explicitly.
    Vector3f delta = -previous_accel;
    if (delta.norm() > jerk_step) { delta *= jerk_step / delta.norm(); }
    result.velocity = constrained(previous_velocity + (previous_accel + delta) * dt, limits);
    if (no_descent) { result.velocity(2) = min(result.velocity(2), 0.f); }
    result.acceleration = (result.velocity - previous_velocity) / dt;
    result.predicted_fov = startup_fov_cost(ray, ray_rate, camera_to_ned,
        result.acceleration - previous_accel, range, half_x, half_y);
    result.limits_override = true;
  }
  return result;
}
} // namespace dock_motion
