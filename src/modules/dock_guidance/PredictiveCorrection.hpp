// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include "TargetMotion.hpp"

namespace dock_motion {
struct MissCorrection {
  Vector3f velocity{};               // 最终规划前的期望速度。
  Vector3f miss{};                   // 按原速度指令预测的 NED 穿越残差。
  Vector3f correction{};             // 期望速度的改变量，不是加速度。
  float time_to_plane{NAN};
  bool valid{false};
};

struct ClosestApproach {
  Vector3f miss{}; // Target center minus aircraft at minimum separation.
  Vector3f approach_axis{}; // Relative velocity direction at that instant.
  float time_s{NAN};
  float relative_speed_m_s{NAN};
  bool valid{false};
};

struct PointEvidence {
  float miss_bound_m{INFINITY};
  float time_upper_s{INFINITY};
  bool valid{false};
};

// Along-track position error mostly shifts encounter time; it does not become
// a spatial miss for a target enterable from either side. Check the transverse
// miss separately and retain a conservative time budget for that depth error.
inline PointEvidence assess_closest_approach(const TargetMotion &motion, uint64_t now,
                                             const ClosestApproach &point) {
  PointEvidence result;
  if (!point.valid || !motion.recent_metric(now) ||
      !std::isfinite(point.relative_speed_m_s) || point.relative_speed_m_s <= .5f) {
    return result;
  }
  const float horizon = motion.age(now) + point.time_s;
  const float across_sigma = motion.transverse_position_sigma(point.approach_axis, horizon);
  const float position_sigma = motion.position_sigma(horizon);
  result.miss_bound_m = point.miss.norm() + 2.f * across_sigma;
  result.time_upper_s = point.time_s + 2.f * position_sigma / point.relative_speed_m_s;
  result.valid = std::isfinite(result.miss_bound_m + result.time_upper_s);
  return result;
}

// A target that may be entered from any direction has no privileged crossing
// plane. Predict the minimum 3-D separation under the same first-order
// velocity response used by the steering correction below. Do not report a
// horizon endpoint as a predicted intercept.
inline ClosestApproach predict_closest_approach(const Vector3f &relative,
    const Vector3f &own_velocity, const Vector3f &target_velocity,
    const Vector3f &command, float response_time, float horizon) {
  ClosestApproach result;
  if (!relative.isAllFinite() || !own_velocity.isAllFinite() ||
      !target_velocity.isAllFinite() || !command.isAllFinite() ||
      !std::isfinite(response_time + horizon) || response_time < .05f || horizon <= 0.f) {
    return result;
  }
  const Vector3f initial_relative_velocity = own_velocity - target_velocity;
  const Vector3f command_change = command - own_velocity;
  const auto miss_at = [&](float time) {
    const float response = time + response_time * expm1f(-time / response_time);
    return relative - initial_relative_velocity * time - command_change * response;
  };
  constexpr int samples = 20;
  int minimum = 0;
  float best = relative.norm_squared();
  for (int i = 1; i <= samples; ++i) {
    const float miss_squared = miss_at(horizon * i / samples).norm_squared();
    if (miss_squared < best) {
      best = miss_squared;
      minimum = i;
    }
  }
  if (minimum == 0 || minimum == samples) {
    return result;
  }
  float lo = horizon * (minimum - 1) / samples;
  float hi = horizon * (minimum + 1) / samples;
  for (int i = 0; i < 22; ++i) {
    const float left = lo + (hi - lo) / 3.f;
    const float right = hi - (hi - lo) / 3.f;
    if (miss_at(left).norm_squared() < miss_at(right).norm_squared()) {
      hi = right;
    } else {
      lo = left;
    }
  }
  result.time_s = .5f * (lo + hi);
  result.miss = miss_at(result.time_s);
  const Vector3f relative_velocity = initial_relative_velocity +
      command_change * (1.f - expf(-result.time_s / response_time));
  result.relative_speed_m_s = relative_velocity.norm();
  if (result.relative_speed_m_s > .5f && std::isfinite(result.relative_speed_m_s)) {
    result.approach_axis = relative_velocity / result.relative_speed_m_s;
  }
  result.valid = result.time_s > 0.f && result.time_s < horizon && result.miss.isAllFinite() &&
      result.approach_axis.isAllFinite() && result.relative_speed_m_s > .5f;
  return result;
}

enum class PredictionReject : uint8_t {
  Ready = 0,
  Disabled = 1,
  NoScale = 2,
  Stale = 3,
  NoMotionEvidence = 4,
  NoCrossing = 5,
  Uncertain = 6,
  ImageEdge = 7,
  Unreachable = 8,
  Confirming = 9,
  Lost = 10,
  NotEligible = 11,
  PlannerLimited = 12,
  Fault = 13
};

struct PredictionEvidence {
  float transverse_sigma{INFINITY};
  float transverse_velocity_sigma{INFINITY};
  PredictionReject reject{PredictionReject::NoScale};
  bool usable{false};
};

// Before the full crossing estimate is trusted, persistent vertical miss may
// still justify a small, reversible steering bias. Never use this for a
// terminal commit or for the unobservable horizontal/depth components.
inline float soft_capture_vertical_delta(const MissCorrection &candidate,
                                         float position_sigma, float acceleration) {
  if (!candidate.valid || !std::isfinite(position_sigma + acceleration) ||
      position_sigma < 0.f || acceleration <= 0.f ||
      candidate.time_to_plane < .25f || candidate.time_to_plane > 2.5f ||
      !candidate.miss.isAllFinite() || !candidate.correction.isAllFinite() ||
      fabsf(candidate.miss(2)) < 1.f ||
      candidate.miss(2) * candidate.correction(2) <= 0.f) {
    return 0.f;
  }
  const float confidence = clamp(fabsf(candidate.miss(2)) / (position_sigma + 1.f), .2f, .5f);
  const float cap = min(2.f, .25f * acceleration);
  return clamp(candidate.correction(2) * confidence, -cap, cap);
}

class VerticalCaptureLatch {
public:
  void reset() { _since = 0; _start_updates = 0; _sign = 0; }
  float update(uint64_t now, int accepted_updates, float delta) {
    if (!now || !std::isfinite(delta) || fabsf(delta) <= .01f || accepted_updates < 0) {
      reset();
      return 0.f;
    }
    const int8_t sign = delta > 0.f ? 1 : -1;
    if (_sign != sign || !_since || now < _since || accepted_updates < _start_updates) {
      _since = now;
      _start_updates = accepted_updates;
      _sign = sign;
    }
    return now - _since >= 200000 && accepted_updates >= _start_updates + 2 ? delta : 0.f;
  }
private:
  uint64_t _since{0};
  int _start_updates{0};
  int8_t _sign{0};
};

// Full 3-D velocity uncertainty may be dominated by optical depth. Test the
// predicted miss IN the crossing plane, including the uncertainty in when the
// target reaches that plane. This does not certify full target velocity.
inline PredictionEvidence assess_prediction(const TargetMotion &motion, uint64_t now,
    const Vector3f &axis, const Vector3f &own_velocity, const Vector3f &target_velocity,
    const MissCorrection &candidate, float max_time, float max_cross_sigma,
    float max_cross_velocity_sigma) {
  PredictionEvidence result;
  if (!motion.initialized()) { return result; }
  if (!motion.fresh_range(now)) {
    result.reject = PredictionReject::Stale;
    return result;
  }
  if (!motion.motion_state()) {
    result.reject = PredictionReject::NoMotionEvidence;
    return result;
  }
  if (!candidate.valid || !std::isfinite(candidate.time_to_plane) ||
      candidate.time_to_plane <= 0.f || candidate.time_to_plane > max_time ||
      !axis.isAllFinite() || fabsf(axis.norm() - 1.f) > .01f) {
    result.reject = PredictionReject::NoCrossing;
    return result;
  }
  const Vector3f relative_velocity = own_velocity - target_velocity;
  const float closing = relative_velocity * axis;
  const float radial_sigma = motion.directional_velocity_sigma(axis);
  const float transverse_speed = (relative_velocity - axis * closing).norm();
  if (!std::isfinite(closing + radial_sigma + transverse_speed) ||
      closing - 2.f * radial_sigma <= .5f) {
    result.reject = PredictionReject::Uncertain;
    return result;
  }
  result.transverse_velocity_sigma = motion.transverse_velocity_sigma(axis);
  const float arrival_error = candidate.time_to_plane * radial_sigma /
                              (closing - 2.f * radial_sigma);
  result.transverse_sigma = motion.transverse_position_sigma(axis, candidate.time_to_plane) +
                            transverse_speed * arrival_error;
  if (!std::isfinite(result.transverse_sigma) ||
      result.transverse_sigma > max_cross_sigma ||
      result.transverse_velocity_sigma > max_cross_velocity_sigma) {
    result.reject = PredictionReject::Uncertain;
    return result;
  }
  result.reject = PredictionReject::Ready;
  result.usable = true;
  return result;
}

// 假设短时目标匀速、本机速度对固定命令呈一阶响应，求到达目标平面时的横向残差。
// 调用方提供新鲜可信的状态，并在本函数后继续执行视场、加速度和加加速度规划。
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
    return result; // 越过平面或正在远离目标时，不做回头式修正。
  }
  const auto response_integral = [response_time](float time) {
    return clamp(time + response_time * expm1f(-time / response_time), 0.f, time);
  };
  const auto distance = [&](float time) {
    return closing * time + (commanded_closing - closing) * response_integral(time);
  };
  // 到达时间和残差都使用相同的一阶速度响应模型；若只按当前速度估计时间，
  // 加减速期间会产生虚假的径向误差，甚至错过需要提前修正的时机。
  if (distance(max_time) < remaining) { return result; }
  float time = remaining / closing;
  if (fabsf(commanded_closing - closing) > 1e-5f) {
    float lo = 0.f, hi = max_time;
    // 两端的闭合速度均为正，距离单调，故可以二分求到达时刻。
    for (int i = 0; i < 18; ++i) {
      const float mid = .5f * (lo + hi);
      if (distance(mid) < remaining) { lo = mid; } else { hi = mid; }
    }
    time = .5f * (lo + hi);
  }
  if (!std::isfinite(time) || time <= 0.f) { return result; }
  // 一阶速度阶跃的积分给出实际可实现的位移；到达时间趋近零时，
  // 分母下限与横向速度上限防止数值发散或要求不可达的大幅转向。
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
  // 保持原地速模长，仅旋转速度方向以减小横向误差；不额外引入末段制动。
  // 最终总速、分轴速度和图像视场仍由调用方检查。
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
