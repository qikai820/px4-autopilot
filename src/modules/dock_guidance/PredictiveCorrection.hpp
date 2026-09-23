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
