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

// LandDetector 状态改变时立即发布，但状态不变时约 1 Hz 保活且存在调度抖动；
// 因此用 2 s 时效容限，避免正常的 1.01 s 间隔被误判为未起飞。
inline bool airborne_state(bool armed, bool landed, uint64_t stamp, uint64_t now) {
  return armed && !landed && stamp != 0 && now >= stamp && now - stamp <= 2000000;
}

inline bool capture_evidence_allowed(bool common_evidence, bool pass_through,
                                     bool already_ready, float range_lower,
                                     float initial_distance) {
  // 普通捕获要求保守距离足够远；确认过的穿越捕获可在近距离继续，
  // 但都必须有新鲜且有限的几何/图像共同证据。
  return common_evidence && std::isfinite(range_lower) &&
         ((pass_through && already_ready) || range_lower > initial_distance);
}

inline bool capture_image_evidence(bool hard, float fov, float align, float soft,
                                   float los_rate, float acquisition_rate,
                                   bool pass_through_ready) {
  if (hard || !std::isfinite(fov + align + soft + los_rate + acquisition_rate)) {
    return false;
  }
  // 首次捕获使用 ALIGN 阈值；穿越确认后放宽到 SOFT，避免小幅相机/机体滞后
  // 反复把目标速度从初始值切到最大值。最终规划仍按实测视线变化预测视场并平滑减速。
  return pass_through_ready ? fov < soft
                            : fov <= align && los_rate < acquisition_rate;
}

// 可靠的方位和距离允许有界 LOS 捕获，并不意味着目标速度已经可信。
// 图像对准和几何证据持续满足确认时间前，保持初始速度上限。
class CaptureSchedule {
public:
  void reset() { _since = 0; _ready = false; }
  bool ready() const { return _ready; }
  float evidence_duration(uint64_t now) const {
    return _since && now >= _since ? (now - _since) * 1e-6f : 0.f;
  }
  float update(uint64_t now, bool evidence, float confirmation, float initial_limit,
               float pursuit_limit, bool pass_through, float range_lower, float closing_bias,
               float stop_distance, float deceleration, float jerk) {
    // 普通视觉捕获上限为 10 m/s；末段穿越明确要求追击时，
    // 可使用配置的相对速度预算，但不超过模块 60 m/s 上限。
    const float low = clamp(initial_limit, 0.f, pass_through ? 60.f : 10.f);
    if (!evidence || low <= 0.f || !std::isfinite(range_lower + closing_bias)) {
      reset();
      return low;
    }
    if (!_since || now < _since) { _since = now; _ready = false; }
    _ready = now - _since >= uint64_t(clamp(confirmation, .1f, 5.f) * 1e6f);
    if (!_ready) { return low; }
    if (pass_through) {
      // 穿越目标平面不预留平面前的停车距离；调用方仍负责总速、分轴速度、
      // 加速度、加加速度与预测视场约束。
      return max(low, clamp(pursuit_limit, 0.f, 60.f));
    }
    const float decel = max(.1f, deceleration);
    const float reaction = .5f + decel / max(.1f, jerk);
    const float distance = max(0.f, range_lower - max(2.f, stop_distance));
    // 普通捕获预留控制响应和加加速度反向所需距离，并计入径向运动不确定度；
    // 此包络只是保守限速，不能当作末段穿越许可。
    const float speed = sqrtf(decel * decel * reaction * reaction + 2.f * decel * distance)
                        - decel * reaction - max(0.f, closing_bias);
    return max(low, min(clamp(pursuit_limit, 0.f, 60.f), max(0.f, speed)));
  }
private:
  uint64_t _since{0};
  bool _ready{false};
};

// NED 坐标中上升速度为负；剩余爬升高度为正时绝不请求下降。
// 视场风险可以暂停爬升，不能把爬升命令反转为下降。
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
  // 在停车包络中预留实测爬升速度、控制延迟和加速度反向所需的距离。
  const float reaction = .25f + max(.1f, decel) / max(.1f, jerk);
  const float available = remaining - max(0.f, -actual_vz) * reaction;
  return startup_climb_velocity(available, up_limit, decel, fov_scale);
}

// 像素框首先给出光学轴向深度，斜距还要按视线角换算。
// 必须已标定目标真实投影尺寸；未知尺寸时不能启用米制距离推算。
inline bool box_range(float width_px, float height_px, float width_m, float height_m, float fx,
                      float fy, float forward_cos, float size_error, float &range, float &sigma) {
  if (!std::isfinite(width_px + height_px + width_m + height_m + fx + fy + forward_cos +
                     size_error) ||
      fx <= 0.f || fy <= 0.f || forward_cos < .2f) {
    return false;
  }
  // 保留跨 5/6 像素量化边界的小目标框，并在下方保留其较大的距离不确定度；
  // 这并不等于允许沿视线高速闭合。
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

// 本地 NED 系下的目标匀速状态估计，状态量为三维位置和三维速度。
// 本机位置取曝光时刻，完整 6x6 协方差保留位置与速度相关性。
// 纯方位角不能初始化米制位置；人为距离先验不等于真实量测。
class TargetMotion {
public:
  void reset() { const uint32_t count = _resets + 1; *this = TargetMotion{}; _resets = count; }
  uint32_t resets() const { return _resets; }
  int accepted_updates() const { return _count; }
  uint8_t motion_state() const { return _motion_state; } // 0 未知，1 静止，2 运动。
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
  // 不把深度方向的大方差直接加到穿越平面的两个横向分量上。
  // 目标尺寸误差是随视线方向共同变化的尺度误差，不能靠多帧平均消掉。
  float transverse_position_sigma(const Vector3f &plane_normal, float horizon) const {
    if (!initialized() || !plane_normal.isAllFinite() ||
        fabsf(plane_normal.norm() - 1.f) > .01f || !std::isfinite(horizon) ||
        horizon < 0.f) {
      return INFINITY;
    }
    float variance = 0.f;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        const float projection = (i == j ? 1.f : 0.f) - plane_normal(i) * plane_normal(j);
        const float predicted_covariance = _p(i, j) +
            horizon * (_p(i, j + 3) + _p(i + 3, j)) +
            horizon * horizon * _p(i + 3, j + 3);
        variance += projection * predicted_covariance;
      }
    }
    const float scale_projection = sqrtf(max(0.f, 1.f -
        (_last_los * plane_normal) * (_last_los * plane_normal)));
    return max(sqrtf(max(0.f, variance)), _scale_floor * scale_projection) +
        .5f * _accel * horizon * horizon;
  }
  float transverse_velocity_sigma(const Vector3f &plane_normal) const {
    if (!initialized() || !plane_normal.isAllFinite() ||
        fabsf(plane_normal.norm() - 1.f) > .01f) {
      return INFINITY;
    }
    return sqrtf(max(0.f, velocity_sigma() * velocity_sigma() -
        directional_velocity_sigma(plane_normal) *
        directional_velocity_sigma(plane_normal)));
  }
  Vector3f following_velocity(uint64_t now, float max_error) const {
    Vector3f result;
    // 在视线/切平面坐标中评估误差，不直接逐个 NED 轴评估；
    // 否则深度不确定会连带压制观察较好的横向速度。
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
    // 切平面使用同一权重，使结果不依赖 NED 与相机轴的朝向。
    const float lateral_error = sqrtf(max(0.f, total_variance - radial_error * radial_error));
    const auto weight = [max_error](float error) {
      return clamp((2.f * max_error - error) / max(max_error, .1f), 0.f, 1.f);
    };
    // 这里只是有界的部分目标跟随，不能宣称米制速度已可信。
    // 目标尺度越不确定，跟随越弱；深度不确定不应抹掉可观测的横向运动。
    // 完整闭合速度仍须通过 valid() 的检查。
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
    // 目标速度成熟、米制量测仍新鲜且速度方差足够低，才允许用于高速闭合。
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
    // 预测-校正流程：有距离时更新视线方向和深度；只有方位时仅更新切平面。
    // 先拒绝时间倒退、过期或离群量测，再更改状态/协方差。
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
    // 切向方位观测可沿用预测距离更新方向，不要求本帧重新测得目标框距离。
    const float observation_range = metric ? range : relative(time, own).norm();
    const Vector3f measurement = own + los * observation_range;
    if (!_time) {
      // 首次米制量测初始化目标位置；初始目标速度方差由允许的最大速度给出。
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
    // 像素量化使连续目标框高度相关，估计器最多以 10 Hz 接受更新。
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
    // 匀速状态转移后加入加速度过程噪声，以允许目标真实机动。
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
    // 先对整条量测做残差门控；任何轴超出 4 倍标准差都不执行本帧校正。
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
      _p = a * _p * a.transpose() + km * k.transpose() * noise[j]; // Joseph 形式，减少协方差数值退化。
    }
    if (!_x.isAllFinite() || !_p.isAllFinite() || velocity().norm() > max_speed) {
      reset();
      return false;
    }
    _accepted = time;
    if (metric) {
      _metric_time = time;
      _scale_floor = range_sigma; // 目标尺寸的系统误差不能靠多帧平均消除。
      _scale_fraction = range_sigma / range;
    }
    _last_los = los;
    _count = min(_count + 1, 1000);
    const float speed = velocity().norm(), error = velocity_sigma();
    const float motion_error = speed > .1f ? directional_velocity_sigma(velocity() / speed) : error;
    const bool consistent = (velocity() - _evidence_velocity).norm() <= max(2.f, 3.f * _accel * dt);
    const uint8_t candidate = speed + 2.f * error < 3.f ? 1 :
                              (speed - 2.f * motion_error > 2.f && consistent ? 2 : 0);
    // 静止/运动判断使用速度置信区间和连续证据；候选状态保持 1.5 s 后才确认。
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
// 短时目标留在视场内的启发式预测，无法保证覆盖目标突然机动。
// 当前图像漂移已包含真实相机转动；只额外预测命令加速度变化造成的倾斜/平移。
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
  // 以锁定的接近方向定义目标平面；横向误差和两倍位置不确定度超限则判为不确定。
  // 只有目标连同不确定度已越过平面及安全余量，才报告 Predicted。
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
  float total, xy, up, down; // 总速、水平速、上升速和下降速上限，单位 m/s。
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
// 求整条速度射线 base + c*los 与总速球、水平速圆柱及竖直上下限的交集。
// 单独的部分目标跟随可能暂时超出竖直限制，但加上闭合分量后又落回可行域；
// 若先拒绝 base，会造成捕获速度突然归零。
inline float closing_budget(const Vector3f &base, const Vector3f &los, const Limits &l,
                            float requested) {
  if (!base.isAllFinite() || !los.isAllFinite() || los.norm() < .5f ||
      !std::isfinite(l.total + l.xy + l.up + l.down + requested) ||
      min(min(l.total, l.xy), min(l.up, l.down)) < 0.f || requested <= 0.f) {
    return 0.f;
  }
  float lo = 0.f, hi = requested;
  const auto intersect_ball = [&](float a, float b, float c) {
    // 把速度球/圆柱约束化为 a*c_speed^2 + 2*b*c_speed + c <= 0。
    if (a <= 0.f) { return c <= 0.f; }
    const float discriminant = b*b - a*c;
    if (discriminant < 0.f) { return false; }
    const float root = sqrtf(discriminant);
    lo = max(lo, (-b-root)/a);
    hi = min(hi, (-b+root)/a);
    return lo <= hi;
  };
  if (!intersect_ball(los.norm_squared(), base*los, base.norm_squared()-l.total*l.total) ||
      !intersect_ball(los(0)*los(0)+los(1)*los(1), base(0)*los(0)+base(1)*los(1),
                      base(0)*base(0)+base(1)*base(1)-l.xy*l.xy)) {
    return 0.f;
  }
  if (fabsf(los(2)) > 1e-8f) {
    const float a = (-l.up-base(2))/los(2), b = (l.down-base(2))/los(2);
    lo = max(lo, min(a,b));
    hi = min(hi, max(a,b));
  } else if (base(2) < -l.up || base(2) > l.down) {
    return 0.f;
  }
  return lo <= hi && hi >= 0.f && feasible(base + los*hi, l) ? hi : 0.f;
}
struct VelocityPlan {
  Vector3f velocity{}, acceleration{};
  float predicted_fov{0.f};
  bool limits_override{false};
};

// 对最终将发布的速度候选逐个评分：先合入高度反馈，再统一限制加加速度。
// 不在选定候选后覆盖 Z 轴，否则会绕过已验证的速度和加加速度约束。
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
    // 新降低的限速或迫近分轴边界可能与加加速度限制暂时不兼容；
    // 此时优先返回受速度约束的候选，并显式记录 override 供日志诊断。
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
