// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace dock_motion {

// 通过后的航向门控：高速时锁实际机头，低速后才恢复原目标，不能仅冻结旧目标。
class RecoveryHeading {
public:
  void reset() { _heading = _target = NAN; _locked = true; }

  float update(const matrix::Vector3f &velocity, float measured_heading, float desired_heading,
               float release_speed, float yaw_rate, float dt,
               uint8_t reset_counter, float delta_heading) {
    if (!velocity.isAllFinite() || !std::isfinite(measured_heading) ||
        !std::isfinite(release_speed) || release_speed <= 0.f ||
        !std::isfinite(yaw_rate) || yaw_rate <= 0.f || !std::isfinite(dt) || dt < 0.f) {
      return NAN;
    }

    if (!std::isfinite(_heading)) {
      _heading = matrix::wrap_pi(measured_heading);
      _target = std::isfinite(desired_heading) ? matrix::wrap_pi(desired_heading) : _heading;
      _reset_counter = reset_counter;
      _locked = true;

    } else if (reset_counter != _reset_counter) {
      // EKF 航向坐标重置不代表飞机真的转向；同步移动锁定值与待恢复目标。
      // 多次重置只有最后一次 delta 可见，无法可靠恢复时交给现有故障路径。
      if (static_cast<uint8_t>(reset_counter - _reset_counter) != 1 || !std::isfinite(delta_heading)) {
        return NAN;
      }
      _heading = matrix::wrap_pi(_heading + delta_heading);
      _target = matrix::wrap_pi(_target + delta_heading);
      _reset_counter = reset_counter;
    }

    if (velocity.norm() >= release_speed) {
      if (!_locked) {
        // 已低速转向后再次超速：以此刻实际机头重新锁住，停止继续追赶旧目标。
        _heading = matrix::wrap_pi(measured_heading);
      }
      _locked = true;

    } else {
      _locked = false;
      const float step = yaw_rate * fminf(dt, .1f);
      const float error = matrix::wrap_pi(_target - _heading);
      _heading = matrix::wrap_pi(_heading + fmaxf(-step, fminf(error, step)));
    }
    return _heading;
  }

private:
  float _heading{NAN}, _target{NAN};
  uint8_t _reset_counter{0};
  bool _locked{true};
};

} // namespace dock_motion
