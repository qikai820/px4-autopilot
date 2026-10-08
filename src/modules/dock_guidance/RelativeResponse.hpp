// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once
#include "TargetMotion.hpp"

namespace dock_motion {
// Identify the aircraft response, independently of target motion. The bias
// accounts for persistent velocity tracking error; measured thrust and net
// acceleration identify effective linear and quadratic resistance for future force
// allocation. This is a local response model, not an aerodynamic calibration.
class RelativeResponse {
public:
  void reset() { *this = RelativeResponse{}; }
  bool fresh(uint64_t now) const { return _time && now >= _time && now - _time <= 150000; }
  bool update(uint64_t sample, const Vector3f &velocity, const Vector3f &acceleration,
      const Vector3f &command, const Vector3f &thrust_ned, float hover, float response) {
    if (!sample || sample <= _time || !velocity.isAllFinite() || !acceleration.isAllFinite() ||
        !command.isAllFinite() || !thrust_ned.isAllFinite() ||
        !std::isfinite(hover + response) || hover < .1f || hover > .9f || response < .05f || response > 2.f ||
        velocity.norm() > 150.f || acceleration.norm() > 50.f || thrust_ned.norm() > 1.01f) { return false; }
    if (_time && sample - _time > 150000) { reset(); }
    const float dt = _time ? clamp((sample - _time) * 1e-6f, .002f, .1f) : .01f;
    const Vector3f raw_bias = velocity - command + acceleration * response;
    _bias += (raw_bias - _bias) * (dt / (.3f + dt));
    _uncertainty = max(.2f, (raw_bias - _bias).norm());
    const Vector3f resistance = thrust_ned * (9.80665f / hover) +
        Vector3f(0.f, 0.f, 9.80665f) - acceleration;
    for (int axis = 0; axis < 3; ++axis) {
      // Small velocities do not provide enough excitation to identify drag.
      const float speed = fabsf(velocity(axis));
      if (speed > 3.f) {
        const float damping = resistance(axis) / velocity(axis);
        if (damping >= 0.f && damping <= 6.f) {
          auto &fit = _fit[axis];
          const float weight = fit.initialized ? dt / (2.f + dt) : 1.f;
          fit.speed += weight * (speed - fit.speed);
          fit.damping += weight * (damping - fit.damping);
          fit.speed_squared += weight * (speed * speed - fit.speed_squared);
          fit.speed_damping += weight * (speed * damping - fit.speed_damping);
          fit.initialized = true;
          const float variance = max(0.f, fit.speed_squared - fit.speed * fit.speed);
          // At one speed the two terms are unidentifiable. Start with local
          // linear damping; identify curvature only after speed excitation.
          float quadratic = variance > .5f ? clamp(
              (fit.speed_damping - fit.speed * fit.damping) / variance, 0.f, .5f) : _drag(axis);
          float linear = fit.damping - quadratic * fit.speed;
          if (linear < 0.f) {
            linear = 0.f;
            quadratic = clamp(fit.speed_damping / max(.01f, fit.speed_squared), 0.f, .5f);
          }
          const float gain = dt / (.2f + dt);
          _linear_drag(axis) += (clamp(linear, 0.f, 6.f) - _linear_drag(axis)) * gain;
          _drag(axis) += (quadratic - _drag(axis)) * gain;
        }
      }
    }
    _time = sample;
    _hover = hover;
    return true;
  }
  const Vector3f &bias() const { return _bias; }
  const Vector3f &drag() const { return _drag; }
  const Vector3f &linear_drag() const { return _linear_drag; }
  float uncertainty() const { return _uncertainty; }
  float hover() const { return _hover; }
private:
  struct DragFit {
    float speed{0.f}, damping{0.f}, speed_squared{0.f}, speed_damping{0.f};
    bool initialized{false};
  } _fit[3];
  Vector3f _bias{}, _drag{}, _linear_drag{};
  float _uncertainty{INFINITY};
  float _hover{.5f};
  uint64_t _time{0};
};
} // namespace dock_motion
