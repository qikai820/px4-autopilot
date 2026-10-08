// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include "TargetMotion.hpp"

namespace dock_motion {

// Relative position/rate at image exposure. Ego displacement and velocity
// changes are known inputs, so aircraft acceleration is not interpreted as
// target acceleration. No stationary/moving classification or absolute target
// velocity convergence is required by the controller.
class RelativeMotion {
public:
  void reset() { *this = RelativeMotion{}; }
  float age(uint64_t now) const {
    return _time && now >= _time ? (now - _time) * 1e-6f : INFINITY;
  }
  bool fresh(uint64_t now) const { return age(now) <= .25f; }
  bool ready(uint64_t now) const {
    return fresh(now) && _updates >= 4 && _time - _first >= 300000;
  }
  unsigned updates() const { return _updates; }
  Vector3f relative(uint64_t now, const Vector3f &own) const {
    return _relative + _rate * age(now) - (own - _own - _own_velocity * age(now));
  }
  Vector3f rate(const Vector3f &own_velocity) const { return _rate - (own_velocity - _own_velocity); }

  bool update(uint64_t time, const Vector3f &own, const Vector3f &own_velocity,
      const Vector3f &measurement, float range_sigma, float angle_sigma, float process_accel) {
    if (!time || time <= _time || !own.isAllFinite() || !own_velocity.isAllFinite() ||
        !measurement.isAllFinite() || measurement.norm() < .2f ||
        !std::isfinite(range_sigma + angle_sigma + process_accel) ||
        range_sigma <= 0.f || angle_sigma <= 0.f || process_accel < 0.f) { return false; }
    if (_time && time - _time < 90000) { return false; } // independent image information, at most 11 Hz
    if (_time && time - _time > 500000) { reset(); }
    const Vector3f ray = measurement.normalized();
    const float tangent_sigma = max(.05f, measurement.norm() * angle_sigma);
    const float radial_var = range_sigma * range_sigma;
    const float tangent_var = tangent_sigma * tangent_sigma;
    if (!_time) {
      _relative = measurement;
      _rate = -own_velocity; // prior only; large rate variance prevents terminal permission
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          _p[i][j] = (i == j ? tangent_var : 0.f) +
              (radial_var - tangent_var) * ray(i) * ray(j);
        }
        _p[i + 3][i + 3] = 900.f;
      }
      _first = time;
    } else {
      const float dt = (time - _time) * 1e-6f;
      Vector3f position = _relative + _rate * dt - (own - _own - _own_velocity * dt);
      Vector3f velocity = _rate - (own_velocity - _own_velocity);
      float p[6][6]{};
      const float q = max(.01f, process_accel * process_accel);
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          p[i][j] = _p[i][j];
          if (i < 3) { p[i][j] += dt * _p[i + 3][j]; }
          if (j < 3) { p[i][j] += dt * _p[i][j + 3]; }
          if (i < 3 && j < 3) { p[i][j] += dt * dt * _p[i + 3][j + 3]; }
        }
      }
      for (int i = 0; i < 3; ++i) {
        p[i][i] += q * dt * dt * dt / 3.f + .0025f;
        p[i][i + 3] += q * dt * dt / 2.f;
        p[i + 3][i] += q * dt * dt / 2.f;
        p[i + 3][i + 3] += q * dt + .01f;
      }
      Vector3f tangent = ray.cross(Vector3f(0.f, 0.f, 1.f));
      if (tangent.norm() < .1f) { tangent = ray.cross(Vector3f(0.f, 1.f, 0.f)); }
      tangent.normalize();
      const Vector3f axes[3] = {ray, tangent, ray.cross(tangent)};
      // Validate the whole observation before applying any component.
      for (int axis = 0; axis < 3; ++axis) {
        const Vector3f &h = axes[axis];
        float variance = axis == 0 ? radial_var : tangent_var;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) { variance += h(i) * p[i][j] * h(j); }
        }
        const float innovation = (measurement - position) * h;
        if (innovation * innovation > 25.f * variance) { return false; }
      }
      for (int axis = 0; axis < 3; ++axis) {
        const Vector3f &h = axes[axis];
        float ph[6]{};
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 3; ++j) { ph[i] += p[i][j] * h(j); }
        }
        float variance = axis == 0 ? radial_var : tangent_var;
        for (int i = 0; i < 3; ++i) { variance += h(i) * ph[i]; }
        const float innovation = (measurement - position) * h;
        for (int i = 0; i < 3; ++i) {
          position(i) += ph[i] * innovation / variance;
          velocity(i) += ph[i + 3] * innovation / variance;
        }
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) { p[i][j] -= ph[i] * ph[j] / variance; }
        }
      }
      if (!position.isAllFinite() || !velocity.isAllFinite()) { return false; }
      _relative = position;
      _rate = velocity;
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) { _p[i][j] = p[i][j]; }
      }
    }
    _time = time;
    _own = own;
    _own_velocity = own_velocity;
    _scale_error = ray * range_sigma; // systematic size error must not average away
    _angle_error = tangent_sigma;
    _process_accel = process_accel;
    ++_updates;
    return true;
  }

  // Horizon starts at the last accepted image; include image age at the call
  // site. Cross-track covariance is projected onto the actual encounter axis.
  float sigma(const Vector3f &axis, float horizon, bool transverse) const {
    if (!_time || !axis.isAllFinite() || fabsf(axis.norm() - 1.f) > .01f ||
        !std::isfinite(horizon) || horizon < 0.f) { return INFINITY; }
    float along = 0.f, trace = 0.f;
    const float q = _process_accel * _process_accel * horizon * horizon * horizon / 3.f;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        const float value = _p[i][j] + horizon * (_p[i][j + 3] + _p[i + 3][j]) +
            horizon * horizon * _p[i + 3][j + 3] + (i == j ? q : 0.f);
        along += axis(i) * value * axis(j);
        if (i == j) { trace += value; }
      }
    }
    const float systematic_along = _scale_error * axis;
    const float floor = transverse
        ? max(0.f, _scale_error.norm_squared() - systematic_along * systematic_along) +
            2.f * _angle_error * _angle_error
        : systematic_along * systematic_along + _angle_error * _angle_error;
    return sqrtf(max(floor, transverse ? trace - along : along));
  }

private:
  Vector3f _relative{}, _rate{}, _own{}, _own_velocity{}, _scale_error{};
  float _p[6][6]{};
  float _angle_error{0.f}, _process_accel{0.f};
  uint64_t _time{0}, _first{0};
  unsigned _updates{0};
};
} // namespace dock_motion
