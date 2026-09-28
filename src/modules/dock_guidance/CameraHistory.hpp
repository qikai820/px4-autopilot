// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace dock_motion {

// A publication timestamp is not the time represented by an estimator state.
// Fall back only for publishers which do not provide a sample timestamp.
inline bool state_sample_time(uint64_t sample, uint64_t published, uint64_t now,
                              uint64_t max_age, uint64_t &stamp) {
  stamp = sample ? sample : published;
  return stamp && published && stamp <= published && published <= now && now - stamp <= max_age;
}

// Bring the attitude to the history-slot time using fresh body rates. The
// short hold is bounded independently of the camera-delay history length.
inline bool attitude_at_time(const matrix::Quatf &sample, const matrix::Vector3f &body_rate,
                             uint64_t sample_time, uint64_t rate_time, uint64_t now,
                             matrix::Quatf &aligned) {
  if (!sample.isAllFinite() || sample.norm() < .5f || !body_rate.isAllFinite() ||
      !sample_time || !rate_time || now < sample_time || now < rate_time ||
      now - sample_time > 20000 || now - rate_time > 20000) {
    return false;
  }
  const float dt = (now - sample_time) * 1e-6f;
  aligned = sample;
  aligned.normalize();
  aligned = aligned * matrix::Quatf(matrix::AxisAnglef(body_rate * dt));
  aligned.normalize();
  return aligned.isAllFinite();
}

// 延时状态历史的槽位索引：保留采样率受 interval 限制，但每次回调仍刷新最新槽位。
// 间隔未满时覆盖最新槽位，避免零延迟观测读到旧姿态；时间倒退则清空历史。
template<int Capacity> class SampleHistoryIndex {
public:
  explicit SampleHistoryIndex(uint64_t interval) : _interval(interval) {}
  void reset() { _next = _count = 0; _retained = _latest = 0; }
  int write(uint64_t time) {
    if (_count && time < _latest) { reset(); }
    _latest = time;
    if (_count && time - _retained < _interval) { return newest(); }
    const int slot = _next;
    _next = (_next + 1) % Capacity;
    if (_count < Capacity) { ++_count; }
    _retained = time;
    return slot;
  }
  int count() const { return _count; }
  int oldest() const { return (_next + Capacity - _count) % Capacity; }
  int newest() const { return (_next + Capacity - 1) % Capacity; }
private:
  static_assert(Capacity > 1, "state interpolation needs two slots");
  const uint64_t _interval;
  uint64_t _retained{0}, _latest{0};
  int _next{0}, _count{0};
};

// 保存相机俯仰命令历史，不代表舵机真实反馈。命令从发布时刻持续有效直到下一条命令，
// 查询旧图像只能读当时已发布的命令，不能把未来命令反向插值到曝光时刻。
// 角度未变化的保活命令不额外占用历史槽位。
template<int Capacity> class CameraHistory {
public:
  void reset() { _next = _count = 0; }
  void record(uint64_t time, float pitch) {
    if (!time || !std::isfinite(pitch)) { return; }
    if (_count) {
      const int last = (_next + Capacity - 1) % Capacity;
      if (time < _times[last]) {
        reset();
      } else if (time == _times[last]) {
        _pitches[last] = pitch;
        return;
      } else if (fabsf(pitch - _pitches[last]) < 1e-6f) {
        return;
      }
    }
    _times[_next] = time;
    _pitches[_next] = pitch;
    _next = (_next + 1) % Capacity;
    if (_count < Capacity) { ++_count; }
  }
  bool at(uint64_t exposure, float &pitch) const {
    for (int n = 1; n <= _count; ++n) {
      const int i = (_next + Capacity - n) % Capacity;
      if (_times[i] <= exposure) {
        pitch = _pitches[i];
        return true;
      }
    }
    return false;
  }
private:
  static_assert(Capacity > 0, "camera history must not be empty");
  uint64_t _times[Capacity]{};
  float _pitches[Capacity]{};
  int _next{0}, _count{0};
};

inline bool exposure_time(uint64_t received, uint64_t delay, uint64_t &exposure) {
  // Viztra 时间戳为接收时刻；仅在可减去标定延迟时给出合法曝光时刻。
  if (!received || received <= delay) { return false; }
  exposure = received - delay;
  return true;
}
} // namespace dock_motion
