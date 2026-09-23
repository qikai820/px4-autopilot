// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>

namespace dock_motion {

// Bound retained history rate without making the latest state stale. Callers
// refresh all state fields at the returned slot on EVERY callback. Between
// retained samples the newest slot is replaced, not silently skipped.
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

// Command history, NOT servo feedback. A published command applies from its
// publication time until the next command; never interpolate a future command
// backwards into an older image. Unchanged keepalives do not consume history.
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
  if (!received || received <= delay) { return false; }
  exposure = received - delay;
  return true;
}
} // namespace dock_motion
