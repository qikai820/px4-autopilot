// Camera pitch in body degrees to the normalized Gimbal Pitch output.
#pragma once

#include <cmath>

namespace dock_motion {

// The H743 camera test stays inside the measured PWM envelope and leaves
// 50 us at the low-angle end. With CENT5=-1, MIN5/MAX5 are the angle endpoints;
// otherwise CENT5 is the high-angle endpoint (the existing board default).
inline bool camera_servo_test_limits(int minimum, int maximum, int center,
                                     int disarmed, int failsafe,
                                     int &test_minimum, int &test_maximum)
{
  if (minimum >= maximum || maximum > 2100 ||
      (center != -1 && (center < minimum || center >= maximum))) {
    return false;
  }

  test_minimum = center == -1 ? minimum : center;
  test_maximum = maximum - 50;
  return test_minimum >= 1400 && test_minimum < test_maximum &&
         disarmed == test_minimum && (failsafe == -1 || failsafe == disarmed);
}

inline bool camera_servo_output(float pitch_deg, float minimum_pitch_deg,
                                float maximum_pitch_deg, float output_at_minimum,
                                float output_at_maximum, float &output)
{
  if (!std::isfinite(pitch_deg) || !std::isfinite(minimum_pitch_deg) ||
      !std::isfinite(maximum_pitch_deg) || !std::isfinite(output_at_minimum) ||
      !std::isfinite(output_at_maximum) ||
      maximum_pitch_deg - minimum_pitch_deg < 1.f ||
      output_at_minimum < -1.f || output_at_minimum > 1.f ||
      output_at_maximum < -1.f || output_at_maximum > 1.f ||
      fabsf(output_at_maximum - output_at_minimum) < 0.01f) {
    return false;
  }

  const float fraction = fminf(1.f, fmaxf(0.f,
      (pitch_deg - minimum_pitch_deg) / (maximum_pitch_deg - minimum_pitch_deg)));
  output = output_at_minimum + fraction * (output_at_maximum - output_at_minimum);
  return true;
}

} // namespace dock_motion
