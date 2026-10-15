// Camera pitch in body degrees to the normalized Gimbal Pitch output.
#pragma once

#include <cmath>

namespace dock_motion {

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
