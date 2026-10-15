#include "CameraServoMap.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
  float output = NAN;
  assert(dock_motion::camera_servo_output(0.f, 0.f, 90.f, -1.f, 1.f, output) && output == -1.f);
  assert(dock_motion::camera_servo_output(45.f, 0.f, 90.f, -1.f, 1.f, output) && output == 0.f);
  assert(dock_motion::camera_servo_output(90.f, 0.f, 90.f, -1.f, 1.f, output) && output == 1.f);

  // MicoAir ES08MA 1:1 measured endpoints: 90 deg/1500 us, 30 deg/2100 us.
  assert(dock_motion::camera_servo_output(30.f, 30.f, 90.f, 1.f, 0.f, output) && output == 1.f);
  assert(dock_motion::camera_servo_output(60.f, 30.f, 90.f, 1.f, 0.f, output) && output == 0.5f);
  assert(dock_motion::camera_servo_output(90.f, 30.f, 90.f, 1.f, 0.f, output) && output == 0.f);
  assert(dock_motion::camera_servo_output(10.f, 30.f, 90.f, 1.f, 0.f, output) && output == 1.f);
  assert(dock_motion::camera_servo_output(100.f, 30.f, 90.f, 1.f, 0.f, output) && output == 0.f);

  // Shipped MicoAir defaults use the measured endpoints and start at 90 deg.
  assert(dock_motion::camera_servo_output(30.f, 30.f, 90.f, 1.f, 0.f, output));
  assert(std::fabs((1500.f + output * 600.f) - 2100.f) < 0.1f);
  assert(dock_motion::camera_servo_output(90.f, 30.f, 90.f, 1.f, 0.f, output) && output == 0.f);

  assert(!dock_motion::camera_servo_output(NAN, 30.f, 90.f, 1.f, 0.f, output));
  assert(!dock_motion::camera_servo_output(60.f, 90.f, 30.f, 1.f, 0.f, output));
  assert(!dock_motion::camera_servo_output(60.f, 30.f, 90.f, 0.f, 0.f, output));
  assert(!dock_motion::camera_servo_output(60.f, 30.f, 90.f, 1.1f, 0.f, output));
  puts("camera servo map: passed");
}
