// Standalone deterministic regression tests; see docs/DockGuidance_dynamic.md.
#include "CameraHistory.hpp"
#include "ImageTracking.hpp"
#include "PredictiveCorrection.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace dock_motion;
using matrix::Dcmf;
using matrix::Eulerf;

static void test_estimator_sample_alignment() {
  uint64_t stamp = 0;
  assert(state_sample_time(1000000, 1002000, 1010000, 20000, stamp) && stamp == 1000000);
  assert(state_sample_time(0, 1002000, 1010000, 20000, stamp) && stamp == 1002000);
  assert(!state_sample_time(1003000, 1002000, 1010000, 20000, stamp));
  assert(!state_sample_time(1000000, 1011000, 1010000, 20000, stamp));
  assert(!state_sample_time(980000, 1002000, 1010000, 20000, stamp));
  assert(!state_sample_time(0, 0, 1010000, 20000, stamp));

  const matrix::Quatf initial(Eulerf(.4f, -.3f, .7f));
  const Vector3f rates(.4f, -1.2f, .7f);
  const Vector3f target(150.f, 30.f, -80.f);
  float old_error = 0.f, new_error = 0.f;
  // Jitter in attitude delivery must not appear as motion of a static target.
  // Analytic constant-body-rate motion provides an independent geometry oracle.
  for (int i = 0; i < 100; ++i) {
    const uint64_t now = 2000000 + i * 10000;
    const uint64_t lag = 5000 + (i % 4) * 5000;
    const float elapsed = (now - 1000000) * 1e-6f;
    const matrix::Quatf actual = initial * matrix::Quatf(matrix::AxisAnglef(rates * elapsed));
    const matrix::Quatf delayed = initial * matrix::Quatf(matrix::AxisAnglef(rates * (elapsed - lag * 1e-6f)));
    matrix::Quatf aligned;
    assert(attitude_at_time(delayed, rates, now - lag, now - 1000, now, aligned));
    const Vector3f ray = Dcmf(actual).transpose() * target;
    old_error = max(old_error, Vector3f(Dcmf(delayed) * ray - target).norm());
    new_error = max(new_error, Vector3f(Dcmf(aligned) * ray - target).norm());
  }
  assert(old_error > 3.f && new_error < .0002f);
  matrix::Quatf unused;
  assert(!attitude_at_time(initial, rates, 1000000, 1020000, 1020001, unused));
  assert(!attitude_at_time(initial, rates, 1020000, 1000000, 1020001, unused));
  assert(!attitude_at_time(initial, rates, 1020001, 1020000, 1020000, unused));
  assert(!attitude_at_time(initial, Vector3f(NAN, 0.f, 0.f), 1020000, 1020000, 1020000, unused));
  assert(attitude_at_time(initial, Vector3f{}, 1000000, 1000000, 1020000, unused));
  assert(Vector3f(Dcmf(unused) * target - Dcmf(initial) * target).norm() < .0001f);
  printf("state sample timing: static-target error before %.3f m, aligned %.6f m\n",
      double(old_error), double(new_error));
}

static void test_fresh_state_history() {
  SampleHistoryIndex<601> history(10000);
  uint64_t stamps[601]{};
  float attitude[601]{};
  // Alternating timer/camera callbacks, including pairs less than 10 ms apart.
  // The old early return used a stale attitude on every intermediate callback.
  for (uint64_t now=1000000; now<9000000; now+=4000) {
    const int slot = history.write(now);
    stamps[slot] = now;
    attitude[slot] = float(now-1000000)*1e-6f*.5f;
    assert(stamps[history.newest()] == now);
    assert(fabsf(attitude[history.newest()] - float(now-1000000)*1e-6f*.5f) < 1e-7f);
    if (now >= 8500000) {
      assert(history.count() == 601);
      assert(now-stamps[history.oldest()] >= 5000000);
      for (int n=1; n<history.count(); ++n) {
        assert(stamps[(history.oldest()+n-1)%601] < stamps[(history.oldest()+n)%601]);
      }
    }
  }
  history.write(500000); // time reset
  assert(history.count() == 1);
  history.reset();
  assert(history.count() == 0);
  history.write(1000000);
  assert(history.write(1001000) == history.newest() && history.count() == 1);
  puts("state history: every callback fresh; five-second coverage retained");
}

static void test_camera_history() {
  CameraHistory<3> history;
  float pitch = NAN;
  assert(!history.at(100, pitch));
  history.record(100, 10);
  history.record(200, 20);
  history.record(300, 30);
  assert(!history.at(99, pitch));
  assert(history.at(199, pitch) && pitch == 10); // no future-angle interpolation
  assert(history.at(200, pitch) && pitch == 20);
  history.record(400, 30); // unchanged keepalive must retain early history
  assert(history.at(100, pitch) && pitch == 10);
  history.record(500, 40);
  assert(!history.at(199, pitch)); // oldest sample evicted
  assert(history.at(499, pitch) && pitch == 30);
  history.record(500, 45);
  assert(history.at(500, pitch) && pitch == 45);
  history.record(600, NAN);
  assert(history.at(10000, pitch) && pitch == 45); // command held until replaced
  history.record(50, 5); // clock reset invalidates old domain
  assert(!history.at(49, pitch));
  assert(history.at(100, pitch) && pitch == 5);
  history.reset();
  assert(!history.at(100, pitch));
  uint64_t exposure = 0;
  assert(!exposure_time(0, 0, exposure));
  assert(!exposure_time(80000, 80000, exposure));
  assert(!exposure_time(50000, 80000, exposure));
  assert(exposure_time(180000, 80000, exposure) && exposure == 100000);
}

static void test_encoder_pitch_alignment() {
  float pitch = NAN;
  // New 90-degree reference: both directions remain correct across counter wrap.
  assert(encoder_pitch_deg(2712, -1, 2712, false, 0.f, 90.f, pitch) && pitch == 90.f);
  assert(encoder_pitch_deg(15000, -1, 2712, false, 0.f, 90.f, pitch) && pitch == 0.f);
  assert(encoder_pitch_deg(10904, -1, 10904, true, 0.f, 90.f, pitch) && pitch == 90.f);
  assert(encoder_pitch_deg(15000, -1, 10904, true, 0.f, 90.f, pitch) && pitch == 0.f);
  // Flipping direction does not change the measured angle at the reference.
  assert(encoder_pitch_deg(2712, -1, 2712, true, 0.f, 90.f, pitch) && pitch == 90.f);
  // A 90-degree calibration takes precedence over a saved legacy zero.
  assert(encoder_pitch_deg(2712, 8000, 2712, false, 0.f, 90.f, pitch) && pitch == 90.f);
  assert(encoder_pitch_deg(15000, 15000, -1, false, 0.f, 90.f, pitch) && pitch == 0.f);
  assert(encoder_pitch_deg(2712, 15000, -1, false, 0.f, 90.f, pitch) && pitch == 90.f);
  assert(!encoder_pitch_deg(2712, -1, -1, false, 0.f, 90.f, pitch));
  assert(!encoder_pitch_deg(6144, 0, -1, false, 0.f, 90.f, pitch));

  EncoderPitchHistory<4> history;
  assert(!history.at(1000000, pitch));
  history.record(1000000, 0.f);
  history.record(1010000, 10.f);
  assert(history.at(1005000, pitch) && fabsf(pitch - 5.f) < 1e-5f);
  assert(history.at(1020000, pitch) && pitch == 10.f);
  assert(!history.at(1040001, pitch)); // no indefinite hold
  assert(!history.at(999999, pitch)); // no future-only sample
  history.record(1070000, 70.f);
  assert(!history.at(1040000, pitch)); // missing samples cannot bridge 60 ms
  history.reset();
  assert(!history.at(1010000, pitch));
}

static void test_delayed_rotating_camera() {
  CameraHistory<1024> history;
  const Vector3f target(150, 30, -100);
  float wrong_max = 0.f, aligned_max = 0.f;
  for (int i = 0; i < 500; ++i) {
    const uint64_t now = 1000000 + uint64_t(i) * 10000;
    const float current_pitch = .5f + .4f * sinf(i * .015f);
    history.record(now, current_pitch); // radians in this generic history test
    if (i < 8) { continue; }
    uint64_t exposure;
    assert(exposure_time(now, 80000, exposure));
    float old_pitch;
    assert(history.at(exposure, old_pitch));
    const float t = (exposure - 1000000) * 1e-6f;
    const Vector3f own(12.f * t, -4.f * t, -2.f * t);
    const Dcmf body(Eulerf(.1f * sinf(t), -.2f, .15f * t));
    const Dcmf mount(Eulerf(.1f, old_pitch, -.05f));
    const Vector3f ray = (body * mount).transpose() * (target - own).normalized();
    const float range = (target - own).norm();
    const Vector3f aligned = own + body * mount * ray * range;
    const Vector3f wrong = own + body * Dcmf(Eulerf(.1f, current_pitch, -.05f)) * ray * range;
    aligned_max = max(aligned_max, (aligned - target).norm());
    wrong_max = max(wrong_max, (wrong - target).norm());
  }
  assert(aligned_max < .0001f && wrong_max > 3.f);
  printf("camera 80 ms delay, moving body/mount: reconstruction max aligned %.6f m, current-angle %.3f m\n",
      double(aligned_max), double(wrong_max));
}

static void test_predictive_geometry() {
  const Vector3f axis(1, 0, 0), own(40, 0, 0), target(10, 0, 0), nominal(40, 0, 0);
  const Vector3f relative(90, 5, -4);
  const auto plan = correct_predicted_miss(relative, own, target, nominal, axis, .5f, 5, .26f, 8);
  assert(plan.valid && fabsf(plan.time_to_plane - 3.f) < 1e-6f);
  assert(plan.correction(1) > 0 && plan.correction(2) < 0);
  assert(fabsf(plan.velocity.norm() - nominal.norm()) < 1e-4f);
  const auto straight = correct_predicted_miss(Vector3f(90,0,0), own, target, nominal, axis, .5f, 5, .26f, 8);
  assert(straight.valid && straight.correction.norm() < 1e-5f);
  for (float speed : {25.f, 55.f}) {
    const auto changing_speed = correct_predicted_miss(relative, own, target,
        Vector3f(speed,0,0), axis, .5f, 10, .26f, 8);
    assert(changing_speed.valid);
    assert(fabsf(changing_speed.miss * axis) < .002f); // model arrival reaches plane
    assert(speed > 40 ? changing_speed.time_to_plane < 3 : changing_speed.time_to_plane > 3);
  }
  const Dcmf rotation(Eulerf(.5f, -.4f, .7f));
  const auto rotated = correct_predicted_miss(rotation * relative, rotation * own, rotation * target,
      rotation * nominal, rotation * axis, .5f, 5, .26f, 8);
  assert(rotated.valid && (rotated.velocity - rotation * plan.velocity).norm() < .0001f);
  const auto bounded = correct_predicted_miss(Vector3f(3,100,-100), own, target, nominal, axis, .5f, 5, .26f, 8);
  assert(bounded.valid && bounded.velocity.isAllFinite());
  assert(bounded.correction.norm() <= 8.f * bounded.time_to_plane + 1e-4f);
  assert(acosf(clamp(bounded.velocity.normalized() * axis, -1, 1)) < .26f + 1e-4f);
  const auto tiny = correct_predicted_miss(Vector3f(.00001f,1,-1), own, target, nominal, axis, .5f, 5, .26f, 8);
  assert(tiny.valid && tiny.velocity.isAllFinite() && tiny.correction.norm() < .0001f);
  for (const Vector3f &bad : {Vector3f(-1,0,0), Vector3f(0,0,0), Vector3f(151,1,0), Vector3f(NAN,0,0)}) {
    const auto p = correct_predicted_miss(bad, own, target, nominal, axis, .5f, 5, .26f, 8);
    assert(!p.valid && (p.velocity - nominal).norm() < 1e-6f);
  }
  assert(!correct_predicted_miss(relative, Vector3f(9,0,0), target, nominal, axis, .5f,5,.26f,8).valid);
  assert(!correct_predicted_miss(relative, own, target, nominal, axis, NAN,5,.26f,8).valid);
  assert(!correct_predicted_miss(relative, own, target, Vector3f{}, axis, .5f,5,.26f,8).valid);
  assert(!correct_predicted_miss(relative, own, target, nominal, Vector3f(2,0,0), .5f,5,.26f,8).valid);
  assert(!correct_predicted_miss(relative, own, target, nominal, axis, .5f,5,0,8).valid);
}

static void test_three_dimensional_closest_approach() {
  const Vector3f relative(90.f, 5.f, -4.f), own(40.f, 0.f, 0.f);
  const Vector3f target(10.f, 0.f, 0.f);
  const auto point = predict_closest_approach(relative, own, target, own, .5f, 5.f);
  assert(point.valid && fabsf(point.time_s - 3.f) < .003f);
  assert((point.miss - Vector3f(0.f, 5.f, -4.f)).norm() < .01f);
  assert(fabsf(point.relative_speed_m_s - 30.f) < .001f);
  assert((point.approach_axis - Vector3f(1.f, 0.f, 0.f)).norm() < .001f);
  assert(!predict_closest_approach(relative, own, target, own, .5f, 2.f).valid);
  assert(!predict_closest_approach(relative, -own, target, -own, .5f, 5.f).valid);
  // A target that can be entered from any direction is evaluated at its true
  // minimum separation, which can occur before a north-facing plane crossing.
  const auto crossing = predict_closest_approach(Vector3f(10.f, 5.f, 0.f),
      Vector3f(5.f, 0.f, 0.f), Vector3f(0.f, 1.f, 0.f),
      Vector3f(5.f, 0.f, 0.f), .5f, 4.f);
  assert(crossing.valid && fabsf(crossing.time_s - 45.f / 26.f) < .003f);
  assert(fabsf(crossing.miss.norm() - 6.8640647f) < .01f);
  const Dcmf rotation(Eulerf(.5f, -.4f, .7f));
  const auto rotated = predict_closest_approach(rotation * relative, rotation * own,
      rotation * target, rotation * own, .5f, 5.f);
  assert(rotated.valid && fabsf(rotated.time_s - point.time_s) < .003f);
  assert((rotated.miss - rotation * point.miss).norm() < .01f);
  assert((rotated.approach_axis - rotation * point.approach_axis).norm() < .001f);
  assert(!predict_closest_approach(relative, own, target, Vector3f(NAN, 0.f, 0.f), .5f, 5.f).valid);
}

static void test_dynamic_image_translation() {
  const Vector3f forward(1.f, 0.f, 0.f);
  const Vector3f right = Vector3f(10.f, 1.f, 0.f).normalized();
  const Vector3f left = Vector3f(10.f, -1.f, 0.f).normalized();
  const Vector3f static_error = image_pointing_error(right, forward, false);
  const Vector3f dynamic_right = image_pointing_error(right, forward, true);
  const Vector3f dynamic_left = image_pointing_error(left, forward, true);
  assert(static_error.norm() < 1e-5f);
  assert(dynamic_right(1) < -.09f && dynamic_left(1) > .09f);
  // Velocity correction is opposite the pointing error: right target -> east.
  const Vector3f east_velocity = -dynamic_right;
  assert(east_velocity(1) > .09f);
  const Vector3f camera_right(0.f, 1.f, 0.f);
  assert((image_pitch_assist(east_velocity, right, camera_right, 0.f) - east_velocity).norm() < 1e-5f);

  const Vector3f above = Vector3f(10.f, 0.f, -1.f).normalized();
  const Vector3f pitch_velocity = -image_pointing_error(above, forward, true);
  assert(pitch_velocity(2) < -.09f);
  assert(image_pitch_assist(pitch_velocity, above, camera_right, 0.f).norm() < 1e-5f);
  assert((image_pitch_assist(pitch_velocity, above, camera_right, 1.f) - pitch_velocity).norm() < 1e-5f);
  // Image-right remains well-defined with an upward-facing camera.
  const Vector3f upward_forward(.07f, 0.f, -.99755f);
  const Vector3f near_zenith_target = Vector3f(.07f, .035f, -.99694f).normalized();
  const Vector3f upward_correction = -image_pointing_error(near_zenith_target, upward_forward, true);
  const Vector3f kept = image_pitch_assist(upward_correction, near_zenith_target, camera_right, 0.f);
  assert(kept(1) > .02f);
  const Vector3f mixed(0.f, 2.f, -6.f);
  const Vector3f pitch_first = limit_image_velocity(
      image_pitch_assist(mixed, forward, camera_right, 0.f), 1.f);
  const Vector3f cap_first = image_pitch_assist(
      limit_image_velocity(mixed, 1.f), forward, camera_right, 0.f);
  assert(fabsf(pitch_first(1) - 1.f) < 1e-5f);
  assert(cap_first(1) < .32f);
  assert(fabsf(pitch_rate_assist(.1f, 5.f, 5.f) - .1f) < 1e-6f);
  assert(fabsf(pitch_rate_assist(.1f, 10.f, 5.f) - .5f) < 1e-6f);
  assert(fabsf(pitch_rate_assist(.1f, -15.f, 5.f) - 1.f) < 1e-6f);
  assert(fabsf(pitch_rate_assist(.3f, NAN, 5.f) - .3f) < 1e-6f);
}

static void test_early_vertical_capture() {
  MissCorrection candidate;
  candidate.valid = true;
  candidate.time_to_plane = 1.3f;
  candidate.miss = Vector3f(2.f, .1f, 4.f);
  candidate.correction = Vector3f(2.f, .1f, 3.8f);
  const float up_bias = soft_capture_vertical_delta(candidate, 10.f, 8.f);
  assert(up_bias > 1.f && up_bias < 2.f);
  candidate.miss(2) = -4.f;
  candidate.correction(2) = -3.8f;
  assert(fabsf(soft_capture_vertical_delta(candidate, 10.f, 8.f) + up_bias) < 1e-6f);
  candidate.correction(2) = 3.8f;
  assert(soft_capture_vertical_delta(candidate, 10.f, 8.f) == 0.f);
  candidate.miss(2) = 20.f;
  assert(soft_capture_vertical_delta(candidate, 0.f, 8.f) <= 2.f);
  candidate.time_to_plane = 3.f;
  assert(soft_capture_vertical_delta(candidate, 10.f, 8.f) == 0.f);
  candidate.time_to_plane = 1.f;
  assert(soft_capture_vertical_delta(candidate, NAN, 8.f) == 0.f);
  VerticalCaptureLatch latch;
  assert(latch.update(1000000, 10, up_bias) == 0.f);
  assert(latch.update(1190000, 12, up_bias) == 0.f);
  assert(latch.update(1200000, 12, up_bias) == up_bias);
  assert(latch.update(1210000, 13, -up_bias) == 0.f);
  assert(latch.update(1410000, 15, -up_bias) == -up_bias);
  assert(latch.update(1420000, 15, 0.f) == 0.f);
  assert(latch.update(1430000, 16, up_bias) == 0.f);
  assert(latch.update(1000000, 17, up_bias) == 0.f);
}

// Separate plant: held velocity command followed by first-order vehicle response.
// Use the production accel/jerk planner and continuous segment closest approach;
// no collision, perfect target state, or camera dynamics are asserted for HIL.
static float run_approach(bool predictive, float plant_tau, const Vector3f &initial_relative,
    const Vector3f &target_velocity, const Vector3f &initial_velocity, float response_tau = .5f,
    bool committed_plane = false) {
  Vector3f relative = initial_relative, velocity = initial_velocity;
  Vector3f command = initial_velocity, accel;
  const Limits limits{45,45,25,25};
  const Dcmf camera(Eulerf(0,0,0));
  float minimum = relative.norm();
  int corrected = 0;
  for (int i=0; i<1600; ++i) {
    const Vector3f axis = committed_plane ? initial_relative.normalized() : relative.normalized();
    const float budget = closing_budget(target_velocity, axis, limits, 60);
    Vector3f nominal = constrained(target_velocity + axis * budget, limits);
    if (predictive) {
      const auto correction = correct_predicted_miss(relative, velocity, target_velocity,
          nominal, axis, response_tau, 5, .261799f, 8);
      if (correction.valid) { nominal = constrained(correction.velocity, limits); ++corrected; }
    }
    const auto plan = plan_velocity(nominal, command, accel, limits, .01f, 8, 8,
        Vector3f(1,0,0), Vector3f{}, camera, 1000, .5f,.4f,.65f,false,false);
    assert(feasible(plan.velocity, limits));
    if (!plan.limits_override) {
      assert((plan.velocity-command).norm() <= .0801f);
      assert((plan.acceleration-accel).norm() <= .0821f); // planner float tolerance
    }
    command = plan.velocity; accel = plan.acceleration;
    velocity += (command - velocity) * (1.f - expf(-.01f / plant_tau));
    const Vector3f step = (target_velocity - velocity) * .01f;
    const float fraction = clamp(-(relative * step) / max(step.norm_squared(),1e-8f), 0, 1);
    minimum = min(minimum, (relative + step * fraction).norm());
    relative += step;
    // Evaluate only the first approach, without allowing a second loop around.
    if (i > 20 && relative * (velocity-target_velocity) < 0) { break; }
  }
  if (predictive) { assert(corrected > 0); }
  return minimum;
}

int main() {
  test_estimator_sample_alignment();
  test_fresh_state_history();
  test_camera_history();
  test_encoder_pitch_alignment();
  test_delayed_rotating_camera();
  test_predictive_geometry();
  test_three_dimensional_closest_approach();
  test_dynamic_image_translation();
  test_early_vertical_capture();
  for (float tau : {.2f, .4f, .7f}) {
    const Vector3f relative(150, 20, -70), target(10, 4, 0), own(38, 3, -8);
    const float baseline = run_approach(false, tau, relative, target, own);
    const float corrected = run_approach(true, tau, relative, target, own);
    const float fitted = run_approach(true, tau, relative, target, own, .5f + tau);
    printf("3D lagged approach, plant tau %.1f s: baseline %.4f m, default prediction %.4f m, matched response %.4f m\n",
        double(tau), double(baseline), double(corrected), double(fitted));
    assert(corrected < baseline);
    assert(fitted < baseline);
  }
  const Vector3f relative(80,10,-40), target(10,4,0), own(38,3,-8);
  const float terminal_before = run_approach(false, .4f, relative, target, own, .5f, true);
  const float terminal_after = run_approach(true, .4f, relative, target, own, .5f, true);
  printf("fixed terminal plane: baseline %.4f m, predictive %.4f m\n",
      double(terminal_before), double(terminal_after));
  assert(terminal_after < terminal_before);
  puts("Predictive correction and camera alignment tests passed");
}
