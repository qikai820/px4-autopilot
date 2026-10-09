// Standalone deterministic regression tests; see docs/DockGuidance_relative.md.
#include "RelativeGuidance.hpp"
#include <cassert>
#include <cstdio>
#include <chrono>
#include <initializer_list>

using namespace dock_motion;

static void test_observer() {
  RelativeMotion motion;
  Vector3f own{}, velocity{}, target(150.f, 0.f, -100.f);
  uint64_t time = 1000000;
  for (int i = 0; i < 80; ++i, time += 100000) {
    // Static target, accelerating aircraft: a relative filter must compensate
    // ego acceleration without waiting for an absolute target velocity lock.
    own += velocity * .1f + Vector3f(.01f, 0.f, -.005f);
    velocity += Vector3f(.2f, 0.f, -.1f);
    assert(motion.update(time, own, velocity, target - own, .5f, .002f, 1.f));
  }
  time -= 100000;
  assert(motion.ready(time));
  assert((motion.relative(time, own) - (target - own)).norm() < .02f);
  assert((motion.rate(velocity) + velocity).norm() < .02f);
  assert(!motion.fresh(time + 250001));
  const unsigned updates = motion.updates();
  assert(!motion.update(time, own, velocity, target - own, .5f, .002f, 1.f));
  assert(!motion.update(time + 100000, own, velocity, Vector3f(1000.f, 1000.f, 1000.f), .5f, .002f, 1.f));
  assert(motion.updates() == updates);
  assert(!motion.update(time + 100000, own, velocity, Vector3f(NAN, 0.f, 0.f), .5f, .002f, 1.f));
  assert(motion.update(time + 600000, own, velocity, target - own, .5f, .002f, 1.f));
  assert(!motion.ready(time + 600000));
  motion.reset();
  assert(!motion.fresh(time));
}

struct Outcome { float miss{INFINITY}, time{0.f}, speed{0.f}; bool committed{false}, passed{false}; };

static void test_fast_visual_handoff() {
  const float visual = -33.f * 3.14159265f / 180.f;
  const float course = 15.f * 3.14159265f / 180.f;
  assert(fabsf(matrix::wrap_pi(far_course_yaw(visual, course, .03f, .30f, .65f) -
      course)) < .001f);
  assert(fabsf(matrix::wrap_pi(far_course_yaw(visual, course, .65f, .30f, .65f) -
      visual)) < .001f);
  assert(far_course_yaw(visual, course, .475f, .30f, .65f) > visual);
  assert(far_course_yaw(visual, course, .475f, .30f, .65f) < course);
  const float across_pi = far_course_yaw(3.12413936f, -3.12413936f,
      .475f, .30f, .65f);
  assert(fabsf(fabsf(across_pi) - 3.14159265f) < .01f);
  Vector3f direction{};
  const Vector3f rear(1050.f, 0.f, -100.f), target(1150.f, 0.f, -100.f);
  for (float north : {0.f, 1030.f, 1049.f, 1050.f, 1051.f, 1100.f}) {
    const Vector3f own(north, 0.f, -100.f);
    assert(far_lookahead_direction(own, rear, target, direction));
    assert(direction(0) > .99f); // no turn or stop at the rear reference
  }
  const Vector3f curved_target(1150.f, 80.f, -100.f);
  Vector3f before{}, after{};
  assert(far_lookahead_direction(Vector3f(1049.f, 0.f, -100.f), rear,
      curved_target, before));
  assert(far_lookahead_direction(Vector3f(1051.f, 0.f, -100.f), rear,
      curved_target, after));
  assert(before * after > .95f);
  assert(!far_lookahead_direction(target, rear, target, direction));
}

static void test_far_rear_approach() {
  RelativeContext c;
  c.limits = {50.f, 100.f, 40.f, 4.f};
  c.acceleration = 8.f;
  c.deceleration = 3.f;
  c.up = 4.f;
  c.down = 3.f;
  c.jerk = 8.f;
  c.response = .5f;
  c.soft = 100.f;
  c.ray = Vector3f(1.f, 0.f, 0.f);
  assert(rear_stop_speed(1000.f, 0.f, c) == 50.f);
  assert(rear_stop_speed(0.f, 0.f, c) == 0.f);

  // Independent first-order vehicle response: a 1 km rear-reference leg
  // without visual handoff should use full speed far away, then stop at the point.
  float position = 0.f, actual = 0.f;
  Vector3f command{}, acceleration{};
  bool reached_full_speed = false, arrived = false;
  for (int i = 0; i < 3000; ++i) {
    const float distance = fabsf(1000.f - position);
    c.relative = Vector3f(distance + 100.f, 0.f, 0.f);
    const float speed = min(.35f * distance,
        rear_stop_speed(distance, fabsf(actual), c));
    const float desired = position <= 1000.f ? speed : -speed;
    const auto plan = relative_step(c, Vector3f(desired, 0.f, 0.f),
        command, acceleration, .02f, false);
    command = plan.velocity;
    acceleration = plan.acceleration;
    actual += (command(0) - actual) * (.02f / c.response);
    position += actual * .02f;
    reached_full_speed |= actual > 45.f;
    if (distance < 2.f && fabsf(actual) < 1.f) {
      assert(i * .02f < 45.f);
      arrived = true;
      break;
    }
  }
  assert(reached_full_speed && arrived);
}

static Outcome scenario(const Vector3f &initial_target_velocity, bool estimated, float plant_response = .5f,
    Vector3f initial_relative = Vector3f(150.f, 0.f, -100.f), Vector3f initial_velocity = Vector3f{},
    bool stop_target = false, bool terminal = false, bool noisy_box = false) {
  RelativeContext c;
  c.limits = {50.f, 50.f, 15.f, 15.f};
  c.up = 8.f;
  c.down = 3.f;
  c.soft = 100.f; // geometric/plant regression; FOV is tested separately
  c.tolerance = .25f;
  Vector3f own{}, actual = initial_velocity, target = initial_relative, command = actual, acceleration{};
  Vector3f target_velocity = initial_target_velocity;
  struct Frame { Vector3f own{}, velocity{}, relative{}; } frames[8];
  RelativeTrajectory cached;
  uint64_t plan_time = 0, confirmation = 0;
  Vector3f pass_axis{};
  RelativeMotion motion;
  Outcome result;
  for (int i = 0; i < 1600; ++i) {
    const uint64_t now = 1000000 + i * 20000;
    if (stop_target && i == 150) { target_velocity.zero(); }
    frames[i % 8] = {own, actual, target - own};
    if (i >= 5 && !result.committed) {
      const Frame &frame = frames[(i - 5) % 8];
      const float scale = noisy_box ? 1.f + .01f * sinf(i * .47f) : 1.f;
      const Vector3f noise = noisy_box ? Vector3f(0.f, .05f * sinf(i * .79f), .05f * cosf(i * .63f)) : Vector3f{};
      motion.update(now - 100000, frame.own, frame.velocity, frame.relative * scale + noise,
          noisy_box ? max(.2f, .15f * frame.relative.norm()) : .2f, .002f, 3.f);
    }
    c.relative = estimated && motion.fresh(now) ? motion.relative(now, own) : target - own;
    c.relative_rate = estimated && motion.fresh(now) ? motion.rate(actual) : target_velocity - actual;
    c.own_velocity = actual;
    c.previous = command;
    c.previous_acceleration = acceleration;
    c.ray = c.relative.normalized();
    if (!result.committed) {
      if (!plan_time || now - plan_time >= 50000) {
        cached = plan_relative_intercept(c);
        plan_time = now;
      }
      assert(cached.valid);
      const Vector3f desired = motion.ready(now) ? cached.desired : c.relative.normalized() * max(3.f, command.norm());
      const auto plan = relative_step(c, desired, command, acceleration, .02f, true);
      if (!plan.limits_override) {
        assert((plan.acceleration - acceleration).norm() <= c.jerk * .02f + .002f);
        assert(plan.acceleration.norm() <= c.acceleration + .002f);
        assert(plan.acceleration(2) >= -c.up - .002f && plan.acceleration(2) <= c.down + .002f);
      }
      command = plan.velocity;
      acceleration = plan.acceleration;
      if (terminal) {
        const auto point = relative_frozen_approach(c, command);
        const auto evidence = relative_point_evidence(motion, now, point);
        if (relative_terminal_window(evidence, motion.age(now), point.relative_speed_m_s, 2.f, 3.f, .35f, .5f, 1.5f)) {
          if (!confirmation) { confirmation = now; }
          if (now - confirmation >= 50000) { result.committed = true; pass_axis = point.approach_axis; }
        } else { confirmation = 0; }
      }
    } else {
      const Vector3f relative = motion.relative(now, own);
      const float along = relative * pass_axis;
      const float across = (relative - pass_axis * along).norm();
      assert(motion.age(now) <= .5f);
      assert(across + 2.f * motion.sigma(pass_axis, motion.age(now), true) <= 3.f);
      if (along + 2.f * motion.sigma(pass_axis, motion.age(now), false) < -2.f) {
        result.passed = true;
        break;
      }
    }
    assert(feasible(command, c.limits));
    // Independent plant integration at 1 kHz, not planner's analytical step.
    for (int sub = 0; sub < 20; ++sub) {
      actual += (command - actual) * (.001f / plant_response);
      own += actual * .001f;
      target += target_velocity * .001f;
      const float miss = (target - own).norm();
      if (miss < result.miss) { result.miss = miss; result.time = i * .02f + sub * .001f; result.speed = actual.norm(); }
    }
    if (!result.committed && (target - own)(0) < -5.f) { break; }
  }
  printf("target=(%.1f,%.1f,%.1f), tau=%.2f, stop=%d, box=%d: miss=%.3f m time=%.3f s speed=%.3f m/s terminal=%d pass=%d\n",
      double(initial_target_velocity(0)), double(initial_target_velocity(1)), double(initial_target_velocity(2)),
      double(plant_response), int(stop_target), int(noisy_box), double(result.miss), double(result.time), double(result.speed),
      int(result.committed), int(result.passed));
  return result;
}

static void test_frozen_and_fov() {
  RelativeContext c;
  c.limits = {50.f, 50.f, 15.f, 15.f};
  c.relative = Vector3f(1.f, .1f, .1f);
  c.relative_rate = Vector3f(-40.f, 0.f, 0.f);
  c.own_velocity = c.previous = Vector3f(40.f, 0.f, 0.f);
  c.ray = Vector3f(1.f, 0.f, 0.f);
  const auto near = relative_frozen_approach(c, c.previous);
  assert(near.valid && fabsf(near.time_s - .025f) < .0001f);
  assert(fabsf(near.miss.norm() - sqrtf(.02f)) < .0001f);
  c.relative_rate = Vector3f(40.f, 0.f, 0.f);
  assert(!relative_frozen_approach(c, c.previous).valid);
  c.relative = Vector3f(1000.f, 0.f, 0.f);
  c.relative_rate = Vector3f(-40.f, 0.f, 0.f);
  assert(!relative_frozen_approach(c, c.previous).valid); // horizon endpoint is not an encounter
  PointEvidence point;
  point.valid = true;
  point.time_upper_s = .2f;
  point.miss_bound_m = .3f;
  assert(relative_terminal_window(point, .1f, 40.f, 2.f, 3.f, .35f, .5f, 1.5f));
  assert(!relative_terminal_window(point, .3f, 40.f, 2.f, 3.f, .35f, .5f, 1.5f));
  assert(!relative_terminal_window(point, .1f, .2f, 2.f, 3.f, .35f, .5f, 1.5f));
  point.miss_bound_m = 3.f;
  assert(!relative_terminal_window(point, .1f, 40.f, 2.f, 3.f, .35f, .5f, 1.5f));
  point.miss_bound_m = NAN;
  assert(!relative_terminal_window(point, .1f, 40.f, 2.f, 3.f, .35f, .5f, 1.5f));

  c.relative = Vector3f(100.f, 0.f, 0.f);
  c.previous.zero();
  c.previous_acceleration.zero();
  c.ray = Vector3f(1.f, 0.f, 0.f);
  const auto centered = relative_step(c, Vector3f(30.f, 0.f, 0.f), c.previous, c.previous_acceleration, .02f, true);
  assert(!centered.limits_override && centered.predicted_fov < c.soft);
  c.ray = Vector3f(cosf(.4f), 0.f, -sinf(.4f));
  c.ray_rate = Vector3f(0.f, 0.f, -.3f);
  const auto edge = relative_step(c, Vector3f(30.f, 0.f, 0.f), c.previous, c.previous_acceleration, .02f, true);
  assert(edge.predicted_fov > c.soft); // cannot certify an image already leaving the FOV
  assert(edge.velocity.norm() <= centered.velocity.norm() + .001f);

  c.ray = Vector3f(1.f, 0.f, 0.f);
  c.ray_rate.zero();
  c.relative = Vector3f(150.f, 0.f, -100.f);
  c.relative_rate.zero();
  c.own_velocity.zero();
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 100; ++i) { assert(plan_relative_intercept(c).valid); }
  const double elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
  printf("bounded planner host mean %.1f us (100 searches; not MCU timing)\n", elapsed / 100.);
}

int main() {
  test_observer();
  test_fast_visual_handoff();
  test_far_rear_approach();
  test_frozen_and_fov();
  for (const Vector3f &velocity : {Vector3f{}, Vector3f(10.f, 0.f, 0.f), Vector3f(-10.f, 0.f, 0.f),
      Vector3f(0.f, 5.f, 0.f), Vector3f(5.f, 3.f, -2.f)}) {
    const auto result = scenario(velocity, true);
    assert(result.miss < 1.f);
    assert(result.time < 25.f);
    assert(result.speed > 15.f);
  }
  for (float response : {.2f, .7f}) {
    const auto result = scenario(Vector3f{}, true, response);
    assert(result.miss < 1.f && result.speed > 15.f);
  }
  assert(scenario(Vector3f{}, true, .5f, Vector3f(150.f, 0.f, 50.f)).miss < 1.f);
  const auto too_late = scenario(Vector3f{}, true, .5f, Vector3f(43.f, 0.f, -30.f),
      Vector3f(28.f, 0.f, -12.8f), false, true);
  assert(too_late.miss > 3.f && !too_late.committed && !too_late.passed);
  assert(scenario(Vector3f{}, true, .5f, Vector3f(43.f, 0.f, -20.f), Vector3f(28.f, 0.f, -12.8f)).miss < 1.f);
  assert(scenario(Vector3f(10.f, 0.f, 0.f), true, .5f, Vector3f(150.f, 0.f, -100.f), Vector3f{}, true).miss < 1.f);
  for (bool noisy : {false, true}) {
    const auto result = scenario(Vector3f{}, true, .5f, Vector3f(150.f, 0.f, -100.f), Vector3f{}, false, true, noisy);
    assert(result.miss < 1.f && result.committed && result.passed);
  }
  puts("relative guidance tests passed");
}
