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
