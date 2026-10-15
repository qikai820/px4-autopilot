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

static void test_terminal_fresh_frame_deadline() {
  RelativeMotion motion;
  const Vector3f target(20.f, 0.f, 0.f), velocity(20.f, 0.f, 0.f);
  uint64_t now = 1000000;
  Vector3f own{};
  for (int i = 0; i < 8; ++i, now += 100000) {
    own = velocity * (.1f * i);
    assert(motion.update(now, own, velocity, target - own, .4f, .002f, 3.f));
  }
  const uint64_t committed = now - 100000;
  const uint64_t first_deadline = relative_terminal_deadline(
      committed, committed, motion.age(committed), .5f, 1.5f);
  own += velocity * .1f;
  assert(motion.update(now, own, velocity, target - own, .4f, .002f, 3.f));
  const uint64_t refreshed_deadline = relative_terminal_deadline(
      committed, now, motion.age(now), .5f, 1.5f);
  assert(refreshed_deadline > first_deadline);
  assert(refreshed_deadline <= committed + 1500000);
  assert((motion.relative(now, own) - (target - own)).norm() < .1f);
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

static void test_preposition_turn_radius() {
  const Vector3f velocity(20.f, 0.f, 0.f);
  const Vector3f acceleration(1.f, 8.f, -2.f);
  const Vector3f limited = limit_preposition_turn_acceleration(velocity, acceleration, 150.f);
  assert(fabsf(limited(0) - 1.f) < .001f);
  assert(fabsf(limited(1) - 400.f / 150.f) < .001f);
  assert(fabsf(limited(2) + 2.f) < .001f);
  assert((limit_preposition_turn_acceleration(velocity, acceleration, 0.f) - acceleration).norm() < .001f);
  assert((limit_preposition_turn_acceleration(Vector3f(1.f, 0.f, 0.f), acceleration, 150.f) -
      acceleration).norm() < .001f);
  const Vector3f right_turn = limit_preposition_turn_acceleration(velocity,
      Vector3f(0.f, -8.f, 0.f), 150.f);
  assert(fabsf(right_turn(1) + 400.f / 150.f) < .001f);
}

static void test_preposition_guidance_clearance() {
  // The logged 99 m target range is irrelevant while the rear reference is
  // still about 198 m away. Arrival alone cannot authorize visual handoff.
  assert(!preposition_reference_arrived(198.f, 25.f, true, true, false));
  assert(preposition_reference_arrived(24.f, 25.f, true, true, false));
  assert(!preposition_reference_arrived(24.f, 25.f, false, true, false));
  assert(!preposition_reference_arrived(24.f, 25.f, true, true, true));
  // Front approach can hand off on stable vision before the stand-off point;
  // rear approach keeps the reference-arrival gate.
  assert(preposition_visual_handoff_ready(false, false, true, true, false, true));
  assert(!preposition_visual_handoff_ready(true, false, true, true, false, true));
  assert(preposition_visual_handoff_ready(true, true, true, true, false, true));
  assert(!preposition_visual_handoff_ready(false, false, false, true, false, true));
  assert(!preposition_visual_handoff_ready(false, false, true, false, false, true));
  assert(!preposition_visual_handoff_ready(false, false, true, true, true, true));
  assert(!preposition_visual_handoff_ready(false, false, true, true, false, false));
  const Vector3f target(0.f, 0.f, 0.f), rear(100.f, 0.f, 0.f);
  Vector3f own(-101.f, 18.f, 0.f), guidance{};
  bool bypass = false;
  int side = 0;
  assert(preposition_segment_clearance_xy(own, rear, target) < 20.f);
  for (int i = 0; i < 30; ++i) {
    assert(preposition_guidance_reference(own, target, rear, 50.f, 250.f,
        side, guidance, bypass));
    assert(preposition_segment_clearance_xy(own, guidance, target) >= 50.f);
    if (!bypass) { break; }
    own = guidance;
  }
  assert(!bypass);
  assert((guidance - rear).norm() < .001f);
  assert(side != 0);

  own = Vector3f(-40.f, 0.f, 0.f);
  assert(!preposition_guidance_reference(own, target, rear, 50.f, 250.f,
      side, guidance, bypass));
  assert(!preposition_clearance_braking_needed(101.f, 0.f, 50.f, 3.f, .5f));
  assert(preposition_clearance_braking_needed(101.f, 20.f, 50.f, 3.f, .5f));
  assert(preposition_clearance_braking_needed(49.f, 0.f, 50.f, 3.f, .5f));
}

static void test_moving_target_rear_bypass() {
  Vector3f own(-800.f, 0.f, 0.f), velocity{}, target{}, guidance{};
  const Vector3f target_velocity(-10.f, 0.f, 0.f);
  RelativeContext context;
  context.limits = {50.f, 100.f, 40.f, 4.f};
  context.acceleration = 8.f;
  context.deceleration = 3.f;
  context.up = 4.f;
  context.down = 3.f;
  context.response = .5f;
  int side = 0;
  bool bypass = false, reached = false, saw_bypass = false;
  float minimum_clearance = INFINITY;
  bool settled = false;
  constexpr float dt = .02f;
  for (int i = 0; i < 6000; ++i) {
    target += target_velocity * dt;
    const Vector3f rear = target + Vector3f(100.f, 0.f, 0.f);
    assert(preposition_guidance_reference(own, target, rear, 50.f, 250.f,
        side, guidance, bypass));
    saw_bypass |= bypass;
    const Vector3f to_target = target - own;
    const float current_range = matrix::Vector2f(to_target(0), to_target(1)).norm();
    const float closing = current_range > .01f
        ? (to_target * (velocity - target_velocity)) / current_range : 0.f;
    if (!bypass && preposition_clearance_braking_needed(
            current_range, closing, 50.f, 3.f, .5f)) {
      assert(current_range > 60.f);
      guidance = own;
      bypass = settled = true;
    }
    const Vector3f error = guidance - own;
    const float distance = error.norm();
    const float relative_speed = (velocity - target_velocity).norm();
    const float speed = min(.35f * distance,
        rear_stop_speed(distance, relative_speed, context));
    Vector3f requested = target_velocity;
    if (distance > .01f) { requested += error * (speed / distance); }
    const Vector3f change = requested - velocity;
    const float acceleration_limit = requested.norm() < velocity.norm() ? 3.f : 8.f;
    velocity += change.norm() > acceleration_limit * dt
        ? change * (acceleration_limit * dt / change.norm()) : change;
    own += velocity * dt;
    const float actual_clearance = matrix::Vector2f(own(0) - target(0), own(1) - target(1)).norm();
    minimum_clearance = min(minimum_clearance, actual_clearance);
    if ((rear - own).norm() < 25.f) { reached = true; break; }
  }
  printf("moving rear bypass: min clearance %.1f m, settled=%d, reached=%d\n",
      (double)minimum_clearance, (int)settled, (int)reached);
  assert(saw_bypass && settled && reached && minimum_clearance >= 50.f);
}

static void test_any_direction_preposition_standoff() {
  const Vector3f target(500.f, 100.f, -50.f);
  Vector3f reference{};
  for (const Vector3f &own : {Vector3f(0.f, 100.f, -60.f),
                              Vector3f(1000.f, 100.f, -60.f),
                              Vector3f(500.f, -400.f, -60.f),
                              Vector3f(500.f, 600.f, -60.f)}) {
    assert(approach_line_standoff(own, target, 100.f, 10.f, reference));
    const matrix::Vector2f to_target(target(0) - own(0), target(1) - own(1));
    const matrix::Vector2f to_reference(reference(0) - own(0), reference(1) - own(1));
    assert(to_target * to_reference > 0.f);
    assert(fabsf(matrix::Vector2f(target(0) - reference(0), target(1) - reference(1)).norm() -
                 100.f) < .001f);
    assert(fabsf(reference(2) + 60.f) < .001f);
  }

  const Vector3f close(490.f, 100.f, -45.f);
  assert(approach_line_standoff(close, target, 100.f, 10.f, reference));
  assert((reference - close).norm() < .001f); // hold before visual acquisition
  assert(!approach_line_standoff(close, Vector3f(NAN, 0.f, 0.f), 100.f, 0.f, reference));
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
        const bool stable = cached.valid && cached.closest.valid && !plan.limits_override &&
            evidence.valid && evidence.miss_bound_m < 3.f &&
            evidence.time_upper_s > 0.f && evidence.time_upper_s <= 1.5f &&
            point.relative_speed_m_s > .5f;
        const bool window = relative_terminal_window(evidence, motion.age(now),
            point.relative_speed_m_s, 2.f, 3.f, .35f, .5f, 1.5f);
        if (relative_terminal_confirmed(now, confirmation, stable, window, .05f)) {
          result.committed = true;
          pass_axis = point.approach_axis;
        }
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

  assert(relative_terminal_deadline(1000000, 1000000, .02f, .5f, 1.5f) == 1480000);
  assert(relative_terminal_deadline(1000000, 1200000, .01f, .5f, 1.5f) == 1690000);
  assert(relative_terminal_deadline(1000000, 2400000, .01f, .5f, 1.5f) == 2500000);
  assert(relative_terminal_deadline(1000000, 1200000, NAN, .5f, 1.5f) == 0);

  uint64_t confirmed_since = 0;
  // The encounter is stable for 60 ms, while the final blind-time window is
  // open only at the last sample. Confirm the encounter before committing.
  assert(!relative_terminal_confirmed(1000000, confirmed_since, true, false, .05f));
  assert(!relative_terminal_confirmed(1040000, confirmed_since, true, false, .05f));
  assert(relative_terminal_confirmed(1060000, confirmed_since, true, true, .05f));
  assert(!relative_terminal_confirmed(1070000, confirmed_since, false, true, .05f));
  assert(confirmed_since == 0);
  assert(!relative_terminal_confirmed(1080000, confirmed_since, true, true, .05f));

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
  test_terminal_fresh_frame_deadline();
  test_fast_visual_handoff();
  test_preposition_turn_radius();
  test_preposition_guidance_clearance();
  test_moving_target_rear_bypass();
  test_any_direction_preposition_standoff();
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
