// Log 94 regression: production PositionControl + independent thrust/drag plant.
#include "RelativeGuidance.hpp"
#include "../mc_pos_control/PositionControl/PositionControl.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace dock_motion;

struct Result { float miss{INFINITY}, time{0.f}, speed{0.f}; };

static Result simulate(bool adaptive, float drag_scale = 1.f, float attitude_time = .1f,
    Vector3f target_velocity = Vector3f{}, bool body_linear_drag = false, bool noisy_observations = false) {
  PositionControl controller;
  controller.setPositionGains(Vector3f(1.f, 1.f, 1.f));
  controller.setVelocityGains(Vector3f(2.6f, 2.6f, 4.f), Vector3f(.6f, .6f, 2.f), Vector3f(.2f, .2f, 0.f));
  controller.setVelocityLimits(100.f, 40.f, 4.f);
  controller.setThrustLimits(.05f, 1.f);
  controller.setHorizontalThrustMargin(.8f); // actual log_94 parameter, deliberately retained
  controller.setTiltLimit(1.55f);
  controller.setHoverThrust(.36f);
  controller.decoupleHorizontalAndVecticalAcceleration(false);
  RelativeContext c;
  c.limits = {50.f, 100.f, 40.f, 4.f};
  c.up = 4.f;
  c.down = 3.f;
  c.soft = 100.f; // isolate the measured propulsion limitation; FOV has separate regressions
  c.ray = Vector3f(1.f, 0.f, 0.f);
  c.tolerance = .25f;
  Vector3f position{}, velocity{}, acceleration{}, command{}, command_accel{};
  Vector3f actual_thrust(0.f, 0.f, -.36f), target(150.f, 0.f, -98.f);
  RelativeTrajectory cached;
  RelativeResponse response;
  RelativeMotion motion;
  struct Frame { Vector3f position{}, velocity{}, relative{}; } history[8];
  Result result;
  for (int i = 0; i < 6000; ++i) {
    if (i % 5 == 0) {
      c.relative = target - position;
      c.relative_rate = target_velocity - velocity;
      if (noisy_observations) {
        const int frame_number = i / 5;
        history[frame_number % 8] = {position, velocity, target - position};
        const uint64_t now = 1000000 + i * 4000;
        if (frame_number >= 5) {
          const auto &frame = history[(frame_number - 5) % 8];
          const Vector3f noise(0.f, .05f * sinf(frame_number * .79f), .05f * cosf(frame_number * .63f));
          motion.update(now - 100000, frame.position, frame.velocity,
              frame.relative * (1.f + .01f * sinf(frame_number * .47f)) + noise,
              max(.2f, .15f * frame.relative.norm()), .002f, 3.f);
        }
        if (motion.fresh(now)) {
          c.relative = motion.relative(now, position);
          c.relative_rate = motion.rate(velocity);
        }
      }
      c.own_velocity = velocity;
      c.previous = command;
      c.previous_acceleration = command_accel;
      if (adaptive) {
        assert(response.update(1000000 + i * 4000, velocity, acceleration, command, actual_thrust, .36f, .5f));
        c.response_valid = true;
        c.response_bias = response.bias();
        c.response_uncertainty = response.uncertainty();
        c.drag_coefficient = response.drag(); // never inject plant's true coefficients into guidance
        c.linear_drag = response.linear_drag();
        c.thrust_acceleration = 9.80665f / .36f;
        c.horizontal_force_margin = .8f * c.thrust_acceleration;
      }
      if (i % 15 == 0) { cached = plan_relative_intercept(c); }
      const Vector3f desired = noisy_observations && !motion.ready(1000000 + i * 4000)
          ? c.relative.normalized() * max(3.f, command.norm()) : cached.desired;
      const auto plan = relative_step(c, desired, command, command_accel, .02f, true);
      command = plan.velocity;
      command_accel = plan.acceleration;
      assert(feasible(command, c.limits));
    }
    auto input = PositionControl::empty_trajectory_setpoint;
    command.copyTo(input.velocity);
    input.yaw = 0.f;
    controller.setState({position, velocity, acceleration, 0.f});
    controller.setInputSetpoint(input);
    assert(controller.update(.004f));
    vehicle_local_position_setpoint_s output{};
    controller.getLocalPositionSetpoint(output);
    assert(Vector3f(output.thrust).norm() <= 1.0001f);
    // Independent 250 Hz integration of thrust direction lag, gravity and
    // quadratic resistance. The prediction's velocity response is not used.
    actual_thrust += (Vector3f(output.thrust) - actual_thrust) * (.004f / attitude_time);
    Vector3f drag = Vector3f(velocity(0) * fabsf(velocity(0)) * .025f,
        velocity(1) * fabsf(velocity(1)) * .025f, velocity(2) * fabsf(velocity(2)) * .04f) * drag_scale;
    if (body_linear_drag) {
      // H7.xml uses body-axis linear resistance: 1.58/1.09/1.09 N/(m/s), mass 1.9 kg.
      const matrix::Dcmf rotation(matrix::Quatf(Vector3f(0.f, 0.f, 1.f), -actual_thrust.normalized()));
      drag = rotation * ((rotation.transpose() * velocity).emult(Vector3f(1.58f, 1.09f, 1.09f) / 1.9f)) * drag_scale;
    }
    acceleration = actual_thrust * (9.80665f / .36f) + Vector3f(0.f, 0.f, 9.80665f) - drag;
    velocity += acceleration * .004f;
    position += velocity * .004f;
    target += target_velocity * .004f;
    const float miss = (target - position).norm();
    if (miss < result.miss) { result = {miss, i * .004f, velocity.norm()}; }
    if ((target - position)(0) < -10.f) { break; }
  }
  printf("response=%d body_linear=%d observations=%d drag=%.2f attitude=%.2f target=(%.1f,%.1f): miss=%.3f m, time=%.3f s, speed=%.3f m/s\n",
      int(adaptive), int(body_linear_drag), int(noisy_observations), double(drag_scale), double(attitude_time), double(target_velocity(0)), double(target_velocity(1)),
      double(result.miss), double(result.time), double(result.speed));
  return result;
}

static void test_response_guards() {
  RelativeResponse response;
  const Vector3f velocity(10.f, 0.f, -5.f), acceleration{}, command = velocity;
  const Vector3f thrust = (Vector3f(2.5f, 0.f, -1.f) - Vector3f(0.f, 0.f, 9.80665f)) * (.36f / 9.80665f);
  for (uint64_t time = 1000000; time <= 5000000; time += 20000) {
    assert(response.update(time, velocity, acceleration, command, thrust, .36f, .5f));
  }
  // One constant speed cannot distinguish linear and quadratic resistance.
  assert(fabsf(response.linear_drag()(0) + response.drag()(0) * 10.f - .25f) < .001f);
  assert(fabsf(response.linear_drag()(2) + response.drag()(2) * 5.f - .2f) < .001f);
  assert(response.fresh(5100000) && !response.fresh(5150001));
  assert(!response.update(4999999, velocity, acceleration, command, thrust, .36f, .5f));
  assert(!response.update(5020000, velocity, Vector3f(NAN, 0.f, 0.f), command, thrust, .36f, .5f));
  assert(!response.update(5020000, velocity, acceleration, command, thrust, NAN, .5f));
  response.reset();
  assert(!response.fresh(5100000));
}

static void test_frozen_response() {
  RelativeContext c;
  c.relative = Vector3f(20.f, 0.f, -10.f);
  c.own_velocity = Vector3f(30.f, 0.f, -10.f);
  c.relative_rate = -c.own_velocity;
  c.previous = Vector3f(30.f, 0.f, -30.f);
  const auto nominal = relative_frozen_approach(c, c.previous);
  c.response_valid = true;
  c.thrust_acceleration = 9.80665f / .36f;
  c.horizontal_force_margin = .8f * c.thrust_acceleration;
  c.drag_coefficient = Vector3f(.025f, .025f, .04f);
  const auto limited = relative_frozen_approach(c, c.previous);
  assert(nominal.valid && limited.valid);
  assert((limited.miss - nominal.miss).norm() > 1.f);
  RelativeMotion motion;
  for (uint64_t time = 1000000; time <= 2000000; time += 100000) {
    const Vector3f own = c.own_velocity * ((time - 1000000) * 1e-6f);
    assert(motion.update(time, own, c.own_velocity, Vector3f(100.f, 0.f, -50.f) - own, .1f, .001f, .1f));
  }
  const auto exact = relative_point_evidence(motion, 2000000, limited, 0.f);
  const auto uncertain = relative_point_evidence(motion, 2000000, limited, 2.f);
  assert(exact.valid && uncertain.valid);
  assert(uncertain.miss_bound_m > exact.miss_bound_m + limited.time_s);
  assert(uncertain.time_upper_s > exact.time_upper_s);
  assert(!relative_point_evidence(motion, 2000000, limited, -1.f).valid);
  assert(!relative_point_evidence(motion, 2000000, limited, NAN).valid);
  c.thrust_acceleration = NAN;
  assert(!relative_frozen_approach(c, c.previous).valid);
}

static void test_drag_identification() {
  for (bool quadratic : {false, true}) {
    RelativeResponse response;
    for (int i = 0; i <= 500; ++i) {
      const Vector3f velocity(3.f + i * .04f, 0.f, -3.f - i * .02f);
      const Vector3f drag = quadratic
          ? Vector3f(.025f * velocity(0) * fabsf(velocity(0)), 0.f, .04f * velocity(2) * fabsf(velocity(2)))
          : velocity.emult(Vector3f(.65f, 0.f, .5f));
      const Vector3f thrust = (drag - Vector3f(0.f, 0.f, 9.80665f)) * (.36f / 9.80665f);
      assert(response.update(1000000 + i * 20000, velocity, Vector3f{}, velocity, thrust, .36f, .5f));
    }
    if (quadratic) {
      assert(response.linear_drag().norm() < .01f);
      assert(fabsf(response.drag()(0) - .025f) < .001f && fabsf(response.drag()(2) - .04f) < .001f);
    } else {
      assert(response.drag().norm() < .001f);
      assert(fabsf(response.linear_drag()(0) - .65f) < .01f && fabsf(response.linear_drag()(2) - .5f) < .01f);
    }
    response.reset();
    assert(response.linear_drag().norm() == 0.f && response.drag().norm() == 0.f);
  }
}

int main() {
  test_response_guards();
  test_frozen_response();
  test_drag_identification();
  const Result baseline = simulate(false), corrected = simulate(true);
  assert(baseline.miss > 8.f);
  assert(corrected.miss < .6f && corrected.speed > 25.f && corrected.time < baseline.time + .5f);
  for (float drag : {.7f, 1.3f}) {
    const auto result = simulate(true, drag);
    assert(result.miss < .8f && result.speed > 20.f && result.time < 12.f);
  }
  for (float lag : {.06f, .15f}) { assert(simulate(true, 1.f, lag).miss < .8f); }
  assert(simulate(true, 1.f, .1f, Vector3f(10.f, 0.f, 0.f)).miss < 1.f);
  assert(simulate(true, 1.f, .1f, Vector3f(0.f, 5.f, 0.f)).miss < 1.f);
  const auto h7 = simulate(true, 1.f, .1f, Vector3f{}, true);
  assert(h7.miss < .5f && h7.speed > 30.f && h7.time < 9.f);
  for (float drag : {.7f, 1.f, 1.3f}) {
    assert(simulate(true, drag, .1f, Vector3f{}, true, true).miss < .8f);
  }
  assert(simulate(true, 1.f, .15f, Vector3f(10.f, 0.f, 0.f), true, true).miss < 1.f);
  assert(simulate(true, 1.f, .15f, Vector3f(0.f, 5.f, 0.f), true, true).miss < 1.f);
  puts("Relative response and production PositionControl regressions passed");
}
