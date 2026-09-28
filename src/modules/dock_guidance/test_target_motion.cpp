// Standalone deterministic tests. See docs/DockGuidance_dynamic.md for command.
#include "TargetMotion.hpp"
#include "PredictiveCorrection.hpp"
#include <cassert>
#include <cstdio>
#include <random>
#include <cstring>

using namespace dock_motion;

static void test_steep_climb_acceleration_direction() {
  // log_50 requested roughly (14.2, 0.72, -40) m/s from near hover.
  // An axis ceiling must reduce the magnitude, not rotate that request into
  // horizontal acceleration and immediately push the image outside the FOV.
  const Vector3f desired(14.2f, .72f, -40.f);
  const Vector3f direction = desired.normalized();
  const matrix::Dcmf camera(matrix::Eulerf(0.f,
      atan2f(-direction(2), hypotf(direction(0), direction(1))),
      atan2f(direction(1), direction(0))));
  const Limits limits{50.f, 100.f, 40.f, 4.f};
  Vector3f velocity, acceleration;
  const float dt = .02f;
  for (int i = 0; i < 100; ++i) {
    const auto plan = plan_velocity(desired, velocity, acceleration, limits, dt, 8.f, 8.f,
        Vector3f(1.f, 0.f, 0.f), Vector3f{}, camera, 440.f, .5f, .4f, .65f,
        false, false, 4.f, 3.f);
    assert(!plan.limits_override);
    assert(plan.acceleration.normalized() * direction > .99999f);
    assert(plan.acceleration(2) >= -4.002f);
    assert(plan.acceleration.norm() <= 8.002f);
    assert((plan.acceleration - acceleration).norm() <= 8.f * dt + .002f);
    velocity = plan.velocity;
    acceleration = plan.acceleration;
  }
  assert(velocity(2) < -6.5f && velocity(0) > 2.f && velocity(0) < 3.f);
  // With no separate axis restriction, preserve the original norm-limited
  // vector direction as well; the default arguments must not alter it.
  const auto unrestricted = plan_velocity(desired, Vector3f{}, Vector3f{}, limits, dt, 8.f, 8.f,
      Vector3f(1.f, 0.f, 0.f), Vector3f{}, camera, 440.f, .5f, .4f, .65f, false, false);
  assert(unrestricted.acceleration.normalized() * direction > .99999f);
  printf("log50 steep climb: after 2 s command NED (%.3f, %.3f, %.3f) m/s, direction preserved\n",
      double(velocity(0)), double(velocity(1)), double(velocity(2)));
}

static void test_vertical_acceleration_limits() {
  const matrix::Dcmf camera(matrix::Eulerf(0.f, 0.f, 0.f));
  const Limits limits{80.f, 80.f, 40.f, 10.f};
  const float dt = .02f;
  Vector3f velocity(30.f, 0.f, -30.f), acceleration;
  float peak_down = 0.f, peak_up = 0.f;
  // Horizontal acceleration and climb braking occur together in log_49.
  // The 8 m/s^2 vector budget must not override the 3 m/s^2 vertical brake.
  for (int i = 0; i < 750; ++i) {
    const Vector3f wanted = i < 375 ? Vector3f(55.f, 0.f, -5.f) : Vector3f(55.f, 0.f, -30.f);
    const auto plan = plan_velocity(wanted, velocity, acceleration, limits, dt, 8.f, 8.f,
        Vector3f(1.f, 0.f, 0.f), Vector3f{}, camera, 200.f, .5f, .4f, .65f,
        false, false, 4.f, 3.f);
    assert(!plan.limits_override);
    assert(plan.acceleration(2) <= 3.002f && plan.acceleration(2) >= -4.002f);
    assert(plan.acceleration.norm() <= 8.002f);
    assert((plan.acceleration - acceleration).norm() <= 8.f * dt + .002f);
    assert(Vector3f((plan.velocity - velocity) / dt - plan.acceleration).norm() < .0001f);
    peak_down = max(peak_down, plan.acceleration(2));
    peak_up = max(peak_up, -plan.acceleration(2));
    velocity = plan.velocity;
    acceleration = plan.acceleration;
  }
  assert(peak_down > 2.9f && peak_up > 3.9f);
  // A parameter reduction can make a hard acceleration bound incompatible
  // with jerk. Honour the bound and expose the exceptional transition.
  const auto reduced = plan_velocity(Vector3f(30.f, 0.f, 0.f), Vector3f(30.f, 0.f, -10.f),
      Vector3f(0.f, 0.f, 3.f), limits, dt, 8.f, 8.f, Vector3f(1.f, 0.f, 0.f),
      Vector3f{}, camera, 200.f, .5f, .4f, .65f, false, false, 4.f, 1.f);
  assert(reduced.limits_override && reduced.acceleration(2) <= 1.002f);
  printf("vertical planner: down %.3f, up %.3f m/s^2; reduced-limit conflict flagged\n",
      double(peak_down), double(peak_up));
}

static void test_recovery_hysteresis() {
  RecoveryHysteresis recovery;
  assert(!recovery.update(1000000, false, false, 1.f));
  assert(recovery.update(1020000, true, false, 1.f));
  // Crossing SOFT on alternate frames cannot re-enable the capture budget.
  for (uint64_t time = 1040000; time < 2000000; time += 20000) {
    assert(recovery.update(time, (time / 20000) % 2, false, 1.f));
  }
  assert(recovery.update(2000000, false, true, 1.f));
  assert(recovery.update(2900000, false, true, 1.f));
  assert(recovery.update(2920000, false, false, 1.f)); // alignment lost: restart dwell
  assert(recovery.update(2940000, false, true, 1.f));
  assert(recovery.update(3920000, false, true, 1.f));
  assert(!recovery.update(3940000, false, true, 1.f));
  assert(recovery.update(4000000, true, true, 1.f)); // hard protection has priority
  assert(recovery.update(4020000, false, true, 1.f));
  assert(recovery.update(3000000, false, true, 1.f)); // timestamp reversal restarts dwell
  assert(recovery.update(3980000, false, true, 1.f));
  assert(!recovery.update(4000000, false, true, 1.f));
  recovery.update(5000000, true, false, 1.f);
  recovery.reset();
  assert(!recovery.update(6000000, false, false, 1.f));
}

static void test_following_outside_axis_limit() {
  const Limits limits{60,100,40,4};
  const Vector3f direction(.6f,0,-.8f), following(5.955f,.197f,4.352f);
  assert(!feasible(following, limits));
  const float budget = closing_budget(following,direction,limits,60);
  const Vector3f command = following + direction*budget;
  assert(budget > 50 && feasible(command,limits) && command(2) < -39.9f);
  // A 2 mm/s change through the DOWN boundary cannot zero a valid climb.
  const float before = closing_budget(Vector3f(6,0,3.999f),direction,limits,60);
  const float after = closing_budget(Vector3f(6,0,4.001f),direction,limits,60);
  assert(before > 50 && after > 50 && fabsf(before-after) < .01f);
  assert(closing_budget(following,-direction,limits,60) < .0001f);
  assert(closing_budget(following,direction,limits,.1f) < .0001f);
  assert(closing_budget(Vector3f(80,0,0),Vector3f(-1,0,0),limits,60) > 59.99f);
  assert(closing_budget(Vector3f(0,101,0),Vector3f(0,0,-1),limits,60) < .0001f);
  // Independent grid oracle: every sampled feasible speed must be no larger
  // than the returned maximum, including rays which enter from outside.
  std::mt19937 rng(33);
  std::uniform_real_distribution<float> sample(-1.f,1.f);
  for (int trial=0; trial<600; ++trial) {
    const Vector3f base(80*sample(rng),80*sample(rng),45*sample(rng));
    const Vector3f axis = Vector3f(sample(rng),sample(rng),sample(rng)).normalized();
    const Limits bounds{40+20*sample(rng),40+20*sample(rng),20+15*sample(rng),5+4*sample(rng)};
    const float result = closing_budget(base,axis,bounds,60);
    assert(result >= 0 && result <= 60);
    if (result > 0) { assert(feasible(base+axis*result,bounds)); }
    for (int k=1; k<=2000; ++k) {
      const float speed = k*.03f;
      if (feasible(base+axis*speed,bounds)) { assert(result+.001f >= speed); }
    }
  }
  printf("log33 following allocation: closure %.3f m/s, final NED (%.3f, %.3f, %.3f) m/s\n",
      double(budget),double(command(0)),double(command(1)),double(command(2)));
}

static void test_capture_schedule() {
  assert(approach_limit(true,40,10) == 40.f);
  assert(approach_limit(false,40,10) == 10.f);
  assert(approach_limit(true,-1,10) == 0.f);
  assert(approach_limit(false,40,100) == 60.f);
  assert(airborne_state(true,false,1000000,2011000)); // normal 1 Hz heartbeat + jitter
  assert(!airborne_state(true,false,1000000,3000001));
  assert(!airborne_state(false,false,1000000,1500000));
  assert(!airborne_state(true,true,1000000,1500000));
  assert(!airborne_state(true,false,2000000,1500000));
  assert(!airborne_state(true,false,0,1500000));
  assert(capture_evidence_allowed(true,false,false,20,10));
  assert(!capture_evidence_allowed(true,false,true,5,10));
  assert(capture_evidence_allowed(true,true,true,5,10));
  assert(!capture_evidence_allowed(false,true,true,20,10));
  assert(!capture_evidence_allowed(true,true,true,NAN,10));
  assert(capture_image_evidence(false,.30f,.30f,.65f,.05f,.1f,false));
  assert(!capture_image_evidence(false,.31f,.30f,.65f,.05f,.1f,false));
  assert(!capture_image_evidence(false,.20f,.30f,.65f,.11f,.1f,false));
  assert(capture_image_evidence(false,.31f,.30f,.65f,.11f,.1f,true));
  assert(capture_image_evidence(false,.64f,.30f,.65f,.5f,.1f,true));
  assert(!capture_image_evidence(false,.65f,.30f,.65f,.05f,.1f,true));
  assert(!capture_image_evidence(true,.20f,.30f,.65f,.05f,.1f,true));
  CaptureSchedule schedule;
  assert(schedule.update(1000000,true,1,3,10,false,80,6,2,3,8) == 3.f);
  assert(schedule.evidence_duration(1000000) == 0.f);
  assert(!schedule.ready());
  assert(schedule.update(1900000,true,1,3,10,false,80,6,2,3,8) == 3.f);
  assert(fabsf(schedule.evidence_duration(1900000) - .9f) < 1e-6f);
  assert(schedule.update(2000000,true,1,3,10,false,80,6,2,3,8) == 10.f);
  assert(schedule.evidence_duration(2000000) == 1.f);
  assert(schedule.ready());
  const float uncertain = schedule.update(2100000,true,1,3,10,false,20,12,2,3,8);
  assert(uncertain == 3.f); // range/motion uncertainty must still bound capture
  const float fast = schedule.update(2200000,true,1,3,40,false,500,5,2,3,8);
  assert(fast == 40.f); // no hidden 3 or 10 m/s ceiling when configured higher
  assert(schedule.update(2300000,false,1,3,40,false,500,5,2,3,8) == 3.f);
  assert(!schedule.ready());
  assert(schedule.evidence_duration(2300000) == 0.f);
  // Terminal pass-through gives its full pursuit budget immediately, while
  // plan_velocity still enforces FOV, axis, acceleration and jerk limits.
  assert(schedule.update(2350000,false,1,60,60,true,500,5,2,3,8) == 60.f);
  assert(schedule.update(2400000,true,1,0,40,false,500,5,2,3,8) == 0.f);
  assert(!schedule.ready());
  assert(schedule.update(2500000,true,1,3,40,false,NAN,5,2,3,8) == 3.f);
  assert(!schedule.ready());
  assert(schedule.update(3000000,true,1,3,60,true,20,50,2,3,8) == 3.f);
  assert(schedule.update(4000000,true,1,3,60,true,20,50,2,3,8) == 60.f);
  assert(schedule.ready()); // pass-through ignores stopping envelope only after confirmation
  const matrix::Dcmf camera(matrix::Eulerf(0,0,0));
  Vector3f previous(1.5f,0,-1.f),acceleration;
  const Vector3f desired = Vector3f(1,0,-2).normalized()*10.f;
  for (int i=0; i<1500; ++i) {
    const auto plan=plan_velocity(desired,previous,acceleration,Limits{40,40,20,4},.01f,8,8,
        Vector3f(1,0,0),Vector3f{},camera,100,.5f,.4f,.65f,false,false);
    assert(!plan.limits_override);
    previous=plan.velocity; acceleration=plan.acceleration;
  }
  assert(previous.norm()>9.9f && previous(2)<-8.f); // no lingering 1 m/s startup Z cap
  const float diagonal_speed = previous.norm();
  previous.zero(); acceleration.zero();
  for (int i=0; i<1500; ++i) {
    const auto plan=plan_velocity(Vector3f(40,0,0),previous,acceleration,
        Limits{40,100,20,4},.01f,8,8,Vector3f(1,0,0),Vector3f{},camera,
        500,.5f,.4f,.65f,false,false);
    assert(!plan.limits_override);
    previous=plan.velocity; acceleration=plan.acceleration;
  }
  assert(previous(0)>39.9f);
  printf("capture schedule: initial 3, stable budget 40; diagonal %.2f, full %.2f m/s\n",
      double(diagonal_speed),double(previous.norm()));
}

static void test_motion(const Vector3f &target_velocity, bool own_moving) {
  TargetMotion tracker;
  std::mt19937 rng(47);
  std::normal_distribution<float> noise(0.f, .15f);
  uint64_t time = 1000000;
  Vector3f own;
  for (int i = 0; i < 200; ++i, time += 100000) {
    const float t = .1f * i;
    own = own_moving ? Vector3f(12.f * t, 5.f * sinf(.3f * t), -.5f * t) : Vector3f{};
    const Vector3f target = Vector3f(100, 25, -30) + target_velocity * t;
    const Vector3f relative = target - own + Vector3f(noise(rng), noise(rng), noise(rng));
    tracker.update(time, own, relative.normalized(), relative.norm(), .5f, .003f, 1.f, 60.f);
  }
  const float error = (tracker.velocity() - target_velocity).norm();
  printf("target=(%.1f,%.1f,%.1f), moving own=%d: velocity error %.3f m/s, sigma %.3f\n",
         double(target_velocity(0)), double(target_velocity(1)), double(target_velocity(2)),
         own_moving, double(error), double(tracker.velocity_sigma()));
  assert(error < 1.f);
  assert(tracker.valid(time, 3.f));
  assert(fabsf(tracker.radial_velocity_sigma() * tracker.radial_velocity_sigma() +
      tracker.lateral_velocity_sigma() * tracker.lateral_velocity_sigma() -
      tracker.velocity_sigma() * tracker.velocity_sigma()) < .0001f);
  assert(!tracker.valid(time + 300000, 3.f));
  const Vector3f before = tracker.velocity();
  // Duplicate timestamps and physically impossible jumps cannot create motion.
  assert(!tracker.update(time - 100000, own, Vector3f(1, 0, 0), 100, .5f, .003f, 1, 60));
  assert((tracker.velocity() - before).norm() < .00001f);
  assert(!tracker.update(time, own, Vector3f(-1, 0, 0), 1800, .5f, .003f, 1, 60));
  assert(!tracker.valid(time, 3.f));
  tracker.reset();
  assert(!tracker.valid(time, 3.f));
  assert(tracker.following_velocity(time, 3.f).norm() < .00001f);
}

static void test_turn() {
  TargetMotion tracker;
  Vector3f target(80, 30, -40), velocity(25, 0, 0);
  uint64_t time = 1000000;
  for (int i = 0; i < 300; ++i, time += 100000) {
    if (i >= 100 && i < 150) {
      const float angle = (i - 100) * .02f;
      velocity = Vector3f(25.f * cosf(angle), 25.f * sinf(angle), 0.f);
    }
    target += velocity * .1f;
    const Vector3f own = target - Vector3f(80, 30, -40);
    const Vector3f relative = target - own;
    tracker.update(time, own, relative.normalized(), relative.norm(), .5f, .002f, 3, 60);
  }
  const float error = (tracker.velocity() - velocity).norm();
  printf("turn: final velocity error %.3f m/s\n", double(error));
  assert(error < 1.f && tracker.valid(time, 3.f));
}

static void test_quantized_boxes() {
  TargetMotion tracker;
  const Vector3f velocity(28, 15, -4);
  uint64_t time = 1000000;
  for (int i = 0; i < 3000; ++i, time += 10000) {
    const float t = i * .01f;
    const Vector3f target = Vector3f(50, 20, -10) + velocity * t;
    const Vector3f own = velocity * t;
    const Vector3f relative = target - own;
    // Rotate the camera while both objects translate. Decode optical depth
    // back to slant range; camera rotation must not appear as target velocity.
    const float forward = .95f + .03f * sinf(.4f * t);
    const float width = roundf(1400.f / (relative.norm() * forward));
    const float height = roundf(1100.f / (relative.norm() * forward));
    float range, sigma;
    assert(box_range(width, height, 1, 1, 1400, 1100, forward, .15f, range, sigma));
    tracker.update(time, own, relative.normalized(), range, sigma, .002f, 3, 60);
  }
  const float error = (tracker.velocity() - velocity).norm();
  printf("quantized 100 Hz boxes: velocity error %.3f, sigma %.3f\n", double(error),
         double(tracker.velocity_sigma()));
  assert(error < 2.f);
  // Confidence gating must remain conservative even when the true error is small.
  assert(!tracker.valid(time, tracker.velocity_sigma() * .9f));
  assert(tracker.valid(time, tracker.velocity_sigma() * 1.1f));
  assert(tracker.position_sigma() >= 5.f);
  // A depth-dominated box estimate may fail the full velocity gate while a
  // short crossing-plane estimate remains bounded. Never relabel full 3-D
  // target_motion_valid merely to activate this correction.
  const Vector3f relative(50,20,-10), axis = relative.normalized();
  const Vector3f own_velocity = tracker.velocity() + axis * 40.f;
  const auto candidate = correct_predicted_miss(relative, own_velocity, tracker.velocity(),
      own_velocity, axis, .5f, 2.f, .261799f, 8.f);
  assert(candidate.valid && !tracker.valid(time, 3.f));
  const auto cross = assess_prediction(tracker, time, axis, own_velocity,
      tracker.velocity(), candidate, 2.f, 100.f, 3.f);
  assert(cross.usable && cross.reject == PredictionReject::Ready);
  assert(cross.transverse_velocity_sigma < tracker.velocity_sigma());
  const auto point = predict_closest_approach(relative, own_velocity,
      tracker.velocity(), own_velocity, .5f, 2.f);
  const auto point_evidence = assess_closest_approach(tracker, time, point);
  assert(point_evidence.valid);
  // The box's radial scale uncertainty changes encounter time, while a
  // sideways miss is bounded by uncertainty perpendicular to the flight path.
  assert(point_evidence.miss_bound_m < point.miss.norm() +
      2.f * tracker.position_sigma(tracker.age(time) + point.time_s));
  assert(assess_prediction(tracker, time, axis, own_velocity, tracker.velocity(),
      candidate, 2.f, .1f, 3.f).reject == PredictionReject::Uncertain);
  assert(assess_prediction(tracker, time + 300000, axis, own_velocity, tracker.velocity(),
      candidate, 2.f, 100.f, 3.f).reject == PredictionReject::Stale);
}

static void test_small_box_following(float speed, float yaw) {
  TargetMotion tracker;
  const matrix::Dcmf rotation(matrix::Eulerf(0.f, 0.f, yaw));
  uint64_t time = 1000000;
  float first_follow = -1.f;
  for (int i = 0; i < 80; ++i, time += 100000) {
    const float t = .1f * i;
    const Vector3f relative = rotation * Vector3f(280.f, speed * t, -20.f);
    // Exact angles with quantized 4..6 px dimensions, as in the HIL log.
    float range, sigma;
    assert(box_range(roundf(1662.77f / relative.norm()), roundf(1303.68f / relative.norm()),
                     1, 1, 1662.77f, 1303.68f, 1, .15f, range, sigma));
    tracker.update(time, Vector3f{}, relative.normalized(), range, sigma, .00154f, 3, 60);
    const Vector3f follow = rotation.transpose() * tracker.following_velocity(time, 3);
    if (first_follow < 0.f && follow(1) > 10.f) {
      first_follow = t;
    }
    assert(follow.isAllFinite() && follow.norm() <= tracker.velocity().norm() + .001f);
  }
  printf("small-box speed %.1f yaw %.1f: following >10 m/s at %.2f s; metric valid=%d\n",
         double(speed), double(yaw), double(first_follow), tracker.valid(time, 3));
  if (speed > 0.f) {
    assert(first_follow >= 1.f && first_follow <= 4.f);
    assert(tracker.motion_state() == 2);
  } else {
    assert(first_follow < 0.f && tracker.following_velocity(time, 3).norm() < .001f);
    assert(tracker.motion_state() != 2);
  }
  assert(!tracker.valid(time, 3));
  assert(tracker.following_velocity(time + 300000, 3).norm() < .00001f);
}

static void test_final_velocity_planner() {
  const matrix::Dcmf identity(matrix::Eulerf(0.f, 0.f, 0.f));
  const Limits limits{40,40,20,4};
  Vector3f previous, acceleration, actual;
  float z = -3.f, highest = z;
  bool reached = false;
  int overrides = 0;
  for (int i = 0; i < 5000; ++i) {
    const float remaining = z + 13.f;
    const float vz = reached ? clamp(.8f * (-13.f-z), -1.f, 1.f)
                            : height_capture_velocity(remaining, actual(2), 20, 3, 8, 1);
    auto plan = plan_velocity(Vector3f(3,0,vz),previous,acceleration,limits,.01f,8,8,
        Vector3f(1,0,0),Vector3f{},identity,280,.5f,.4f,.65f,true,!reached);
    assert(feasible(plan.velocity, limits));
    assert((plan.acceleration - (plan.velocity-previous)/.01f).norm() < .001f);
    if (!plan.limits_override) {
      assert((plan.acceleration-acceleration).norm() <= .0821f);
      assert(plan.acceleration.norm() <= 8.0021f);
    } else { ++overrides; }
    previous = plan.velocity;
    acceleration = plan.acceleration;
    actual += (previous-actual)*(.01f/.2f);
    z += actual(2)*.01f;
    if (i > 3000) { z += .001f; } // Position drift with near-zero measured velocity.
    highest = min(highest,z);
    if (remaining <= .3f && fabsf(actual(2)) < .5f) { reached = true; }
  }
  printf("height capture: reached=%d final z=%.3f highest=%.3f overrides=%d\n",reached,double(z),double(highest),overrides);
  assert(reached && highest > -14.f && fabsf(z+13.f) < .3f);
  assert(previous(0) > 2.5f);
  const auto limited = plan_velocity(Vector3f{},Vector3f(30,0,0),Vector3f{},Limits{5,5,2,2},.01f,4,8,
      Vector3f(1,0,0),Vector3f{},identity,50,.5f,.4f,.65f,false,false);
  assert(limited.limits_override && limited.velocity.norm() <= 5.001f);

  // Include near-edge image drift and tilted-camera cases. Check the risk of
  // the actual final velocity, including vertical control and speed clipping.
  for (int height = 0; height < 2; ++height) {
    const matrix::Dcmf camera(matrix::Eulerf(.1f, 1.1f, .4f));
    const Vector3f ray = Vector3f(1,.28f,-.2f).normalized();
    const Vector3f ray_rate(0,.08f,-.03f);
    previous = Vector3f(4,2,-.5f);
    acceleration.zero();
    for (int i = 0; i < 1000; ++i) {
      const auto plan = plan_velocity(Vector3f(30,10,-2),previous,acceleration,limits,.01f,8,8,
          ray,ray_rate,camera,50,.5f,.4f,.65f,height != 0,height != 0);
      const float actual_fov = startup_fov_cost(ray,ray_rate,camera,
          (plan.velocity-previous)/.01f-acceleration,50,.5f,.4f);
      assert(fabsf(actual_fov-plan.predicted_fov) < .0001f);
      assert(feasible(plan.velocity,limits));
      if (!plan.limits_override) {
        assert((plan.acceleration-acceleration).norm() <= .0821f);
        assert(plan.acceleration.norm() <= 8.0021f);
      }
      previous = plan.velocity;
      acceleration = plan.acceleration;
    }
  }
}

static int replay() {
  TargetMotion tracker;
  unsigned long long stamp;
  float own[3], los[3], range, sigma, angle;
  unsigned samples = 0, moving = 0;
  float peak = 0.f;
  while (scanf("%llu %f %f %f %f %f %f %f %f %f", &stamp, &own[0], &own[1], &own[2],
               &los[0], &los[1], &los[2], &range, &sigma, &angle) == 10) {
    tracker.update(stamp, Vector3f(own), Vector3f(los), range, sigma, angle, 3, 60);
    ++samples;
    moving += tracker.motion_state() == 2;
    peak = max(peak, tracker.following_velocity(stamp, 3).norm());
  }
  printf("replay samples=%u resets=%u moving_samples=%u peak_follow=%.3f final_speed=%.3f sigma=%.3f\n",
         samples, tracker.resets(), moving, double(peak), double(tracker.velocity().norm()),
         double(tracker.velocity_sigma()));
  return samples ? 0 : 1;
}

static void test_static_camera_motion() {
  TargetMotion tracker;
  unsigned moving = 0;
  const Vector3f target(150,0,-260);
  for (int i = 0; i < 2000; ++i) {
    const float t = i * .01f;
    const Vector3f own(3.f*t, sinf(t)*.2f, -min(10.f,2.f*t));
    const matrix::Dcmf camera(matrix::Eulerf(.02f*sinf(t),1.22f+.15f*sinf(.6f*t),.03f*sinf(t)));
    const Vector3f relative = target-own;
    const Vector3f ray = Vector3f(camera.transpose()*relative).normalized();
    float range, sigma;
    assert(box_range(roundf(1662.77f/(relative.norm()*ray(0))),roundf(1303.68f/(relative.norm()*ray(0))),
        1,1,1662.77f,1303.68f,ray(0),.15f,range,sigma));
    const auto stamp = uint64_t(1000000+i*10000);
    tracker.update(stamp,own,camera*ray,range,sigma,.00154f,3,60);
    moving += tracker.motion_state() == 2;
  }
  printf("static camera/own motion: moving samples=%u final speed=%.3f\n",moving,double(tracker.velocity().norm()));
  assert(moving == 0);
}

static void test_bearing_gap() {
  TargetMotion tracker;
  const Vector3f los(1,0,0);
  assert(tracker.validity_reason(1000000,3) == MotionValidityReason::NoState);
  assert(!tracker.update(1000000,Vector3f{},los,NAN,NAN,.002f,1,60));
  assert(tracker.last_update_result() == MotionUpdateResult::InvalidInput);
  for (uint64_t t = 1000000; t <= 4000000; t += 100000) {
    assert(tracker.update(t,Vector3f{},los,100,.5f,.002f,1,60));
  }
  assert(tracker.validity_reason(4000000,3) == MotionValidityReason::Ready);
  assert(tracker.validity_reason(4000000,.001f) == MotionValidityReason::VelocityUncertain);
  const auto point = predict_closest_approach(Vector3f(10,1,0), Vector3f(5,0,0),
      Vector3f{}, Vector3f(5,0,0), .5f, 4.f);
  const auto evidence = assess_closest_approach(tracker,4000000,point);
  assert(evidence.valid && evidence.miss_bound_m >= point.miss.norm());
  assert(evidence.time_upper_s >= point.time_s);
  assert(!assess_closest_approach(tracker,4300000,point).valid);
  assert(!tracker.update(4050000,Vector3f{},los,100,.5f,.002f,1,60));
  assert(tracker.last_update_result() == MotionUpdateResult::RateLimited);
  const auto resets = tracker.resets();
  for (uint64_t t = 4100000; t <= 4900000; t += 100000) {
    assert(tracker.update(t,Vector3f{},los,NAN,NAN,.002f,1,60));
  }
  assert(tracker.resets() == resets && tracker.valid(4900000,3));
  assert(!tracker.update(5100000,Vector3f{},los,NAN,NAN,.002f,1,60));
  assert(tracker.last_update_result() == MotionUpdateResult::NoRecentMetric);
  assert(tracker.validity_reason(5100000,3) == MotionValidityReason::StaleMetric);
  assert(tracker.validity_reason(5200000,3) == MotionValidityReason::StaleUpdate);
  assert(!tracker.valid(5100000,3));
  assert(tracker.update(6000000,Vector3f{},los,100,.5f,.002f,1,60));
  assert(tracker.resets() == resets+1 && tracker.accepted_updates() == 1);
  assert(tracker.validity_reason(6000000,3) == MotionValidityReason::TooFewUpdates);
}

static void test_target_height_envelope() {
  // Reachable log_46-like climb: a speed setpoint that previously continued
  // upward is passed through the actual jerk-limited velocity planner and a
  // lagged, acceleration-limited vertical plant.
  constexpr float dt = .02f;
  float remaining = 220.f;
  float actual_vz = -30.f;
  Vector3f previous(30.f, 0.f, -30.f), acceleration;
  const matrix::Dcmf camera(matrix::Eulerf(0.f, 0.f, 0.f));
  for (int i = 0; i < 600; ++i) {
    const float vz = limit_target_climb_velocity(-40.f, -remaining, actual_vz,
        0.f, 10.f, 3.f, 8.f);
    const auto plan = plan_velocity(Vector3f(30.f, 0.f, vz), previous,
        acceleration, Limits{60.f, 60.f, 40.f, 10.f}, dt, 3.f, 8.f,
        Vector3f(1.f, 0.f, 0.f), Vector3f{}, camera, 200.f,
        .5f, .4f, .65f, false, false);
    previous = plan.velocity;
    acceleration = plan.acceleration;
    const float plant_acceleration = clamp((plan.velocity(2) - actual_vz) / .2f, -3.f, 3.f);
    actual_vz += plant_acceleration * dt;
    remaining += actual_vz * dt;
    assert(remaining > -.6f);
  }
  assert(remaining < 10.f && actual_vz > -3.f);
}

int main(int argc, char **argv) {
  if (argc == 2 && strcmp(argv[1], "--replay") == 0) { return replay(); }
  test_following_outside_axis_limit();
  test_steep_climb_acceleration_direction();
  test_vertical_acceleration_limits();
  test_recovery_hysteresis();
  test_final_velocity_planner();
  test_capture_schedule();
  test_static_camera_motion();
  test_bearing_gap();
  test_target_height_envelope();
  assert(startup_climb_velocity(10, 20, 3, 1) == -3.f);
  assert(startup_climb_velocity(10, 1, 3, 1) == -1.f);
  assert(startup_climb_velocity(.2f, 20, 3, 1) > -.21f);
  assert(startup_climb_velocity(-1, 20, 3, 1) == 0.f);
  assert(startup_climb_velocity(10, 20, 3, 0) == 0.f);
  assert(startup_climb_velocity(NAN, 20, 3, 1) == 0.f);
  assert(startup_climb_velocity(10, 20, 3, .5f) == -1.5f);
  // In log_46 the aircraft was still climbing at 27 m/s with only about
  // 70 m of height remaining. The target-height envelope must brake before
  // the altitude crossing, without reversing an already slower command.
  const float early_brake = limit_target_climb_velocity(-20.f, -70.f, -27.f, 0.f, 10.f, 3.f, 8.f);
  assert(early_brake > -17.f && early_brake < -15.f);
  assert(limit_target_climb_velocity(-5.f, -70.f, -27.f, 0.f, 10.f, 3.f, 8.f) == -5.f);
  assert(limit_target_climb_velocity(-6.f, 2.8f, -7.f, 0.f, 1.f, 3.f, 8.f) == 0.f);
  assert(limit_target_climb_velocity(-30.f, -400.f, -2.f, 0.f, 20.f, 3.f, 8.f) == -30.f);
  assert(limit_target_climb_velocity(-6.f, NAN, -7.f, 0.f, 1.f, 3.f, 8.f) == -6.f);
  float range = 0, sigma = 0;
  assert(!box_range(10, 10, 0, 0, 1000, 1000, 1, .15f, range, sigma));
  assert(!box_range(2, 2, 1, 1, 1000, 1000, 1, .15f, range, sigma));
  assert(!box_range(10, 40, 1, 1, 1000, 1000, 1, .15f, range, sigma));
  assert(!box_range(NAN, 10, 1, 1, 1000, 1000, 1, .15f, range, sigma));
  assert(box_range(20, 20, 1, 1, 1000, 1000, .5f, .15f, range, sigma));
  assert(fabsf(range - 100.f) < .001f && sigma >= 15.f);
  assert(!box_range(65535, 65535, 1, 1, 1000, 1000, 1, .15f, range, sigma));
  TargetMotion empty;
  assert(!empty.update(1000000, Vector3f{}, Vector3f(1, 0, 0), NAN, 1, .01f, 3, 60));
  assert(!empty.valid(2000000, 3));
  const Vector3f north(1, 0, 0);
  assert(classify_pass(Vector3f(5, 0, 0), north, .5f, 2, 3) == PassState::Approaching);
  assert(classify_pass(Vector3f(-3, 0, 0), north, .4f, 2, 3) == PassState::Predicted);
  assert(classify_pass(Vector3f(-2.5f, 0, 0), north, .5f, 2, 3) == PassState::Approaching);
  assert(classify_pass(Vector3f(-5, 4, 0), north, .5f, 2, 3) == PassState::Uncertain);
  assert(classify_pass(Vector3f(-5, 0, 0), north, 2, 2, 3) == PassState::Uncertain);
  assert(classify_pass(Vector3f(-5, 0, 0), north, NAN, 2, 3) == PassState::Uncertain);
  const Limits fast{45, 45, 15, 10}, slow{20, 20, 10, 5};
  assert(fabsf(closing_budget(Vector3f(35, 0, 0), north, fast, 20) - 10.f) < .001f);
  assert(closing_budget(Vector3f(35, 0, 0), north, slow, 20) < .001f);
  const Vector3f direction = Vector3f(1, .5f, -.7f).normalized();
  const Vector3f base(30, 2, -3);
  const float budget = closing_budget(base, direction, fast, 60);
  assert(budget > 0 && feasible(base + direction * budget, fast));
  assert(!feasible(base + direction * (budget + .01f), fast));
  assert(feasible(constrained(Vector3f(100, 100, -100), slow), slow));
  for (float speed : {0.f, 20.f, 25.f, 30.f, 35.f}) {
    test_motion(Vector3f(speed, 0, 0), true);
    test_motion(Vector3f(0, speed, 0), false);
  }
  test_motion(Vector3f(20, -20, 5), true);
  test_turn();
  test_quantized_boxes();
  for (float speed : {0.f, 20.f, 25.f, 35.f}) {
    test_small_box_following(speed, 0.f);
    test_small_box_following(speed, .8f);
  }
  puts("TargetMotion tests passed");
}
