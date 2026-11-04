#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <stdint.h>

namespace vector_control
{

/** Motion-based common-tilt observation, not a servo position measurement.
 *
 * BODY FRD: Fx = S sin(a), Fz = -S cos(a) - T5,
 * My = z Fx + x S cos(a) - tail T5. Eliminating T5 gives
 * S cos(a) = (My - tail Fz - z Fx) / (x + tail).
 * Use accelerometer specific force (do NOT subtract gravity again) and
 * My = [J w_dot + w x Jw]_y. External contact forces invalidate this model.
 */
class VoliroTiltMonitor
{
public:
	struct Settings {
		float x{.14f}, z{0.f}, tail{.64f}, mass{5.f};
		matrix::Vector3f inertia{.1f, .1f, .2f};
		float servo_rate{math::radians(90.f)}, servo_tau{0.f};
		float error_limit{math::radians(10.f)}, error_time{.5f};
	};

	struct Sample {
		uint64_t acceleration_timestamp{0}, angular_timestamp{0}, command_timestamp{0};
		matrix::Vector3f specific_force{}, rates{}, angular_acceleration{};
		float command{0.f};
	};

	void reset() { *this = VoliroTiltMonitor{}; }
	bool fault() const { return _fault; }
	bool valid() const { return _valid; }
	float observed() const { return _observed; }
	float expected() const { return _expected_filtered; }
	float error() const { return _valid ? _observed - _expected_filtered : NAN; }
	float errorDuration() const { return _bad_time; }

	static bool fresh(uint64_t now, uint64_t timestamp, uint64_t max_age = 100000)
	{
		return timestamp != 0 && timestamp <= now && now - timestamp <= max_age;
	}

	void update(const Settings &s, const Sample &sample, uint64_t now, bool enabled, bool armed, bool airborne)
	{
		if (!armed) { reset(); return; }

		// A parameter change or a mode change must never release an airborne latch.
		if ((!enabled && !_fault) || !airborne || !fresh(now, sample.acceleration_timestamp)
		    || !fresh(now, sample.angular_timestamp) || !fresh(now, sample.command_timestamp)
		    || !sample.specific_force.isAllFinite() || !sample.rates.isAllFinite()
		    || !sample.angular_acceleration.isAllFinite() || !PX4_ISFINITE(sample.command)
		    || !s.inertia.isAllFinite() || s.inertia.min() <= 0.f
		    || !PX4_ISFINITE(s.mass) || s.mass <= .01f || !PX4_ISFINITE(s.x) || s.x <= 0.f
		    || !PX4_ISFINITE(s.tail) || s.tail <= 0.f || !PX4_ISFINITE(s.z)
		    || !PX4_ISFINITE(s.servo_rate) || s.servo_rate <= 0.f || !PX4_ISFINITE(s.servo_tau) || s.servo_tau < 0.f
		    || !PX4_ISFINITE(s.error_limit) || s.error_limit <= 0.f || !PX4_ISFINITE(s.error_time) || s.error_time <= 0.f) {
			invalidate(); return;
		}

		const uint64_t skew = sample.acceleration_timestamp > sample.angular_timestamp
				      ? sample.acceleration_timestamp - sample.angular_timestamp
				      : sample.angular_timestamp - sample.acceleration_timestamp;

		if (skew > 30000) { invalidate(); return; }

		// Re-reading a sample cannot advance the fault timer.
		if (sample.acceleration_timestamp == _last_sample) { return; }

		if (_last_sample != 0 && (sample.acceleration_timestamp < _last_sample
					  || sample.acceleration_timestamp - _last_sample > 100000)) { invalidate(); }

		const float dt = _last_sample == 0 ? .01f : (sample.acceleration_timestamp - _last_sample) * 1e-6f;
		_last_sample = sample.acceleration_timestamp;
		const matrix::Vector3f moment = s.inertia.emult(sample.angular_acceleration)
						+ sample.rates.cross(s.inertia.emult(sample.rates));
		const float u = sample.specific_force(0);
		const float v = (moment(1) / s.mass - s.tail * sample.specific_force(2) - s.z * u) / (s.x + s.tail);
		const float front_specific_force = sqrtf(u * u + v * v);

		// Low thrust, large out-of-plane force and extreme motion do not provide
		// a trustworthy common-tilt observation. Unknown is not an actuator fault.
		if (!PX4_ISFINITE(front_specific_force) || front_specific_force < 1.96f || front_specific_force > 39.23f
		    || fabsf(sample.specific_force(1)) > 1.f + .35f * front_specific_force
		    || sample.rates.norm() > 4.f || sample.angular_acceleration.norm() > 50.f) {
			invalidate(); return;
		}

		const float observation = atan2f(u, v);
		const float filter_alpha = 1.f - expf(-dt / .15f);

		if (!_initialized) {
			_observed = observation;
			_expected = _expected_filtered = sample.command;
			_initialized = true;
		}

		const float servo_alpha = s.servo_tau > 1e-4f ? 1.f - expf(-dt / s.servo_tau) : 1.f;
		_expected += math::constrain(servo_alpha * (sample.command - _expected), -s.servo_rate * dt, s.servo_rate * dt);
		_expected_filtered += filter_alpha * (_expected - _expected_filtered);
		// Mechanical travel is bounded, so compare signed angles without wrap_pi:
		// +160 and -160 degrees are opposite mechanical positions.
		_observed += filter_alpha * (observation - _observed);
		_valid = true;
		_warmup += dt;

		if (_warmup >= math::max(.5f, 3.f * s.servo_tau) && fabsf(error()) > s.error_limit) {
			_bad_time += dt;

			if (enabled && _bad_time >= s.error_time) { _fault = true; }

		} else {
			_bad_time = 0.f;
		}
	}

private:
	void invalidate()
	{
		_valid = _initialized = false;
		_last_sample = 0;
		_bad_time = _warmup = 0.f;
	}

	uint64_t _last_sample{0};
	float _observed{0.f}, _expected{0.f}, _expected_filtered{0.f}, _bad_time{0.f}, _warmup{0.f};
	bool _fault{false}, _valid{false}, _initialized{false};
};

/** Pitch rotation that aligns the fixed-tilt, zero-pitch-moment resultant
 * with a conventional multicopter's -Z thrust axis. Includes the tail trim. */
inline float voliroFallbackThrustAngle(float tilt, float x, float z, float tail)
{
	return atan2f(tail * sinf(tilt), (tail + x) * cosf(tilt) + z * sinf(tilt));
}

/** Bound changes of a quaternion reference, including the first fallback step. */
inline matrix::Quatf slewFallbackAttitude(const matrix::Quatf &previous, const matrix::Quatf &target, float step)
{
	matrix::AxisAnglef error((previous.inversed() * target).canonical());
	const float angle = error.angle();
	matrix::Quatf result = angle > step && angle > 1e-6f
			       ? previous * matrix::Quatf(matrix::AxisAnglef(error.axis(), math::max(step, 0.f))) : target;
	result.normalize();
	return result;
}

} // namespace vector_control
