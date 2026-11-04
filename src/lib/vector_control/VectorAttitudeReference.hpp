#pragma once

#include "VectorGeometry.hpp"

namespace vector_control
{

class AttitudeReference
{
public:
	struct Settings {
		float bank_limit{math::radians(20.f)};
		float pitch_limit{math::radians(160.f)};
		float pitch_rate{math::radians(20.f)};
		float pitch_acceleration{math::radians(60.f)};
		float bank_rate{math::radians(80.f)};
		float bank_acceleration{math::radians(360.f)};
		float heading_rate{math::radians(45.f)};
		float heading_acceleration{math::radians(90.f)};
		float tracking_frequency{8.f}; // rad/s, reference model (not closed-loop bandwidth)
		float thrust_limit{1.f};
	};

	struct Output {
		matrix::Quatf attitude{};
		matrix::Vector3f force{}, omega_ned{}, alpha_ned{};
		float heading{0.f};
		float bank{0.f};
		float pitch{0.f};
	};

	void reset() { _initialized = false; _heading = {}; _bank = {}; _pitch = {}; }
	bool initialized() const { return _initialized; }
	matrix::Quatf attitude() const
	{
		return matrix::Quatf(matrix::Eulerf(0.f, 0.f, _heading.position))
		       * matrix::Quatf(matrix::Eulerf(_bank.position, 0.f, 0.f))
		       * matrix::Quatf(matrix::Eulerf(0.f, _pitch.position, 0.f));
	}
	float pitchAuthority(const matrix::Quatf &actual) const
	{
		if (!_initialized || !actual.isAllFinite()) { return .2f; }

		const auto error = (actual.inversed() * attitude()).canonical();
		const float angle = 2.f * atan2f(error.imag().norm(), fabsf(error(0)));
		// Reserve tracking authority for recovery before continuing an independent
		// pitch maneuver. Rate changes themselves remain acceleration-limited.
		return math::constrain((math::radians(15.f) - angle) / math::radians(10.f), .2f, 1.f);
	}

	bool initialize(const matrix::Quatf &q, float bank_limit)
	{
		const float heading = forcePlaneHeading(q);

		if (!PX4_ISFINITE(heading)) { reset(); return false; }

		const matrix::Dcmf r(q);
		_heading = {heading, 0.f, 0.f};
		_bank = {math::constrain(asinf(math::constrain(r(2, 1), -1.f, 1.f)), -bank_limit, bank_limit), 0.f, 0.f};
		_pitch = {atan2f(-r(2, 0), r(2, 2)), 0.f, 0.f};
		_initialized = true;
		return true;
	}

	// A local-frame heading reset changes the reference frame, not the maneuver.
	void resetHeading(float delta)
	{
		if (_initialized && PX4_ISFINITE(delta)) { _heading.position = matrix::wrap_pi(_heading.position + delta); }
	}

	void adaptFrame(const matrix::Quatf &delta, float bank_limit)
	{
		if (_initialized) {
			const matrix::Quatf q = matrix::Quatf(matrix::Eulerf(0.f, 0.f, _heading.position))
						* matrix::Quatf(matrix::Eulerf(_bank.position, 0.f, 0.f))
						* matrix::Quatf(matrix::Eulerf(0.f, _pitch.position, 0.f));
			// Discard derivatives at a discontinuous estimator frame reset.
			initialize(delta * q, bank_limit);
		}
	}

	bool update(const matrix::Vector3f &force, float heading_target, float heading_rate_ff, float pitch_target,
		    float pitch_authority, float dt, const Settings &s, Output &out)
	{
		if (!_initialized || !force.isAllFinite() || !PX4_ISFINITE(pitch_target)
		    || !PX4_ISFINITE(dt) || dt <= 0.f || dt > .04f || force(2) >= -1e-5f) {
			return false;
		}

		const float yaw_ff = PX4_ISFINITE(heading_rate_ff) ? heading_rate_ff : 0.f;
		// NAN means rate-only heading control. Integrate the filtered rate without
		// continually relatching the measured Euler yaw.
		const float heading_error = PX4_ISFINITE(heading_target) ? matrix::wrap_pi(heading_target - _heading.position) : 0.f;
		// Heading maneuvers need pitch/force recovery margin at the ends of the
		// common-tilt envelope. Slow the heading reference near those endpoints;
		// retain the full bank stabilization response.
		const float heading_authority = math::constrain((s.pitch_limit - fabsf(_pitch.position)) / math::radians(20.f), .2f, 1.f);
		advance(_heading, heading_error, yaw_ff, s.heading_rate * heading_authority,
			s.heading_acceleration * heading_authority, s.tracking_frequency, dt);
		_heading.position = matrix::wrap_pi(_heading.position);

		const float cs = cosf(_heading.position), sn = sinf(_heading.position);
		const float lateral = -sn * force(0) + cs * force(1);
		const float bank_target = math::constrain(atan2f(lateral, -force(2)), -s.bank_limit, s.bank_limit);
		advance(_bank, bank_target - _bank.position, 0.f, s.bank_rate, s.bank_acceleration, s.tracking_frequency, dt, s.bank_limit);
		limitPosition(_bank, s.bank_limit);
		const float pitch = math::constrain(pitch_target, -s.pitch_limit, s.pitch_limit);
		advance(_pitch, pitch - _pitch.position, 0.f, s.pitch_rate * math::constrain(pitch_authority, .2f, 1.f),
			s.pitch_acceleration, s.tracking_frequency, dt, s.pitch_limit);
		limitPosition(_pitch, s.pitch_limit);

		const matrix::Dcmf rz(matrix::Eulerf(0.f, 0.f, _heading.position));
		const matrix::Dcmf rx(matrix::Eulerf(_bank.position, 0.f, 0.f));
		const matrix::Vector3f z(0.f, 0.f, 1.f), x = rz.col(0), y = (rz * rx).col(1);
		out.attitude = matrix::Quatf(rz * rx * matrix::Dcmf(matrix::Eulerf(0.f, _pitch.position, 0.f)));
		out.attitude.normalize();
		out.omega_ned = z * _heading.rate + x * _bank.rate + y * _pitch.rate;
		out.alpha_ned = z * _heading.acceleration + x * _bank.acceleration + y * _pitch.acceleration
				+ z.cross(x) * (_heading.rate * _bank.rate)
				+ (z * _heading.rate + x * _bank.rate).cross(y) * _pitch.rate;

		// Produce force and attitude from the SAME limited bank. Preserve vertical
		// lift and longitudinal force; report the lateral shortfall to the caller.
		const float longitudinal = cs * force(0) + sn * force(1);
		const float limited_lateral = -force(2) * tanf(_bank.position);
		out.force = matrix::Vector3f(cs * longitudinal - sn * limited_lateral,
					     sn * longitudinal + cs * limited_lateral, force(2));
		const float norm = out.force.norm();

		if (norm > s.thrust_limit && norm > 1e-5f) { out.force *= s.thrust_limit / norm; }

		out.heading = _heading.position; out.bank = _bank.position; out.pitch = _pitch.position;
		return out.attitude.isAllFinite() && out.force.isAllFinite() && out.omega_ned.isAllFinite() && out.alpha_ned.isAllFinite();
	}

private:
	struct Axis { float position{0.f}, rate{0.f}, acceleration{0.f}; };
	static float stoppingRate(float distance, float rate, float acceleration, float dt)
	{
		// Reserve v^2/(2*a) + v*dt/2 after the trapezoidal position step.
		// This makes braking feasible on the NEXT sample as well, including the
		// final sample where the rate reaches zero before the full step elapses.
		const float step = acceleration * dt;
		return math::max(0.f, sqrtf(math::max(step * step - step * rate + 2.f * acceleration * distance, 0.f)) - step);
	}
	static void advance(Axis &axis, float error, float feedforward, float rate_limit, float acceleration_limit, float frequency, float dt,
			    float position_limit = INFINITY)
	{
		const float w = math::constrain(frequency, 1.f, 20.f);
		const float desired_rate = math::constrain(feedforward + .5f * w * error, -rate_limit, rate_limit);
		const float acceleration = math::constrain(2.f * w * (desired_rate - axis.rate),
					   -acceleration_limit, acceleration_limit);
		// A decreasing authority/rate limit must not cause an instantaneous rate
		// step (and an unbounded acceleration feedforward).
		float next_rate = math::constrain(axis.rate + acceleration * dt,
						  math::min(axis.rate, desired_rate), math::max(axis.rate, desired_rate));

		if (PX4_ISFINITE(position_limit)) {
			next_rate = math::constrain(next_rate,
						    -stoppingRate(position_limit + axis.position, -axis.rate, acceleration_limit, dt),
						    stoppingRate(position_limit - axis.position, axis.rate, acceleration_limit, dt));
		}

		axis.acceleration = (next_rate - axis.rate) / dt;
		axis.position += .5f * (axis.rate + next_rate) * dt;
		axis.rate = next_rate;
	}
	static void limitPosition(Axis &axis, float limit)
	{
		const float bounded = math::constrain(axis.position, -limit, limit);

		if (axis.position > limit || axis.position < -limit) {
			axis.position = bounded;
			axis.rate = 0.f;
			axis.acceleration = 0.f;
		}
	}
	Axis _heading{}, _bank{}, _pitch{};
	bool _initialized{false};
};

} // namespace vector_control
