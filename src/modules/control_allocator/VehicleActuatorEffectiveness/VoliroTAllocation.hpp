#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

// BODY FRD. M1/M2 at (+x,+y,z), M3/M4 at (+x,-y,z), M5 at (-tail,0,0).
// Front direction = (sin(tilt),0,-cos(tilt)); signed M5 thrust acts along -Z.
// Viewed from above: M1 CCW, M2 CW, M3 CW, M4 CCW. Their reaction
// torque signs along the front thrust direction are -,+,+,-.
namespace voliro_t
{
struct Geometry {
	float x, y, z, tail, front_max, tail_max, km;
	float tail_reverse_max{0.f}; // zero preserves the legacy symmetric map
};
inline float tailMax(const Geometry &g, float signed_value)
{
	return signed_value < 0.f && g.tail_reverse_max > 0.f ? g.tail_reverse_max : g.tail_max;
}
inline float tailThrust(const Geometry &g, float command)
{
	return command * fabsf(command) * tailMax(g, command);
}
inline float tailCommand(const Geometry &g, float thrust)
{
	return copysignf(sqrtf(fabsf(thrust) / tailMax(g, thrust)), thrust);
}
struct Result {
	float thrust[5] {};
	float tilt{0.f};
	matrix::Vector3f force{};
	matrix::Vector3f torque{};
};
inline void forward(const Geometry &g, Result &r)
{
	const float sn = sinf(r.tilt), cs = cosf(r.tilt);
	const matrix::Vector3f axis(sn, 0.f, -cs);
	r.force.zero(); r.torque.zero();

	for (int i = 0; i < 4; ++i) {
		const matrix::Vector3f p(g.x, i < 2 ? g.y : -g.y, g.z);
		const matrix::Vector3f f = axis * r.thrust[i];
		r.force += f;
		const float reaction_sign = (i == 0 || i == 3) ? -1.f : 1.f;
		r.torque += p.cross(f) + axis * (reaction_sign * g.km * r.thrust[i]);
	}

	r.force(2) -= r.thrust[4];
	r.torque(1) -= g.tail * r.thrust[4];
}
struct ForcePitchSolution {
	float tilt{0.f}, front_sum{0.f}, tail{0.f};
};

inline float reactionTorque(float tilt, const matrix::Vector3f &torque)
{
	return sinf(tilt) * torque(0) - cosf(tilt) * torque(2);
}

/** Stateful actuator-mode filter. The state is a scalar rotor reaction torque,
 * not a body-Z component; it remains meaningful while the common tilt moves. */
class ReactionTorqueFilter
{
public:
	float update(float demand, float cutoff, float dt)
	{
		if (!PX4_ISFINITE(demand)) { reset(); return 0.f; }

		if (cutoff <= 0.f) { _state = demand; }

		else if (PX4_ISFINITE(dt) && dt > 0.f) { _state += (1.f - expf(-2.f * M_PI_F * cutoff * math::min(dt, .02f))) * (demand - _state); }

		return _state;
	}
	void reset() { _state = 0.f; }
private:
	float _state{0.f};
};

inline ForcePitchSolution allocateForcePitch(const Geometry &g, const matrix::Vector3f &force, const matrix::Vector3f &torque,
		float previous_tilt, float angle_limit, float angle_step, bool pitch_priority = false)
{
	ForcePitchSolution r;
	// The exact unconstrained force/pitch solution, before angular constraints:
	// u=S*sin(a)=Fx; v=S*cos(a)=(My-tail*Fz-z*Fx)/(x+tail).
	const float v = (torque(1) - g.tail * force(2) - g.z * force(0)) / (g.x + g.tail);
	// For a near-vertical upright lift request, a negative v would point the
	// front rotors past 90 degrees to satisfy a small pitch moment (including
	// the attitude correction before takeoff). Keep them upright and leave
	// the infeasible moment in the allocation residual. Large horizontal force
	// requests still permit the full common-tilt range.
	const bool upright_lift_with_negative_front = force(2) < 0.f && v < 0.f
			&& fabsf(force(0)) < .25f * fabsf(force(2));
	// At zero collective, manual attitude feedback can still request pitch
	// torque. It must not turn atan2(0, negative v) into a 180-degree tilt
	// request on arming. Center the servo at the normal slew rate instead.
	const bool no_force = force.norm_squared() <= 1e-12f;
	float target = no_force || upright_lift_with_negative_front ? 0.f
		       : ((fabsf(force(0)) + fabsf(v) > 1e-6f) ? atan2f(force(0), v) : previous_tilt);

	if (pitch_priority && !no_force && !upright_lift_with_negative_front) {
		// Crossing atan2's +/-180 boundary must not send the servo through zero.
		target = previous_tilt + matrix::wrap_pi(target - previous_tilt);
	}

	r.tilt = math::constrain(math::constrain(target, -angle_limit, angle_limit),
				 previous_tilt - angle_step, previous_tilt + angle_step);
	const float sn = sinf(r.tilt), cs = cosf(r.tilt);

	if (pitch_priority && fabsf(target) > angle_limit) {
		// At a mechanical stop, enforce feasible pitch torque first, then fit
		// force. A force-first compromise can leave a pitch bias that drives
		// unbounded horizontal drift while the position integrator fights it.
		const float moment_arm = g.z * sn + g.x * cs;

		if (fabsf(moment_arm) > 1e-5f) {
			const float s1 = (torque(1) - g.tail * tailMax(g, -1.f)) / moment_arm;
			const float s2 = (torque(1) + g.tail * g.tail_max) / moment_arm;
			const float lower = math::max(0.f, math::min(s1, s2));
			const float upper = math::min(4.f * g.front_max, math::max(s1, s2));

			if (lower <= upper) {
				const matrix::Vector2f axis(sn, -cs - moment_arm / g.tail);
				const matrix::Vector2f demand(force(0), force(2) - torque(1) / g.tail);
				r.front_sum = math::constrain(axis.dot(demand) / axis.norm_squared(), lower, upper);
				r.tail = math::constrain((moment_arm * r.front_sum - torque(1)) / g.tail, -tailMax(g, -1.f), g.tail_max);
				return r;
			}
		}
	}

	// At the reachable angle, least-squares fit Fx,Fz,My. Scale moment by
	// a characteristic arm length to avoid mixing N and N*m weights.
	const float arm = g.x + g.tail;
	const matrix::Vector3f a(sn, -cs, (g.z * sn + g.x * cs) / arm);
	const matrix::Vector3f b(0.f, -1.f, -g.tail / arm);
	const matrix::Vector3f w(force(0), force(2), torque(1) / arm);
	const float aa = a.dot(a), ab = a.dot(b), bb = b.dot(b);
	const float det = aa * bb - ab * ab;
	float sum = math::constrain((a.dot(w) * bb - b.dot(w) * ab) / det, 0.f, 4.f * g.front_max);
	const float tail = math::constrain((b.dot(w) - ab * sum) / bb, -tailMax(g, -1.f), g.tail_max);
	// Refit sum if tail saturation changed the solution.
	sum = math::constrain(a.dot(w - b * tail) / aa, 0.f, 4.f * g.front_max);
	r.front_sum = sum;
	r.tail = tail;
	return r;
}

/** Fixed tilt: prioritize the closest feasible pitch torque, then fit force.
 * Used by multicopter fallback and fixed-servo allocation. */
inline ForcePitchSolution allocateAtTilt(const Geometry &g, const matrix::Vector3f &force,
				       const matrix::Vector3f &torque, float tilt)
{
	const float sn = sinf(tilt), cs = cosf(tilt), h = g.z * sn + g.x * cs;
	const float max_front = 4.f * g.front_max;
	const float min_pitch = math::min(0.f, h * max_front) - g.tail * g.tail_max;
	const float max_pitch = math::max(0.f, h * max_front) + g.tail * tailMax(g, -1.f);
	// Keep pitch recovery priority even beyond the attainable moment envelope.
	// Reverting to force-weighted least squares here can reverse the tail and
	// produce the opposite pitch moment precisely when damping is saturated.
	const float pitch = math::constrain(torque(1), min_pitch, max_pitch);
	float lower = 0.f, upper = max_front;

	if (fabsf(h) > 1e-8f) {
		const float s1 = (pitch - g.tail * tailMax(g, -1.f)) / h;
		const float s2 = (pitch + g.tail * g.tail_max) / h;
		// Clamped pitch guarantees a feasible interval; also absorb roundoff
		// at the actuator corners so it cannot reopen the force-first branch.
		lower = math::constrain(math::min(s1, s2), 0.f, max_front);
		upper = math::constrain(math::max(s1, s2), lower, max_front);
	}

	const matrix::Vector2f axis(sn, -cs - h / g.tail);
	const matrix::Vector2f demand(force(0), force(2) - pitch / g.tail);
	const float sum = math::constrain(axis.dot(demand) / math::max(axis.norm_squared(), 1e-6f), lower, upper);
	return {tilt, sum, math::constrain((h * sum - pitch) / g.tail, -tailMax(g, -1.f), g.tail_max)};
}

inline ForcePitchSolution refitAtTiltForTail(const Geometry &g, const matrix::Vector3f &force,
		const matrix::Vector3f &torque, float tilt, float tail)
{
	const float h = g.z * sinf(tilt) + g.x * cosf(tilt);
	const float sum = fabsf(h) > 1e-5f ? (torque(1) + g.tail * tail) / h
			 : force(0) * sinf(tilt) - (force(2) + tail) * cosf(tilt);
	return {tilt, math::constrain(sum, 0.f, 4.f * g.front_max), tail};
}

inline Result allocateRollYaw(const Geometry &g, const ForcePitchSolution &solution,
			      const matrix::Vector3f &torque, float reaction_torque)
{
	Result r;
	r.tilt = solution.tilt;
	const float sum = solution.front_sum;
	const float sn = sinf(r.tilt), cs = cosf(r.tilt);
	const float difference = math::constrain(-(cs * torque(0) + sn * torque(2)) / g.y,
				 -math::min(sum, 4.f * g.front_max - sum), math::min(sum, 4.f * g.front_max - sum));
	const float right = .5f * (sum + difference), left = .5f * (sum - difference);
	const float right_room = math::max(0.f, math::min(right, 2.f * g.front_max - right));
	const float left_room = math::max(0.f, math::min(left, 2.f * g.front_max - left));
	const float room = right_room + left_room;
	const float reaction = math::constrain(reaction_torque / g.km, -room, room);
	// Right coaxial pair has -,+ reaction signs; the left pair has +,-.
	// Thus the total reaction coefficient is -dr + dl.
	const float dr = room > 1e-6f ? -reaction * right_room / room : 0.f;
	const float dl = reaction + dr;
	r.thrust[0] = .5f * (right + dr); r.thrust[1] = .5f * (right - dr);
	r.thrust[2] = .5f * (left + dl); r.thrust[3] = .5f * (left - dl);

	// At a saturated reaction moment, float roundoff can put one rotor a few
	// ulps below zero. Keep the public force solution inside physical bounds,
	// including callers that convert thrust to command with sqrt(thrust).
	for (int i = 0; i < 4; ++i) { r.thrust[i] = math::constrain(r.thrust[i], 0.f, g.front_max); }

	r.thrust[4] = solution.tail;
	forward(g, r);
	return r;
}

/** Refit the front force when M5 protection makes the requested tail thrust
 * unavailable. A small, bounded force deviation can retain pitch damping
 * while the tail is in its deadband or waiting to reverse. The original
 * requested wrench is retained for final-command residual reporting. */
inline ForcePitchSolution refitForTail(const Geometry &g, const matrix::Vector3f &force,
				       const matrix::Vector3f &torque, float tail_thrust, float previous_tilt, float angle_limit, float angle_step)
{
	ForcePitchSolution result;
	result.tail = math::constrain(tail_thrust, -tailMax(g, -1.f), g.tail_max);
	matrix::Vector2f front(force(0), -force(2) - result.tail); // (S sin a, S cos a)
	const matrix::Vector2f arm(g.z, g.x);
	const float pitch_error = torque(1) + g.tail * result.tail - arm.dot(front);
	// Weighted least squares: retain 80% of the feasible pitch correction.
	matrix::Vector2f correction = arm * (pitch_error / math::max(1.25f * arm.norm_squared(), 1e-6f));
	const float force_budget = .15f * force.norm();

	if (correction.norm() > force_budget && correction.norm() > 1e-6f) { correction *= force_budget / correction.norm(); }

	front += correction;
	const bool upright_lift_with_negative_front = force(2) < 0.f && front(1) < 0.f
			&& fabsf(force(0)) < .25f * fabsf(force(2));
	float target = !upright_lift_with_negative_front && front.norm_squared() > 1e-10f ? atan2f(front(0), front(1)) : 0.f;

	if (!upright_lift_with_negative_front && front.norm_squared() > 1e-10f) { target = previous_tilt + matrix::wrap_pi(target - previous_tilt); }

	result.tilt = math::constrain(math::constrain(target, -angle_limit, angle_limit),
				      previous_tilt - angle_step, previous_tilt + angle_step);
	result.front_sum = math::constrain(front.dot(matrix::Vector2f(sinf(result.tilt), cosf(result.tilt))), 0.f, 4.f * g.front_max);

	if (fabsf(target) > angle_limit) {
		const float moment_arm = g.z * sinf(result.tilt) + g.x * cosf(result.tilt);

		if (fabsf(moment_arm) > 1e-5f) {
			result.front_sum = math::constrain((torque(1) + g.tail * result.tail) / moment_arm, 0.f, 4.f * g.front_max);
		}
	}

	return result;
}

inline Result allocate(const Geometry &g, const matrix::Vector3f &force, const matrix::Vector3f &torque,
		       float previous_tilt, float angle_limit, float angle_step)
{
	const auto solution = allocateForcePitch(g, force, torque, previous_tilt, angle_limit, angle_step);
	return allocateRollYaw(g, solution, torque, reactionTorque(solution.tilt, torque));
}
} // namespace voliro_t
