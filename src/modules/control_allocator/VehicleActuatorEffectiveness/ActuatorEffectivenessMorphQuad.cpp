/****************************************************************************
 * MorphQuad actuator effectiveness / thrust allocation
 *
 * Coordinate convention used by PX4 and JSBSim BODY:
 *   x: forward
 *   y: right
 *   z: down
 *
 * The paper writes its thrust vector with a positive-up z axis.  The
 * implementation below uses the PX4 BODY vector, kept consistent with the
 * JSBSim model (GimbalVectoring.xml: ty = +sin(beta)):
 *
 *   g_body(alpha,beta) = [ sin(alpha) cos(beta),
 *                         +sin(beta),
 *                         -cos(alpha) cos(beta) ]^T
 *
 * The allocation itself is the paper's constant 6x12 matrix M:
 *   w = M t
 * and NOT an angle-dependent 6x4 pseudoinverse.  This distinction is
 * essential: M^+ gives the 12-D thrust-vector solution directly, so the
 * servo angles are recovered after allocation and do not need an ad-hoc
 * Gauss-Newton angle iteration.
 ****************************************************************************/

#include "ActuatorEffectivenessMorphQuad.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <cmath>

using namespace matrix;

namespace
{
// The installed servos have a +/-180 degree mechanical range.  PX4 actuator
// setpoints are normalized to [-1, 1], so this is the shared conversion used
// by the allocator and the JSBSim bridge.
constexpr float kServoAngleRange = M_PI_F;

Vector3f thrustUnitVector(float alpha, float beta)
{
	const float sa = sinf(alpha);
	const float ca = cosf(alpha);
	const float sb = sinf(beta);
	const float cb = cosf(beta);

	return Vector3f(sa * cb, sb, -ca * cb);
}
}

ActuatorEffectivenessMorphQuad::ActuatorEffectivenessMorphQuad(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool ActuatorEffectivenessMorphQuad::getEffectivenessMatrix(
	Configuration &configuration, EffectivenessUpdateReason external_update)
{
	// Pattern B: the wrench-to-actuator mapping is nonlinear because the
	// servo angles are part of the actuator state.  We therefore register
	// the physical actuators here and perform the paper allocation in
	// updateSetpoint().
	for (int i = 0; i < NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::MOTORS,
					 Vector3f{}, Vector3f{0.f, 0.f, -1.f});
	}

	for (int i = 0; i < 2 * NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessMorphQuad::updateSetpoint(
	const Vector<float, NUM_AXES> &control_sp, int matrix_index,
	ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min,
	const ActuatorVector &actuator_max)
{
	(void)matrix_index;

	// PX4 ControlAllocator supplies:
	// [Mx, My, Mz, Fx, Fy, Fz]
	const Vector3f tau_des(control_sp(ControlAxis::ROLL),
			      control_sp(ControlAxis::PITCH),
			      control_sp(ControlAxis::YAW));

	const Vector3f f_req(control_sp(ControlAxis::THRUST_X),
			     control_sp(ControlAxis::THRUST_Y),
			     control_sp(ControlAxis::THRUST_Z));

	_motor_saturated = false;
	_servo_saturated = false;
	_allocation_saturated = false;

	float t[NUM_ROTORS][3] {};
	_allocation_saturated |= computeThrustVectors(f_req, tau_des, t);

	const float tmax = math::max(_param_ca_mq_tmax.get(), 0.001f);

	float omega_cmd[NUM_ROTORS] {};
	float alpha_cmd[NUM_ROTORS] {};
	float beta_cmd[NUM_ROTORS] {};

	for (int i = 0; i < NUM_ROTORS; ++i) {
		const float T = sqrtf(t[i][0] * t[i][0] +
				      t[i][1] * t[i][1] +
				      t[i][2] * t[i][2]);

		float alpha = _alpha_prev[i];
		float beta = _beta_prev[i];
		_servo_saturated |= recoverServoAngles(t[i], i, alpha, beta);

		// T = c_t * Omega^2 and TMAX = c_t * OMEGA_MAX^2.
		// Therefore the normalized rotor-speed command is
		//     Omega/Omega_MAX = sqrt(T/TMAX).
		omega_cmd[i] = (T > 1e-5f) ? sqrtf(math::constrain(T / tmax, 0.f, 1.f)) : 0.f;

		// PX4 actuator outputs are normalized to [-1, 1], and the bridge
		// maps +/-1 to the installed servo's +/-180 degree range.
		alpha_cmd[i] = alpha / kServoAngleRange;
		beta_cmd[i] = beta / kServoAngleRange;

		// 假设 MOTORS 占 0..3，SERVOS 占 4..11
		static constexpr int SERVO_OFFSET = NUM_ROTORS; // 4

		const int motor_idx = i;
		const int alpha_idx = SERVO_OFFSET + 2 * i;
		const int beta_idx  = SERVO_OFFSET + 2 * i + 1;

		const float motor_actual = math::constrain(omega_cmd[i],
							  actuator_min(motor_idx),
							  math::min(actuator_max(motor_idx),
									math::constrain(_param_ca_mq_mmax.get(), 0.f, 1.f)));
		const float alpha_actual = math::constrain(alpha_cmd[i],
							   actuator_min(alpha_idx), actuator_max(alpha_idx));
		const float beta_actual = math::constrain(beta_cmd[i],
							  actuator_min(beta_idx), actuator_max(beta_idx));

		_motor_saturated |= fabsf(motor_actual - omega_cmd[i]) > 1e-6f;
		_servo_saturated |= fabsf(alpha_actual - alpha_cmd[i]) > 1e-6f;
		_servo_saturated |= fabsf(beta_actual - beta_cmd[i]) > 1e-6f;

		actuator_sp(motor_idx) = motor_actual;
		actuator_sp(alpha_idx) = alpha_actual;
		actuator_sp(beta_idx) = beta_actual;

		_alpha_prev[i] = alpha_actual * kServoAngleRange;
		_beta_prev[i] = beta_actual * kServoAngleRange;
	}

	computeUnallocatedWrench(f_req, tau_des, actuator_sp);
}

Vector<float, ActuatorEffectivenessMorphQuad::WRENCH_AXES>
ActuatorEffectivenessMorphQuad::computeNullspaceCoefficient(const Vector3f &f_des) const
{
	Vector<float, WRENCH_AXES> c{};
	const float f_norm = f_des.norm();

	if (f_norm < 1e-5f) {
		return c;
	}

	const Vector3f f_hat = f_des / f_norm;

	// Eq. (16) of MorphQuad.
	// c1 = c3 = c5 = <f_hat, j_B>
	c(0) = f_hat(1);
	c(2) = f_hat(1);
	c(4) = f_hat(1);

	// c2, c4, c6 are selected to break the downwash collinearity
	// around the body x axis. This is exactly the conditional rule in Eq.16.
	if (f_hat(0) > 0.f) {
		c(1) = -f_hat(0);
		c(3) = +f_hat(0);
		c(5) = 0.f;

	} else {
		c(1) = 0.f;
		c(3) = 0.f;
		c(5) = -f_hat(0);
	}

	return c;
}

bool ActuatorEffectivenessMorphQuad::computeThrustVectors(
	const Vector3f &f_des, const Vector3f &tau_des,
	float t_out[NUM_ROTORS][3])
{
	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float z = _param_ca_mq_z.get();

	// Rotor order is the same as the paper's M matrix:
	//   0 RF (+l,+b)
	//   1 LF (+l,-b)
	//   2 LB (-l,-b)
	//   3 RB (-l,+b)
	//
	// For each rotor r_i=[x_i,y_i,z], the torque is r_i x t_i.
	// Hence the constant input matrix is exactly:
	//
	// [ 1 0 0 | 1 0 0 | 1 0 0 | 1 0 0 ]
	// [ 0 1 0 | 0 1 0 | 0 1 0 | 0 1 0 ]
	// [ 0 0 1 | 0 0 1 | 0 0 1 | 0 0 1 ]
	// [-z 0 b | -z 0-b | -z 0-b | -z 0 b ]
	// [ z 0-l |  z 0-l |  z 0 l |  z 0 l ]
	// [ -b l 0 | b l 0 | b -l 0 | -b -l 0 ]
	//
	// This is the paper's Eq. (9)/(21), expressed in the PX4 body frame.
	Matrix<float, WRENCH_AXES, NUM_THRUST_COMPONENTS> M{};

	const float x[NUM_ROTORS] = { l, l, -l, -l };
	const float y[NUM_ROTORS] = { b, -b, -b, b };

	for (int i = 0; i < NUM_ROTORS; ++i) {
		const int j = 3 * i;

		M(0, j + 0) = 1.f;
		M(1, j + 1) = 1.f;
		M(2, j + 2) = 1.f;

		// r x t = [y*tz-z*ty, z*tx-x*tz, x*ty-y*tx]
		M(3, j + 1) = -z;
		M(3, j + 2) = y[i];
		M(4, j + 0) = z;
		M(4, j + 2) = -x[i];
		M(5, j + 0) = -y[i];
		M(5, j + 1) = x[i];
	}

	Vector<float, WRENCH_AXES> w{};
	w(0) = f_des(0);
	w(1) = f_des(1);
	w(2) = f_des(2);
	w(3) = tau_des(0);
	w(4) = tau_des(1);
	w(5) = tau_des(2);

	Matrix<float, NUM_THRUST_COMPONENTS, WRENCH_AXES> M_pinv{};

	if (!geninv(M, M_pinv)) {
		for (int i = 0; i < NUM_ROTORS; ++i) {
			t_out[i][0] = 0.f;
			t_out[i][1] = 0.f;
			t_out[i][2] = 0.f;
		}
		return true;
	}

	// Minimum-norm solution: t0 = M^+ w.
	Vector<float, NUM_THRUST_COMPONENTS> t0 = M_pinv * w;

	// The paper uses an explicit 12x6 null-space BASIS N_M (Eq.19), not a
	// 12x12 projector.  Keeping the explicit basis avoids an SVD/QR step on
	// the flight controller and exactly matches the six coefficients c in Eq.16.
	Matrix<float, NUM_THRUST_COMPONENTS, WRENCH_AXES> N_M{};

	// N_M^T from paper Eq. (19), transposed here into a 12x6 matrix.
	// The basis satisfies M*N_M = 0 for any non-zero l,b.
	const float l_over_b = l / math::max(fabsf(b), 1e-6f);

	// Column c1
	N_M(0, 0) = 0.f; N_M(1, 0) = -1.f; N_M(2, 0) = 0.f;
	N_M(3, 0) = 0.f; N_M(4, 0) = 1.f;  N_M(5, 0) = 0.f;
	N_M(6, 0) = 0.f; N_M(7, 0) = 0.f;  N_M(8, 0) = 0.f;
	N_M(9, 0) = 0.f; N_M(10,0) = 0.f;  N_M(11,0) = 0.f;

	// Column c2
	N_M(0, 1) = 0.f; N_M(1, 1) = 0.f; N_M(2, 1) = 0.f;
	N_M(3, 1) = -1.f; N_M(4, 1) = 0.f; N_M(5, 1) = 0.f;
	N_M(6, 1) = 1.f; N_M(7, 1) = 0.f; N_M(8, 1) = 0.f;
	N_M(9, 1) = 0.f; N_M(10,1) = 0.f; N_M(11,1) = 0.f;

	// Column c3
	N_M(0, 2) = -l_over_b; N_M(1, 2) = -1.f; N_M(2, 2) = 0.f;
	N_M(3, 2) = l_over_b;  N_M(4, 2) = 0.f; N_M(5, 2) = 0.f;
	N_M(6, 2) = 0.f; N_M(7, 2) = 1.f; N_M(8, 2) = 0.f;
	N_M(9, 2) = 0.f; N_M(10,2) = 0.f; N_M(11,2) = 0.f;

	// Column c4
	N_M(0, 3) = -1.f; N_M(1, 3) = 0.f; N_M(2, 3) = 0.f;
	N_M(3, 3) = 0.f; N_M(4, 3) = 0.f; N_M(5, 3) = 0.f;
	N_M(6, 3) = 0.f; N_M(7, 3) = 0.f; N_M(8, 3) = 0.f;
	N_M(9, 3) = 1.f; N_M(10,3) = 0.f; N_M(11,3) = 0.f;

	// Column c5
	N_M(0, 4) = -l_over_b; N_M(1, 4) = -1.f; N_M(2, 4) = 0.f;
	N_M(3, 4) = l_over_b;  N_M(4, 4) = 0.f; N_M(5, 4) = 0.f;
	N_M(6, 4) = 0.f; N_M(7, 4) = 0.f; N_M(8, 4) = 0.f;
	N_M(9, 4) = 0.f; N_M(10,4) = 1.f; N_M(11,4) = 0.f;

	// Column c6
	N_M(0, 5) = 0.f; N_M(1, 5) = 0.f; N_M(2, 5) = -1.f;
	N_M(3, 5) = 0.f; N_M(4, 5) = 0.f; N_M(5, 5) = 1.f;
	N_M(6, 5) = 0.f; N_M(7, 5) = 0.f; N_M(8, 5) = -1.f;
	N_M(9, 5) = 0.f; N_M(10,5) = 0.f; N_M(11,5) = 1.f; 

	// The basis satisfies M*N_M = 0 for any non-zero l,b.  Validate it now
	// that N_M is fully populated; checking on an all-zero matrix before the
	// fill would pass trivially and detect nothing.
	{
		static bool nullspace_error_reported = false;
		Matrix<float, WRENCH_AXES, WRENCH_AXES> M_N_M = M * N_M;

		for(int r = 0; r < WRENCH_AXES; ++r) {
			for(int c = 0; c < WRENCH_AXES; ++c) {
				if (fabsf(M_N_M(r, c)) > 1e-3f && !nullspace_error_reported) {
					PX4_ERR("Nullspace basis N_M is inconsistent with M at (%d,%d): %f", r, c, (double)M_N_M(r, c));
					nullspace_error_reported = true;
				}
			}
		}
	}

	// Eq. (16): use the force direction to redistribute the 12-D thrust
	// vector without changing the commanded wrench.
	const Vector<float, WRENCH_AXES> c = computeNullspaceCoefficient(f_des);
	Vector<float, NUM_THRUST_COMPONENTS> t = t0 + N_M * c;

	// Physical constraint: each rotor-system thrust magnitude is bounded by TMAX.
	// We do NOT clip individual vector components, because that changes its
	// direction. Instead scale the complete 12-D solution if any rotor exceeds
	// its magnitude limit. This preserves all thrust-vector directions and gives
	// a well-defined residual wrench that computeUnallocatedWrench() can report.
	const float tmax = math::max(_param_ca_mq_tmax.get(), 0.001f);
	float scale = 1.f;

	for (int i = 0; i < NUM_ROTORS; ++i) {
		const float T = sqrtf(t(3 * i + 0) * t(3 * i + 0) +
				      t(3 * i + 1) * t(3 * i + 1) +
				      t(3 * i + 2) * t(3 * i + 2));

		if (T > tmax) {
			scale = math::min(scale, tmax / T);
		}
	}

	if (scale < 1.f) {
		t *= scale;
	}

	for (int i = 0; i < NUM_ROTORS; ++i) {
		t_out[i][0] = t(3 * i + 0);
		t_out[i][1] = t(3 * i + 1);
		t_out[i][2] = t(3 * i + 2);
	}

	return scale < 1.f;
}

bool ActuatorEffectivenessMorphQuad::recoverServoAngles(
	const float t[3], int idx, float &alpha_out, float &beta_out)
{
	const float norm = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);

	if (norm < 1e-5f) {
		alpha_out = _alpha_prev[idx];
		beta_out = _beta_prev[idx];
		return false;
	}

	const float tx = t[0] / norm;
	const float ty = t[1] / norm;
	const float tz = t[2] / norm;

	// PX4 BODY is FRD. The paper uses a positive-up z in Eq. (20).
	// The JSBSim model (GimbalVectoring.xml) defines the body thrust unit
	// vector as (sin a, sin b, -sqrt(1-..)), i.e. ty = +sin(beta). Keep the
	// allocator consistent with that physical convention:
	//   beta = asin(ty)
	//   alpha = atan2(tx, -tz)
	const float beta1 = asinf(math::constrain(ty, -1.f, 1.f));
	const float alpha1 = atan2f(tx, -tz);

	// Equivalent second trigonometric branch.
	const float beta2 = M_PI_F - beta1;
	const float alpha2 = M_PI_F + alpha1;

	// Equation (14) of the paper selects the equivalent inverse-kinematics
	// branch nearest to the previous servo state.  The actual servo range is
	// +/-180 degrees, so apply that hardware bound here.
	const float servo_global_limit = math::radians(_param_ca_mq_smax.get());
	const float alpha_limit = math::max(0.f, math::min(math::min(math::radians(_param_ca_mq_alpm.get()),
									 servo_global_limit), kServoAngleRange));
	const float beta_limit  = math::max(0.f, math::min(math::min(math::radians(_param_ca_mq_betm.get()),
									 servo_global_limit), kServoAngleRange));

	float best_alpha = alpha1;
	float best_beta = beta1;
	float best_cost = INFINITY;
	bool angle_limited = false;

	// Compare both branches and multiple revolutions against the previous
	// measured/commanded servo state. This is the paper's minimal-motion rule.
	for (int branch = 0; branch < 2; ++branch) {
		const float a0 = (branch == 0) ? alpha1 : alpha2;
		const float b0 = (branch == 0) ? beta1 : beta2;

		for (int ka = -NR; ka <= NR; ++ka) {
			const float a = a0 + 2.f * M_PI_F * static_cast<float>(ka);

			if (a < -alpha_limit - 1e-4f ||
			    a >  alpha_limit + 1e-4f) {
				continue;
			}

			for (int kb = -NR; kb <= NR; ++kb) {
				const float b = b0 + 2.f * M_PI_F * static_cast<float>(kb);

				if (b < -beta_limit - 1e-4f ||
				    b >  beta_limit + 1e-4f) {
					continue;
				}

				const float cost = fabsf(a - _alpha_prev[idx]) + fabsf(b - _beta_prev[idx]);

				if (cost < best_cost) {
					best_cost = cost;
					best_alpha = a;
					best_beta = b;
				}
			}
		}
	}

	// If no candidate survived the configured mechanical range, clamp the
	// closest principal solution. The achieved-wrench feedback will expose the
	// resulting allocation loss.
	if (!PX4_ISFINITE(best_cost)) {
		best_alpha = math::constrain(alpha1, -alpha_limit, alpha_limit);
		best_beta = math::constrain(beta1, -beta_limit, beta_limit);
		angle_limited = true;
	}

	const Vector3f target_dir(tx, ty, tz);
	const Vector3f achieved_dir = thrustUnitVector(best_alpha, best_beta);
	angle_limited |= (target_dir - achieved_dir).norm() > 1e-3f;

	_alpha_prev[idx] = best_alpha;
	_beta_prev[idx] = best_beta;
	alpha_out = best_alpha;
	beta_out = best_beta;

	return angle_limited;
}

void ActuatorEffectivenessMorphQuad::computeUnallocatedWrench(
	const Vector3f &f_des, const Vector3f &tau_des,
	const ActuatorVector &actuator_sp)
{
	_unallocated_torque[0] = 0.f;
	_unallocated_torque[1] = 0.f;
	_unallocated_torque[2] = 0.f;
	_unallocated_thrust[0] = 0.f;
	_unallocated_thrust[1] = 0.f;
	_unallocated_thrust[2] = 0.f;

	// Keep the status quiet in normal operation while still feeding anti-windup
	// for all allocation losses: internal feasibility gates, thrust scaling,
	// servo angle limiting, and PX4 actuator output clipping.
	if (!_allocation_saturated && !_motor_saturated && !_servo_saturated) {
		return;
	}

	const float l = _param_ca_mq_l.get();
	const float b = _param_ca_mq_b.get();
	const float z = _param_ca_mq_z.get();
	const float tmax = math::max(_param_ca_mq_tmax.get(), 0.001f);

	const float x[NUM_ROTORS] = { l, l, -l, -l };
	const float y[NUM_ROTORS] = { b, -b, -b, b };

	Vector3f f_achieved(0.f, 0.f, 0.f);
	Vector3f tau_achieved(0.f, 0.f, 0.f);

	for (int i = 0; i < NUM_ROTORS; ++i) {
		const float esc = actuator_sp(i);
		// Normalized +/-1 maps to the installed servo's +/-180 degree range.
		const float alpha = actuator_sp(4 + 2 * i) * kServoAngleRange;
		const float beta = actuator_sp(5 + 2 * i) * kServoAngleRange;

		// Same actuator model as the JSBSim XML: T = TMAX * esc^2.
		const float T = esc * esc * tmax;

		// Body thrust unit vector. Must match JSBSim (GimbalVectoring.xml):
		//   tx = sin(a)cos(b), ty = +sin(b), tz = -cos(a)cos(b)
		const Vector3f g = thrustUnitVector(alpha, beta);
		const Vector3f t = T * g;
		const Vector3f r(x[i], y[i], z);

		f_achieved += t;
		tau_achieved += r.cross(t);
	}

	const Vector3f df = f_des - f_achieved;
	const Vector3f dtau = tau_des - tau_achieved;

	for (int i = 0; i < 3; ++i) {
		_unallocated_thrust[i] = df(i);
		_unallocated_torque[i] = dtau(i);
	}
}

void ActuatorEffectivenessMorphQuad::getUnallocatedControl(
	int matrix_index, control_allocator_status_s &status)
{
	(void)matrix_index;

	for (int i = 0; i < 3; ++i) {
		status.unallocated_torque[i] = _unallocated_torque[i];
		status.unallocated_thrust[i] = _unallocated_thrust[i];
	}
}
