#pragma once

#include "../../vector_rate_control/RateControl/rate_control.hpp"
#include <lib/vector_control/VectorReferenceGains.hpp>

// The normalized VC_* gains and limits from the mode-0 real-flight baseline.
// Tests call the same PID/reference-feedforward implementation as flight code.
inline void configureBaselineRate(VectorRateCore &rate)
{
	rate.setPidGains(matrix::Vector3f(.26f, .1f, .3f), matrix::Vector3f(.2f, .4f, .1f),
			 matrix::Vector3f(.003f, .001f, 0.f));
	rate.setIntegratorLimit(matrix::Vector3f(.3f, .3f, .3f));
	rate.setFeedForwardGain({});
}

inline matrix::Vector3f baselineRateTorque(VectorRateCore &rate, const matrix::Vector3f &omega,
		const matrix::Vector3f &rate_sp, const matrix::Vector3f &alpha,
		const matrix::Vector3f &alpha_ff, const matrix::Vector3f &residual_nm,
		const matrix::Vector3f &inertia, float dt, const matrix::Quatf &attitude = {})
{
	matrix::Vector3<bool> positive, negative;

	for (int i = 0; i < 3; ++i) {
		positive(i) = residual_nm(i) > FLT_EPSILON;
		negative(i) = residual_nm(i) < -FLT_EPSILON;
	}

	rate.setSaturationStatus(positive, negative);
	const matrix::Vector3f integral_rate_scale(1.f, vector_control::referenceFeedbackScale(attitude, 1.f / 3.f), 1.f);
	return rate.update(omega, rate_sp, alpha, dt, false, alpha_ff, inertia / 10.f, integral_rate_scale) * 10.f;
}
