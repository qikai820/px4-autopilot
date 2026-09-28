/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

// Limit the final body-Z collective throttle immediately before control
// allocation. The attitude/rate torque commands remain under PX4 control.
class DockThrottleSlew
{
public:
	void reset() { _last = NAN; }

	float update(float automatic, float requested, float rate, float dt, float min_throttle, float max_throttle)
	{
		if (!std::isfinite(automatic)) {
			_last = NAN;
			return automatic;
		}

		if (!std::isfinite(requested) || !std::isfinite(rate) || rate <= 0.f) {
			_last = automatic;
			return automatic;
		}

		const float lower = fmaxf(0.f, fminf(min_throttle, 1.f));
		const float upper = fmaxf(lower, fminf(max_throttle, 1.f));
		const float target = fmaxf(lower, fminf(requested, upper));
		const float previous = std::isfinite(_last) ? _last : automatic;
		const float step = rate * fmaxf(dt, 0.f);
		_last = fmaxf(lower, fminf(previous + fmaxf(-step, fminf(target - previous, step)), upper));
		return _last;
	}

private:
	float _last{NAN};
};
