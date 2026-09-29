// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <matrix/matrix/math.hpp>

// Limit the velocity-setpoint jump when control returns from a stationary yaw
// alignment to an RTL trajectory that has continued planning in the background.
class DockRtlVelocityRamp {
public:
	void start() { _active = true; _velocity.zero(); }
	void reset() { _active = false; _velocity.zero(); }
	bool active() const { return _active; }

	matrix::Vector3f update(const matrix::Vector3f &target, float acceleration, float dt)
	{
		if (!_active || !target.isAllFinite() || !std::isfinite(acceleration)
		    || acceleration <= 0.f || !std::isfinite(dt) || dt <= 0.f) {
			reset();
			return target;
		}

		const matrix::Vector3f change = target - _velocity;
		const float limit = acceleration * fminf(dt, .1f);
		if (change.norm() <= limit) {
			reset();
			return target;
		}
		_velocity += change * (limit / change.norm());
		return _velocity;
	}

private:
	bool _active{false};
	matrix::Vector3f _velocity{};
};
