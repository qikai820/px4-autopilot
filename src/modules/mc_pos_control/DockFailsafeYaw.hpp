// Copyright (c) 2026 PX4 Development Team. BSD-3-Clause.
#pragma once

#include <cmath>
#include <cstdint>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

// Hold heading while braking a Dock fault, then align with RTL before translation resumes.
class DockFailsafeYaw {
public:
	struct Output {
		bool active{false};
		bool hold_translation{false};
		bool braking{false};
		bool release_to_rtl{false};
		bool aligned{false};
		float yaw{NAN};
	};

	void reset()
	{
		_active = false;
		_locked = true;
		_yaw = NAN;
		_last_target = NAN;
		_target_stable_s = 0.f;
		_align_elapsed_s = 0.f;
	}
	bool active() const { return _active; }

	void latch(float measured_yaw, uint8_t reset_counter)
	{
		if (std::isfinite(measured_yaw)) {
			_yaw = matrix::wrap_pi(measured_yaw);
			_reset_counter = reset_counter;
			_locked = _active = true;
			_last_target = NAN;
			_target_stable_s = 0.f;
			_align_elapsed_s = 0.f;
		}
	}

	Output update(bool dock_fault_rtl, float speed, float measured_yaw, float rtl_yaw,
		      float release_speed, float yaw_rate, float dt, uint8_t reset_counter, float delta_heading)
	{
		if (!_active) { return {}; }

		if (!dock_fault_rtl || !std::isfinite(speed) || speed < 0.f || !std::isfinite(measured_yaw)
		    || !std::isfinite(release_speed) || release_speed <= 0.f
		    || !std::isfinite(yaw_rate) || yaw_rate <= 0.f || !std::isfinite(dt) || dt < 0.f) {
			reset();
			return {};
		}

		if (reset_counter != _reset_counter) {
			// A change of EKF heading frame is not a real aircraft turn.
			if (static_cast<uint8_t>(reset_counter - _reset_counter) != 1 || !std::isfinite(delta_heading)) {
				reset();
				return {};
			}
			_yaw = matrix::wrap_pi(_yaw + delta_heading);
			if (std::isfinite(_last_target)) { _last_target = matrix::wrap_pi(_last_target + delta_heading); }
			_reset_counter = reset_counter;
		}

		if (speed >= release_speed) {
			if (!_locked) {
				_yaw = matrix::wrap_pi(measured_yaw);
				_last_target = NAN;
				_target_stable_s = 0.f;
				_align_elapsed_s = 0.f;
			}
			_locked = true;
			return {true, true, true, false, false, _yaw};

		}

		_locked = false;
		if (!std::isfinite(rtl_yaw)) {
			// No known return heading: return control to the native RTL path.
			reset();
			return {false, false, false, true, false, NAN};
		}
		_align_elapsed_s += fminf(dt, .1f);
		const float target = matrix::wrap_pi(rtl_yaw);
		if (std::isfinite(_last_target) && fabsf(matrix::wrap_pi(target - _last_target)) < math::radians(.25f)) {
			_target_stable_s += fminf(dt, .1f);
		} else {
			_target_stable_s = 0.f;
		}
		_last_target = target;
		const float error = matrix::wrap_pi(target - _yaw);
		const float step = yaw_rate * fminf(dt, .1f);
		_yaw = matrix::wrap_pi(_yaw + fmaxf(-step, fminf(error, step)));

		// Check actual attitude as well as the command. A fast RTL yaw setpoint may
		// briefly pass the held heading while still rotating toward the return direction.
		if (_target_stable_s >= .5f && fabsf(matrix::wrap_pi(target - _yaw)) < math::radians(2.f)
		    && fabsf(matrix::wrap_pi(target - measured_yaw)) < math::radians(5.f)) {
			reset();
			return {false, false, false, true, true, NAN};
		}
		// A failed yaw actuator must not leave the aircraft hovering indefinitely.
		const float maximum_alignment_s = fminf(30.f, fmaxf(5.f, 2.f * M_PI_F / yaw_rate + 2.f));
		if (_align_elapsed_s >= maximum_alignment_s) {
			reset();
			return {false, false, false, true, false, NAN};
		}

		return {true, true, false, false, false, _yaw};
	}

private:
	float _yaw{NAN};
	float _last_target{NAN};
	float _target_stable_s{0.f};
	float _align_elapsed_s{0.f};
	uint8_t _reset_counter{0};
	bool _active{false};
	bool _locked{true};
};
