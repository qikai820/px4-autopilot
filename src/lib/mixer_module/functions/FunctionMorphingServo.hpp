/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "FunctionProviderBase.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <uORB/topics/manual_control_setpoint.h>

/**
 * Function: Morphing_Servo
 *
 * Routes manual-control AUX1 to a shape-changing servo. The output is rate
 * limited in PX4 so that a transmitter-side AUX step cannot abruptly change
 * the airframe geometry. It intentionally has no control-allocation input.
 */
class FunctionMorphingServo : public FunctionProviderBase
{
public:
	FunctionMorphingServo()
	{
		_slew_param = param_find("MORPH_SLEW");
		updateSlewRate();
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionMorphingServo(); }

	void update() override
	{
		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			if (manual_control_setpoint.valid && PX4_ISFINITE(manual_control_setpoint.aux1)) {
				_target = math::constrain(manual_control_setpoint.aux1, -1.f, 1.f);
				_valid = true;

			} else {
				_valid = false;
				_initialized = false;
				_output = NAN;
			}
		}

		if (!_valid) {
			return;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (!_initialized) {
			// Start from the mechanical centre, then always respect MORPH_SLEW.
			_output = 0.f;
			_last_update = now;
			_initialized = true;
			return;
		}

		const float dt = math::constrain((now - _last_update) * 1e-6f, 0.f, 0.1f);
		_last_update = now;
		updateSlewRate();

		const float max_delta = _slew_rate * dt;
		_output += math::constrain(_target - _output, -max_delta, max_delta);
	}

	float value(OutputFunction func) override { return _output; }

private:
	void updateSlewRate()
	{
		float slew_rate{};

		if (_slew_param != PARAM_INVALID && param_get(_slew_param, &slew_rate) == PX4_OK) {
			_slew_rate = math::constrain(slew_rate, 0.01f, 10.f);
		}
	}

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	param_t _slew_param{PARAM_INVALID};
	hrt_abstime _last_update{};
	float _target{};
	float _output{NAN};
	float _slew_rate{0.25f};
	bool _valid{false};
	bool _initialized{false};
};
