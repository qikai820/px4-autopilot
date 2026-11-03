#pragma once

// PX4's ERROR macro conflicts with JSBSim's scoped LogLevel::ERROR enum.
#pragma push_macro("ERROR")
#undef ERROR
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGPropagate.h>
#pragma pop_macro("ERROR")
#include <matrix/matrix/math.hpp>
#include <cassert>

/** In-process JSBSim plant. The runner removes network input/output blocks
 * from a temporary model copy; this never connects to a vehicle or bridge. */
class NativeVoliroPlant
{
public:
	NativeVoliroPlant(const char *model, const char *systems, const matrix::Quatf &q,
			  const float trim_commands[5], float trim_tilt, float dt) : _dt(dt)
	{
		_fdm.SetDebugLevel(0);
		_fdm.Setdt(dt); // initialize FCS lag coefficients at the actual step size
		assert(_fdm.LoadModel(SGPath(model), SGPath(model), SGPath(systems), "voliro_t", false));
		const matrix::Eulerf euler(q);
		_fdm.GetIC()->SetAltitudeASLFtIC(10000.);
		_fdm.GetIC()->SetPhiRadIC(euler.phi());
		_fdm.GetIC()->SetThetaRadIC(euler.theta());
		_fdm.GetIC()->SetPsiRadIC(euler.psi());
		assert(_fdm.RunIC());
		assert((orientation().inversed() * q).canonical().imag().norm() < .001f);
		setCommands(trim_commands, trim_tilt);
		// Warm actuator states with the rigid body held at its initial state.
		_fdm.SuspendIntegration();

		for (int i = 0; i < 1200; ++i) { assert(_fdm.Run()); }

		_fdm.ResumeIntegration();
	}

	void step(const float commands[5], float tilt, matrix::Quatf &q, matrix::Vector3f &omega,
		  matrix::Vector3f &angular_acceleration, matrix::Vector3f &position,
		  matrix::Vector3f &velocity, matrix::Vector3f &acceleration)
	{
		setCommands(commands, tilt);
		assert(_fdm.Run());
		q = orientation();
		const auto propagation = _fdm.GetPropagate();
		const matrix::Vector3f next_omega(propagation->GetPQR(1), propagation->GetPQR(2), propagation->GetPQR(3));
		const matrix::Vector3f next_velocity = matrix::Vector3f(propagation->GetVel(1), propagation->GetVel(2), propagation->GetVel(3)) * .3048f;
		angular_acceleration = (next_omega - omega) / _dt;
		acceleration = (next_velocity - velocity) / _dt;
		omega = next_omega; velocity = next_velocity;
		position += velocity * _dt;
	}

private:
	matrix::Quatf orientation() const
	{
		const auto q = _fdm.GetPropagate()->GetQuaternion();
		return matrix::Quatf(q(1), q(2), q(3), q(4));
	}
	void setCommands(const float commands[5], float tilt)
	{
		for (int i = 0; i < 5; ++i) {
			char property[64];
			snprintf(property, sizeof(property), "fcs/voliro_t/motor-cmd[%d]", i);
			_fdm.SetPropertyValue(property, commands[i]);
		}

		_fdm.SetPropertyValue("fcs/voliro_t/pitch-cmd-norm", tilt / math::radians(120.f));
	}
	JSBSim::FGFDMExec _fdm;
	float _dt;
};
