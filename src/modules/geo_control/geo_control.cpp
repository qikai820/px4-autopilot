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

/**
 * @file geo_control.cpp
 *
 * Geometric Controller for MorphQuad.
 */

#include "geo_control.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

GeoControl::GeoControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

bool GeoControl::init()
{
    if (!_sub_vehicle_local_position.registerCallback()) {
        PX4_ERR("callback registration failed");
        return false;
    }

    return true;
}

void GeoControl::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s pupdate;
        _parameter_update_sub.copy(&pupdate);
        // update parameters from storage
        updateParams();
    }
}

void GeoControl::control_step()
{
    vehicle_attitude_s att;
    vehicle_angular_velocity_s angular_vel;
    vehicle_local_position_s local_pos;
    vehicle_local_position_setpoint_s local_pos_sp;
    vehicle_attitude_setpoint_s att_sp;

    if (!_sub_vehicle_local_position.update(&local_pos) ||
        !_sub_vehicle_attitude.update(&att) ||
        !_sub_vehicle_angular_velocity.update(&angular_vel) ||
        !_sub_local_pos_sp.update(&local_pos_sp)) {
        return;
    }

    _sub_vehicle_attitude_setpoint.update(&att_sp);

    // State vectors (NED frame)
    Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
    Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
    Vector3f pos_sp(local_pos_sp.x, local_pos_sp.y, local_pos_sp.z);
    Vector3f vel_sp(local_pos_sp.vx, local_pos_sp.vy, local_pos_sp.vz);
    Vector3f acc_sp(local_pos_sp.acceleration[0],
                    local_pos_sp.acceleration[1],
                    local_pos_sp.acceleration[2]);

    // Current attitude: rotation from body (FRD) to world (NED)
    matrix::Dcmf R(matrix::Quatf(att.q));

    // Angular velocity in body frame
    Vector3f omega(angular_vel.xyz[0], angular_vel.xyz[1], angular_vel.xyz[2]);

    // Desired attitude
    matrix::Dcmf R_sp(matrix::Quatf(att_sp.q_d));

    // Position and velocity errors
    Vector3f e_p = pos - pos_sp;
    Vector3f e_v = vel - vel_sp;

    // Attitude error via vee map: e_R = vee(0.5 * (R_sp^T R - R^T R_sp))
    matrix::Dcmf e_R_mat = (R_sp.transpose() * R - R.transpose() * R_sp) * 0.5f;
    Vector3f e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));

    // Angular velocity error
    Vector3f omega_sp(0.f, 0.f, 0.f);
    Vector3f omega_d_in_B = R.transpose() * R_sp * omega_sp;
    Vector3f e_omega = omega - omega_d_in_B;

    // Gains
    Vector3f kp(_param_gc_xy_p.get(), _param_gc_xy_p.get(), _param_gc_z_p.get());
    Vector3f kv(_param_gc_xy_vel_p.get(), _param_gc_xy_vel_p.get(), _param_gc_z_vel_p.get());
    Vector3f kR(_param_gc_rp_p.get(), _param_gc_rp_p.get(), _param_gc_yaw_p.get());
    Vector3f kOmega(_param_gc_rp_rate_p.get(), _param_gc_rp_rate_p.get(), _param_gc_yaw_rate_p.get());

    float mass = _param_gc_mass.get();

    // Inertia matrix (diagonal, from parameters)
    Matrix3f J = matrix::diag(Vector3f(
        _param_gc_inertia_xx.get(),
        _param_gc_inertia_yy.get(),
        _param_gc_inertia_zz.get()));

    // Geometric control law on SE(3) — NED convention
    // Gravity in NED: g = [0, 0, +9.81]^T
    // f_des_world = -Kp*e_p - Kv*e_v + m*g + m*a_sp
    //   (g compensates gravity downward, a_sp is feedforward)
    Vector3f g(0.f, 0.f, 9.81f);
    Vector3f f_des_world = -kp.emult(e_p) - kv.emult(e_v) + mass * g + mass * acc_sp;

    // Transform to body frame
    Vector3f f_des_body = R.transpose() * f_des_world;

    // Publish body-frame thrust (the full vector for MorphQuad allocator)
    vehicle_thrust_setpoint_s thrust_msg{};
    thrust_msg.timestamp = hrt_absolute_time();
    thrust_msg.timestamp_sample = local_pos.timestamp_sample;
    thrust_msg.xyz[0] = f_des_body(0);
    thrust_msg.xyz[1] = f_des_body(1);
    thrust_msg.xyz[2] = f_des_body(2);
    _thrust_pub.publish(thrust_msg);

    // Desired torque: tau = -K_R*e_R - K_Omega*e_omega + Omega x (J*Omega)
    Vector3f tau_des = -kR.emult(e_R) - kOmega.emult(e_omega) + omega.cross(J * omega);

    vehicle_torque_setpoint_s torque_msg{};
    torque_msg.timestamp = hrt_absolute_time();
    torque_msg.timestamp_sample = local_pos.timestamp_sample;
    torque_msg.xyz[0] = tau_des(0);
    torque_msg.xyz[1] = tau_des(1);
    torque_msg.xyz[2] = tau_des(2);
    _torque_pub.publish(torque_msg);
}

void GeoControl::Run()
{
    if (should_exit()) {
        _sub_vehicle_local_position.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // update parameters from storage
    parameters_update();

    vehicle_local_position_s local_pos;

    if (_sub_vehicle_local_position.update(&local_pos)) {
        //const float dt = math::constrain(((local_pos.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
        _last_run = local_pos.timestamp_sample;

        // 执行控制步骤
        control_step();
    }

    perf_end(_loop_perf);
}

int GeoControl::task_spawn(int argc, char *argv[])
{
    GeoControl *instance = new GeoControl();
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init()) {
            return PX4_OK;
        }
    }
    return PX4_ERROR;
}

GeoControl *GeoControl::instantiate(int argc, char *argv[])
{
    // 直接创建实例，init 在 task_spawn 中调用
    return new GeoControl();
}

int GeoControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int GeoControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Geometric controller for MorphQuad, outputting thrust and torque setpoints.

### Implementation
Implements a geometric tracking controller on SE(3) as described in "Geometric Tracking Control of a Quadrotor UAV on SE(3)".
Publishes `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` messages.

### Examples
CLI usage example:
$ geo_control start
$ geo_control status
$ geo_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("geo_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int geo_control_main(int argc, char *argv[])
{
    return GeoControl::main(argc, argv);
}
