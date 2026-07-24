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
 * @file geo_control.h
 *
 * Geometric Controller for MorphQuad.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class GeoControl : public ModuleBase<GeoControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    GeoControl();
    ~GeoControl() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static GeoControl *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    /**
     * Update our local parameter cache.
     */
    void parameters_update(bool force = false);

    /**
     * Compute and publish thrust and torque setpoints.
     */
    void control_step();

    // uORB Subscriptions
    uORB::SubscriptionCallbackWorkItem _sub_vehicle_local_position{this, ORB_ID(vehicle_local_position)};
    uORB::Subscription _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sub_vehicle_angular_velocity{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _sub_local_pos_sp{ORB_ID(vehicle_local_position_setpoint)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // uORB Publications
    uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};

    // Parameters (GC_ prefix)
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::GC_MASS>) _param_gc_mass,
        (ParamFloat<px4::params::GC_XY_P>) _param_gc_xy_p,
        (ParamFloat<px4::params::GC_XY_VEL_P>) _param_gc_xy_vel_p,
        (ParamFloat<px4::params::GC_Z_P>) _param_gc_z_p,
        (ParamFloat<px4::params::GC_Z_VEL_P>) _param_gc_z_vel_p,
        (ParamFloat<px4::params::GC_RP_P>) _param_gc_rp_p,
        (ParamFloat<px4::params::GC_YAW_P>) _param_gc_yaw_p,
        (ParamFloat<px4::params::GC_RP_RATE_P>) _param_gc_rp_rate_p,
        (ParamFloat<px4::params::GC_YAW_RATE_P>) _param_gc_yaw_rate_p
    )

    hrt_abstime _last_run{0};
    perf_counter_t _loop_perf;
};
