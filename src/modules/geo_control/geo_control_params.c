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
 * @file geo_control_params.c
 *
 * Parameters for the Geometric Controller.
 */

/**
 * Geometric Controller Mass
 *
 * @unit kg
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_MASS, 4.56);

/**
 * Geometric Controller maximum execution rate
 *
 * @unit Hz
 * @min 10
 * @max 400
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_RATE_HZ, 400.0);

/**
 * Geometric Controller Position Proportional Gain (XY)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_XY_P, 1.0);

/**
 * Geometric Controller Velocity Proportional Gain (XY)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_XY_VEL_P, 0.5);

/**
 * Geometric Controller Position Proportional Gain (Z)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_Z_P, 1.5);

/**
 * Geometric Controller Velocity Proportional Gain (Z)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_Z_VEL_P, 0.8);

/**
 * Geometric Controller Attitude Proportional Gain (Roll/Pitch)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_RP_P, 6.0);

/**
 * Geometric Controller Attitude Proportional Gain (Yaw)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_YAW_P, 4.0);

/**
 * Geometric Controller Angular Rate Proportional Gain (Roll/Pitch)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_RP_RATE_P, 0.8);

/**
 * Geometric Controller Angular Rate Proportional Gain (Yaw)
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_YAW_RATE_P, 0.7);

/**
 * Geometric Controller Inertia Matrix XX
 *
 * @unit kg m^2
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_INERTIA_XX, 0.1);

/**
 * Geometric Controller Inertia Matrix YY
 *
 * @unit kg m^2
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_INERTIA_YY, 0.1);

/**
 * Geometric Controller Inertia Matrix ZZ
 *
 * @unit kg m^2
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_INERTIA_ZZ, 0.2);

/**
 * Geometric Controller Manual Mode Max Angular Rate
 *
 * Maximum angular rate for manual mode stick input.
 * Stick deflection of +/-1 maps to +/- this value.
 *
 * @unit rad/s
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_MAN_RATE_MAX, 1.5);

/**
 * Geometric Controller Manual Mode Max Velocity
 *
 * Maximum velocity for manual mode aux stick input.
 * Stick deflection of +/-1 maps to +/- this value.
 *
 * @unit m/s
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_MAN_VEL_MAX, 5.0);

/**
 * Geometric Controller maximum X velocity setpoint
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_VX_MAX, 2.0);

/**
 * Geometric Controller maximum Y velocity setpoint
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_VY_MAX, 2.0);

/**
 * Geometric Controller maximum Z velocity setpoint
 *
 * Positive NED Z is downward.
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_VZ_MAX, 1.0);

/**
 * Geometric Controller maximum X setpoint acceleration
 *
 * Limits the rate of change of the NED X velocity setpoint.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_AX_MAX, 2.0);

/**
 * Geometric Controller maximum Y setpoint acceleration
 *
 * Limits the rate of change of the NED Y velocity setpoint.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_AY_MAX, 2.0);

/**
 * Geometric Controller maximum Z setpoint acceleration
 *
 * Limits the rate of change of the NED Z velocity setpoint.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_AZ_MAX, 1.0);

/**
 * Geometric Controller maximum manual roll angle
 *
 * @unit deg
 * @min 1.0
 * @max 85.0
 * @decimal 0
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_ROLL_MAX, 20.0);

/**
 * Geometric Controller maximum manual pitch angle
 *
 * @unit deg
 * @min 1.0
 * @max 85.0
 * @decimal 0
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_PITCH_MAX, 20.0);

/**
 * Geometric Controller Manual Mode Stick Deadzone
 *
 * Deadzone applied to all stick inputs in manual mode.
 * Stick values within [-dz, +dz] are treated as zero.
 * Values outside are rescaled linearly to maintain full range.
 *
 * @min 0
 * @max 0.9
 * @decimal 2
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_MAN_DZ, 0.1);

/**
 * Geometric Controller Manual Mode Max Body Force
 *
 * Maximum body-frame force magnitude per axis commanded by the sticks in
 * Manual (force open-loop) mode. aux1/aux2 command body x/y force, throttle
 * commands body z force (center = hover trim -mass*g, up = climb).
 * Keep below the available thrust margin (N_rotors * CA_MQ_TMAX - mass*g).
 *
 * @unit N
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_MAN_FMAX, 20.0);

/**
 * Geometric Controller Rate/Angle Integral Gain
 *
 * Integral gain for the attitude/rate error integrator (torque loop).
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_RATE_I, 0.10);

/**
 * Geometric Controller Attitude Error Integral Gain
 *
 * Integral gain for cancelling constant roll/pitch/yaw torque bias. This
 * integrates the SO(3) attitude error, so a zero-rate but non-zero attitude
 * error is corrected instead of becoming a steady-state offset.
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_ATT_I, 0.15);

/**
 * Geometric Controller Rate Integrator Limit
 *
 * Maximum absolute value of the rate/torque integrator.
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_RATE_INT_LIM, 0.3);

/**
 * Geometric Controller Velocity Integral Gain
 *
 * Integral gain for the velocity error integrator (force loop).
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_VEL_I, 0.1);

/**
 * Geometric Controller Velocity Integrator Limit
 *
 * Maximum absolute value of the velocity/force integrator.
 *
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_VEL_INT_LIM, 2.0);

/**
 * Geometric Controller Total Maximum Thrust
 *
 * Sum of the maximum thrust of every rotor. Used to normalize the desired
 * force into a [0,1] collective-throttle estimate for the debug setpoints.
 * Should equal N_rotors * CA_MQ_TMAX (4 * 39.5 = 158 N for the calibrated
 * JSBSim morphquad).
 *
 * @unit N
 * @group Geometric Controller
 */
PARAM_DEFINE_FLOAT(GC_THRUST_MAX, 158.0);
