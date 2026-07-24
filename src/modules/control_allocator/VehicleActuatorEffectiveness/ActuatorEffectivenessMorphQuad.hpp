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

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

class ActuatorEffectivenessMorphQuad : public ModuleParams, public ActuatorEffectiveness
{
public:
    ActuatorEffectivenessMorphQuad(ModuleParams *parent);
    virtual ~ActuatorEffectivenessMorphQuad() = default;

    bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;
    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

    const char *name() const override { return "MorphQuad"; }

private:
    // 论文中的物理参数
    float _mass{1.5f};           ///< 飞行器质量 (kg)
    float _arm_length{0.25f};    ///< 机臂长度 (m)
    float _motor_to_servo_arm{0.05f}; ///< 电机到舵机摇臂的距离 (m)

    // 论文中的控制增益
    float _k_p{1.0f};            ///< 位置环比例增益
    float _k_d{0.5f};            ///< 位置环微分增益
    float _k_rp{5.0f};           ///< 姿态环比例增益
    float _k_rd{1.0f};           ///< 姿态环微分增益

    // 中间计算变量
    matrix::Vector3f _desired_force_b; ///< 机体坐标系下的期望合力
    float _total_thrust;               ///< 四个电机的总推力
    float _delta_tilt;                 ///< 倾转舵机的偏转角度

    // 辅助函数：根据论文公式计算期望的机体合力和倾转角度
    void _computeForceAndTilt(const matrix::Vector<float, NUM_AXES> &control_sp);
};
