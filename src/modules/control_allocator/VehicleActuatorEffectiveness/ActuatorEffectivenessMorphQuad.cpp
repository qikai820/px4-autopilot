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

#include "ActuatorEffectivenessMorphQuad.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessMorphQuad::ActuatorEffectivenessMorphQuad(ModuleParams *parent) :
    ModuleParams(parent)
{
    // 从参数服务器加载参数（此处为示例，实际应使用 DEFINE_PARAMETERS 宏）
    // 例如: _mass = param_find("MQ_MASS");
}

bool ActuatorEffectivenessMorphQuad::getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update)
{
    if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
        return false;
    }

    // 1. 添加4个电机
    // 电机只产生推力，其反扭矩效应由控制分配算法处理
    for (int i = 0; i < 4; ++i) {
        configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
    }

    // 2. 添加4个舵机 (用于倾转)
    // 舵机产生力矩，其具体效应由算法计算
    for (int i = 0; i < 4; ++i) {
        configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
    }

    return true;
}

void ActuatorEffectivenessMorphQuad::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
    // 1. 根据高层控制器输出的期望力/力矩，计算总推力和倾转角度
    // control_sp 包含了 [Roll Torque, Pitch Torque, Yaw Torque, Thrust X, Thrust Y, Thrust Z]
    _computeForceAndTilt(control_sp);

    // 2. 将总推力和倾转角度分配给4个电机和4个舵机
    // 这是一个简化的分配模型，具体取决于 MorphQuad 的机械结构
    // 假设是 X 型布局，电机编号为 0, 1, 2, 3

    // 每个电机的基础推力是总推力的一半（假设两两一组）
    float motor_thrust_base = _total_thrust / 4.0f;

    // 根据期望的滚转和俯仰力矩，对电机推力进行微调
    // 这里简化处理，实际应根据论文中的力矩平衡方程精确计算
    float roll_torque = control_sp(ControlAxis::ROLL);
    float pitch_torque = control_sp(ControlAxis::PITCH);

    // 电机推力分配 (示例)
    actuator_sp(0) = motor_thrust_base + roll_torque * _k_rp + pitch_torque * _k_rp; // 前右
    actuator_sp(1) = motor_thrust_base - roll_torque * _k_rp + pitch_torque * _k_rp; // 前左
    actuator_sp(2) = motor_thrust_base - roll_torque * _k_rp - pitch_torque * _k_rp; // 后左
    actuator_sp(3) = motor_thrust_base + roll_torque * _k_rp - pitch_torque * _k_rp; // 后右

    // 3. 舵机角度分配
    // 舵机角度由倾转角度 _delta_tilt 和偏航力矩决定
    float yaw_torque = control_sp(ControlAxis::YAW);

    // 舵机角度分配 (示例)
    // 倾转舵机用于产生滚转和俯仰力矩
    actuator_sp(4) = _delta_tilt + yaw_torque * _k_rp; // 前右舵机
    actuator_sp(5) = _delta_tilt - yaw_torque * _k_rp; // 前左舵机
    actuator_sp(6) = _delta_tilt + yaw_torque * _k_rp; // 后左舵机
    actuator_sp(7) = _delta_tilt - yaw_torque * _k_rp; // 后右舵机

    // 4. 饱和处理
    for (int i = 0; i < 8; ++i) {
        if (actuator_sp(i) > actuator_max(i)) {
            actuator_sp(i) = actuator_max(i);
        } else if (actuator_sp(i) < actuator_min(i)) {
            actuator_sp(i) = actuator_min(i);
        }
    }
}

void ActuatorEffectivenessMorphQuad::_computeForceAndTilt(const matrix::Vector<float, NUM_AXES> &control_sp)
{
    // 从控制设定点中提取期望的力和力矩
    float desired_torque_roll = control_sp(ControlAxis::ROLL);
    float desired_torque_pitch = control_sp(ControlAxis::PITCH);
    float desired_thrust_z = control_sp(ControlAxis::THRUST_Z); // 期望的Z轴推力

    // 论文中的核心算法：计算总推力和倾转角度
    // 这是一个简化的示例，实际公式会更复杂

    // 1. 计算机体坐标系下的期望合力
    // 假设期望的合力主要在Z轴，X和Y轴由倾转产生
    _desired_force_b(0) = 0.0f; // 简化处理
    _desired_force_b(1) = 0.0f; // 简化处理
    _desired_force_b(2) = -desired_thrust_z; // 期望的升力

    // 2. 计算总推力
    // 总推力需要克服重力并提供期望的加速度
    _total_thrust = _desired_force_b.norm();

    // 3. 计算倾转角度
    // 倾转角度由期望的滚转和俯仰力矩决定
    // 这是一个简化的计算，实际应根据论文中的公式
    _delta_tilt = atan2f(sqrtf(desired_torque_roll * desired_torque_roll + desired_torque_pitch * desired_torque_pitch), _total_thrust * _arm_length);

    // 限制倾转角度在合理范围内
    _delta_tilt = math::constrain(_delta_tilt, -0.5f, 0.5f); // 限制在 +/- 30度左右
}
