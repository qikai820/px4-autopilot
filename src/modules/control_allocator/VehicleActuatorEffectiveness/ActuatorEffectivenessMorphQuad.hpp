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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
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

	bool getEffectivenessMatrix(Configuration &configuration,
				    EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "MorphQuad"; }

private:
	static constexpr int NUM_ROTORS = 4;
	static constexpr int NR = 2;

	// 模块可调参数（通过 CA_MQ_* 参数在地面站/参数文件中配置）。
	// 这些参数定义了 MorphQuad 的物理几何与分配算法的权重，是推力分配的核心配置。
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MQ_L>)     _param_ca_mq_l,      ///< 机臂半长 l [m]：机身中心到每个旋翼在 X 方向的臂长
		(ParamFloat<px4::params::CA_MQ_B>)     _param_ca_mq_b,      ///< 机臂半宽 b [m]：机身中心到每个旋翼在 Y 方向的臂宽
		(ParamFloat<px4::params::CA_MQ_CT>)    _param_ca_mq_ct,     ///< 推力系数 c_T [N/(rad/s)^2]：满足转子模型 T_i = c_T * Ω_i^2
		(ParamFloat<px4::params::CA_MQ_OMAX>)  _param_ca_mq_omax,   ///< 电机最大转速 Ω_max [rad/s]：esc 指令为归一化转速 Ω/Ω_max
		(ParamFloat<px4::params::CA_MQ_ALPM>)  _param_ca_mq_alpm,   ///< 外环舵机（alpha）机械限位角 [deg]：绕偏航轴的最大偏转角
		(ParamFloat<px4::params::CA_MQ_BETM>)  _param_ca_mq_betm,   ///< 内环舵机（beta）机械限位角 [deg]：绕俯仰轴的最大偏转角
		(ParamFloat<px4::params::CA_MQ_DWG>)   _param_ca_mq_dwg,    ///< 尾流（下洗）代价权重：零空间优化中用于减小相邻旋翼间的下洗干扰
		(ParamFloat<px4::params::CA_MQ_TMAX>)  _param_ca_mq_tmax,   ///< 满油门时单旋翼最大推力 TMAX [N]：满足 TMAX = c_T * Ω_max^2
		(ParamFloat<px4::params::CA_MQ_NS_GAIN>) _param_ca_mq_ns_gain, ///< 零空间下降增益 k：控制零空间冗余自由度优化的步长
		(ParamFloat<px4::params::CA_MQ_NS_POW>)  _param_ca_mq_ns_pow,  ///< 零空间电机功率代价权重：优化目标 J_pow = w_pow * Σ u_i^2
		(ParamFloat<px4::params::CA_MQ_ANGLE_GAIN>) _param_ca_mq_angle_gain ///< 云台角重定向步长系数：缩放每次迭代中舵机角度的修正量
	)

	// 上一周期的舵机角度，用于"最小运动"准则选择：
	// 在多解情况下，优先选择与上一次角度最接近的解，避免舵机在等价构型之间跳变抖动。
	float _alpha_prev[NUM_ROTORS] {};
	float _beta_prev[NUM_ROTORS] {};

	// 饱和状态跟踪标志，用于向位置/姿态控制器（geo_control）做抗积分饱和（anti-windup）反馈。
	bool _motor_saturated{false};     ///< 是否存在任意电机达到转速上限 Ω_max（即油门被钳位）
	bool _servo_saturated{false};     ///< 是否存在任意舵机达到角度机械限位（被钳位）
	float _unallocated_torque[3] {};  ///< 未能实现的残余力矩 [N·m]：期望力矩与实际输出力矩之差
	float _unallocated_thrust[3] {};  ///< 未能实现的残余推力 [N]：期望推力与实际输出推力之差

	/**
	 * 由期望广义力（wrench）求解 4 个旋翼的推力矢量（对应论文公式 Eq.12 + Eq.15-16）。
	 *
	 * 本函数是 MorphQuad 全驱动分配的核心：它先在当前云台角下用最小范数解算出推力大小，
	 * 再通过高斯-牛顿（含 Levenberg–Marquardt 阻尼）迭代重定向云台角，使实现的 wrench 跟踪期望值，
	 * 最后在零空间（冗余自由度）内做二次优化以抑制电机功率与相邻旋翼下洗干扰。
	 *
	 * @param f_des  期望力，机体坐标系 [N]，分量顺序为 (x, y, z)
	 * @param tau_des 期望力矩，机体坐标系 [N·m]，分量顺序为 (滚转, 俯仰, 偏航)
	 * @param t_out  输出参数，4×3 的推力矢量数组，每一行对应一个旋翼的三轴推力分量
	 */
	void computeThrustVectors(const matrix::Vector3f &f_des, const matrix::Vector3f &tau_des,
				  float t_out[NUM_ROTORS][3]);

	/**
	 * 由单个旋翼的推力矢量反解双轴云台舵机角度（对应论文公式 Eq.14）。
	 *
	 * 注意：本函数不再计算电机转速；转速由调用方把推力大小按转子模型归一化为
	 * JSBSim morphquad 模型期望的 [0,1] 油门比例（esc 指令 = Ω/Ω_max）。
	 * 反解存在两支等价解（beta 处于不同区间），并会结合上一次角度做"最小运动"选择，
	 * 再处理多圈环绕后钳制到机械限位。
	 *
	 * @param t         单个旋翼的 3 维推力矢量 (Tx, Ty, Tz)
	 * @param idx       旋翼索引（0-3），用于读取/更新该旋翼的历史角度
	 * @param alpha_out 输出：外环舵机角度（绕偏航轴）[rad]
	 * @param beta_out  输出：内环舵机角度（绕俯仰轴）[rad]
	 */
	void recoverServoAngles(const float t[3], int idx,
				float &alpha_out, float &beta_out);

	/**
	 * 在作动器饱和时计算未能实现的广义力（unallocated wrench）。
	 *
	 * 通过比较"期望 wrench"与"由钳位后的实际作动器输出重建得到的 wrench"，
	 * 求出残余的力/力矩，供 getUnallocatedControl() 上报给控制器，
	 * 用于抗饱和反馈（anti-windup）与故障诊断。
	 *
	 * @param f_des       期望力 [N]
	 * @param tau_des     期望力矩 [N·m]
	 * @param omega_raw   饱和前归一化转速（未钳位），长度 NUM_ROTORS
	 * @param alpha_raw   饱和前外环角（未钳位，已归一化为 1/π），长度 NUM_ROTORS
	 * @param beta_raw    饱和前内环角（未钳位，已归一化为 1/π），长度 NUM_ROTORS
	 * @param actuator_sp 经钳位后的实际作动器输出（包含电机与舵机指令）
	 * @param actuator_min 作动器下界
	 * @param actuator_max 作动器上界
	 */
	void computeUnallocatedWrench(const matrix::Vector3f &f_des, const matrix::Vector3f &tau_des,
				      const float omega_raw[NUM_ROTORS], const float alpha_raw[NUM_ROTORS],
				      const float beta_raw[NUM_ROTORS],
				      const ActuatorVector &actuator_sp,
				      const ActuatorVector &actuator_min,
				      const ActuatorVector &actuator_max);
};
