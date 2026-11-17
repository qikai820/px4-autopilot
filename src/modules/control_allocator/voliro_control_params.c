/**
 * Voliro T large-angle reference control mode
 *
 * Both modes use VC_* normalized-torque PID with the same integral limits.
 * Mode 1 maps attitude gains to task directions, shapes reaction-axis rate P
 * with VT_RCT_P_SCALE and adds fresh angular references. Inertia is used for acceleration
 * feedforward and optional VEC_GYRO_FF, not to scale PID feedback. Applied at
 * startup only; reboot so tasks, controllers and allocation agree.
 *
 * @value 0 Legacy quaternion and normalized-torque rate PID
 * @value 1 Task-frame attitude, reaction-axis P shaping and reference feedforward
 * @reboot_required true
 * @group Voliro Dynamics
 */
PARAM_DEFINE_INT32(VT_CTRL_MODE, 0);

/**
 * Voliro reaction-axis proportional feedback scale
 *
 * In VT_CTRL_MODE=1, reduce only the rate PID proportional torque along the
 * common rotor axis. Smoothly blend from 1 at 10 deg tilt to this scale at
 * 30 deg, retaining this scale through inverted tilts. Upright vertical
 * thrust retains original gains. Integral, derivative and feedforward
 * torques, and the original control allocation, are unchanged.
 * 1 disables the shaping. Applied only to CA_AIRFRAME=17.
 * @min 0.1
 * @max 1
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RCT_P_SCALE, 0.5f);

/**
 * Near-vertical feedback bandwidth scale
 *
 * Mode 1 retains VC_* attitude strength through 45 deg independent pitch.
 * Smoothly reduces outer attitude gains to this fraction at 75..105 deg,
 * where tail reversal around zero trim adds delay. Pitch integral accumulation
 * is slowed by the same fraction; its state and per-axis limits are NOT scaled.
 * Base VC_* rate gains remain unchanged. Independent-pitch reference speed
 * is limited by the same schedule; its consistent derivatives feed forward.
 * @min 0.1
 * @max 1
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_ATT_SCALE, 0.333333f);

/**
 * Voliro T maximum reverse tail thrust
 *
 * Zero preserves CA_CT_T5MAX in both directions. Set to the calibrated
 * reverse maximum for an asymmetric reversible motor/propeller.
 * The force normalization remains 4*CA_CT_TMAX + CA_CT_T5MAX.
 * @unit N
 * @min 0
 * @max 100
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_CT_T5RMAX, 0.0f);

/**
 * Position integral allocation shortfall back-calculation gain
 *
 * Used by VT_CTRL_MODE=1 only. Converts normalized NED force residual
 * to acceleration before unwinding the velocity-controller integrator.
 * @unit 1/s
 * @min 0
 * @max 10
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_POS_AW, 1.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_BANK_P, 2.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_PITCH_P, 2.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_HEAD_P, 1.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_PX, 8.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_IX, 1.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_DX, 0.3f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_PY, 12.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_IY, 0.3f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_DY, 0.3f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_PZ, 8.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_IZ, 1.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_DZ, 0.3f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_ILIM, 2.0f);

/**
 * Unused legacy Voliro feedback setting
 *
 * Retained for parameter-file compatibility. Both modes now use VC_* attitude
 * and rate PID gains, VC_RR/PR/YR_INT_LIM, and saturation-direction gating.
 * This parameter has no control effect.
 * @group Voliro Legacy
 */
PARAM_DEFINE_FLOAT(VT_RATE_AW, 2.0f);

/**
 * Rotor reaction torque low-pass cutoff (0 disables)
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_REACT_FILT, 2.0f);

/**
 * Attitude reference model natural frequency
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit rad/s
 * @min 1
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_REF_FREQ, 8.0f);

/**
 * Force-plane bank reference rate limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit deg/s
 * @min 1
 * @max 180
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_BANK_RATE, 80.0f);

/**
 * Force-plane bank reference acceleration limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit deg/s^2
 * @min 1
 * @max 1000
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_BANK_ACC, 360.0f);

/**
 * Independent pitch reference acceleration limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit deg/s^2
 * @min 1
 * @max 360
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_PITCH_ACC, 60.0f);

/**
 * Heading reference rate limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit deg/s
 * @min 1
 * @max 180
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_HEAD_RATE, 45.0f);

/**
 * Heading reference acceleration limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit deg/s^2
 * @min 1
 * @max 720
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_HEAD_ACC, 90.0f);

/**
 * Predicted motor command lag (0 disables motor lag prediction)
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit s
 * @min 0
 * @max 1
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_MOT_TAU, 0.0f);

/**
 * Predicted common-tilt lag (0 disables servo lag prediction)
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit s
 * @min 0
 * @max 1
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_SRV_TAU, 0.0f);
