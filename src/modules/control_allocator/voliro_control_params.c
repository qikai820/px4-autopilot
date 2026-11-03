/**
 * Voliro T dynamics control mode
 *
 * Requires calibrated VEC_IXX/IYY/IZZ and VEC_TQ_R/P/Y. Applied at module
 * startup only. Reboot after changes so tasks, controllers and allocation agree.
 * Legacy VC_* parameters retain their meaning in mode 0.
 *
 * @value 0 Legacy quaternion and normalized-torque rate PID
 * @value 1 Task-frame attitude and inertia-normalized rate control
 * @reboot_required true
 * @group Voliro Dynamics
 */
PARAM_DEFINE_INT32(VT_CTRL_MODE, 0);

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
 * Task-frame bank attitude gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 10
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_BANK_P, 2.0f);

/**
 * Task-frame pitch attitude gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 10
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_PITCH_P, 2.0f);

/**
 * Task-frame heading attitude gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 10
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_HEAD_P, 1.0f);

/**
 * Body X rate feedback to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 40
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_PX, 8.0f);

/**
 * Body X rate integral to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * Gain unit: 1/s^2 (inverse squared seconds).
 * @min 0
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_IX, 1.0f);

/**
 * Body X angular acceleration feedback gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @min 0
 * @max 3
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_DX, 0.3f);

/**
 * Body Y rate feedback to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 40
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_PY, 12.0f);

/**
 * Body Y rate integral to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * Gain unit: 1/s^2 (inverse squared seconds).
 * @min 0
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_IY, 0.3f);

/**
 * Body Y angular acceleration feedback gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @min 0
 * @max 3
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_DY, 0.3f);

/**
 * Body Z rate feedback to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0.1
 * @max 40
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_PZ, 8.0f);

/**
 * Body Z rate integral to angular acceleration
 *
 * Used by VT_CTRL_MODE=1 only.
 * Gain unit: 1/s^2 (inverse squared seconds).
 * @min 0
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_IZ, 1.0f);

/**
 * Body Z angular acceleration feedback gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @min 0
 * @max 3
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_DZ, 0.3f);

/**
 * Angular acceleration integral limit
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit rad/s^2
 * @min 0
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
 */
PARAM_DEFINE_FLOAT(VT_RATE_ILIM, 2.0f);

/**
 * Allocation residual back-calculation gain
 *
 * Used by VT_CTRL_MODE=1 only.
 * @unit 1/s
 * @min 0
 * @max 20
 * @decimal 3
 * @group Voliro Dynamics
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
