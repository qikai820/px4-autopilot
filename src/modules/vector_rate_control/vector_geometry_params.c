/**
 * Enable rigid-body gyroscopic torque feedforward
 *
 * Requires calibrated body inertia and normalized torque reference scales.
 * Applies to both control modes. Disabled by default; the normalized PID
 * feedback and its integral limits do not depend on this model.
 * @boolean
 * @group Vector Rate Control
 */
PARAM_DEFINE_INT32(VEC_GYRO_FF, 0);
/**
 * Body inertia IXX
 *
 * Used only by mode-1 reference acceleration feedforward, optional
 * VEC_GYRO_FF and tilt monitoring. Body-fixed; never scales PID feedback.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IXX, 0.1f);
/**
 * Body inertia IYY
 *
 * Used only by mode-1 reference acceleration feedforward, optional
 * VEC_GYRO_FF and tilt monitoring. Body-fixed; never scales PID feedback.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IYY, 0.1f);
/**
 * Body inertia IZZ
 *
 * Used only by mode-1 reference acceleration feedforward, optional
 * VEC_GYRO_FF and tilt monitoring. Body-fixed; never scales PID feedback.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IZZ, 0.2f);
