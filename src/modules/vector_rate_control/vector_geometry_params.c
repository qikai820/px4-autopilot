/**
 * Enable rigid-body gyroscopic torque feedforward
 *
 * Requires calibrated body inertia and normalized torque reference scales.
 * @boolean
 * @group Vector Rate Control
 */
PARAM_DEFINE_INT32(VEC_GYRO_FF, 0);
/**
 * Body inertia IXX
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IXX, 0.1f);
/**
 * Body inertia IYY
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IYY, 0.1f);
/**
 * Body inertia IZZ
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IZZ, 0.2f);
