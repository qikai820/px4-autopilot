/**
 * Enable rigid-body gyroscopic torque feedforward
 *
 * Requires calibrated body inertia and normalized torque reference scales.
 * Applies to the legacy rate PID. VT_CTRL_MODE=1 always includes this
 * rigid-body term as part of its angular-acceleration control law.
 * @boolean
 * @group Vector Rate Control
 */
PARAM_DEFINE_INT32(VEC_GYRO_FF, 0);
/**
 * Body inertia IXX
 *
 * Required by VT_CTRL_MODE=1 for torque computation; it remains body-fixed
 * while the vehicle pitches. Also used by legacy VEC_GYRO_FF.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IXX, 0.1f);
/**
 * Body inertia IYY
 *
 * Required by VT_CTRL_MODE=1 for torque computation; it remains body-fixed
 * while the vehicle pitches. Also used by legacy VEC_GYRO_FF.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IYY, 0.1f);
/**
 * Body inertia IZZ
 *
 * Required by VT_CTRL_MODE=1 for torque computation; it remains body-fixed
 * while the vehicle pitches. Also used by legacy VEC_GYRO_FF.
 * @unit kg m^2
 * @min 0.001
 * @group Vector Rate Control
 */
PARAM_DEFINE_FLOAT(VEC_IZZ, 0.2f);
