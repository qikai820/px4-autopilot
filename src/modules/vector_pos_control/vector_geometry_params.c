/**
 * Independent attitude input in position control modes
 *
 * MorphQuad uses AUX1/AUX2 for roll/pitch in manual position/altitude modes.
 * Voliro T uses RC AUX1 for body pitch in Position, Position Slow and Hold.
 * RC_MAP_AUX1 must select a calibrated RC channel for Voliro T.
 * Standard roll/pitch stick trajectory generation remains in FlightModeManager.
 * @boolean
 * @group Vector Position Control
 */
PARAM_DEFINE_INT32(VEC_AUX_ATT, 0);

/**
 * Voliro T AUX1 body pitch angle limit
 *
 * Used only when VEC_AUX_ATT is enabled. The active position-control tilt
 * limit can further reduce the requested pitch angle.
 *
 * @unit deg
 * @min 0.0
 * @max 45.0
 * @decimal 1
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_MAX, 10.0f);

/**
 * Voliro T AUX1 body pitch setpoint slew rate
 *
 * @unit deg/s
 * @min 1.0
 * @max 90.0
 * @decimal 1
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_RATE, 20.0f);
