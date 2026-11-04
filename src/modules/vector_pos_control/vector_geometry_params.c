/**
 * Independent attitude input in position control modes
 *
 * MorphQuad uses AUX1/AUX2 for roll/pitch in manual position/altitude modes.
 * Voliro T uses RC AUX1 for body pitch in Position, Position Slow and Hold
 * when RC_MAP_VECTOR_SW is unassigned.
 * RC_MAP_AUX1 must select a calibrated RC channel for Voliro T.
 * Standard roll/pitch stick trajectory generation remains in FlightModeManager.
 * @boolean
 * @group Vector Position Control
 */
PARAM_DEFINE_INT32(VEC_AUX_ATT, 0);

/**
 * Voliro T AUX1 body pitch angle limit
 *
 * Used only when VEC_AUX_ATT is enabled. Independent pitch rotates within
 * the banked force plane; the position-control tilt limit bounds bank
 * separately. Large angles require sufficient tilt travel and torque authority.
 *
 * @unit deg
 * @min 0.0
 * @max 160.0
 * @decimal 1
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_MAX, 10.0f);

/**
 * Voliro T body pitch setpoint slew rate
 *
 * @unit deg/s
 * @min 1.0
 * @max 90.0
 * @decimal 1
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_RATE, 20.0f);

/**
 * RC channel for vector attitude mode
 *
 * For Voliro T, 0 retains AUX1 independent pitch. When assigned, above center
 * selects force-derived pitch in Position, Position Slow and Hold; at or below
 * center commands level pitch. RC loss also commands level pitch. MorphQuad
 * retains its independent attitude enable switch behavior.
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 */
PARAM_DEFINE_INT32(RC_MAP_VECTOR_SW, 0);

/**
 * RC channel for independent vector attitude leveling
 *
 * A calibrated channel above its center commands zero independent roll and
 * pitch, overriding RC_MAP_VECTOR_SW. Voliro T roll remains controlled by
 * lateral position demand. 0 disables the switch.
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 */
PARAM_DEFINE_INT32(RC_MAP_LEVEL_SW, 0);
