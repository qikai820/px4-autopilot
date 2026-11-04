/**
 * Independent attitude input in position control modes
 *
 * MorphQuad uses AUX1/AUX2 for roll/pitch in manual position/altitude modes.
 * Voliro T uses RC AUX1 for body pitch in Position, Position Slow and Hold.
 * RC_MAP_VECTOR_SW optionally gates AUX1; switch off commands level pitch.
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
 * Voliro T tilt monitoring and automatic multicopter fallback
 *
 * Observe common tilt from specific force and pitch dynamics in airborne
 * position/velocity control. A persistent mismatch switches to conventional
 * multicopter force-to-attitude control and commands the tilt servo to center.
 * The remaining estimated tilt is compensated. Fallback stays latched until
 * disarm, even if this parameter is disabled in flight. Requires calibrated
 * CA_CT geometry, VEC inertia and MPC_THR_HOVER. External contact is unsupported.
 * @boolean
 * @group Vector Position Control
 */
PARAM_DEFINE_INT32(VT_TILT_EN, 0);

/**
 * Voliro T observed tilt error threshold
 *
 * Absolute difference between observed angle and lag/rate-predicted angle.
 * The same observation filter is applied to both angles. Used by VT_TILT_EN.
 * @unit deg
 * @min 1
 * @max 90
 * @decimal 1
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_ERR, 10.0f);

/**
 * Voliro T tilt error confirmation time
 *
 * Continuous valid observation time above VT_TILT_ERR before fallback.
 * Invalid observations reset the timer; fallback itself clears only on disarm.
 * @unit s
 * @min 0.1
 * @max 10
 * @decimal 2
 * @group Vector Position Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_T, 0.5f);

/**
 * RC channel for independent vector attitude enable
 *
 * For Voliro T with VEC_AUX_ATT enabled, above center permits AUX1 independent
 * body pitch in Position, Position Slow and Hold; at or below center commands
 * level pitch. RC_MAP_LEVEL_SW has priority and can always command level pitch.
 * 0 leaves AUX1 enabled without this switch. Invalid/stale RC commands level
 * pitch. MorphQuad retains its independent attitude enable switch behavior.
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
