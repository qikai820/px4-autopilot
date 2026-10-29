/**
 * MorphQuad independent roll/pitch on AUX1/AUX2 in manual position/altitude modes
 *
 * Standard roll/pitch stick trajectory generation remains in FlightModeManager.
 * Has no effect in autonomous or Offboard modes, or on Voliro T.
 * @boolean
 * @group Vector Position Control
 */
PARAM_DEFINE_INT32(VEC_AUX_ATT, 0);
