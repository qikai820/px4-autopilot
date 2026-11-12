/**
 * Vector multicopter air-mode
 *
 * The air-mode enables the mixer to increase the total thrust of the multirotor
 * in order to keep attitude and rate control even at low and high throttle.
 *
 * For CA_AIRFRAME=17, applies in armed manual direct-throttle control
 * (Stabilized/Acro), in both VT_CTRL_MODE settings. Roll/Pitch and optional
 * yaw headroom are subordinate to the pilot force limit. Ground/unknown land
 * state allows no added thrust. In flight, assistance is bounded to 0.02 total
 * normalized thrust and 25% of the pilot request. Infeasible torque remains a
 * residual; no lift floor is retained. Manual idle clamps a zero-anchored
 * throttle curve instead of remapping its lower half after takeoff.
 * Below MPC_THR_HOVER, pitch integral accumulation scales with collective/hover
 * (minimum 0.25); integral state/limits remain. Position, altitude and automatic
 * control retain their normal allocation. MC_AIRMODE is separate.
 *
 * This function should be disabled during tuning as it will help the controller
 * to diverge if the closed-loop is unstable (i.e. the vehicle is not tuned yet).
 *
 * Enabling air-mode for yaw requires the use of an arming switch.
 *
 * @value 0 Disabled
 * @value 1 Roll/Pitch
 * @value 2 Roll/Pitch/Yaw
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(VC_AIRMODE, 0);
