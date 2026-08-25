/**
 * Forward MorphQuad bench-test actuator outputs while disarmed
 *
 * Enables the dedicated actuator path used by geo_control verify. It is
 * intended exclusively for a JSBSim held-down thrust stand and does not arm
 * the vehicle or affect normal flight outputs.
 *
 * @boolean
 * @group Simulator
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SIM_MQ_BENCH, 0);
