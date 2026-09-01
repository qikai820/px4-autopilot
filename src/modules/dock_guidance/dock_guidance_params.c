/**
 * Camera delay compensation [ms]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_CAM_DLY, 0);

/**
 * Visual observation timeout [ms]
 *
 * The guidance setpoint becomes invalid when no fresh visual frame is received
 * within this interval.
 *
 * @min 20
 * @max 5000
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_OBS_TOUT, 150);

/**
 * Camera-to-body roll rotation [deg]
 *
 * @min -180
 * @max 180
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_ROLL, -90.0f);

/**
 * Camera-to-body pitch rotation [deg]
 *
 * @min -180
 * @max 180
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_PITCH, 0.0f);

/**
 * Camera-to-body yaw rotation [deg]
 *
 * @min -180
 * @max 180
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_YAW, 0.0f);

/**
 * LOS vector filter cutoff frequency [Hz]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FILT_HZ, 2.0f);

/**
 * Camera image width
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_RES_X, 1920.0f);

/**
 * Camera image height
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_RES_Y, 1080.0f);

/**
 * Camera horizontal field of view [deg]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_X, 60.0f);

/**
 * Camera vertical field of view [deg]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_Y, 45.0f);

/**
 * Yaw angle proportional gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_YAW_P, 1.0f);

/**
 * Pitch angle proportional gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_PIT_P, 1.0f);

/**
 * Maximum angular rate command [deg/s]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_RATE_MAX, 60.0f);

/**
 * Maximum angular-rate command slew [deg/s^2]
 *
 * @min 1
 * @max 2000
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_RATE_SLEW, 300.0f);

/**
 * Visibility limit angle [deg]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_VIS_LIM, 20.0f);

/**
 * Visibility-stage hysteresis [deg]
 *
 * @min 0
 * @max 20
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_VIS_HYST, 3.0f);

/**
 * Pitch-capture entry angle [deg]
 *
 * @min 10
 * @max 89
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_PCAP_ENT, 60.0f);

/**
 * Pitch-capture exit angle [deg]
 *
 * @min 5
 * @max 89
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_PCAP_EXT, 50.0f);

/**
 * Maximum accepted camera LOS angle [deg]
 *
 * @min 10
 * @max 89.9
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_LOS_MAX, 89.0f);

/**
 * LOS yaw rate P gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_NAV_Y_P, 3.0f);

/**
 * LOS pitch rate P gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_NAV_P_P, 3.0f);

/**
 * Forward pitch angle target [deg]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FWD_PIT, 30.0f);

/**
 * Track yaw proportional gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TRC_Y, 1.0f);

/**
 * Track pitch proportional gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TRC_P, 1.0f);

/**
 * Dock throttle [0, 1]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_THR, 0.5f);

/**
 * Dock throttle rate limit [1/s]
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_THR_RATE, 0.1f);

/**
 * Camera roll level gain
 *
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_RLL_P, 1.0f);
