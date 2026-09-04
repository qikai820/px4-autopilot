/**
 * Enable Dock Guidance startup
 *
 * @boolean
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_GUIDANCE, 1);

/**
 * Camera delay compensation [ms]
 *
 * @min 0
 * @max 5000
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
 * Maximum accepted camera LOS angle [deg]
 *
 * @min 10
 * @max 89.9
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_LOS_MAX, 89.0f);

/**
 * Maximum approach speed [m/s]
 *
 * @min 0
 * @max 100
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_VEL_MAX, 15.0f);

/**
 * Maximum approach acceleration [m/s^2]
 *
 * Limits how quickly the commanded LOS speed rises toward DOCK_VEL_MAX.
 *
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_ACC_MAX, 3.0f);

/**
 * Maximum approach deceleration [m/s^2]
 *
 * Used by the stopping-distance speed envelope and recovery state.
 *
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_DEC_MAX, 3.0f);

/**
 * Final stand-off distance [m]
 *
 * The velocity envelope reaches zero at this target slant range.
 *
 * @min 0
 * @max 100
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_STOP_DIST, 2.0f);

/**
 * Required target alignment time [s]
 *
 * The target must remain inside DOCK_FOV_ALIGN for this duration before
 * high-speed approach can start.
 *
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_ALIGN_T, 0.30f);

/**
 * Alignment region as fraction of camera half-FOV
 *
 * 1.0 is the image edge. Both image axes must remain inside this ratio before
 * approach can start.
 *
 * @min 0.05
 * @max 0.8
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_ALIGN, 0.30f);

/**
 * Soft FOV boundary as fraction of camera half-FOV
 *
 * Crossing this boundary leaves high-speed approach and starts recovery.
 *
 * @min 0.1
 * @max 0.95
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_SOFT, 0.65f);

/**
 * Hard FOV boundary as fraction of camera half-FOV
 *
 * Crossing this boundary invalidates Dock Guidance so Commander can execute
 * the configured mode-loss action before the target leaves the image.
 *
 * @min 0.2
 * @max 0.99
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_HARD, 0.85f);

/**
 * Maximum alignment translation speed [m/s]
 *
 * While the target is not centered, Dock Guidance translates in the plane
 * perpendicular to the installed camera optical axis. This removes image
 * error without starting the high-speed approach.
 *
 * @min 0.1
 * @max 5
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_ALIGN_SPD, 1.0f);

/**
 * Visual-loss grace period [ms]
 *
 * During this interval body-rate commands decay to zero and throttle can only
 * move toward a lower conservative value. The mode becomes unhealthy after
 * the grace period expires.
 *
 * @min 20
 * @max 2000
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_VIS_GRACE, 300);

/**
 * Relative-position timeout [ms]
 *
 * High-speed approach requires a fresh finite target-relative NED vector so
 * the stopping-distance envelope always has a valid range.
 *
 * @min 20
 * @max 2000
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_REL_TOUT, 200);
