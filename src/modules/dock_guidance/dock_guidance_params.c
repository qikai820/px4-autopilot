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
 * Enable variable camera pitch servo
 *
 * When enabled, Dock Guidance slowly changes the camera's body-relative pitch
 * angle to remove sustained vertical image error. The servo is deliberately
 * not commanded from vehicle attitude or angular rate, so it is not used as a
 * stabilization gimbal. Assign an unused output to Gimbal Pitch (function 421).
 *
 * @boolean
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_CAM_SERVO, 0);

/**
 * Camera pitch servo minimum angle [deg]
 *
 * This body-relative angle maps to normalized Gimbal Pitch output -1. Reverse
 * the assigned PWM output when the physical servo direction is opposite.
 *
 * @min -180
 * @max 180
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_P_MIN, 0.0f);

/**
 * Camera pitch servo maximum angle [deg]
 *
 * This body-relative angle maps to normalized Gimbal Pitch output +1.
 *
 * @min -180
 * @max 180
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_P_MAX, 90.0f);

/**
 * Camera pitch servo maximum slew rate [deg/s]
 *
 * The low rate prevents the position servo from being used for attitude
 * stabilization or from following high-frequency airframe motion.
 *
 * @min 0.1
 * @max 30
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_P_RATE, 5.0f);

/**
 * Camera pitch image-centering time constant [s]
 *
 * Vertical image error is converted to a slow body-relative servo rate by
 * dividing the error by this value. Larger values move the camera more slowly.
 *
 * @min 0.2
 * @max 20
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_P_TC, 2.0f);

/**
 * Camera pitch servo image deadband [deg]
 *
 * Image errors inside this band do not move the servo.
 *
 * @min 0
 * @max 10
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_CAM_P_DB, 0.5f);

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
PARAM_DEFINE_FLOAT(DOCK_ALIGN_T, 1.0f);

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
 * Crossing this boundary starts emergency recovery while the observation is
 * still valid. Dock Guidance only becomes unhealthy after actual visual loss.
 *
 * @min 0.2
 * @max 0.99
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_HARD, 0.85f);

/**
 * Hard FOV confirmation time [s]
 *
 * The target must remain beyond DOCK_FOV_HARD for this duration before
 * emergency recovery is latched. This rejects single-frame image noise.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_FOV_H_T, 0.15f);

/**
 * Recovery alignment hold time [s]
 *
 * Recovery may return to ALIGN only after speed is nearly zero and the target
 * remains inside DOCK_FOV_ALIGN for this duration.
 *
 * @min 0.1
 * @max 5
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_REC_T, 1.0f);

/**
 * Maximum alignment translation speed [m/s]
 *
 * While the target is not centered, Dock Guidance limits the image-servo
 * translation to this speed. The correction stays perpendicular to target LOS,
 * so it removes image error without starting the high-speed approach.
 *
 * @min 0.1
 * @max 5
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_ALIGN_SPD, 1.0f);

/**
 * Image tracking proportional gain [1/s]
 *
 * Converts the mounting-aware camera pointing error into a desired LOS angular
 * correction. Target range then converts that angular correction into a
 * translational velocity perpendicular to the target LOS.
 *
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_IMG_P, 0.3f);

/**
 * Maximum image tracking translation speed [m/s]
 *
 * Limits the transverse velocity reserved for keeping the target in the fixed
 * camera. During maximum-speed approach this correction uses part of the total
 * DOCK_VEL_MAX velocity budget instead of increasing total speed.
 *
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_IMG_VMAX, 1.0f);

/**
 * Maximum image-correction fraction
 *
 * Maximum fraction of the total velocity budget reserved for fixed-camera
 * transverse image tracking. Lower values preserve closing speed and prevent
 * image correction from reversing the horizontal approach direction.
 *
 * @min 0.05
 * @max 0.8
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_IMG_FRAC, 0.40f);

/**
 * Maximum angle-only approach speed [m/s]
 *
 * Applied when DOCK_LEGACY_APP is enabled but no target-relative NED position
 * is available. Range-less operation has no terminal stopping-distance
 * envelope, so it must remain slower than a range-aided approach.
 *
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_LEG_VMAX, 5.0f);

/**
 * Allow angle-only approach with legacy Viztra frames
 *
 * When enabled, a valid 21-byte Viztra frame may enter acceleration and
 * maximum-speed approach without relative NED position. Image-angle tracking
 * and FOV protection remain active, but distance-based terminal braking is not
 * available. Disable to restrict range-less operation to ALIGN.
 *
 * @boolean
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_LEGACY_APP, 0);

/**
 * Enable maximum-speed terminal dash
 *
 * When enabled, a confirmed near-target condition commits the controller to
 * maximum approach speed. Image tracking continues while vision is valid; a
 * short final visual loss holds the last valid velocity direction.
 *
 * @boolean
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_TERM_EN, 1);

/**
 * Terminal-dash distance threshold [m]
 *
 * A positive value enables range-based terminal entry when target-relative
 * NED position is valid. Zero disables the distance trigger.
 *
 * @min 0
 * @max 100
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_DIST, 0.0f);

/**
 * Terminal-dash target-size threshold
 *
 * Maximum of target width/image width and target height/image height. This is
 * an entry trigger only and never reduces the commanded speed.
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_SIZE, 0.10f);

/**
 * Terminal-dash entry confirmation time [s]
 *
 * @min 0.02
 * @max 2.0
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_CFM, 0.05f);

/**
 * Terminal visual-loss hold time [s]
 *
 * Maximum time to retain the last valid maximum-speed velocity after vision
 * is lost in TERMINAL_DASH.
 *
 * @min 0.05
 * @max 3.0
 * @decimal 2
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_BLIND, 0.50f);

/**
 * Maximum terminal-dash duration [s]
 *
 * Safety bound for the complete terminal phase. Expiry is a fault boundary,
 * not a docking-complete indication.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_MAX_T, 1.5f);

/**
 * Terminal LOS filter cutoff [Hz]
 *
 * Higher bandwidth reduces LOS lag close to the target while retaining the
 * normal DOCK_FILT_HZ setting for the rest of the approach.
 *
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_TERM_FILT, 10.0f);

/**
 * Camera yaw-error deadband [deg]
 *
 * Horizontal image error inside this band does not change the vehicle yaw
 * setpoint.
 *
 * @min 0
 * @max 10
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_YAW_DB, 1.0f);

/**
 * Maximum yaw-setpoint slew rate [deg/s]
 *
 * Limits Dock Guidance yaw changes and prevents small image errors from
 * producing large heading steps when the LOS is steep.
 *
 * @min 1
 * @max 90
 * @decimal 1
 * @group Dock Guidance
 */
PARAM_DEFINE_FLOAT(DOCK_YAW_RATE, 10.0f);

/**
 * Visual-loss grace period [ms]
 *
 * During this interval the NED velocity setpoint decelerates toward zero.
 * The mode becomes unhealthy after the grace period expires.
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
 * the stopping-distance envelope always has a valid range. Without one, Dock
 * Guidance remains available in low-speed image-alignment mode only.
 *
 * @min 20
 * @max 2000
 * @group Dock Guidance
 */
PARAM_DEFINE_INT32(DOCK_REL_TOUT, 200);
