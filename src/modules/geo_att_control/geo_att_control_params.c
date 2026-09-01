#include <px4_platform_common/param.h>

/** Attitude and angular-rate controller execution rate. */
// @unit Hz
// @min 50
// @max 400
// @increment 1
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_RATE_HZ, 400.0);

/** Roll attitude error proportional gain. */
// @min 0
// @max 50
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_ROLL_P, 6.0);

/** Pitch attitude error proportional gain. */
// @min 0
// @max 50
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_PITCH_P, 6.0);

/** Yaw attitude error proportional gain. */
// @min 0
// @max 50
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_YAW_P, 4.0);

/** Roll angular-rate error proportional gain. */
// @min 0
// @max 20
// @increment 0.005
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_R_RATE_P, 4.0);

/** Pitch angular-rate error proportional gain. */
// @min 0
// @max 20
// @increment 0.005
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_P_RATE_P, 3.2);

/** Yaw angular-rate error proportional gain. */
// @min 0
// @max 20
// @increment 0.005
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_Y_RATE_P, 6.5);

/** Roll angular-rate integral gain. */
// @min 0
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_R_RATE_I, 0.05);

/** Pitch angular-rate integral gain. */
// @min 0
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_P_RATE_I, 0.05);

/** Yaw angular-rate integral gain. */
// @min 0
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_Y_RATE_I, 0.08);

/** Attitude error integral gain for steady trim and disturbance compensation. */
// @min 0
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_ATT_I, 0.15);

/** Common absolute limit for attitude and rate integral states. */
// @min 0
// @max 10
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_INT_LIM, 0.3);

/** Hard roll attitude reference limit. */
// @unit deg
// @min 1
// @max 179.9
// @increment 1
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_ROLL_MAX, 35.0);

/** Hard pitch attitude reference limit. */
// @unit deg
// @min 1
// @max 179.9
// @increment 1
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_PITCH_MAX, 35.0);

/** Enable continuous quaternion attitude references. */
// @boolean
// @group Geometric Attitude Control
PARAM_DEFINE_INT32(GAC_Q_REF_EN, 0);

/** Minimum average normalized motor command for yaw control. */
// @min 0
// @max 1
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_YAW_THR_MIN, 0.15);

/** Equivalent torque disturbance observer mode: 0 off, 1 observe, 2 compensate. */
// @min 0
// @max 2
// @increment 1
// @group Geometric Attitude Control
PARAM_DEFINE_INT32(GAC_DOB_MODE, 1);

/** Low-pass cutoff frequency of the equivalent torque disturbance observer. */
// @unit rad/s
// @min 0.05
// @max 20
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_DOB_CUTOFF, 1.0);

/** Per-axis absolute limit of the estimated disturbance torque. */
// @unit N m
// @min 0
// @max 10
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_DOB_LIM, 2.0);

/** Maximum attitude error allowed for disturbance observer learning. */
// @unit deg
// @min 0.5
// @max 20
// @increment 0.1
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_DOB_AMAX, 5.0);

/** Maximum angular rate allowed for disturbance observer learning; not a rate limit. */
// @unit rad/s
// @min 0.05
// @max 2
// @increment 0.01
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_DOB_RMAX, 0.35);

/** Roll-axis moment of inertia used by the geometric controller. */
// @unit kg m^2
// @min 0.001
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_IXX, 0.182);

/** Pitch-axis moment of inertia used by the geometric controller. */
// @unit kg m^2
// @min 0.001
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_IYY, 0.260);

/** Yaw-axis moment of inertia used by the geometric controller. */
// @unit kg m^2
// @min 0.001
// @max 10
// @increment 0.001
// @group Geometric Attitude Control
PARAM_DEFINE_FLOAT(GAC_IZZ, 0.384);
