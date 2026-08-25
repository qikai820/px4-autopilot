/**
 * Bifilar pendulum inertia measurement parameters.
 */

/**
 * Measured all-up mass
 *
 * Include battery, payload, and the fixed MorphQuad gimbal configuration.
 *
 * @unit kg
 * @min 0.05
 * @max 100
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_MASS, 4.56f);

/**
 * Distance between the two suspension ropes
 *
 * This is the centre-to-centre distance. The calculation uses half this
 * distance, as required by the bifilar pendulum equation.
 *
 * @unit m
 * @min 0.02
 * @max 10
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_DIST, 0.40f);

/**
 * Suspension rope length
 *
 * Measured from the upper support to the airframe attachment point.
 *
 * @unit m
 * @min 0.10
 * @max 20
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_LEN, 1.00f);

/**
 * Measurements per axis
 *
 * Number of separate bifilar-pendulum releases to average in one
 * inertia_measure measure command.
 *
 * @min 1
 * @max 20
 * @group Inertia Measure
 */
PARAM_DEFINE_INT32(INR_CNT, 3);

/**
 * Measured body X-axis inertia
 *
 * @unit kg m^2
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_IXX, 0.0f);

/**
 * Measured body Y-axis inertia
 *
 * @unit kg m^2
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_IYY, 0.0f);

/**
 * Measured body Z-axis inertia
 *
 * @unit kg m^2
 * @group Inertia Measure
 */
PARAM_DEFINE_FLOAT(INR_IZZ, 0.0f);
