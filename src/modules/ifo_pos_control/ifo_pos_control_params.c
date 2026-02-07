/**
 * IFODRONE Position Controller Parameters
 */

/**
 * Position Z P gain
 *
 * Proportional gain for altitude control
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_POS_Z_P, 1.0f);

/**
 * Velocity Z P gain
 *
 * Proportional gain for vertical velocity control
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_Z_P, 0.5f);

/**
 * Position I gain
 *
 * Integral gain for position control.
 * Helps remove steady-state position error caused by hover thrust mismatch.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_POS_I, 0.2f);

/**
 * Velocity I gain
 *
 * Integral gain for velocity control.
 * Helps remove steady-state velocity error caused by hover thrust mismatch.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_I, 0.2f);

/**
 * Position XY P gain
 *
 * Proportional gain for horizontal position control (tilt motors)
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_POS_XY_P, 0.8f);

/**
 * Velocity XY P gain
 *
 * Proportional gain for horizontal velocity control (tilt motors)
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_XY_P, 0.4f);

/**
 * Hover thrust
 *
 * Normalized thrust needed to hover (0..1)
 *
 * @min 0.1
 * @max 0.9
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_HOVER, 0.5f);

/**
 * Maximum thrust
 *
 * Maximum normalized thrust (0..1)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_MAX, 0.9f);

/**
 * Minimum thrust
 *
 * Minimum normalized thrust (0..1)
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_MIN, 0.1f);

/**
 * Maximum XY thrust
 *
 * Maximum normalized horizontal thrust from tilt motors (0..1)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_XY_MAX, 0.3f);
