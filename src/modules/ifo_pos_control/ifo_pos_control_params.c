/**
 * IFODRONE Position Control Parameters
 */

/**
 * Position Z proportional gain
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_POS_Z_P, 1.0f);

/**
 * Velocity Z proportional gain
 * @min 0.0
 * @max 8.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_Z_P, 4.0f);

/**
 * Velocity Z integral gain
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_Z_I, 0.2f);

/**
 * Velocity Z derivative gain
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_Z_D, 0.0f);

/**
 * Position XY proportional gain
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_POS_XY_P, 0.95f);

/**
 * Velocity XY proportional gain
 * @min 0.0
 * @max 8.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_XY_P, 1.8f);

/**
 * Velocity XY integral gain
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_XY_I, 0.4f);

/**
 * Velocity XY derivative gain
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_XY_D, 0.2f);

/**
 * Maximum thrust (main motors)
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_MAX, 0.9f);

/**
 * Minimum thrust (main motors)
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_MIN, 0.12f);

/**
 * Hover thrust (normalized, vehicle hovers at this value)
 * @min 0.05
 * @max 0.9
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_HOVER, 0.5f);

/**
 * Maximum XY thrust
 *
 * Maximum normalized horizontal thrust (0..1).
 * Limits the magnitude of the horizontal body-frame thrust vector.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_XY_MAX, 0.35f);

/**
 * Maximum horizontal velocity
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_MAX_XY, 5.0f);

/**
 * Maximum upward velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_MAX_UP, 3.0f);

/**
 * Maximum downward velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_MAX_DN, 1.5f);

/**
 * Takeoff climb speed
 *
 * @unit m/s
 * @min 0.5
 * @max 5.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_TKO_SPEED, 1.5f);

/**
 * Landing descent speed
 *
 * @unit m/s
 * @min 0.2
 * @max 5.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_LAND_SPEED, 0.7f);

/**
 * Takeoff ramp time
 *
 * Time to ramp up velocity during takeoff.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_TKO_RAMP_T, 2.0f);
