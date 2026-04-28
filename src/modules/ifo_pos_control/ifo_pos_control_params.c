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
 * @max 2.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_Z_P, 0.2f);

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
 * @max 2.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_VEL_XY_P, 0.2f);

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
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_THR_HOVER, 0.5f);

/**
 * IFODRONE position filter mode
 *
 * When enabled, intercepts mc_pos_control's attitude setpoint and
 * redistributes thrust for the IFODRONE body-frame motors (level body,
 * XY thrust via tilt motors).
 *
 * 0: Disabled (mc_pos_control attitude setpoint passes through)
 * 1: Enabled (level body, XY thrust to tilt motors)
 *
 * @min 0
 * @max 1
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_INT32(IFO_POS_MODE, 0);

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
PARAM_DEFINE_FLOAT(IFO_THR_XY_MAX, 0.35f);

/**
 * Horizontal thrust scale
 *
 * Multiplies mc_pos_control XY thrust before the IFODRONE-specific corrections are added.
 *
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_XY_THR_SCL, 1.0f);

/**
 * Horizontal velocity correction gain
 *
 * Extra XY thrust from velocity tracking error in NED. Higher values accelerate and brake more aggressively.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_XY_VEL_P, 0.0f);

/**
 * Horizontal acceleration feed-forward gain
 *
 * Extra XY thrust from mc_pos_control acceleration setpoint. This helps anticipate braking near the waypoint.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group IFODRONE Position Control
 */
PARAM_DEFINE_FLOAT(IFO_XY_ACC_FF, 0.0f);
