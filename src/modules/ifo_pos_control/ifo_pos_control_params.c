/**
 * IFODRONE Position Filter Parameters
 */

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
