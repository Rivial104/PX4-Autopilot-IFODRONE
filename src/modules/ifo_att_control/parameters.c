/**
* Maximum value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_XY_P, 1.0f);

/**
* Minimum value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_Z_P, -1.0f);

/**
* Step value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_ACC_HOR, 0.1f);

/**
* Step value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_ACC_UP, 0.1f);

/**
* IFODRONE attitude control mode.
*
* When > 0 the ifo_att_control module forces roll=0, pitch=0
* in the attitude setpoint and feeds the standard quaternion
* attitude controller.  0 = disabled (mc_att_control runs instead).
*
* @min 0
* @max 1
* @value 0 Disabled
* @value 1 Enabled – forced level
* @group IFODRONE
*/
PARAM_DEFINE_INT32(IFO_ATT_MODE, 0);
