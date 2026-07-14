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
* IFODRONE tilt hover angle.
*
* Default tilt angle of the side EDF servos when no roll/pitch correction is
* applied (hover). Positive = nozzle pointing down (thrust component upward).
*
* @unit deg
* @min -60.0
* @max 90.0
* @decimal 1
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_TILT_HOVER, 45.0f);
