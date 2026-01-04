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
