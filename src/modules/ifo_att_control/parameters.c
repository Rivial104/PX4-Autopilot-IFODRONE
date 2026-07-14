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

/**
* IFODRONE tilt servo authority gain.
*
* Multiplies the tilt-servo deflection commanded by the control allocator
* around the IFO_TILT_HOVER offset. >1 = larger servo angles for the same
* demanded roll/pitch torque. Applied after allocation, so it is not
* cancelled by the allocator's roll/pitch mix normalization.
*
* @min 0.1
* @max 5.0
* @decimal 2
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_TILT_GAIN, 2.0f);
