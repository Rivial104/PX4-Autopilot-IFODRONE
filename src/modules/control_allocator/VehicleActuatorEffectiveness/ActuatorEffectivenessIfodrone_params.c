/**
* IFODRONE tilt hover angle.
*
* Default tilt angle of the side EDF servos when no roll/pitch correction is
* applied (hover). Positive = nozzle pointing down (thrust component upward).
* Read by ActuatorEffectivenessIfodrone to bias the tilt servos at CA=0.
*
* @unit deg
* @min -60.0
* @max 90.0
* @decimal 1
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(IFO_TILT_HOVER, 45.0f);
