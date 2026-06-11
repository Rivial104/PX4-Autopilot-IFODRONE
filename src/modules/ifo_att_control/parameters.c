/**
* IFODRONE tilt hover angle.
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
* Attitude P gain — roll/pitch (outer loop).
*
* Converts roll/pitch angle error [rad] to rate setpoint [rad/s].
*
* @unit rad/s
* @min 0.0
* @max 20.0
* @decimal 2
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_ATT_P, 4.0f);

/**
* Yaw attitude P gain (outer loop).
*
* @unit rad/s
* @min 0.0
* @max 10.0
* @decimal 2
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_YAW_P, 2.0f);
