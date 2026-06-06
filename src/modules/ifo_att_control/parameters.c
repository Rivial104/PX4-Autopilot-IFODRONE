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
* Attitude controller proportional gain (roll/pitch).
*
* P gain applied to roll and pitch angle error.
* Increase to stiffen attitude hold; decrease if oscillating.
*
* @unit norm
* @min 0.0
* @max 10.0
* @decimal 2
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_ATT_P, 2.0f);

/**
* Attitude controller derivative gain (roll/pitch).
*
* D gain applied to roll and pitch rate (angular velocity).
* Provides damping — critical for preventing diverging oscillations.
* Start at 0.05 and increase until oscillations are damped.
*
* @unit norm
* @min 0.0
* @max 2.0
* @decimal 3
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_ATT_D, 0.2f);

/**
* Yaw controller proportional gain.
*
* @unit norm
* @min 0.0
* @max 5.0
* @decimal 2
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_YAW_P, 0.3f);

/**
* Yaw controller derivative gain.
*
* @unit norm
* @min 0.0
* @max 1.0
* @decimal 3
* @group IFODRONE Attitude Control
*/
PARAM_DEFINE_FLOAT(IFO_YAW_D, 0.1f);
