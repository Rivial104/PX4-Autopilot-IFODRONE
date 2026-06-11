/**
* Rate P gain — roll/pitch (inner loop).
*
* Converts roll/pitch rate error [rad/s] to normalized torque setpoint.
* Increase if attitude tracks too slowly; decrease if oscillating.
*
* @min 0.0
* @max 5.0
* @decimal 3
* @increment 0.01
* @group IFODRONE Rate Control
*/
PARAM_DEFINE_FLOAT(IFO_RATE_P, 0.15f);

/**
* Rate D gain — roll/pitch (inner loop).
*
* Derivative damping on roll/pitch rate.
* Suppresses high-frequency oscillation of the tilt servos.
* Start at 0 and increase carefully.
*
* @min 0.0
* @max 0.1
* @decimal 4
* @increment 0.001
* @group IFODRONE Rate Control
*/
PARAM_DEFINE_FLOAT(IFO_RATE_D, 0.002f);

/**
* Yaw rate P gain (inner loop).
*
* @min 0.0
* @max 5.0
* @decimal 3
* @increment 0.01
* @group IFODRONE Rate Control
*/
PARAM_DEFINE_FLOAT(IFO_YRATE_P, 0.3f);

/**
* Yaw rate D gain (inner loop).
*
* @min 0.0
* @max 0.1
* @decimal 4
* @increment 0.001
* @group IFODRONE Rate Control
*/
PARAM_DEFINE_FLOAT(IFO_YRATE_D, 0.0f);
