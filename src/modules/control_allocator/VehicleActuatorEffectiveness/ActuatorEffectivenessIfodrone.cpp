/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * Both motors and tilt servos participate in the CA effectiveness matrix.
 * ifo_att_control publishes vehicle_torque_setpoint (roll+pitch+yaw);
 * the CA allocates to all 10 actuators simultaneously.
 *
 *   Motors 0-1 : coaxial pair (−Z axis) — Z-thrust + yaw via differential KM
 *   Motors 2-5 : side EDFs — lateral XY thrust (static horizontal axes)
 *   Servos 0-3 : tilt servos — roll/pitch torque
 *     Servo 0 (front, TD=0):  +1 cmd → −pitch torque (nose down)
 *     Servo 1 (right, TD=90): +1 cmd → +roll torque  (right wing down)
 *     Servo 2 (back, TD=180): +1 cmd → +pitch torque (nose up)
 *     Servo 3 (left, TD=270): +1 cmd → −roll torque  (right wing up)
 */

#include "ActuatorEffectivenessIfodrone.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_motors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	_tilt_offsets.setAll(0.f);
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason /*external_update*/)
{
	configuration.selected_matrix = 0;

	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	const bool motors_ok = _mc_motors.addActuators(configuration);

	// Side EDFs are unidirectional. Keep each opposing pair on the same positive
	// baseline and let CA realize X/Y by increasing one motor and decreasing the
	// opposite one around that shared midpoint.
	for (int i = 2; i <= 5; ++i) {
		configuration.trim[configuration.selected_matrix](i) = 0.5f;
	}

	_first_tilt_idx = configuration.num_actuators_matrix[0];

	// Manually add 4 tilt servos with physically correct roll/pitch effectiveness.
	// The standard ActuatorEffectivenessTilts::updateTorqueSign only handles pitch and
	// yaw; it cannot represent right/left tilts that create roll torque.
	// Sign derivation: a +1 servo command on the front EDF tilts its thrust vector to
	// create nose-up (positive pitch) torque; back EDF is opposite; right/left create ±roll.
	static const Vector3f tilt_torques[4] = {
		{0.f, -1.f, 0.f},   // Servo 0: front (TD=0)   → −pitch (nose down)
		{ 1.f, 0.f, 0.f},   // Servo 1: right (TD=90)  → +roll  (right wing down)
		{0.f,  1.f, 0.f},   // Servo 2: back  (TD=180) → +pitch (nose up)
		{-1.f, 0.f, 0.f},   // Servo 3: left  (TD=270) → −roll  (right wing up)
	};

	for (int i = 0; i < 4; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, tilt_torques[i], Vector3f{});
	}

	// Tilt offsets: correction so that CA=0 corresponds to level (0° tilt).
	// For symmetric min/max (e.g. −80° to +80°) this evaluates to 0.
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		const float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			_tilt_offsets(_first_tilt_idx + i) = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
		}
	}

	return motors_ok;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &/*control_sp*/,
		int /*matrix_index*/, ActuatorVector &actuator_sp,
		const ActuatorVector &/*actuator_min*/, const ActuatorVector &/*actuator_max*/)
{
	actuator_sp += _tilt_offsets;
}
