/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * Minimal IFODRONE control allocator effectiveness.
 * Motors 0-1 (coaxial, upward) provide Z-thrust and yaw via differential thrust.
 * Motors 2-5 (side EDFs with tilts) provide XY-thrust; tilts provide roll/pitch torque.
 */

#include "ActuatorEffectivenessIfodrone.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilt_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	_tilt_offsets.setZero();
	_last_actuator_sp.setAll(NAN);
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !_tilt_setpoint_updated) {
		return false;
	}

	configuration.selected_matrix = 0;

	// Main motors handle Z and yaw, while side motors contribute in XY.
	_mc_rotors.enableYawByDifferentialThrust(true);
	_mc_rotors.enablePropellerTorqueNonUpwards(false);
	const bool mc_motors_added = _mc_rotors.addActuators(configuration);

	_tilt_rotors.enableYawByDifferentialThrust(false);
	_tilt_rotors.enablePropellerTorqueNonUpwards(true);

	if (_tilts.count() > 0) {
		const int first_tilt_idx_for_axis_update = (_first_tilt_idx >= 0) ? _first_tilt_idx : _tilt_rotors.geometry().num_rotors;

		if (!_tilt_setpoints_initialized) {
			for (int i = 0; i < _tilts.count(); ++i) {
				const int actuator_idx = first_tilt_idx_for_axis_update + i;

				if (actuator_idx >= 0 && actuator_idx < NUM_ACTUATORS) {
					_last_actuator_sp(actuator_idx) = neutralTiltControl(i);
				}
			}

			_tilt_setpoints_initialized = true;
		}

		_tilt_rotors.updateAxisFromTiltSetpoints(_tilts, _last_actuator_sp, first_tilt_idx_for_axis_update);
	}

	const bool tilt_motors_added = _tilt_rotors.addActuators(configuration);

	// Tilt servos
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_tilt_rotors.geometry(), true);
	const bool tilts_added = _tilts.addActuators(configuration);

	updateTiltOffsets();
	overwriteTiltEffectiveness(configuration.effectiveness_matrices[0]);
	_tilt_setpoint_updated = false;

	return mc_motors_added && tilt_motors_added && tilts_added;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index != 0 || _first_tilt_idx < 0 || _tilts.count() == 0) {
		return;
	}

	bool tilt_setpoint_changed = false;

	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;

		if (actuator_idx < 0 || actuator_idx >= NUM_ACTUATORS) {
			continue;
		}

		float tilt_sp = tiltCommandFromTorqueSetpoint(i, control_sp);

		if (!PX4_ISFINITE(tilt_sp)) {
			tilt_sp = actuator_sp(actuator_idx) + _tilt_offsets(actuator_idx);
		}

		if (!PX4_ISFINITE(tilt_sp)) {
			tilt_sp = _tilt_offsets(actuator_idx);
		}

		tilt_sp = math::constrain(tilt_sp, actuator_min(actuator_idx), actuator_max(actuator_idx));
		actuator_sp(actuator_idx) = tilt_sp;

		if (!PX4_ISFINITE(_last_actuator_sp(actuator_idx))
		    || fabsf(tilt_sp - _last_actuator_sp(actuator_idx)) > 0.005f) {
			tilt_setpoint_changed = true;
		}

		_last_actuator_sp(actuator_idx) = tilt_sp;
	}

	_tilt_setpoint_updated = _tilt_setpoint_updated || tilt_setpoint_changed;
}

float ActuatorEffectivenessIfodrone::neutralTiltControl(int tilt_index) const
{
	if (tilt_index < 0 || tilt_index >= _tilts.count()) {
		return 0.f;
	}

	const ActuatorEffectivenessTilts::Params &tilt_cfg = _tilts.config(tilt_index);
	const float delta_angle = tilt_cfg.max_angle - tilt_cfg.min_angle;

	if (delta_angle <= FLT_EPSILON) {
		return 0.f;
	}

	// Control value that commands 0 rad physical tilt angle.
	const float neutral = -1.f - 2.f * tilt_cfg.min_angle / delta_angle;
	return math::constrain(neutral, -1.f, 1.f);
}

float ActuatorEffectivenessIfodrone::tiltCommandFromTorqueSetpoint(int tilt_index,
		const matrix::Vector<float, NUM_AXES> &control_sp) const
{
	if (tilt_index < 0 || tilt_index >= _tilts.count()) {
		return NAN;
	}

	const float roll_torque_sp = math::constrain(control_sp(ControlAxis::ROLL), -1.f, 1.f);
	const float pitch_torque_sp = math::constrain(control_sp(ControlAxis::PITCH), -1.f, 1.f);
	const float neutral = neutralTiltControl(tilt_index);

	// Fixed body-frame mapping (independent of world position):
	// 0=front(+X), 1=right(+Y), 2=back(-X), 3=left(-Y).
	switch (tilt_index) {
	case 0: return neutral + pitch_torque_sp; // front tilt -> pitch+
	case 1: return neutral + roll_torque_sp;  // right tilt -> roll+
	case 2: return neutral - pitch_torque_sp; // back tilt -> pitch-
	case 3: return neutral - roll_torque_sp;  // left tilt -> roll-
	default: return NAN;
	}
}

void ActuatorEffectivenessIfodrone::updateTiltOffsets()
{
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;

		if (actuator_idx >= 0 && actuator_idx < NUM_ACTUATORS) {
			_tilt_offsets(actuator_idx) = neutralTiltControl(i);
		}
	}
}

void ActuatorEffectivenessIfodrone::overwriteTiltEffectiveness(EffectivenessMatrix &effectiveness) const
{
	// Tilts are used as orientation actuators (roll/pitch only) for IFODRONE.
	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;

		if (actuator_idx < 0 || actuator_idx >= NUM_ACTUATORS) {
			continue;
		}

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			effectiveness(axis, actuator_idx) = 0.f;
		}
		const float pitch_eff = (i == 0) ? 1.f : ((i == 2) ? -1.f : 0.f);
		const float roll_eff = (i == 1) ? 1.f : ((i == 3) ? -1.f : 0.f);

		effectiveness(ControlAxis::PITCH, actuator_idx) = pitch_eff;
		effectiveness(ControlAxis::ROLL, actuator_idx) = roll_eff;
	}
}
