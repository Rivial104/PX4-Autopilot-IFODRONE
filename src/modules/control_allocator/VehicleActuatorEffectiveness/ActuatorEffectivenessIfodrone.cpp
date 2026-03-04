/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * Allocation strategy:
 *   Motors 0-1 (coaxial, upward): THRUST_Z + YAW (differential KM).
 *   Motors 2-5 (side EDFs):       Tilted by servos. As the tilt angle
 *                                  changes, their thrust axes rotate,
 *                                  providing XY thrust + Z component
 *                                  + roll/pitch torque (cross product).
 *                                  An idle trim keeps them spinning.
 *   Tilts  0-3 (servos):          Allocated by CA for ROLL/PITCH.
 *                                  Effectiveness = 15.0 (high value to
 *                                  overcome RPY normalization scaling).
 *                                  updateAxisFromTiltSetpoints() re-linearizes
 *                                  the side motor axes every iteration.
 */

#include "ActuatorEffectivenessIfodrone.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
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

	// Configurable rotor set with tilt support.
	_mc_rotors.enableYawByDifferentialThrust(true);
	_mc_rotors.enablePropellerTorqueNonUpwards(true);

	auto &effectiveness = configuration.effectiveness_matrices[0];
	const int first_motor_idx = configuration.num_actuators_matrix[0];
	const bool motors_added = _mc_rotors.addActuators(configuration);

	// Set tilt index AFTER adding motors so we know the correct starting index
	const int first_non_motor_idx = configuration.num_actuators_matrix[0];
	_first_tilt_idx = first_non_motor_idx;

	// Initialize tilt setpoints if this is the first call
	if (_tilts.count() > 0) {
		if (!_tilt_setpoints_initialized) {
			for (int i = 0; i < _tilts.count(); ++i) {
				const int actuator_idx = _first_tilt_idx + i;

				if (actuator_idx >= 0 && actuator_idx < NUM_ACTUATORS) {
					_last_actuator_sp(actuator_idx) = neutralTiltControl(i);
				}
			}

			_tilt_setpoints_initialized = true;
		}

		_mc_rotors.updateAxisFromTiltSetpoints(_tilts, _last_actuator_sp, _first_tilt_idx);
	}
	// --- Post-process motor effectiveness rows ---
	// Main motors (0-1): leave untouched (pure Z + yaw differential).
	// Side motors (2-5): leave untouched — their roll/pitch/thrust_Z
	//                     effectiveness comes naturally from the tilted
	//                     axis (updated by updateAxisFromTiltSetpoints).
	//                     Only zero YAW (yaw is handled by main motors).
	const auto &geometry = _mc_rotors.geometry();

	for (int i = 0; i < geometry.num_rotors && (first_motor_idx + i) < first_non_motor_idx; ++i) {
		const int actuator_idx = first_motor_idx + i;

		if (actuator_idx >= NUM_ACTUATORS) {
			break;
		}

		const int tilt_index = geometry.rotors[i].tilt_index;

		if (tilt_index >= 0 && tilt_index < _tilts.count()) {
			// Side motors: zero YAW only.  Roll/pitch/thrust come from tilted axis.
			effectiveness(ControlAxis::YAW, actuator_idx) = 0.f;
		}
	}

	// --- Side motor idle trim ---
	// Keep side motors spinning so the tilts have a base thrust to redirect.
	for (int i = 0; i < geometry.num_rotors && (first_motor_idx + i) < first_non_motor_idx; ++i) {
		const int actuator_idx = first_motor_idx + i;

		if (actuator_idx >= NUM_ACTUATORS) {
			break;
		}

		if (geometry.rotors[i].tilt_index >= 0) {
			configuration.trim[0](actuator_idx) = _side_motor_idle;
		}
	}

	// --- Tilt servos ---
	// (_first_tilt_idx already set above after adding motors)
	_tilts.updateTorqueSign(_mc_rotors.geometry(), true);
	const bool tilts_added = _tilts.addActuators(configuration);

	updateTiltOffsets();

	// Restore tilt effectiveness for roll/pitch.  Use a high value (15.0)
	// to overcome the RPY normalization scaling in the pseudo-inverse.
	overwriteTiltEffectiveness(effectiveness);
	_tilt_setpoint_updated = false;

	return motors_added && tilts_added;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index != 0 || _first_tilt_idx < 0 || _tilts.count() == 0) {
		return;
	}

	// CA has already allocated roll/pitch to the tilt servos via the
	// effectiveness matrix.  We just add the neutral offset and track changes.

	bool tilt_setpoint_changed = false;

	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;

		if (actuator_idx < 0 || actuator_idx >= NUM_ACTUATORS) {
			continue;
		}

		// Add neutral offset to the CA output
		float tilt_sp = actuator_sp(actuator_idx) + _tilt_offsets(actuator_idx);

		if (!PX4_ISFINITE(tilt_sp)) {
			tilt_sp = _tilt_offsets(actuator_idx);
		}

		tilt_sp = math::constrain(tilt_sp, actuator_min(actuator_idx), actuator_max(actuator_idx));
		actuator_sp(actuator_idx) = tilt_sp;

		// Track changes to trigger axis re-linearization
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
	// Set tilt effectiveness for roll/pitch torque AND XY thrust.
	// Each tilt controls both torque and thrust based on rotor base axis direction.
	//
	// Physical mapping (body-frame):
	//   Tilt 0 (front, AX=+1): +pitch torque, +X thrust (when tilted positive)
	//   Tilt 1 (right, AY=+1): -roll torque,  +Y thrust (when tilted positive)
	//   Tilt 2 (back,  AX=-1): -pitch torque, -X thrust (when tilted positive)
	//   Tilt 3 (left,  AY=-1): +roll torque,  -Y thrust (when tilted positive)
	//
	// Symmetry preserved: front/back pair controls pitch+X, right/left pair controls roll+Y.
	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;

		if (actuator_idx < 0 || actuator_idx >= NUM_ACTUATORS) {
			continue;
		}

		// Zero all axes first
		for (int axis = 0; axis < NUM_AXES; ++axis) {
			effectiveness(axis, actuator_idx) = 0.f;
		}

		// Set effectiveness for torque + thrust (based on rotor base axis direction)
		if (i == 0) {
			// Front (AX=+1): positive tilt → +X thrust, +pitch torque
			effectiveness(ControlAxis::PITCH,    actuator_idx) =  1.f;
			effectiveness(ControlAxis::THRUST_X, actuator_idx) =  1.f;

		} else if (i == 1) {
			// Right (AY=+1): positive tilt → +Y thrust, -roll torque
			effectiveness(ControlAxis::ROLL,     actuator_idx) = -1.f;
			effectiveness(ControlAxis::THRUST_Y, actuator_idx) =  1.f;

		} else if (i == 2) {
			// Back (AX=-1): positive tilt → -X thrust, -pitch torque
			effectiveness(ControlAxis::PITCH,    actuator_idx) = 1.f;
			effectiveness(ControlAxis::THRUST_X, actuator_idx) = 1.f;

		} else if (i == 3) {
			// Left (AY=-1): positive tilt → -Y thrust, +roll torque
			effectiveness(ControlAxis::ROLL,     actuator_idx) =  1.f;
			effectiveness(ControlAxis::THRUST_Y, actuator_idx) = -1.f;
		}
	}
}
