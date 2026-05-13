/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * Two-stage allocation:
 *   1. Tilt servos (4) — primary roll/pitch torque actuators
 *   2. Motors (6)      — thrust (XYZ) + yaw (differential KM on motors 0-1)
 *                         + secondary roll/pitch torque via tilted axes
 *
 * Every cycle:
 *   - Read current tilt positions from actuator_servos (CA output, prev cycle)
 *   - Update side motor axes via Rodrigues rotation
 *   - Rebuild the 6×10 effectiveness matrix (6 motors + 4 tilt servos)
 *
 * Tilt effectiveness (verified via Rodrigues rotation, matching SDF joint axes):
 *   Tilt 0 (front, base +X, hinge +Y): +1 command → axis gains −Z → lifts front → +pitch
 *   Tilt 1 (right, base +Y, hinge +X): +1 command → axis gains +Z → pushes right down → +roll
 *   Tilt 2 (back,  base −X, hinge −Y): +1 command → axis gains −Z → lifts back  → −pitch
 *   Tilt 3 (left,  base −Y, hinge −X): +1 command → axis gains +Z → pushes left down → −roll
 */

#include "ActuatorEffectivenessIfodrone.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_motors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	_current_tilt_values.setAll(0.f);
	_tilt_offsets.setAll(0.f);
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Always recompute — tilt angles change every cycle.
	configuration.selected_matrix = 0;

	// Yaw: only from motors 0-1 differential thrust (KM = ±0.6).
	// Side motors 2-5: KM zeroed by enablePropellerTorqueNonUpwards(false).
	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	// ── Read current tilt state (CA output from previous cycle) ──────
	actuator_servos_s actuator_servos{};

	if (_actuator_servos_sub.copy(&actuator_servos)) {
		for (int i = 0; i < _tilts.count() && i < actuator_servos_s::NUM_CONTROLS; ++i) {
			const float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

			if (delta_angle > FLT_EPSILON) {
				const float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
				_current_tilt_values(i) = actuator_servos.control[i] + trim;

			} else {
				_current_tilt_values(i) = 0.f;
			}
		}
	}

	// ── Update motor axes from current tilts ─────────────────────────
	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _current_tilt_values, 0);

	// ── Add motors (columns 0-5) ─────────────────────────────────────
	const bool motors_ok = _mc_motors.addActuators(configuration);

	// ── Add tilts (columns 6-9) with roll/pitch effectiveness ────────
	_first_tilt_col = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool tilts_ok = _tilts.addActuators(configuration);

	// Override the tilt effectiveness — _tilts.addActuators sets _torque
	// from CA_SV_TL*_CT params (=0 → zero columns). We override manually
	// because IFODRONE tilts produce roll AND pitch, which the standard
	// tilt system doesn't support.
	//
	// Sensitivity at tilt angle θ (linearised at current θ):
	//   d(moment)/d(servo) ≈ ct × arm × cos(θ_current) × (max_angle − min_angle)/2
	// For simplicity use a constant K that works well across the range.
	const float K = 0.5f;

	auto &eff = configuration.effectiveness_matrices[configuration.selected_matrix];
	// Tilt 0 (front): +pitch
	eff(1, _first_tilt_col + 0) =  K;
	// Tilt 1 (right): +roll  (SDF hinge +X: positive tilt → +Z → pushes right down)
	eff(0, _first_tilt_col + 1) =  -K;
	// Tilt 2 (back):  −pitch
	eff(1, _first_tilt_col + 2) = -K;
	// Tilt 3 (left):  −roll  (SDF hinge −X: positive tilt → +Z → pushes left down)
	eff(0, _first_tilt_col + 3) = K;

	// Compute tilt trim offsets (for symmetric ±45° range, trim = 0)
	_tilt_offsets.setAll(0.f);

	for (int i = 0; i < _tilts.count(); ++i) {
		const float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			_tilt_offsets(_first_tilt_col + i) = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
		}
	}

	return motors_ok && tilts_ok;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(
	const matrix::Vector<float, NUM_AXES> &control_sp,
	int matrix_index, ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// Add tilt trim offsets so that servo=0 corresponds to zero tilt angle.
	// For symmetric range (−45°..+45°) this is a no-op (trim=0).
	actuator_sp += _tilt_offsets;
}
