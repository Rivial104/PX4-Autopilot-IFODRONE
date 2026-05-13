/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * The effectiveness matrix is recomputed every cycle because tilt angles
 * (commanded externally by ifo_att_control) change the motor thrust axes:
 *
 *   Side motor axis (tilted) = base_axis * cos(θ) + hinge × base_axis * sin(θ)
 *
 * When a side motor tilts by angle θ:
 *   - Horizontal thrust component scales by cos(θ)  (X or Y)
 *   - Vertical   thrust component appears  as sin(θ) (−Z = upward)
 *   - Moment = position × axis  →  tilted motors generate roll/pitch torque
 *
 * Motors 0-1: coaxial pair, fixed upward axis (−Z), yaw via differential KM
 * Motors 2-5: side EDFs, axes updated from tilt servo positions each cycle
 *
 * Tilts are NOT added as CA actuators — ifo_att_control publishes them
 * directly via actuator_servos.  Adding them would make CA overwrite those
 * commands with zeros.
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
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Always recompute — tilt angles change every cycle.
	// ControlAllocator already bypasses the 100 ms rate-limit for IFODRONE.
	configuration.selected_matrix = 0;

	// ── Yaw control ──────────────────────────────────────────────────
	// Yaw is produced ONLY by motors 0-1 via differential thrust (KM = ±0.1).
	// Side motors 2-5 have KM = 0.01 but enablePropellerTorqueNonUpwards(false)
	// zeroes KM for any motor whose axis is not pointing upward, so their yaw
	// column is always zero.  enableYawByDifferentialThrust(true) keeps the
	// yaw column from being blanked globally.
	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	// ── Read current tilt positions ──────────────────────────────────
	// ifo_att_control publishes control[0..3] in [−1, +1].
	// Convert to the internal tilt-control range expected by updateAxisFromTiltSetpoints.
	actuator_servos_s actuator_servos{};

	if (_actuator_servos_sub.copy(&actuator_servos)) {
		for (int i = 0; i < _tilts.count() && i < actuator_servos_s::NUM_CONTROLS; ++i) {
			const float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

			if (delta_angle > FLT_EPSILON) {
				// For symmetric range (−45°..+45°) trim = 0, so _current = control directly.
				const float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
				_current_tilt_values(i) = actuator_servos.control[i] + trim;

			} else {
				_current_tilt_values(i) = 0.f;
			}
		}
	}

	// ── Update motor axes BEFORE computing the effectiveness matrix ──
	// Rodrigues rotation applied per-motor:
	//   Motors 0-1: tilt_index = −1  → skipped, axis stays (0,0,−1)
	//   Motors 2-5: axis = base*cos(θ) + (hinge × base)*sin(θ)
	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _current_tilt_values, 0);

	// ── Build the effectiveness matrix (6 motors, NO servos) ─────────
	// computeEffectivenessMatrix (called inside addActuators) fills:
	//   rows 0-2: moment  = ct * position × axis  −  ct * km * axis
	//   rows 3-5: thrust  = ct * axis
	// Only called ONCE, after axes are already updated.
	return _mc_motors.addActuators(configuration);
}

void ActuatorEffectivenessIfodrone::updateSetpoint(
	const matrix::Vector<float, NUM_AXES> &control_sp,
	int matrix_index, ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// Nothing to do — tilts are commanded externally by ifo_att_control,
	// and motor setpoints are fully handled by the sequential-desaturation allocator.
}
