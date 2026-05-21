/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * Tilt servo angles are commanded directly by ifo_att_control (actuator_servos).
 * This module only allocates motor forces; it reads the current servo state
 * to keep side-EDF motor axes up-to-date for correct lateral thrust allocation.
 *
 *   Motors 0-1 : coaxial pair (−Z axis) — Z-thrust + yaw via differential KM
 *   Motors 2-5 : side EDFs — lateral XY thrust, axes updated from tilt state
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
		EffectivenessUpdateReason /*external_update*/)
{
	configuration.selected_matrix = 0;

	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	// Read tilt angles published directly by ifo_att_control.
	// Values are in [-1, 1] (normalized servo command).
	// Used only to update motor axes so lateral thrust is correctly allocated.
	actuator_servos_s actuator_servos{};

	if (_actuator_servos_sub.copy(&actuator_servos)) {
		for (int i = 0; i < _tilts.count() && i < actuator_servos_s::NUM_CONTROLS; ++i) {
			_current_tilt_values(i) = actuator_servos.control[i];
		}
	}

	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _current_tilt_values, 0);

	return _mc_motors.addActuators(configuration);
}
