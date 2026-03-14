/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 * Allocates motor forces with dynamic motor axes based on tilt positions.
 * Tilt servos are commanded externally by the attitude controller (ifo_att_control).
 *
 * Motors 0-1: coaxial pair (Z-axis) providing Z-thrust and yaw via differential thrust
 * Motors 2-5: side EDFs with axes updated based on tilt servo positions
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

	// Refresh axes from the most recent externally commanded tilt setpoints.
	actuator_servos_s actuator_servos{};
	if (_actuator_servos_sub.copy(&actuator_servos)) {
		for (int i = 0; i < _tilts.count() && i < actuator_servos_s::NUM_CONTROLS; ++i) {
			_current_tilt_values(i) = actuator_servos.control[i];
		}
	}

	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _current_tilt_values, 0);

	return _mc_motors.addActuators(configuration);
}
