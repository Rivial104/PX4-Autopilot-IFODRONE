#include "ActuatorEffectivenessIfodrone.hpp"

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	_first_main_idx = 0;
	_first_side_idx = MAIN_MOTORS_NUM;
}


bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// All 6 motors (2 main + 4 side)
	configuration.selected_matrix = 0;

	// Enable differential yaw for main motors (tilts don't contribute to yaw)
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());

	const bool motors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts for side motors
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset such that tilts point upwards when control input == 0
	// (trim is 0 if min_angle == -max_angle)
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	return (motors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// BASIC FLIGHT MODE - No tilts, all 6 motors provide Z thrust
	// control_sp indices:
	// [0] = roll torque (tau_x)
	// [1] = pitch torque (tau_y)
	// [2] = yaw torque (tau_z)
	// [3] = thrust X (Fx) - not used in basic flight
	// [4] = thrust Y (Fy) - not used in basic flight
	// [5] = thrust Z (Fz) - NEGATIVE in NED means UP

	// Don't zero - let the standard allocation work
	// The standard mixer will handle roll/pitch/yaw/thrust distribution
	// based on the effectiveness matrix from CA_ROTOR* parameters

	// We only need to handle special cases here
	// For basic flight, let the default allocation do its job
	(void)control_sp;
	(void)matrix_index;
	(void)actuator_sp;
	(void)actuator_min;
	(void)actuator_max;
}

void ActuatorEffectivenessIfodrone::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}
