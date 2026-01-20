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
	// control_sp indices:
	// [0] = roll torque (tau_x)
	// [1] = pitch torque (tau_y)
	// [2] = yaw torque (tau_z)
	// [3] = thrust X (Fx)
	// [4] = thrust Y (Fy)
	// [5] = thrust Z (Fz)

	actuator_sp.setZero();

	const float tau_x = control_sp(0);  // Roll torque
	const float tau_y = control_sp(1);  // Pitch torque
	const float tau_z = control_sp(2);  // Yaw torque
	const float Fz_des = control_sp(5); // Vertical thrust (negative = up in NED)

	// ========================================
	// MAIN MOTORS (indices 0, 1) - Z-axis thrust + differential yaw
	// ========================================
	// Motor 0: CW rotation (positive yaw moment with increased thrust)
	// Motor 1: CCW rotation (negative yaw moment with increased thrust)

	const float thrust_per_main = Fz_des / static_cast<float>(MAIN_MOTORS_NUM);
	const float yaw_differential = tau_z * 0.5f;

	// Motor 0 (CW): base thrust + yaw contribution
	actuator_sp(_first_main_idx + 0) = thrust_per_main + yaw_differential;
	// Motor 1 (CCW): base thrust - yaw contribution
	actuator_sp(_first_main_idx + 1) = thrust_per_main - yaw_differential;

	// Constrain main motors
	for (int i = 0; i < MAIN_MOTORS_NUM; ++i) {
		actuator_sp(_first_main_idx + i) = math::constrain(
			actuator_sp(_first_main_idx + i),
			actuator_min(_first_main_idx + i),
			actuator_max(_first_main_idx + i)
		);
	}

	// ========================================
	// SIDE MOTORS (indices 2, 3, 4, 5) - Plus configuration with tilt
	// ========================================

	// Get rotor arm lengths from geometry
	const float arm_x_front = fabsf(_mc_rotors.geometry().rotors[2].position(0)); // Front motor X position
	const float arm_x_back  = fabsf(_mc_rotors.geometry().rotors[4].position(0)); // Back motor X position
	const float arm_y_right = fabsf(_mc_rotors.geometry().rotors[3].position(1)); // Right motor Y position
	const float arm_y_left  = fabsf(_mc_rotors.geometry().rotors[5].position(1)); // Left motor Y position

	// Calculate tilt angles for roll (tau_x) and pitch (tau_y) control

	float tilt_front = 0.f;  // Motor 2
	float tilt_right = 0.f;  // Motor 3
	float tilt_back  = 0.f;  // Motor 4
	float tilt_left  = 0.f;  // Motor 5

	// Base thrust for side motors (can be zero or small for hover stabilization)
	const float side_motor_base_thrust = 0.1f;

	// Pitch control: front/back motors tilt to create pitch torque
	// Positive pitch (nose up) = front motor tilts forward, back motor tilts backward
	if (_tilts.count() >= 4 && (arm_x_front > FLT_EPSILON) && (arm_x_back > FLT_EPSILON)) {
		const float pitch_gain = 1.0f;
		tilt_front = -tau_y * pitch_gain;  // Negative: tilt forward for positive pitch
		tilt_back  =  tau_y * pitch_gain;  // Positive: tilt backward for positive pitch
	}

	// Roll control: right/left motors tilt to create roll torque
	// Positive roll (right wing down) = right motor tilts right, left motor tilts left
	if (_tilts.count() >= 4 && (arm_y_right > FLT_EPSILON) && (arm_y_left > FLT_EPSILON)) {
		const float roll_gain = 1.0f;
		tilt_right = -tau_x * roll_gain;  // Negative: tilt right for positive roll
		tilt_left  =  tau_x * roll_gain;  // Positive: tilt left for positive roll
	}

	// Get tilt constraints from configuration
	const float tilt_min = (_tilts.count() > 0) ? _tilts.config(0).min_angle : -M_PI_4_F;
	const float tilt_max = (_tilts.count() > 0) ? _tilts.config(0).max_angle : M_PI_4_F;

	// Apply side motor thrusts (constant for now, could be modulated)
	for (int i = 0; i < SIDE_MOTORS_NUM; ++i) {
		actuator_sp(_first_side_idx + i) = math::constrain(
			side_motor_base_thrust,
			actuator_min(_first_side_idx + i),
			actuator_max(_first_side_idx + i)
		);
	}

	// Apply tilt setpoints
	// Tilt servo indices: _first_tilt_idx + 0..3 correspond to motors 2..5
	actuator_sp(_first_tilt_idx + 0) = math::constrain(tilt_front, tilt_min, tilt_max);
	actuator_sp(_first_tilt_idx + 1) = math::constrain(tilt_right, tilt_min, tilt_max);
	actuator_sp(_first_tilt_idx + 2) = math::constrain(tilt_back, tilt_min, tilt_max);
	actuator_sp(_first_tilt_idx + 3) = math::constrain(tilt_left, tilt_min, tilt_max);
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
