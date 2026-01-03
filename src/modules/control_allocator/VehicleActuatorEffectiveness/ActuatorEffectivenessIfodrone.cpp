#include "ActuatorEffectivenessIfodrone.hpp"

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _main_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, false),
	  _side_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
}


bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Main motors
	_main_rotors.enableYawByDifferentialThrust(true);
	const bool main_rotors_added_successfully = _main_rotors.addActuators(configuration);

	// Side motors
	_side_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	const bool side_rotors_added_successfully = _side_rotors.addActuators(configuration);

	// Tilts
	_tilts.updateTorqueSign(_side_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset such that tilts point upwards when control input == 0 (trim is 0 if min_angle == -max_angle).
	// Note that we don't set configuration.trim here, because in the case of trim == +-1, yaw is always saturated
	// and reduced to 0 with the sequential desaturation method. Instead we add it after.
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	return (main_rotors_added_successfully && side_rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	actuator_sp.setZero();

	const float Fz_des = control_sp(5);

	if(MAIN_MOTORS_NUM > 0) {
		// Distribute collective thrust equally to main rotors
		const float thrust_per_main_motor = Fz_des / static_cast<float>(MAIN_MOTORS_NUM);

		for (int i = 0; i < MAIN_MOTORS_NUM; ++i) {
			actuator_sp(_first_main_idx + i) = math::constrain(thrust_per_main_motor, actuator_min(_first_main_idx + i), actuator_max(_first_main_idx + i));
		}

		if (MAIN_MOTORS_NUM == 2) {
			float yaw_cmd = control_sp(2);
			actuator_sp(0) += yaw_cmd / 2.f; // +1
			actuator_sp(1) -= yaw_cmd / 2.f; // -1
			actuator_sp(0) = math::constrain(actuator_sp(0), actuator_min(0), actuator_max(0));
			actuator_sp(1) = math::constrain(actuator_sp(1), actuator_min(1), actuator_max(1));
		}
	}

	const Vector2f F_xy_des(control_sp(3), control_sp(4));
	const float fx = F_xy_des(0);
	const float fy = F_xy_des(1);
	const float F_xy_norm = F_xy_des.norm();

	const float F_xy_max = 4.f * actuator_max(_first_side_idx); // first tilting motor index



	float tilt_angle = 0.f;
	if (F_xy_norm > FLT_MIN)
	{
		auto &config = _tilts.config(0);
		// 0 = poziom, +pi/2 = pion
		tilt_angle = math::constrain(
			acosf(math::constrain(F_xy_norm / F_xy_max, 0.f, 1.f)),
			config.min_angle,
			config.max_angle
		);
	}

	float tau_x = control_sp(0);
	float tau_y = control_sp(1);
	float tau_z = control_sp(2);

	for (int i = 0; i < _tilts.count(); ++i) {
		auto &geometry = _side_rotors.geometry().rotors[i];
		const float dx = geometry.position(0);
		const float dy = geometry.position(1);

		// actuator_sp(tilt_base + i) = math::constrain(
		// 	tilt_angle,
		// 	actuator_min(tilt_base + i),
		// 	actuator_max(tilt_base + i)
		// );

		// float signX = (i == 0) ? 1.f : (i == 2) ? -1.f : 0.f; // front/back
		// float signY = (i == 1) ? 1.f : (i == 3) ? -1.f : 0.f; // right/left

		float Fx_i = fx * 0.25f + tau_y / (4.f * dx); // roll → różnicowanie
		float Fy_i = fy * 0.25f - tau_x / (4.f * dy); // pitch → różnicowanie
		float Fz_i = tau_z / 4.f; // yaw przez boczne silniki

		// całkowity ciąg = sqrt(Fx_i^2 + Fy_i^2 + Fz_i^2)
		float T = sqrt(Fx_i*Fx_i + Fy_i*Fy_i + Fz_i*Fz_i);

		actuator_sp(_first_side_idx + i) = math::constrain(T, actuator_min(_first_side_idx + i), actuator_max(_first_side_idx + i));

		actuator_sp(_first_tilt_idx + i) = tilt_angle;
	}
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
