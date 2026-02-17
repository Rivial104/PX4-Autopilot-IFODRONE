#include "ActuatorEffectivenessIfodrone.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

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
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// All 6 motors (2 main + 4 side)
	configuration.selected_matrix = 0;

	// IFODRONE: main motors always use differential thrust for yaw.
	// Side motor tilts handle roll/pitch only (not yaw).
	_mc_rotors.enableYawByDifferentialThrust(true);

	const bool motors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts for side motors
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Override tilt effectiveness for IFODRONE roll/pitch stabilization.
	// Standard PX4 tilt code only models Yaw/Pitch. IFODRONE needs:
	//   - Tilt 0 (front motor at +X): pitch torque
	//   - Tilt 1 (right motor at +Y): roll torque
	//   - Tilt 2 (back motor at -X): pitch torque (opposite sign)
	//   - Tilt 3 (left motor at -Y): roll torque (opposite sign)
	// NOTE: If drone stabilizes in wrong direction, flip all pitch/roll signs.
	auto &eff = configuration.effectiveness_matrices[configuration.selected_matrix];

	for (int i = 0; i < _tilts.count(); ++i) {
		const int idx = _first_tilt_idx + i;
		eff(ControlAxis::ROLL, idx)  = 0.0f;
		eff(ControlAxis::PITCH, idx) = 0.0f;
		eff(ControlAxis::YAW, idx)   = 0.0f;
	}

	if (_tilts.count() >= 4) {
		eff(ControlAxis::PITCH, _first_tilt_idx + 0) =  1.0f;  // front tilt → pitch
		eff(ControlAxis::ROLL,  _first_tilt_idx + 1) =  1.0f;  // right tilt → roll
		eff(ControlAxis::PITCH, _first_tilt_idx + 2) = -1.0f;  // back  tilt → pitch (opposite)
		eff(ControlAxis::ROLL,  _first_tilt_idx + 3) = -1.0f;  // left  tilt → roll  (opposite)
	}

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

	// const float trim0 = (_tilts.count() > 0) ? _tilt_offsets(_first_tilt_idx + 0) : NAN;
	// const float trim1 = (_tilts.count() > 1) ? _tilt_offsets(_first_tilt_idx + 1) : NAN;
	// const float trim2 = (_tilts.count() > 2) ? _tilt_offsets(_first_tilt_idx + 2) : NAN;
	// const float trim3 = (_tilts.count() > 3) ? _tilt_offsets(_first_tilt_idx + 3) : NAN;

	// PX4_INFO("IFO CA matrix: rotors=%d tilts=%d first_tilt_idx=%d yaw_diff=%d trim=[%.3f %.3f %.3f %.3f]",
	// 	 _mc_rotors.geometry().num_rotors, _tilts.count(), _first_tilt_idx,
	// 	 !_mc_rotors.geometry().yaw_by_differential_thrust_disabled,
	// 	 (double)trim0, (double)trim1, (double)trim2, (double)trim3);

	return (motors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	(void)matrix_index;

	// Keep tilt neutral orientation centered at "up" without using static trim.
	actuator_sp += _tilt_offsets;

	// NaN guard for tilt actuators
	for (int i = 0; i < _tilts.count(); ++i) {
		const int idx = i + _first_tilt_idx;

		if (!PX4_ISFINITE(actuator_sp(idx))) {
			actuator_sp(idx) = _tilt_offsets(idx);
		}
	}

	// Tilts don't provide yaw on IFODRONE (main motors handle yaw via differential thrust)
	_yaw_tilt_saturation_flags.tilt_yaw_neg = false;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = false;

	static hrt_abstime last_dbg = 0;
	const hrt_abstime now = hrt_absolute_time();

	if (now - last_dbg > 1000000) {
		last_dbg = now;

		const float t0 = (_tilts.count() > 0) ? actuator_sp(_first_tilt_idx + 0) : NAN;
		const float t1 = (_tilts.count() > 1) ? actuator_sp(_first_tilt_idx + 1) : NAN;
		const float t2 = (_tilts.count() > 2) ? actuator_sp(_first_tilt_idx + 2) : NAN;
		const float t3 = (_tilts.count() > 3) ? actuator_sp(_first_tilt_idx + 3) : NAN;

		PX4_INFO("IFO CA: roll=%.3f pitch=%.3f tz=%.3f tilt=[%.3f %.3f %.3f %.3f]",
			 (double)control_sp(ControlAxis::ROLL), (double)control_sp(ControlAxis::PITCH),
			 (double)control_sp(ControlAxis::THRUST_Z),
			 (double)t0, (double)t1, (double)t2, (double)t3);
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
