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
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !_matrix_update_needed) {
		return false;
	}

	// All 6 motors (2 main + 4 side)
	configuration.selected_matrix = 0;

	// IFODRONE: main motors always use differential thrust for yaw.
	// Side motor tilts handle roll/pitch only (not yaw).
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());

	// Re-linearize rotor axes around the latest tilt actuator positions.
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && _has_last_actuator_sp
	    && _first_tilt_idx >= 0) {
		_mc_rotors.updateAxisFromTiltSetpoints(_tilts, _last_actuator_sp, _first_tilt_idx);
	}

	const bool motors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts for side motors
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	auto &eff = configuration.effectiveness_matrices[configuration.selected_matrix];

	for (int i = 0; i < _tilts.count(); ++i) {
		const int idx = _first_tilt_idx + i;
		eff(ControlAxis::ROLL, idx)  = 0.0f;
		eff(ControlAxis::PITCH, idx) = 0.0f;
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

	_matrix_update_needed = false;

	return (motors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	(void)control_sp;
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

	// Keep diagnostics and request matrix refresh when tilt commands change.
	bool tilt_changed = !_has_last_actuator_sp;
	const int num_tilts = _tilts.count();

	for (int i = 0; i < num_tilts; ++i) {
		const int idx = i + _first_tilt_idx;

		if (_has_last_actuator_sp && fabsf(actuator_sp(idx) - _last_actuator_sp(idx)) > TILT_MATRIX_UPDATE_THRESHOLD) {
			tilt_changed = true;
		}
	}

	const int num_used_actuators = (_first_tilt_idx >= 0) ? math::min(NUM_ACTUATORS, _first_tilt_idx + num_tilts) : 0;
	_sat_upper_count = 0;
	_sat_lower_count = 0;

	for (int i = 0; i < num_used_actuators; ++i) {
		const float sp = actuator_sp(i);

		if (!PX4_ISFINITE(sp)) {
			continue;
		}

		if (sp >= actuator_max(i) - FLT_EPSILON) {
			++_sat_upper_count;

		} else if (sp <= actuator_min(i) + FLT_EPSILON) {
			++_sat_lower_count;
		}
	}

	_last_actuator_sp = actuator_sp;
	_has_last_actuator_sp = true;

	if (tilt_changed) {
		_matrix_update_needed = true;
	}

	// static hrt_abstime last_dbg = 0;
	// const hrt_abstime now = hrt_absolute_time();

	// if (now - last_dbg > 1000000) {
	// 	last_dbg = now;

		// const float t0 = (_tilts.count() > 0) ? actuator_sp(_first_tilt_idx + 0) : NAN;
		// const float t1 = (_tilts.count() > 1) ? actuator_sp(_first_tilt_idx + 1) : NAN;
		// const float t2 = (_tilts.count() > 2) ? actuator_sp(_first_tilt_idx + 2) : NAN;
		// const float t3 = (_tilts.count() > 3) ? actuator_sp(_first_tilt_idx + 3) : NAN;

		// PX4_INFO("IFO CA: roll=%.3f pitch=%.3f tz=%.3f tilt=[%.3f %.3f %.3f %.3f]",
		// 	 (double)control_sp(ControlAxis::ROLL), (double)control_sp(ControlAxis::PITCH),
		// 	 (double)control_sp(ControlAxis::THRUST_Z),
		// 	 (double)t0, (double)t1, (double)t2, (double)t3);
	// }
}

void ActuatorEffectivenessIfodrone::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	(void)matrix_index;

	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	// const Vector3f unallocated_torque(status.unallocated_torque[0], status.unallocated_torque[1], status.unallocated_torque[2]);
	// const Vector3f unallocated_thrust(status.unallocated_thrust[0], status.unallocated_thrust[1], status.unallocated_thrust[2]);
	// const bool unallocated_large = unallocated_torque.norm() > UNALLOCATED_LOG_THRESHOLD
	// 			       || unallocated_thrust.norm() > UNALLOCATED_LOG_THRESHOLD;
	// const bool saturated = (_sat_upper_count > 0) || (_sat_lower_count > 0);
	// const hrt_abstime now = hrt_absolute_time();

	// if ((unallocated_large || saturated) && now - _last_diag_log > DIAG_LOG_INTERVAL_US) {
	// 	_last_diag_log = now;
	// 	PX4_WARN("IFO CA: unalloc_t=(%.3f %.3f %.3f) unalloc_f=(%.3f %.3f %.3f) sat(u=%d l=%d)",
	// 		 (double)status.unallocated_torque[0], (double)status.unallocated_torque[1], (double)status.unallocated_torque[2],
	// 		 (double)status.unallocated_thrust[0], (double)status.unallocated_thrust[1], (double)status.unallocated_thrust[2],
	// 		 _sat_upper_count, _sat_lower_count);
	// }
}
