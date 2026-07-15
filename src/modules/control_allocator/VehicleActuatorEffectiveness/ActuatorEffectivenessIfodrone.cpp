/**
 * @file ActuatorEffectivenessIfodrone.cpp
 *
 * IFODRONE control allocator effectiveness.
 *
 * The matrix is state-dependent and recomputed every cycle (the IFODRONE
 * source runs on the high-rate update path in ControlAllocator):
 *
 *   Motor columns : from rotor geometry, with the side-EDF axes rotated to the
 *                   current tilt angles — a tilted EDF correctly contributes
 *                   both lateral and vertical force.
 *   Servo columns : full wrench Jacobian d(torque, thrust)/d(servo command),
 *                   evaluated at the current tilt angle and scaled by the
 *                   current EDF thrust (tilt authority ∝ EDF thrust).
 *
 * Trims: the side EDFs idle at IFO_EDF_TRIM (opposing pairs allocate
 * differentially around it), and the tilt servos rest at IFO_TILT_HOVER
 * degrees, so zero demand returns all actuators to the hover operating point
 * (absolute allocation, no drift).
 */

#include "ActuatorEffectivenessIfodrone.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_motors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
}

float ActuatorEffectivenessIfodrone::tiltTrim(int tilt_index, float hover_angle_rad) const
{
	// Servo command in [-1, 1] mapping to the hover tilt angle
	const float delta_angle = _tilts.config(tilt_index).max_angle - _tilts.config(tilt_index).min_angle;

	if (delta_angle > FLT_EPSILON) {
		return math::constrain(2.f * (hover_angle_rad - _tilts.config(tilt_index).min_angle) / delta_angle - 1.f,
				       -1.f, 1.f);
	}

	return 0.f;
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason /*external_update*/)
{
	configuration.selected_matrix = 0;

	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	const float hover_angle_rad = math::radians(_param_ifo_tilt_hover.get());
	const float edf_trim = math::constrain(_param_ifo_edf_trim.get(), 0.f, 0.9f);
	const int num_rotors = _mc_motors.geometry().num_rotors;
	_first_tilt_idx = num_rotors;

	// Before the first allocation there is no setpoint yet: linearize around the
	// hover operating point (EDFs at trim, tilts at the hover angle).
	if (!_last_actuator_sp_valid) {
		_last_actuator_sp.setZero();

		for (int i = 0; i < num_rotors; ++i) {
			if (_mc_motors.geometry().rotors[i].tilt_index >= 0) {
				_last_actuator_sp(i) = edf_trim;
			}
		}

		for (int i = 0; i < _tilts.count(); ++i) {
			_last_actuator_sp(_first_tilt_idx + i) = tiltTrim(i, hover_angle_rad);
		}

		_last_actuator_sp_valid = true;
	}

	// Rotate the side-EDF axes to the current tilt angles, then add the motor
	// columns (3D force + torque at the current operating point)
	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _last_actuator_sp, _first_tilt_idx);
	const bool motors_ok = _mc_motors.addActuators(configuration);

	// Side EDFs are unidirectional. Trim sets the shared idle baseline for each
	// opposing pair; the allocation is differential around it.
	for (int i = 0; i < num_rotors; ++i) {
		if (_mc_motors.geometry().rotors[i].tilt_index >= 0) {
			configuration.trim[configuration.selected_matrix](i) = edf_trim;
		}
	}

	// Tilt servo columns: d(wrench)/d(command) at the current operating point.
	// d(axis)/d(angle) = hinge × axis, so for EDF thrust m and command range
	// [min, max]: dF/ds = ct·m·(hinge × axis)·(max−min)/2, dτ/ds = r × dF/ds.
	for (int i = 0; i < _tilts.count(); ++i) {
		int rotor_idx = -1;

		for (int r = 0; r < num_rotors; ++r) {
			if (_mc_motors.geometry().rotors[r].tilt_index == i) {
				rotor_idx = r;
				break;
			}
		}

		Vector3f dtorque{};
		Vector3f dthrust{};

		if (rotor_idx >= 0) {
			const auto &rotor = _mc_motors.geometry().rotors[rotor_idx];
			const Vector3f hinge = ActuatorEffectivenessRotors::hingeAxisForBase(_mc_motors.baseAxis(rotor_idx));
			const float dangle_dcmd = (_tilts.config(i).max_angle - _tilts.config(i).min_angle) / 2.f;
			const float edf_thrust = math::max(_last_actuator_sp(rotor_idx), edf_trim);

			dthrust = hinge.cross(rotor.axis) * (rotor.thrust_coef * edf_thrust * dangle_dcmd);
			dtorque = rotor.position.cross(dthrust);
		}

		const int actuator_idx = configuration.addActuator(ActuatorType::SERVOS, dtorque, dthrust);

		if (actuator_idx >= 0) {
			// Allocation zero = hover tilt angle
			configuration.trim[configuration.selected_matrix](actuator_idx) = tiltTrim(i, hover_angle_rad);
		}
	}

	return motors_ok;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &/*control_sp*/,
		int /*matrix_index*/, ActuatorVector &actuator_sp,
		const ActuatorVector &/*actuator_min*/, const ActuatorVector &/*actuator_max*/)
{
	const float hover_angle_rad = math::radians(_param_ifo_tilt_hover.get());
	const float edf_trim = math::constrain(_param_ifo_edf_trim.get(), 0.f, 0.9f);
	const float edf_gain = math::constrain(_param_ifo_edf_gain.get(), 0.f, 1.f);
	const float tilt_gain = math::constrain(_param_ifo_tilt_gain.get(), 0.f, 1.f);
	const int num_rotors = _mc_motors.geometry().num_rotors;

	// Reduce side-EDF authority around the shared idle trim. Fixed coaxial motors
	// have no tilt index and are intentionally left untouched.
	for (int i = 0; i < num_rotors; ++i) {
		if (_mc_motors.geometry().rotors[i].tilt_index >= 0) {
			actuator_sp(i) = edf_trim + edf_gain * (actuator_sp(i) - edf_trim);
		}
	}

	// Reduce servo authority around the hover angle so gain changes never move
	// the neutral tilt position.
	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;
		const float trim = tiltTrim(i, hover_angle_rad);
		actuator_sp(actuator_idx) = trim + tilt_gain * (actuator_sp(actuator_idx) - trim);
	}

	// Cache the command actually sent: next cycle's state-dependent matrix must
	// be linearized at the reduced EDF thrust and tilt angles.
	_last_actuator_sp = actuator_sp;
}
