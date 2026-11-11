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
 *   Servo columns : torque Jacobian d(torque)/d(servo command), evaluated at
 *                   the current tilt angle and scaled by the current EDF
 *                   thrust (tilt authority ∝ EDF thrust). Force rows are zero
 *                   so Fz demand is never allocated to collective tilt.
 *
 * Linearization point: the clipped allocation result, low-pass filtered with
 * IFO_CA_LIN_TAU (see updateSetpoint()).
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

	if (delta_angle <= FLT_EPSILON) {
		return 0.f;
	}

	return math::constrain(2.f * (hover_angle_rad - _tilts.config(tilt_index).min_angle) / delta_angle - 1.f, -1.f, 1.f);
}

float ActuatorEffectivenessIfodrone::hoverAngleRad() const
{
	return math::radians(_param_ifo_tilt_hover.get());
}

float ActuatorEffectivenessIfodrone::edfTrim() const
{
	return math::constrain(_param_ifo_edf_trim.get(), 0.f, 0.9f);
}

bool ActuatorEffectivenessIfodrone::isSideEdf(int rotor_index) const
{
	return _mc_motors.geometry().rotors[rotor_index].tilt_index >= 0;
}

int ActuatorEffectivenessIfodrone::rotorForTilt(int tilt_index, int num_rotors) const
{
	for (int r = 0; r < num_rotors; ++r) {
		if (_mc_motors.geometry().rotors[r].tilt_index == tilt_index) {
			return r;
		}
	}

	return -1;
}

void ActuatorEffectivenessIfodrone::ensureLinPoint(int num_rotors)
{
	// Before the first allocation there is no setpoint: linearize around the hover
	// operating point (EDFs at trim, tilts at the hover angle).
	if (!_lin_point_valid) {
		_lin_point = hoverOperatingPoint(num_rotors);
		_lin_point_valid = true;
	}
}

bool
ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason /*external_update*/)
{
	configuration.selected_matrix = 0;

	_mc_motors.enableYawByDifferentialThrust(true);
	_mc_motors.enablePropellerTorqueNonUpwards(false);

	const float hover_angle_rad = hoverAngleRad();
	const float edf_trim = edfTrim();
	const int num_rotors = _mc_motors.geometry().num_rotors;
	_first_tilt_idx = num_rotors;

	ensureLinPoint(num_rotors);

	// Rotate the side-EDF axes to the current tilt angles, then add the motor
	// columns (3D force + torque at the current operating point)
	_mc_motors.updateAxisFromTiltSetpoints(_tilts, _lin_point, _first_tilt_idx);
	const bool motors_ok = _mc_motors.addActuators(configuration);

	// Side EDFs are unidirectional: trim is the shared idle baseline, allocation is differential around it
	for (int i = 0; i < num_rotors; ++i) {
		if (isSideEdf(i)) {
			configuration.trim[configuration.selected_matrix](i) = edf_trim;
		}
	}

	// Tilt servo columns: d(torque)/d(command) at the current operating point.
	// d(axis)/d(angle) = hinge x axis, so dF/ds = ct*m*(hinge x axis)*(max-min)/2, dtau/ds = r x dF/ds.
	// Force rows are intentionally ZERO: Fz demand must never be realized by collective tilt
	// (asymmetries torque the body to roll 180 deg); Fz belongs to the coax pair.
	float torque_ref = 0.f;   // |servo torque| at idle EDF thrust, roll/pitch row unit

	for (int i = 0; i < _tilts.count(); ++i) {
		// A servo column is always added (zero torque if no rotor maps to it) to keep actuator indexing
		const int rotor_idx = rotorForTilt(i, num_rotors);
		Vector3f dtorque{};

		if (rotor_idx >= 0) {
			const auto &rotor = _mc_motors.geometry().rotors[rotor_idx];
			// Single source of hinge sign (gz ENU vs FRD): never hard-code it here
			const Vector3f hinge = ActuatorEffectivenessRotors::hingeAxisForBase(_mc_motors.baseAxis(rotor_idx));
			const float dangle_dcmd = (_tilts.config(i).max_angle - _tilts.config(i).min_angle) / 2.f;
			// _lin_point is clipped to [0, 1], so the scale stays within [1, 1/edf_trim] times its trim value
			const float edf_thrust = math::max(_lin_point(rotor_idx), edf_trim);

			const Vector3f dthrust = hinge.cross(rotor.axis) * (rotor.thrust_coef * edf_thrust * dangle_dcmd);
			dtorque = rotor.position.cross(dthrust);

			// Floored: with IFO_EDF_TRIM = 0 the reference would vanish and roll/pitch get zeroed (see below)
			torque_ref = math::max(torque_ref,
					       rotor.position.norm() * rotor.thrust_coef * math::max(edf_trim, EDF_TRIM_REF_MIN)
					       * fabsf(dangle_dcmd));
		}

		const int actuator_idx = configuration.addActuator(ActuatorType::SERVOS, dtorque, Vector3f{});

		if (actuator_idx >= 0) {
			// Allocation zero = hover tilt angle
			configuration.trim[configuration.selected_matrix](actuator_idx) = tiltTrim(i, hover_angle_rad);
		}
	}

	// Rescale roll/pitch rows to normalized units by torque_ref, FIXED at the trim operating point (keeps
	// the live EDF-thrust scaling of servo columns). In physical CT units entries (~0.007) fall below the
	// ControlAllocator weak-row threshold (0.05) and roll/pitch would be zeroed (frozen servos, drift).
	if (torque_ref > FLT_EPSILON) {
		auto &matrix = configuration.effectiveness_matrices[configuration.selected_matrix];
		matrix.row(ControlAxis::ROLL) /= torque_ref;
		matrix.row(ControlAxis::PITCH) /= torque_ref;
	}

	return motors_ok;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float, NUM_AXES> &/*control_sp*/,
		int /*matrix_index*/, ActuatorVector &actuator_sp,
		const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	const float hover_angle_rad = hoverAngleRad();
	const float edf_trim = edfTrim();
	const float edf_gain = math::constrain(_param_ifo_edf_gain.get(), 0.f, 1.f);
	const float tilt_gain = math::constrain(_param_ifo_tilt_gain.get(), 0.f, 1.f);
	const int num_rotors = _mc_motors.geometry().num_rotors;

	// Reduce side-EDF authority around the shared idle trim (fixed coax motors untouched)
	for (int i = 0; i < num_rotors; ++i) {
		if (isSideEdf(i)) {
			actuator_sp(i) = edf_trim + edf_gain * (actuator_sp(i) - edf_trim);
		}
	}

	// Reduce servo authority around the hover angle so gain changes never move the neutral tilt
	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;
		const float trim = tiltTrim(i, hover_angle_rad);
		actuator_sp(actuator_idx) = trim + tilt_gain * (actuator_sp(actuator_idx) - trim);
	}

	// Next cycle's matrix is linearized around the command actually driven (after gains, clipped to the
	// actuator range), low-pass filtered with IFO_CA_LIN_TAU. Feeding back the raw previous allocation is
	// an algebraic loop with a negative eigenvalue: ringing at half the allocation rate and a full-range
	// servo limit cycle at large hover tilt (2026-09-03 flights).
	const hrt_abstime now = hrt_absolute_time();
	const float tau = math::max(_param_ifo_ca_lin_tau.get(), 0.f);
	float alpha = 1.f;

	if (_lin_point_last_update != 0 && tau > FLT_EPSILON) {
		const float dt = math::constrain((now - _lin_point_last_update) * 1e-6f, 0.0002f, 0.02f);
		alpha = dt / (tau + dt);
	}

	_lin_point_last_update = now;

	ensureLinPoint(num_rotors);

	const int num_actuators = _first_tilt_idx + _tilts.count();

	for (int i = 0; i < num_actuators && i < NUM_ACTUATORS; ++i) {
		const float sp = actuator_sp(i);

		// A non-finite setpoint must not poison the operating point for good
		if (!PX4_ISFINITE(sp)) {
			continue;
		}

		const float sp_clipped = (actuator_min(i) < actuator_max(i)) ? math::constrain(sp, actuator_min(i), actuator_max(i)) : sp;
		_lin_point(i) += alpha * (sp_clipped - _lin_point(i));
	}
}

ActuatorEffectiveness::ActuatorVector ActuatorEffectivenessIfodrone::hoverOperatingPoint(int num_rotors) const
{
	const float hover_angle_rad = hoverAngleRad();
	const float edf_trim = edfTrim();
	ActuatorVector point{};

	for (int i = 0; i < num_rotors && i < NUM_ACTUATORS; ++i) {
		if (isSideEdf(i)) {
			point(i) = edf_trim;
		}
	}

	for (int i = 0; i < _tilts.count(); ++i) {
		const int idx = num_rotors + i;

		if (idx < NUM_ACTUATORS) {
			point(idx) = tiltTrim(i, hover_angle_rad);
		}
	}

	return point;
}
