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

#include <inttypes.h>

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

float ActuatorEffectivenessIfodrone::edfMin() const
{
	// Never above the idle trim: the trim is where an EDF rests without any lateral demand
	return math::constrain(_param_ifo_edf_min.get(), 0.f, edfTrim());
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

int ActuatorEffectivenessIfodrone::boosterIndex(int num_rotors)
{
	const int32_t param = _param_ifo_bst_rotor.get();

	if (param < 0) {
		return -1;
	}

	// The booster is slaved to the coax pair: it must exist, must not be one of the coax motors
	// and must not carry a tilt servo (its column is folded into the fixed coax columns)
	const bool valid = (num_rotors > COAX_BOTTOM + 1) && (param > COAX_BOTTOM) && (param < num_rotors)
			   && (_mc_motors.geometry().rotors[param].tilt_index < 0);

	if (!valid) {
		if (_booster_warned_param != param) {
			PX4_WARN("IFO_BST_ROTOR %" PRId32 " invalid (needs untilted rotor 2..%d), booster disabled",
				 param, num_rotors - 1);
			_booster_warned_param = param;
		}

		return -1;
	}

	return param;
}

float ActuatorEffectivenessIfodrone::boosterRatio() const
{
	// Developer mode: booster stays configured (motor test, actuator_test) but is commanded 0 in flight
	if (_param_ifo_devel.get()) {
		return 0.f;
	}

	return math::constrain(_param_ifo_bst_ratio.get(), 0.f, 10.f);
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
	const int motor_start_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool motors_ok = _mc_motors.addActuators(configuration);

	// Vertical booster: slaved to the coax mean, u_boost = g * (u_top + u_bottom) / 2 (updateSetpoint()).
	// Its wrench therefore enters through the coax columns, g/2 of it per coax motor, and its own
	// column is zeroed so the pseudo-inverse never allocates it. B * u stays the physical wrench, so
	// the allocator status (unallocated thrust/torque) remains correct. The whole column is folded,
	// not only Fz: a booster offset from the centroid or a non-zero KM is then attributed correctly.
	_booster_idx = motors_ok ? boosterIndex(num_rotors) : -1;

	if (_booster_idx >= 0) {
		auto &matrix = configuration.effectiveness_matrices[configuration.selected_matrix];
		const int col_boost = motor_start_idx + _booster_idx;
		const float half_ratio = 0.5f * boosterRatio();

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			const float boost = matrix(axis, col_boost);
			matrix(axis, motor_start_idx + COAX_TOP) += half_ratio * boost;
			matrix(axis, motor_start_idx + COAX_BOTTOM) += half_ratio * boost;
			matrix(axis, col_boost) = 0.f;
		}

		// Booster index is used as an actuator index in updateSetpoint()
		_booster_idx = col_boost;
	}

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
		_servo_rotor[i] = rotor_idx;
		_servo_rp_unit[i].setZero();
		_servo_thrust_lin[i] = 0.f;

		if (rotor_idx >= 0) {
			const auto &rotor = _mc_motors.geometry().rotors[rotor_idx];
			// Single source of hinge sign (gz ENU vs FRD): never hard-code it here
			const Vector3f hinge = ActuatorEffectivenessRotors::hingeAxisForBase(_mc_motors.baseAxis(rotor_idx));
			const float dangle_dcmd = (_tilts.config(i).max_angle - _tilts.config(i).min_angle) / 2.f;
			// _lin_point is clipped to [0, 1], so the scale stays within [1, 1/edf_trim] times its trim value
			const float edf_thrust = math::max(_lin_point(rotor_idx), edf_trim);

			const Vector3f dthrust_unit = hinge.cross(rotor.axis) * (rotor.thrust_coef * dangle_dcmd);
			const Vector3f dtorque_unit = rotor.position.cross(dthrust_unit);
			dtorque = dtorque_unit * edf_thrust;
			_servo_rp_unit[i] = Vector2f(dtorque_unit(0), dtorque_unit(1));
			_servo_thrust_lin[i] = edf_thrust;

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
	auto &matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	if (torque_ref > FLT_EPSILON) {
		matrix.row(ControlAxis::ROLL) /= torque_ref;
		matrix.row(ControlAxis::PITCH) /= torque_ref;

		for (int i = 0; i < _tilts.count(); ++i) {
			_servo_rp_unit[i] /= torque_ref;
		}
	}

	// Cache side-EDF roll/pitch columns and opposing pairs for updateSetpoint(). Pairs are found from
	// the untilted base axes, so they do not depend on the current operating point.
	for (int i = 0; i < MAX_ROTORS; ++i) {
		_edf_pair[i] = -1;
		_edf_rp[i].setZero();
	}

	for (int i = 0; i < num_rotors && i < MAX_ROTORS; ++i) {
		if (!isSideEdf(i)) {
			continue;
		}

		_edf_rp[i] = Vector2f(matrix(ControlAxis::ROLL, motor_start_idx + i), matrix(ControlAxis::PITCH, motor_start_idx + i));

		for (int j = 0; j < num_rotors && j < MAX_ROTORS; ++j) {
			if (j != i && isSideEdf(j) && (_mc_motors.baseAxis(i).dot(_mc_motors.baseAxis(j)) < -0.9f)) {
				_edf_pair[i] = j;
				break;
			}
		}
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

	if (_param_ifo_ca_pair.get()) {
		// The pseudo-inverse splits a lateral force symmetrically around the EDF trim, so a small force
		// can drive one EDF of the pair to 0: its servo then has no torque authority although the
		// matrix assumes it does, and the clipped differential also takes the EDF pitch/roll lever
		// torque with it. Fix the EDF pair first, then re-solve the servos with the real EDF thrust.
		const ActuatorVector edf_sp_allocated = actuator_sp;
		desaturateEdfPairs(actuator_sp, actuator_min, actuator_max, edfMin());
		redistributeServoTorque(edf_sp_allocated, actuator_sp, actuator_min, actuator_max, hover_angle_rad);
	}

	// Reduce servo authority around the hover angle so gain changes never move the neutral tilt
	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = _first_tilt_idx + i;
		const float trim = tiltTrim(i, hover_angle_rad);
		actuator_sp(actuator_idx) = trim + tilt_gain * (actuator_sp(actuator_idx) - trim);
	}

	// Booster: thrust-priority desaturation of the coax pair, then the booster follows the coax mean.
	if (_booster_idx >= 0 && _booster_idx < NUM_ACTUATORS) {
		auto finite = [&](int i) { return PX4_ISFINITE(actuator_sp(i)) ? actuator_sp(i) : 0.f; };
		auto clipped = [&](int i, float sp) {
			return (actuator_min(i) < actuator_max(i)) ? math::constrain(sp, actuator_min(i), actuator_max(i)) : sp;
		};

		const float coax_min = math::max(actuator_min(COAX_TOP), actuator_min(COAX_BOTTOM));
		const float coax_max = math::min(actuator_max(COAX_TOP), actuator_max(COAX_BOTTOM));
		float coax_mean = 0.5f * (finite(COAX_TOP) + finite(COAX_BOTTOM));
		float coax_diff = 0.5f * (finite(COAX_TOP) - finite(COAX_BOTTOM));

		if (coax_min < coax_max) {
			coax_mean = math::constrain(coax_mean, coax_min, coax_max);
			const float diff_max = math::min(coax_mean - coax_min, coax_max - coax_mean);
			coax_diff = math::constrain(coax_diff, -diff_max, diff_max);
		}

		actuator_sp(COAX_TOP) = coax_mean + coax_diff;
		actuator_sp(COAX_BOTTOM) = coax_mean - coax_diff;
		actuator_sp(_booster_idx) = clipped(_booster_idx, boosterRatio() * coax_mean);
	}

	// Next cycle's matrix is linearized around the command actually driven (after gains, clipped to the
	// actuator range), low-pass filtered with IFO_CA_LIN_TAU
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

void ActuatorEffectivenessIfodrone::desaturateEdfPairs(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max, float edf_min) const
{
	const int num_rotors = math::min(_mc_motors.geometry().num_rotors, MAX_ROTORS);

	for (int i = 0; i < num_rotors; ++i) {
		const int j = _edf_pair[i];

		// Each pair once; rotor index == actuator index (motors are the first actuators)
		if (j <= i || j >= NUM_ACTUATORS) {
			continue;
		}

		const float lo = math::max(edf_min, math::max(actuator_min(i), actuator_min(j)));
		const float hi = math::min(actuator_max(i), actuator_max(j));

		if (!(hi > lo)) {
			continue;
		}

		const float a = PX4_ISFINITE(actuator_sp(i)) ? actuator_sp(i) : lo;
		const float b = PX4_ISFINITE(actuator_sp(j)) ? actuator_sp(j) : lo;

		// The differential is the lateral force (and the PZ lever torque): keep it, limited to what the
		// pair can produce with both EDFs at or above IFO_EDF_MIN. The common mode cancels out, so it is
		// free to move: shift it so the low EDF stays at the minimum (servo authority on both sides).
		const float d = math::constrain(a - b, lo - hi, hi - lo);
		const float half_abs_d = 0.5f * fabsf(d);
		const float m = math::constrain(0.5f * (a + b), lo + half_abs_d, hi - half_abs_d);

		actuator_sp(i) = m + 0.5f * d;
		actuator_sp(j) = m - 0.5f * d;
	}
}

void ActuatorEffectivenessIfodrone::redistributeServoTorque(const ActuatorVector &edf_sp_allocated,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max,
		float hover_angle_rad) const
{
	const int num_rotors = math::min(_mc_motors.geometry().num_rotors, MAX_ROTORS);
	const int num_tilts = math::min(_tilts.count(), MAX_TILTS);

	auto finite = [](float v, float fallback) { return PX4_ISFINITE(v) ? v : fallback; };

	// Roll/pitch torque the allocation intended from the servos (with the matrix' EDF thrust), plus the
	// EDF torque lost or gained by the pair desaturation, which the servos have to make up for
	Vector2f torque{};
	float trim[MAX_TILTS] {};

	for (int i = 0; i < num_tilts; ++i) {
		const int col = _first_tilt_idx + i;

		if (col >= NUM_ACTUATORS) {
			return;
		}

		trim[i] = tiltTrim(i, hover_angle_rad);
		torque += _servo_rp_unit[i] * (_servo_thrust_lin[i] * (finite(actuator_sp(col), trim[i]) - trim[i]));
	}

	for (int r = 0; r < num_rotors; ++r) {
		if (isSideEdf(r)) {
			torque += _edf_rp[r] * (finite(edf_sp_allocated(r), actuator_sp(r)) - finite(actuator_sp(r), 0.f));
		}
	}

	// Servo columns at the commanded EDF thrust; a servo on an EDF without thrust has no authority
	Vector2f g[MAX_TILTS] {};
	float delta[MAX_TILTS] {};
	float delta_min[MAX_TILTS] {};
	float delta_max[MAX_TILTS] {};
	bool active[MAX_TILTS] {};
	bool any_free = false;
	bool enabled[MAX_TILTS] {};

	for (int i = 0; i < num_tilts; ++i) {
		const int col = _first_tilt_idx + i;
		const int r = _servo_rotor[i];
		const float thrust = (r >= 0 && r < num_rotors) ? math::max(finite(actuator_sp(r), 0.f), 0.f) : 0.f;
		g[i] = _servo_rp_unit[i] * thrust;
		delta_min[i] = actuator_min(col) - trim[i];
		delta_max[i] = actuator_max(col) - trim[i];
		enabled[i] = actuator_min(col) < actuator_max(col);
		active[i] = enabled[i] && (g[i].norm_squared() > 1e-12f);
		any_free = any_free || active[i];
	}

	if (!any_free) {
		return;
	}

	// Minimum-norm solution of G * delta = torque; a servo that hits its limit is fixed there and the
	// rest of the torque is re-solved with the remaining servos (it does not just get clipped away)
	for (int iter = 0; iter < num_tilts; ++iter) {
		Vector2f remaining = torque;
		matrix::SquareMatrix<float, 2> gram{};

		for (int i = 0; i < num_tilts; ++i) {
			if (active[i]) {
				gram(0, 0) += g[i](0) * g[i](0);
				gram(0, 1) += g[i](0) * g[i](1);
				gram(1, 1) += g[i](1) * g[i](1);

			} else {
				remaining -= g[i] * delta[i];
			}
		}

		gram(1, 0) = gram(0, 1);

		// Regularized per axis: an axis without any servo authority left stays unallocated instead of
		// blowing up (lambda grows only along a direction no servo column has a component in)
		gram(0, 0) += 1e-5f * gram(0, 0) + 1e-9f;
		gram(1, 1) += 1e-5f * gram(1, 1) + 1e-9f;
		const float det = gram(0, 0) * gram(1, 1) - gram(0, 1) * gram(1, 0);
		const Vector2f lambda((gram(1, 1) * remaining(0) - gram(0, 1) * remaining(1)) / det,
				      (gram(0, 0) * remaining(1) - gram(1, 0) * remaining(0)) / det);

		int worst = -1;
		float worst_excess = 0.f;

		for (int i = 0; i < num_tilts; ++i) {
			if (!active[i]) {
				continue;
			}

			delta[i] = g[i].dot(lambda);
			const float excess = math::max(delta[i] - delta_max[i], delta_min[i] - delta[i]);

			if (excess > worst_excess) {
				worst_excess = excess;
				worst = i;
			}
		}

		if (worst < 0) {
			break;
		}

		delta[worst] = math::constrain(delta[worst], delta_min[worst], delta_max[worst]);
		active[worst] = false;

		bool still_free = false;

		for (int i = 0; i < num_tilts; ++i) {
			still_free = still_free || active[i];
		}

		if (!still_free) {
			break;
		}
	}

	for (int i = 0; i < num_tilts; ++i) {
		if (enabled[i]) {
			actuator_sp(_first_tilt_idx + i) = trim[i] + math::constrain(delta[i], delta_min[i], delta_max[i]);
		}
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
