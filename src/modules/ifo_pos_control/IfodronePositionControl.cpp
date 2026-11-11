#include "IfodronePositionControl.hpp"
#include "PositionControl/ControlMath.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;
using namespace ifodrone;

IfodronePositionControl::IfodronePositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
	_takeoff_status_pub.advertise();
}

IfodronePositionControl::~IfodronePositionControl()
{
	perf_free(_cycle_perf);
}

bool IfodronePositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	PX4_INFO("IFODRONE position controller initialized");
	return true;
}

void IfodronePositionControl::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		ModuleParams::updateParams();

		// Set gains on the PositionControl library
		_control.setPositionGains(Vector3f(_param_ifo_pos_xy_p.get(), _param_ifo_pos_xy_p.get(), _param_ifo_pos_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_ifo_vel_xy_p.get(), _param_ifo_vel_xy_p.get(), _param_ifo_vel_z_p.get()),
			Vector3f(_param_ifo_vel_xy_i.get(), _param_ifo_vel_xy_i.get(), _param_ifo_vel_z_i.get()),
			Vector3f(_param_ifo_vel_xy_d.get(), _param_ifo_vel_xy_d.get(), _param_ifo_vel_z_d.get()));

		_control.setHoverThrust(_param_ifo_thr_hover.get());
		_control.setThrustLimits(_param_ifo_thr_min.get(), _param_ifo_thr_max.get());
		// Apply the side-EDF limit before anti-windup.
		_control.setHorizontalThrustLimit(_param_ifo_thr_xy_max.get());

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_ifo_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_ifo_vel_z_p.get());
	}
}

void IfodronePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	runCycle();
	perf_end(_cycle_perf);
}

void IfodronePositionControl::runCycle()
{
	vehicle_local_position_s local_pos;

	if (!_local_pos_sub.update(&local_pos)) {
		return;
	}

	const float dt = math::constrain(((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
	_time_stamp_last_loop = local_pos.timestamp_sample;

	updateSubscriptions();

	const PositionControlStates states = buildStates(local_pos);
	updateSetpointSource(local_pos, states);

	const vehicle_control_mode_s &mode = _vehicle_control_mode;
	const bool pos_ctrl = mode.flag_multicopter_position_control_enabled;
	// Stabilized uses manual thrust despite the position-control flag being clear.
	const bool manual_stabilized = mode.flag_control_manual_enabled && mode.flag_control_attitude_enabled && !pos_ctrl;
	const bool active = (pos_ctrl && (mode.flag_control_manual_enabled
					  || _setpoint.timestamp >= _time_position_control_enabled)) || manual_stabilized;

	if (!active) {
		_takeoff.updateTakeoffState(mode.flag_armed, _vehicle_land_detected.landed, false, true,
					    local_pos.timestamp_sample);
		_control.resetIntegral();
		resetManualState();

	} else {
		const bool ok = mode.flag_control_manual_enabled ? runManual(local_pos, states, dt)
				: runAuto(local_pos, states, dt);

		if (!ok) {
			return;
		}

		publishSetpoints(states);
	}

	publishTakeoffStatus();
}

void IfodronePositionControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		const bool prev_pos_ctrl = _vehicle_control_mode.flag_multicopter_position_control_enabled;

		if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
			const bool pos_ctrl = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (!prev_pos_ctrl && pos_ctrl) {
				_time_position_control_enabled = _vehicle_control_mode.timestamp;

			} else if (prev_pos_ctrl && !pos_ctrl) {
				_setpoint = PositionControl::empty_trajectory_setpoint;
			}
		}
	}

	_vehicle_land_detected_sub.update(&_vehicle_land_detected);

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_q_att = Quatf(att.q);
		_q_att_valid = true;
	}
}

PositionControlStates IfodronePositionControl::buildStates(const vehicle_local_position_s &lp)
{
	const auto valid_or_nan = [](bool xy_valid, bool z_valid, float x, float y, float z) {
		return Vector3f(xy_valid ? x : NAN, xy_valid ? y : NAN, z_valid ? z : NAN);
	};

	PositionControlStates states{};
	states.position = valid_or_nan(lp.xy_valid, lp.z_valid, lp.x, lp.y, lp.z);
	states.velocity = valid_or_nan(lp.v_xy_valid, lp.v_z_valid, lp.vx, lp.vy, lp.vz);
	states.acceleration = Vector3f(lp.ax, lp.ay, lp.az);
	states.yaw = lp.heading;
	return states;
}

void IfodronePositionControl::updateSetpointSource(const vehicle_local_position_s &local_pos,
		const PositionControlStates &states)
{
	const bool pos_ctrl = _vehicle_control_mode.flag_multicopter_position_control_enabled;

	if (pos_ctrl && !_trajectory_setpoint_sub.updated()) {
		goto_setpoint_s goto_sp;

		if (_goto_setpoint_sub.update(&goto_sp) && goto_sp.timestamp != 0
		    && (hrt_absolute_time() - goto_sp.timestamp < TRAJECTORY_STREAM_TIMEOUT_US)) {
			_setpoint = PositionControl::empty_trajectory_setpoint;
			_setpoint.timestamp = goto_sp.timestamp;
			_setpoint.position[0] = goto_sp.position[0];
			_setpoint.position[1] = goto_sp.position[1];
			_setpoint.position[2] = goto_sp.position[2];

			if (goto_sp.flag_control_heading && PX4_ISFINITE(goto_sp.heading)) {
				_setpoint.yaw = goto_sp.heading;
			}
		}
	}

	_trajectory_setpoint_sub.update(&_setpoint);
	adjustSetpointForEKFResets(local_pos, _setpoint);

	// Setpoint predates the switch into position control: hold instead of jumping to it.
	if (pos_ctrl && (_setpoint.timestamp < _time_position_control_enabled)
	    && (local_pos.timestamp_sample > _time_position_control_enabled)) {
		_setpoint = generateFailsafeSetpoint(local_pos.timestamp_sample, states);
	}
}

trajectory_setpoint_s IfodronePositionControl::groundSetpoint(hrt_abstime timestamp)
{
	// On ground: push down, no integral.
	_control.resetIntegral();
	trajectory_setpoint_s sp = PositionControl::empty_trajectory_setpoint;
	sp.timestamp = timestamp;
	Vector3f(0.f, 0.f, 100.f).copyTo(sp.acceleration);
	return sp;
}

void IfodronePositionControl::resetLocks()
{
	_alt_lock = NAN;
	_pos_lock.setNaN();
}

void IfodronePositionControl::resetManualState()
{
	_hold_initialized = false;
	resetLocks();
	_vel_damp_body.setZero();
}

float IfodronePositionControl::updateManualYaw(float heading)
{
	if (!_hold_initialized) {
		_control.resetIntegral();
		_hold_yaw_angle = heading;
		_hold_initialized = true;
	}

	manual_control_setpoint_s yaw_stick{};
	_manual_control_setpoint_sub.copy(&yaw_stick);
	const float yaw_rate_sp = yaw_stick.yaw * MANUAL_YAW_RATE_MAX;

	const bool yaw_locked = (fabsf(yaw_rate_sp) < FLT_EPSILON)
				&& _vehicle_control_mode.flag_armed && !_vehicle_land_detected.landed;

	if (!yaw_locked) {
		_hold_yaw_angle = heading;
		return yaw_rate_sp;
	}

	const float yaw_error = wrap_pi(_hold_yaw_angle - heading);

	if (fabsf(yaw_error) > MANUAL_YAW_ERR_MAX) {
		_hold_yaw_angle = wrap_pi(heading + matrix::sign(yaw_error) * MANUAL_YAW_ERR_MAX);
	}

	return yaw_rate_sp;
}

bool IfodronePositionControl::runManual(const vehicle_local_position_s &local_pos,
					const PositionControlStates &states, float dt)
{
	const bool armed = _vehicle_control_mode.flag_armed;
	const bool landed = _vehicle_land_detected.landed;
	const float yaw_rate_sp = updateManualYaw(PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f);

	// Damping estimate is meaningless on the ground and would carry into takeoff.
	updateManualVelocityDamping(dt, !armed || landed);

	trajectory_setpoint_s manual_sp = generateManualSetpoint(local_pos, states, false);
	manual_sp.yawspeed = yaw_rate_sp;

	// Takeoff gating follows usable altitude feedback; direct-thrust modes skip the ramp.
	const bool alt_hold = _manual_alt_assisted;
	const bool want_takeoff = alt_hold && armed && PX4_ISFINITE(manual_sp.velocity[2]) && (manual_sp.velocity[2] < 0.f);
	_takeoff.updateTakeoffState(armed, landed, want_takeoff, !alt_hold, local_pos.timestamp_sample);

	const TakeoffState tko = _takeoff.getTakeoffState();
	const bool not_taken_off = alt_hold && (tko < TakeoffState::rampup);
	const bool flying = !alt_hold || (tko >= TakeoffState::flight);

	if (!flying) {
		_control.setHoverThrust(_param_ifo_thr_hover.get());
	}

	const float speed_up = _takeoff.updateRamp(dt, _param_ifo_vel_max_up.get());

	// Ramp intentionally suppresses lift: don't let Z integral wind up.
	if (alt_hold && (tko == TakeoffState::rampup)) {
		_control.resetIntegralZ();
	}

	_control.setVelocityLimits(_param_ifo_vel_max_xy.get(),
				   alt_hold ? math::min(speed_up, _param_ifo_vel_max_up.get()) : _param_ifo_vel_max_up.get(),
				   _param_ifo_vel_max_dn.get());

	// Thrust floor only after takeoff.
	const bool apply_thrust_floor = alt_hold ? flying : !landed;
	_control.setThrustLimits(apply_thrust_floor ? _param_ifo_thr_min.get() : 0.f, _param_ifo_thr_max.get());

	if (not_taken_off || (flying && alt_hold && _vehicle_land_detected.ground_contact)) {
		manual_sp = groundSetpoint(local_pos.timestamp_sample);
		resetLocks();
	}

	_control.setInputSetpoint(manual_sp);
	_control.setState(states);

	if (_control.update(dt)) {
		return true;
	}

	// Estimate went bad mid-flight: stay controllable on sticks alone (open loop).
	_control.resetIntegral();
	trajectory_setpoint_s degraded_sp = generateManualSetpoint(local_pos, states, true);
	degraded_sp.yawspeed = yaw_rate_sp;
	_control.setInputSetpoint(degraded_sp);
	return _control.update(dt);
}

bool IfodronePositionControl::runAuto(const vehicle_local_position_s &local_pos,
				      const PositionControlStates &states, float dt)
{
	resetManualState();

	_vehicle_constraints_sub.update(&_vehicle_constraints);

	if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_ifo_vel_max_up.get())) {
		_vehicle_constraints.speed_up = _param_ifo_vel_max_up.get();
	}

	if (_vehicle_control_mode.flag_control_offboard_enabled) {
		const bool sp_fresh = _vehicle_control_mode.flag_armed && (local_pos.timestamp_sample < _setpoint.timestamp + 1_s);
		_vehicle_constraints.want_takeoff =
		sp_fresh && ((PX4_ISFINITE(_setpoint.position[2])
		&& _setpoint.position[2] < states.position(2)) || (PX4_ISFINITE(_setpoint.velocity[2])
		&& _setpoint.velocity[2] < 0.f) || (PX4_ISFINITE(_setpoint.acceleration[2])
		&& _setpoint.acceleration[2] < 0.f));

		_vehicle_constraints.speed_up = _param_ifo_vel_max_up.get();
		_vehicle_constraints.speed_down = _param_ifo_vel_max_dn.get();
	}

	_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
				    _vehicle_constraints.want_takeoff, false, local_pos.timestamp_sample);

	const TakeoffState tko = _takeoff.getTakeoffState();
	const bool flying = (tko >= TakeoffState::flight);

	if (!flying) {
		_control.setHoverThrust(_param_ifo_thr_hover.get());
	}

	trajectory_setpoint_s control_sp = _setpoint;

	if (tko == TakeoffState::rampup) {
		// Ramp intentionally suppresses lift: no accel feedforward, no Z windup.
		if (PX4_ISFINITE(control_sp.velocity[2])) {
			control_sp.acceleration[2] = NAN;
		}

		_control.resetIntegralZ();
	}

	if ((tko < TakeoffState::rampup) || (flying && _vehicle_land_detected.ground_contact)) {
		control_sp = groundSetpoint(local_pos.timestamp_sample);
	}

	const float speed_up = _takeoff.updateRamp(dt, PX4_ISFINITE(_vehicle_constraints.speed_up)
				? _vehicle_constraints.speed_up : _param_ifo_vel_max_up.get());
	const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down)
				 ? _vehicle_constraints.speed_down : _param_ifo_vel_max_dn.get();

	// Allow ramping from zero thrust on takeoff.
	_control.setThrustLimits(flying ? _param_ifo_thr_min.get() : 0.f, _param_ifo_thr_max.get());
	_control.setVelocityLimits(_param_ifo_vel_max_xy.get(), math::min(speed_up, _param_ifo_vel_max_up.get()),
				   math::max(speed_down, 0.f));
	_control.setInputSetpoint(control_sp);

	const bool no_xy_input = !Vector2f(control_sp.velocity).isAllFinite() && !Vector2f(control_sp.position).isAllFinite();

	if (no_xy_input) {
		_control.resetIntegralXY();
	}

	_control.setState(states);

	if (_control.update(dt)) {
		return true;
	}

	_vehicle_constraints = {0, NAN, NAN, false, {}};
	_control.setInputSetpoint(generateFailsafeSetpoint(local_pos.timestamp_sample, states));
	_control.setVelocityLimits(_param_ifo_vel_max_xy.get(), _param_ifo_vel_max_up.get(), _param_ifo_vel_max_dn.get());
	return _control.update(dt);
}

void IfodronePositionControl::publishTakeoffStatus()
{
	const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

	if (takeoff_state == _takeoff_status_pub.get().takeoff_state) {
		return;
	}

	_takeoff_status_pub.get().takeoff_state = takeoff_state;
	_takeoff_status_pub.get().tilt_limit = M_PI_2_F; // always level
	_takeoff_status_pub.get().timestamp = hrt_absolute_time();
	_takeoff_status_pub.update();
}

void IfodronePositionControl::publishSetpoints(const PositionControlStates &states)
{
	vehicle_local_position_setpoint_s local_pos_sp{};
	_control.getLocalPositionSetpoint(local_pos_sp);
	local_pos_sp.timestamp = hrt_absolute_time();
	_local_pos_sp_pub.publish(local_pos_sp);

	// Full 3D thrust setpoint (NED) computed by the PositionControl library.
	const Vector3f thr_ned(local_pos_sp.thrust);

	const float yaw = PX4_ISFINITE(states.yaw) ? states.yaw : 0.f;
	float yaw_sp = PX4_ISFINITE(local_pos_sp.yaw) ? local_pos_sp.yaw : yaw;

	// Limit heading error to preserve collective authority in the coaxial pair.
	if (PX4_ISFINITE(states.yaw)) {
		const float yaw_lead = wrap_pi(yaw_sp - yaw);

		if (fabsf(yaw_lead) > MANUAL_YAW_ERR_MAX) {
			yaw_sp = wrap_pi(yaw + matrix::sign(yaw_lead) * MANUAL_YAW_ERR_MAX);
		}
	}

	const float yawspeed_sp = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;

	// Rotate into body axes and enforce actuator thrust limits.
	const Quatf q = _q_att_valid ? _q_att : Quatf(Eulerf(0.f, 0.f, yaw));
	const Vector3f thr_body = ControlMath::constrainIfodroneBodyThrust(
					  thr_ned, q, _param_ifo_thr_xy_max.get());

	// Attitude setpoint: always level, yaw only
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = hrt_absolute_time();
	att_sp.yaw_sp_move_rate = yawspeed_sp;
	const Quatf q_sp(Eulerf(0.f, 0.f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);
	thr_body.copyTo(att_sp.thrust_body);
	_attitude_setpoint_pub.publish(att_sp);
}

void IfodronePositionControl::updateManualVelocityDamping(float dt, bool reset)
{
	vehicle_acceleration_s accel{};

	if (reset || !_q_att_valid || !_vehicle_acceleration_sub.copy(&accel)) {
		_vel_damp_body.setZero();
		return;
	}

	// Add gravity in body axes to recover acceleration.
	const Vector3f specific_force(accel.xyz);
	const Vector3f gravity_body = _q_att.rotateVectorInverse(Vector3f(0.f, 0.f, CONSTANTS_ONE_G));
	const Vector2f acc_body = (specific_force + gravity_body).xy();

	if (!acc_body.isAllFinite()) {
		return;
	}

	// Clipped IMU samples would accumulate false lateral velocity.
	sensor_combined_s sensor_combined;

	if (_sensor_combined_sub.update(&sensor_combined) && (sensor_combined.accelerometer_clipping != 0)) {
		_accel_clip_last = sensor_combined.timestamp;
	}

	const bool accel_clipping = (_accel_clip_last != 0) && (hrt_elapsed_time(&_accel_clip_last) < ACCEL_CLIP_HOLDOFF);
	const Vector2f acc_integrated = accel_clipping ? Vector2f{} : acc_body;

	// Wash out integrated acceleration to bound drift from sensor bias.
	const float tau = math::max(_param_ifo_stb_vd_tau.get(), 0.1f);
	_vel_damp_body = (_vel_damp_body + acc_integrated * dt) * math::max(1.f - dt / tau, 0.f);

	const float vel_damp_max = math::max(_param_ifo_vel_max_xy.get(), 0.1f);
	const float vel_damp_norm = _vel_damp_body.norm();

	if (vel_damp_norm > vel_damp_max) {
		_vel_damp_body *= vel_damp_max / vel_damp_norm;
	}
}

trajectory_setpoint_s IfodronePositionControl::generateManualSetpoint(
	const vehicle_local_position_s &local_pos, const PositionControlStates &states, bool force_open_loop)
{
	manual_control_setpoint_s manual{};
	_manual_alt_assisted = false;

	if (!_manual_control_setpoint_sub.copy(&manual) || !manual.valid) {
		return PositionControl::empty_trajectory_setpoint;
	}

	auto deadband = [](float v) -> float {
		if (fabsf(v) <= STICK_DEADBAND) { return 0.f; }

		const float s = (v > 0.f) ? 1.f : -1.f;
		return (v - s * STICK_DEADBAND) / (1.f - STICK_DEADBAND);
	};

	const bool alt_hold = _vehicle_control_mode.flag_control_altitude_enabled;
	const bool pos_hold = _vehicle_control_mode.flag_control_position_enabled;

	// Never reuse a lock from the previous assist level.
	if ((alt_hold != _manual_alt_hold_prev) || (pos_hold != _manual_pos_hold_prev)) {
		resetLocks();
		_manual_alt_hold_prev = alt_hold;
		_manual_pos_hold_prev = pos_hold;
	}

	trajectory_setpoint_s sp = PositionControl::empty_trajectory_setpoint;
	sp.timestamp = local_pos.timestamp_sample;
	sp.yaw = _hold_yaw_angle;

	const float stick_x = deadband(manual.pitch);
	const float stick_y = deadband(manual.roll);

	const bool earth_frame_xy = (alt_hold || pos_hold) && !force_open_loop && local_pos.v_xy_valid
				    && Vector2f(states.velocity).isAllFinite()
				    && Vector2f(states.acceleration).isAllFinite();

	if (earth_frame_xy) {
		sp.velocity[0] = stick_x * _param_ifo_vel_max_xy.get();
		sp.velocity[1] = stick_y * _param_ifo_vel_max_xy.get();

		// Position lock (Position mode only): latch once the vehicle has stopped.
		const bool xy_pos_usable = pos_hold && local_pos.xy_valid && Vector2f(states.position).isAllFinite();

		if (!xy_pos_usable || (fabsf(stick_x) > 0.f) || (fabsf(stick_y) > 0.f)) {
			_pos_lock.setNaN();

		} else if (!_pos_lock.isAllFinite() && (Vector2f(states.velocity).norm() < LOCK_VEL_MAX)) {
			_pos_lock = Vector2f(states.position);
		}

		if (_pos_lock.isAllFinite()) {
			sp.position[0] = _pos_lock(0);
			sp.position[1] = _pos_lock(1);
			sp.velocity[0] = NAN;
			sp.velocity[1] = NAN;
		}

	} else {
		// Without velocity feedback, use body-frame thrust with IMU damping.
		_pos_lock.setNaN();
		const float acc_xy_max = _param_ifo_thr_xy_max.get() * CONSTANTS_ONE_G
					 / math::max(_param_ifo_thr_hover.get(), 0.1f);

		const Vector2f acc_body(stick_x * acc_xy_max - _param_ifo_stb_vd_p.get() * _vel_damp_body(0),
					stick_y * acc_xy_max - _param_ifo_stb_vd_p.get() * _vel_damp_body(1));

		// Rotate lateral demand by yaw; body conversion removes heading dependence.
		const float yaw = _q_att_valid ? Eulerf(_q_att).psi() : 0.f;
		const float cy = cosf(yaw);
		const float sy = sinf(yaw);
		sp.acceleration[0] = cy * acc_body(0) - sy * acc_body(1);
		sp.acceleration[1] = sy * acc_body(0) + cy * acc_body(1);
	}

	const float stick_thr = deadband(manual.throttle);
	const bool alt_usable = alt_hold && !force_open_loop && local_pos.z_valid && local_pos.v_z_valid
				&& PX4_ISFINITE(states.position(2)) && PX4_ISFINITE(states.velocity(2))
				&& PX4_ISFINITE(states.acceleration(2));

	// Takeoff handling must follow the available feedback, not only the mode flag.
	_manual_alt_assisted = alt_usable;

	if (alt_usable) {
		// Throttle stick centred ⇒ hold altitude, deflected ⇒ climb rate.
		if (fabsf(stick_thr) > 0.f) {
			_alt_lock = NAN;
			sp.velocity[2] = (stick_thr > 0.f) ? -stick_thr * _param_ifo_vel_max_up.get()
					 : -stick_thr * _param_ifo_vel_max_dn.get();

		} else {
			if (!PX4_ISFINITE(_alt_lock) && (fabsf(states.velocity(2)) < LOCK_VEL_MAX)) {
				_alt_lock = states.position(2);
			}

			if (PX4_ISFINITE(_alt_lock)) {
				sp.position[2] = _alt_lock;

			} else {
				sp.velocity[2] = 0.f;  // brake first, latch once stopped
			}
		}

	} else {
		// Map throttle around hover, then invert the acceleration-to-thrust model.
		_alt_lock = NAN;
		const float thr_01 = math::constrain((manual.throttle + 1.f) * 0.5f, 0.f, 1.f);
		const float thr_min = _param_ifo_thr_min.get();
		const float thr_max = _param_ifo_thr_max.get();
		const float hover = math::constrain(_param_ifo_thr_hover.get(), thr_min + 0.01f, thr_max - 0.01f);
		const float thrust = (thr_01 < 0.5f) ? (thr_min + (hover - thr_min) * (thr_01 / 0.5f))
				     : (hover + (thr_max - hover) * ((thr_01 - 0.5f) / 0.5f));
		sp.acceleration[2] = CONSTANTS_ONE_G * (1.f - thrust / hover);
	}

	return sp;
}

void IfodronePositionControl::adjustSetpointForEKFResets(
	const vehicle_local_position_s &local_pos, trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < local_pos.timestamp)) {
		if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += local_pos.delta_vxy[0];
			setpoint.velocity[1] += local_pos.delta_vxy[1];
		}

		if (local_pos.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += local_pos.delta_vz;
		}

		if (local_pos.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += local_pos.delta_xy[0];
			setpoint.position[1] += local_pos.delta_xy[1];
		}

		if (local_pos.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += local_pos.delta_z;
		}

		if (local_pos.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + local_pos.delta_heading);
		}
	}

	if (local_pos.xy_reset_counter != _xy_reset_counter) {
		_pos_lock += Vector2f(local_pos.delta_xy);
	}

	if (local_pos.z_reset_counter != _z_reset_counter) {
		_alt_lock += local_pos.delta_z;
	}

	if (local_pos.heading_reset_counter != _heading_reset_counter) {
		_hold_yaw_angle = wrap_pi(_hold_yaw_angle + local_pos.delta_heading);
	}

	_vxy_reset_counter = local_pos.vxy_reset_counter;
	_vz_reset_counter = local_pos.vz_reset_counter;
	_xy_reset_counter = local_pos.xy_reset_counter;
	_z_reset_counter = local_pos.z_reset_counter;
	_heading_reset_counter = local_pos.heading_reset_counter;
}

trajectory_setpoint_s IfodronePositionControl::generateFailsafeSetpoint(
	const hrt_abstime &now, const PositionControlStates &states)
{
	trajectory_setpoint_s failsafe = PositionControl::empty_trajectory_setpoint;
	failsafe.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// Stop XY
		failsafe.velocity[0] = 0.f;
		failsafe.velocity[1] = 0.f;

	} else {
		failsafe.acceleration[0] = 0.f;
		failsafe.acceleration[1] = 0.f;
		failsafe.velocity[2] = _param_ifo_land_speed.get();
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		if (!PX4_ISFINITE(failsafe.velocity[2])) {
			failsafe.velocity[2] = 0.f;
		}

	} else {
		failsafe.velocity[2] = NAN;
		failsafe.acceleration[2] = .3f;
	}

	return failsafe;
}

int IfodronePositionControl::task_spawn(int argc, char *argv[])
{
	IfodronePositionControl *instance = new IfodronePositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int IfodronePositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IFODRONE position controller. Cascaded P-position + PID-velocity controller
for an airframe with decoupled position/orientation (level body, no pitch/roll).

Publishes body-frame thrust and yaw-only attitude setpoint.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int IfodronePositionControl::print_status()
{
	PX4_INFO("IFODRONE Position Controller  takeoff_state=%d",
		 (int)_takeoff.getTakeoffState());
	perf_print_counter(_cycle_perf);
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
