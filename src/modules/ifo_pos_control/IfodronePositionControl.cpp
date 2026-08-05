/**
 * IFODRONE Position Controller
 *
 * Structure modeled after mc_pos_control. Uses the PositionControl library
 * (cascaded P-position + PID-velocity) to produce a full 3D thrust setpoint,
 * which is rotated into the body frame, constrained to the actuator-feasible
 * wrench, and published with an always-level (roll/pitch = 0) attitude
 * setpoint. The control allocator realizes the resulting thrust and torque.
 *
 * Supports:
 *   - Offboard mode (external trajectory_setpoint)
 *   - goto_setpoint (direct position target, converted here)
 *   - the three stick-flown modes, whose setpoints are built here from the
 *     sticks rather than taken from flight_mode_manager:
 *       Stabilized: BODY lateral thrust damped by an accelerometer-only velocity
 *                   estimate + throttle stick → collective thrust. No heading,
 *                   no GPS, no compass anywhere in the loop.
 *       Altitude:   XY velocity in local NED (pitch → north, roll → east)
 *                   + climb rate, throttle centred = altitude hold
 *       Position:   as Altitude, plus XY position hold when the sticks are centred
 */

#include "IfodronePositionControl.hpp"
#include "PositionControl/ControlMath.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

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
		_control.setHorizontalThrustMargin(_param_ifo_thr_xy_max.get());
		_control.setTiltLimit(M_PI_2_F); // no tilt limit: IFODRONE is always level

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

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const float dt = math::constrain(
					 ((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = local_pos.timestamp_sample;

		// --- Update control mode ---
		if (_vehicle_control_mode_sub.updated()) {
			const bool prev_pos_ctrl = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!prev_pos_ctrl && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (prev_pos_ctrl && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
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

		// --- Set vehicle states ---
		PositionControlStates states{};

		if (local_pos.xy_valid) {
			states.position.xy() = Vector2f(local_pos.x, local_pos.y);
		} else {
			states.position(0) = states.position(1) = NAN;
		}

		if (local_pos.z_valid) {
			states.position(2) = local_pos.z;
		} else {
			states.position(2) = NAN;
		}

		if (local_pos.v_xy_valid) {
			states.velocity.xy() = Vector2f(local_pos.vx, local_pos.vy);
		} else {
			states.velocity(0) = states.velocity(1) = NAN;
		}

		if (local_pos.v_z_valid) {
			states.velocity(2) = local_pos.vz;
		} else {
			states.velocity(2) = NAN;
		}

		// Use velocity derivative as acceleration estimate
		states.acceleration = Vector3f(local_pos.ax, local_pos.ay, local_pos.az);
		states.yaw = local_pos.heading;

		// --- Handle goto_setpoint → trajectory_setpoint conversion ---
		// If a goto_setpoint is available and no trajectory_setpoint was freshly published,
		// convert goto to a trajectory_setpoint (simple: position target, no smoothing).
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && !_trajectory_setpoint_sub.updated()) {

			goto_setpoint_s goto_sp;

			if (_goto_setpoint_sub.update(&goto_sp) && goto_sp.timestamp != 0
			    && (hrt_absolute_time() - goto_sp.timestamp < TRAJECTORY_STREAM_TIMEOUT_US)) {

				// Convert goto to trajectory setpoint
				trajectory_setpoint_s traj = PositionControl::empty_trajectory_setpoint;
				traj.timestamp = goto_sp.timestamp;
				traj.position[0] = goto_sp.position[0];
				traj.position[1] = goto_sp.position[1];
				traj.position[2] = goto_sp.position[2];

				if (goto_sp.flag_control_heading && PX4_ISFINITE(goto_sp.heading)) {
					traj.yaw = goto_sp.heading;
				}

				_setpoint = traj;
			}
		}

		// --- Read trajectory setpoint ---
		_trajectory_setpoint_sub.update(&_setpoint);
		adjustSetpointForEKFResets(local_pos, _setpoint);

		// --- Failsafe if no setpoint since control enabled ---
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (local_pos.timestamp_sample > _time_position_control_enabled)) {
				_setpoint = generateFailsafeSetpoint(local_pos.timestamp_sample, states);
			}
		}

		// Stabilized/Manual: commander leaves flag_multicopter_position_control_enabled
		// clear, but this module still owns the horizontal loop there (sticks command
		// XY velocity), so it has to run. Acro (no attitude stabilization) is excluded.
		const bool manual_stabilized = _vehicle_control_mode.flag_control_manual_enabled
					       && _vehicle_control_mode.flag_control_attitude_enabled
					       && !_vehicle_control_mode.flag_multicopter_position_control_enabled;

		// --- Run position control ---
		if ((_vehicle_control_mode.flag_multicopter_position_control_enabled
		     && (_vehicle_control_mode.flag_control_manual_enabled
			 || _setpoint.timestamp >= _time_position_control_enabled))
		    || manual_stabilized) {

			if (_vehicle_control_mode.flag_control_manual_enabled) {
				// -------------------------------------------------------
				// IFODRONE MANUAL MODES (sticks)
				//   Stabilized: damped body lateral thrust + direct collective
				//               (compass-free, GPS-free)
				//   Altitude:   XY velocity (NED) + climb rate / altitude lock
				//   Position:   as Altitude + XY position lock
				// -------------------------------------------------------
				const bool alt_hold = _vehicle_control_mode.flag_control_altitude_enabled;

				const float heading = PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f;

				if (!_hold_initialized) {
					_control.resetIntegral();
					_hold_initialized = true;
				}

				// Yaw stick: turning is commanded as a rate and the heading setpoint
				// simply follows the vehicle; it only locks once the stick is centred.
				// Nothing is integrated, so a drifting or wrong heading estimate cannot
				// build up an error — which matters here because yaw comes from the
				// coaxial pair differentially, and a yaw error it cannot track saturates
				// the pair and eats the collective thrust.
				manual_control_setpoint_s yaw_stick{};
				_manual_control_setpoint_sub.copy(&yaw_stick);

				const float yaw_rate_sp = yaw_stick.yaw * MANUAL_YAW_RATE_MAX;
				const bool yaw_locked = (fabsf(yaw_rate_sp) < FLT_EPSILON)
							&& _vehicle_control_mode.flag_armed && !_vehicle_land_detected.landed;

				if (!yaw_locked) {
					_hold_yaw_angle = heading;

				} else {
					// Hold, but never further ahead of the vehicle than it can track.
					const float yaw_error = wrap_pi(_hold_yaw_angle - heading);

					if (fabsf(yaw_error) > MANUAL_YAW_ERR_MAX) {
						_hold_yaw_angle = wrap_pi(heading + matrix::sign(yaw_error) * MANUAL_YAW_ERR_MAX);
					}
				}

				// The damping estimate is meaningless on the ground and would carry a
				// stale value into the takeoff.
				updateManualVelocityDamping(dt, !_vehicle_control_mode.flag_armed || _vehicle_land_detected.landed);

				trajectory_setpoint_s manual_sp = generateManualSetpoint(local_pos, states, false);
				manual_sp.yawspeed = yaw_rate_sp;

				// Takeoff. Stabilized commands thrust directly, so the state machine is
				// only kept in sync there (skip_takeoff) and the stick owns the motors.
				// With altitude assist the throttle stick requests the takeoff and the
				// climb rate is ramped, otherwise the ground push-down below and the
				// land detector's low-thrust ground contact would latch each other.
				const bool want_takeoff = alt_hold && _vehicle_control_mode.flag_armed
							  && PX4_ISFINITE(manual_sp.velocity[2]) && (manual_sp.velocity[2] < 0.f);
				_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
							    want_takeoff, _param_ifo_vel_max_up.get(), !alt_hold,
							    local_pos.timestamp_sample);

				const bool not_taken_off = alt_hold && (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying = !alt_hold || (_takeoff.getTakeoffState() >= TakeoffState::flight);

				if (!flying) {
					_control.setHoverThrust(_param_ifo_thr_hover.get());
				}

				const float speed_up = _takeoff.updateRamp(dt, _param_ifo_vel_max_up.get());
				_control.setVelocityLimits(
					_param_ifo_vel_max_xy.get(),
					alt_hold ? math::min(speed_up, _param_ifo_vel_max_up.get()) : _param_ifo_vel_max_up.get(),
					_param_ifo_vel_max_dn.get());

				// The pilot must be able to spool up from zero on the ground: with
				// altitude assist the floor appears once the takeoff ramp is done,
				// in Stabilized as soon as the vehicle is off the ground.
				const bool apply_thrust_floor = alt_hold ? flying : !_vehicle_land_detected.landed;
				_control.setThrustLimits(apply_thrust_floor ? _param_ifo_thr_min.get() : 0.f,
							 _param_ifo_thr_max.get());

				if (not_taken_off || (flying && alt_hold && _vehicle_land_detected.ground_contact)) {
					// On ground with altitude assist: push down, no integral
					_control.resetIntegral();
					manual_sp = PositionControl::empty_trajectory_setpoint;
					manual_sp.timestamp = local_pos.timestamp_sample;
					Vector3f(0.f, 0.f, 100.f).copyTo(manual_sp.acceleration);
					_alt_lock = NAN;
					_pos_lock.setNaN();
				}

				_control.setInputSetpoint(manual_sp);
				_control.setState(states);

				if (!_control.update(dt)) {
					// Estimate went bad mid-flight: keep the vehicle controllable on
					// sticks alone (open-loop lateral thrust + collective from throttle).
					_control.resetIntegral();
					trajectory_setpoint_s degraded_sp = generateManualSetpoint(local_pos, states, true);
					degraded_sp.yawspeed = yaw_rate_sp;
					_control.setInputSetpoint(degraded_sp);
					_control.update(dt);
				}

				publishSetpoints(states);

			} else {
				// -------------------------------------------------------
				// POSITION / OFFBOARD CONTROL (PID loop)
				// -------------------------------------------------------
				_hold_initialized = false; // reset for next manual entry
				_alt_lock = NAN;
				_pos_lock.setNaN();
				_vel_damp_body.setZero();

				// Update constraints
				_vehicle_constraints_sub.update(&_vehicle_constraints);

				if (!PX4_ISFINITE(_vehicle_constraints.speed_up)
				    || (_vehicle_constraints.speed_up > _param_ifo_vel_max_up.get())) {
					_vehicle_constraints.speed_up = _param_ifo_vel_max_up.get();
				}

				// Offboard: determine want_takeoff
				if (_vehicle_control_mode.flag_control_offboard_enabled) {
					const bool want_takeoff = _vehicle_control_mode.flag_armed
								  && (local_pos.timestamp_sample < _setpoint.timestamp + 1_s);

					if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
					    && (_setpoint.position[2] < states.position(2))) {
						_vehicle_constraints.want_takeoff = true;

					} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
						   && (_setpoint.velocity[2] < 0.f)) {
						_vehicle_constraints.want_takeoff = true;

					} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
						   && (_setpoint.acceleration[2] < 0.f)) {
						_vehicle_constraints.want_takeoff = true;

					} else {
						_vehicle_constraints.want_takeoff = false;
					}

					_vehicle_constraints.speed_up = _param_ifo_vel_max_up.get();
					_vehicle_constraints.speed_down = _param_ifo_vel_max_dn.get();
				}

				// Takeoff state machine
				_takeoff.updateTakeoffState(
					_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
					_vehicle_constraints.want_takeoff,
					_vehicle_constraints.speed_up, false, local_pos.timestamp_sample);

				const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				if (!flying) {
					_control.setHoverThrust(_param_ifo_thr_hover.get());
				}

				// During ramp, don't allow acceleration feedforward to interfere
				if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
					_setpoint.acceleration[2] = NAN;
				}

				if (not_taken_off || flying_but_ground_contact) {
					// On ground: zero everything, push down
					_setpoint = PositionControl::empty_trajectory_setpoint;
					_setpoint.timestamp = local_pos.timestamp_sample;
					Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration);
					_control.resetIntegral();
				}

				// Velocity limits with takeoff ramp
				const float speed_up = _takeoff.updateRamp(dt,
						       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_ifo_vel_max_up.get());
				const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down)
							 ? _vehicle_constraints.speed_down : _param_ifo_vel_max_dn.get();

				// Allow ramping from zero thrust on takeoff
				const float minimum_thrust = flying ? _param_ifo_thr_min.get() : 0.f;
				_control.setThrustLimits(minimum_thrust, _param_ifo_thr_max.get());

				_control.setVelocityLimits(
					_param_ifo_vel_max_xy.get(),
					math::min(speed_up, _param_ifo_vel_max_up.get()),
					math::max(speed_down, 0.f));

				_control.setInputSetpoint(_setpoint);

				// If no XY control input, reset XY integrator
				if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
				    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
					_control.resetIntegralXY();
				}

				_control.setState(states);

				// Run PID control (position → velocity → acceleration → thrust internally)
				const hrt_abstime now = hrt_absolute_time();

				if (_control.update(dt)) {
					// Valid control update — store for fallback
					_last_valid_setpoint = _setpoint;

				} else {
					// Update failed — try last valid setpoint first (200 ms window)
					if (now < _last_valid_setpoint.timestamp + 200_ms) {
						adjustSetpointForEKFResets(local_pos, _last_valid_setpoint);
						_control.setInputSetpoint(_last_valid_setpoint);
					}

					// Still failing — go to failsafe
					if (!_control.update(dt)) {
						_vehicle_constraints = {0, NAN, NAN, false, {}};
						_control.setInputSetpoint(generateFailsafeSetpoint(local_pos.timestamp_sample, states));
						_control.setVelocityLimits(_param_ifo_vel_max_xy.get(), _param_ifo_vel_max_up.get(),
									   _param_ifo_vel_max_dn.get());
						_control.update(dt);
					}
				}

				publishSetpoints(states);
			}

		} else {
			// Not in position control mode: update takeoff state machine but do nothing
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    false, 10.f, true, local_pos.timestamp_sample);
			_control.resetIntegral();
			_hold_initialized = false;
			_alt_lock = NAN;
			_pos_lock.setNaN();
			_vel_damp_body.setZero();
		}

		// --- Publish takeoff status ---
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = M_PI_2_F; // always level
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
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
	const float yaw_sp = PX4_ISFINITE(local_pos_sp.yaw) ? local_pos_sp.yaw : yaw;
	const float yawspeed_sp = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;

	// Rotate NED→body with the full current attitude, but only publish a
	// physically feasible wrench. In particular, body +Z cannot be produced by
	// the fixed coaxial pair and must not be synthesized by tilting the EDFs.
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
	vehicle_acceleration_s accel;
	_vehicle_acceleration_sub.update(&accel);

	if (reset || !_q_att_valid) {
		_vel_damp_body.setZero();
		return;
	}

	// Specific force → acceleration: subtracting gravity needs the attitude, but a
	// rotation about z leaves the gravity vector unchanged, so only roll and pitch
	// enter here. This estimate is therefore completely heading-free.
	const Vector3f specific_force(accel.xyz);
	const Vector3f gravity_body = _q_att.rotateVectorInverse(Vector3f(0.f, 0.f, CONSTANTS_ONE_G));
	const Vector2f acc_body = (specific_force + gravity_body).xy();

	if (!acc_body.isAllFinite()) {
		return;
	}

	// Integrate with a washout: without an absolute reference the integral would
	// run away on bias alone, so it is only trusted over the washout horizon. That
	// is enough to damp gusts and rebound, and deliberately not enough to hold a
	// position against a steady wind.
	const float tau = math::max(_param_ifo_stb_vd_tau.get(), 0.1f);
	_vel_damp_body = (_vel_damp_body + acc_body * dt) * math::max(1.f - dt / tau, 0.f);
}

trajectory_setpoint_s IfodronePositionControl::generateManualSetpoint(
	const vehicle_local_position_s &local_pos, const PositionControlStates &states, bool force_open_loop)
{
	manual_control_setpoint_s manual{};
	_manual_control_setpoint_sub.copy(&manual);

	auto deadband = [](float v) -> float {
		if (fabsf(v) <= STICK_DEADBAND) { return 0.f; }

		const float s = (v > 0.f) ? 1.f : -1.f;
		return (v - s * STICK_DEADBAND) / (1.f - STICK_DEADBAND);
	};

	const bool alt_hold = _vehicle_control_mode.flag_control_altitude_enabled;
	const bool pos_hold = _vehicle_control_mode.flag_control_position_enabled;

	// Drop the latches whenever the assist level changes, so a mode switch never
	// re-uses a stale lock from the previous mode.
	if ((alt_hold != _manual_alt_hold_prev) || (pos_hold != _manual_pos_hold_prev)) {
		_alt_lock = NAN;
		_pos_lock.setNaN();
		_manual_alt_hold_prev = alt_hold;
		_manual_pos_hold_prev = pos_hold;
	}

	trajectory_setpoint_s sp = PositionControl::empty_trajectory_setpoint;
	sp.timestamp = local_pos.timestamp_sample;
	sp.yaw = _hold_yaw_angle;

	// --- Horizontal ------------------------------------------------------------
	// Altitude/Position fly the earth-frame law (pitch → +North, roll → +East);
	// Stabilized flies the body-frame law below, which needs no heading at all.
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
		// Stabilized (and the degraded fallback of the assisted modes): the sticks
		// command lateral thrust in the BODY frame, damped by a velocity estimate
		// integrated from the accelerometer. No heading, no GPS, no compass — the
		// only earth reference is gravity, which fixes roll/pitch but never yaw.
		//
		// IFO_THR_XY_MAX is a normalized thrust, converted here to the acceleration
		// the library maps back to that thrust at hover (a = T * g / hover_thrust).
		_pos_lock.setNaN();
		const float acc_xy_max = _param_ifo_thr_xy_max.get() * CONSTANTS_ONE_G
					 / math::max(_param_ifo_thr_hover.get(), 0.1f);

		const Vector2f acc_body(stick_x * acc_xy_max - _param_ifo_stb_vd_p.get() * _vel_damp_body(0),
					stick_y * acc_xy_max - _param_ifo_stb_vd_p.get() * _vel_damp_body(1));

		// The library works in NED, publishSetpoints() rotates its output back into
		// the body frame with the same attitude, so this rotation cancels exactly:
		// a wrong yaw estimate cannot enter the loop, it only cancels against itself.
		const float yaw = _q_att_valid ? Eulerf(_q_att).psi() : 0.f;
		const float cy = cosf(yaw);
		const float sy = sinf(yaw);
		sp.acceleration[0] = cy * acc_body(0) - sy * acc_body(1);
		sp.acceleration[1] = sy * acc_body(0) + cy * acc_body(1);
	}

	// --- Vertical -------------------------------------------------------------
	const float stick_thr = deadband(manual.throttle);
	const bool alt_usable = alt_hold && !force_open_loop && local_pos.z_valid && local_pos.v_z_valid
				&& PX4_ISFINITE(states.position(2)) && PX4_ISFINITE(states.velocity(2))
				&& PX4_ISFINITE(states.acceleration(2));

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
		// Stabilized (or no altitude estimate): throttle stick → collective thrust,
		// mapped so that mid-stick is the hover thrust (same idea as MPC_THR_CURVE
		// "rescale to hover thrust"), otherwise the vehicle only lifts off well above
		// centre. Inverting the library's thrust model T = a_z * hover/g - hover gives
		// the acceleration setpoint that produces exactly the demanded thrust.
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
