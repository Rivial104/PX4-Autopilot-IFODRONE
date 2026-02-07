/**
 * IFODRONE Position Controller
 *
 * Full position controller with state machine based on MulticopterPositionControl.
 * Custom thrust/torque generation for IFODRONE airframe.
 */

#include "IfodronePositionControl.hpp"

#include <float.h>
#include <px4_platform_common/events.h>

using namespace matrix;

IfodronePositionControl::IfodronePositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_tilt_limit_slew_rate.setSlewRate(0.2f);
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
	parameters_update(true);
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

		// Initialize takeoff handling
		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());

		// Initialize hover thrust if not using HTE
		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_hover_thrust = _param_ifo_thr_hover.get();
			_hover_thrust_initialized = true;
		}
	}
}

void IfodronePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime now = hrt_absolute_time();

		const float dt = math::constrain(((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = local_pos.timestamp_sample;

		// ========================================
		// 1. Update control mode
		// ========================================
		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// Clear setpoint when controller is no longer active
					_setpoint = trajectory_setpoint_s{};
					_setpoint.position[0] = _setpoint.position[1] = _setpoint.position[2] = NAN;
					_setpoint.velocity[0] = _setpoint.velocity[1] = _setpoint.velocity[2] = NAN;
					_setpoint.acceleration[0] = _setpoint.acceleration[1] = _setpoint.acceleration[2] = NAN;
				}
			}
		}

		// ========================================
		// 2. Update land detected
		// ========================================
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// ========================================
		// 3. Update hover thrust estimate
		// ========================================
		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_hover_thrust = hte.hover_thrust;
				}
			}
		}

		// ========================================
		// 4. Update vehicle state
		// ========================================
		_position = Vector3f(local_pos.x, local_pos.y, local_pos.z);
		_velocity = Vector3f(local_pos.vx, local_pos.vy, local_pos.vz);
		_yaw = local_pos.heading;
		float dbg_vz_sp = NAN;
		float dbg_vel_err_z = NAN;
		float dbg_accel_sp_z = NAN;
		float dbg_thr_z = NAN;

		// ========================================
		// 5. Get trajectory setpoint
		// ========================================
		_trajectory_setpoint_sub.update(&_setpoint);

		// ========================================
		// 6. Adjust setpoint for EKF resets
		// ========================================
		adjustSetpointForEKFResets(local_pos, _setpoint);

		// ========================================
		// 7. MAIN CONTROL STATE MACHINE
		// ========================================
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			// Set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (local_pos.timestamp_sample > _time_position_control_enabled)) {
				_setpoint = generateFailsafeSetpoint(local_pos.timestamp_sample, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// ========================================
			// 7a. Update vehicle constraints
			// ========================================
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// Fix constraints if invalid
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			// ========================================
			// 7b. Handle offboard takeoff detection
			// ========================================
			if (_vehicle_control_mode.flag_control_offboard_enabled) {
				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (local_pos.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < _position(2))) {
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

				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// ========================================
			// 7c. UPDATE TAKEOFF STATE MACHINE
			// ========================================
			const bool skip_takeoff = false; // Set to true for throw launch
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed,
						    _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up,
						    skip_takeoff,
						    local_pos.timestamp_sample);

			const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			// Reset hover thrust when on ground
			if (!flying) {
				_hover_thrust = _param_ifo_thr_hover.get();
			}

			// ========================================
			// 7d. Handle pre-takeoff and ground contact
			// ========================================
			Vector3f thrust_sp_body;
			thrust_sp_body.setZero();
			Vector3f pos_sp(NAN, NAN, NAN);
			Vector3f vel_sp(NAN, NAN, NAN);
			Vector3f accel_sp(NAN, NAN, NAN);

			if (not_taken_off || flying_but_ground_contact) {
				// Not flying yet - set zero thrust with high downward acceleration
				// to prevent unwanted movement
				_setpoint = trajectory_setpoint_s{};
				_setpoint.timestamp = local_pos.timestamp_sample;
				_setpoint.position[0] = _setpoint.position[1] = _setpoint.position[2] = NAN;
				_setpoint.velocity[0] = _setpoint.velocity[1] = _setpoint.velocity[2] = NAN;
				_setpoint.acceleration[0] = 0.f;
				_setpoint.acceleration[1] = 0.f;
				_setpoint.acceleration[2] = 100.f; // High downwards acceleration -> no thrust

				// Pass through to thrust calculation
				thrust_sp_body(2) = 0.f; // No vertical thrust when not taken off

				_vel_integral = Vector3f{};
				_pos_integral = Vector3f{};

			} else {
				// ========================================
				// 7e. NORMAL FLIGHT - CONTROL LOOP
				// ========================================

				// Get velocity limit from takeoff ramp
				const float speed_up = _takeoff.updateRamp(dt,
						       PX4_ISFINITE(_vehicle_constraints.speed_up) ?
						       _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());

				const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ?
							 _vehicle_constraints.speed_down : _param_mpc_z_vel_max_dn.get();

				// ----------------------------------------
				// Position setpoint
				// ----------------------------------------
				pos_sp = _position; // Default: hold current position

				if (PX4_ISFINITE(_setpoint.position[0])) {
					pos_sp(0) = _setpoint.position[0];
				}

				if (PX4_ISFINITE(_setpoint.position[1])) {
					pos_sp(1) = _setpoint.position[1];
				}

				if (PX4_ISFINITE(_setpoint.position[2])) {
					pos_sp(2) = _setpoint.position[2];
				}

				// ----------------------------------------
				// TODO: Call your position->velocity control
				// ----------------------------------------
				vel_sp = computeVelocitySetpoint(pos_sp, _position, speed_up, speed_down, dt);

				// Override with trajectory velocity if provided
				if (PX4_ISFINITE(_setpoint.velocity[0])) {
					vel_sp(0) = _setpoint.velocity[0];
					_pos_integral(0) = 0.f;
				}

				if (PX4_ISFINITE(_setpoint.velocity[1])) {
					vel_sp(1) = _setpoint.velocity[1];
					_pos_integral(1) = 0.f;
				}

				if (PX4_ISFINITE(_setpoint.velocity[2])) {
					vel_sp(2) = _setpoint.velocity[2];
					_pos_integral(2) = 0.f;
				}

				// Apply velocity limits
				vel_sp(0) = math::constrain(vel_sp(0), -_param_mpc_xy_vel_max.get(), _param_mpc_xy_vel_max.get());
				vel_sp(1) = math::constrain(vel_sp(1), -_param_mpc_xy_vel_max.get(), _param_mpc_xy_vel_max.get());
				vel_sp(2) = math::constrain(vel_sp(2), -speed_up, speed_down);

				// ----------------------------------------
				// TODO: Call your velocity->acceleration control
				// ----------------------------------------
				accel_sp = computeAccelerationSetpoint(vel_sp, _velocity, dt);

				// Override with trajectory acceleration if provided
				if (PX4_ISFINITE(_setpoint.acceleration[0])) {
					accel_sp(0) = _setpoint.acceleration[0];
					_vel_integral(0) = 0.f;
				}

				if (PX4_ISFINITE(_setpoint.acceleration[1])) {
					accel_sp(1) = _setpoint.acceleration[1];
					_vel_integral(1) = 0.f;
				}

				if (PX4_ISFINITE(_setpoint.acceleration[2])) {
					accel_sp(2) = _setpoint.acceleration[2];
					_vel_integral(2) = 0.f;
				}

				// ----------------------------------------
				// TODO: Call your IFODRONE thrust generation
				// ----------------------------------------
				thrust_sp_body = computeIfodroneThrust(accel_sp, _yaw);
				dbg_vz_sp = vel_sp(2);
				dbg_vel_err_z = vel_sp(2) - _velocity(2);
				dbg_accel_sp_z = accel_sp(2);
				dbg_thr_z = thrust_sp_body(2);

				// Apply thrust limits
				// const float thrust_min = flying ? _param_ifo_thr_min.get() : 0.f;
				const float thrust_min = 0.f;
				thrust_sp_body(2) = math::constrain(thrust_sp_body(2), -_param_ifo_thr_max.get(), -thrust_min);

				// Update last valid setpoint for fallback
				_last_valid_setpoint = _setpoint;
			}

			// ========================================
			// 7f. Get yaw setpoint
			// ========================================
			float yaw_sp = _yaw; // Default: hold current yaw

			if (PX4_ISFINITE(_setpoint.yaw)) {
				yaw_sp = _setpoint.yaw;
			}

			const float yawspeed_sp = PX4_ISFINITE(_setpoint.yawspeed) ? _setpoint.yawspeed : 0.f;

			// ========================================
			// 7g. Tilt limit (smooth transitions)
			// ========================================
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get()
						     : _param_mpc_tiltmax_air.get();
			const float tilt_limit_rad = _tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt);

			// ========================================
			// 7h. Publish local position setpoint
			// ========================================
			vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = now;
			local_pos_sp.x = PX4_ISFINITE(pos_sp(0)) ? pos_sp(0) : NAN;
			local_pos_sp.y = PX4_ISFINITE(pos_sp(1)) ? pos_sp(1) : NAN;
			local_pos_sp.z = PX4_ISFINITE(pos_sp(2)) ? pos_sp(2) : NAN;
			local_pos_sp.vx = PX4_ISFINITE(vel_sp(0)) ? vel_sp(0) : NAN;
			local_pos_sp.vy = PX4_ISFINITE(vel_sp(1)) ? vel_sp(1) : NAN;
			local_pos_sp.vz = PX4_ISFINITE(vel_sp(2)) ? vel_sp(2) : NAN;
			local_pos_sp.acceleration[0] = accel_sp(0);
			local_pos_sp.acceleration[1] = accel_sp(1);
			local_pos_sp.acceleration[2] = accel_sp(2);
			local_pos_sp.thrust[0] = thrust_sp_body(0);
			local_pos_sp.thrust[1] = thrust_sp_body(1);
			local_pos_sp.thrust[2] = thrust_sp_body(2);
			local_pos_sp.yaw = yaw_sp;
			local_pos_sp.yawspeed = yawspeed_sp;
			_local_pos_sp_pub.publish(local_pos_sp);

			// ========================================
			// 7i. Publish thrust setpoint
			// ========================================
			vehicle_thrust_setpoint_s thrust_sp{};
			thrust_sp.timestamp = now;
			thrust_sp.timestamp_sample = local_pos.timestamp_sample;
			thrust_sp.xyz[0] = thrust_sp_body(0);
			thrust_sp.xyz[1] = thrust_sp_body(1);
			thrust_sp.xyz[2] = thrust_sp_body(2);
			_thrust_setpoint_pub.publish(thrust_sp);

			// ========================================
			// 7j. Publish attitude setpoint
			// ========================================
			vehicle_attitude_setpoint_s att_sp = computeAttitudeSetpoint(thrust_sp_body, yaw_sp, yawspeed_sp, tilt_limit_rad);
			att_sp.timestamp = now;
			_attitude_setpoint_pub.publish(att_sp);

		} else {
			// Not in position control mode - update takeoff state anyway
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed,
						    _vehicle_land_detected.landed,
						    false, 10.f, true,
						    local_pos.timestamp_sample);
			_vel_integral = Vector3f{};
			_pos_integral = Vector3f{};
		}

		// ========================================
		// 8. Publish takeoff status
		// ========================================
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !matrix::isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = now;
			_takeoff_status_pub.update();
		}

		// ========================================
		// 9. Debug output (1 Hz)
		// ========================================
		static hrt_abstime last_dbg_ts = 0;

		if (now - last_dbg_ts > DEBUG_INTERVAL_US) {
			last_dbg_ts = now;
			PX4_INFO("IFO: z=%.2f vz=%.2f vz_sp=%.2f evz=%.2f az_sp=%.2f iz=%.2f thr_z=%.3f hover=%.3f state=%d armed=%d",
				 (double)_position(2), (double)_velocity(2), (double)dbg_vz_sp, (double)dbg_vel_err_z,
				 (double)dbg_accel_sp_z, (double)_vel_integral(2), (double)dbg_thr_z, (double)_hover_thrust,
				 (int)takeoff_state, (int)_vehicle_control_mode.flag_armed);
		}
	}

	perf_end(_cycle_perf);
}

void IfodronePositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < local_pos.timestamp)) {
		// Adjust position setpoints
		if (local_pos.xy_reset_counter != _xy_reset_counter) {
			if (PX4_ISFINITE(setpoint.position[0])) {
				setpoint.position[0] += local_pos.delta_xy[0];
			}

			if (PX4_ISFINITE(setpoint.position[1])) {
				setpoint.position[1] += local_pos.delta_xy[1];
			}
		}

		if (local_pos.z_reset_counter != _z_reset_counter) {
			if (PX4_ISFINITE(setpoint.position[2])) {
				setpoint.position[2] += local_pos.delta_z;
			}
		}

		// Adjust velocity setpoints
		if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
			if (PX4_ISFINITE(setpoint.velocity[0])) {
				setpoint.velocity[0] += local_pos.delta_vxy[0];
			}

			if (PX4_ISFINITE(setpoint.velocity[1])) {
				setpoint.velocity[1] += local_pos.delta_vxy[1];
			}
		}

		if (local_pos.vz_reset_counter != _vz_reset_counter) {
			if (PX4_ISFINITE(setpoint.velocity[2])) {
				setpoint.velocity[2] += local_pos.delta_vz;
			}
		}

		// Adjust yaw setpoint
		if (local_pos.heading_reset_counter != _heading_reset_counter) {
			if (PX4_ISFINITE(setpoint.yaw)) {
				setpoint.yaw = matrix::wrap_pi(setpoint.yaw + local_pos.delta_heading);
			}
		}
	}

	// Save reset counters
	_xy_reset_counter = local_pos.xy_reset_counter;
	_z_reset_counter = local_pos.z_reset_counter;
	_vxy_reset_counter = local_pos.vxy_reset_counter;
	_vz_reset_counter = local_pos.vz_reset_counter;
	_heading_reset_counter = local_pos.heading_reset_counter;
}

trajectory_setpoint_s IfodronePositionControl::generateFailsafeSetpoint(const hrt_abstime &now, bool warn)
{
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints - failsafe");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint{};
	failsafe_setpoint.timestamp = now;
	failsafe_setpoint.position[0] = failsafe_setpoint.position[1] = failsafe_setpoint.position[2] = NAN;
	failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = failsafe_setpoint.velocity[2] = NAN;
	failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = failsafe_setpoint.acceleration[2] = NAN;
	failsafe_setpoint.yaw = NAN;
	failsafe_setpoint.yawspeed = NAN;

	if (PX4_ISFINITE(_velocity(0)) && PX4_ISFINITE(_velocity(1))) {
		// Can stop horizontally
		failsafe_setpoint.velocity[0] = 0.f;
		failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// Cant stop - descend
		failsafe_setpoint.acceleration[0] = 0.f;
		failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(_velocity(2))) {
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = 0.3f; // Slight downward acceleration

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

bool IfodronePositionControl::isSetpointValid(const trajectory_setpoint_s &setpoint) const
{
	return PX4_ISFINITE(setpoint.position[0]) || PX4_ISFINITE(setpoint.position[1]) || PX4_ISFINITE(setpoint.position[2])
	       || PX4_ISFINITE(setpoint.velocity[0]) || PX4_ISFINITE(setpoint.velocity[1]) || PX4_ISFINITE(setpoint.velocity[2])
	       || PX4_ISFINITE(setpoint.acceleration[0]) || PX4_ISFINITE(setpoint.acceleration[1])
	       || PX4_ISFINITE(setpoint.acceleration[2]);
}

// ============================================================
// USER CUSTOMIZATION - IMPLEMENT YOUR CONTROL ALGORITHMS HERE
// ============================================================

matrix::Vector3f IfodronePositionControl::computeVelocitySetpoint(const matrix::Vector3f &pos_sp,
		const matrix::Vector3f &pos,
		float speed_up,
		float speed_down,
		float dt)
{
	Vector3f vel_sp{};
	const Vector3f pos_err = pos_sp - pos;
	const bool valid_dt = PX4_ISFINITE(dt) && (dt > 0.f);

	const float vel_xy_limit = math::max(_param_mpc_xy_vel_max.get(), 0.f);
	const float vel_up_limit = math::max(speed_up, 0.f);
	const float vel_down_limit = math::max(speed_down, 0.f);

	const Vector3f vel_sp_min(-vel_xy_limit, -vel_xy_limit, -vel_up_limit);
	const Vector3f vel_sp_max(vel_xy_limit, vel_xy_limit, vel_down_limit);
	const Vector3f gain_p(_param_ifo_pos_xy_p.get(), _param_ifo_pos_xy_p.get(), _param_ifo_pos_z_p.get());
	const float gain_i = _param_ifo_pos_i.get();

	for (int axis = 0; axis < 3; axis++) {
		if (!PX4_ISFINITE(pos_err(axis)) || !PX4_ISFINITE(gain_p(axis))) {
			_pos_integral(axis) = 0.f;
			vel_sp(axis) = 0.f;
			continue;
		}

		const float p_term = pos_err(axis) * gain_p(axis);
		const float vel_unsat = p_term + _pos_integral(axis);

		if (valid_dt && (gain_i > FLT_EPSILON)) {
			const bool upper_saturated = (vel_unsat >= vel_sp_max(axis)) && (pos_err(axis) > 0.f);
			const bool lower_saturated = (vel_unsat <= vel_sp_min(axis)) && (pos_err(axis) < 0.f);

			if (!upper_saturated && !lower_saturated) {
				_pos_integral(axis) += pos_err(axis) * gain_i * dt;
			}

			_pos_integral(axis) = math::constrain(_pos_integral(axis),
							      vel_sp_min(axis) - p_term,
							      vel_sp_max(axis) - p_term);

		} else {
			_pos_integral(axis) = 0.f;
		}

		vel_sp(axis) = math::constrain(p_term + _pos_integral(axis), vel_sp_min(axis), vel_sp_max(axis));
	}

	return vel_sp;
}

matrix::Vector3f IfodronePositionControl::computeAccelerationSetpoint(const matrix::Vector3f &vel_sp,
		const matrix::Vector3f &vel,
		float dt)
{
	Vector3f accel_sp{};
	const Vector3f vel_err = vel_sp - vel;
	const bool valid_dt = PX4_ISFINITE(dt) && (dt > 0.f);

	const float gain_i = _param_ifo_vel_i.get();
	const Vector3f gain_p(_param_ifo_vel_xy_p.get(), _param_ifo_vel_xy_p.get(), _param_ifo_vel_z_p.get());

	const float thrust_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.f, 1.f);
	const float accel_xy_max = thrust_xy_max * CONSTANTS_ONE_G;

	const float thrust_max = math::max(_param_ifo_thr_max.get(), 0.f);
	const float thrust_min = math::constrain(_param_ifo_thr_min.get(), 0.f, thrust_max);
	const float hover = math::max(_hover_thrust, 0.05f);

	const float accel_z_from_thr_max = -(thrust_max - hover) * (CONSTANTS_ONE_G / hover);
	const float accel_z_from_thr_min = (hover - thrust_min) * (CONSTANTS_ONE_G / hover);
	const float accel_z_min = math::min(accel_z_from_thr_max, accel_z_from_thr_min);
	const float accel_z_max = math::max(accel_z_from_thr_max, accel_z_from_thr_min);

	const Vector3f accel_sp_min(-accel_xy_max, -accel_xy_max, accel_z_min);
	const Vector3f accel_sp_max(accel_xy_max, accel_xy_max, accel_z_max);

	for (int axis = 0; axis < 3; axis++) {
		if (!PX4_ISFINITE(vel_err(axis)) || !PX4_ISFINITE(gain_p(axis))) {
			_vel_integral(axis) = 0.f;
			accel_sp(axis) = 0.f;
			continue;
		}

		const float p_term = vel_err(axis) * gain_p(axis);
		const float accel_unsat = p_term + _vel_integral(axis);

		if (valid_dt && (gain_i > FLT_EPSILON)) {
			const bool upper_saturated = (accel_unsat >= accel_sp_max(axis)) && (vel_err(axis) > 0.f);
			const bool lower_saturated = (accel_unsat <= accel_sp_min(axis)) && (vel_err(axis) < 0.f);

			if (!upper_saturated && !lower_saturated) {
				_vel_integral(axis) += vel_err(axis) * gain_i * dt;
			}

			_vel_integral(axis) = math::constrain(_vel_integral(axis),
							      accel_sp_min(axis) - p_term,
							      accel_sp_max(axis) - p_term);

		} else {
			_vel_integral(axis) = 0.f;
		}

		accel_sp(axis) = math::constrain(p_term + _vel_integral(axis), accel_sp_min(axis), accel_sp_max(axis));
	}

	return accel_sp;
}

matrix::Vector3f IfodronePositionControl::computeIfodroneThrust(const matrix::Vector3f &accel_sp, float yaw)
{
	// IFODRONE-specific thrust generation:
	// - thrust[0,1]: XY tilt motors (controls horizontal position)
	// - thrust[2]: Main motors (controls altitude)
	//
	// Input accel_sp is in NED frame (positive Z is down)
	// NOTE: For IFODRONE, we do NOT rotate by yaw - the mixer/allocation
	// expects NED-frame thrust commands which it then routes to tilt motors.

	(void)yaw; // Not used for IFODRONE - tilt motors handle body rotation

	Vector3f thrust_body;

	// XY thrust (tilt motors) - convert acceleration to normalized thrust
	// Direct mapping without yaw rotation (matches original working implementation)
	thrust_body(0) = accel_sp(0) / CONSTANTS_ONE_G;
	thrust_body(1) = accel_sp(1) / CONSTANTS_ONE_G;

	// Limit XY thrust
	const float thrust_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.0f, 1.0f);
	Vector2f thrust_xy(thrust_body(0), thrust_body(1));
	const float thrust_xy_norm = thrust_xy.norm();

	if (thrust_xy_norm > thrust_xy_max && thrust_xy_norm > FLT_EPSILON) {
		thrust_xy *= thrust_xy_max / thrust_xy_norm;
		thrust_body(0) = thrust_xy(0);
		thrust_body(1) = thrust_xy(1);
	}

	// Z thrust (main motors)
	// NED: negative Z is up, so we need negative thrust for upward force
	// accel_sp(2) positive = downward accel = less thrust needed
	// accel_sp(2) negative = upward accel = more thrust needed
	const float thrust_z = _hover_thrust - accel_sp(2) * (_hover_thrust / CONSTANTS_ONE_G);
	thrust_body(2) = -math::constrain(thrust_z, _param_ifo_thr_min.get(), _param_ifo_thr_max.get());

	return thrust_body;
}

vehicle_attitude_setpoint_s IfodronePositionControl::computeAttitudeSetpoint(const matrix::Vector3f &thrust_body,
		float yaw_sp, float yawspeed_sp, float tilt_limit_rad)
{
	// TODO: Implement IFODRONE attitude setpoint generation
	//
	// For IFODRONE, roll and pitch may not be used for position control
	// (since tilt motors handle XY). This may just be yaw control.
	// tilt_limit_rad can be used to constrain XY thrust if needed.
	(void)tilt_limit_rad;  // Currently unused for IFODRONE, but available for constraints

	vehicle_attitude_setpoint_s att_sp{};

	// Simple yaw-only attitude (level flight)
	const Quatf q_sp(Eulerf(0.0f, 0.0f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);

	att_sp.yaw_sp_move_rate = yawspeed_sp;

	// Pass through thrust
	att_sp.thrust_body[0] = thrust_body(0);
	att_sp.thrust_body[1] = thrust_body(1);
	att_sp.thrust_body[2] = thrust_body(2);

	return att_sp;
}

// ============================================================
// Module interface
// ============================================================

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

int IfodronePositionControl::print_status()
{
	PX4_INFO("IFODRONE Position Controller");
	PX4_INFO("  Takeoff state: %d", (int)_takeoff.getTakeoffState());
	PX4_INFO("  Position: %.2f, %.2f, %.2f", (double)_position(0), (double)_position(1), (double)_position(2));
	PX4_INFO("  Hover thrust: %.3f", (double)_hover_thrust);
	return 0;
}

int IfodronePositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IFODRONE position controller with custom thrust allocation.
Based on MulticopterPositionControl state machine.

Uses TakeoffHandling for takeoff state machine.
Custom thrust generation for IFODRONE airframe.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
