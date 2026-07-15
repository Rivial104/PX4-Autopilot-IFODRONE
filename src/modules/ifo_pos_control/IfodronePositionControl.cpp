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
 *   - Manual/Position/Hold (trajectory_setpoint from flight_mode_manager)
 *   - goto_setpoint (direct position target, converted here)
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

		// --- Run position control ---
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_vehicle_control_mode.flag_control_manual_enabled
			|| _setpoint.timestamp >= _time_position_control_enabled)) {

			if (_vehicle_control_mode.flag_control_manual_enabled) {
				// -------------------------------------------------------
				// IFODRONE HOLD MODE
				//   XY:       position hold via PID
				//   altitude: throttle stick → vertical velocity setpoint
				//   yaw:      yaw stick → yaw rate
				// -------------------------------------------------------
				manual_control_setpoint_s manual_sp{};
				_manual_control_setpoint_sub.copy(&manual_sp);

				// Initialise heading hold on first entry
				if (!_hold_initialized) {
					_hold_yaw_angle = PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f;
					_control.resetIntegral();
					_hold_initialized = true;
				}

				// --- Manual stick mapping → velocity setpoints ----------------------
				//   roll  stick → body +Y velocity (right)
				//   pitch stick → body +X velocity (forward)
				//   throttle    → vertical velocity (centre = hold altitude)
				//   yaw   stick → yaw rate
				// Velocity-only: sticks centred ⇒ zero velocity (no XY position hold).
				// NOTE sign-sensitive: if a stick drives the wrong way, flip its sign here.
				static constexpr float STICK_DEADBAND = 0.1f;
				auto deadband = [](float v) -> float {
					if (fabsf(v) <= STICK_DEADBAND) { return 0.f; }
					const float s = (v > 0.f) ? 1.f : -1.f;
					return (v - s * STICK_DEADBAND) / (1.f - STICK_DEADBAND);
				};

				const float vx_body = deadband(manual_sp.pitch) * _param_ifo_vel_max_xy.get();
				const float vy_body = deadband(manual_sp.roll)  * _param_ifo_vel_max_xy.get();

				const float thr_db  = deadband(manual_sp.throttle);
				const float vel_z_sp = (thr_db > 0.f) ? -thr_db * _param_ifo_vel_max_up.get()
								      : -thr_db * _param_ifo_vel_max_dn.get();

				// Yaw stick → integrate heading setpoint
				static constexpr float YAW_RATE_MAX = 1.5f;
				_hold_yaw_angle = wrap_pi(_hold_yaw_angle + manual_sp.yaw * YAW_RATE_MAX * dt);

				// Rotate body-frame XY velocity into NED by heading
				const float cy = cosf(_hold_yaw_angle);
				const float sy = sinf(_hold_yaw_angle);
				const float vx_ned = cy * vx_body - sy * vy_body;
				const float vy_ned = sy * vx_body + cy * vy_body;

				// Build trajectory setpoint: velocity-only (XY + Z), no position hold
				trajectory_setpoint_s hold_sp = PositionControl::empty_trajectory_setpoint;
				hold_sp.timestamp   = local_pos.timestamp_sample;
				hold_sp.position[0] = NAN;
				hold_sp.position[1] = NAN;
				hold_sp.position[2] = NAN;
				hold_sp.velocity[0] = vx_ned;
				hold_sp.velocity[1] = vy_ned;
				hold_sp.velocity[2] = vel_z_sp;
				hold_sp.yaw         = _hold_yaw_angle;

				// Run PID
				_control.setVelocityLimits(
					_param_ifo_vel_max_xy.get(),
					_param_ifo_vel_max_up.get(),
					_param_ifo_vel_max_dn.get());
				_control.setThrustLimits(_param_ifo_thr_min.get(), _param_ifo_thr_max.get());

				if (!_hold_initialized || _vehicle_land_detected.ground_contact) {
					// On ground or not yet latched: push down, no integral
					_control.resetIntegral();
					trajectory_setpoint_s ground_sp = PositionControl::empty_trajectory_setpoint;
					ground_sp.timestamp = local_pos.timestamp_sample;
					Vector3f(0.f, 0.f, 100.f).copyTo(ground_sp.acceleration);
					_control.setInputSetpoint(ground_sp);

				} else {
					_control.setInputSetpoint(hold_sp);
				}

				_control.setState(states);
				_control.update(dt);

				publishSetpoints(states);

			} else {
				// -------------------------------------------------------
				// POSITION / OFFBOARD CONTROL (PID loop)
				// -------------------------------------------------------
				_hold_initialized = false; // reset for next hold entry

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
