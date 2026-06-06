/**
 * IFODRONE Position Controller
 *
 * Structure modeled after mc_pos_control but simplified for the IFODRONE
 * airframe where position and orientation are decoupled.
 *
 * Uses the PositionControl library for cascaded P-position + PID-velocity
 * to produce an acceleration setpoint, then converts acceleration to
 * body-frame thrust using the current attitude.
 *
 * Supports:
 *   - Offboard mode (external trajectory_setpoint)
 *   - Manual/Position/Hold (trajectory_setpoint from flight_mode_manager)
 *   - goto_setpoint (direct position target, converted here)
 */

#include "IfodronePositionControl.hpp"

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
				//   XY:       direct velocity setpoint, no local position latch
				//   altitude: throttle stick → vertical velocity setpoint
				//   yaw:      yaw stick → yaw rate
				// -------------------------------------------------------
				manual_control_setpoint_s manual_sp{};
				_manual_control_setpoint_sub.copy(&manual_sp);

				// Initialise on first entry into hold/POSCTL.
				if (!_hold_initialized) {
					_hold_yaw_angle = PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f;
					_hold_z = NAN;
					_control.resetIntegral();
					_hold_initialized = true;
				}

				// Throttle stick [-1,+1] → vertical velocity (NED: negative = up)
				// Centre stick (0) → hold altitude (vel = 0)
				static constexpr float THROTTLE_DEADBAND = 0.1f;
				float vel_z_sp = 0.0f;
				const float throttle = manual_sp.throttle;

				if (fabsf(throttle) > THROTTLE_DEADBAND) {
					const float t = (fabsf(throttle) - THROTTLE_DEADBAND) / (1.f - THROTTLE_DEADBAND);
					vel_z_sp = (throttle > 0.f) ? -t * _param_ifo_vel_max_up.get()
								    :  t * _param_ifo_vel_max_dn.get();
				}

				// Yaw stick → integrate hold yaw
				static constexpr float YAW_RATE_MAX = 1.5f;
				_hold_yaw_angle = wrap_pi(_hold_yaw_angle + manual_sp.yaw * YAW_RATE_MAX * dt);

				// Build trajectory setpoint.
				// XY: pass flight_mode_manager setpoint through directly.
				//     FlightTaskManualPositionSmooth publishes position_sp when stick
				//     released (position latch) and velocity_sp when stick is pushed.
				// Z:  latch altitude when throttle is in deadband; velocity otherwise.
				trajectory_setpoint_s hold_sp = PositionControl::empty_trajectory_setpoint;
				hold_sp.timestamp = local_pos.timestamp_sample;

				hold_sp.position[0]     = _setpoint.position[0];
				hold_sp.position[1]     = _setpoint.position[1];
				hold_sp.velocity[0]     = _setpoint.velocity[0];
				hold_sp.velocity[1]     = _setpoint.velocity[1];
				hold_sp.acceleration[0] = _setpoint.acceleration[0];
				hold_sp.acceleration[1] = _setpoint.acceleration[1];

				// Fallback: if flight_mode_manager hasn't sent an XY setpoint yet, hold current position.
				if (!PX4_ISFINITE(hold_sp.position[0]) && !PX4_ISFINITE(hold_sp.velocity[0])) {
					if (PX4_ISFINITE(states.position(0)) && PX4_ISFINITE(states.position(1))) {
						hold_sp.position[0] = states.position(0);
						hold_sp.position[1] = states.position(1);
					} else {
						hold_sp.velocity[0] = 0.f;
						hold_sp.velocity[1] = 0.f;
					}
				}

				// Z: altitude latch when throttle at centre, velocity when pushed.
				if (fabsf(vel_z_sp) < 1e-5f) {
					if (!PX4_ISFINITE(_hold_z)) {
						_hold_z = PX4_ISFINITE(states.position(2)) ? states.position(2) : NAN;
					}
					hold_sp.position[2] = _hold_z;
					hold_sp.velocity[2] = NAN;
				} else {
					_hold_z = NAN;
					hold_sp.position[2] = NAN;
					hold_sp.velocity[2] = vel_z_sp;
				}

				hold_sp.yaw = _hold_yaw_angle;

				// Run PID
				_control.setVelocityLimits(
					FLT_MAX,
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

				// Publish local position setpoint
				vehicle_local_position_setpoint_s local_pos_sp{};
				_control.getLocalPositionSetpoint(local_pos_sp);
				local_pos_sp.timestamp = hrt_absolute_time();
				_local_pos_sp_pub.publish(local_pos_sp);

				// Convert acceleration setpoint → body-frame thrust
				Vector3f acc_sp(
					local_pos_sp.acceleration[0],
					local_pos_sp.acceleration[1],
					local_pos_sp.acceleration[2]);

				for (int i = 0; i < 3; i++) {
					if (!PX4_ISFINITE(acc_sp(i))) { acc_sp(i) = 0.f; }
				}

				const Vector3f thr_body = accelerationToThrust(acc_sp);

				vehicle_thrust_setpoint_s thrust_msg{};
				thrust_msg.timestamp        = hrt_absolute_time();
				thrust_msg.timestamp_sample = local_pos.timestamp_sample;
				thrust_msg.xyz[0] = thr_body(0);
				thrust_msg.xyz[1] = thr_body(1);
				thrust_msg.xyz[2] = thr_body(2);
				_thrust_sp_pub.publish(thrust_msg);

				// Publish attitude setpoint: level body, yaw from hold state.
				// Thrust is published separately on vehicle_thrust_setpoint for CA.
				vehicle_attitude_setpoint_s att_sp{};
				att_sp.timestamp        = hrt_absolute_time();
				att_sp.yaw_sp_move_rate = 0.0f;
				const Quatf q_sp(Eulerf(0.0f, 0.0f, _hold_yaw_angle));
				q_sp.copyTo(att_sp.q_d);
				_attitude_setpoint_pub.publish(att_sp);

			} else {
				// -------------------------------------------------------
				// POSITION / OFFBOARD CONTROL (PID loop)
				// -------------------------------------------------------
				_hold_initialized = false;
				_hold_z = NAN;

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

				// --- Get library outputs ---
				vehicle_local_position_setpoint_s local_pos_sp{};
				_control.getLocalPositionSetpoint(local_pos_sp);
				local_pos_sp.timestamp = hrt_absolute_time();
				_local_pos_sp_pub.publish(local_pos_sp);

				Vector3f acc_sp(
					local_pos_sp.acceleration[0],
					local_pos_sp.acceleration[1],
					local_pos_sp.acceleration[2]
				);

				for (int i = 0; i < 3; i++) {
					if (!PX4_ISFINITE(acc_sp(i))) {
						acc_sp(i) = 0.f;
					}
				}

				const float yaw_sp = PX4_ISFINITE(local_pos_sp.yaw) ? local_pos_sp.yaw : states.yaw;
				const float yawspeed_sp = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;

				const Vector3f thr_body = accelerationToThrust(acc_sp);

				vehicle_thrust_setpoint_s thrust_msg{};
				thrust_msg.timestamp        = hrt_absolute_time();
				thrust_msg.timestamp_sample = local_pos.timestamp_sample;
				thrust_msg.xyz[0] = thr_body(0);
				thrust_msg.xyz[1] = thr_body(1);
				thrust_msg.xyz[2] = thr_body(2);
				_thrust_sp_pub.publish(thrust_msg);

				// --- Publish attitude setpoint (yaw only, level body) ---
				// Thrust is published separately on vehicle_thrust_setpoint for CA.
				vehicle_attitude_setpoint_s att_sp{};
				att_sp.timestamp = hrt_absolute_time();
				att_sp.yaw_sp_move_rate = yawspeed_sp;
				const Quatf q_sp(Eulerf(0.0f, 0.0f, yaw_sp));
				q_sp.copyTo(att_sp.q_d);
				_attitude_setpoint_pub.publish(att_sp);


			}

		} else {
			// Not in position control mode: update takeoff state machine but do nothing
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    false, 10.f, true, local_pos.timestamp_sample);
			_control.resetIntegral();
			_hold_initialized = false;
			_hold_z = NAN;
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

matrix::Vector3f IfodronePositionControl::accelerationToThrust(const Vector3f &acc_sp) const
{
	const float hover_thr  = math::constrain(_param_ifo_thr_hover.get(), 0.05f, 0.9f);
	const float thr_min    = math::constrain(_param_ifo_thr_min.get(), 0.0f, 0.9f);
	const float thr_max    = math::constrain(_param_ifo_thr_max.get(), thr_min, 1.0f);
	const float thr_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.0f, 1.0f);
	const float scale      = hover_thr / CONSTANTS_ONE_G;

	// IFODRONE body is always level — no attitude rotation.
	// Using the actual attitude rotation here projects Z-thrust onto body XY whenever
	// the platform tilts even slightly, creating a feedback loop: tilt → spurious XY
	// command → side EDF activation → more tilt.
	// Treating body XY = NED XY breaks that loop.

	// Z: hover baseline ± vertical correction
	const float thrust_z = math::constrain(hover_thr - acc_sp(2) * scale, thr_min, thr_max);

	// XY: direct proportional scaling, NED ≈ body for a level platform
	Vector2f thr_xy(acc_sp(0) * scale, acc_sp(1) * scale);
	const float thr_xy_norm = thr_xy.norm();

	if (thr_xy_norm > thr_xy_max && thr_xy_norm > 1e-5f) {
		thr_xy *= thr_xy_max / thr_xy_norm;
	}

	return Vector3f(thr_xy(0), thr_xy(1), -thrust_z);
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
