/**
 * IFODRONE Position Controller
 *
 * Full 3D position controller:
 * - Z-axis: Main motors (0-1) for altitude (thrust)
 * - XY-axis: Tilt servos (0-3) for horizontal acceleration
 *
 * Architecture (like mc_pos_control/PositionControl):
 * 1. Position P-controller: pos_error → vel_sp
 * 2. Velocity PID-controller: vel_error → acc_sp
 * 3. Acceleration control: acc_sp → thrust (Z) and tilt angles (XY)
 *
 * Key assumptions:
 * - tilt_front = -tilt_back (zeroes Z force component from tilts)
 * - tilt_right = -tilt_left (zeroes Z force component from tilts)
 * - Tilts generate horizontal acceleration without affecting Z thrust
 */

#include "IfodronePositionControl.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <geo/geo.h>

using namespace matrix;

IfodronePositionControl::IfodronePositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// Initialize takeoff handling
	_takeoff.setSpoolupTime(1.0f);
	_takeoff.setTakeoffRampTime(2.0f);
	_takeoff.generateInitialRampValue(_param_ifo_vel_z_p.get());

	// Initialize setpoints to NAN (like PositionControl)
	_pos_sp.setNaN();
	_vel_sp.setNaN();
	_acc_sp.setNaN();
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

	PX4_INFO("IFODRONE position controller initialized");
	return true;
}

void IfodronePositionControl::_resetIntegrals()
{
	_vel_int.setZero();
}

void IfodronePositionControl::_positionControl()
{
	// ================================================================
	// P-POSITION CONTROLLER
	// vel_sp_from_pos = (pos_sp - pos) * Kp_pos
	// Like PositionControl::_positionControl()
	// ================================================================

	Vector3f gain_pos_p(_param_ifo_pos_xy_p.get(),
			    _param_ifo_pos_xy_p.get(),
			    _param_ifo_pos_z_p.get());

	// P-controller: position error → velocity setpoint
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(gain_pos_p);

	// Add position-derived velocity to feedforward velocity setpoint
	// If pos_sp is NAN, vel_sp_position will be NAN and won't affect _vel_sp
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(vel_sp_position(i))) {
			if (PX4_ISFINITE(_vel_sp(i))) {
				_vel_sp(i) += vel_sp_position(i);
			} else {
				_vel_sp(i) = vel_sp_position(i);
			}
		}
	}

	// Constrain velocity setpoints
	// Horizontal velocity
	const float vel_xy_norm = Vector2f(_vel_sp).norm();
	if (vel_xy_norm > _param_ifo_vel_xy_max.get()) {
		_vel_sp.xy() = Vector2f(_vel_sp) / vel_xy_norm * _param_ifo_vel_xy_max.get();
	}

	// Vertical velocity
	_vel_sp(2) = math::constrain(_vel_sp(2), -_param_ifo_vel_z_up.get(), _param_ifo_vel_z_dn.get());
}

void IfodronePositionControl::_velocityControl(float dt)
{
	// ================================================================
	// PID-VELOCITY CONTROLLER
	// acc_sp = vel_error * Kp_vel + integral - vel_dot * Kd_vel
	// Like PositionControl::_velocityControl()
	// ================================================================

	Vector3f gain_vel_p(_param_ifo_vel_xy_p.get(),
			    _param_ifo_vel_xy_p.get(),
			    _param_ifo_vel_z_p.get());

	Vector3f gain_vel_i(_param_ifo_vel_xy_i.get(),
			    _param_ifo_vel_xy_i.get(),
			    _param_ifo_vel_z_i.get());

	Vector3f gain_vel_d(_param_ifo_vel_xy_d.get(),
			    _param_ifo_vel_xy_d.get(),
			    _param_ifo_vel_z_d.get());

	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// Velocity error
	Vector3f vel_error = _vel_sp - _vel;

	// Handle NAN in vel_sp (no velocity control on that axis)
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_vel_sp(i))) {
			vel_error(i) = 0.0f;
		}
	}

	// PID: acc_sp = P * vel_error + I * integral - D * vel_dot
	Vector3f acc_sp_velocity = vel_error.emult(gain_vel_p) + _vel_int - _vel_dot.emult(gain_vel_d);

	// Add to acceleration setpoint (feedforward if available)
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(acc_sp_velocity(i))) {
			if (PX4_ISFINITE(_acc_sp(i))) {
				_acc_sp(i) += acc_sp_velocity(i);
			} else {
				_acc_sp(i) = acc_sp_velocity(i);
			}
		}
	}

	// Ensure acc_sp is finite
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_acc_sp(i))) {
			_acc_sp(i) = 0.0f;
		}
	}

	// Convert acceleration to thrust and tilts
	_accelerationControl();

	// ================================================================
	// INTEGRATOR ANTI-WINDUP
	// ================================================================

	// Vertical anti-windup: don't integrate if thrust saturated
	if ((_thr_sp(2) >= -_param_ifo_thr_min.get() && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_param_ifo_thr_max.get() && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Horizontal anti-windup: don't integrate if tilts saturated
	const float tilt_norm = sqrtf(_tilt_pitch * _tilt_pitch + _tilt_roll * _tilt_roll);
	const float tilt_max = _param_ifo_tilt_max.get();
	if (tilt_norm > tilt_max * 0.95f) {
		vel_error(0) = 0.f;
		vel_error(1) = 0.f;
	}

	// Update integral
	_vel_int += vel_error.emult(gain_vel_i) * dt;

	// Limit integral
	_vel_int(0) = math::constrain(_vel_int(0), -5.0f, 5.0f);
	_vel_int(1) = math::constrain(_vel_int(1), -5.0f, 5.0f);
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
}

void IfodronePositionControl::_accelerationControl()
{
	// ================================================================
	// ACCELERATION → THRUST (Z) AND TILT ANGLES (XY)
	//
	// For IFODRONE:
	// - Z acceleration → main motor thrust
	// - XY acceleration → tilt angles of side motors
	//
	// Key assumption: tilt_front = -tilt_back, tilt_right = -tilt_left
	// This zeroes the Z component from tilts and only generates XY force
	// ================================================================

	// --- Z-AXIS: Thrust from main motors ---
	// thrust = hover_thrust - acc_z * (hover_thrust / g)
	// In NED: negative acc_z = upward acceleration = more thrust
	const float thrust_scale = _param_ifo_thr_hover.get() / CONSTANTS_ONE_G;
	const float thrust_z_raw = _param_ifo_thr_hover.get() - _acc_sp(2) * thrust_scale;

	// Constrain thrust
	_thr_sp(2) = -math::constrain(thrust_z_raw, _param_ifo_thr_min.get(), _param_ifo_thr_max.get());

	// --- XY-AXIS: Tilt angles from side motors ---
	// For small angles: tan(tilt) ≈ tilt = a_horizontal / g
	// tilt = acc_sp * (1/g)
	//
	// Convert world frame acceleration to body frame (rotate by -yaw)
	const float cos_yaw = cosf(_yaw);
	const float sin_yaw = sinf(_yaw);

	// Body frame acceleration from world frame
	// acc_body_x = cos(yaw) * acc_world_x + sin(yaw) * acc_world_y
	// acc_body_y = -sin(yaw) * acc_world_x + cos(yaw) * acc_world_y
	const float acc_body_x = cos_yaw * _acc_sp(0) + sin_yaw * _acc_sp(1);
	const float acc_body_y = -sin_yaw * _acc_sp(0) + cos_yaw * _acc_sp(1);

	// Tilt angles (radians)
	// Positive tilt_pitch = tilt forward = positive X acceleration
	// Positive tilt_roll = tilt right = positive Y acceleration
	const float tilt_scale = 1.0f / CONSTANTS_ONE_G;
	float tilt_pitch_raw = acc_body_x * tilt_scale;
	float tilt_roll_raw = acc_body_y * tilt_scale;

	// Constrain individual tilts
	const float tilt_max = _param_ifo_tilt_max.get();
	_tilt_pitch = math::constrain(tilt_pitch_raw, -tilt_max, tilt_max);
	_tilt_roll = math::constrain(tilt_roll_raw, -tilt_max, tilt_max);

	// Also set horizontal thrust setpoint (for compatibility)
	// In body frame
	_thr_sp(0) = _tilt_pitch * _param_ifo_thr_hover.get();  // Approximate
	_thr_sp(1) = _tilt_roll * _param_ifo_thr_hover.get();
}

bool IfodronePositionControl::_inputValid()
{
	// Check if we have valid position/velocity estimates
	bool valid = true;

	// At least one setpoint must be valid for each axis
	for (int i = 0; i < 3; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// For controlled states, estimate must be valid
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}
		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void IfodronePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Get current time
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.002f, 0.04f);
	_last_run = now;

	// Parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// ================================================================
	// GET CURRENT STATE
	// ================================================================

	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);

	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	vehicle_local_position_s local_pos{};
	if (!_local_pos_sub.copy(&local_pos)) {
		perf_end(_cycle_perf);
		return;
	}

	// Set current state (like PositionControl::setState)
	_pos = Vector3f(local_pos.x, local_pos.y, local_pos.z);
	_vel = Vector3f(local_pos.vx, local_pos.vy, local_pos.vz);
	_vel_dot = Vector3f(local_pos.ax, local_pos.ay, local_pos.az);

	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const Quatf q_current(attitude.q);
	_yaw = Eulerf(q_current).psi();

	// ================================================================
	// GET SETPOINTS
	// ================================================================

	trajectory_setpoint_s traj_sp{};
	const bool has_trajectory = _trajectory_setpoint_sub.copy(&traj_sp);

	manual_control_setpoint_s manual_control{};
	const bool has_manual = _manual_control_sub.copy(&manual_control);

	// ================================================================
	// TAKEOFF STATE MACHINE
	// ================================================================

	const float takeoff_desired_vz = -1.0f;  // 1 m/s up
	const bool want_takeoff = control_mode.flag_armed &&
				  (control_mode.flag_control_altitude_enabled ||
				   control_mode.flag_control_position_enabled);

	_takeoff.updateTakeoffState(
		control_mode.flag_armed,
		land_detected.landed,
		want_takeoff,
		takeoff_desired_vz,
		false,
		now
	);

	const TakeoffState takeoff_state = _takeoff.getTakeoffState();
	const float velocity_limit = _takeoff.updateRamp(dt, takeoff_desired_vz);

	// ================================================================
	// RESET WHEN DISARMED OR LANDED
	// ================================================================

	if (!control_mode.flag_armed || land_detected.landed) {
		_hold_position_initialized = false;
		_resetIntegrals();
	}

	// ================================================================
	// DETERMINE SETPOINTS
	// ================================================================

	// Reset setpoints each cycle
	_pos_sp.setNaN();
	_vel_sp.setNaN();
	_acc_sp.setNaN();

	const bool run_control = control_mode.flag_armed &&
				 (control_mode.flag_control_altitude_enabled ||
				  control_mode.flag_control_position_enabled);

	const bool in_flight = (takeoff_state == TakeoffState::rampup ||
				takeoff_state == TakeoffState::flight);

	if (run_control && in_flight && local_pos.z_valid && local_pos.v_z_valid) {

		const bool manual_mode = control_mode.flag_control_manual_enabled;
		const bool trajectory_valid = has_trajectory &&
					      (PX4_ISFINITE(traj_sp.position[0]) ||
					       PX4_ISFINITE(traj_sp.velocity[0]) ||
					       PX4_ISFINITE(traj_sp.position[2]));

		// Initialize hold position
		if (!_hold_position_initialized && local_pos.xy_valid) {
			_hold_x = local_pos.x;
			_hold_y = local_pos.y;
			_hold_z = local_pos.z;
			_hold_position_initialized = true;
			PX4_INFO("Hold: (%.1f, %.1f, %.1f)", (double)_hold_x, (double)_hold_y, (double)_hold_z);
		}

		// --- Set position/velocity/acceleration setpoints ---
		if (trajectory_valid) {
			// From trajectory (Mission/Auto mode)
			_pos_sp = Vector3f(traj_sp.position);
			_vel_sp = Vector3f(traj_sp.velocity);
			_acc_sp = Vector3f(traj_sp.acceleration);
			_yaw_sp = PX4_ISFINITE(traj_sp.yaw) ? traj_sp.yaw : _yaw;

			// Update hold position
			if (PX4_ISFINITE(traj_sp.position[0])) _hold_x = traj_sp.position[0];
			if (PX4_ISFINITE(traj_sp.position[1])) _hold_y = traj_sp.position[1];
			if (PX4_ISFINITE(traj_sp.position[2])) _hold_z = traj_sp.position[2];

		} else if (manual_mode && has_manual) {
			// Manual control: sticks → velocity setpoints
			const float max_vel_xy = _param_ifo_vel_xy_max.get();
			const float max_vel_z = 2.0f;

			// XY velocity from sticks (body frame → world frame)
			const float pitch_input = fabsf(manual_control.pitch) > 0.05f ? manual_control.pitch : 0.0f;
			const float roll_input = fabsf(manual_control.roll) > 0.05f ? manual_control.roll : 0.0f;

			const float vx_body = pitch_input * max_vel_xy;
			const float vy_body = roll_input * max_vel_xy;

			const float cos_yaw = cosf(_yaw);
			const float sin_yaw = sinf(_yaw);
			float vx_sp = cos_yaw * vx_body - sin_yaw * vy_body;
			float vy_sp = sin_yaw * vx_body + cos_yaw * vy_body;

			// Z velocity from throttle
			float vz_sp = -manual_control.throttle * max_vel_z;

			// Position hold when sticks centered
			if (fabsf(vx_sp) < 0.1f && fabsf(vy_sp) < 0.1f) {
				_pos_sp(0) = _hold_x;
				_pos_sp(1) = _hold_y;
				vx_sp = 0.0f;
				vy_sp = 0.0f;
			} else {
				_hold_x = local_pos.x;
				_hold_y = local_pos.y;
			}

			if (fabsf(vz_sp) < 0.1f) {
				_pos_sp(2) = _hold_z;
				vz_sp = 0.0f;
			} else {
				_hold_z = local_pos.z;
			}

			_vel_sp = Vector3f(vx_sp, vy_sp, vz_sp);
			_yaw_sp = _yaw;  // Keep current yaw

		} else {
			// No input: position hold
			_pos_sp = Vector3f(_hold_x, _hold_y, _hold_z);
			_vel_sp = Vector3f(0.0f, 0.0f, 0.0f);
			_yaw_sp = _yaw;
		}

		// Apply takeoff velocity limit
		if (PX4_ISFINITE(_vel_sp(2)) && _vel_sp(2) < velocity_limit) {
			_vel_sp(2) = velocity_limit;
		}

		// ================================================================
		// RUN CONTROL CASCADE
		// ================================================================

		if (_inputValid()) {
			_positionControl();
			_velocityControl(dt);
		}

		// Debug output
		static int counter = 0;
		if (++counter >= 250) {
			counter = 0;
			PX4_INFO("POS: (%.1f,%.1f,%.1f) SP: (%.1f,%.1f,%.1f) TILT: (%.1f,%.1f) THR: %.2f",
				 (double)_pos(0), (double)_pos(1), (double)_pos(2),
				 (double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
				 (double)math::degrees(_tilt_pitch), (double)math::degrees(_tilt_roll),
				 (double)(-_thr_sp(2)));
		}

	} else if (control_mode.flag_armed && takeoff_state == TakeoffState::spoolup) {
		// Spoolup: minimal thrust
		_thr_sp(2) = -0.1f;
		_tilt_pitch = 0.0f;
		_tilt_roll = 0.0f;

	} else if (control_mode.flag_armed && takeoff_state == TakeoffState::ready_for_takeoff) {
		// Ready: slightly more thrust
		_thr_sp(2) = -0.15f;
		_tilt_pitch = 0.0f;
		_tilt_roll = 0.0f;

	} else {
		// Disarmed or unknown
		_thr_sp.setZero();
		_tilt_pitch = 0.0f;
		_tilt_roll = 0.0f;
	}

	// ================================================================
	// PUBLISH OUTPUTS
	// ================================================================

	// --- Thrust setpoint (for control allocator) ---
	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = now;
	thrust_sp.timestamp_sample = local_pos.timestamp;
	thrust_sp.xyz[0] = _thr_sp(0);
	thrust_sp.xyz[1] = _thr_sp(1);
	thrust_sp.xyz[2] = _thr_sp(2);  // Already negative for up
	_thrust_setpoint_pub.publish(thrust_sp);

	// --- Torque setpoint (for attitude stabilization) ---
	// From attitude controller, not position controller
	// Set to zero here, attitude controller will override
	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = now;
	torque_sp.timestamp_sample = local_pos.timestamp;
	torque_sp.xyz[0] = 0.0f;  // Roll
	torque_sp.xyz[1] = 0.0f;  // Pitch
	torque_sp.xyz[2] = 0.0f;  // Yaw
	_torque_setpoint_pub.publish(torque_sp);

	// --- Tilt controls (for side motor servos) ---
	// tilt_front (0) = -tilt_back (2) = _tilt_pitch
	// tilt_right (1) = -tilt_left (3) = _tilt_roll
	tiltrotor_extra_controls_s tilt_ctrl{};
	tilt_ctrl.timestamp = now;
	tilt_ctrl.tilt_front = _tilt_pitch;
	tilt_ctrl.tilt_right = _tilt_roll;
	tilt_ctrl.tilt_back = -_tilt_pitch;  // Opposite for zero Z force
	tilt_ctrl.tilt_left = -_tilt_roll;   // Opposite for zero Z force
	_tilt_controls_pub.publish(tilt_ctrl);

	// --- Attitude setpoint (level attitude with yaw) ---
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.yaw_sp_move_rate = 0.0f;

	// Level attitude (roll=0, pitch=0) with desired yaw
	const Quatf q_sp = Quatf(Eulerf(0.0f, 0.0f, _yaw_sp));
	q_sp.copyTo(att_sp.q_d);

	// Thrust in body frame
	att_sp.thrust_body[0] = _thr_sp(0);
	att_sp.thrust_body[1] = _thr_sp(1);
	att_sp.thrust_body[2] = _thr_sp(2);
	_attitude_setpoint_pub.publish(att_sp);

	// --- Local position setpoint ---
	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.x = _pos_sp(0);
	local_pos_sp.y = _pos_sp(1);
	local_pos_sp.z = _pos_sp(2);
	local_pos_sp.vx = _vel_sp(0);
	local_pos_sp.vy = _vel_sp(1);
	local_pos_sp.vz = _vel_sp(2);
	local_pos_sp.acceleration[0] = _acc_sp(0);
	local_pos_sp.acceleration[1] = _acc_sp(1);
	local_pos_sp.acceleration[2] = _acc_sp(2);
	local_pos_sp.thrust[0] = _thr_sp(0);
	local_pos_sp.thrust[1] = _thr_sp(1);
	local_pos_sp.thrust[2] = _thr_sp(2);
	_local_pos_sp_pub.publish(local_pos_sp);

	perf_end(_cycle_perf);
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

int IfodronePositionControl::print_status()
{
	PX4_INFO("IFODRONE Position Controller");
	PX4_INFO("  Pos SP: (%.2f, %.2f, %.2f)", (double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2));
	PX4_INFO("  Vel SP: (%.2f, %.2f, %.2f)", (double)_vel_sp(0), (double)_vel_sp(1), (double)_vel_sp(2));
	PX4_INFO("  Acc SP: (%.2f, %.2f, %.2f)", (double)_acc_sp(0), (double)_acc_sp(1), (double)_acc_sp(2));
	PX4_INFO("  Thrust: %.2f", (double)(-_thr_sp(2)));
	PX4_INFO("  Tilts: pitch=%.1f° roll=%.1f°",
		 (double)math::degrees(_tilt_pitch), (double)math::degrees(_tilt_roll));
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
