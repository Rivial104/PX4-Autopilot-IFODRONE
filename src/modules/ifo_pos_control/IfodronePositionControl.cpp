/**
 * IFODRONE Position Controller
 *
 * Full 3D position controller:
 * - Z-axis: Main motors (0-1) for altitude
 * - XY-axis: Tilt motors (2-5) for horizontal position
 *
 * Uses TakeoffHandling for proper takeoff sequence.
 */

#include "IfodronePositionControl.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

IfodronePositionControl::IfodronePositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// // Initialize takeoff handling
	// _takeoff.setSpoolupTime(1.0f);  // 1 second spoolup
	// _takeoff.setTakeoffRampTime(2.0f);  // 2 second ramp
	// _takeoff.generateInitialRampValue(_param_ifo_vel_z_p.get());
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

	PX4_INFO("IFODRONE position controller initialized - Z-axis only mode");
	return true;
}

void IfodronePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Get current time
	const hrt_abstime now = hrt_absolute_time();
	(void)(_last_run);  // dt not used yet, but keep for future PID
	_last_run = now;

	// Get control mode
	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);

	// Get vehicle status
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	// Get land detected
	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	// Get current position
	vehicle_local_position_s local_pos{};
	if (!_local_pos_sub.copy(&local_pos)) {
		perf_end(_cycle_perf);
		return;
	}

	// Get trajectory setpoint (from flight mode manager)
	trajectory_setpoint_s traj_sp{};
	const bool has_trajectory_setpoint = _trajectory_setpoint_sub.copy(&traj_sp);

	// Get manual control input (from RC/Joystick)
	// manual_control_setpoint_s manual_control{};
	// const bool has_manual_control = _manual_control_sub.copy(&manual_control);

	// Get current attitude for yaw
	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const Quatf q_current(attitude.q);
	const float yaw_current = Eulerf(q_current).psi();

	// ================================================================
	// TAKEOFF STATE MACHINE
	// ================================================================
	// const float takeoff_desired_vz = -1.0f;  // 1 m/s upward (negative in NED)
	// const bool want_takeoff = control_mode.flag_armed &&
	// 			  (control_mode.flag_control_altitude_enabled ||
	// 			   control_mode.flag_control_position_enabled);

	// _takeoff.updateTakeoffState(
	// 	control_mode.flag_armed,
	// 	land_detected.landed,
	// 	want_takeoff,
	// 	takeoff_desired_vz,
	// 	false,  // skip_takeoff
	// 	now
	// );

	// Calculate dt for ramp
	// const float dt = math::constrain((now - _last_run) * 1e-6f, 0.002f, 0.04f);

	// // Get velocity limit from takeoff ramp
	// const float velocity_limit = _takeoff.updateRamp(dt, takeoff_desired_vz);

	// ================================================================
	// 3D POSITION CONTROLLER
	// Z-axis: Main motors (0-1)
	// XY-axis: Tilt motors (2-5) for horizontal thrust
	// ================================================================

	float thrust_x = 0.0f;  // Body frame X thrust (forward) - from tilt motors
	float thrust_y = 0.0f;  // Body frame Y thrust (right) - from tilt motors
	float thrust_z = 0.0f;  // Body frame Z thrust (down) - from main motors

	float x_sp = 0.0f, y_sp = 0.0f, z_sp = 0.0f;
	float vx_sp = 0.0f, vy_sp = 0.0f, vz_sp = 0.0f;
	float ax_sp = 0.0f, ay_sp = 0.0f, az_sp = 0.0f;

	// Reset hold position when disarmed or landed
	if (!control_mode.flag_armed || land_detected.landed) {
		_hold_position_initialized = false;
	}

	constexpr float KP_Z = 1.2f;      // position -> velocity
	constexpr float KD_Z = 0.4f;      // velocity damping
	constexpr float THR_HOVER = 0.5f; // hover thrust estimate
	constexpr float THR_MIN = 0.1f;
	constexpr float THR_MAX = 0.9f;
	// constexpr float MAX_VZ = 1.5f;    // m/s up

	if (control_mode.flag_armed &&
	control_mode.flag_control_altitude_enabled &&
	local_pos.z_valid && local_pos.v_z_valid)
	{

	if (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.position[2])) {
		z_sp = traj_sp.position[2];
	} else {
		// HOLD current altitude if nothing commanded
		z_sp = _hold_z;
	}

	// Errors (NED!)
	const float z_err  = z_sp - local_pos.z;
	const float vz_err = 0.0f - local_pos.vz;

	// Desired vertical acceleration
	float az_cmd = KP_Z * z_err + KD_Z * vz_err;

	// Convert acceleration to thrust
	thrust_z = THR_HOVER - az_cmd * 0.1f;

	thrust_z = math::constrain(thrust_z, THR_MIN, THR_MAX);

	} else {
	thrust_z = 0.0f;
	}

	// Check takeoff state
	// const TakeoffState takeoff_state = _takeoff.getTakeoffState();
	// const bool in_flight_or_takeoff = (takeoff_state == TakeoffState::rampup ||
	// 				   takeoff_state == TakeoffState::flight);

	// if (run_position_control && local_pos.z_valid && local_pos.v_z_valid && in_flight_or_takeoff) {

	// 	// ============================================
	// 	// DETERMINE SETPOINT SOURCE
	// 	// Priority: 1. Trajectory (Mission), 2. Manual RC, 3. Position Hold
	// 	// ============================================

	// 	const bool manual_mode = control_mode.flag_control_manual_enabled;
	// 	const bool trajectory_valid = has_trajectory_setpoint &&
	// 				      (PX4_ISFINITE(traj_sp.position[0]) || PX4_ISFINITE(traj_sp.velocity[0]));

	// 	// ============================================
	// 	// Z-AXIS ALTITUDE CONTROL (Main motors)
	// 	// ============================================

	// 	// Initialize hold position on first run or when entering position control
	// 	if (!_hold_position_initialized && local_pos.z_valid && local_pos.xy_valid) {
	// 		_hold_x = local_pos.x;
	// 		_hold_y = local_pos.y;
	// 		_hold_z = local_pos.z;
	// 		_hold_position_initialized = true;
	// 		PX4_INFO("Hold position initialized: (%.1f, %.1f, %.1f)",
	// 			 (double)_hold_x, (double)_hold_y, (double)_hold_z);
	// 	}

	// 	// --- Z setpoints ---
	// 	if (trajectory_valid && PX4_ISFINITE(traj_sp.position[2])) {
	// 		// Use trajectory setpoint (Mission mode)
	// 		z_sp = traj_sp.position[2];
	// 		vz_sp = PX4_ISFINITE(traj_sp.velocity[2]) ? traj_sp.velocity[2] : 0.0f;
	// 		az_sp = PX4_ISFINITE(traj_sp.acceleration[2]) ? traj_sp.acceleration[2] : 0.0f;
	// 		// Update hold position
	// 		_hold_z = z_sp;

	// 	} else if (manual_mode && has_manual_control) {
	// 		// Manual control: stick = velocity command
	// 		// Throttle stick: center = hover, up = climb, down = descend
	// 		// manual_control.throttle: -1.0 (down) to +1.0 (up)
	// 		const float max_vel_z = 2.0f;  // m/s max vertical velocity

	// 		// Convert throttle to velocity (inverted: stick up = negative vz = climb in NED)
	// 		vz_sp = -manual_control.throttle * max_vel_z;

	// 		// Small deadzone around center
	// 		if (fabsf(vz_sp) < 0.1f) {
	// 			// Hold altitude when stick is centered
	// 			z_sp = _hold_z;
	// 			vz_sp = 0.0f;
	// 		} else {
	// 			// Update hold position while moving
	// 			_hold_z = local_pos.z;
	// 			z_sp = NAN;  // Velocity control only
	// 		}

	// 	} else {
	// 		// No input: hold current position
	// 		z_sp = _hold_z;
	// 		vz_sp = 0.0f;
	// 	}

	// 	const float z = local_pos.z;
	// 	const float vz = local_pos.vz;

	// 	// Position + Velocity control
	// 	float vz_cmd;
	// 	if (PX4_ISFINITE(z_sp)) {
	// 		const float z_error = z_sp - z;
	// 		vz_cmd = z_error * _param_ifo_pos_z_p.get() + vz_sp;
	// 	} else {
	// 		// Velocity only control
	// 		vz_cmd = vz_sp;
	// 	}

	// 	// Apply takeoff velocity limit
	// 	if (vz_cmd < velocity_limit) {
	// 		vz_cmd = velocity_limit;
	// 	}

	// 	const float vz_error = vz_cmd - vz;
	// 	az_sp = az_sp + vz_error * _param_ifo_vel_z_p.get();

	// 	// Thrust Z (main motors)
	// 	thrust_z = _param_ifo_thr_hover.get() - az_sp * 0.1f;
	// 	thrust_z = math::constrain(thrust_z, _param_ifo_thr_min.get(), _param_ifo_thr_max.get());

	// 	// ============================================
	// 	// XY-AXIS POSITION CONTROL (Tilt motors)
	// 	// ============================================

	// 	// --- XY setpoints ---
	// 	if (trajectory_valid && control_mode.flag_control_position_enabled) {
	// 		// Use trajectory setpoint (Mission mode)
	// 		if (PX4_ISFINITE(traj_sp.position[0])) {
	// 			x_sp = traj_sp.position[0];
	// 			_hold_x = x_sp;
	// 		}
	// 		if (PX4_ISFINITE(traj_sp.position[1])) {
	// 			y_sp = traj_sp.position[1];
	// 			_hold_y = y_sp;
	// 		}
	// 		if (PX4_ISFINITE(traj_sp.velocity[0])) {
	// 			vx_sp = traj_sp.velocity[0];
	// 		}
	// 		if (PX4_ISFINITE(traj_sp.velocity[1])) {
	// 			vy_sp = traj_sp.velocity[1];
	// 		}

	// 	} else if (manual_mode && has_manual_control && control_mode.flag_control_position_enabled) {
	// 		// Manual control: stick = velocity command
	// 		// manual_control.pitch: -1 (back) to +1 (forward) -> velocity X
	// 		// manual_control.roll: -1 (left) to +1 (right) -> velocity Y
	// 		const float max_vel_xy = 3.0f;  // m/s max horizontal velocity

	// 		// Deadzone
	// 		const float pitch_input = fabsf(manual_control.pitch) > 0.05f ? manual_control.pitch : 0.0f;
	// 		const float roll_input = fabsf(manual_control.roll) > 0.05f ? manual_control.roll : 0.0f;

	// 		// Velocity in body frame from sticks
	// 		const float vx_body = pitch_input * max_vel_xy;
	// 		const float vy_body = roll_input * max_vel_xy;

	// 		// Convert body velocity to world frame (NED)
	// 		const float cos_yaw = cosf(yaw_current);
	// 		const float sin_yaw = sinf(yaw_current);
	// 		vx_sp = cos_yaw * vx_body - sin_yaw * vy_body;
	// 		vy_sp = sin_yaw * vx_body + cos_yaw * vy_body;

	// 		// If sticks centered, use position hold
	// 		if (fabsf(vx_sp) < 0.1f && fabsf(vy_sp) < 0.1f) {
	// 			x_sp = _hold_x;
	// 			y_sp = _hold_y;
	// 			vx_sp = 0.0f;
	// 			vy_sp = 0.0f;
	// 		} else {
	// 			// Update hold position while moving
	// 			_hold_x = local_pos.x;
	// 			_hold_y = local_pos.y;
	// 			x_sp = NAN;  // Velocity control only
	// 			y_sp = NAN;
	// 		}

	// 	} else {
	// 		// No input: hold current position
	// 		x_sp = _hold_x;
	// 		y_sp = _hold_y;
	// 		vx_sp = 0.0f;
	// 		vy_sp = 0.0f;
	// 	}

	// 	if (local_pos.xy_valid && local_pos.v_xy_valid) {
	// 		const float x = local_pos.x;
	// 		const float y = local_pos.y;
	// 		const float vx = local_pos.vx;
	// 		const float vy = local_pos.vy;

	// 		// Velocity command: from position error (if position valid) + feedforward
	// 		float vx_cmd, vy_cmd;

	// 		if (PX4_ISFINITE(x_sp) && PX4_ISFINITE(y_sp)) {
	// 			// Position + Velocity control
	// 			const float x_error = x_sp - x;
	// 			const float y_error = y_sp - y;
	// 			vx_cmd = x_error * _param_ifo_pos_xy_p.get() + vx_sp;
	// 			vy_cmd = y_error * _param_ifo_pos_xy_p.get() + vy_sp;
	// 		} else {
	// 			// Velocity only control (sticks active)
	// 			vx_cmd = vx_sp;
	// 			vy_cmd = vy_sp;
	// 		}

	// 		// Velocity error
	// 		const float vx_error = vx_cmd - vx;
	// 		const float vy_error = vy_cmd - vy;

	// 		// Acceleration setpoint (NED world frame)
	// 		ax_sp = vx_error * _param_ifo_vel_xy_p.get();
	// 		ay_sp = vy_error * _param_ifo_vel_xy_p.get();

	// 		// Convert world frame acceleration to body frame thrust
	// 		// Rotation from world (NED) to body frame using current yaw
	// 		const float cos_yaw = cosf(yaw_current);
	// 		const float sin_yaw = sinf(yaw_current);

	// 		// Body frame thrust = R_yaw^T * world_accel
	// 		// thrust_body_x = cos(yaw) * ax + sin(yaw) * ay
	// 		// thrust_body_y = -sin(yaw) * ax + cos(yaw) * ay
	// 		float thrust_x_raw = cos_yaw * ax_sp + sin_yaw * ay_sp;
	// 		float thrust_y_raw = -sin_yaw * ax_sp + cos_yaw * ay_sp;

	// 		// Scale and limit
	// 		const float thr_xy_max = _param_ifo_thr_xy_max.get();
	// 		thrust_x = math::constrain(thrust_x_raw * 0.1f, -thr_xy_max, thr_xy_max);
	// 		thrust_y = math::constrain(thrust_y_raw * 0.1f, -thr_xy_max, thr_xy_max);
	// 	}

	// 	// Debug output
	// 	static int counter = 0;
	// 	if (++counter >= 50) {  // ~1 Hz
	// 		counter = 0;
	// 		PX4_INFO("POS: xyz=(%.1f,%.1f,%.1f) sp=(%.1f,%.1f,%.1f) thr=(%.2f,%.2f,%.2f)",
	// 			 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
	// 			 (double)x_sp, (double)y_sp, (double)z_sp,
	// 			 (double)thrust_x, (double)thrust_y, (double)thrust_z);
	// 	}

	// } else if (control_mode.flag_armed && takeoff_state == TakeoffState::spoolup) {
	// 	thrust_z = 0.1f;
	// 	static int counter_spoolup = 0;
	// 	if (++counter_spoolup >= 50) {
	// 		counter_spoolup = 0;
	// 		PX4_INFO("SPOOLUP: thr=%.2f state=%d", (double)thrust_z, (int)takeoff_state);
	// 	}

	// } else if (control_mode.flag_armed && takeoff_state == TakeoffState::ready_for_takeoff) {
	// 	thrust_z = 0.15f;
	// 	static int counter_ready = 0;
	// 	if (++counter_ready >= 50) {
	// 		counter_ready = 0;
	// 		PX4_INFO("READY: thr=%.2f state=%d", (double)thrust_z, (int)takeoff_state);
	// 	}

	// } else if (control_mode.flag_armed) {
	// 	thrust_z = 0.0f;
	// 	static int counter2 = 0;
	// 	if (++counter2 >= 100) {
	// 		counter2 = 0;
	// 		PX4_WARN("Armed unknown: state=%d", (int)takeoff_state);
	// 	}
	// }

	// ================================================================
	// PUBLISH ATTITUDE SETPOINT
	// Contains thrust commands for attitude controller:
	// - thrust_body[0]: X thrust from tilt motors (forward)
	// - thrust_body[1]: Y thrust from tilt motors (right)
	// - thrust_body[2]: Z thrust from main motors (up = negative)
	// ================================================================
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.yaw_sp_move_rate = 0.0f;

	// Get yaw setpoint from trajectory if available, otherwise keep current
	float yaw_sp = yaw_current;
	if (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.yaw)) {
		yaw_sp = traj_sp.yaw;
	}

	// Quaternion for level attitude (roll=0, pitch=0) with desired yaw
	const Quatf q_sp = Quatf(Eulerf(0.0f, 0.0f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);

	// Thrust in body frame (NED convention)
	att_sp.thrust_body[0] = thrust_x;   // Forward thrust (tilt motors)
	att_sp.thrust_body[1] = thrust_y;   // Right thrust (tilt motors)
	att_sp.thrust_body[2] = -thrust_z;  // Upward thrust (main motors, negative = UP)
	_attitude_setpoint_pub.publish(att_sp);

	// ================================================================
	// PUBLISH LOCAL POSITION SETPOINT (for other modules)
	// ================================================================
	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.x = x_sp;
	local_pos_sp.y = y_sp;
	local_pos_sp.z = z_sp;
	local_pos_sp.vx = vx_sp;
	local_pos_sp.vy = vy_sp;
	local_pos_sp.vz = vz_sp;
	local_pos_sp.acceleration[0] = ax_sp;
	local_pos_sp.acceleration[1] = ay_sp;
	local_pos_sp.acceleration[2] = az_sp;
	local_pos_sp.thrust[0] = thrust_x;
	local_pos_sp.thrust[1] = thrust_y;
	local_pos_sp.thrust[2] = -thrust_z;
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
	PX4_INFO("IFODRONE Position Controller - Z-axis only");
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
