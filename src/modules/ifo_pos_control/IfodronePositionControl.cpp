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

#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

IfodronePositionControl::IfodronePositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

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

	configureGotoControl();

	PX4_INFO("IFODRONE position controller initialized - minimal mode");
	return true;
}

void IfodronePositionControl::configureGotoControl()
{
	_goto_control.setParamMpcAccHor(GOTO_ACC_HOR);
	_goto_control.setParamMpcAccDownMax(GOTO_ACC_DOWN_MAX);
	_goto_control.setParamMpcAccUpMax(GOTO_ACC_UP_MAX);
	_goto_control.setParamMpcJerkAuto(GOTO_JERK_AUTO);
	_goto_control.setParamMpcXyCruise(GOTO_XY_CRUISE);
	_goto_control.setParamMpcXyErrMax(GOTO_XY_ERR_MAX);
	_goto_control.setParamMpcXyVelMax(GOTO_XY_VEL_MAX);
	_goto_control.setParamMpcYawrautoMax(GOTO_YAW_RATE_MAX);
	_goto_control.setParamMpcYawrautoAcc(GOTO_YAW_ACCEL_MAX);
	_goto_control.setParamMpcZVAutoDn(GOTO_Z_VEL_DOWN);
	_goto_control.setParamMpcZVAutoUp(GOTO_Z_VEL_UP);
}

void IfodronePositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();

	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);

	vehicle_local_position_s local_pos{};
	if (!_local_pos_sub.copy(&local_pos)) {
		perf_end(_cycle_perf);
		return;
	}

	const float dt = (_last_run > 0)
			 ? math::constrain((local_pos.timestamp_sample - _last_run) * 1e-6f, 0.002f, 0.04f)
			 : 0.01f;
	_last_run = local_pos.timestamp_sample;

	// If a goto setpoint is available this publishes a trajectory setpoint to go there.
	// If trajectory_setpoint is published elsewhere, do not use the goto setpoint.
	const bool goto_setpoint_enable = control_mode.flag_multicopter_position_control_enabled
					  && !_trajectory_setpoint_sub.updated();

	if (_goto_control.checkForSetpoint(local_pos.timestamp_sample, goto_setpoint_enable)) {
		const matrix::Vector3f current_position(local_pos.x, local_pos.y, local_pos.z);
		const float current_yaw = PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f;
		_goto_control.update(dt, current_position, current_yaw);
	}

	trajectory_setpoint_s traj_sp{};
	const bool has_trajectory_setpoint = _trajectory_setpoint_sub.copy(&traj_sp);
	const bool trajectory_setpoint_fresh = has_trajectory_setpoint
					       && (traj_sp.timestamp != 0)
					       && (now <= traj_sp.timestamp + TRAJ_SP_TIMEOUT_US);

	const bool armed = control_mode.flag_armed;
	const bool altitude_control_enabled = control_mode.flag_control_altitude_enabled;
	const bool position_control_enabled = control_mode.flag_control_position_enabled
					      || control_mode.flag_multicopter_position_control_enabled;

	float z_sp = _hold_z;

	if (!armed || !altitude_control_enabled) {
		_hold_position_initialized = false;
	}

	if (!armed || !position_control_enabled) {
		_hold_xy_initialized = false;
	}

	if (armed && altitude_control_enabled && local_pos.z_valid) {
		if (trajectory_setpoint_fresh && PX4_ISFINITE(traj_sp.position[2])) {
			// Latch setpoint only when it meaningfully differs from current position
			const float candidate_z = traj_sp.position[2];
			const float diff = fabsf(candidate_z - local_pos.z);

			if (!_hold_position_initialized || diff > 0.2f) {
				_hold_z = candidate_z;
				_hold_position_initialized = true;
				PX4_INFO("IFO_DBG: z_sp latched to %.3f", (double)_hold_z);
			}

			z_sp = _hold_z;

		} else {
			if (!_hold_position_initialized) {
				_hold_z = local_pos.z;
				_hold_position_initialized = true;
				PX4_INFO("IFO_DBG: z_sp latched to %.3f", (double)_hold_z);
			}
			z_sp = _hold_z;
		}
	}

	_z_sp = z_sp;

	// Cascaded P controller: position -> velocity -> acceleration -> thrust
	const bool run_xy_control = armed && position_control_enabled && local_pos.xy_valid && local_pos.v_xy_valid;
	const bool run_z_control = armed && altitude_control_enabled && local_pos.z_valid && local_pos.v_z_valid;

	const matrix::Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const matrix::Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);

	matrix::Vector3f pos_sp = pos;
	pos_sp(2) = z_sp; // Z is handled with special latching logic above

	const bool traj_x_finite = trajectory_setpoint_fresh && PX4_ISFINITE(traj_sp.position[0]);
	const bool traj_y_finite = trajectory_setpoint_fresh && PX4_ISFINITE(traj_sp.position[1]);

	if (armed && position_control_enabled && local_pos.xy_valid) {
		// If any XY trajectory component is missing (or setpoint is stale), hold the current XY point.
		// This prevents pos_sp from tracking the current position sample and drifting away in HOLD.
		if (!traj_x_finite || !traj_y_finite) {
			if (!_hold_xy_initialized) {
				_hold_x = local_pos.x;
				_hold_y = local_pos.y;
				_hold_xy_initialized = true;
			}
		}

		if (traj_x_finite) {
			pos_sp(0) = traj_sp.position[0];

		} else {
			pos_sp(0) = _hold_x;
		}

		if (traj_y_finite) {
			pos_sp(1) = traj_sp.position[1];
		} else {
			pos_sp(1) = _hold_y;
		}

		if (traj_x_finite && traj_y_finite) {
			_hold_xy_initialized = false;
		}

	} else if (!trajectory_setpoint_fresh) {
		// Keep previous behavior on stale setpoint when XY control is not active.
		if (!_hold_xy_initialized) {
			_hold_x = local_pos.x;
			_hold_y = local_pos.y;
			_hold_xy_initialized = true;
		}

		pos_sp(0) = _hold_x;
		pos_sp(1) = _hold_y;
	}

	pos_sp(0) = 0.0f;
	pos_sp(1) = 0.0f;

	matrix::Vector3f pos_err = pos_sp - pos;

	if (!run_xy_control) {
		pos_err(0) = 0.0f;
		pos_err(1) = 0.0f;
	}

	if (!run_z_control) {
		pos_err(2) = 0.0f;
	}

	matrix::Vector3f vel_sp;
	vel_sp(0) = pos_err(0) * _param_ifo_pos_xy_p.get();
	vel_sp(1) = pos_err(1) * _param_ifo_pos_xy_p.get();
	vel_sp(2) = pos_err(2) * _param_ifo_pos_z_p.get();

	matrix::Vector3f vel_err = vel_sp - vel;

	if (!run_xy_control) {
		vel_err(0) = 0.0f;
		vel_err(1) = 0.0f;
	}

	if (!run_z_control) {
		vel_err(2) = 0.0f;
	}

	matrix::Vector3f accel_sp;
	accel_sp(0) = vel_err(0) * _param_ifo_vel_xy_p.get();
	accel_sp(1) = vel_err(1) * _param_ifo_vel_xy_p.get();
	accel_sp(2) = vel_err(2) * _param_ifo_vel_z_p.get();

	matrix::Vector3f thrust_sp_body;
	thrust_sp_body.setZero();

	// Convert accel setpoint from NED frame to body frame
	// XY: Rotate by yaw so side motor tilts push in the correct world direction
	// Z: Compensate for tilt angle (tilted body needs more thrust to maintain vertical force)
	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);
	const Quatf q_current(att.q);
	const Eulerf euler_current(q_current);

	const float roll_current = euler_current.phi();
	const float pitch_current = euler_current.theta();
	const float yaw_current = euler_current.psi();

	// Rotate NED XY acceleration into body frame (yaw rotation)
	const float cos_yaw = cosf(yaw_current);
	const float sin_yaw = sinf(yaw_current);
	const float accel_body_x =  cos_yaw * accel_sp(0) + sin_yaw * accel_sp(1);
	const float accel_body_y = -sin_yaw * accel_sp(0) + cos_yaw * accel_sp(1);

	// Tilt compensation: vertical component of body-Z thrust = T * cos(roll) * cos(pitch)
	// To maintain desired vertical force: T_corrected = T_desired / cos_tilt
	const float cos_tilt = fmaxf(cosf(roll_current) * cosf(pitch_current), 0.5f);

	if (run_xy_control || run_z_control) {
		thrust_sp_body(0) = accel_body_x / CONSTANTS_ONE_G;
		thrust_sp_body(1) = accel_body_y / CONSTANTS_ONE_G;

		const float thrust_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.0f, 1.0f);
		matrix::Vector2f thrust_xy(thrust_sp_body(0), thrust_sp_body(1));
		const float thrust_xy_norm = thrust_xy.norm();

		if (thrust_xy_norm > thrust_xy_max && thrust_xy_norm > 1e-5f) {
			thrust_xy *= thrust_xy_max / thrust_xy_norm;
		}

		thrust_sp_body(0) = run_xy_control ? thrust_xy(0) : 0.0f;
		thrust_sp_body(1) = run_xy_control ? thrust_xy(1) : 0.0f;

		const float hover_thrust = math::constrain(_param_ifo_thr_hover.get(), 0.0f, 1.0f);
		const float thrust_min = math::constrain(_param_ifo_thr_min.get(), 0.0f, 1.0f);
		const float thrust_max = math::constrain(_param_ifo_thr_max.get(), thrust_min, 1.0f);

		// NED: positive acceleration setpoint in +Z (down) requires less upward thrust.
		// Compensate for tilt: when body is tilted, need more thrust to maintain vertical force.
		float thrust_z = (hover_thrust - accel_sp(2) * (hover_thrust / CONSTANTS_ONE_G)) / cos_tilt;
		thrust_z = math::constrain(thrust_z, thrust_min, thrust_max);
		thrust_sp_body(2) = run_z_control ? -thrust_z : 0.0f;
	}

	float yaw_sp = 0.0f;
	if (trajectory_setpoint_fresh && PX4_ISFINITE(traj_sp.yaw)) {
		yaw_sp = traj_sp.yaw;

	} else if (PX4_ISFINITE(local_pos.heading)) {
		yaw_sp = local_pos.heading;
	}

	const float yawspeed_sp = (trajectory_setpoint_fresh && PX4_ISFINITE(traj_sp.yawspeed)) ? traj_sp.yawspeed : 0.0f;

	// // Debug output (1 Hz)
	// static hrt_abstime last_dbg_ts = 0;
	// if (now - last_dbg_ts > DEBUG_INTERVAL_US) {
	// 	last_dbg_ts = now;
	// 	PX4_INFO("IFO_DBG: pos=(%.2f %.2f %.2f) sp=(%.2f %.2f %.2f) thr=(%.3f %.3f %.3f) yaw=%.1f cos_tilt=%.3f",
	// 		 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
	// 		 (double)pos_sp(0), (double)pos_sp(1), (double)pos_sp(2),
	// 		 (double)thrust_sp_body(0), (double)thrust_sp_body(1), (double)thrust_sp_body(2),
	// 		 (double)math::degrees(yaw_current), (double)cos_tilt);
	// }

	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.x = run_xy_control ? pos_sp(0) : NAN;
	local_pos_sp.y = run_xy_control ? pos_sp(1) : NAN;
	local_pos_sp.z = run_z_control ? pos_sp(2) : NAN;
	local_pos_sp.vx = run_xy_control ? vel_sp(0) : NAN;
	local_pos_sp.vy = run_xy_control ? vel_sp(1) : NAN;
	local_pos_sp.vz = run_z_control ? vel_sp(2) : NAN;
	local_pos_sp.acceleration[0] = run_xy_control ? accel_sp(0) : NAN;
	local_pos_sp.acceleration[1] = run_xy_control ? accel_sp(1) : NAN;
	local_pos_sp.acceleration[2] = run_z_control ? accel_sp(2) : NAN;
	local_pos_sp.thrust[0] = run_xy_control ? thrust_sp_body(0) : NAN;
	local_pos_sp.thrust[1] = run_xy_control ? thrust_sp_body(1) : NAN;
	local_pos_sp.thrust[2] = run_z_control ? thrust_sp_body(2) : NAN;
	local_pos_sp.yaw = yaw_sp;
	local_pos_sp.yawspeed = yawspeed_sp;
	_local_pos_sp_pub.publish(local_pos_sp);

	// Thrust flows to control allocator via attitude controller (att_sp.thrust_body),
	// NOT directly. Attitude controller publishes vehicle_thrust_setpoint.

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.yaw_sp_move_rate = yawspeed_sp;
	const Quatf q_sp(Eulerf(0.0f, 0.0f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);
	att_sp.thrust_body[0] = thrust_sp_body(0);
	att_sp.thrust_body[1] = thrust_sp_body(1);
	att_sp.thrust_body[2] = thrust_sp_body(2);
	_attitude_setpoint_pub.publish(att_sp);

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
	PX4_INFO("IFODRONE Position Controller - minimal mode");
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
