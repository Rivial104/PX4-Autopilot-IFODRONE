/**
 * IFODRONE Position Controller
 *
 * Full 3D position controller:
 * - Z-axis: Main motors (0-1) for altitude
 * - XY-axis: Tilt motors (2-5) for horizontal position
 *
 */

#include "IfodronePositionControl.hpp"

#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

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

	PX4_INFO("IFODRONE position controller initialized - minimal mode");
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

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();
	_last_run = now;

	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);

	vehicle_local_position_s local_pos{};
	if (!_local_pos_sub.copy(&local_pos)) {
		perf_end(_cycle_perf);
		return;
	}

	// Adjust hold setpoints for EKF resets (keeps setpoints fixed in world frame)
	if (local_pos.xy_reset_counter != _xy_reset_counter) {
		if (_hold_xy_initialized) {
			_hold_x += local_pos.delta_xy[0];
			_hold_y += local_pos.delta_xy[1];
			PX4_INFO("IFO_DBG: XY reset - hold adjusted by (%.3f, %.3f)",
				 (double)local_pos.delta_xy[0], (double)local_pos.delta_xy[1]);
		}
	}

	if (local_pos.z_reset_counter != _z_reset_counter) {
		if (_hold_position_initialized) {
			_hold_z += local_pos.delta_z;
			PX4_INFO("IFO_DBG: Z reset - hold_z adjusted by %.3f", (double)local_pos.delta_z);
		}
	}

	trajectory_setpoint_s traj_sp{};
	const bool has_trajectory_setpoint = _trajectory_setpoint_sub.copy(&traj_sp);

	// Check if trajectory setpoint is fresh (like MulticopterPositionControl)
	const bool traj_sp_fresh = has_trajectory_setpoint && (traj_sp.timestamp > 0)
				   && (now - traj_sp.timestamp < 500_ms);

	// Adjust setpoint for EKF resets (similar to MulticopterPositionControl::adjustSetpointForEKFResets)
	if (has_trajectory_setpoint && traj_sp.timestamp != 0 && traj_sp.timestamp < local_pos.timestamp) {
		if (local_pos.xy_reset_counter != _xy_reset_counter) {
			if (PX4_ISFINITE(traj_sp.position[0])) { traj_sp.position[0] += local_pos.delta_xy[0]; }
			if (PX4_ISFINITE(traj_sp.position[1])) { traj_sp.position[1] += local_pos.delta_xy[1]; }
		}
		if (local_pos.z_reset_counter != _z_reset_counter) {
			if (PX4_ISFINITE(traj_sp.position[2])) { traj_sp.position[2] += local_pos.delta_z; }
		}
		if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
			if (PX4_ISFINITE(traj_sp.velocity[0])) { traj_sp.velocity[0] += local_pos.delta_vxy[0]; }
			if (PX4_ISFINITE(traj_sp.velocity[1])) { traj_sp.velocity[1] += local_pos.delta_vxy[1]; }
		}
		if (local_pos.vz_reset_counter != _vz_reset_counter) {
			if (PX4_ISFINITE(traj_sp.velocity[2])) { traj_sp.velocity[2] += local_pos.delta_vz; }
		}
	}

	// Save reset counters AFTER all adjustments are done
	_xy_reset_counter = local_pos.xy_reset_counter;
	_z_reset_counter = local_pos.z_reset_counter;
	_vxy_reset_counter = local_pos.vxy_reset_counter;
	_vz_reset_counter = local_pos.vz_reset_counter;

	const bool armed = control_mode.flag_armed;
	const bool altitude_control_enabled = control_mode.flag_control_altitude_enabled;
	const bool position_control_enabled = control_mode.flag_control_position_enabled;

	float z_sp = _hold_z;

	if (!armed || !altitude_control_enabled) {
		_hold_position_initialized = false;
	}

	if (!armed || !position_control_enabled) {
		_hold_xy_initialized = false;
	}

	if (armed && altitude_control_enabled && local_pos.z_valid) {
		if (traj_sp_fresh && PX4_ISFINITE(traj_sp.position[2])) {
			// Check if trajectory setpoint is changing rapidly (indicates takeoff/landing/mission, not Hold)
			bool traj_changing_rapidly = false;
			if (PX4_ISFINITE(_last_traj_sp_pos(2)) && _last_traj_sp_update > 0) {
				const float dt_s = (now - _last_traj_sp_update) * 1e-6f;
				if (dt_s > 0.01f && dt_s < 1.0f) {
					const float dz = fabsf(traj_sp.position[2] - _last_traj_sp_pos(2));
					const float z_rate = dz / dt_s;
					// If Z setpoint changing faster than 0.2 m/s, it's takeoff/landing/mission
					if (z_rate > 0.2f) {
						traj_changing_rapidly = true;
					}
				}
			}

			// Update last trajectory setpoint
			_last_traj_sp_pos(2) = traj_sp.position[2];
			_last_traj_sp_update = now;

			// Only latch hold position if trajectory is stable (not changing rapidly)
			if (!traj_changing_rapidly) {
				const float candidate_z = traj_sp.position[2];
				const float diff_from_hold = fabsf(candidate_z - _hold_z);

				if (!_hold_position_initialized || diff_from_hold > 0.1f) {
					_hold_z = candidate_z;
					_hold_position_initialized = true;
					PX4_INFO("IFO_DBG: Z hold latched to %.3f from traj_sp (diff=%.3f)",
						 (double)_hold_z, (double)diff_from_hold);
				}
			} else {
				// During rapid changes, use current trajectory setpoint directly
				_hold_z = traj_sp.position[2];
				_hold_position_initialized = true;
			}

			z_sp = _hold_z;

		} else {
			if (!_hold_position_initialized) {
				_hold_z = local_pos.z;
				_hold_position_initialized = true;
				PX4_INFO("IFO_DBG: Z hold latched to current pos %.3f (no fresh traj_sp)",
					 (double)_hold_z);
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

	// XY: latch last valid setpoint or hold current position to avoid drifting
	// Key insight from MulticopterPositionControl: only update hold position from fresh trajectory setpoints
	if (run_xy_control) {
		const bool traj_xy_valid = traj_sp_fresh
					   && PX4_ISFINITE(traj_sp.position[0])
					   && PX4_ISFINITE(traj_sp.position[1]);

		if (traj_xy_valid) {
			// Check if trajectory setpoint is changing rapidly (indicates mission, not Hold)
			bool traj_xy_changing_rapidly = false;
			if (PX4_ISFINITE(_last_traj_sp_pos(0)) && PX4_ISFINITE(_last_traj_sp_pos(1)) && _last_traj_sp_update > 0) {
				const float dt_s = (now - _last_traj_sp_update) * 1e-6f;
				if (dt_s > 0.01f && dt_s < 1.0f) {
					const float dx = traj_sp.position[0] - _last_traj_sp_pos(0);
					const float dy = traj_sp.position[1] - _last_traj_sp_pos(1);
					const float dist = sqrtf(dx * dx + dy * dy);
					const float xy_rate = dist / dt_s;
					// If XY setpoint changing faster than 0.3 m/s, it's a mission
					if (xy_rate > 0.3f) {
						traj_xy_changing_rapidly = true;
					}
				}
			}

			// Update last trajectory setpoint
			_last_traj_sp_pos(0) = traj_sp.position[0];
			_last_traj_sp_pos(1) = traj_sp.position[1];

			// Only latch hold position if trajectory is stable
			if (!traj_xy_changing_rapidly) {
				const float dx = traj_sp.position[0] - _hold_x;
				const float dy = traj_sp.position[1] - _hold_y;
				const float dist_sq = dx * dx + dy * dy;

				// Only update hold position if trajectory setpoint changed significantly or not initialized
				if (!_hold_xy_initialized || dist_sq > 0.01f) { // 10cm threshold
					_hold_x = traj_sp.position[0];
					_hold_y = traj_sp.position[1];
					_hold_xy_initialized = true;
					PX4_INFO("IFO_DBG: XY hold latched to (%.3f, %.3f) from traj_sp",
						 (double)_hold_x, (double)_hold_y);
				}
			} else {
				// During rapid changes, use current trajectory setpoint directly
				_hold_x = traj_sp.position[0];
				_hold_y = traj_sp.position[1];
				_hold_xy_initialized = true;
			}

		} else if (!_hold_xy_initialized) {
			// No valid trajectory setpoint - hold current position
			_hold_x = local_pos.x;
			_hold_y = local_pos.y;
			_hold_xy_initialized = true;
			PX4_INFO("IFO_DBG: XY hold latched to current pos (%.3f, %.3f)",
				 (double)_hold_x, (double)_hold_y);
		}

		if (_hold_xy_initialized) {
			pos_sp(0) = _hold_x;
			pos_sp(1) = _hold_y;
		}
	}

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

	if (run_xy_control || run_z_control) {
		thrust_sp_body(0) = accel_sp(0) / CONSTANTS_ONE_G;
		thrust_sp_body(1) = accel_sp(1) / CONSTANTS_ONE_G;

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
		float thrust_z = hover_thrust - accel_sp(2) * (hover_thrust / CONSTANTS_ONE_G);
		thrust_z = math::constrain(thrust_z, thrust_min, thrust_max);
		thrust_sp_body(2) = run_z_control ? -thrust_z : 0.0f;
	}

	float yaw_sp = 0.0f;
	if (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.yaw)) {
		yaw_sp = traj_sp.yaw;

	} else if (PX4_ISFINITE(local_pos.heading)) {
		yaw_sp = local_pos.heading;
	}

	const float yawspeed_sp = (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.yawspeed)) ? traj_sp.yawspeed : 0.0f;

	// Debug output (1 Hz)
	static hrt_abstime last_dbg_ts = 0;
	if (now - last_dbg_ts > DEBUG_INTERVAL_US) {
		last_dbg_ts = now;
		PX4_INFO("IFO_DBG: === Position Control State ===");
		PX4_INFO("IFO_DBG: ctrl_mode: armed=%d alt_en=%d pos_en=%d",
			 (int)armed, (int)altitude_control_enabled, (int)position_control_enabled);
		PX4_INFO("IFO_DBG: local_pos: xy_valid=%d z_valid=%d v_xy_valid=%d v_z_valid=%d",
			 (int)local_pos.xy_valid, (int)local_pos.z_valid,
			 (int)local_pos.v_xy_valid, (int)local_pos.v_z_valid);
		PX4_INFO("IFO_DBG: pos_cur=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)",
			 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
			 (double)local_pos.vx, (double)local_pos.vy, (double)local_pos.vz);
		PX4_INFO("IFO_DBG: hold_xy=(%.3f, %.3f) hold_z=%.3f init_xy=%d init_z=%d",
			 (double)_hold_x, (double)_hold_y, (double)_hold_z,
			 (int)_hold_xy_initialized, (int)_hold_position_initialized);
		PX4_INFO("IFO_DBG: pos_sp=(%.3f, %.3f, %.3f) pos_err=(%.3f, %.3f, %.3f)",
			 (double)pos_sp(0), (double)pos_sp(1), (double)pos_sp(2),
			 (double)pos_err(0), (double)pos_err(1), (double)pos_err(2));
		PX4_INFO("IFO_DBG: vel_sp=(%.3f, %.3f, %.3f) vel_err=(%.3f, %.3f, %.3f)",
			 (double)vel_sp(0), (double)vel_sp(1), (double)vel_sp(2),
			 (double)vel_err(0), (double)vel_err(1), (double)vel_err(2));
		PX4_INFO("IFO_DBG: thrust=(%.4f, %.4f, %.4f) run_xy=%d run_z=%d",
			 (double)thrust_sp_body(0), (double)thrust_sp_body(1), (double)thrust_sp_body(2),
			 (int)run_xy_control, (int)run_z_control);
		PX4_INFO("IFO_DBG: traj_sp_fresh=%d traj_ts=%llu now=%llu",
			 (int)traj_sp_fresh, (unsigned long long)traj_sp.timestamp, (unsigned long long)now);
	}

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

	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = now;
	thrust_sp.timestamp_sample = local_pos.timestamp_sample;
	thrust_sp.xyz[0] = thrust_sp_body(0);
	thrust_sp.xyz[1] = thrust_sp_body(1);
	thrust_sp.xyz[2] = thrust_sp_body(2);
	_thrust_setpoint_pub.publish(thrust_sp);

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.yaw_sp_move_rate = yawspeed_sp;
	const matrix::Quatf q_sp(matrix::Eulerf(0.0f, 0.0f, yaw_sp));
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
