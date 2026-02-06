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

	trajectory_setpoint_s traj_sp{};
	const bool has_trajectory_setpoint = _trajectory_setpoint_sub.copy(&traj_sp);

	float z_sp = _hold_z;

	if (!control_mode.flag_armed || !control_mode.flag_control_altitude_enabled) {
		_hold_position_initialized = false;
	}

	if (control_mode.flag_armed && control_mode.flag_control_altitude_enabled && local_pos.z_valid) {
		if (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.position[2])) {
			// Latch setpoint only when it meaningfully differs from current position
			const float candidate_z = traj_sp.position[2];
			const float diff = fabsf(candidate_z - local_pos.z);

			if (!_hold_position_initialized || diff > 0.5f) {
				_hold_z = candidate_z;
				_hold_position_initialized = true;
				PX4_INFO("IFO_DBG: z_sp latched to %.3f\n", (double)_hold_z);
			}

			z_sp = _hold_z;

		} else {
			if (!_hold_position_initialized) {
				_hold_z = local_pos.z;
				_hold_position_initialized = true;
				PX4_INFO("IFO_DBG: z_sp latched to %.3f\n", (double)_hold_z);
			}
			z_sp = _hold_z;
		}
	}

	_z_sp = z_sp;

	// Simple PD: position -> velocity -> acceleration -> thrust
	float thrust_cmd = 0.0f;
	float z_err = 0.0f;
	float vz_sp = 0.0f;
	float vz_err = 0.0f;

	if (control_mode.flag_armed && control_mode.flag_control_altitude_enabled
	    && local_pos.z_valid && local_pos.v_z_valid) {
		z_err = _z_sp - local_pos.z;
		vz_sp = z_err * _param_ifo_pos_z_p.get();
		vz_err = vz_sp - local_pos.vz;
		const float az_sp = vz_err * _param_ifo_vel_z_p.get();

		const float hover = math::constrain(_param_ifo_thr_hover.get(), 0.0f, 1.0f);
		// NED: positive az_sp means down, so reduce thrust
		thrust_cmd = hover - az_sp * (hover / CONSTANTS_ONE_G);
		thrust_cmd = math::constrain(thrust_cmd, 0.0f, 1.0f);
	}

	// Debug output (1 Hz)
	static hrt_abstime last_dbg_ts = 0;
	if (now - last_dbg_ts > DEBUG_INTERVAL_US) {
		last_dbg_ts = now;

		PX4_INFO("IFO_DBG: z_sp set to %.3f", (double)z_sp);

		PX4_INFO("IFO_DBG: z=%.3f z_sp=%.3f z_err=%.3f vz=%.3f vz_sp=%.3f vz_err=%.3f thr=%.3f\n",
			(double)local_pos.z, (double)_z_sp, (double)z_err,
			(double)local_pos.vz, (double)vz_sp, (double)vz_err,
			(double)thrust_cmd);
	}

	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = now;
	local_pos_sp.z = z_sp;
	_local_pos_sp_pub.publish(local_pos_sp);

	// vehicle_thrust_setpoint_s thrust_sp{};
	// thrust_sp.timestamp = now;
	// if (control_mode.flag_armed && control_mode.flag_control_altitude_enabled) {
	// 	thrust_sp.xyz[0] = 0.0f;
	// 	thrust_sp.xyz[1] = 0.0f;
	// 	thrust_sp.xyz[2] = -0.6f; // NED: negative Z is upward thrust
	// }
	// _thrust_setpoint_pub.publish(thrust_sp);

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.thrust_body[0] = 0.0f;
	att_sp.thrust_body[1] = 0.0f;
	att_sp.thrust_body[2] = -thrust_cmd; // NED: negative Z is upward thrust
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
