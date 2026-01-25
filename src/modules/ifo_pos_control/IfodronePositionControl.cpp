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
	float dt{0.01f};
	if (_last_run != 0) {
		dt = (now - _last_run) * 1e-6f; // convert to seconds
		if(dt <= 0.0f || dt > 1.0f) {
			dt = 0.01f;
		}
	}
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

	// Get current attitude for yaw
	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const Quatf q_current(attitude.q);
	const float yaw_current = Eulerf(q_current).psi();

	// ================================================================
	// 3D POSITION CONTROLLER
	// Z-axis: Main motors (0-1)
	// XY-axis: Tilt motors (2-5) for horizontal thrust
	// ================================================================

	float thrust_x = 0.0f;
	float thrust_y = 0.0f;
	float thrust_z = 0.0f;
	float x_sp = 0.0f, y_sp = 0.0f, z_sp = 0.0f;
	float vx_sp = 0.0f, vy_sp = 0.0f, vz_sp = 0.0f;
	float ax_sp = 0.0f, ay_sp = 0.0f, az_sp = 0.0f;

	// Reset hold position when disarmed or landed
	if (!control_mode.flag_armed || land_detected.landed) {
		_hold_position_initialized = false;
	}

	constexpr float KP_Z = 1.2f;      // position proportional gain
	constexpr float KD_Z = 0.4f;      // position derivative gain
	constexpr float KP_VZ = 1.2f;      // velocity proportional gain
	constexpr float KD_VZ = 0.4f;      // velocity derivative gain
	constexpr float THR_MIN = 0.3f;
	constexpr float THR_MAX = 1.0f;
	constexpr float MAX_VZ = 1.5f;    // m/s up

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

		// Position and velocity controllers for Z axis
		const float z_err  = z_sp - local_pos.z;
		vz_sp = z_err * KP_Z + (z_err - _prev_error_z) * KD_Z;
		const float vz_sp_limited = math::constrain(vz_sp, -MAX_VZ, MAX_VZ);
		const float vz_err = vz_sp_limited - local_pos.vz;

		// Desired vertical acceleration
		az_sp = vz_err * KP_VZ + (vz_err - _prev_error_vz) * KD_VZ + _integrator_z;

		// ### Adaptive hover thrust estimation ###
		const float thrust_from_vz = vz_err * KP_VZ;
		const float thrust_derivative = az_sp * 0.01f;

		const float thrust_cmd_unclamped = _thr_hover_est + thrust_from_vz + thrust_derivative;
		float thrust_cmd = math::constrain(thrust_cmd_unclamped, THR_MIN, THR_MAX);


		bool thrust_saturated = !isEqualF(thrust_cmd, thrust_cmd_unclamped, 1e-6f);
		if (!thrust_saturated) {
			_integrator_z += z_err * dt;
		}

		if (fabsf(vz_err) > _thr_adapt_deadband) {
			_thr_hover_est += (_thr_adapt_rate * vz_err) * dt;
		}
		_thr_hover_est = math::constrain(_thr_hover_est, _thr_adapt_min, _thr_adapt_max);

		// przypisz finalny thrust
		thrust_z = thrust_cmd;

		// --- update prev errors (ważne, inaczej D będzie niepoprawne) ---
		_prev_error_z = z_err;
		_prev_error_vz = vz_err;

		// --- THROTTLED DEBUG LOG (co ~200 ms) ---
		static hrt_abstime _last_dbg_ts = 0;
		const hrt_abstime dbg_interval_us = 200000; // 200 ms

		if (now - _last_dbg_ts > dbg_interval_us) {
			_last_dbg_ts = now;

			PX4_INFO("IFO_DBG: armed=%u alt_ctrl=%u z_valid=%u vz_valid=%u",
				(unsigned)control_mode.flag_armed,
				(unsigned)control_mode.flag_control_altitude_enabled,
				(unsigned)local_pos.z_valid,
				(unsigned)local_pos.v_z_valid);

			PX4_INFO("IFO_DBG: z=%.3f m vz=%.3f m/s z_sp=%.3f z_err=%.3f vz_sp=%.3f vz_err=%.3f",
				(double)local_pos.z, (double)local_pos.vz,
				(double)z_sp, (double)z_err,
				(double)vz_sp_limited, (double)vz_err);

			PX4_INFO("IFO_DBG: thr_est=%.3f thr_unclamped=%.3f thr_cmd=%.3f min=%.3f max=%.3f",
				(double)_thr_hover_est, (double)thrust_cmd_unclamped,
				(double)thrust_cmd, (double)THR_MIN, (double)THR_MAX);

			if (fabs(thrust_cmd - thrust_cmd_unclamped) > 1e-6f) {
				PX4_WARN("IFO_DBG: thrust was constrained (unclamped=%.3f -> clamped=%.3f)",
					(double)thrust_cmd_unclamped, (double)thrust_cmd);
			}

			// jeśli estymata hover siedzi na granicy, wypisz ostrzeżenie
			if (_thr_hover_est <= _thr_adapt_min + 1e-6f || _thr_hover_est >= _thr_adapt_max - 1e-6f) {
				PX4_WARN("IFO_DBG: thr_hover_est at bound: %.3f (min=%.3f max=%.3f)",
					(double)_thr_hover_est, (double)_thr_adapt_min, (double)_thr_adapt_max);
			}
		}
	}

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
