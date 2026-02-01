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

		_lim_thr_min = _param_ifo_thr_min.get();
		_lim_thr_max = _param_ifo_thr_max.get();
		_thr_adapt_min = _lim_thr_min;
		_thr_adapt_max = _lim_thr_max;

		if (_param_ifo_thr_hover.get() > _lim_thr_max || _param_ifo_thr_hover.get() < _lim_thr_min) {
			setHoverThrust(math::constrain(_param_ifo_thr_hover.get(), _lim_thr_min, _lim_thr_max));
			_hover_thrust_initialized = true;

		} else if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			setHoverThrust(_param_ifo_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_ifo_vel_z_p.get());
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

	// Update hover thrust estimate (if enabled)
	if (_param_mpc_use_hte.get()) {
		hover_thrust_estimate_s hte{};

		if (_hover_thrust_estimate_sub.update(&hte)) {
			if (hte.valid) {
				updateHoverThrust(hte.hover_thrust);
			}
		}
	}

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
	// TAKEOFF HANDLING (smooth ramp + hover thrust handling)
	// ================================================================
	const bool want_takeoff =
		control_mode.flag_armed && has_trajectory_setpoint && (
			(PX4_ISFINITE(traj_sp.position[2]) && local_pos.z_valid && (traj_sp.position[2] < local_pos.z)) ||
			(PX4_ISFINITE(traj_sp.velocity[2]) && (traj_sp.velocity[2] < 0.f)) ||
			(PX4_ISFINITE(traj_sp.acceleration[2]) && (traj_sp.acceleration[2] < 0.f))
		);

	if (control_mode.flag_control_altitude_enabled) {
		_takeoff.updateTakeoffState(control_mode.flag_armed, land_detected.landed, want_takeoff,
					    _lim_vel_up, _param_com_throw_en.get(), now);

	} else {
		// keep takeoff state in sync even when not controlling altitude
		_takeoff.updateTakeoffState(control_mode.flag_armed, land_detected.landed, false, 10.f, true, now);
	}

	const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);
	const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);
	const bool flying_but_ground_contact = (flying && land_detected.ground_contact);

	if (!flying) {
		setHoverThrust(_param_ifo_thr_hover.get());
	}

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
		_takeoff_target_valid = false;
		_takeoff_hold = false;
		_hover_sp_valid = false;
	}

	const float KP_Z = _param_ifo_pos_z_p.get();      // position proportional gain
	const float KD_Z = 0.4f;                          // position derivative gain
	const float KP_VZ = _param_ifo_vel_z_p.get();     // velocity proportional gain
	const float KD_VZ = 0.4f;                         // velocity derivative gain
	const float MAX_VZ_DOWN = _lim_vel_down;

	if (control_mode.flag_armed &&
	    control_mode.flag_control_altitude_enabled &&
	    local_pos.z_valid && local_pos.v_z_valid) {
		const bool suppress_control = (not_taken_off || flying_but_ground_contact);

		if (suppress_control) {
			thrust_z = 0.0f;
			_integrator_z = 0.0f;
			_prev_error_z = 0.0f;
			_prev_error_vz = 0.0f;
			_last_acc_sp_z = 0.0f;
			if (!_hold_position_initialized && local_pos.z_valid) {
				_hold_z = local_pos.z;
				_hold_position_initialized = true;
			}
			_hover_sp_valid = false;
		}

		const bool in_auto_takeoff = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);

		if (in_auto_takeoff) {
			if (!_takeoff_target_valid) {
				// Use a single fixed takeoff target relative to local origin (z=0)
				_takeoff_target_z = -_param_mis_takeoff_alt.get();
				_takeoff_target_valid = true;
				_takeoff_hold = false;
			}

			if (_takeoff_target_valid) {
				z_sp = _takeoff_target_z;

				if (fabsf(local_pos.z - _takeoff_target_z) < 0.3f && fabsf(local_pos.vz) < 0.3f) {
					_hold_z = local_pos.z;
					_hold_position_initialized = true;
					_takeoff_hold = true;
				}
			}

		} else if (has_trajectory_setpoint && PX4_ISFINITE(traj_sp.position[2])) {
			z_sp = traj_sp.position[2];
		} else {
			// HOLD current altitude if nothing commanded
			if (!_hold_position_initialized && local_pos.z_valid) {
				_hold_z = local_pos.z;
				_hold_position_initialized = true;
			}
			z_sp = _hold_z;
		}

		if (!in_auto_takeoff) {
			_takeoff_target_valid = false;
			_takeoff_hold = false;
		}

		if (_takeoff_hold) {
			z_sp = _hold_z;
		}

		if (!suppress_control) {
			const float max_vz_up = _takeoff.updateRamp(dt, _lim_vel_up);
			// Position and velocity controllers for Z axis
			const float z_err  = z_sp - local_pos.z;
			constexpr float Z_ERR_FAR = 0.5f;   // m
			constexpr float Z_ERR_NEAR = 0.1f;  // m
			const float abs_z_err = fabsf(z_err);

			if (abs_z_err > Z_ERR_FAR) {
				// Far from target: drive velocity directly toward setpoint
				vz_sp = math::constrain(z_err * KP_Z, -max_vz_up, MAX_VZ_DOWN);
				_hover_sp_valid = false;

			} else {
				// Near target: adaptively scale velocity setpoint and remember hover thrust
				const float scale = math::constrain(abs_z_err / Z_ERR_NEAR, 0.0f, 1.0f);
				vz_sp = (z_err * KP_Z + (z_err - _prev_error_z) * KD_Z) * scale;

				if (!_hover_sp_valid) {
					_hover_thrust_sp = _thr_hover_est;
					_hover_sp_valid = true;
				}
			}

			const float vz_sp_limited = math::constrain(vz_sp, -max_vz_up, MAX_VZ_DOWN);
			const float vz_err = vz_sp_limited - local_pos.vz;

			const float abs_vz_err = fabsf(vz_err);
			constexpr float VZ_ERR_SMALL = 1.0f;  // m/s
			constexpr float VZ_ERR_LARGE = 5.0f;  // m/s
			const float gain_scale = 1.0f + 0.5f * math::constrain(1.0f - abs_vz_err / VZ_ERR_SMALL, 0.0f, 1.0f);
			const float kp_vz_scaled = KP_VZ * gain_scale;

			// Desired vertical acceleration
			az_sp = vz_err * kp_vz_scaled + (vz_err - _prev_error_vz) * KD_VZ + _integrator_z;

			// NED: positive az_sp means accelerate down -> reduce thrust
			const float hover_sp = _hover_sp_valid ? _hover_thrust_sp : _thr_hover_est;
			const float thrust_cmd_unclamped = hover_sp - az_sp * (hover_sp / CONSTANTS_ONE_G);
			const float min_thrust = flying ? _lim_thr_min : 0.0f;
			float thrust_cmd = math::constrain(thrust_cmd_unclamped, min_thrust, _lim_thr_max);

			bool thrust_saturated = !isEqualF(thrust_cmd, thrust_cmd_unclamped, 1e-6f);

			if (!thrust_saturated) {
				_integrator_z += vz_err * dt * 0.5f; // integrator gain
				_integrator_z = math::constrain(_integrator_z, -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
			}

			if (abs_vz_err > VZ_ERR_LARGE) {
				thrust_cmd = math::min(thrust_cmd, _thr_hover_est);
				_integrator_z = 0.0f;
			}

			if (!_hover_sp_valid && fabsf(vz_err) > _thr_adapt_deadband) {
				_thr_hover_est += (_thr_adapt_rate * vz_err) * dt;
			}
			_thr_hover_est = math::constrain(_thr_hover_est, _thr_adapt_min, _thr_adapt_max);

			// assign final thrust
			thrust_z = thrust_cmd;

			_last_acc_sp_z = az_sp;

			// --- update prev errors (important for D term) ---
			_prev_error_z = z_err;
			_prev_error_vz = vz_err;
			// --- THROTTLED DEBUG LOG (co ~1 s) ---
			static hrt_abstime _last_dbg_ts = 0;
			const hrt_abstime dbg_interval_us = 1000000; // 1 s

			if (now - _last_dbg_ts > dbg_interval_us) {
				_last_dbg_ts = now;
				const double t_s = (double)now * 1e-6;

				PX4_INFO("IFO_DBG: t=%.1f s armed=%u alt_ctrl=%u z_valid=%u vz_valid=%u",
					t_s,
					(unsigned)control_mode.flag_armed,
					(unsigned)control_mode.flag_control_altitude_enabled,
					(unsigned)local_pos.z_valid,
					(unsigned)local_pos.v_z_valid);

				PX4_INFO("IFO_DBG: nav_state=%u MIS_TAKEOFF_ALT=%.2f",
					(unsigned)vehicle_status.nav_state,
					(double)_param_mis_takeoff_alt.get());

				PX4_INFO("IFO_DBG: z=%.3f m vz=%.3f m/s z_sp=%.3f z_err=%.3f vz_sp=%.3f vz_err=%.3f",
					(double)local_pos.z, (double)local_pos.vz,
					(double)z_sp, (double)z_err,
					(double)vz_sp_limited, (double)vz_err);

				PX4_INFO("IFO_DBG: thr_est=%.3f thr_unclamped=%.3f thr_cmd=%.3f min=%.3f max=%.3f",
					(double)_thr_hover_est, (double)thrust_cmd_unclamped,
					(double)thrust_cmd, (double)min_thrust, (double)_lim_thr_max);

				if (has_trajectory_setpoint) {
					PX4_INFO("IFO_DBG: traj_sp pos_z=%.3f vel_z=%.3f acc_z=%.3f yaw=%.3f",
						(double)traj_sp.position[2],
						(double)traj_sp.velocity[2],
						(double)traj_sp.acceleration[2],
						(double)traj_sp.yaw);
				}

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

void IfodronePositionControl::setHoverThrust(const float hover_thrust)
{
	_thr_hover_est = math::constrain(hover_thrust, _thr_adapt_min, _thr_adapt_max);
}

void IfodronePositionControl::updateHoverThrust(const float hover_thrust_new)
{
	const float previous_hover_thrust = _thr_hover_est;
	setHoverThrust(hover_thrust_new);

	// wzor z PositionControl:
	// a_sp' = (a_sp - g) * Th / Th' + g
	// integrator += a_sp' - a_sp
	const float a_sp = _last_acc_sp_z;
	const float Th = previous_hover_thrust;
	const float Thp = _thr_hover_est;

	if (Thp > 1e-6f) {
		const float a_sp_new = (a_sp - CONSTANTS_ONE_G) * Th / Thp + CONSTANTS_ONE_G;
		_integrator_z += (a_sp_new - a_sp);
		// ogranicz integrator
		_integrator_z = math::constrain(_integrator_z, -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
	}
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
