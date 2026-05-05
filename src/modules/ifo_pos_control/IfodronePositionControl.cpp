/**
 * IFODRONE Position Controller
 *
 * Setpoint resolution (priority order):
 *   1. goto_setpoint.position  — direct target, no smoothing
 *   2. trajectory_setpoint     — from flight_mode_manager (vel/accel feedforward used)
 *   3. hold last commanded position
 *
 * Cascaded P:
 *   pos_sp  →[P]→  vel_sp  →[P]→  accel_sp  →  body-frame thrust
 *
 * IFODRONE: body nominally level, no pitch/roll in force computation.
 * Only yaw rotation applied when converting NED XY → body XY.
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

	// Ramp starts at vel=0 (default _takeoff_ramp_vz_init=0), climbs to TAKEOFF_DESIRED_VZ.
	_takeoff.setSpoolupTime(TAKEOFF_SPOOLUP_TIME);
	_takeoff.setTakeoffRampTime(TAKEOFF_RAMP_TIME);

	PX4_INFO("IFODRONE position controller initialized");
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

	// --- Inputs ---
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

	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	// --- Goto setpoint (priority 1: direct target position) ---
	goto_setpoint_s goto_sp{};
	const bool goto_sp_fresh = _goto_setpoint_sub.copy(&goto_sp)
				   && (goto_sp.timestamp != 0)
				   && (now - goto_sp.timestamp < TRAJ_SP_TIMEOUT_US);

	// --- Trajectory setpoint (priority 2: from flight_mode_manager) ---
	trajectory_setpoint_s traj_sp{};
	const bool traj_sp_fresh = _trajectory_setpoint_sub.copy(&traj_sp)
				   && (traj_sp.timestamp != 0)
				   && (now <= traj_sp.timestamp + TRAJ_SP_TIMEOUT_US);

	// --- Mode flags ---
	const bool armed               = control_mode.flag_armed;
	const bool altitude_ctrl       = control_mode.flag_control_altitude_enabled;
	const bool position_ctrl       = control_mode.flag_control_position_enabled
					 || control_mode.flag_multicopter_position_control_enabled;
	const bool run_xy_control = armed && position_ctrl && local_pos.xy_valid && local_pos.v_xy_valid;
	const bool run_z_control  = armed && altitude_ctrl  && local_pos.z_valid  && local_pos.v_z_valid;

	if (!armed || !altitude_ctrl)  { _hold_position_initialized = false; }
	if (!armed || !position_ctrl)  { _hold_xy_initialized = false; }

	const Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
	const Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);

	// ---------------------------------------------------------------
	// Resolve Z setpoint
	//
	// Tracking strategy:
	//   - When a fresh setpoint with FINITE position[2] is available, follow it directly
	//     (no proximity gating: previous version stopped tracking once within 20 cm,
	//     which prevented the descend/hover phase from following the trajectory).
	//   - When the trajectory is velocity-only (NaN position[2], finite velocity[2]),
	//     skip position tracking by marking _hold_z as NAN-equivalent (use current z).
	//   - When no fresh setpoint, snap _hold_z to the CURRENT altitude so the drone
	//     stays in place instead of being yanked back to the last commanded altitude.
	// ---------------------------------------------------------------
	bool z_position_track = false;
	float z_sp = _hold_z;

	if (run_z_control) {
		// Priority 1: goto_setpoint
		if (goto_sp_fresh && PX4_ISFINITE(goto_sp.position[2])) {
			_hold_z = goto_sp.position[2];
			_hold_position_initialized = true;
			z_position_track = true;

		// Priority 2: trajectory_setpoint position
		} else if (traj_sp_fresh && PX4_ISFINITE(traj_sp.position[2])) {
			_hold_z = traj_sp.position[2];
			_hold_position_initialized = true;
			z_position_track = true;

		// Priority 3a: trajectory says "velocity only" (no position) -> follow current
		// altitude, let velocity feed-forward drive the climb/descent.
		} else if (traj_sp_fresh && PX4_ISFINITE(traj_sp.velocity[2])) {
			_hold_z = local_pos.z;
			_hold_position_initialized = true;

		// Priority 3b: nothing fresh -> snap hold to current altitude
		} else {
			_hold_z = local_pos.z;
			_hold_position_initialized = true;
		}

		z_sp = _hold_z;
	}

	_z_sp = z_sp;

	// ---------------------------------------------------------------
	// Resolve XY setpoint
	//
	// We need three behaviors to cover the user-visible cases:
	//   (A) Trajectory carries a finite XY position -> track it (HOLD/POSITION/AUTO).
	//   (B) Trajectory carries velocity only (NaN position, finite velocity)
	//       -> drone should move in that direction without being pulled back to a
	//          stale position; pos_err must be 0 so only velocity feed-forward acts.
	//   (C) Nothing fresh -> snap pos_sp to the CURRENT position so the drone holds
	//       in place (do NOT pull back toward the last commanded XY).
	// ---------------------------------------------------------------
	bool xy_position_track = false;
	Vector3f pos_sp = pos;
	pos_sp(2) = z_sp;

	if (armed && position_ctrl && local_pos.xy_valid) {
		// Priority 1: goto_setpoint XY (axis-wise FINITE check)
		const bool gx = goto_sp_fresh && PX4_ISFINITE(goto_sp.position[0]);
		const bool gy = goto_sp_fresh && PX4_ISFINITE(goto_sp.position[1]);

		// Priority 2: trajectory_setpoint XY
		const bool tx = traj_sp_fresh && PX4_ISFINITE(traj_sp.position[0]);
		const bool ty = traj_sp_fresh && PX4_ISFINITE(traj_sp.position[1]);

		// Priority 2b: velocity-only trajectory (NaN position but finite velocity)
		const bool tvx = traj_sp_fresh && PX4_ISFINITE(traj_sp.velocity[0]);
		const bool tvy = traj_sp_fresh && PX4_ISFINITE(traj_sp.velocity[1]);

		if (gx)      { _hold_x = goto_sp.position[0]; }
		else if (tx) { _hold_x = traj_sp.position[0]; }
		else         { _hold_x = local_pos.x; }   // velocity-only or stale -> snap to current

		if (gy)      { _hold_y = goto_sp.position[1]; }
		else if (ty) { _hold_y = traj_sp.position[1]; }
		else         { _hold_y = local_pos.y; }

		_hold_xy_initialized = true;
		xy_position_track = (gx || tx) && (gy || ty);

		// If trajectory is velocity-only but no position, do not engage position P
		// (pos_err will be zeroed below). The velocity feed-forward alone drives motion.
		(void)tvx; (void)tvy;

		pos_sp(0) = _hold_x;
		pos_sp(1) = _hold_y;

	} else if (!goto_sp_fresh && !traj_sp_fresh) {
		// Not in position control and no setpoint -> hold whatever we have NOW.
		_hold_x = local_pos.x;
		_hold_y = local_pos.y;
		_hold_xy_initialized = true;

		pos_sp(0) = _hold_x;
		pos_sp(1) = _hold_y;
	}

	// ---------------------------------------------------------------
	// Cascaded P: position error → velocity setpoint
	//
	// Only engage the position P loop on axes where we are actually tracking a
	// finite position setpoint. For velocity-only trajectories the velocity
	// feed-forward below is the sole velocity command.
	// ---------------------------------------------------------------
	Vector3f pos_err = pos_sp - pos;
	if (!run_xy_control || !xy_position_track) { pos_err(0) = 0.0f; pos_err(1) = 0.0f; }
	if (!run_z_control  || !z_position_track)  { pos_err(2) = 0.0f; }

	Vector3f vel_sp;
	vel_sp(0) = pos_err(0) * _param_ifo_pos_xy_p.get();
	vel_sp(1) = pos_err(1) * _param_ifo_pos_xy_p.get();
	vel_sp(2) = pos_err(2) * _param_ifo_pos_z_p.get();

	// Velocity feedforward from trajectory_setpoint (not available from goto_setpoint)
	if (!goto_sp_fresh && traj_sp_fresh) {
		if (run_xy_control) {
			if (PX4_ISFINITE(traj_sp.velocity[0])) { vel_sp(0) += traj_sp.velocity[0]; }
			if (PX4_ISFINITE(traj_sp.velocity[1])) { vel_sp(1) += traj_sp.velocity[1]; }
		}

		if (run_z_control && PX4_ISFINITE(traj_sp.velocity[2])) {
			vel_sp(2) += traj_sp.velocity[2];
		}
	}

	// ---------------------------------------------------------------
	// TakeoffHandling: clamp upward velocity during ramp
	// ---------------------------------------------------------------
	const bool want_takeoff = run_z_control && !land_detected.landed && (z_sp < local_pos.z - 0.2f);
	_takeoff.updateTakeoffState(armed, land_detected.landed, want_takeoff, TAKEOFF_DESIRED_VZ, false, now);

	if (run_z_control) {
		// ramp goes 0 → TAKEOFF_DESIRED_VZ (negative NED); max() prevents exceeding upward limit
		vel_sp(2) = math::max(vel_sp(2), _takeoff.updateRamp(dt, TAKEOFF_DESIRED_VZ));
	}

	// ---------------------------------------------------------------
	// Cascaded P: velocity error → acceleration setpoint
	// ---------------------------------------------------------------
	Vector3f vel_err = vel_sp - vel;
	if (!run_xy_control) { vel_err(0) = 0.0f; vel_err(1) = 0.0f; }
	if (!run_z_control)  { vel_err(2) = 0.0f; }

	Vector3f accel_sp;
	accel_sp(0) = vel_err(0) * _param_ifo_vel_xy_p.get();
	accel_sp(1) = vel_err(1) * _param_ifo_vel_xy_p.get();
	accel_sp(2) = vel_err(2) * _param_ifo_vel_z_p.get();

	// Acceleration feedforward from trajectory_setpoint
	if (!goto_sp_fresh && traj_sp_fresh) {
		if (run_xy_control) {
			if (PX4_ISFINITE(traj_sp.acceleration[0])) { accel_sp(0) += traj_sp.acceleration[0]; }
			if (PX4_ISFINITE(traj_sp.acceleration[1])) { accel_sp(1) += traj_sp.acceleration[1]; }
		}

		if (run_z_control && PX4_ISFINITE(traj_sp.acceleration[2])) {
			accel_sp(2) += traj_sp.acceleration[2];
		}
	}

	// ---------------------------------------------------------------
	// Yaw setpoint
	//
	// Hold current heading by default. Only override if an explicit goto_sp
	// heading is provided. We intentionally do NOT track traj_sp.yaw on every
	// cycle: in Auto, FlightTaskAuto can derive yaw from the smoothed velocity
	// vector, which during takeoff/hover oscillates with the platform and pulls
	// the heading around (observed in flight: yaw drifted to -2.8 rad while
	// the drone was supposed to hover). Holding heading keeps the body-frame
	// thrust mapping stable.
	// ---------------------------------------------------------------
	const float yaw_current = PX4_ISFINITE(local_pos.heading) ? local_pos.heading : 0.0f;
	float yaw_sp = yaw_current;

	if (goto_sp_fresh && goto_sp.flag_control_heading && PX4_ISFINITE(goto_sp.heading)) {
		yaw_sp = goto_sp.heading;

	} else if (traj_sp_fresh && PX4_ISFINITE(traj_sp.yaw) && PX4_ISFINITE(traj_sp.yawspeed)
		   && fabsf(traj_sp.yawspeed) > 0.05f) {
		// Only follow the trajectory yaw when an explicit yaw rate is being commanded.
		yaw_sp = traj_sp.yaw;
	}

	const float yawspeed_sp = (!goto_sp_fresh && traj_sp_fresh && PX4_ISFINITE(traj_sp.yawspeed))
				  ? traj_sp.yawspeed : 0.0f;

	// ---------------------------------------------------------------
	// Acceleration → body-frame thrust
	// IFODRONE: body level, no pitch/roll. Yaw rotation only.
	// ---------------------------------------------------------------
	Vector3f thrust_sp_body;
	thrust_sp_body.setZero();

	if (run_xy_control || run_z_control) {
		const float cos_yaw = cosf(yaw_current);
		const float sin_yaw = sinf(yaw_current);

		// NED XY → body XY (yaw rotation only, no tilt)
		const float ax_body =  cos_yaw * accel_sp(0) + sin_yaw * accel_sp(1);
		const float ay_body = -sin_yaw * accel_sp(0) + cos_yaw * accel_sp(1);

		thrust_sp_body(0) = run_xy_control ? ax_body / CONSTANTS_ONE_G : 0.0f;
		thrust_sp_body(1) = run_xy_control ? ay_body / CONSTANTS_ONE_G : 0.0f;

		// Clamp XY thrust magnitude
		const float thr_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.0f, 1.0f);
		Vector2f thr_xy(thrust_sp_body(0), thrust_sp_body(1));
		const float thr_xy_norm = thr_xy.norm();

		if (thr_xy_norm > thr_xy_max && thr_xy_norm > 1e-5f) {
			thr_xy *= thr_xy_max / thr_xy_norm;
			thrust_sp_body(0) = thr_xy(0);
			thrust_sp_body(1) = thr_xy(1);
		}

		// Z thrust: hover baseline corrected by NED Z acceleration.
		// No tilt compensation — IFODRONE main motors always along body Z.
		// NED: accel_sp(2) > 0 means down → less upward thrust needed.
		const float hover_thr = math::constrain(_param_ifo_thr_hover.get(), 0.0f, 1.0f);
		const float thr_min   = math::constrain(_param_ifo_thr_min.get(),   0.0f, 1.0f);
		const float thr_max   = math::constrain(_param_ifo_thr_max.get(),   thr_min, 1.0f);
		const float thrust_z  = math::constrain(
						hover_thr - accel_sp(2) * (hover_thr / CONSTANTS_ONE_G),
						thr_min, thr_max);
		thrust_sp_body(2) = run_z_control ? -thrust_z : 0.0f;
	}

	// ---------------------------------------------------------------
	// Publish
	// ---------------------------------------------------------------
	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp       = now;
	local_pos_sp.x               = run_xy_control ? pos_sp(0)        : NAN;
	local_pos_sp.y               = run_xy_control ? pos_sp(1)        : NAN;
	local_pos_sp.z               = run_z_control  ? pos_sp(2)        : NAN;
	local_pos_sp.vx              = run_xy_control ? vel_sp(0)        : NAN;
	local_pos_sp.vy              = run_xy_control ? vel_sp(1)        : NAN;
	local_pos_sp.vz              = run_z_control  ? vel_sp(2)        : NAN;
	local_pos_sp.acceleration[0] = run_xy_control ? accel_sp(0)      : NAN;
	local_pos_sp.acceleration[1] = run_xy_control ? accel_sp(1)      : NAN;
	local_pos_sp.acceleration[2] = run_z_control  ? accel_sp(2)      : NAN;
	local_pos_sp.thrust[0]       = run_xy_control ? thrust_sp_body(0) : NAN;
	local_pos_sp.thrust[1]       = run_xy_control ? thrust_sp_body(1) : NAN;
	local_pos_sp.thrust[2]       = run_z_control  ? thrust_sp_body(2) : NAN;
	local_pos_sp.yaw             = yaw_sp;
	local_pos_sp.yawspeed        = yawspeed_sp;
	_local_pos_sp_pub.publish(local_pos_sp);

	vehicle_thrust_setpoint_s thrust_msg{};
	thrust_msg.timestamp        = now;
	thrust_msg.timestamp_sample = local_pos.timestamp_sample;
	thrust_msg.xyz[0]           = thrust_sp_body(0);
	thrust_msg.xyz[1]           = thrust_sp_body(1);
	thrust_msg.xyz[2]           = thrust_sp_body(2);
	_thrust_sp_pub.publish(thrust_msg);

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp        = now;
	att_sp.yaw_sp_move_rate = yawspeed_sp;
	const Quatf q_sp(Eulerf(0.0f, 0.0f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);
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
	PX4_INFO("IFODRONE Position Controller  z_sp=%.3f  takeoff=%d",
		 (double)_z_sp, (int)_takeoff.getTakeoffState());
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfodronePositionControl::main(argc, argv);
}
