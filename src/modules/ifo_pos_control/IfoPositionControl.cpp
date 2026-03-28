/**
 * IFODRONE Position Control Filter
 *
 * Intercepts mc_pos_control's NED thrust and republishes
 * vehicle_attitude_setpoint with a level body and body-frame thrust.
 */

#include "IfoPositionControl.hpp"

#include <lib/mathlib/mathlib.h>

using namespace matrix;

IfoPositionControl::IfoPositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

IfoPositionControl::~IfoPositionControl()
{
	perf_free(_cycle_perf);
}

bool IfoPositionControl::init()
{
	if (!_local_pos_sp_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("IFO position filter initialized (mode=%d)", _param_ifo_pos_mode.get());
	return true;
}

void IfoPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sp_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Handle parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Skip if filter disabled
	if (_param_ifo_pos_mode.get() <= 0) {
		perf_end(_cycle_perf);
		return;
	}

	// Read the local position setpoint published by mc_pos_control
	vehicle_local_position_setpoint_s local_pos_sp{};
	const hrt_abstime now = hrt_absolute_time();

	if (!_local_pos_sp_sub.copy(&local_pos_sp)) {
		perf_end(_cycle_perf);
		return;
	}

	// Get current vehicle attitude only as fallback when mc_pos_control does not provide yaw.
	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);
	const Eulerf euler_current(Quatf(att.q));

	// mc_pos_control's thrust is in NED frame (via PositionControl::_thr_sp)
	const float thrx_I = PX4_ISFINITE(local_pos_sp.thrust[0]) ? local_pos_sp.thrust[0] : 0.f;
	const float thry_I = PX4_ISFINITE(local_pos_sp.thrust[1]) ? local_pos_sp.thrust[1] : 0.f;
	const float thrz_I = PX4_ISFINITE(local_pos_sp.thrust[2]) ? local_pos_sp.thrust[2] : 0.f;

	// Use navigator yaw setpoint; fall back to current yaw to avoid sudden rotations.
	const float yaw_sp_traj = PX4_ISFINITE(local_pos_sp.yaw) ? local_pos_sp.yaw : NAN;
	const float yaw_sp = PX4_ISFINITE(yaw_sp_traj) ? yaw_sp_traj : euler_current.psi();

	const float yawspeed_sp = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;

	// local_pos_sp.thrust already contains the normalized closed-loop output of mc_pos_control.
	// This filter should only rotate and clamp it for the IFODRONE allocator, not run another XY loop on top.
	Vector2f thrust_xy_I{thrx_I, thry_I};
	thrust_xy_I *= math::max(_param_ifo_xy_thr_scl.get(), 0.f);

	// Attitude setpoint quaternion: level body (roll=0, pitch=0) with navigator yaw.
	const Quatf q_sp(Eulerf(0.f, 0.f, yaw_sp));

	// Rotate into the level body frame commanded for IFODRONE. Using the actual roll/pitch here
	// would project vertical thrust into XY and break the normalized thrust contract when attitude diverges.
	const Dcmf R_IB = Dcmf(q_sp).transpose();
	const Vector3f thrust_I{thrust_xy_I(0), thrust_xy_I(1), thrz_I};
	const Vector3f thrust_body = R_IB * thrust_I;

	float thrx_B = thrust_body(0);
	float thry_B = thrust_body(1);
	float thrz_B = thrust_body(2);

	// Clamp horizontal thrust to IFO_THR_XY_MAX
	const float thr_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.f, 1.f);
	Vector2f thr_xy(thrx_B, thry_B);
	const float thr_xy_norm = thr_xy.norm();

	if (thr_xy_norm > thr_xy_max && thr_xy_norm > 1e-5f) {
		thr_xy *= thr_xy_max / thr_xy_norm;
		thrx_B = thr_xy(0);
		thry_B = thr_xy(1);
	}

	// Keep the published body-frame thrust normalized. Main motors only provide upward body-Z thrust.
	thrx_B = math::constrain(thrx_B, -1.f, 1.f);
	thry_B = math::constrain(thry_B, -1.f, 1.f);
	thrz_B = math::constrain(thrz_B, -1.f, 0.f);

	// Publish body-frame thrust directly to ControlAllocator
	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = now;
	thrust_sp.timestamp_sample = att.timestamp;
	thrust_sp.xyz[0] = thrx_B;
	thrust_sp.xyz[1] = thry_B;
	thrust_sp.xyz[2] = thrz_B;
	_vehicle_thrust_setpoint_pub.publish(thrust_sp);

	// Build level attitude setpoint: roll=0, pitch=0, desired yaw
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = now;
	att_sp.yaw_sp_move_rate = yawspeed_sp;

	q_sp.copyTo(att_sp.q_d);

	// Body-frame thrust: XY from tilt motors, Z from main motors
	att_sp.thrust_body[0] = thrx_B;
	att_sp.thrust_body[1] = thry_B;
	att_sp.thrust_body[2] = thrz_B;

	_vehicle_attitude_setpoint_pub.publish(att_sp);

	// static hrt_abstime last_debug{0};

	// if (hrt_elapsed_time(&last_debug) > 250_ms) {
	// 	PX4_INFO("IFO_POS sp_pos=(%.2f %.2f %.2f) sp_vel=(%.2f %.2f %.2f) sp_acc=(%.2f %.2f %.2f) "
	// 		 "thr_I=(%.2f %.2f %.2f) yaw_sp=%.1f yaw_curr=%.1f thr_B=(%.2f %.2f %.2f)",
	// 		 (double)local_pos_sp.x, (double)local_pos_sp.y, (double)local_pos_sp.z,
	// 		 (double)local_pos_sp.vx, (double)local_pos_sp.vy, (double)local_pos_sp.vz,
	// 		 (double)local_pos_sp.acceleration[0], (double)local_pos_sp.acceleration[1],
	// 		 (double)local_pos_sp.acceleration[2],
	// 		 (double)thrx_I, (double)thry_I, (double)thrz_I,
	// 		 (double)math::degrees(yaw_sp), (double)math::degrees(euler_current.psi()),
	// 		 (double)thrx_B, (double)thry_B, (double)thrz_B);
	// 	last_debug = now;
	// }

	perf_end(_cycle_perf);
}

int IfoPositionControl::task_spawn(int argc, char *argv[])
{
	IfoPositionControl *instance = new IfoPositionControl();

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

int IfoPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude setpoint filter for IFODRONE. Sits after mc_pos_control and rewrites
vehicle_attitude_setpoint to keep body level, redistributing thrust to body-frame
XY (tilt motors) and Z (main motors).

Activated by IFO_POS_MODE parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfoPositionControl::main(argc, argv);
}
