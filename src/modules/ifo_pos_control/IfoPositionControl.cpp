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

	if (!_local_pos_sp_sub.copy(&local_pos_sp)) {
		perf_end(_cycle_perf);
		return;
	}

	// Get current vehicle attitude for yaw (NED→body rotation)
	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);
	const Quatf q_current(att.q);
	const Eulerf euler_current(q_current);
	const float yaw_current = euler_current.psi();

	// mc_pos_control's thrust is in NED frame (via PositionControl::_thr_sp)
	const float thr_ned_x = PX4_ISFINITE(local_pos_sp.thrust[0]) ? local_pos_sp.thrust[0] : 0.f;
	const float thr_ned_y = PX4_ISFINITE(local_pos_sp.thrust[1]) ? local_pos_sp.thrust[1] : 0.f;
	const float thr_ned_z = PX4_ISFINITE(local_pos_sp.thrust[2]) ? local_pos_sp.thrust[2] : 0.f;

	// Rotate NED XY thrust into body frame using current yaw
	const float cos_yaw = cosf(yaw_current);
	const float sin_yaw = sinf(yaw_current);
	float thr_body_x =  cos_yaw * thr_ned_x + sin_yaw * thr_ned_y;
	float thr_body_y = -sin_yaw * thr_ned_x + cos_yaw * thr_ned_y;

	// Clamp horizontal thrust to IFO_THR_XY_MAX
	const float thr_xy_max = math::constrain(_param_ifo_thr_xy_max.get(), 0.f, 1.f);
	Vector2f thr_xy(thr_body_x, thr_body_y);
	const float thr_xy_norm = thr_xy.norm();

	if (thr_xy_norm > thr_xy_max && thr_xy_norm > 1e-5f) {
		thr_xy *= thr_xy_max / thr_xy_norm;
		thr_body_x = thr_xy(0);
		thr_body_y = thr_xy(1);
	}

	// Yaw setpoint from mc_pos_control
	const float yaw_sp = PX4_ISFINITE(local_pos_sp.yaw) ? local_pos_sp.yaw : yaw_current;
	const float yawspeed_sp = PX4_ISFINITE(local_pos_sp.yawspeed) ? local_pos_sp.yawspeed : 0.f;

	// Build level attitude setpoint: roll=0, pitch=0, desired yaw
	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = hrt_absolute_time();
	att_sp.yaw_sp_move_rate = yawspeed_sp;

	const Quatf q_sp(Eulerf(0.f, 0.f, yaw_sp));
	q_sp.copyTo(att_sp.q_d);

	// Body-frame thrust: XY from tilt motors, Z from main motors
	// When body is level, NED Z maps directly to body Z
	att_sp.thrust_body[0] = thr_body_x;
	att_sp.thrust_body[1] = thr_body_y;
	att_sp.thrust_body[2] = thr_ned_z;

	_vehicle_attitude_setpoint_pub.publish(att_sp);

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
