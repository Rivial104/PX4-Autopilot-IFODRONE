/**
 * IFODRONE Position Control Filter
 *
 * Reads NED thrust from mc_pos_control's local_position_setpoint,
 * rotates XY into body frame, publishes level attitude setpoint.
 */

#include "IfoPositionControl.hpp"

using namespace matrix;

IfoPositionControl::IfoPositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool IfoPositionControl::init()
{
	if (!_local_pos_sp_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void IfoPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sp_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s p;
		_parameter_update_sub.copy(&p);
		updateParams();
	}

	if (_param_ifo_pos_mode.get() <= 0) {
		return;
	}

	vehicle_local_position_setpoint_s sp{};

	if (!_local_pos_sp_sub.copy(&sp)) {
		return;
	}

	// Current yaw for NED → body rotation
	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);
	const float yaw = Eulerf(Quatf(att.q)).psi();

	// Rotate NED XY thrust into body frame
	const float c = cosf(yaw);
	const float s = sinf(yaw);

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = hrt_absolute_time();

	// Level body, desired yaw
	const float yaw_sp = PX4_ISFINITE(sp.yaw) ? sp.yaw : yaw;
	Quatf(Eulerf(0.f, 0.f, yaw_sp)).copyTo(att_sp.q_d);
	att_sp.yaw_sp_move_rate = PX4_ISFINITE(sp.yawspeed) ? sp.yawspeed : 0.f;

	// Body-frame thrust
	att_sp.thrust_body[0] =  c * sp.thrust[0] + s * sp.thrust[1];
	att_sp.thrust_body[1] = -s * sp.thrust[0] + c * sp.thrust[1];
	att_sp.thrust_body[2] = sp.thrust[2];

	_att_sp_pub.publish(att_sp);
}

int IfoPositionControl::task_spawn(int argc, char *argv[])
{
	auto *instance = new IfoPositionControl();

	if (instance && instance->init()) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int IfoPositionControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_USAGE_NAME("ifo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ifo_pos_control_main(int argc, char *argv[])
{
	return IfoPositionControl::main(argc, argv);
}
