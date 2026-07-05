#include "IfodroneLevelAdapter.hpp"

#include <lib/mathlib/mathlib.h>

using namespace matrix;

IfodroneLevelAdapter::IfodroneLevelAdapter() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

IfodroneLevelAdapter::~IfodroneLevelAdapter()
{
	perf_free(_loop_perf);
}

bool IfodroneLevelAdapter::init()
{
	if (!_virtual_attitude_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void IfodroneLevelAdapter::Run()
{
	if (should_exit()) {
		_virtual_attitude_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	vehicle_attitude_setpoint_s virtual_sp;

	if (_virtual_attitude_setpoint_sub.update(&virtual_sp)) {

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		// Only bridge while the stock position controller owns vehicle_attitude_setpoint.
		// In pure Stabilized/Acro mc_att_control publishes it itself, so we stay silent to
		// preserve the single-publisher invariant on the topic.
		if (!_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			perf_end(_loop_perf);
			return;
		}

		const Quatf q(virtual_sp.q_d);

		if (!q.isAllFinite()) {
			perf_end(_loop_perf);
			return;
		}

		// Reconstruct the desired thrust vector in NED. mc_pos_control encodes the thrust
		// direction in q_d and the magnitude in thrust_body[2] (negative = up); thrust_body
		// [0/1] are zero. Rotating the body thrust vector by q_d recovers the NED thrust.
		Vector3f thrust_body_in(virtual_sp.thrust_body);

		for (int i = 0; i < 3; i++) {
			if (!PX4_ISFINITE(thrust_body_in(i))) { thrust_body_in(i) = 0.f; }
		}

		const Vector3f thrust_ned = Dcmf(q) * thrust_body_in;

		// Heading of the commanded attitude — the only rotation the level body keeps.
		const float yaw = Eulerf(q).psi();

		// Re-express the NED thrust in the LEVEL body frame (rotate by heading only), so the
		// horizontal demand lands in thrust_body[0/1] for the side EDFs and the vertical
		// demand stays in thrust_body[2] for the coaxial pair.
		const Vector3f thrust_body_level = Dcmf(Eulerf(0.f, 0.f, yaw)).transpose() * thrust_ned;

		vehicle_attitude_setpoint_s att_sp{};
		Quatf(Eulerf(0.f, 0.f, yaw)).copyTo(att_sp.q_d);
		thrust_body_level.copyTo(att_sp.thrust_body);
		att_sp.yaw_sp_move_rate = PX4_ISFINITE(virtual_sp.yaw_sp_move_rate) ? virtual_sp.yaw_sp_move_rate : 0.f;
		att_sp.timestamp = hrt_absolute_time();
		_vehicle_attitude_setpoint_pub.publish(att_sp);
	}

	perf_end(_loop_perf);
}

int IfodroneLevelAdapter::task_spawn(int argc, char *argv[])
{
	IfodroneLevelAdapter *instance = new IfodroneLevelAdapter();

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

int IfodroneLevelAdapter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IFODRONE level adapter.

Bridges the stock mc_pos_control (started with the "vtol" argument, publishing to
mc_virtual_attitude_setpoint) to the stock mc_att_control by rewriting the tilted attitude
setpoint into a LEVEL attitude setpoint (yaw only) plus a body-frame thrust vector. This keeps
the IFODRONE body level (roll/pitch = 0) while translation is produced by the side EDFs via
thrust_body[0/1] in the control allocator.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_level_adapter", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int IfodroneLevelAdapter::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

extern "C" __EXPORT int ifo_level_adapter_main(int argc, char *argv[])
{
	return IfodroneLevelAdapter::main(argc, argv);
}
