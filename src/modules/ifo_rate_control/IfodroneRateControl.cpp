#include "IfodroneRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

IfodroneRateControl::IfodroneRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

IfodroneRateControl::~IfodroneRateControl()
{
	perf_free(_loop_perf);
}

bool IfodroneRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void IfodroneRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_angular_velocity_s ang_vel;

	if (!_vehicle_angular_velocity_sub.update(&ang_vel)) {
		perf_end(_loop_perf);
		return;
	}

	const float dt = math::constrain(((ang_vel.timestamp_sample - _last_run) * 1e-6f), 0.0001f, 0.02f);
	_last_run = ang_vel.timestamp_sample;

	const float roll_rate  = ang_vel.xyz[0];
	const float pitch_rate = ang_vel.xyz[1];
	const float yaw_rate   = ang_vel.xyz[2];

	// ── Low-pass filtered rate derivative (for D term) ────────────────
	_roll_rate_dot  = DERIV_FILTER * ((roll_rate  - _prev_roll_rate)  / dt) + (1.f - DERIV_FILTER) * _roll_rate_dot;
	_pitch_rate_dot = DERIV_FILTER * ((pitch_rate - _prev_pitch_rate) / dt) + (1.f - DERIV_FILTER) * _pitch_rate_dot;
	_yaw_rate_dot   = DERIV_FILTER * ((yaw_rate   - _prev_yaw_rate)   / dt) + (1.f - DERIV_FILTER) * _yaw_rate_dot;

	_prev_roll_rate  = roll_rate;
	_prev_pitch_rate = pitch_rate;
	_prev_yaw_rate   = yaw_rate;

	// ── Read rate setpoint ────────────────────────────────────────────
	vehicle_rates_setpoint_s rates_sp{};

	if (!_vehicle_rates_setpoint_sub.copy(&rates_sp) || rates_sp.timestamp == 0) {
		// No setpoint yet — publish zero torque
		vehicle_torque_setpoint_s torque_sp{};
		torque_sp.timestamp        = hrt_absolute_time();
		torque_sp.timestamp_sample = ang_vel.timestamp_sample;
		_vehicle_torque_setpoint_pub.publish(torque_sp);
		perf_end(_loop_perf);
		return;
	}

	// ── PD on rate error → torque ─────────────────────────────────────
	const float kp  = _param_ifo_rate_p.get();
	const float kd  = _param_ifo_rate_d.get();
	const float kyp = _param_ifo_yrate_p.get();
	const float kyd = _param_ifo_yrate_d.get();

	const float roll_rate_err  = rates_sp.roll  - roll_rate;
	const float pitch_rate_err = rates_sp.pitch - pitch_rate;
	const float yaw_rate_err   = rates_sp.yaw   - yaw_rate;

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp        = hrt_absolute_time();
	torque_sp.timestamp_sample = ang_vel.timestamp_sample;
	torque_sp.xyz[0] = kp  * roll_rate_err  - kd  * _roll_rate_dot;
	torque_sp.xyz[1] = kp  * pitch_rate_err - kd  * _pitch_rate_dot;
	torque_sp.xyz[2] = kyp * yaw_rate_err   - kyd * _yaw_rate_dot;
	_vehicle_torque_setpoint_pub.publish(torque_sp);

	perf_end(_loop_perf);
}

int IfodroneRateControl::task_spawn(int argc, char *argv[])
{
	IfodroneRateControl *instance = new IfodroneRateControl();

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

int IfodroneRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IFODRONE inner-loop rate controller (PD).
Reads vehicle_rates_setpoint and vehicle_angular_velocity,
publishes vehicle_torque_setpoint for the control allocator.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int IfodroneRateControl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

extern "C" __EXPORT int ifo_rate_control_main(int argc, char *argv[])
{
	return IfodroneRateControl::main(argc, argv);
}
