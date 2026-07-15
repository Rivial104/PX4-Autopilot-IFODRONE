#include "IfodroneAttitudeControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

IfodroneAttitudeControl::IfodroneAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

IfodroneAttitudeControl::~IfodroneAttitudeControl()
{
	perf_free(_loop_perf);
}

bool IfodroneAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void IfodroneAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_attitude_s att;

	if (!_vehicle_attitude_sub.update(&att)) {
		perf_end(_loop_perf);
		return;
	}

	const float dt = math::constrain(((att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	_last_run = att.timestamp_sample;

	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	// ── Current attitude ──────────────────────────────────────────────
	const Eulerf euler(Quatf(att.q));
	const float roll  = euler.phi();
	const float pitch = euler.theta();
	const float yaw   = euler.psi();

	// ── Mode ──────────────────────────────────────────────────────────
	// "manual" here = stabilized manual flight without position/altitude assist.
	const bool manual_mode = _vehicle_control_mode.flag_control_manual_enabled &&
				 !_vehicle_control_mode.flag_control_altitude_enabled &&
				 !_vehicle_control_mode.flag_control_velocity_enabled &&
				 !_vehicle_control_mode.flag_control_position_enabled;

	// ── Yaw setpoint + thrust source ──────────────────────────────────
	float    yaw_rate_ff = 0.f;
	Vector3f thrust_body{0.f, 0.f, 0.f};
	float    roll_sp  = 0.f;   // always level; tracks att_sp (which publishes 0) in Auto/Position
	float    pitch_sp = 0.f;

	if (manual_mode) {
		// Yaw stick integrates the heading setpoint.
		if (!PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_setpoint = yaw;
		}

		const float yaw_stick_rate = _manual_control_setpoint.yaw * YAW_RATE_MAX;
		_yaw_setpoint = wrap_pi(_yaw_setpoint + yaw_stick_rate * dt);
		yaw_rate_ff   = yaw_stick_rate;

		// Throttle stick → vertical thrust (level body: XY thrust = 0).
		const float throttle_raw = (_manual_control_setpoint.throttle + 1.f) * 0.5f;
		const float throttle     = THROTTLE_IDLE + throttle_raw * (1.f - THROTTLE_IDLE);
		thrust_body(2) = -throttle;

	} else {
		// Auto / Position: yaw and thrust come from ifo_pos_control via vehicle_attitude_setpoint.
		_yaw_setpoint = NAN;

		vehicle_attitude_setpoint_s att_sp{};

		if (_vehicle_attitude_setpoint_sub.copy(&att_sp)) {
			const Eulerf e_sp(Quatf(att_sp.q_d));

			// ifo_pos_control always publishes a level attitude setpoint (roll/pitch = 0);
			// translation is realized by the thrust setpoint via the control allocator.
			if (PX4_ISFINITE(e_sp.phi()))   { roll_sp  = e_sp.phi(); }
			if (PX4_ISFINITE(e_sp.theta())) { pitch_sp = e_sp.theta(); }

			if (PX4_ISFINITE(e_sp.psi())) {
				_yaw_setpoint = e_sp.psi();
			}

			if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
				yaw_rate_ff = att_sp.yaw_sp_move_rate;
			}

			thrust_body(0) = att_sp.thrust_body[0];
			thrust_body(1) = att_sp.thrust_body[1];
			thrust_body(2) = att_sp.thrust_body[2];
		}

		if (!PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_setpoint = yaw;
		}
	}

	// Final feasibility guard at the allocator boundary. The fixed coaxial pair
	// cannot produce body +Z; never forward that component even if another
	// attitude-setpoint publisher bypasses ifo_pos_control.
	thrust_body(2) = math::min(thrust_body(2), 0.f);

	// ── Attitude error → rate setpoint (P controller) ─────────────────
	// The IFODRONE body is kept level: roll/pitch setpoint is 0 in all modes.
	const float roll_error  = roll_sp  - roll;
	const float pitch_error = pitch_sp - pitch;
	const float yaw_error   = wrap_pi(_yaw_setpoint - yaw);

	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.roll  = math::constrain(_param_mc_roll_p.get()  * roll_error,  -RATE_LIMIT_RP,  RATE_LIMIT_RP);
	rates_sp.pitch = math::constrain(_param_mc_pitch_p.get() * pitch_error, -RATE_LIMIT_RP,  RATE_LIMIT_RP);
	rates_sp.yaw   = math::constrain(_param_mc_yaw_p.get()   * yaw_error + yaw_rate_ff,
					 -RATE_LIMIT_YAW, RATE_LIMIT_YAW);
	thrust_body.copyTo(rates_sp.thrust_body);
	rates_sp.timestamp = hrt_absolute_time();
	_vehicle_rates_setpoint_pub.publish(rates_sp);

	perf_end(_loop_perf);
}

int IfodroneAttitudeControl::task_spawn(int argc, char *argv[])
{
	IfodroneAttitudeControl *instance = new IfodroneAttitudeControl();

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

int IfodroneAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IFODRONE outer-loop attitude controller.

P controller: roll/pitch/yaw attitude error → body rate setpoint.
Publishes vehicle_rates_setpoint (with thrust_body) for mc_rate_control,
which runs the inner-loop rate PID at gyro rate.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int IfodroneAttitudeControl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

extern "C" __EXPORT int ifo_att_control_main(int argc, char *argv[])
{
	return IfodroneAttitudeControl::main(argc, argv);
}
