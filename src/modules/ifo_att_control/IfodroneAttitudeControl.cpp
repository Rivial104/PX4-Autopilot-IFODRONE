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
	parameters_updated();
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

void IfodroneAttitudeControl::parameters_updated()
{
	_attitude_control.setProportionalGain(
		Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
		_param_mc_yaw_weight.get());

	using math::radians;
	_attitude_control.setRateLimit(
		Vector3f(radians(_param_mc_rollrate_max.get()),
			 radians(_param_mc_pitchrate_max.get()),
			 radians(_param_mc_yawrate_max.get())));
}

void IfodroneAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// IFODRONE: roll=0, pitch=0 always
	// Yaw: stick controls yaw rate, integrate heading
	const float yaw = Eulerf(q).psi();

	if (!PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint = yaw;
	}

	// Yaw stick -> yaw rate -> integrate heading
	static constexpr float YAW_RATE_MAX = 1.5f; // rad/s
	const float yaw_rate = _manual_control_setpoint.yaw * YAW_RATE_MAX;
	_yaw_setpoint = wrap_pi(_yaw_setpoint + yaw_rate * dt);

	attitude_setpoint.yaw_sp_move_rate = yaw_rate;

	// Build quaternion: roll=0, pitch=0, yaw=_yaw_setpoint
	const Quatf q_sp(Eulerf(0.f, 0.f, _yaw_setpoint));
	q_sp.copyTo(attitude_setpoint.q_d);

	// Thrust from sticks:
	//  Throttle stick [-1,1] -> [IDLE,1] -> body Z (negative = up)
	//  IDLE ensures both coaxial motors always have a thrust budget at arming
	//  so the yaw channel never clips one motor to 0.
	//  Roll/pitch sticks -> body X/Y (IFODRONE body-frame force)
	static constexpr float THROTTLE_IDLE = 0.1f;
	const float throttle_raw = (_manual_control_setpoint.throttle + 1.f) * 0.5f;
	const float throttle = THROTTLE_IDLE + throttle_raw * (1.f - THROTTLE_IDLE);
	attitude_setpoint.thrust_body[0] = _manual_control_setpoint.roll;
	attitude_setpoint.thrust_body[1] = _manual_control_setpoint.pitch;
	attitude_setpoint.thrust_body[2] = -throttle;

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void IfodroneAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	// Run controller on attitude updates
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		// Update subscriptions
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled;

		if (run_att_ctrl) {

			// Manual/Stabilize: generate attitude setpoint from sticks
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt);

			} else {
				// Auto mode: reset manual yaw tracking
				_yaw_setpoint = NAN;
			}

			// Read the latest attitude setpoint (from generate_attitude_setpoint or ifo_pos_control)
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(
						Quatf(vehicle_attitude_setpoint.q_d),
						vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Run quaternion P-controller -> rate setpoints
			Vector3f rates_sp = _attitude_control.update(q);

			// Publish rate setpoint for mc_rate_control
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll  = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw   = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			// Attitude control disabled - reset yaw
			_yaw_setpoint = NAN;
		}
	}

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
IFODRONE attitude controller (outer loop).

Quaternion P-controller producing body rate setpoints.
Roll/pitch setpoints are always zero (level body).
The inner rate PID loop is handled by mc_rate_control.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ifo_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ifo_att_control_main(int argc, char *argv[])
{
	return IfodroneAttitudeControl::main(argc, argv);
}
