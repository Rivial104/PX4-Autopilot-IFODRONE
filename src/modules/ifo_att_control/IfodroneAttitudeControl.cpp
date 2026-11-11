#include "IfodroneAttitudeControl.hpp"

#include <float.h>
#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

IfodroneAttitudeControl::IfodroneAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_update();
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

void IfodroneAttitudeControl::parameters_update()
{
	updateParams();
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(),
					      _param_mc_yaw_p.get()), 1.f);
	_attitude_control.setRateLimit(Vector3f(math::radians(_param_mc_rollrate_max.get()),
						math::radians(_param_mc_pitchrate_max.get()),
						math::radians(_param_mc_yawrate_max.get())));
}

void IfodroneAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	update();
	perf_end(_loop_perf);
}

void IfodroneAttitudeControl::update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		parameters_update();
	}

	vehicle_attitude_s att;

	if (!_vehicle_attitude_sub.update(&att)) {
		return;
	}

	const float dt = math::constrain(((att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	_last_run = att.timestamp_sample;
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	if (!_vehicle_control_mode.flag_control_attitude_enabled) {
		_yaw_setpoint = NAN;
		_thrust_z = 0.f;
		return;
	}

	const Quatf q(att.q);
	const hrt_abstime now = hrt_absolute_time();

	vehicle_attitude_setpoint_s att_sp{};
	const bool att_sp_valid = _vehicle_attitude_setpoint_sub.copy(&att_sp) && att_sp.timestamp != 0;
	const bool att_sp_fresh = att_sp_valid && now < att_sp.timestamp + ATT_SP_TIMEOUT;

	Setpoint sp;

	if (_vehicle_control_mode.flag_control_manual_enabled && !att_sp_fresh) {
		sp = manualSetpoint(Eulerf(q).psi(), dt, now);

	} else {
		_yaw_setpoint = NAN;
		sp = attitudeSetpoint(att_sp, att_sp_valid, att_sp_fresh, Eulerf(q).psi());
	}

	for (int i = 0; i < 3; ++i) {
		if (!PX4_ISFINITE(sp.thrust(i))) {
			sp.thrust(i) = 0.f;
		}
	}

	// The fixed coaxial pair cannot produce body +Z thrust.
	sp.thrust(2) = math::min(sp.thrust(2), 0.f);
	_thrust_z = _vehicle_control_mode.flag_armed ? sp.thrust(2) : 0.f;

	_attitude_control.setAttitudeSetpoint(sp.q, sp.yaw_rate_ff);
	const Vector3f rates = _attitude_control.update(q);

	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.roll = rates(0);
	rates_sp.pitch = rates(1);
	rates_sp.yaw = rates(2);
	sp.thrust.copyTo(rates_sp.thrust_body);
	rates_sp.timestamp = now;
	_vehicle_rates_setpoint_pub.publish(rates_sp);
}

IfodroneAttitudeControl::Setpoint IfodroneAttitudeControl::attitudeSetpoint(const vehicle_attitude_setpoint_s &att_sp,
		bool valid, bool fresh, float yaw) const
{
	Setpoint sp{Quatf(Eulerf(0.f, 0.f, yaw)), Vector3f(), 0.f};

	if (!valid) {
		return sp;
	}

	const Quatf requested(att_sp.q_d);

	if (requested.isAllFinite() && requested.norm_squared() > FLT_EPSILON) {
		sp.q = requested;
	}

	sp.thrust = Vector3f(att_sp.thrust_body);

	if (fresh) {
		sp.yaw_rate_ff = PX4_ISFINITE(att_sp.yaw_sp_move_rate) ? att_sp.yaw_sp_move_rate : 0.f;

	} else {
		// Without feedback, stop lateral acceleration and yaw feedforward.
		sp.thrust.xy() = Vector2f();
	}

	return sp;
}

IfodroneAttitudeControl::Setpoint IfodroneAttitudeControl::manualSetpoint(float yaw, float dt, hrt_abstime now)
{
	// Hold the last collective until commander handles a lost manual input.
	Setpoint sp{Quatf(), Vector3f(0.f, 0.f, _thrust_z), 0.f};

	manual_control_setpoint_s manual{};
	const bool manual_valid = _manual_control_setpoint_sub.copy(&manual) && manual.valid
				  && now < manual.timestamp + 500_ms
				  && PX4_ISFINITE(manual.yaw) && PX4_ISFINITE(manual.throttle);

	if (!manual_valid || !PX4_ISFINITE(_yaw_setpoint) || !_vehicle_control_mode.flag_armed) {
		_yaw_setpoint = yaw;
	}

	if (manual_valid) {
		sp.yaw_rate_ff = math::constrain(manual.yaw, -1.f, 1.f) * YAW_RATE_MAX;
		_yaw_setpoint = wrap_pi(_yaw_setpoint + sp.yaw_rate_ff * dt);
		const float throttle = math::constrain((manual.throttle + 1.f) * 0.5f, 0.f, 1.f);
		sp.thrust(2) = -(THROTTLE_IDLE + throttle * (1.f - THROTTLE_IDLE));
	}

	sp.q = Quatf(Eulerf(0.f, 0.f, _yaw_setpoint));
	return sp;
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

Quaternion attitude controller producing body rate setpoints.
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
