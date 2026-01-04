
#include "IfodroneAttitudeControl.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

IfodroneAttitudeControl::IfodroneAttitudeControl() :
	ModuleParams(nullptr),
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_loop_interval_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": loop interval");
	_control_updated_perf = perf_alloc(PC_COUNT, MODULE_NAME": control updated");
}

IfodroneAttitudeControl::~IfodroneAttitudeControl()
{
	perf_free(_loop_interval_perf);
	perf_free(_control_updated_perf);
	ScheduleClear();
}

bool IfodroneAttitudeControl::init()
{
	PX4_INFO("IFO module initialized!");

	ScheduleOnInterval(20_ms); // 50 Hz
	return true;
}

void IfodroneAttitudeControl::Run()
{
	perf_count(_loop_interval_perf);

	parameter_update_s params;

	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	if (_parameter_update_sub.updated()) {

		_parameter_update_sub.update(&params);
		updateParams();
	}

	if (!_vehicle_attitude_sub.updated()) {
		return;
	}

	// Hover thrust estimate can be used for feedforward control
	hover_thrust_estimate_s hover{};
	float thrust_hover = 200.0f;

	if (_hover_thrust_estimate_sub.update(&hover)) {
		if (PX4_ISFINITE(hover.hover_thrust)) {
			thrust_hover = hover.hover_thrust;
		}
	}

	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = hrt_absolute_time();
	thrust.xyz[0] = 0.f;
	thrust.xyz[1] = 0.f;
	thrust.xyz[2] = -thrust_hover;
	_thrust_pub.publish(thrust);

	// Vehicle control mode logic
	vehicle_control_mode_s vcm{};

	vcm.flag_control_position_enabled = true;
	vcm.flag_control_velocity_enabled = true;
	vcm.flag_control_attitude_enabled = true;
	vcm.flag_control_rates_enabled = true;
	vcm.flag_control_allocation_enabled = true;

	vcm.timestamp = hrt_absolute_time();
	_vehicle_control_mode_pub.publish(vcm);

	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);

	vehicle_attitude_setpoint_s att_sp{};
	bool has_sp = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	Quatf q_sp(att_sp.q_d);
	Quatf q(att.q);

	Vector3f euler_sp = Eulerf(q_sp);
	Vector3f euler = Eulerf(q);

	float yaw_sp = has_sp ? euler_sp(2) : euler(2); // hold yaw

	Quatf q_des = Quatf(Eulerf(0.f, 0.f, yaw_sp));
	Quatf q_err = q.inversed() * q_des;

	Vector3f torque = 2.f * Vector3f(q_err.imag()) * _kp_att;

	// publish torque setpoint
	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		torque_sp.xyz[i] = torque(i);
	}
	_torque_pub.publish(torque_sp);

	perf_count(_control_updated_perf);
}

int IfodroneAttitudeControl::task_spawn(int argc, char *argv[])
{
	IfodroneAttitudeControl *instance = new IfodroneAttitudeControl();

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

void IfodroneAttitudeControl::_parameters_updated()
{
	ModuleParams::updateParams();
}

int IfodroneAttitudeControl::print_status()
{
	PX4_INFO("IFO module running");
	return 0;
}

extern "C" __EXPORT int ifo_att_control_main(int argc, char *argv[])
{
	return IfodroneAttitudeControl::main(argc, argv);
}
