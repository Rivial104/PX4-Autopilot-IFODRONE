
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
	PX4_INFO("IFO attitude control module initialized!");
	ScheduleOnInterval(20_ms); // 50 Hz
	return true;
}

void IfodroneAttitudeControl::Run()
{
	perf_count(_loop_interval_perf);

	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Update parameters if changed
	parameter_update_s params;
	if (_parameter_update_sub.updated()) {
		_parameter_update_sub.update(&params);
		updateParams();
	}

	// Wait for new attitude data
	if (!_vehicle_attitude_sub.updated()) {
		return;
	}

	// Get current attitude
	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);

	// Get attitude setpoint
	vehicle_attitude_setpoint_s att_sp{};
	const bool has_setpoint = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	// Get hover thrust estimate for vertical thrust
	hover_thrust_estimate_s hover{};
	float thrust_hover = 0.5f; // default hover thrust (normalized)

	if (_hover_thrust_estimate_sub.update(&hover)) {
		if (PX4_ISFINITE(hover.hover_thrust)) {
			thrust_hover = hover.hover_thrust;
		}
	}

	// ========================================
	// THRUST SETPOINT
	// ========================================
	// For IfoDrone: main motors handle Z thrust, side motors handle X/Y via tilts
	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = hrt_absolute_time();
	thrust.xyz[0] = 0.0f;  // No direct X thrust (handled by tilts)
	thrust.xyz[1] = 0.0f;  // No direct Y thrust (handled by tilts)
	thrust.xyz[2] = -thrust_hover;  // Negative in NED = upward thrust
	_thrust_pub.publish(thrust);

	// ========================================
	// ATTITUDE CONTROL
	// ========================================
	Quatf q(att.q);              // Current attitude
	Quatf q_sp(att_sp.q_d);      // Desired attitude
	Vector3f euler = Eulerf(q);
	Vector3f euler_sp = Eulerf(q_sp);

	// Hold yaw from setpoint, or current yaw if no setpoint
	float yaw_sp = has_setpoint ? euler_sp(2) : euler(2);

	// Target: level attitude (roll=0, pitch=0) with desired yaw
	Quatf q_des = Quatf(Eulerf(0.f, 0.f, yaw_sp));

	// Quaternion error (body frame)
	Quatf q_err = q.inversed() * q_des;

	// Proportional attitude control
	// The imaginary part of quaternion error gives rotation axis scaled by sin(angle/2)
	Vector3f torque = 2.f * Vector3f(q_err.imag()) * _kp_att;

	// ========================================
	// TORQUE SETPOINT
	// ========================================
	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = hrt_absolute_time();
	torque_sp.xyz[0] = torque(0);  // Roll torque -> side motor tilts (right/left)
	torque_sp.xyz[1] = torque(1);  // Pitch torque -> side motor tilts (front/back)
	torque_sp.xyz[2] = torque(2);  // Yaw torque -> main motor differential
	_torque_pub.publish(torque_sp);

	// ========================================
	// VEHICLE CONTROL MODE
	// ========================================
	vehicle_control_mode_s vcm{};
	vcm.timestamp = hrt_absolute_time();
	vcm.flag_control_position_enabled = true;
	vcm.flag_control_velocity_enabled = true;
	vcm.flag_control_attitude_enabled = true;
	vcm.flag_control_rates_enabled = true;
	vcm.flag_control_allocation_enabled = true;
	vcm.flag_control_climb_rate_enabled = true;
	vcm.flag_control_altitude_enabled = true;
	vcm.flag_control_manual_enabled = false;
	_vehicle_control_mode_pub.publish(vcm);

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
