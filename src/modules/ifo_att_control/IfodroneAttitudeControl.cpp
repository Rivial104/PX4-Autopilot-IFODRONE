
#include "IfodroneAttitudeControl.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

IfodroneAttitudeControl::IfodroneAttitudeControl() :
	ModuleParams(nullptr),
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
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
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("IFO attitude control initialized - active stabilization mode");
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

	// ================================================================
	// GET CURRENT STATE
	// ================================================================

	// Get current attitude
	vehicle_attitude_s att{};
	if (!_vehicle_attitude_sub.copy(&att)) {
		return;
	}

	// Get control mode
	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);

	// Get attitude setpoint (from position controller)
	vehicle_attitude_setpoint_s att_sp{};
	const bool has_setpoint = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	// Get land detected
	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	// ================================================================
	// ATTITUDE CONTROL
	// - Roll/Pitch torque: From attitude error (stabilization)
	// - Yaw torque: From attitude error (main motors differential)
	// - Thrust XYZ: Passed through from position controller
	// ================================================================

	const hrt_abstime now = hrt_absolute_time();
	Vector3f torque(0.0f, 0.0f, 0.0f);
	Vector3f thrust(0.0f, 0.0f, 0.0f);

	const bool run_attitude_control = control_mode.flag_armed &&
					  control_mode.flag_control_attitude_enabled;

	if (run_attitude_control && has_setpoint) {

		// Current attitude quaternion
		const Quatf q_current(att.q);

		// Desired attitude quaternion (from position controller)
		const Quatf q_desired(att_sp.q_d);

		// Quaternion error: q_error = q_desired * q_current^-1
		Quatf q_error = q_desired * q_current.inversed();

		// Ensure quaternion has positive scalar part (shortest path)
		if (q_error(0) < 0.0f) {
			q_error = -q_error;
		}

		// Proportional attitude control for torque
		// Roll and Pitch torque compensate attitude errors
		// These will be applied via tilt motors
		torque = 2.0f * Vector3f(q_error(1), q_error(2), q_error(3)) * _kp_att;

		// Get thrust from attitude setpoint (from position controller)
		// thrust_body[0]: X (forward) - tilt motors
		// thrust_body[1]: Y (right) - tilt motors
		// thrust_body[2]: Z (down, negative=up) - main motors
		thrust(0) = att_sp.thrust_body[0];
		thrust(1) = att_sp.thrust_body[1];
		thrust(2) = att_sp.thrust_body[2];

		// Debug output
		static int counter = 0;
		if (++counter >= 250) {  // ~1 Hz at 250 Hz
			counter = 0;
			const Eulerf euler_current(q_current);
			const Eulerf euler_desired(q_desired);
			PX4_INFO("ATT: r=%.1f p=%.1f y=%.1f | torque=(%.2f,%.2f,%.2f) | thrust=(%.2f,%.2f,%.2f)",
				 (double)math::degrees(euler_current.phi()),
				 (double)math::degrees(euler_current.theta()),
				 (double)math::degrees(euler_current.psi()),
				 (double)torque(0), (double)torque(1), (double)torque(2),
				 (double)thrust(0), (double)thrust(1), (double)thrust(2));
		}

	} else if (!control_mode.flag_armed) {
		// Not armed: zero everything
		torque.setZero();
		thrust.setZero();
	}
	// NOTE: When armed but no setpoint, keep previous values

	// ================================================================
	// PUBLISH THRUST SETPOINT
	// X, Y: from tilt motors (horizontal position control)
	// Z: from main motors (altitude control)
	// ================================================================
	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = now;
	thrust_sp.timestamp_sample = att.timestamp;
	thrust_sp.xyz[0] = thrust(0);  // X thrust (tilt motors - forward)
	thrust_sp.xyz[1] = thrust(1);  // Y thrust (tilt motors - right)
	thrust_sp.xyz[2] = thrust(2);  // Z thrust (main motors - up is negative)
	_thrust_pub.publish(thrust_sp);

	// ================================================================
	// PUBLISH TORQUE SETPOINT
	// Roll/Pitch: from tilt motors (attitude stabilization)
	// Yaw: from main motors differential
	// ================================================================
	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = now;
	torque_sp.timestamp_sample = att.timestamp;
	torque_sp.xyz[0] = torque(0);  // Roll torque (tilt motors)
	torque_sp.xyz[1] = torque(1);  // Pitch torque (tilt motors)
	torque_sp.xyz[2] = torque(2);  // Yaw torque (main motors differential)
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
