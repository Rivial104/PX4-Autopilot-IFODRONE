
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
	// IFODRONE ATTITUDE CONTROL
	//
	// SIMPLE ARCHITECTURE:
	// - Position controller provides thrust XYZ (passed through unchanged)
	// - Attitude controller ONLY stabilizes to horizontal (roll=0, pitch=0)
	// - Yaw is taken from attitude setpoint (heading control)
	//
	// Thrust:
	//   X, Y → side motors (2-5) via tilts for horizontal movement
	//   Z → main motors (0, 1) for altitude
	//
	// Torque:
	//   Roll, Pitch → correction to keep drone level (setpoint = 0)
	//   Yaw → from attitude setpoint (heading)
	// ================================================================

	const hrt_abstime now = hrt_absolute_time();
	Vector3f torque(0.0f, 0.0f, 0.0f);
	Vector3f thrust(0.0f, 0.0f, 0.0f);

	const bool run_attitude_control = control_mode.flag_armed &&
					  control_mode.flag_control_attitude_enabled;

	if (run_attitude_control && has_setpoint) {

		// Current attitude as Euler angles
		const Quatf q_current(att.q);
		const Eulerf euler_current(q_current);

		const float roll_current = euler_current.phi();    // Current roll
		const float pitch_current = euler_current.theta(); // Current pitch
		const float yaw_current = euler_current.psi();     // Current yaw

		// SETPOINT: Always level (roll=0, pitch=0), yaw from position controller
		const float roll_setpoint = 0.0f;
		const float pitch_setpoint = 0.0f;

		// Get desired yaw from attitude setpoint quaternion
		const Quatf q_desired(att_sp.q_d);
		const Eulerf euler_desired(q_desired);
		const float yaw_setpoint = euler_desired.psi();

		// Attitude errors (setpoint - current)
		const float roll_error = roll_setpoint - roll_current;
		const float pitch_error = pitch_setpoint - pitch_current;

		// Yaw error with wrap-around handling
		float yaw_error = yaw_setpoint - yaw_current;
		if (yaw_error > M_PI_F) {
			yaw_error -= 2.0f * M_PI_F;
		} else if (yaw_error < -M_PI_F) {
			yaw_error += 2.0f * M_PI_F;
		}

		vehicle_angular_velocity_s rates{};
		_vehicle_angular_velocity_sub.copy(&rates);

		torque(0) = _kp_att * roll_error - _kd_att * rates.xyz[0];
		torque(1) = _kp_att * pitch_error - _kd_att * rates.xyz[1];

		// P controller for attitude stabilization
		// Torque = Kp * error
		// torque(0) = _kp_att * roll_error;   // Roll torque
		// torque(1) = _kp_att * pitch_error;  // Pitch torque
		// torque(2) = 0.0f;    // Yaw torque

		// Pass through thrust from position controller unchanged
		// X, Y: side motors (horizontal position control)
		// Z: main motors (altitude control)
		thrust(0) = att_sp.thrust_body[0];  // Forward/back
		thrust(1) = att_sp.thrust_body[1];  // Left/right
		thrust(2) = att_sp.thrust_body[2];  // Up/down (negative = up)

		// // Debug output
		// static int counter = 0;
		// if (++counter >= 250) {  // ~1 Hz at 250 Hz
		// 	counter = 0;
		// 	PX4_INFO("ATT: r=%.1f p=%.1f y=%.1f | err=(%.2f,%.2f,%.2f) | thrust=(%.2f,%.2f,%.2f)",
		// 		 (double)math::degrees(roll_current),
		// 		 (double)math::degrees(pitch_current),
		// 		 (double)math::degrees(yaw_current),
		// 		 (double)math::degrees(roll_error),
		// 		 (double)math::degrees(pitch_error),
		// 		 (double)math::degrees(yaw_error),
		// 		 (double)thrust(0), (double)thrust(1), (double)thrust(2));
		// }

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
