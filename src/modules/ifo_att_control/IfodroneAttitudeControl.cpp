
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

	if (run_attitude_control) {

		// Current attitude as Euler angles
		const Quatf q_current(att.q);
		const Eulerf euler_current(q_current);

		const float roll_current = euler_current.phi();    // Current roll
		const float pitch_current = euler_current.theta(); // Current pitch
		const float yaw_current = euler_current.psi();     // Current yaw

		// SETPOINT: Always level (roll=0, pitch=0).
		// In STABILIZED mode this gives auto-level; pilot stick roll/pitch is intentionally ignored.
		const float roll_setpoint = 0.0f;
		const float pitch_setpoint = 0.0f;

		// Yaw setpoint:
		//  - if a vehicle_attitude_setpoint was received, use its q_d yaw (e.g. heading from
		//    ifo_pos_control or flight_mode_manager);
		//  - otherwise hold current yaw so the drone does not spin freely in stabilize mode.
		float yaw_setpoint = yaw_current;
		float yaw_rate_setpoint = 0.f;

		if (has_setpoint) {
			const Quatf q_desired(att_sp.q_d);
			const Eulerf euler_desired(q_desired);

			if (PX4_ISFINITE(euler_desired.psi())) {
				yaw_setpoint = euler_desired.psi();
			}

			if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
				yaw_rate_setpoint = att_sp.yaw_sp_move_rate;
			}
		}

		// Euler angles (orientation) errors
		const float roll_error = roll_setpoint - roll_current;
		const float pitch_error = pitch_setpoint - pitch_current;
		// Yaw error with wrap-around handling
		const float yaw_error = matrix::wrap_pi(yaw_setpoint - yaw_current);

		vehicle_angular_velocity_s rates{};
		_vehicle_angular_velocity_sub.copy(&rates);

		// vehicle_torque_setpoint is normalized, so keep all body torque demands in [-1, 1].
		torque(0) = math::constrain(_kp_att * roll_error - _kd_att * rates.xyz[0],
					    -_att_torque_limit, _att_torque_limit);
		torque(1) = math::constrain(_kp_att * pitch_error - _kd_att * rates.xyz[1],
					    -_att_torque_limit, _att_torque_limit);
		torque(2) = math::constrain(-_kp_yaw * yaw_error
					    - _kd_yaw * rates.xyz[2]
					    - _kff_yaw * yaw_rate_setpoint,
					    -_yaw_torque_limit, _yaw_torque_limit);

		// P controller for attitude stabilization
		// Torque = Kp * error
		// torque(0) = _kp_att * roll_error;   // Roll torque
		// torque(1) = _kp_att * pitch_error;  // Pitch torque
		// torque(2) = 0.0f;    // Yaw torque

		// Pass through thrust from position controller unchanged
		// X, Y: side motors (horizontal position control)
		// Z: main motors (altitude control)
		// thrust(0) = att_sp.thrust_body[0];  // Forward/back
		// thrust(1) = att_sp.thrust_body[1];  // Left/right
		// thrust(2) = att_sp.thrust_body[2];  // Up/down (negative = up)

		// static hrt_abstime last_debug{0};

		// if (hrt_elapsed_time(&last_debug) > 250_ms) {
		// 	PX4_INFO("IFO_ATT att=(%.1f %.1f %.1f) err=(%.1f %.1f %.1f) "
		// 		 "thr=(%.2f %.2f %.2f) tq=(%.2f %.2f %.2f)",
		// 		 (double)math::degrees(roll_current),
		// 		 (double)math::degrees(pitch_current),
		// 		 (double)math::degrees(yaw_current),
		// 		 (double)math::degrees(roll_error),
		// 		 (double)math::degrees(pitch_error),
		// 		 (double)math::degrees(yaw_error),
		// 		 (double)thrust(0), (double)thrust(1), (double)thrust(2),
		// 		 (double)torque(0), (double)torque(1), (double)torque(2));
		// 	last_debug = now;
		// }

	} else if (!control_mode.flag_armed) {
		// Not armed: zero everything
		torque.setZero();
		thrust.setZero();
	}
	// NOTE: When armed but no setpoint, keep previous values

	// ================================================================
	// COMPUTE TILT SERVO ANGLES
	//
	// Physics (verified via Rodrigues rotation for each motor axis):
	//   Motor 2 Front  (axis +X): positive tilt → axis gains -Z (up) → pushes UP at front → +pitch torque
	//   Motor 4 Back   (axis -X): positive tilt → axis gains -Z (up) → pushes UP at back  → -pitch torque
	//   Motor 3 Right  (axis -Y): positive tilt → axis gains -Z (up) → pushes UP at left  → +roll torque
	//   Motor 5 Left   (axis +Y): positive tilt → axis gains -Z (up) → pushes UP at right → -roll torque
	//
	// Therefore front/back are antiphase, right/left are antiphase.
	// Signed values in [-1, 1] allow the allocator to see which side tilts which way.
	// ================================================================
	const float pitch_tilt = torque(1);
	const float roll_tilt  = torque(0);

	// Publish servo angles first so the allocator can rebuild the matrix from the current tilt state.
	actuator_servos_s theta_T{};
	theta_T.timestamp = now;
	theta_T.timestamp_sample = att.timestamp;
	// Sign verified from flight logs (log_3_2026-3-28):
	// SDF joint [0,+1,0] on Front motor rotates axis toward -Z_FLU (down) for positive angle.
	// So positive servo command = nose-DOWN moment — opposite of what Rodrigues assumes in FRD.
	// Negate pitch_tilt and roll_tilt to get the corrective (stabilising) direction.
	theta_T.control[0] = pitch_tilt; // Front:  positive pitch_tilt → negative tilt → nose UP
	theta_T.control[1] = roll_tilt;  // Right:  positive roll_tilt  → negative tilt → right side UP
	theta_T.control[2] =  pitch_tilt; // Back:   positive pitch_tilt → positive tilt → nose UP (antisymmetric)
	theta_T.control[3] =  roll_tilt;  // Left:   positive roll_tilt  → positive tilt → right side UP (antisymmetric)
	_theta_pub.publish(theta_T);

	// NOTE: vehicle_thrust_setpoint is published by ifo_pos_control directly.
	// ifo_att_control runs at IMU rate and would overwrite the position controller's
	// carefully computed body-frame thrust — so thrust publication lives upstream.

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
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_control_updated_perf);
	return 0;
}

extern "C" __EXPORT int ifo_att_control_main(int argc, char *argv[])
{
	return IfodroneAttitudeControl::main(argc, argv);
}
