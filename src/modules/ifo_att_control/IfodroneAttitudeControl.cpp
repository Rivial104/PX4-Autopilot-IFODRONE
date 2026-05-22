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

void IfodroneAttitudeControl::parameters_updated()
{
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
		parameters_updated();
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

	// ── Angular rates ─────────────────────────────────────────────────
	vehicle_angular_velocity_s rates{};
	_vehicle_angular_velocity_sub.copy(&rates);

	const float roll_rate  = rates.xyz[0];
	const float pitch_rate = rates.xyz[1];
	const float yaw_rate   = rates.xyz[2];

	// ── Yaw setpoint ──────────────────────────────────────────────────
	// Manual/Stabilize: yaw stick integrates heading.
	// Auto/Position: yaw comes from vehicle_attitude_setpoint.
	const bool manual_mode = _vehicle_control_mode.flag_control_manual_enabled &&
				 !_vehicle_control_mode.flag_control_altitude_enabled &&
				 !_vehicle_control_mode.flag_control_velocity_enabled &&
				 !_vehicle_control_mode.flag_control_position_enabled;

	float yaw_rate_sp = 0.f;

	if (manual_mode) {
		if (!PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_setpoint = yaw;
		}

		static constexpr float YAW_RATE_MAX = 1.5f;
		yaw_rate_sp = _manual_control_setpoint.yaw * YAW_RATE_MAX;
		_yaw_setpoint = wrap_pi(_yaw_setpoint + yaw_rate_sp * dt);

	} else {
		_yaw_setpoint = NAN;

		vehicle_attitude_setpoint_s att_sp{};

		if (_vehicle_attitude_setpoint_sub.copy(&att_sp)) {
			const Eulerf e_sp(Quatf(att_sp.q_d));

			if (PX4_ISFINITE(e_sp.psi())) {
				_yaw_setpoint = e_sp.psi();
			}

			if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
				yaw_rate_sp = att_sp.yaw_sp_move_rate;
			}
		}

		if (!PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_setpoint = yaw;
		}
	}

	// ── Attitude errors ───────────────────────────────────────────────
	const float roll_error  = -roll;   // setpoint always 0 (level)
	const float pitch_error = -pitch;
	const float yaw_error   = wrap_pi(_yaw_setpoint - yaw);

	// ── Roll/pitch → tilt servo angles ───────────────────────────────
	// PD controller: error [rad] → normalized servo command [-1,1]
	const float pitch_tilt = math::constrain(KP_ATT * pitch_error - KD_ATT * pitch_rate, -TILT_LIMIT, TILT_LIMIT);
	const float roll_tilt  = math::constrain(KP_ATT * roll_error  - KD_ATT * roll_rate,  -TILT_LIMIT, TILT_LIMIT);

	// Tilt sign convention derived from SDF joint axes:
	//   Front (motor_2_joint axis [0,+Y,0]) and Back (motor_4_joint axis [0,-Y,0]) have opposite axes.
	//   Right (motor_3_joint axis [+X,0,0]) and Left (motor_5_joint axis [-X,0,0]) have opposite axes.
	//   Opposite axis → same command = physically opposite rotation = opposing torques.
	//   Negate back and left to make all four servos cooperate in the same torque direction.
	actuator_servos_s servos{};
	servos.timestamp        = hrt_absolute_time();
	servos.timestamp_sample = att.timestamp;
	servos.control[0] =  pitch_tilt;   // Front: axis +Y
	servos.control[1] =  roll_tilt;    // Right: axis +X
	servos.control[2] = -pitch_tilt;   // Back:  axis -Y → negate for same pitch torque direction
	servos.control[3] = -roll_tilt;    // Left:  axis -X → negate for same roll torque direction
	_actuator_servos_pub.publish(servos);

	// ── Yaw torque → main motor differential (via CA) ─────────────────
	const float yaw_torque = math::constrain(
					 KP_YAW * yaw_error - KD_YAW * yaw_rate - KFF_YAW * yaw_rate_sp,
					 -YAW_TORQUE_LIMIT, YAW_TORQUE_LIMIT);

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp        = hrt_absolute_time();
	torque_sp.timestamp_sample = att.timestamp;
	torque_sp.xyz[0] = 0.f;          // roll torque: handled by tilt servos
	torque_sp.xyz[1] = 0.f;          // pitch torque: handled by tilt servos
	torque_sp.xyz[2] = yaw_torque;
	_vehicle_torque_setpoint_pub.publish(torque_sp);

	// ── Thrust (manual mode only) ──────────────────────────────────────
	// In position/auto modes, ifo_pos_control publishes vehicle_thrust_setpoint.
	// XY = 0: CA commands all 4 side EDFs symmetrically at PWM_MIN idle.
	// A non-zero XY stick would activate only the EDFs pointing in that direction,
	// leaving the opposing pair at zero — asymmetric and bad for stabilization.
	if (manual_mode) {
		const float throttle_raw = (_manual_control_setpoint.throttle + 1.f) * 0.5f;
		const float throttle     = THROTTLE_IDLE + throttle_raw * (1.f - THROTTLE_IDLE);

		vehicle_thrust_setpoint_s thrust_sp{};
		thrust_sp.timestamp        = hrt_absolute_time();
		thrust_sp.timestamp_sample = att.timestamp;
		thrust_sp.xyz[0] = 0.f;
		thrust_sp.xyz[1] = 0.f;
		thrust_sp.xyz[2] = -throttle;
		_vehicle_thrust_setpoint_pub.publish(thrust_sp);
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
IFODRONE attitude controller.

Direct PD controller: roll/pitch errors drive tilt servos directly.
Yaw error drives main motor differential via control allocator.
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
