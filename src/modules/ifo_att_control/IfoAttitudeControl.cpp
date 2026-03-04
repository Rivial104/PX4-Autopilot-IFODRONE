/**
 * @file IfoAttitudeControl.cpp
 *
 * IFODRONE Attitude Control – passthrough with forced-level setpoint.
 *
 * Pipeline:
 *   vehicle_attitude_setpoint  (from IfoPositionControl / mc_pos_control)
 *       ↓  force roll=0, pitch=0, keep yaw & thrust
 *   AttitudeControl::update()  (standard quaternion P-controller → rates)
 *       ↓
 *   vehicle_rates_setpoint     (consumed by mc_rate_control)
 */

#include "IfoAttitudeControl.hpp"

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

IfoAttitudeControl::IfoAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_thrust_setpoint_body.zero();
	parameters_updated();
}

bool IfoAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("IFO attitude control (passthrough) initialized");
	return true;
}

void IfoAttitudeControl::parameters_updated()
{
	using math::radians;

	_attitude_control.setProportionalGain(
		Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
		_param_mc_yaw_weight.get());

	_attitude_control.setRateLimit(
		Vector3f(radians(_param_mc_rollrate_max.get()),
			 radians(_param_mc_pitchrate_max.get()),
			 radians(_param_mc_yawrate_max.get())));
}

void IfoAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// ── parameter hot-reload ──────────────────────────────────────────
	if (_parameter_update_sub.updated()) {
		parameter_update_s p;
		_parameter_update_sub.copy(&p);
		updateParams();
		parameters_updated();
	}

	// ── guard: IFO_ATT_MODE must be > 0 ──────────────────────────────
	if (_param_ifo_att_mode.get() <= 0) {
		return;   // disabled – let the standard mc_att_control run
	}

	// ── current attitude ──────────────────────────────────────────────
	vehicle_attitude_s att;

	if (!_vehicle_attitude_sub.update(&att)) {
		return;
	}

	const float dt = math::constrain(((att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	_last_run = att.timestamp_sample;
	(void)dt;   // not needed by the P-controller, kept for future D-term

	const Quatf q{att.q};

	// ── attitude setpoint ─────────────────────────────────────────────
	if (_vehicle_attitude_setpoint_sub.updated()) {
		vehicle_attitude_setpoint_s sp;

		if (_vehicle_attitude_setpoint_sub.copy(&sp) && sp.timestamp > _last_attitude_setpoint) {

			// ====================================================
			// KEY IFODRONE MODIFICATION:
			// Force roll = 0, pitch = 0.  Only yaw is taken from
			// the upstream setpoint.  The drone is always level;
			// horizontal movement comes from the tilt servos, not
			// from banking the airframe.
			//
			// Yaw is extracted directly from the desired quaternion
			// using atan2 on the Z-rotation components (indices 0,3)
			// to avoid Euler psi() discontinuity at ±180°.
			// ====================================================
			const Quatf q_desired(sp.q_d);

			// Build a pure-yaw quaternion from q_desired:
			// For a ZYX Euler quaternion with roll=pitch=0,
			// q = (cos(yaw/2), 0, 0, sin(yaw/2)).
			// We can extract the yaw half-angle directly from
			// the scalar (w) and z components of the desired quaternion,
			// re-normalizing the 2D vector (w, z) to strip roll/pitch.
			float w_yaw = q_desired(0);
			float z_yaw = q_desired(3);
			const float wz_norm = sqrtf(w_yaw * w_yaw + z_yaw * z_yaw);

			if (wz_norm > FLT_EPSILON) {
				w_yaw /= wz_norm;
				z_yaw /= wz_norm;

			} else {
				// Fallback: no valid yaw info → keep current heading
				const float cur_yaw = Eulerf(q).psi();
				w_yaw = cosf(cur_yaw * 0.5f);
				z_yaw = sinf(cur_yaw * 0.5f);
			}

			const Quatf q_level_yaw(w_yaw, 0.f, 0.f, z_yaw);

			const float yawspeed_sp = PX4_ISFINITE(sp.yaw_sp_move_rate)
						  ? sp.yaw_sp_move_rate : 0.f;

			_attitude_control.setAttitudeSetpoint(q_level_yaw, yawspeed_sp);

			// Thrust passes through unchanged
			_thrust_setpoint_body = Vector3f(sp.thrust_body);

			_last_attitude_setpoint = sp.timestamp;
		}
	}

	// ── heading reset handling ────────────────────────────────────────
	if (_quat_reset_counter != att.quat_reset_counter) {
		const Quatf delta_q_reset(att.delta_q_reset);

		if (att.timestamp > _last_attitude_setpoint) {
			_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
		}

		_quat_reset_counter = att.quat_reset_counter;
	}

	// ── run attitude → rate controller ───────────────────────────────
const Vector3f rates_sp = _attitude_control.update(q);

	// ── publish vehicle_rates_setpoint ────────────────────────────────
	vehicle_rates_setpoint_s rates_setpoint{};
	rates_setpoint.roll  = rates_sp(0);
	rates_setpoint.pitch = rates_sp(1);
	rates_setpoint.yaw   = rates_sp(2);
	_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
	rates_setpoint.timestamp = hrt_absolute_time();

	_vehicle_rates_setpoint_pub.publish(rates_setpoint);
}

// ── ModuleBase boilerplate ───────────────────────────────────────────────

int IfoAttitudeControl::task_spawn(int argc, char *argv[])
{
	auto *instance = new IfoAttitudeControl();

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

int IfoAttitudeControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_USAGE_NAME("ifo_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ifo_att_control_main(int argc, char *argv[])
{
	return IfoAttitudeControl::main(argc, argv);
}
