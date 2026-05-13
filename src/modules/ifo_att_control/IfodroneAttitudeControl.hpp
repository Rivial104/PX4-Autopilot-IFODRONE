/**
 * IFODRONE Attitude Controller (outer loop).
 *
 * Quaternion P-controller that outputs body rate setpoints.
 * Modeled after mc_att_control but simplified for IFODRONE:
 *   - Roll/pitch setpoints are always 0 (level body).
 *   - Yaw setpoint comes from attitude_setpoint (auto) or stick (manual).
 *   - Thrust is passed through from attitude_setpoint or generated from sticks.
 *   - No VTOL, no autotune, no tilt mapping from sticks.
 *
 * The inner rate loop is handled by mc_rate_control (standard PX4 module).
 */

#pragma once

#include <mc_att_control/AttitudeControl/AttitudeControl.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class IfodroneAttitudeControl :
	public ModuleBase<IfodroneAttitudeControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	IfodroneAttitudeControl();
	~IfodroneAttitudeControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_updated();

	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	AttitudeControl _attitude_control;

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription                 _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription                 _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_rates_setpoint_s>    _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	vehicle_control_mode_s    _vehicle_control_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};

	matrix::Vector3f _thrust_setpoint_body{};
	hrt_abstime      _last_run{0};
	hrt_abstime      _last_attitude_setpoint{0};
	bool             _landed{true};
	bool             _spooled_up{false};
	float            _yaw_setpoint{NAN};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)   _param_mpc_manthr_min,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	)
};
