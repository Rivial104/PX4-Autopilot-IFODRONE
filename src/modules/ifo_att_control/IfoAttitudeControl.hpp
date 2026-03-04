/**
 * @file IfoAttitudeControl.hpp
 *
 * IFODRONE Attitude Control – passthrough with forced-level setpoint.
 *
 * Replaces mc_att_control for the IFODRONE.
 * The module reads vehicle_attitude_setpoint (from IfoPositionControl or
 * mc_pos_control), forces roll = 0 and pitch = 0 while keeping the desired
 * yaw and thrust, then runs the standard PX4 quaternion attitude controller
 * (AttitudeControl::update) to produce vehicle_rates_setpoint.
 * The downstream mc_rate_control converts rates → torque/thrust.
 *
 * Activated by the IFO_ATT_MODE parameter.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

#include <AttitudeControl.hpp>

using namespace time_literals;

class IfoAttitudeControl :
	public ModuleBase<IfoAttitudeControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	IfoAttitudeControl();
	~IfoAttitudeControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_updated();

	AttitudeControl _attitude_control;   ///< standard PX4 quaternion attitude controller

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};

	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	matrix::Vector3f _thrust_setpoint_body;   ///< body-frame thrust from attitude setpoint

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	uint8_t _quat_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IFO_ATT_MODE>) _param_ifo_att_mode,

		/* Use the same standard MC gains so QGC tuning works: */
		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max
	)
};
