/**
 * IFODRONE Attitude Controller (outer loop)
 *
 * P-only attitude controller — converts angle error to rate setpoint.
 * Publishes vehicle_rates_setpoint for ifo_rate_control (inner loop).
 *
 *   - Roll/pitch error (body level setpoint) → rates_sp.roll/pitch
 *   - Yaw error                              → rates_sp.yaw
 *   - Manual/pos mode: ifo_att_control or ifo_pos_control own vehicle_thrust_setpoint
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
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
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

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
	int print_status() override;

private:
	void Run() override;
	void parameters_updated();

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription                 _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_rates_setpoint_s>  _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	vehicle_control_mode_s    _vehicle_control_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};

	hrt_abstime _last_run{0};
	float       _yaw_setpoint{NAN};

	static constexpr float THROTTLE_IDLE{0.05f};
	static constexpr float KFF_YAW{0.0f};
	static constexpr float RATE_LIMIT_RP{6.0f};   // max roll/pitch rate setpoint [rad/s]
	static constexpr float RATE_LIMIT_YAW{3.0f};  // max yaw rate setpoint [rad/s]

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_ATT_P>)    _param_ifo_att_p,
		(ParamFloat<px4::params::IFO_YAW_P>)    _param_ifo_yaw_p,
		(ParamFloat<px4::params::IFO_THR_HOVER>) _param_ifo_thr_hover
	)

	perf_counter_t _loop_perf;
};
