/**
 * IFODRONE Attitude Controller
 *
 * PD attitude controller — publishes full vehicle_torque_setpoint (roll+pitch+yaw).
 * The control allocator translates that to tilt servo angles (roll/pitch) and
 * main motor differential (yaw). mc_rate_control is NOT used.
 *
 *   - Roll/pitch error → torque_sp.xyz[0/1] → CA → tilt servos
 *   - Yaw error        → torque_sp.xyz[2]   → CA → coaxial motor differential
 *   - Manual mode: throttle stick → vehicle_thrust_setpoint
 *   - Position mode: ifo_pos_control publishes vehicle_thrust_setpoint
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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

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
	uORB::Subscription                 _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	vehicle_control_mode_s    _vehicle_control_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};

	hrt_abstime _last_run{0};
	float       _yaw_setpoint{NAN};

	static constexpr float TILT_LIMIT{100.0f};
	static constexpr float YAW_TORQUE_LIMIT{100.0f};
	static constexpr float THROTTLE_IDLE{0.05f};
	static constexpr float KFF_YAW{0.0f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_ATT_P>)   _param_ifo_att_p,
		(ParamFloat<px4::params::IFO_ATT_D>)   _param_ifo_att_d,
		(ParamFloat<px4::params::IFO_YAW_P>)   _param_ifo_yaw_p,
		(ParamFloat<px4::params::IFO_YAW_D>)   _param_ifo_yaw_d,
		(ParamFloat<px4::params::IFO_THR_HOVER>) _param_ifo_thr_hover
	)

	perf_counter_t _loop_perf;
};
