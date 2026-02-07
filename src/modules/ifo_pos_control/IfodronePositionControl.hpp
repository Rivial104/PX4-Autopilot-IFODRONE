/**
 * IFODRONE Position Controller
 *
 * Minimal position controller scaffold for IFODRONE.
 *
 * Subscribes to:
 *   - vehicle_local_position (current position)
 *   - trajectory_setpoint (desired position from flight mode manager)
 *   - vehicle_control_mode (to check if position control enabled)
 *
 * Publishes:
 *   - vehicle_local_position_setpoint (for other modules)
 *   - vehicle_attitude_setpoint (empty for now)
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

using namespace time_literals;

class IfodronePositionControl :
	public ModuleBase<IfodronePositionControl>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	IfodronePositionControl();
	~IfodronePositionControl() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }
		PX4_INFO("usage: ifo_pos_control start|stop|status");
		return 0;
	}

	bool init();
	int print_status() override;

private:
	void Run() override;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	// Publications
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	// State
	hrt_abstime _last_run{0};

	// Position hold setpoint (used when no mission setpoint available)
	bool _hold_position_initialized{false};
	float _hold_z{5.0f};
	float _z_sp{0.0f};

	static constexpr int DEBUG_INTERVAL_US = 1000000;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_POS_Z_P>) _param_ifo_pos_z_p,
		(ParamFloat<px4::params::IFO_VEL_Z_P>) _param_ifo_vel_z_p,
		(ParamFloat<px4::params::IFO_POS_XY_P>) _param_ifo_pos_xy_p,
		(ParamFloat<px4::params::IFO_VEL_XY_P>) _param_ifo_vel_xy_p,
		(ParamFloat<px4::params::IFO_THR_MAX>) _param_ifo_thr_max,
		(ParamFloat<px4::params::IFO_THR_MIN>) _param_ifo_thr_min,
		(ParamFloat<px4::params::IFO_THR_XY_MAX>) _param_ifo_thr_xy_max,
		(ParamFloat<px4::params::IFO_THR_HOVER>) _param_ifo_thr_hover
	)

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
