/**
 * IFODRONE Position Control Filter
 *
 * Lightweight filter that sits after mc_pos_control and overrides
 * vehicle_attitude_setpoint for the IFODRONE platform.
 *
 * mc_pos_control computes a NED-frame thrust vector and a tilted attitude
 * quaternion (standard multicopter approach). This filter intercepts
 * the local_position_setpoint, extracts the NED thrust, and republishes
 * vehicle_attitude_setpoint with:
 *   - Level body quaternion (roll=0, pitch=0, desired yaw)
 *   - Body-frame 3D thrust (XY for tilt motors, Z for main motors)
 *
 * Activated by IFO_POS_MODE parameter.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class IfoPositionControl :
	public ModuleBase<IfoPositionControl>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	IfoPositionControl();
	~IfoPositionControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Triggered by vehicle_local_position_setpoint from mc_pos_control
	uORB::SubscriptionCallbackWorkItem _local_pos_sp_sub{this, ORB_ID(vehicle_local_position_setpoint)};

	// Current vehicle attitude (for yaw in NED→body rotation)
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Overwrites mc_pos_control's vehicle_attitude_setpoint
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IFO_POS_MODE>) _param_ifo_pos_mode,
		(ParamFloat<px4::params::IFO_THR_XY_MAX>) _param_ifo_thr_xy_max,
		(ParamFloat<px4::params::IFO_XY_THR_SCL>) _param_ifo_xy_thr_scl,
		(ParamFloat<px4::params::IFO_XY_VEL_P>) _param_ifo_xy_vel_p,
		(ParamFloat<px4::params::IFO_XY_ACC_FF>) _param_ifo_xy_acc_ff
	)

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
