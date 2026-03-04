/**
 * IFODRONE Position Control Filter
 *
 * Sits after mc_pos_control. Reads NED thrust from vehicle_local_position_setpoint,
 * rotates XY into body frame, publishes vehicle_attitude_setpoint with level body.
 *
 * Activated by IFO_POS_MODE parameter.
 */

#pragma once

#include <drivers/drv_hrt.h>
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
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class IfoPositionControl :
	public ModuleBase<IfoPositionControl>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	IfoPositionControl();
	~IfoPositionControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sp_sub{this, ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IFO_POS_MODE>) _param_ifo_pos_mode
	)
};
