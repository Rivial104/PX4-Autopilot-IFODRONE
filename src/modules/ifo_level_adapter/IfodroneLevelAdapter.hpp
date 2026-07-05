/**
 * IFODRONE Level Adapter
 *
 * Thin seam that lets the IFODRONE fly the STOCK multicopter control cascade while
 * keeping its body perfectly level (roll/pitch = 0). The stock mc_pos_control is
 * started with the "vtol" argument so it publishes its (tilted) attitude setpoint to
 * mc_virtual_attitude_setpoint instead of vehicle_attitude_setpoint.
 *
 * This module subscribes to that virtual setpoint, reconstructs the desired NED thrust
 * vector (direction encoded in q_d, magnitude in thrust_body[2]), and re-expresses it as:
 *   - a LEVEL attitude setpoint (yaw only, roll/pitch = 0), and
 *   - a body-frame thrust vector thrust_body[0..2] (NED thrust rotated into the level
 *     body frame using heading only).
 * It then publishes the result on vehicle_attitude_setpoint for the stock mc_att_control.
 *
 * Downstream chain is entirely stock:
 *   mc_att_control : holds roll/pitch = 0, tracks yaw, forwards thrust_body[0..2]
 *   mc_rate_control: rate PID → vehicle_torque_setpoint + vehicle_thrust_setpoint (full xyz)
 *   control_allocator + ActuatorEffectivenessIfodrone:
 *       thrust_body[2] → coaxial pair (Z), thrust_body[0/1] → side EDFs (lateral),
 *       roll/pitch torque → tilt servos (hold level), yaw torque → coax differential.
 *
 * Single-publisher invariant: the adapter only publishes while multicopter position
 * control is enabled. In pure Stabilized/Acro mc_att_control generates its own setpoint,
 * so the two never publish vehicle_attitude_setpoint at the same time.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

using namespace time_literals;

class IfodroneLevelAdapter :
	public ModuleBase<IfodroneLevelAdapter>,
	public px4::WorkItem
{
public:
	IfodroneLevelAdapter();
	~IfodroneLevelAdapter() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	// Trigger on the stock (vtol) mc_pos_control output.
	uORB::SubscriptionCallbackWorkItem _virtual_attitude_setpoint_sub{this, ORB_ID(mc_virtual_attitude_setpoint)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	vehicle_control_mode_s _vehicle_control_mode{};

	perf_counter_t _loop_perf;
};
