/**
 * IFODRONE Attitude Controller (outer loop)
 *
 * P attitude controller — converts attitude error into a body-rate setpoint and
 * publishes vehicle_rates_setpoint for the inner loop (mc_rate_control), which
 * runs at gyro rate and produces vehicle_torque_setpoint + vehicle_thrust_setpoint.
 *
 * IFODRONE is always kept level: roll/pitch attitude setpoint = 0.
 *   - Roll/pitch error → roll/pitch rate setpoint (P gains MC_ROLL_P / MC_PITCH_P)
 *   - Yaw error        → yaw rate setpoint        (P gain  MC_YAW_P)
 *   - thrust_body is carried through to the rate controller:
 *       Manual:    throttle stick → -Z
 *       Auto/Pos:  copied from vehicle_attitude_setpoint.thrust_body (ifo_pos_control)
 *
 * The control allocator (ActuatorEffectivenessIfodrone) maps the resulting torque to
 * tilt servos (roll/pitch) and coaxial motor differential (yaw); thrust to motors.
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

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription                 _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	vehicle_control_mode_s    _vehicle_control_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};

	hrt_abstime _last_run{0};
	float       _yaw_setpoint{NAN};

	// Manual-mode mapping
	static constexpr float YAW_RATE_MAX{1.5f};     // manual yaw stick → yaw rate setpoint [rad/s]
	static constexpr float THROTTLE_IDLE{0.05f};   // minimum throttle floor when armed

	// Rate setpoint safety clamps (the inner loop tracks these) [rad/s]
	static constexpr float RATE_LIMIT_RP{0.5f};    // ~200 deg/s roll/pitch
	static constexpr float RATE_LIMIT_YAW{0.5f};   // ~200 deg/s yaw

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>)  _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)   _param_mc_yaw_p
	)

	perf_counter_t _loop_perf;
};
