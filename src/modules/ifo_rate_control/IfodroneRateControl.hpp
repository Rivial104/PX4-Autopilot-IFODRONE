/**
 * IFODRONE Rate Controller (inner loop)
 *
 * PD controller on angular rate error.
 * Reads vehicle_rates_setpoint (from ifo_att_control) and
 * vehicle_angular_velocity (from IMU), publishes vehicle_torque_setpoint.
 *
 *   - rate_error = rates_sp - rates_actual
 *   - torque = Kp * rate_error - Kd * rate_derivative
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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class IfodroneRateControl :
	public ModuleBase<IfodroneRateControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	IfodroneRateControl();
	~IfodroneRateControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription                 _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};

	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	hrt_abstime _last_run{0};

	// Previous rates for derivative computation
	float _prev_roll_rate{0.f};
	float _prev_pitch_rate{0.f};
	float _prev_yaw_rate{0.f};

	// Low-pass filtered rate derivative
	float _roll_rate_dot{0.f};
	float _pitch_rate_dot{0.f};
	float _yaw_rate_dot{0.f};

	static constexpr float TORQUE_LIMIT{1.0f};
	static constexpr float DERIV_FILTER{0.15f};  // derivative low-pass coefficient

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_RATE_P>)  _param_ifo_rate_p,
		(ParamFloat<px4::params::IFO_RATE_D>)  _param_ifo_rate_d,
		(ParamFloat<px4::params::IFO_YRATE_P>) _param_ifo_yrate_p,
		(ParamFloat<px4::params::IFO_YRATE_D>) _param_ifo_yrate_d
	)

	perf_counter_t _loop_perf;
};
