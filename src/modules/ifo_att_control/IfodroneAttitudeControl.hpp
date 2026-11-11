#pragma once

#include <AttitudeControl.hpp>
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
	struct Setpoint {
		matrix::Quatf    q;
		matrix::Vector3f thrust; // body frame
		float            yaw_rate_ff;
	};

	void Run() override;
	void update();
	void parameters_update();
	Setpoint attitudeSetpoint(const vehicle_attitude_setpoint_s &att_sp, bool valid, bool fresh, float yaw) const;
	Setpoint manualSetpoint(float yaw, float dt, hrt_abstime now);

	AttitudeControl _attitude_control;

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription                 _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	vehicle_control_mode_s    _vehicle_control_mode{};

	hrt_abstime _last_run{0};
	float       _yaw_setpoint{NAN};
	float       _thrust_z{0.f};

	static constexpr float YAW_RATE_MAX{1.5f}; // rad/s
	static constexpr float THROTTLE_IDLE{0.05f};
	static constexpr hrt_abstime ATT_SP_TIMEOUT{200_ms};
	static constexpr hrt_abstime MAUAL_SP_TIMEOUT{500_ms};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>)  _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)   _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max
	)

	perf_counter_t _loop_perf;
};
