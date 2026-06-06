/**
 * IFODRONE Position Controller
 *
 * Uses the PositionControl library (P-position + PID-velocity) to compute an
 * acceleration setpoint, then converts it to body-frame thrust using the
 * current vehicle attitude.
 *
 * Setpoint source (via trajectory_setpoint topic):
 *   - Offboard: external trajectory_setpoint
 *   - Manual/Stabilize/Position: trajectory_setpoint from flight_mode_manager
 *   - goto_setpoint: direct position target (converted to trajectory_setpoint internally)
 *
 * IFODRONE specifics:
 *   - Body setpoint is always level: no pitch/roll attitude generation.
 *   - Z thrust is independent (hover_thrust ± correction).
 *   - XY thrust accounts for the current body attitude, including tilted Z thrust.
 *   - Only yaw is published in vehicle_attitude_setpoint.
 */

#pragma once

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"

#include <lib/matrix/matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

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

	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_update(bool force);

	/**
	 * Convert PositionControl library acceleration setpoint into body-frame
	 * thrust for the IFODRONE.
	 */
	matrix::Vector3f accelerationToThrust(const matrix::Vector3f &acc_sp) const;

	/**
	 * Adjust setpoint for EKF resets (position/velocity jumps).
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos, trajectory_setpoint_s &setpoint);

	/**
	 * Generate a failsafe setpoint (stop or descend).
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states);

	// --- Core position control ---
	PositionControl _control;
	TakeoffHandling _takeoff;

	// --- Subscriptions ---
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription                 _goto_setpoint_sub{ORB_ID(goto_setpoint)};
	uORB::Subscription                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription                 _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription                 _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// --- Publications ---
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>       _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>         _thrust_sp_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};

	// --- Timing ---
	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_position_control_enabled{0};

	// --- Cached state ---
	trajectory_setpoint_s  _setpoint{PositionControl::empty_trajectory_setpoint};
	trajectory_setpoint_s  _last_valid_setpoint{PositionControl::empty_trajectory_setpoint};
	vehicle_control_mode_s _vehicle_control_mode{};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	// --- Hold mode state ---
	float            _hold_yaw_angle{0.f};
	float            _hold_z{NAN};
	bool             _hold_initialized{false};

	// --- EKF reset counters ---
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// --- Constants ---
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IFO_POS_Z_P>)   _param_ifo_pos_z_p,
		(ParamFloat<px4::params::IFO_VEL_Z_P>)   _param_ifo_vel_z_p,
		(ParamFloat<px4::params::IFO_POS_XY_P>)  _param_ifo_pos_xy_p,
		(ParamFloat<px4::params::IFO_VEL_XY_P>)  _param_ifo_vel_xy_p,
		(ParamFloat<px4::params::IFO_THR_MAX>)    _param_ifo_thr_max,
		(ParamFloat<px4::params::IFO_THR_MIN>)    _param_ifo_thr_min,
		(ParamFloat<px4::params::IFO_THR_XY_MAX>) _param_ifo_thr_xy_max,
		(ParamFloat<px4::params::IFO_THR_HOVER>)  _param_ifo_thr_hover,
		(ParamFloat<px4::params::IFO_VEL_Z_I>)   _param_ifo_vel_z_i,
		(ParamFloat<px4::params::IFO_VEL_XY_I>)  _param_ifo_vel_xy_i,
		(ParamFloat<px4::params::IFO_VEL_Z_D>)   _param_ifo_vel_z_d,
		(ParamFloat<px4::params::IFO_VEL_XY_D>)  _param_ifo_vel_xy_d,
		(ParamFloat<px4::params::IFO_VEL_MAX_XY>) _param_ifo_vel_max_xy,
		(ParamFloat<px4::params::IFO_VEL_MAX_UP>) _param_ifo_vel_max_up,
		(ParamFloat<px4::params::IFO_VEL_MAX_DN>) _param_ifo_vel_max_dn,
		(ParamFloat<px4::params::IFO_TKO_SPEED>)  _param_ifo_tko_speed,
		(ParamFloat<px4::params::IFO_LAND_SPEED>) _param_ifo_land_speed,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,
		(ParamFloat<px4::params::IFO_TKO_RAMP_T>) _param_ifo_tko_ramp_t
	)

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
