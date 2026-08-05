/**
 * IFODRONE Position Controller
 *
 * Uses the PositionControl library (P-position + PID-velocity) to compute an
 * acceleration setpoint, then converts it to body-frame thrust using the
 * current vehicle attitude.
 *
 * Setpoint source:
 *   - Offboard/Auto: trajectory_setpoint topic
 *   - goto_setpoint: direct position target (converted to trajectory_setpoint internally)
 *   - Stabilized/Altitude/Position: built here from the sticks, see
 *     generateManualSetpoint() (flight_mode_manager's trajectory setpoint is ignored)
 *
 * IFODRONE specifics:
 *   - Attitude setpoint is always level (roll/pitch = 0), only yaw is commanded.
 *   - The full 3D thrust setpoint from the PositionControl library is rotated
 *     into the body frame, constrained to the unidirectional-coax/side-EDF
 *     envelope, and published in vehicle_attitude_setpoint.thrust_body.
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
#include <uORB/topics/vehicle_attitude.h>
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
	 * Publish the PositionControl library outputs: vehicle_local_position_setpoint
	 * and a level (roll/pitch = 0) vehicle_attitude_setpoint carrying the
	 * actuator-feasible body-frame thrust setpoint.
	 */
	void publishSetpoints(const PositionControlStates &states);

	/**
	 * Build the trajectory setpoint for manual (stick-flown) modes.
	 *
	 * The stick mapping is identical in all three manual modes; what differs is
	 * how much of it is closed around the estimator:
	 *   Stabilized: XY velocity (NED), throttle stick → collective thrust
	 *   Altitude:   XY velocity (NED), throttle stick → climb rate + altitude lock
	 *   Position:   as Altitude + XY position lock when the sticks are centred
	 *
	 * XY sticks command velocity in the INERTIAL (local NED) frame, not in body:
	 * pitch stick → +North, roll stick → +East regardless of the heading.
	 * Degrades axis by axis when the estimate is missing; force_open_loop drops
	 * every axis to the estimator-free mapping (lateral thrust + collective).
	 * The heading setpoint is integrated by the caller, this only reads it.
	 */
	trajectory_setpoint_s generateManualSetpoint(const vehicle_local_position_s &local_pos,
			const PositionControlStates &states, bool force_open_loop);

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
	uORB::Subscription                 _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription                 _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription                 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// --- Publications ---
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>       _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};

	// --- Timing ---
	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_position_control_enabled{0};

	// --- Cached state ---
	matrix::Quatf          _q_att{};   ///< current attitude (NED→body thrust rotation)
	bool                   _q_att_valid{false};
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

	// --- Manual mode state ---
	float            _hold_yaw_angle{0.f};
	bool             _hold_initialized{false};
	matrix::Vector2f _pos_lock{NAN, NAN};  ///< latched XY position (Position mode, sticks centred)
	float            _alt_lock{NAN};       ///< latched altitude   (Altitude/Position, throttle centred)
	bool             _manual_alt_hold_prev{false};
	bool             _manual_pos_hold_prev{false};

	// --- EKF reset counters ---
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// --- Constants ---
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	static constexpr float STICK_DEADBAND = 0.1f;      ///< manual stick deadband [-1, 1]
	static constexpr float LOCK_VEL_MAX = 0.3f;        ///< latch position/altitude below this speed [m/s]
	static constexpr float MANUAL_YAW_RATE_MAX = 1.5f; ///< yaw stick → heading rate [rad/s]
	static constexpr float MANUAL_YAW_ERR_MAX = 0.5f;  ///< cap on heading setpoint lead [rad]

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
