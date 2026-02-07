/**
 * IFODRONE Position Controller
 *
 * Position controller based on MulticopterPositionControl architecture
 * but with custom thrust/torque generation for IFODRONE airframe.
 *
 * Uses:
 * - TakeoffHandling for takeoff state machine (from mc_pos_control)
 * - EKF reset handling (like mc_pos_control)
 * - Failsafe setpoint generation
 * - Vehicle constraints
 *
 * Custom:
 * - Thrust generation for IFODRONE (separate XY tilt and Z main motors)
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

// Takeoff handling (local copy from mc_pos_control)
#include "Takeoff.hpp"

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
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	/**
	 * Update parameters from storage
	 * @param force force update (ignore update flag)
	 */
	void parameters_update(bool force);

	/**
	 * Adjust trajectory setpoint for EKF position/velocity resets
	 * Keeps setpoints fixed in world frame after EKF resets
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos,
					trajectory_setpoint_s &setpoint);

	/**
	 * Generate failsafe setpoint when trajectory setpoint is invalid
	 * @param now current timestamp
	 * @param warn whether to log warnings
	 * @return failsafe trajectory setpoint
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, bool warn);

	/**
	 * Check if setpoint is valid for control
	 * @param setpoint trajectory setpoint to check
	 * @return true if setpoint has at least one valid control component
	 */
	bool isSetpointValid(const trajectory_setpoint_s &setpoint) const;

	// ============================================================
	// USER CUSTOMIZATION AREA - Implement these for IFODRONE
	// ============================================================

	/**
	 * TODO: Implement position control algorithm
	 * Compute velocity setpoint from position error
	 *
	 * @param pos_sp desired position (NED)
	 * @param pos current position (NED)
	 * @param speed_up max ascent speed [m/s]
	 * @param speed_down max descent speed [m/s]
	 * @param dt time step
	 * @return velocity setpoint (NED)
	 */
	matrix::Vector3f computeVelocitySetpoint(const matrix::Vector3f &pos_sp,
						 const matrix::Vector3f &pos,
						 float speed_up,
						 float speed_down,
						 float dt);

	/**
	 * TODO: Implement velocity control algorithm
	 * Compute acceleration setpoint from velocity error
	 *
	 * @param vel_sp desired velocity (NED)
	 * @param vel current velocity (NED)
	 * @param dt time step
	 * @return acceleration setpoint (NED)
	 */
	matrix::Vector3f computeAccelerationSetpoint(const matrix::Vector3f &vel_sp,
						     const matrix::Vector3f &vel,
						     float dt);

	/**
	 * TODO: Implement IFODRONE-specific thrust generation
	 * Convert acceleration setpoint to body-frame thrust
	 *
	 * For IFODRONE:
	 * - thrust_body[2] -> main motors (Z axis, altitude)
	 * - thrust_body[0,1] -> tilt motors (XY, horizontal position)
	 *
	 * @param accel_sp acceleration setpoint (NED frame)
	 * @param yaw current yaw angle
	 * @return thrust setpoint in body frame (normalized 0-1)
	 */
	matrix::Vector3f computeIfodroneThrust(const matrix::Vector3f &accel_sp, float yaw);

	/**
	 * TODO: Implement attitude setpoint generation for IFODRONE
	 * For IFODRONE this may be simpler than multicopter (no roll/pitch for position)
	 *
	 * @param thrust_body body frame thrust
	 * @param yaw_sp desired yaw
	 * @param yawspeed_sp desired yaw rate
	 * @param tilt_limit_rad maximum allowable tilt angle in rad
	 * @return attitude setpoint message
	 */
	vehicle_attitude_setpoint_s computeAttitudeSetpoint(const matrix::Vector3f &thrust_body,
							    float yaw_sp, float yawspeed_sp, float tilt_limit_rad);

	// ============================================================
	// END USER CUSTOMIZATION AREA
	// ============================================================

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

	// Publications
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::PublicationData<takeoff_status_s> _takeoff_status_pub{ORB_ID(takeoff_status)};

	// Reused from mc_pos_control
	TakeoffHandling _takeoff;                       ///< Takeoff state machine
	SlewRate<float> _tilt_limit_slew_rate;          ///< Smooth tilt limit changes

	// Control state
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_constraints_s _vehicle_constraints{};
	trajectory_setpoint_s _setpoint{};              ///< Current trajectory setpoint
	trajectory_setpoint_s _last_valid_setpoint{};   ///< Last valid setpoint for fallback

	// Timing
	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_position_control_enabled{0};
	hrt_abstime _last_warn{0};

	// EKF reset counters
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// Hover thrust
	float _hover_thrust{0.45f};
	bool _hover_thrust_initialized{false};
	matrix::Vector3f _pos_integral{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _vel_integral{0.0f, 0.0f, 0.0f};

	// State
	matrix::Vector3f _velocity{};
	matrix::Vector3f _position{};
	float _yaw{0.0f};

	// Debug
	static constexpr uint64_t DEBUG_INTERVAL_US = 1000000; // 1 Hz

	// Parameters - reuse some MPC params + custom IFO params
	DEFINE_PARAMETERS(
		// Position gains (custom for IFODRONE)
		(ParamFloat<px4::params::IFO_POS_Z_P>) _param_ifo_pos_z_p,
		(ParamFloat<px4::params::IFO_VEL_Z_P>) _param_ifo_vel_z_p,
		(ParamFloat<px4::params::IFO_POS_XY_P>) _param_ifo_pos_xy_p,
		(ParamFloat<px4::params::IFO_VEL_XY_P>) _param_ifo_vel_xy_p,
		(ParamFloat<px4::params::IFO_POS_I>) _param_ifo_pos_i,
		(ParamFloat<px4::params::IFO_VEL_I>) _param_ifo_vel_i,

		// Thrust limits (custom for IFODRONE)
		(ParamFloat<px4::params::IFO_THR_MAX>) _param_ifo_thr_max,
		(ParamFloat<px4::params::IFO_THR_MIN>) _param_ifo_thr_min,
		(ParamFloat<px4::params::IFO_THR_XY_MAX>) _param_ifo_thr_xy_max,
		(ParamFloat<px4::params::IFO_THR_HOVER>) _param_ifo_thr_hover,

		// Reuse some MPC params for takeoff/landing
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _param_mpc_tiltmax_lnd,

		// Takeoff params
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>) _param_mpc_z_vel_p_acc,

		// Hover thrust estimation
		(ParamBool<px4::params::MPC_USE_HTE>) _param_mpc_use_hte
	)

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
