/**
 * IFODRONE Position Controller
 *
 * Full 3D position controller for IFODRONE:
 * - Z-axis: Main motors (0-1) for altitude (thrust)
 * - XY-axis: Tilt servos (0-3) for horizontal position
 *
 * Key assumptions:
 * - tilt_front = -tilt_back (zeroes Z force component)
 * - tilt_right = -tilt_left (zeroes Z force component)
 * - Tilts generate horizontal acceleration without affecting Z
 *
 * Control cascade:
 * - Position P-controller → velocity setpoint
 * - Velocity PID-controller → acceleration setpoint
 * - Acceleration → tilt angles (XY) and thrust (Z)
 *
 * Subscribes to:
 *   - vehicle_local_position (current position)
 *   - trajectory_setpoint (desired position from flight mode manager)
 *   - vehicle_control_mode (to check if position control enabled)
 *
 * Publishes:
 *   - vehicle_local_position_setpoint (for other modules)
 *   - vehicle_thrust_setpoint (for control allocator - main motors)
 *   - vehicle_torque_setpoint (for control allocator - attitude/tilt control)
 *   - vehicle_attitude_setpoint (for attitude controller)
 */

#pragma once

#include <drivers/drv_hrt.h>
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
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/tiltrotor_extra_controls.h>

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
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	// Publications
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<tiltrotor_extra_controls_s> _tilt_controls_pub{ORB_ID(tiltrotor_extra_controls)};

	// Takeoff handling
	TakeoffHandling _takeoff;

	// State
	hrt_abstime _last_run{0};

	// Position hold setpoints (used when no mission setpoint available)
	bool _hold_position_initialized{false};
	float _hold_x{0.0f};
	float _hold_y{0.0f};
	float _hold_z{-5.0f};  // NED: negative = above ground

	// Velocity PID integral terms (in world frame NED)
	matrix::Vector3f _vel_int{0.0f, 0.0f, 0.0f};

	// Controller state (like PositionControl.cpp)
	matrix::Vector3f _pos{};           // Current position
	matrix::Vector3f _vel{};           // Current velocity
	matrix::Vector3f _vel_dot{};       // Current acceleration (for D term)
	matrix::Vector3f _pos_sp{};        // Position setpoint
	matrix::Vector3f _vel_sp{};        // Velocity setpoint
	matrix::Vector3f _acc_sp{};        // Acceleration setpoint
	matrix::Vector3f _thr_sp{};        // Thrust setpoint
	float _yaw{0.0f};                  // Current yaw
	float _yaw_sp{0.0f};               // Yaw setpoint

	// Tilt angles (radians) - output to control allocator
	// tilt_front = -tilt_back, tilt_right = -tilt_left
	float _tilt_pitch{0.0f};  // For front/back tilts (positive = tilt forward)
	float _tilt_roll{0.0f};   // For right/left tilts (positive = tilt right)

	// Kaskadowe regulatory PID:
	// Poziom 1: P-kontroler pozycji (pos_error * Kp_pos -> vel_sp)
	// Poziom 2: PID-kontroler prędkości (vel_error * Kp_vel + integral - vel_dot * Kd_vel -> acc_sp)
	// Poziom 3: Konwersja acc_sp -> thrust (Z) i tilt angles (XY)
	DEFINE_PARAMETERS(
		// Pozycja -> Prędkość (P gain)
		(ParamFloat<px4::params::IFO_POS_Z_P>) _param_ifo_pos_z_p,
		(ParamFloat<px4::params::IFO_POS_XY_P>) _param_ifo_pos_xy_p,
		// Prędkość -> Przyspieszenie (PID gains)
		(ParamFloat<px4::params::IFO_VEL_Z_P>) _param_ifo_vel_z_p,
		(ParamFloat<px4::params::IFO_VEL_Z_I>) _param_ifo_vel_z_i,
		(ParamFloat<px4::params::IFO_VEL_Z_D>) _param_ifo_vel_z_d,
		(ParamFloat<px4::params::IFO_VEL_XY_P>) _param_ifo_vel_xy_p,
		(ParamFloat<px4::params::IFO_VEL_XY_I>) _param_ifo_vel_xy_i,
		(ParamFloat<px4::params::IFO_VEL_XY_D>) _param_ifo_vel_xy_d,
		// Thrust limits
		(ParamFloat<px4::params::IFO_THR_HOVER>) _param_ifo_thr_hover,
		(ParamFloat<px4::params::IFO_THR_MAX>) _param_ifo_thr_max,
		(ParamFloat<px4::params::IFO_THR_MIN>) _param_ifo_thr_min,
		// Tilt limits
		(ParamFloat<px4::params::IFO_TILT_MAX>) _param_ifo_tilt_max,
		// Velocity limits
		(ParamFloat<px4::params::IFO_VEL_XY_MAX>) _param_ifo_vel_xy_max,
		(ParamFloat<px4::params::IFO_VEL_Z_UP>) _param_ifo_vel_z_up,
		(ParamFloat<px4::params::IFO_VEL_Z_DN>) _param_ifo_vel_z_dn
	)

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	// Private methods
	void _positionControl();
	void _velocityControl(float dt);
	void _accelerationControl();
	void _resetIntegrals();
	bool _inputValid();
};
