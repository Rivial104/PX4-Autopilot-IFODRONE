#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

namespace ifodrone
{

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

// P position + PID velocity with acceleration feedforward. NaN disables a setpoint.
class PositionControl
{
public:
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);
	void setVelocityLimits(float vel_horizontal, float vel_up, float vel_down);
	void setThrustLimits(float min, float max);
	void setHorizontalThrustLimit(float limit) { _lim_thr_xy_max = math::max(limit, 0.f); }
	void setHoverThrust(float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.05f, 0.9f); }

	void setState(const PositionControlStates &states);
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);
	bool update(float dt);
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	void resetIntegral() { _vel_int.setZero(); }
	void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); }
	void resetIntegralZ() { _vel_int(2) = 0.f; }

	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	bool _inputValid();
	void _positionControl();
	void _velocityControl(float dt);
	void _accelerationControl();

	matrix::Vector3f _gain_pos_p;
	matrix::Vector3f _gain_vel_p;
	matrix::Vector3f _gain_vel_i;
	matrix::Vector3f _gain_vel_d;

	float _lim_vel_horizontal{};
	float _lim_vel_up{};
	float _lim_vel_down{};
	float _lim_thr_min{};
	float _lim_thr_max{};
	float _lim_thr_xy_max{};
	float _hover_thrust{};

	matrix::Vector3f _pos;
	matrix::Vector3f _vel;
	matrix::Vector3f _vel_dot;
	matrix::Vector3f _vel_int;
	float _yaw{};

	matrix::Vector3f _pos_sp;
	matrix::Vector3f _vel_sp;
	matrix::Vector3f _acc_sp;
	matrix::Vector3f _thr_sp;
	float _yaw_sp{};
	float _yawspeed_sp{};
};

} // namespace ifodrone
