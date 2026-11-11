/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

namespace ifodrone
{

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	_lim_thr_max = math::max(max, 0.f);
	_lim_thr_min = math::constrain(min, 0.f, _lim_thr_max);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	if (!_inputValid()) {
		return false;
	}

	_positionControl();
	_velocityControl(dt);

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;

	return _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Position-derived velocity has priority over feed-forward when limiting horizontally.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID with D on measured acceleration; NaN setpoints get no feedback contribution.
	Vector3f vel_error = _vel_sp - _vel;
	ControlMath::addIfNotNanVector3f(_acc_sp, vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d));

	// Independent axes: horizontal thrust does not depend on climb acceleration.
	_thr_sp = _acc_sp * (_hover_thrust / CONSTANTS_ONE_G);
	_thr_sp(2) = math::min(_thr_sp(2) - _hover_thrust, -_lim_thr_min);

	// Vertical priority, keeping a horizontal margin.
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);
	const float allocated_xy = math::min(thrust_sp_xy_norm, math::min(_lim_thr_xy_max, _lim_thr_max));
	const float thrust_z_max = sqrtf(math::max(thrust_max_squared - math::sq(allocated_xy), 0.f));

	_thr_sp(2) = math::max(_thr_sp(2), -thrust_z_max);

	// Z anti-windup at the thrust limits.
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -thrust_z_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	const float thrust_max_xy = math::min(sqrtf(math::max(thrust_max_squared - math::sq(_thr_sp(2)), 0.f)), _lim_thr_xy_max);

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;

		// Tracking anti-windup; with P disabled, only block integration into saturation.
		const Vector2f unproduced_acc = _acc_sp.xy() - Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

		if (_gain_vel_p(0) > 0.f) {
			vel_error.xy() -= (2.f / _gain_vel_p(0)) * unproduced_acc;

		} else {
			for (int i = 0; i < 2; ++i) {
				if (vel_error(i) * unproduced_acc(i) > 0.f) {
					vel_error(i) = 0.f;
				}
			}
		}
	}

	ControlMath::setZeroIfNanVector3f(vel_error);
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

bool PositionControl::_inputValid() const
{
	for (int i = 0; i < 3; i++) {
		// Every axis needs some setpoint.
		if (!PX4_ISFINITE(_pos_sp(i)) && !PX4_ISFINITE(_vel_sp(i)) && !PX4_ISFINITE(_acc_sp(i))) {
			return false;
		}

		// Controlled states need a valid estimate.
		if (PX4_ISFINITE(_pos_sp(i)) && !PX4_ISFINITE(_pos(i))) {
			return false;
		}

		if ((PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i))) &&
		    !(PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i)))) {
			return false;
		}
	}

	// x and y setpoints come in pairs.
	return PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1))
	       && PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1))
	       && PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1));
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

} // namespace ifodrone
