/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.hpp
 *
 * Control-specific vector helpers.
 */

#pragma once

#include <matrix/matrix/math.hpp>

namespace ifodrone
{

namespace ControlMath
{
// Sum constrained to max magnitude, prioritizing v0 over feedforward v1.
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

// NaN marks an unset axis; finite values are added, a NaN setpoint takes the addition.
void addIfNotNan(float &setpoint, const float addition);
void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);
void setZeroIfNanVector3f(matrix::Vector3f &vector);

// Rotate NED thrust into body axes, reject +Z and limit side-EDF demand.
matrix::Vector3f constrainIfodroneBodyThrust(const matrix::Vector3f &thrust_ned,
		const matrix::Quatf &attitude, float thrust_xy_max);
}

} // namespace ifodrone
