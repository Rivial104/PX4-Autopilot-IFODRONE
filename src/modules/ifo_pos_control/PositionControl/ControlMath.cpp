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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ifodrone
{

namespace ControlMath
{
Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		return v0 + v1;
	}

	// v0 has priority and alone exceeds the limit.
	if (v0.length() >= max) {
		return v0.normalized() * max;
	}

	// Intersect v0 + s * unit(v1) with the limit circle.
	const Vector2f u1 = v1.normalized();
	const float m = u1.dot(v0);
	const float c = v0.dot(v0) - max * max;
	const float s = -m + sqrtf(m * m - c);
	return v0 + u1 * s;
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (!PX4_ISFINITE(setpoint)) {
		setpoint = addition;

	} else if (PX4_ISFINITE(addition)) {
		setpoint += addition;
	}
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	addIfNotNanVector3f(vector, Vector3f());
}

Vector3f constrainIfodroneBodyThrust(const Vector3f &thrust_ned, const Quatf &attitude,
				     float thrust_xy_max)
{
	Vector3f thrust_finite = thrust_ned;
	setZeroIfNanVector3f(thrust_finite);

	Vector3f thrust_body = attitude.rotateVectorInverse(thrust_finite);

	// The coaxial pair is unidirectional and cannot produce force along body +Z.
	thrust_body(2) = math::min(thrust_body(2), 0.f);

	// Limit side-EDF demand after the full attitude transformation.
	const float xy_limit = math::max(thrust_xy_max, 0.f);
	const float xy_norm = thrust_body.xy().norm();

	if (xy_norm > xy_limit && xy_norm > FLT_EPSILON) {
		thrust_body.xy() *= xy_limit / xy_norm;
	}

	return thrust_body;
}

} // ControlMath

} // namespace ifodrone
