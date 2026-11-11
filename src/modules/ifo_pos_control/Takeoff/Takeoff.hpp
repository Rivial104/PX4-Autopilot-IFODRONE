/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.hpp
 *
 * Takeoff state machine and smooth climb-rate ramp.
 */

#pragma once

#include <lib/hysteresis/hysteresis.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/takeoff_status.h>

using namespace time_literals;

namespace ifodrone
{

enum class TakeoffState {
	disarmed = takeoff_status_s::TAKEOFF_STATE_DISARMED,
	spoolup = takeoff_status_s::TAKEOFF_STATE_SPOOLUP,
	ready_for_takeoff = takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF,
	rampup = takeoff_status_s::TAKEOFF_STATE_RAMPUP,
	flight = takeoff_status_s::TAKEOFF_STATE_FLIGHT
};

class TakeoffHandling
{
public:
	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); }
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; }

	// Ramp start velocity that gives zero thrust from the velocity controller.
	void generateInitialRampValue(const float velocity_p_gain);

	// Must be called every cycle (also outside altitude control) so mode switches in flight skip takeoff.
	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const bool skip_takeoff, const hrt_abstime &now_us);

	// Climb-rate limit: ramp start value on the ground, takeoff_desired_vz in flight.
	float updateRamp(const float dt, const float takeoff_desired_vz);

	TakeoffState getTakeoffState() const { return _takeoff_state; }

private:
	TakeoffState _takeoff_state{TakeoffState::disarmed};
	systemlib::Hysteresis _spoolup_time_hysteresis{false};

	float _takeoff_ramp_time{0.f};
	float _takeoff_ramp_vz_init{0.f};
	float _takeoff_ramp_progress{0.f}; // 0 to 1
};

} // namespace ifodrone
