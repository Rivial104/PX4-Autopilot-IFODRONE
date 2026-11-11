/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessIfodroneTest.cpp
 *
 * Functional tests for the IFODRONE vertical booster (IFO_BST_ROTOR / IFO_BST_RATIO):
 * folding of the booster column into the coax columns and the slaved booster setpoint.
 */

#include <gtest/gtest.h>
#include <parameters/param.h>

#include <cmath>

#include "ActuatorEffectivenessIfodrone.hpp"

using namespace matrix;

using EffMatrix = ActuatorEffectiveness::EffectivenessMatrix;
using ActuatorVector = ActuatorEffectiveness::ActuatorVector;

static constexpr int NUM_AXES = ActuatorEffectiveness::NUM_AXES;
static constexpr int NUM_ACTUATORS = ActuatorEffectiveness::NUM_ACTUATORS;
static constexpr int BOOSTER = 6;
static constexpr int NUM_ROTORS = 7;
static constexpr int NUM_SERVOS = 4;
static constexpr float TOL = 1e-5f;

class ActuatorEffectivenessIfodroneTest : public ::testing::Test
{
protected:
	static void setF(const char *name, float value)
	{
		const param_t h = param_find(name);
		ASSERT_NE(h, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(h, &value), 0) << name;
	}

	static void setI(const char *name, int32_t value)
	{
		const param_t h = param_find(name);
		ASSERT_NE(h, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(h, &value), 0) << name;
	}

	static void setRotor(int i, Vector3f p, Vector3f a, float km, float ct, int32_t tilt)
	{
		char buf[24];
		const char *suffix_f[] = {"PX", "PY", "PZ", "AX", "AY", "AZ"};
		const float values[] = {p(0), p(1), p(2), a(0), a(1), a(2)};

		for (int k = 0; k < 6; ++k) {
			snprintf(buf, sizeof(buf), "CA_ROTOR%d_%s", i, suffix_f[k]);
			setF(buf, values[k]);
		}

		snprintf(buf, sizeof(buf), "CA_ROTOR%d_KM", i);
		setF(buf, km);
		snprintf(buf, sizeof(buf), "CA_ROTOR%d_CT", i);
		setF(buf, ct);
		snprintf(buf, sizeof(buf), "CA_ROTOR%d_TILT", i);
		setI(buf, tilt);
	}

	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		setI("CA_ROTOR_COUNT", NUM_ROTORS);
		setRotor(0, {0.f, 0.f, -0.1f}, {0.f, 0.f, -1.f}, 0.4f, 0.6f, 0);
		setRotor(1, {0.f, 0.f, 0.1f}, {0.f, 0.f, -1.f}, -0.4f, 0.6f, 0);
		setRotor(2, {0.535f, 0.f, 0.045f}, {1.f, 0.f, 0.f}, 0.f, 0.1f, 1);
		setRotor(3, {0.f, 0.535f, 0.045f}, {0.f, 1.f, 0.f}, 0.f, 0.1f, 2);
		setRotor(4, {-0.535f, 0.f, 0.045f}, {-1.f, 0.f, 0.f}, 0.f, 0.1f, 3);
		setRotor(5, {0.f, -0.535f, 0.045f}, {0.f, -1.f, 0.f}, 0.f, 0.1f, 4);
		setRotor(6, {0.f, 0.f, 0.045f}, {0.f, 0.f, -1.f}, 0.f, 1.4f, 0);

		setI("CA_SV_TL_COUNT", NUM_SERVOS);
		const int32_t td[NUM_SERVOS] = {0, 90, 180, 270};
		char buf[24];

		for (int i = 0; i < NUM_SERVOS; ++i) {
			snprintf(buf, sizeof(buf), "CA_SV_TL%d_MINA", i);
			setF(buf, -35.f);
			snprintf(buf, sizeof(buf), "CA_SV_TL%d_MAXA", i);
			setF(buf, 35.f);
			snprintf(buf, sizeof(buf), "CA_SV_TL%d_TD", i);
			setI(buf, td[i]);
			snprintf(buf, sizeof(buf), "CA_SV_TL%d_CT", i);
			setI(buf, 0);
		}

		setF("IFO_TILT_HOVER", 0.f);
		setF("IFO_EDF_TRIM", 0.2f);
		setF("IFO_EDF_GAIN", 1.f);
		setF("IFO_TILT_GAIN", 1.f);
		setF("IFO_CA_LIN_TAU", 0.05f);
		setI("IFO_BST_ROTOR", -1);
		setF("IFO_BST_RATIO", 2.f);
	}

	// Params are read at construction: construct a fresh instance after setting them
	static EffMatrix computeMatrix(ActuatorEffectivenessIfodrone &ifo, int *num_actuators = nullptr)
	{
		ActuatorEffectiveness::Configuration config{};
		EXPECT_TRUE(ifo.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));

		if (num_actuators) {
			*num_actuators = config.num_actuators_matrix[0];
		}

		return config.effectiveness_matrices[0];
	}

	static EffMatrix matrixWithBooster(int32_t bst_rotor, int *num_actuators = nullptr)
	{
		setI("IFO_BST_ROTOR", bst_rotor);
		ActuatorEffectivenessIfodrone ifo(nullptr);
		return computeMatrix(ifo, num_actuators);
	}

	static void expectColumnsEqual(const EffMatrix &a, int col_a, const EffMatrix &b, int col_b, const char *what)
	{
		for (int axis = 0; axis < NUM_AXES; ++axis) {
			EXPECT_NEAR(a(axis, col_a), b(axis, col_b), TOL) << what << " axis " << axis
					<< " col " << col_a << "/" << col_b;
		}
	}

	static void expectMatricesEqual(const EffMatrix &a, const EffMatrix &b, const char *what)
	{
		for (int col = 0; col < NUM_ACTUATORS; ++col) {
			expectColumnsEqual(a, col, b, col, what);
		}
	}

	static void defaultLimits(ActuatorVector &min, ActuatorVector &max)
	{
		for (int i = 0; i < NUM_ACTUATORS; ++i) {
			const bool servo = i >= NUM_ROTORS && i < NUM_ROTORS + NUM_SERVOS;
			min(i) = servo ? -1.f : 0.f;
			max(i) = 1.f;
		}
	}

	// Runs updateSetpoint() on an instance whose matrix was already computed; returns the booster setpoint
	static float boosterSp(ActuatorEffectivenessIfodrone &ifo, float sp0, float sp1, float sp_boost_in = 0.f)
	{
		ActuatorVector sp{}, min{}, max{};
		defaultLimits(min, max);

		for (int i = 2; i < NUM_ROTORS - 1; ++i) {
			sp(i) = 0.2f;
		}

		sp(0) = sp0;
		sp(1) = sp1;
		sp(BOOSTER) = sp_boost_in;
		ifo.updateSetpoint(Vector<float, NUM_AXES> {}, 0, sp, min, max);

		// Coax setpoints themselves must not be touched
		if (std::isnan(sp0)) {
			EXPECT_TRUE(std::isnan(sp(0)));

		} else {
			EXPECT_FLOAT_EQ(sp(0), sp0);
		}

		if (std::isnan(sp1)) {
			EXPECT_TRUE(std::isnan(sp(1)));

		} else {
			EXPECT_FLOAT_EQ(sp(1), sp1);
		}

		return sp(BOOSTER);
	}
};

TEST_F(ActuatorEffectivenessIfodroneTest, DisabledMatchesPlainRotors)
{
	int num_act = 0;
	const EffMatrix m = matrixWithBooster(-1, &num_act);
	EXPECT_EQ(num_act, NUM_ROTORS + NUM_SERVOS);

	// Booster column is the plain rotor column: only Fz = -CT, no torque (on the Z axis, KM 0)
	for (int axis = 0; axis < NUM_AXES; ++axis) {
		const float expected = (axis == ActuatorEffectiveness::ControlAxis::THRUST_Z) ? -1.4f : 0.f;
		EXPECT_NEAR(m(axis, BOOSTER), expected, TOL) << "axis " << axis;
	}

	// Coax columns: Fz = -0.6, yaw = -ct*km*az = +/-0.24
	EXPECT_NEAR(m(ActuatorEffectiveness::ControlAxis::THRUST_Z, 0), -0.6f, TOL);
	EXPECT_NEAR(m(ActuatorEffectiveness::ControlAxis::THRUST_Z, 1), -0.6f, TOL);
	EXPECT_NEAR(m(ActuatorEffectiveness::ControlAxis::YAW, 0), 0.24f, TOL);
	EXPECT_NEAR(m(ActuatorEffectiveness::ControlAxis::YAW, 1), -0.24f, TOL);
}

TEST_F(ActuatorEffectivenessIfodroneTest, DisabledEqualsConfigWithoutBoosterRotor)
{
	const EffMatrix with_rotor = matrixWithBooster(-1);

	setI("CA_ROTOR_COUNT", NUM_ROTORS - 1);
	int num_act = 0;
	const EffMatrix without_rotor = matrixWithBooster(-1, &num_act);
	EXPECT_EQ(num_act, NUM_ROTORS - 1 + NUM_SERVOS);

	// Motors 0..5 identical
	for (int col = 0; col < NUM_ROTORS - 1; ++col) {
		expectColumnsEqual(with_rotor, col, without_rotor, col, "motor");
	}

	// Servo columns shifted by one
	for (int s = 0; s < NUM_SERVOS; ++s) {
		expectColumnsEqual(with_rotor, NUM_ROTORS + s, without_rotor, NUM_ROTORS - 1 + s, "servo");
	}
}

TEST_F(ActuatorEffectivenessIfodroneTest, EnabledFoldsBoosterIntoCoax)
{
	const EffMatrix off = matrixWithBooster(-1);
	setF("IFO_BST_RATIO", 2.f);
	int num_act = 0;
	const EffMatrix on = matrixWithBooster(BOOSTER, &num_act);

	// Actuator count/indexing unchanged
	EXPECT_EQ(num_act, NUM_ROTORS + NUM_SERVOS);

	// Booster column zeroed
	for (int axis = 0; axis < NUM_AXES; ++axis) {
		EXPECT_NEAR(on(axis, BOOSTER), 0.f, TOL) << "axis " << axis;
	}

	// Coax Fz: -0.6 + 0.5 * 2 * (-1.4) = -2.0
	EXPECT_NEAR(on(ActuatorEffectiveness::ControlAxis::THRUST_Z, 0), -2.0f, TOL);
	EXPECT_NEAR(on(ActuatorEffectiveness::ControlAxis::THRUST_Z, 1), -2.0f, TOL);

	// Coax: every other row unchanged (booster on the Z axis, KM 0)
	for (int col = 0; col < 2; ++col) {
		for (int axis = 0; axis < NUM_AXES; ++axis) {
			if (axis != ActuatorEffectiveness::ControlAxis::THRUST_Z) {
				EXPECT_NEAR(on(axis, col), off(axis, col), TOL) << "coax " << col << " axis " << axis;
			}
		}
	}

	EXPECT_NEAR(on(ActuatorEffectiveness::ControlAxis::YAW, 0), 0.24f, TOL);
	EXPECT_NEAR(on(ActuatorEffectiveness::ControlAxis::YAW, 1), -0.24f, TOL);

	// Side EDFs, servos and the unused columns identical to the disabled case
	for (int col = 2; col < NUM_ACTUATORS; ++col) {
		if (col != BOOSTER) {
			expectColumnsEqual(on, col, off, col, "non-coax");
		}
	}

	// Physical wrench preserved: B_on * u (booster slaved) == B_off * u (booster driven explicitly)
	const float u0 = 0.35f, u1 = 0.25f, g = 2.f;
	const float ub = g * 0.5f * (u0 + u1);

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		const float w_on = on(axis, 0) * u0 + on(axis, 1) * u1;
		const float w_off = off(axis, 0) * u0 + off(axis, 1) * u1 + off(axis, BOOSTER) * ub;
		EXPECT_NEAR(w_on, w_off, TOL) << "axis " << axis;
	}
}

TEST_F(ActuatorEffectivenessIfodroneTest, BoosterSetpointFollowsCoaxMean)
{
	setF("IFO_BST_RATIO", 2.f);
	setI("IFO_BST_ROTOR", BOOSTER);
	ActuatorEffectivenessIfodrone ifo(nullptr);
	computeMatrix(ifo);

	EXPECT_NEAR(boosterSp(ifo, 0.3f, 0.1f), 0.4f, TOL);
	EXPECT_NEAR(boosterSp(ifo, 0.f, 0.f), 0.f, TOL);
	// 2 * 0.7 = 1.4 -> clipped to booster max 1
	EXPECT_NEAR(boosterSp(ifo, 0.8f, 0.6f), 1.f, TOL);
	// Coax clipped before averaging: (1.0 + 0.0) / 2 * 2 = 1.0, not (1.5 + 0.0)
	EXPECT_NEAR(boosterSp(ifo, 1.5f, 0.f), 1.f, TOL);
	// Negative coax clipped to 0: (0.2 + 0) / 2 * 2 = 0.2
	EXPECT_NEAR(boosterSp(ifo, 0.2f, -0.6f), 0.2f, TOL);
	// Non-finite coax setpoint is treated as 0
	EXPECT_NEAR(boosterSp(ifo, NAN, 0.3f), 0.3f, TOL);
}

TEST_F(ActuatorEffectivenessIfodroneTest, BoosterSetpointClipBeforeMeanRatio1)
{
	setF("IFO_BST_RATIO", 1.f);
	setI("IFO_BST_ROTOR", BOOSTER);
	ActuatorEffectivenessIfodrone ifo(nullptr);
	computeMatrix(ifo);

	// mean(clip(1.5)=1, 0.2) = 0.6 (unclipped mean would be 0.85)
	EXPECT_NEAR(boosterSp(ifo, 1.5f, 0.2f), 0.6f, TOL);
	EXPECT_NEAR(boosterSp(ifo, 0.5f, 0.3f), 0.4f, TOL);
}

TEST_F(ActuatorEffectivenessIfodroneTest, PureYawDoesNotChangeBooster)
{
	setF("IFO_BST_RATIO", 2.f);
	setI("IFO_BST_ROTOR", BOOSTER);
	ActuatorEffectivenessIfodrone ifo(nullptr);
	computeMatrix(ifo);

	const float m = 0.4f;
	const float ds[] = {0.f, 0.05f, 0.1f, 0.2f, 0.3f, 0.4f, -0.1f, -0.4f};

	for (float d : ds) {
		EXPECT_NEAR(boosterSp(ifo, m + d, m - d), 2.f * m, TOL) << "d " << d;
	}
}

TEST_F(ActuatorEffectivenessIfodroneTest, InvalidIndexDisablesBooster)
{
	const EffMatrix off = matrixWithBooster(-1);
	const int32_t invalid[] = {0, 1, 3, 9};

	for (int32_t idx : invalid) {
		SCOPED_TRACE(::testing::Message() << "IFO_BST_ROTOR " << idx);
		setI("IFO_BST_ROTOR", idx);
		ActuatorEffectivenessIfodrone ifo(nullptr);
		const EffMatrix m = computeMatrix(ifo);
		expectMatricesEqual(m, off, "invalid index");

		// Booster setpoint passes through untouched
		EXPECT_NEAR(boosterSp(ifo, 0.3f, 0.1f, 0.77f), 0.77f, TOL);
	}
}

TEST_F(ActuatorEffectivenessIfodroneTest, DisabledDoesNotTouchBoosterSetpoint)
{
	setI("IFO_BST_ROTOR", -1);
	ActuatorEffectivenessIfodrone ifo(nullptr);
	computeMatrix(ifo);
	EXPECT_NEAR(boosterSp(ifo, 0.8f, 0.6f, 0.123f), 0.123f, TOL);
}
