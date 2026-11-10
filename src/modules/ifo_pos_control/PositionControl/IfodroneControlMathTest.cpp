#include <gtest/gtest.h>
#include <px4_platform_common/defines.h>

#include "ControlMath.hpp"

using namespace matrix;

TEST(IfodroneControlMath, KeepsFeasibleLevelThrust)
{
	const Vector3f thrust_body = ControlMath::constrainIfodroneBodyThrust(
					     Vector3f(0.1f, -0.2f, -0.5f), Quatf(Eulerf()), 0.3f);

	EXPECT_NEAR(thrust_body(0), 0.1f, 1e-6f);
	EXPECT_NEAR(thrust_body(1), -0.2f, 1e-6f);
	EXPECT_NEAR(thrust_body(2), -0.5f, 1e-6f);
}

TEST(IfodroneControlMath, RejectsPositiveBodyZ)
{
	const Vector3f thrust_body = ControlMath::constrainIfodroneBodyThrust(
					     Vector3f(0.f, 0.f, 0.5f), Quatf(Eulerf()), 0.3f);

	EXPECT_FLOAT_EQ(thrust_body(0), 0.f);
	EXPECT_FLOAT_EQ(thrust_body(1), 0.f);
	EXPECT_NEAR(thrust_body(2), 0.f, 1e-6f);
}

TEST(IfodroneControlMath, RejectsUpwardDemandWhenInverted)
{
	const Quatf inverted(Eulerf(M_PI_F, 0.f, 0.f));
	const Vector3f thrust_body = ControlMath::constrainIfodroneBodyThrust(
					     Vector3f(0.f, 0.f, -0.5f), inverted, 0.3f);

	EXPECT_NEAR(thrust_body(0), 0.f, 1e-6f);
	EXPECT_NEAR(thrust_body(1), 0.f, 1e-6f);
	EXPECT_NEAR(thrust_body(2), 0.f, 1e-6f);
}

TEST(IfodroneControlMath, LimitsAttitudeCoupledSideEdfDemand)
{
	const Quatf rolled(Eulerf(M_PI_2_F, 0.f, 0.f));
	const Vector3f thrust_body = ControlMath::constrainIfodroneBodyThrust(
					     Vector3f(0.f, 0.f, -0.5f), rolled, 0.2f);

	EXPECT_NEAR(thrust_body.xy().norm(), 0.2f, 1e-6f);
	EXPECT_LE(thrust_body(2), 0.f);
}

TEST(IfodroneControlMath, ReplacesNonFiniteComponentsWithZero)
{
	const Vector3f thrust_body = ControlMath::constrainIfodroneBodyThrust(
					     Vector3f(NAN, 0.1f, -0.5f), Quatf(Eulerf()), 0.3f);

	EXPECT_FLOAT_EQ(thrust_body(0), 0.f);
	EXPECT_NEAR(thrust_body(1), 0.1f, 1e-6f);
	EXPECT_NEAR(thrust_body(2), -0.5f, 1e-6f);
}

#include "PositionControl.hpp"

namespace
{
PositionControl makeController(float xy_limit)
{
	PositionControl control;
	control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	control.setVelocityGains(Vector3f(1.2f, 1.2f, 1.f), Vector3f(0.4f, 0.4f, 0.2f), Vector3f());
	control.setVelocityLimits(5.f, 3.f, 1.5f);
	control.setThrustLimits(0.12f, 0.8f);
	control.setHorizontalThrustMargin(0.2f);
	control.setHoverThrust(0.5f);
	control.setTiltLimit(M_PI_2_F);
	control.setHorizontalThrustLimit(xy_limit);

	PositionControlStates states{};
	states.position = Vector3f(0.f, 0.f, -2.f);
	states.velocity = Vector3f();
	states.acceleration = Vector3f();
	states.yaw = 0.f;
	control.setState(states);
	return control;
}

trajectory_setpoint_s velocitySetpoint(float vx)
{
	trajectory_setpoint_s sp = PositionControl::empty_trajectory_setpoint;
	sp.velocity[0] = vx;
	sp.velocity[1] = 0.f;
	sp.velocity[2] = 0.f;
	sp.yaw = 0.f;
	return sp;
}
} // namespace

TEST(IfodronePositionControl, HorizontalThrustLimitedInsideLibrary)
{
	PositionControl control = makeController(0.2f);
	control.setInputSetpoint(velocitySetpoint(5.f));
	ASSERT_TRUE(control.update(0.01f));

	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_LE(Vector2f(out.thrust[0], out.thrust[1]).norm(), 0.2f + 1e-5f);
	EXPECT_GT(out.thrust[0], 0.19f);
}

TEST(IfodronePositionControl, NoHorizontalWindupAgainstSideEdfLimit)
{
	// Saturated for 10 s against the 0.2 side-EDF limit, then the velocity error
	// reverses: the commanded thrust must reverse promptly (no wound-up integral).
	PositionControl control = makeController(0.2f);

	for (int i = 0; i < 1000; ++i) {
		control.setInputSetpoint(velocitySetpoint(5.f));
		ASSERT_TRUE(control.update(0.01f));
	}

	for (int i = 0; i < 50; ++i) { // 0.5 s
		control.setInputSetpoint(velocitySetpoint(-2.f));
		ASSERT_TRUE(control.update(0.01f));
	}

	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_LT(out.thrust[0], 0.f);
}
