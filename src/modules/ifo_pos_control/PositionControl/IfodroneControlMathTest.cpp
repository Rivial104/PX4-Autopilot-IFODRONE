#include <gtest/gtest.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>

#include "ControlMath.hpp"
#include "PositionControl.hpp"

using namespace matrix;
using namespace ifodrone;

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

namespace
{
PositionControl makeController(float xy_limit)
{
	PositionControl control;
	control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
	control.setVelocityGains(Vector3f(1.2f, 1.2f, 1.f), Vector3f(0.4f, 0.4f, 0.2f), Vector3f());
	control.setVelocityLimits(5.f, 3.f, 1.5f);
	control.setThrustLimits(0.12f, 0.8f);
	control.setHoverThrust(0.5f);
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

TEST(IfodronePositionControl, HorizontalAccelerationIndependentOfClimb)
{
	for (float az : {-2.f, 0.f, 2.f}) {
		PositionControl control = makeController(0.2f);
		auto sp = PositionControl::empty_trajectory_setpoint;
		Vector3f(1.f, -0.5f, az).copyTo(sp.acceleration);
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
		vehicle_local_position_setpoint_s out{};
		control.getLocalPositionSetpoint(out);
		EXPECT_NEAR(out.thrust[0], 0.5f / CONSTANTS_ONE_G, 1e-6f);
		EXPECT_NEAR(out.thrust[1], -0.25f / CONSTANTS_ONE_G, 1e-6f);
	}
}

TEST(IfodronePositionControl, GroundDemandProducesZeroThrust)
{
	PositionControl control = makeController(0.2f);
	control.setThrustLimits(0.f, 0.8f);
	auto sp = PositionControl::empty_trajectory_setpoint;
	Vector3f(0.f, 0.f, 100.f).copyTo(sp.acceleration);
	control.setInputSetpoint(sp);
	ASSERT_TRUE(control.update(0.01f));
	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_FLOAT_EQ(Vector3f(out.thrust).norm(), 0.f);
}

TEST(IfodronePositionControl, HorizontalLimitCanBeZeroOrExceedTotal)
{
	for (float xy_limit : {0.f, 1.f}) {
		PositionControl control = makeController(xy_limit);
		auto sp = PositionControl::empty_trajectory_setpoint;
		Vector3f(30.f, 0.f, -10.f).copyTo(sp.acceleration);
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
		vehicle_local_position_setpoint_s out{};
		control.getLocalPositionSetpoint(out);
		EXPECT_LE(Vector3f(out.thrust).norm(), 0.8f + 1e-6f);
		EXPECT_LE(Vector2f(out.thrust).norm(), xy_limit + 1e-6f);
	}
}

TEST(IfodronePositionControl, ZeroProportionalGainRemainsFinite)
{
	PositionControl control = makeController(0.2f);
	control.setVelocityGains(Vector3f(), Vector3f(), Vector3f());
	auto sp = velocitySetpoint(0.f);
	sp.acceleration[0] = 10.f;
	sp.acceleration[1] = 0.f;

	for (int i = 0; i < 100; ++i) {
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
	}

	sp.acceleration[0] = 0.f;
	control.setInputSetpoint(sp);
	ASSERT_TRUE(control.update(0.01f));
	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_NEAR(out.thrust[0], 0.f, 1e-6f);
}

TEST(IfodronePositionControl, VerticalIntegralRespectsHorizontalReserve)
{
	PositionControl control = makeController(0.2f);
	auto sp = velocitySetpoint(0.f);
	sp.velocity[2] = -2.f;
	Vector3f(4.f, 0.f, -3.5f).copyTo(sp.acceleration);

	for (int i = 0; i < 1000; ++i) {
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
	}

	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_NEAR(out.acceleration[2], -5.5f, 1e-5f);
}

TEST(IfodronePositionControl, PositionFeedbackRequiresVelocityEstimate)
{
	PositionControl control = makeController(0.2f);
	PositionControlStates states{};
	states.velocity(0) = NAN;
	control.setState(states);
	auto sp = PositionControl::empty_trajectory_setpoint;
	Vector3f().copyTo(sp.position);
	Vector3f().copyTo(sp.acceleration);
	control.setInputSetpoint(sp);
	EXPECT_FALSE(control.update(0.01f));

	Vector3f(NAN, NAN, NAN).copyTo(sp.position);
	control.setInputSetpoint(sp);
	EXPECT_TRUE(control.update(0.01f));
}

TEST(IfodronePositionControl, ReachesPointWithConstantDisturbance)
{
	PositionControl control = makeController(0.2f);
	PositionControlStates states{};
	const Vector3f target(2.f, -1.f, -3.f);
	auto sp = PositionControl::empty_trajectory_setpoint;
	target.copyTo(sp.position);

	for (int i = 0; i < 3000; ++i) {
		control.setState(states);
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
		vehicle_local_position_setpoint_s out{};
		control.getLocalPositionSetpoint(out);
		states.acceleration = Vector3f(out.thrust) * (CONSTANTS_ONE_G / 0.5f)
				      + Vector3f(0.2f, -0.1f, CONSTANTS_ONE_G);
		states.velocity += states.acceleration * 0.01f;
		states.position += states.velocity * 0.01f;
	}

	EXPECT_LT((states.position - target).norm(), 0.03f);
	EXPECT_LT(states.velocity.norm(), 0.03f);
}

TEST(IfodronePositionControl, TracksThreeDimensionalTrajectory)
{
	PositionControl control = makeController(0.2f);
	PositionControlStates states{};
	states.position = Vector3f(1.f, 0.f, -2.f);
	states.velocity = Vector3f(0.f, 0.4f, 0.2f);

	for (int i = 0; i < 2000; ++i) {
		const float phase = i * 0.004f;
		const Vector3f position(cosf(phase), sinf(phase), -2.f + 0.5f * sinf(phase));
		const Vector3f velocity(-0.4f * sinf(phase), 0.4f * cosf(phase), 0.2f * cosf(phase));
		const Vector3f acceleration(-0.16f * cosf(phase), -0.16f * sinf(phase), -0.08f * sinf(phase));
		auto sp = PositionControl::empty_trajectory_setpoint;
		position.copyTo(sp.position);
		velocity.copyTo(sp.velocity);
		acceleration.copyTo(sp.acceleration);
		control.setState(states);
		control.setInputSetpoint(sp);
		ASSERT_TRUE(control.update(0.01f));
		vehicle_local_position_setpoint_s out{};
		control.getLocalPositionSetpoint(out);
		states.acceleration = Vector3f(out.thrust) * (CONSTANTS_ONE_G / 0.5f) + Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
		states.velocity += states.acceleration * 0.01f;
		states.position += states.velocity * 0.01f;
		EXPECT_LT((states.position - position).norm(), 0.02f);
	}
}

TEST(IfodronePositionControl, IntegralOnlyControlDoesNotWindUp)
{
	PositionControl control = makeController(0.2f);
	control.setVelocityGains(Vector3f(), Vector3f(0.4f, 0.4f, 0.2f), Vector3f());

	for (int i = 0; i < 1000; ++i) {
		control.setInputSetpoint(velocitySetpoint(5.f));
		ASSERT_TRUE(control.update(0.01f));
	}

	vehicle_local_position_setpoint_s out{};
	control.getLocalPositionSetpoint(out);
	EXPECT_LE(out.acceleration[0], 0.2f * CONSTANTS_ONE_G / 0.5f + 0.021f);
	EXPECT_NEAR(out.thrust[0], 0.2f, 1e-5f);
}
