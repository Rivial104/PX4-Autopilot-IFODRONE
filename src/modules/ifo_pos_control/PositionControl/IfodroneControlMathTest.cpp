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
