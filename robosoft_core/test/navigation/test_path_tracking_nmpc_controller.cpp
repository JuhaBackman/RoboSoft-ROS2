// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <robosoft_core/navigation/path_tracking_nmpc_controller.hpp>

#include <cmath>

namespace
{
std::vector<robosoft_core::PathTrackingNmpcReference> straightRoute(double speed = 1.0)
{
  return {{0.0, 0.0, 0.0, speed, 0.0}, {20.0, 0.0, 0.0, speed, 0.0}};
}
}  // namespace

TEST(PathTrackingNmpcController, ProducesFiniteStraightRouteCommand)
{
  robosoft_core::PathTrackingNmpcController controller;
  robosoft_core::PathTrackingNmpcInput input;
  input.route = straightRoute();
  input.measured_speed_m_s = 0.5;
  const auto output = controller.calculate(input);

  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(std::isfinite(output.speed_m_s));
  EXPECT_TRUE(std::isfinite(output.curvature_m_inv));
  EXPECT_GE(output.speed_m_s, -1.2);
  EXPECT_LE(output.speed_m_s, 2.3);
  EXPECT_GE(output.curvature_m_inv, -0.6);
  EXPECT_LE(output.curvature_m_inv, 0.6);
}

TEST(PathTrackingNmpcController, AppliesRuntimeVehicleLimits)
{
  robosoft_core::PathTrackingNmpcController controller;
  robosoft_core::PathTrackingNmpcParameters parameters;
  parameters.minimum_speed_m_s = -0.4;
  parameters.maximum_speed_m_s = 0.7;
  parameters.minimum_curvature_m_inv = -0.2;
  parameters.maximum_curvature_m_inv = 0.2;
  controller.configure(parameters);

  robosoft_core::PathTrackingNmpcInput input;
  input.y_m = 1.0;
  input.route = straightRoute(2.0);
  input.measured_speed_m_s = 0.5;
  const auto output = controller.calculate(input);

  ASSERT_TRUE(output.valid);
  EXPECT_GE(output.speed_m_s, parameters.minimum_speed_m_s);
  EXPECT_LE(output.speed_m_s, parameters.maximum_speed_m_s);
  EXPECT_GE(output.curvature_m_inv, parameters.minimum_curvature_m_inv);
  EXPECT_LE(output.curvature_m_inv, parameters.maximum_curvature_m_inv);
}

TEST(PathTrackingNmpcController, CorrectsOppositeLateralErrorsInOppositeDirections)
{
  robosoft_core::PathTrackingNmpcController left_controller;
  robosoft_core::PathTrackingNmpcController right_controller;
  robosoft_core::PathTrackingNmpcInput input;
  input.route = straightRoute();
  input.measured_speed_m_s = 1.0;
  input.y_m = 0.5;
  const auto positive_error = left_controller.calculate(input);
  input.y_m = -0.5;
  const auto negative_error = right_controller.calculate(input);

  ASSERT_TRUE(positive_error.valid);
  ASSERT_TRUE(negative_error.valid);
  EXPECT_LT(
    positive_error.curvature_m_inv * negative_error.curvature_m_inv, 0.0);
}

TEST(PathTrackingNmpcController, AppliesCommandRateLimitsToFirstCommand)
{
  robosoft_core::PathTrackingNmpcController controller;
  robosoft_core::PathTrackingNmpcInput input;
  input.route = straightRoute(2.0);
  const auto output = controller.calculate(input);

  ASSERT_TRUE(output.valid);
  // The generated horizon step is 0.1 s and dV is limited to 5.6 m/s^2.
  EXPECT_LE(output.speed_m_s, 0.56 + 1e-6);
  EXPECT_GE(output.speed_m_s, -0.52 - 1e-6);
}

TEST(PathTrackingNmpcController, MaintainsMinimumSpeedForNonzeroTaskSpeed)
{
  robosoft_core::PathTrackingNmpcParameters parameters;
  parameters.minimum_tracking_speed_m_s = 0.3;

  robosoft_core::PathTrackingNmpcController forward_controller;
  forward_controller.configure(parameters);
  robosoft_core::PathTrackingNmpcInput input;
  input.route = straightRoute(0.1);
  const auto forward = forward_controller.calculate(input);
  ASSERT_TRUE(forward.valid);
  EXPECT_GE(forward.speed_m_s, 0.1 - 1e-6);

  robosoft_core::PathTrackingNmpcController reverse_controller;
  reverse_controller.configure(parameters);
  input.route = straightRoute(-0.1);
  const auto reverse = reverse_controller.calculate(input);
  ASSERT_TRUE(reverse.valid);
  EXPECT_LE(reverse.speed_m_s, -0.1 + 1e-6);
}

TEST(PathTrackingNmpcController, StartsFromRestOnPackagedFieldRoute)
{
  robosoft_core::PathTrackingNmpcController controller;
  robosoft_core::PathTrackingNmpcParameters parameters;
  parameters.reference_update_iterations = 2;
  parameters.solver_iterations = 4;
  controller.configure(parameters);

  robosoft_core::PathTrackingNmpcInput input;
  input.yaw_rad = 2.188234692820414;
  input.route = {
    {0.0, 0.0, 8.471420, 0.5, 0.0435},
    {-0.600489909, 0.813901831, 8.515368, 0.5, 0.0},
    {-1.183674273, 1.604347299, 8.515368, 0.5, 0.0},
    {-1.753300349, 2.376415458, 8.515368, 0.5, 0.0},
    {-2.337830301, 3.168683772, 8.515368, 0.5, 0.0}};

  const auto output = controller.calculate(input);

  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.speed_m_s, 0.0);
}

TEST(PathTrackingNmpcController, RejectsMissingRouteHorizon)
{
  robosoft_core::PathTrackingNmpcController controller;
  const auto output = controller.calculate({});
  EXPECT_FALSE(output.valid);
}

TEST(PathTrackingNmpcController, DoesNotJumpToLaterBranchAtCrossing)
{
  robosoft_core::PathTrackingNmpcController controller;
  robosoft_core::PathTrackingNmpcInput input;
  input.x_m = 0.0;
  input.y_m = 0.2;
  input.yaw_rad = 0.0;
  input.measured_speed_m_s = 0.5;
  input.start_segment_index = 0;
  input.route = {
    {-10.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0},
    {10.0, 0.0, M_PI, 1.0, 0.0},
    {0.0, 0.0, M_PI_2, 0.0, 0.0},
    {0.0, 10.0, M_PI_2, 0.0, 0.0}};

  const auto output = controller.calculate(input);

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.reference_segment_index, 0U);
  EXPECT_GT(output.speed_m_s, 0.0);
}
