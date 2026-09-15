// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "lidar_safety.hpp"
#include "point_transform.hpp"

#include <cmath>
#include <limits>
#include <vector>

using robosoft_core::LidarPoint2D;
using robosoft_core::LidarSafetyParameters;
using robosoft_core::calculateLidarSafety;

TEST(LidarSafety, AllowsConfiguredSpeedWhenCorridorIsClear)
{
  LidarSafetyParameters parameters;
  const std::vector<LidarPoint2D> points = {
    {1.0, 1.0}, {-1.0, 0.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0}};
  const auto result = calculateLidarSafety(points, parameters);
  EXPECT_DOUBLE_EQ(result.maximum_speed_m_s, 2.0);
  EXPECT_TRUE(std::isinf(result.closest_forward_m));
}

TEST(LidarSafety, StopsInsideMinimumForwardDistance)
{
  LidarSafetyParameters parameters;
  const auto result = calculateLidarSafety({{0.3, 0.1}}, parameters);
  EXPECT_DOUBLE_EQ(result.closest_forward_m, 0.3);
  EXPECT_DOUBLE_EQ(result.maximum_speed_m_s, 0.0);
}

TEST(LidarSafety, AppliesForwardSpeedRamp)
{
  LidarSafetyParameters parameters;
  const auto result = calculateLidarSafety({{1.4, -0.2}}, parameters);
  EXPECT_NEAR(result.maximum_speed_m_s, 0.5, 1e-12);
}

TEST(LidarSafety, IgnoresPointsOutsideForwardCorridor)
{
  LidarSafetyParameters parameters;
  const auto result = calculateLidarSafety({{0.2, 0.6}}, parameters);
  EXPECT_DOUBLE_EQ(result.maximum_speed_m_s, 2.0);
}

TEST(LidarSafety, ReportsClosestLateralObstacle)
{
  LidarSafetyParameters parameters;
  const auto result =
    calculateLidarSafety({{0.0, 0.3}, {0.1, -0.1}}, parameters);
  EXPECT_DOUBLE_EQ(result.closest_lateral_m, -0.1);
}

TEST(LidarSafety, VehicleOriginLimitsPreserveSensorRelativeClearance)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.094;
  transform.rotation.w = 1.0;
  const auto point = robosoft_core::transformPoint(
    {0.3, 0.1, 0.0}, transform);

  LidarSafetyParameters parameters;
  parameters.minimum_forward_distance_m = 1.494;
  parameters.forward_ignore_distance_m = 1.144;
  parameters.side_forward_min_m = 0.814;
  parameters.side_forward_max_m = 1.444;
  const auto result = calculateLidarSafety({{point.x, point.y}}, parameters);

  EXPECT_NEAR(result.closest_forward_m, 1.394, 1e-12);
  EXPECT_DOUBLE_EQ(result.maximum_speed_m_s, 0.0);

  const auto ramp_point = robosoft_core::transformPoint(
    {1.4, -0.2, 0.0}, transform);
  const auto ramp = calculateLidarSafety(
    {{ramp_point.x, ramp_point.y}}, parameters);
  EXPECT_NEAR(ramp.maximum_speed_m_s, 0.5, 1e-12);

  const auto ignored_point = robosoft_core::transformPoint(
    {0.03, 0.0, 0.0}, transform);
  const auto ignored = calculateLidarSafety(
    {{ignored_point.x, ignored_point.y}}, parameters);
  EXPECT_DOUBLE_EQ(ignored.maximum_speed_m_s, 2.0);

  const auto side_point = robosoft_core::transformPoint(
    {0.0, 0.3, 0.0}, transform);
  const auto side = calculateLidarSafety(
    {{side_point.x, side_point.y}}, parameters);
  EXPECT_NEAR(side.closest_lateral_m, 0.3, 1e-12);
}

TEST(LidarSafety, PointTransformAppliesYawAndTranslation)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0;
  transform.translation.y = 2.0;
  transform.rotation.z = std::sin(M_PI / 4.0);
  transform.rotation.w = std::cos(M_PI / 4.0);

  const auto point = robosoft_core::transformPoint(
    {2.0, 0.0, 0.0}, transform);
  EXPECT_NEAR(point.x, 1.0, 1e-12);
  EXPECT_NEAR(point.y, 4.0, 1e-12);
  EXPECT_NEAR(point.z, 0.0, 1e-12);
}
