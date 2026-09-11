// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "lidar_safety.hpp"

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
