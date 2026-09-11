// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <robosoft_core/navigation/path_tracking_simple_controller.hpp>

TEST(PathTrackingSimpleController, ConvertsControllerCurvatureToRosUnits)
{
  robosoft_core::PathTrackingInput input;
  input.path_curvature_km = 100.0;
  input.lookahead_curvature_km = 100.0;
  input.target_speed_ms = 1.0;
  input.measured_speed_ms = 1.0;
  const auto output = robosoft_core::calculateSimplePathTracking(
    input, -500.0, -5.0, 0.3, 0.3, 2.0, 8.031, 0.5);
  EXPECT_DOUBLE_EQ(output.speed_ms, 1.0);
  EXPECT_NEAR(output.curvature_m_inv, 0.1, 1e-9);
}

TEST(PathTrackingSimpleController, AppliesConfiguredSpeedRamp)
{
  robosoft_core::PathTrackingInput input;
  input.target_speed_ms = 1.5;
  input.measured_speed_ms = 0.4;
  const auto output = robosoft_core::calculateSimplePathTracking(
    input, -500.0, -5.0, 0.3, 0.3, 2.0, 8.031, 0.5);
  EXPECT_NEAR(output.speed_ms, 0.7, 1e-9);
}

TEST(PathTrackingSimpleController, UsesCrawlSpeedOutsideTenCentimetres)
{
  robosoft_core::PathTrackingInput input;
  input.cross_track_error_m = 0.2;
  input.target_speed_ms = 1.5;
  input.measured_speed_ms = 1.0;
  const auto output = robosoft_core::calculateSimplePathTracking(
    input, -500.0, -5.0, 0.3, 0.3, 2.0, 8.031, 0.5);
  EXPECT_DOUBLE_EQ(output.speed_ms, 0.5);
}

TEST(PathTrackingSimpleController, BlendsCurrentAndLookaheadCurvature)
{
  robosoft_core::PathTrackingInput input;
  input.path_curvature_km = 100.0;
  input.lookahead_curvature_km = 300.0;
  input.target_speed_ms = 1.0;
  input.measured_speed_ms = 1.0;
  const auto output = robosoft_core::calculateSimplePathTracking(
    input, 0.0, 0.0, 0.3, 0.3, 2.0, 8.031, 0.5);
  EXPECT_NEAR(output.curvature_m_inv, 0.2, 1e-9);
}
