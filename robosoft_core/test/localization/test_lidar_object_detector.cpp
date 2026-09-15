// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <limits>

#include "lidar_object_detector.hpp"

TEST(LidarObjectDetector, DetectsObjectBetweenRangeEdges)
{
  robosoft_core::LidarObjectDetector detector;
  const auto detections = detector.detect({
    {2.0, -0.1}, {1.0, 0.00}, {1.0, 0.03}, {1.0, 0.06}, {2.0, 0.1}});

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_NEAR(detections.front().measurement.x, 1.0, 1e-6);
  EXPECT_NEAR(detections.front().measurement.y, 0.03, 1e-6);
}

TEST(LidarObjectDetector, DetectsObjectBetweenMissingReturns)
{
  robosoft_core::LidarObjectDetector detector;
  const double missing = std::numeric_limits<double>::quiet_NaN();
  const auto detections = detector.detect({
    {missing, missing}, {4.965, -0.03}, {4.965, 0.0}, {4.965, 0.03},
    {missing, missing}});

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_NEAR(detections.front().measurement.x, 4.965, 1e-6);
  EXPECT_NEAR(detections.front().measurement.y, 0.0, 1e-6);
}

TEST(LidarObjectDetector, RejectsSurfaceOutsideConfiguredSize)
{
  robosoft_core::LidarObjectDetector detector;
  const auto detections = detector.detect({
    {2.0, -0.2}, {1.0, 0.0}, {1.0, 0.1}, {2.0, 0.2}});

  EXPECT_TRUE(detections.empty());
}
