// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include <limits>

#include "lidar_cluster.hpp"

TEST(LidarCluster, AssociatesDetectedObjectWithKnownLandmark)
{
  robosoft_core::LidarCluster cluster;
  robosoft_core::ClusterLandmark landmark;
  landmark.center = {2.094, 0.03};
  cluster.setLandmarks({landmark});
  cluster.setPose(0.0, 0.0, 0.0);
  cluster.process({
    {2.0, -0.1}, {1.0, 0.00}, {1.0, 0.03}, {1.0, 0.06}, {2.0, 0.1}});

  const auto active = cluster.active(6);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_NEAR(active.front()->measurement.x, 2.094, 1e-6);
  EXPECT_NEAR(active.front()->measurement.y, 0.03, 1e-6);
}

TEST(LidarCluster, MappingAddsUnknownLandmark)
{
  robosoft_core::LidarCluster cluster;
  cluster.setMapping(true);
  cluster.setPose(10.0, 20.0, 0.0);
  cluster.process({
    {2.0, -0.1}, {1.0, 0.00}, {1.0, 0.03}, {1.0, 0.06}, {2.0, 0.1}});

  ASSERT_EQ(cluster.landmarks().size(), 1U);
  EXPECT_NEAR(cluster.landmarks().front().center.x, 12.094, 1e-6);
  EXPECT_NEAR(cluster.landmarks().front().center.y, 20.03, 1e-6);
}

TEST(LidarCluster, AssociatesPoleSurroundedByNoReturnBeams)
{
  robosoft_core::LidarCluster cluster;
  robosoft_core::ClusterLandmark landmark;
  landmark.center = {6.059, 0.0};
  cluster.setLandmarks({landmark});
  cluster.setPose(0.0, 0.0, 0.0);
  const double missing = std::numeric_limits<double>::quiet_NaN();
  cluster.process({
    {missing, missing}, {4.965, -0.03}, {4.965, 0.0}, {4.965, 0.03},
    {missing, missing}});

  const auto active = cluster.active(6);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_NEAR(active.front()->measurement.x, 6.059, 1e-6);
  EXPECT_NEAR(active.front()->measurement.y, 0.0, 1e-6);
}
