// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "landmark_clusterer.hpp"

TEST(LandmarkClusterer, AssociatesDetectionWithKnownLandmark)
{
  robosoft_core::LandmarkClusterer clusterer;
  robosoft_core::Landmark landmark;
  landmark.center = {2.094, 0.03};
  clusterer.setLandmarks({landmark});
  clusterer.setPose(0.0, 0.0, 0.0);
  clusterer.process({robosoft_core::LandmarkDetection{{2.094, 0.03}}});

  const auto active = clusterer.active(6);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_NEAR(active.front()->measurement.x, 2.094, 1e-6);
  EXPECT_NEAR(active.front()->measurement.y, 0.03, 1e-6);
}

TEST(LandmarkClusterer, MappingAddsUnknownLandmark)
{
  robosoft_core::LandmarkClusterer clusterer;
  clusterer.setMapping(true);
  clusterer.setPose(10.0, 20.0, 0.0);
  clusterer.process({robosoft_core::LandmarkDetection{{2.094, 0.03}}});

  ASSERT_EQ(clusterer.landmarks().size(), 1U);
  EXPECT_NEAR(clusterer.landmarks().front().center.x, 12.094, 1e-6);
  EXPECT_NEAR(clusterer.landmarks().front().center.y, 20.03, 1e-6);
}

TEST(LandmarkClusterer, KeepsClosestDuplicateDetection)
{
  robosoft_core::LandmarkClusterer clusterer;
  robosoft_core::Landmark landmark;
  landmark.center = {11.0, 20.0};
  clusterer.setLandmarks({landmark});
  clusterer.setPose(10.0, 20.0, 0.0);
  clusterer.process({
    robosoft_core::LandmarkDetection{{1.2, 0.0}},
    robosoft_core::LandmarkDetection{{1.05, 0.0}}});

  const auto active = clusterer.active(6);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_NEAR(active.front()->measurement.x, 1.05, 1e-6);
}

TEST(LandmarkClusterer, PreservesIdentityAcrossEstimatorFeedback)
{
  robosoft_core::LandmarkClusterer clusterer;
  robosoft_core::Landmark first;
  first.center = {1.0, 2.0};
  robosoft_core::Landmark second;
  second.center = {3.0, 4.0};
  clusterer.setLandmarks({first, second});

  const auto id = clusterer.landmarks().back().id;
  EXPECT_TRUE(clusterer.updateLandmark(
      id, {3.2, 4.1}, {0.01, 0.002, 0.002, 0.02}));
  EXPECT_NEAR(clusterer.landmarks().back().center.x, 3.2, 1e-9);
  EXPECT_NEAR(clusterer.landmarks().back().center.y, 4.1, 1e-9);
  EXPECT_NEAR(clusterer.landmarks().back().covariance[0], 0.01, 1e-9);
  EXPECT_NEAR(clusterer.landmarks().back().covariance[3], 0.02, 1e-9);
  EXPECT_FALSE(clusterer.updateLandmark(
      id + 100U, {0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}));
}

TEST(LandmarkClusterer, TransfersDetectionMetadata)
{
  robosoft_core::LandmarkClusterer clusterer;
  robosoft_core::Landmark landmark;
  landmark.center = {1.0, 0.0};
  clusterer.setLandmarks({landmark});
  clusterer.setPose(0.0, 0.0, 0.0);
  robosoft_core::LandmarkDetection detection;
  detection.measurement = {1.0, 0.0};
  detection.covariance = {0.03, 0.001, 0.001, 0.04};
  detection.object_type = "tree";
  detection.confidence = 0.8F;
  clusterer.process({detection});

  const auto active = clusterer.active(1);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_EQ(active.front()->object_type, "tree");
  EXPECT_FLOAT_EQ(active.front()->confidence, 0.8F);
  EXPECT_NEAR(active.front()->measurement_covariance[0], 0.03, 1e-9);
  EXPECT_NEAR(active.front()->measurement_covariance[3], 0.04, 1e-9);
}
