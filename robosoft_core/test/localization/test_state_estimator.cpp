// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>

#include <gtest/gtest.h>

#include "state_estimator.hpp"

TEST(StateEstimator, InitializesAndProducesFiniteEstimate)
{
  robosoft_core::StateEstimator estimator;
  estimator.setPositionMeasurement(10.0, 20.0, 0.25, true);
  estimator.setControls(0.0, 0.0);

  ASSERT_TRUE(estimator.update({}));
  EXPECT_TRUE(estimator.initialized());
  EXPECT_TRUE(std::isfinite(estimator.x()));
  EXPECT_TRUE(std::isfinite(estimator.y()));
  EXPECT_TRUE(std::isfinite(estimator.yaw()));
}
