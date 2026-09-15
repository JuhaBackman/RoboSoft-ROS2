// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <vector>

#include "landmark_types.hpp"

namespace robosoft_core
{

/// Extracts small foreground objects from an ordered planar lidar scan.
class LidarObjectDetector
{
public:
  void setThresholds(
    double edge_threshold, double object_size_min, double object_size_max);

  /// Return object centres in the input scan frame.
  std::vector<LandmarkDetection> detect(
    const std::vector<Point2D> & scan) const;

private:
  double edge_threshold_{0.20};
  double object_size_min_squared_{0.02 * 0.02};
  double object_size_max_squared_{0.08 * 0.08};
};

}  // namespace robosoft_core
