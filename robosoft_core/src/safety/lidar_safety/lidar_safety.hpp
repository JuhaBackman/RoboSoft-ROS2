// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robosoft_core
{

struct LidarPoint2D
{
  double x{0.0};
  double y{0.0};
};

struct LidarSafetyParameters
{
  double minimum_forward_distance_m{0.4};
  double forward_corridor_half_width_m{0.4};
  double forward_ignore_distance_m{0.05};
  double side_forward_min_m{-0.28};
  double side_forward_max_m{0.35};
  double side_ignore_distance_m{0.05};
  double speed_ramp_per_s{0.5};
  double maximum_speed_m_s{2.0};
};

struct LidarSafetyResult
{
  double closest_forward_m{std::numeric_limits<double>::infinity()};
  double closest_lateral_m{std::numeric_limits<double>::infinity()};
  double maximum_speed_m_s{0.0};
};

/**
 * Calculate an obstacle speed limit using the configured corridor geometry.
 *
 * Coordinates follow ROS REP-103: x points forward and y to the left.
 * The old lateral speed branch was disabled because it caused false positives;
 * lateral distance is retained for status and future validated use.
 */
inline LidarSafetyResult calculateLidarSafety(
  const std::vector<LidarPoint2D> & points,
  const LidarSafetyParameters & parameters)
{
  LidarSafetyResult result;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    if (point.x > parameters.side_forward_min_m &&
      point.x < parameters.side_forward_max_m &&
      std::abs(point.y) > parameters.side_ignore_distance_m &&
      std::abs(point.y) < std::abs(result.closest_lateral_m))
    {
      result.closest_lateral_m = point.y;
    }

    if (std::abs(point.y) < parameters.forward_corridor_half_width_m &&
      point.x > parameters.forward_ignore_distance_m &&
      point.x < result.closest_forward_m)
    {
      result.closest_forward_m = point.x;
    }
  }

  if (!std::isfinite(result.closest_forward_m)) {
    result.maximum_speed_m_s = parameters.maximum_speed_m_s;
  } else {
    result.maximum_speed_m_s = std::clamp(
      parameters.speed_ramp_per_s *
      (std::abs(result.closest_forward_m) -
      parameters.minimum_forward_distance_m),
      0.0, parameters.maximum_speed_m_s);
  }
  return result;
}

}  // namespace robosoft_core
