// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "lidar_object_detector.hpp"

#include <cmath>
#include <stdexcept>

namespace robosoft_core
{

void LidarObjectDetector::setThresholds(
  double edge_threshold, double object_size_min, double object_size_max)
{
  if (edge_threshold <= 0.0 || object_size_min < 0.0 ||
    object_size_max <= object_size_min)
  {
    throw std::invalid_argument("Invalid lidar object-detection thresholds");
  }
  edge_threshold_ = edge_threshold;
  object_size_min_squared_ = object_size_min * object_size_min;
  object_size_max_squared_ = object_size_max * object_size_max;
}

std::vector<LandmarkDetection> LidarObjectDetector::detect(
  const std::vector<Point2D> & scan) const
{
  std::vector<LandmarkDetection> detections;
  Point2D start{};
  Point2D last{};
  Point2D center_sum{};
  std::size_t point_count = 0;
  double previous_range = 0.0;
  bool previous_valid = false;
  bool foreground_active = false;

  const auto begin_object = [&start, &last, &center_sum, &point_count,
      &foreground_active](const Point2D & point) {
      start = point;
      last = point;
      center_sum = point;
      point_count = 1;
      foreground_active = true;
    };

  const auto finish_object = [this, &detections, &start, &last, &center_sum,
      &point_count, &foreground_active]() {
      if (foreground_active && point_count >= 2U) {
        const double object_size = distanceSquared(start, last);
        if (object_size > object_size_min_squared_ &&
          object_size < object_size_max_squared_)
        {
          const Point2D sensor_center{
            center_sum.x / static_cast<double>(point_count),
            center_sum.y / static_cast<double>(point_count)};
          detections.push_back(LandmarkDetection{{sensor_center.x, sensor_center.y}});
        }
      }
      point_count = 0;
      center_sum = {};
      foreground_active = false;
    };

  for (const auto & point : scan) {
    const double range = std::hypot(point.x, point.y);
    const bool valid =
      std::isfinite(point.x) && std::isfinite(point.y) && range > 1.0e-6;
    if (!valid) {
      finish_object();
      previous_valid = false;
      continue;
    }

    if (!previous_valid) {
      // A return following a missing beam can be an isolated foreground
      // surface, so use the no-return boundary as its falling edge.
      begin_object(point);
    } else if (range < previous_range - edge_threshold_) {
      // Falling range edge: the beam has entered a foreground object.
      finish_object();
      begin_object(point);
    } else if (range > previous_range + edge_threshold_) {
      // Rising range edge: the beam has left the foreground object. The
      // current point belongs to the background and is not accumulated.
      finish_object();
    } else if (foreground_active) {
      center_sum.x += point.x;
      center_sum.y += point.y;
      ++point_count;
      last = point;
    }

    previous_range = range;
    previous_valid = true;
  }
  finish_object();
  return detections;
}

}  // namespace robosoft_core
