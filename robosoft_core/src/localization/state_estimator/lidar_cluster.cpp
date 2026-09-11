// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "lidar_cluster.hpp"

#include <limits>

namespace robosoft_core
{

void LidarCluster::acceptObject(const ClusterPoint & center)
{
  const auto rotated = rotate(center, lidar_angle_error_);
  const ClusterPoint raw{lidar_x_offset_ + rotated.x, rotated.y};
  const auto global_offset = rotate(raw, yaw_);
  const ClusterPoint global{
    position_.x + global_offset.x, position_.y + global_offset.y};

  ClusterLandmark * closest = nullptr;
  double closest_distance = association_distance_squared_;
  for (auto & cluster : clusters_) {
    const double distance = distanceSquared(cluster.center, global);
    if (distance < closest_distance) {
      closest = &cluster;
      closest_distance = distance;
    }
  }

  if (closest == nullptr) {
    if (mapping_) {
      ClusterLandmark landmark;
      landmark.center = global;
      clusters_.push_back(landmark);
    }
    return;
  }
  if (!closest->measured) {
    active_.push_back(closest);
  } else if (
    distanceSquared(closest->center, closest->measurement) <=
    distanceSquared(closest->center, global))
  {
    return;
  }
  closest->measurement = raw;
  closest->measured = true;
}

void LidarCluster::process(const std::vector<ClusterPoint> & scan)
{
  if (scan.size() < 2U) {
    return;
  }

  // Divide the ordered scan into continuous surfaces. This handles both
  // range jumps against a background and isolated returns surrounded by the
  // NaN/zero no-return samples produced by common LaserScan-to-cloud paths.
  ClusterPoint start{};
  ClusterPoint previous_point{};
  ClusterPoint center_sum{};
  std::size_t point_count = 0;

  const auto finish_surface = [this, &start, &previous_point, &center_sum,
      &point_count]() {
      if (point_count >= 2U) {
        const double size = distanceSquared(start, previous_point);
        if (size > object_size_min_squared_ &&
          size < object_size_max_squared_)
        {
          acceptObject(
            {center_sum.x / static_cast<double>(point_count),
              center_sum.y / static_cast<double>(point_count)});
        }
      }
      point_count = 0;
      center_sum = {};
    };

  for (const auto & point : scan) {
    const double range = std::hypot(point.x, point.y);
    const bool valid =
      std::isfinite(point.x) && std::isfinite(point.y) && range > 1.0e-6;
    if (!valid) {
      finish_surface();
      continue;
    }

    if (point_count > 0U &&
      distanceSquared(previous_point, point) >
      edge_threshold_ * edge_threshold_)
    {
      finish_surface();
    }

    if (point_count == 0U) {
      start = point;
    }
    center_sum.x += point.x;
    center_sum.y += point.y;
    ++point_count;
    previous_point = point;
  }
  finish_surface();
}

}  // namespace robosoft_core
