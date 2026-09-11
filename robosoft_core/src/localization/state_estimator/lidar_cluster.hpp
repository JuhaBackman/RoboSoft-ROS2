// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace robosoft_core
{

struct ClusterPoint
{
  double x{0.0};
  double y{0.0};
};

struct ClusterLandmark
{
  ClusterPoint center;
  ClusterPoint measurement;
  std::array<double, 4> covariance{{0.1, 0.0, 0.0, 0.1}};
  bool measured{false};
};

/// ROS-independent range-discontinuity clustering.
class LidarCluster
{
public:
  LidarCluster() {clusters_.reserve(4096);}
  void setPose(double x, double y, double yaw)
  {
    position_ = {x, y};
    yaw_ = yaw;
    active_.clear();
    for (auto & cluster : clusters_) {
      cluster.measured = false;
    }
  }

  void setMapping(bool enabled) {mapping_ = enabled;}
  void setSensorPose(double x_offset, double angle_error)
  {
    lidar_x_offset_ = x_offset;
    lidar_angle_error_ = angle_error;
  }
  void setLandmarks(std::vector<ClusterLandmark> landmarks)
  {
    clusters_ = std::move(landmarks);
    active_.clear();
  }
  const std::vector<ClusterLandmark> & landmarks() const {return clusters_;}

  std::vector<ClusterLandmark *> active(std::size_t maximum)
  {
    auto result = active_;
    std::sort(
      result.begin(), result.end(), [this](const auto * a, const auto * b) {
        return distanceSquared(a->center, position_) <
               distanceSquared(b->center, position_);
      });
    if (result.size() > maximum) {
      result.resize(maximum);
    }
    return result;
  }

  void process(const std::vector<ClusterPoint> & scan);

private:
  static double distanceSquared(const ClusterPoint & a, const ClusterPoint & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
  }
  static ClusterPoint rotate(const ClusterPoint & point, double angle)
  {
    return {
      point.x * std::cos(angle) - point.y * std::sin(angle),
      point.x * std::sin(angle) + point.y * std::cos(angle)};
  }
  void acceptObject(const ClusterPoint & center);

  double edge_threshold_{0.20};
  double object_size_min_squared_{0.02 * 0.02};
  double object_size_max_squared_{0.08 * 0.08};
  double association_distance_squared_{0.5 * 0.5};
  double lidar_x_offset_{1.094};
  double lidar_angle_error_{0.0};
  bool mapping_{false};
  ClusterPoint position_;
  double yaw_{0.0};
  std::vector<ClusterLandmark> clusters_;
  std::vector<ClusterLandmark *> active_;
};

}  // namespace robosoft_core
