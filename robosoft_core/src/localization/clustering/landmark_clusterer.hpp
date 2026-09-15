// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include "landmark_types.hpp"

namespace robosoft_core
{

/// Associates vehicle-frame object detections with persistent map landmarks.
class LandmarkClusterer
{
public:
  void setPose(double x, double y, double yaw);
  void setMapping(bool enabled) {mapping_ = enabled;}
  void setAssociationDistance(double maximum_distance);
  void setLandmarks(std::vector<Landmark> landmarks);
  bool updateLandmark(
    std::uint32_t id, const Point2D & center,
    const std::array<double, 4> & covariance);

  void process(const std::vector<LandmarkDetection> & detections);
  std::vector<Landmark *> active(std::size_t maximum);
  const std::deque<Landmark> & landmarks() const {return landmarks_;}

private:
  void associate(const LandmarkDetection & detection);

  double association_distance_squared_{0.5 * 0.5};
  bool mapping_{false};
  Point2D position_;
  double yaw_{0.0};
  // Mapping may append landmarks while active_ holds pointers to existing
  // entries. deque preserves those addresses when elements are appended.
  std::deque<Landmark> landmarks_;
  std::vector<Landmark *> active_;
  std::uint32_t next_landmark_id_{0U};
};

}  // namespace robosoft_core
