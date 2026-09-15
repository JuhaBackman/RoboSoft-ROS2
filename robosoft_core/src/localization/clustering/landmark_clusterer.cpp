// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "landmark_clusterer.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robosoft_core
{

void LandmarkClusterer::setPose(double x, double y, double yaw)
{
  position_ = {x, y};
  yaw_ = yaw;
  active_.clear();
  for (auto & landmark : landmarks_) {
    landmark.measured = false;
  }
}

void LandmarkClusterer::setAssociationDistance(double maximum_distance)
{
  if (maximum_distance <= 0.0) {
    throw std::invalid_argument("Landmark association distance must be positive");
  }
  association_distance_squared_ = maximum_distance * maximum_distance;
}

void LandmarkClusterer::setLandmarks(std::vector<Landmark> landmarks)
{
  landmarks_.clear();
  next_landmark_id_ = 0U;
  for (auto & landmark : landmarks) {
    landmark.id = next_landmark_id_++;
    landmarks_.push_back(std::move(landmark));
  }
  active_.clear();
}

bool LandmarkClusterer::updateLandmark(
  std::uint32_t id, const Point2D & center,
  const std::array<double, 4> & covariance)
{
  const auto match = std::find_if(
    landmarks_.begin(), landmarks_.end(), [id](const auto & landmark) {
      return landmark.id == id;
    });
  if (match == landmarks_.end()) {
    return false;
  }
  match->center = center;
  match->covariance = covariance;
  return true;
}

void LandmarkClusterer::associate(const LandmarkDetection & detection)
{
  const auto global_offset = rotate(detection.measurement, yaw_);
  const Point2D global{
    position_.x + global_offset.x, position_.y + global_offset.y};

  Landmark * closest = nullptr;
  double closest_distance = association_distance_squared_;
  for (auto & landmark : landmarks_) {
    const double distance = distanceSquared(landmark.center, global);
    if (distance < closest_distance) {
      closest = &landmark;
      closest_distance = distance;
    }
  }

  if (closest == nullptr) {
    if (mapping_) {
      Landmark landmark;
      landmark.id = next_landmark_id_++;
      landmark.center = global;
      landmark.object_type = detection.object_type;
      landmarks_.push_back(landmark);
    }
    return;
  }

  if (!closest->measured) {
    active_.push_back(closest);
  } else {
    const auto previous_global_offset = rotate(closest->measurement, yaw_);
    const Point2D previous_global{
      position_.x + previous_global_offset.x,
      position_.y + previous_global_offset.y};
    if (distanceSquared(closest->center, previous_global) <=
      distanceSquared(closest->center, global))
    {
      // Multiple detections may fall inside one association gate. Keep only
      // the detection whose provisional global position is nearest the map
      // landmark; this rejects the less plausible alternatives for this scan.
      return;
    }
  }
  closest->measurement = detection.measurement;
  closest->measurement_covariance = detection.covariance;
  closest->object_type = detection.object_type;
  closest->confidence = detection.confidence;
  closest->measured = true;
}

void LandmarkClusterer::process(
  const std::vector<LandmarkDetection> & detections)
{
  for (const auto & detection : detections) {
    associate(detection);
  }
}

std::vector<Landmark *> LandmarkClusterer::active(std::size_t maximum)
{
  auto result = active_;
  std::sort(
    result.begin(), result.end(), [this](const auto * first, const auto * second) {
      return distanceSquared(first->center, position_) <
             distanceSquared(second->center, position_);
    });
  if (result.size() > maximum) {
    result.resize(maximum);
  }
  return result;
}

}  // namespace robosoft_core
