// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <string>

namespace robosoft_core
{

/// Two-dimensional point in either the vehicle or local-map frame.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// Sensor-independent object observation expressed in the vehicle frame.
struct LandmarkDetection
{
  Point2D measurement;
  std::array<double, 4> covariance{{2e-5, 0.0, 0.0, 2e-5}};
  std::string object_type{"pole"};
  float confidence{1.0F};
};

/// Persistent map landmark and its most recent associated observation.
struct Landmark
{
  std::uint32_t id{0U};
  Point2D center;
  Point2D measurement;
  std::array<double, 4> covariance{{0.1, 0.0, 0.0, 0.1}};
  std::array<double, 4> measurement_covariance{{2e-5, 0.0, 0.0, 2e-5}};
  std::string object_type{"pole"};
  float confidence{1.0F};
  bool measured{false};
};

inline double distanceSquared(const Point2D & first, const Point2D & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  return dx * dx + dy * dy;
}

inline Point2D rotate(const Point2D & point, double angle)
{
  return {
    point.x * std::cos(angle) - point.y * std::sin(angle),
    point.x * std::sin(angle) + point.y * std::cos(angle)};
}

}  // namespace robosoft_core
