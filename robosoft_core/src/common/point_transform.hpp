// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <geometry_msgs/msg/transform.hpp>

namespace robosoft_core
{

/// Cartesian point used at PointCloud2-to-TF boundaries.
struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Apply a geometry_msgs rigid transform without changing point ordering.
inline Point3D transformPoint(
  const Point3D & point, const geometry_msgs::msg::Transform & transform)
{
  const auto & q = transform.rotation;
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  return {
    transform.translation.x +
    (1.0 - 2.0 * (yy + zz)) * point.x +
    2.0 * (xy - wz) * point.y + 2.0 * (xz + wy) * point.z,
    transform.translation.y + 2.0 * (xy + wz) * point.x +
    (1.0 - 2.0 * (xx + zz)) * point.y +
    2.0 * (yz - wx) * point.z,
    transform.translation.z + 2.0 * (xz - wy) * point.x +
    2.0 * (yz + wx) * point.y +
    (1.0 - 2.0 * (xx + yy)) * point.z};
}

}  // namespace robosoft_core
