// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <cmath>

namespace robosoft_core
{

struct PathTrackingInput
{
  double path_curvature_km{0.0};
  double lookahead_curvature_km{0.0};
  double cross_track_error_m{0.0};
  double heading_error_rad{0.0};
  double target_speed_ms{0.0};
  double measured_speed_ms{0.0};
};

struct PathTrackingOutput
{
  double speed_ms{0.0};
  double curvature_m_inv{0.0};
};

/**
 * Stateless route-tracking control law.
 *
 * Curvature is converted from the controller's 1/km convention
 * to the Ros2ISOBUS 1/m API.
 */
inline PathTrackingOutput calculateSimplePathTracking(
  const PathTrackingInput & input, double lateral_gain,
  double heading_gain, double minimum_speed_ms, double speed_ramp_ms,
  double maximum_speed_ms, double maximum_curvature_m_inv,
  double lookahead_curvature_weight)
{
  PathTrackingOutput output;
  const double direction = input.target_speed_ms < 0.0 ? -1.0 : 1.0;
  double speed = input.target_speed_ms;

  if (std::abs(input.cross_track_error_m) > 0.1) {
    speed = std::copysign(std::min(0.5, maximum_speed_ms), speed);
  }
  if (speed > 0.0) {
    speed = std::min(speed, input.measured_speed_ms + speed_ramp_ms);
    speed = std::max(speed, minimum_speed_ms);
  } else if (speed < 0.0) {
    speed = std::max(speed, input.measured_speed_ms - speed_ramp_ms);
    speed = std::min(speed, -minimum_speed_ms);
  }
  output.speed_ms = std::clamp(speed, -maximum_speed_ms, maximum_speed_ms);

  const double curvature_weight =
    std::clamp(lookahead_curvature_weight, 0.0, 1.0);
  const double feedforward_curvature_km =
    (1.0 - curvature_weight) * input.path_curvature_km +
    curvature_weight * input.lookahead_curvature_km;
  const double corrected_curvature_km =
    feedforward_curvature_km +
    lateral_gain *
    (direction * input.cross_track_error_m +
    output.speed_ms * heading_gain * std::sin(input.heading_error_rad));
  output.curvature_m_inv = std::clamp(
    corrected_curvature_km / 1000.0,
    -maximum_curvature_m_inv, maximum_curvature_m_inv);
  return output;
}

}  // namespace robosoft_core
