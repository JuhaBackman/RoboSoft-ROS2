// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

class PathTrackingNmpcSolver;

namespace robosoft_core
{

struct PathTrackingNmpcParameters
{
  double speed_acceleration_response_s{4.5};
  double speed_deceleration_response_s{1.35};
  double curvature_response_s{1.4};
  double command_delay_s{0.1};
  double minimum_speed_m_s{-1.2};
  double maximum_speed_m_s{2.3};
  double minimum_tracking_speed_m_s{0.0};
  double minimum_acceleration_m_s2{-5.2};
  double maximum_acceleration_m_s2{5.6};
  double minimum_curvature_m_inv{-0.6};
  double maximum_curvature_m_inv{0.6};
  double minimum_curvature_rate_m_inv_s{-0.28};
  double maximum_curvature_rate_m_inv_s{0.28};
  double position_weight{8.0};
  double heading_weight{25.0};
  double speed_weight{4.0};
  double curvature_weight{12.0};
  double command_state_weight{0.2};
  double speed_rate_weight{4.0};
  double curvature_rate_weight{12.0};
  double terminal_weight_multiplier{3.0};
  double initial_reference_search_distance_m{3.0};
  double reference_step_search_distance_m{2.0};
  double reference_heading_tolerance_rad{1.5707963267948966};
  int reference_update_iterations{3};
  int solver_iterations{8};
};

struct PathTrackingNmpcReference
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_m_s{0.0};
  double curvature_m_inv{0.0};
};

struct PathTrackingNmpcInput
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double measured_speed_m_s{0.0};
  double measured_yaw_rate_rad_s{0.0};
  std::size_t start_segment_index{0};
  std::vector<PathTrackingNmpcReference> route;
};

struct PathTrackingNmpcOutput
{
  double speed_m_s{0.0};
  double curvature_m_inv{0.0};
  std::size_t reference_segment_index{0};
  bool valid{false};
};

/** Runtime-configurable path NMPC around the VIATOC-generated solver. */
class PathTrackingNmpcController
{
public:
  PathTrackingNmpcController();
  ~PathTrackingNmpcController();
  PathTrackingNmpcController(const PathTrackingNmpcController &) = delete;
  PathTrackingNmpcController & operator=(const PathTrackingNmpcController &) = delete;

  void configure(const PathTrackingNmpcParameters & parameters);
  PathTrackingNmpcOutput calculate(const PathTrackingNmpcInput & input);
  void reset();

private:
  std::unique_ptr<PathTrackingNmpcSolver> solver_;
  PathTrackingNmpcParameters parameters_;
  double previous_speed_command_{0.0};
  double previous_curvature_command_{0.0};
  bool initialized_{false};
};

}  // namespace robosoft_core
