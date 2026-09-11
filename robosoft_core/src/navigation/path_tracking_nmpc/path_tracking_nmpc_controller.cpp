// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <robosoft_core/navigation/path_tracking_nmpc_controller.hpp>

#include "generated/PathTrackingNmpcSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robosoft_core
{
namespace
{
enum StateIndex {POSITION_X, POSITION_Y, YAW, SPEED, CURVATURE,
  DELAYED_SPEED_COMMAND, DELAYED_CURVATURE_COMMAND, SPEED_COMMAND,
  CURVATURE_COMMAND};
enum ControlIndex {SPEED_COMMAND_RATE, CURVATURE_COMMAND_RATE};
enum ParameterIndex {SPEED_RESPONSE, CURVATURE_RESPONSE, COMMAND_DELAY,
  SPEED_RESPONSE_RATE, CURVATURE_RESPONSE_RATE};

double positive(double value) {return std::max(value, 1e-4);}
double normalizeAngle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

struct ProjectedReference
{
  PathTrackingNmpcReference value;
  std::size_t segment{0};
  bool valid{false};
};

std::size_t lastSegmentWithinDistance(
  const std::vector<PathTrackingNmpcReference> & route,
  std::size_t first_segment, double maximum_distance_m)
{
  const std::size_t route_last_segment = route.size() - 2;
  first_segment = std::min(first_segment, route_last_segment);
  double distance_m = 0.0;
  std::size_t last_segment = first_segment;
  for (std::size_t segment = first_segment; segment <= route_last_segment; ++segment) {
    // Always expose the immediately following segment so a projection at the
    // current segment endpoint can advance even when route samples are sparse.
    if (segment > first_segment + 1 && distance_m >= maximum_distance_m) break;
    last_segment = segment;
    distance_m += std::hypot(
      route[segment + 1].x_m - route[segment].x_m,
      route[segment + 1].y_m - route[segment].y_m);
  }
  return last_segment;
}

ProjectedReference projectToRoute(
  const std::vector<PathTrackingNmpcReference> & route,
  std::size_t first_segment, std::size_t last_segment,
  double x, double y, double predicted_yaw, double heading_tolerance_rad)
{
  ProjectedReference result;
  double minimum_distance_squared = std::numeric_limits<double>::max();
  const std::size_t route_last_segment = route.size() - 2;
  first_segment = std::min(first_segment, route_last_segment);
  last_segment = std::clamp(last_segment, first_segment, route_last_segment);
  for (std::size_t segment = first_segment; segment <= last_segment; ++segment) {
    const auto & a = route[segment];
    const auto & b = route[segment + 1];
    const double dx = b.x_m - a.x_m;
    const double dy = b.y_m - a.y_m;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-10) continue;
    const double t = std::clamp(
      ((x - a.x_m) * dx + (y - a.y_m) * dy) / length_squared, 0.0, 1.0);
    const double reference_x = a.x_m + t * dx;
    const double reference_y = a.y_m + t * dy;
    const double interpolated_yaw = a.yaw_rad +
      t * normalizeAngle(b.yaw_rad - a.yaw_rad);
    if (std::abs(normalizeAngle(interpolated_yaw - predicted_yaw)) >
      heading_tolerance_rad)
    {
      continue;
    }
    const double distance_squared =
      std::pow(x - reference_x, 2) + std::pow(y - reference_y, 2);
    if (distance_squared >= minimum_distance_squared) continue;
    minimum_distance_squared = distance_squared;
    result.segment = segment;
    result.value.x_m = reference_x;
    result.value.y_m = reference_y;
    result.value.yaw_rad = predicted_yaw +
      normalizeAngle(interpolated_yaw - predicted_yaw);
    result.value.speed_m_s = a.speed_m_s + t * (b.speed_m_s - a.speed_m_s);
    result.value.curvature_m_inv = a.curvature_m_inv +
      t * (b.curvature_m_inv - a.curvature_m_inv);
    result.valid = true;
  }
  return result;
}
}  // namespace

PathTrackingNmpcController::PathTrackingNmpcController()
: solver_(std::make_unique<PathTrackingNmpcSolver>()) {configure(parameters_);}
PathTrackingNmpcController::~PathTrackingNmpcController() = default;

void PathTrackingNmpcController::configure(const PathTrackingNmpcParameters & parameters)
{
  parameters_ = parameters;
  if (parameters_.minimum_speed_m_s >= parameters_.maximum_speed_m_s ||
    parameters_.minimum_tracking_speed_m_s < 0.0 ||
    parameters_.minimum_curvature_m_inv >= parameters_.maximum_curvature_m_inv ||
    parameters_.minimum_acceleration_m_s2 >= 0.0 ||
    parameters_.maximum_acceleration_m_s2 <= 0.0 ||
    parameters_.minimum_curvature_rate_m_inv_s >= 0.0 ||
    parameters_.maximum_curvature_rate_m_inv_s <= 0.0 ||
    parameters_.initial_reference_search_distance_m <= 0.0 ||
    parameters_.reference_step_search_distance_m <= 0.0 ||
    parameters_.reference_heading_tolerance_rad <= 0.0 ||
    parameters_.reference_heading_tolerance_rad > M_PI ||
    parameters_.reference_update_iterations <= 0 || parameters_.solver_iterations <= 0)
  {
    throw std::invalid_argument("Invalid NMPC vehicle limits or iteration count");
  }
  auto & solver = *solver_;
  std::fill(solver.Q, solver.Q + solver.numStates * solver.numStates, 0.0);
  std::fill(solver.R, solver.R + solver.numStates * solver.numControls, 0.0);
  std::fill(solver.P, solver.P + solver.numStates * solver.numStates, 0.0);
  const double state_weights[] = {
    parameters_.position_weight, parameters_.position_weight,
    parameters_.heading_weight, parameters_.speed_weight,
    parameters_.curvature_weight, parameters_.command_state_weight,
    parameters_.command_state_weight, parameters_.command_state_weight,
    parameters_.command_state_weight};
  for (int i = 0; i < solver.numStates; ++i) {
    solver.Q[i * solver.numStates + i] = state_weights[i];
    solver.P[i * solver.numStates + i] =
      state_weights[i] * parameters_.terminal_weight_multiplier;
  }
  solver.R[SPEED_COMMAND_RATE * solver.numControls + SPEED_COMMAND_RATE] =
    parameters_.speed_rate_weight;
  solver.R[CURVATURE_COMMAND_RATE * solver.numControls + CURVATURE_COMMAND_RATE] =
    parameters_.curvature_rate_weight;
  for (const int state : {SPEED, DELAYED_SPEED_COMMAND, SPEED_COMMAND}) {
    solver.xMin[state] = parameters_.minimum_speed_m_s;
    solver.xMax[state] = parameters_.maximum_speed_m_s;
  }
  for (const int state : {CURVATURE, DELAYED_CURVATURE_COMMAND, CURVATURE_COMMAND}) {
    solver.xMin[state] = parameters_.minimum_curvature_m_inv;
    solver.xMax[state] = parameters_.maximum_curvature_m_inv;
  }
  solver.uMin[SPEED_COMMAND_RATE] = parameters_.minimum_acceleration_m_s2;
  solver.uMax[SPEED_COMMAND_RATE] = parameters_.maximum_acceleration_m_s2;
  solver.uMin[CURVATURE_COMMAND_RATE] = parameters_.minimum_curvature_rate_m_inv_s;
  solver.uMax[CURVATURE_COMMAND_RATE] = parameters_.maximum_curvature_rate_m_inv_s;
}

PathTrackingNmpcOutput PathTrackingNmpcController::calculate(
  const PathTrackingNmpcInput & input)
{
  PathTrackingNmpcOutput output;
  if (input.route.size() < 2 || !std::isfinite(input.x_m) ||
    !std::isfinite(input.y_m) || !std::isfinite(input.yaw_rad)) return output;
  auto & solver = *solver_;
  double measured_curvature = previous_curvature_command_;
  if (std::abs(input.measured_speed_m_s) > 1e-3) {
    measured_curvature = input.measured_yaw_rate_rad_s / input.measured_speed_m_s;
  }
  measured_curvature = std::clamp(
    measured_curvature, parameters_.minimum_curvature_m_inv,
    parameters_.maximum_curvature_m_inv);
  const std::size_t route_last_segment = input.route.size() - 2;
  const std::size_t start_segment = std::min(
    input.start_segment_index, route_last_segment);
  const std::size_t initial_last_segment = lastSegmentWithinDistance(
    input.route, start_segment, parameters_.initial_reference_search_distance_m);
  const auto initial_reference = projectToRoute(
    input.route, start_segment, initial_last_segment,
    input.x_m, input.y_m, input.yaw_rad,
    parameters_.reference_heading_tolerance_rad);
  if (!initial_reference.valid) return output;
  output.reference_segment_index = initial_reference.segment;
  const bool decelerating =
    input.measured_speed_m_s * initial_reference.value.speed_m_s < 0.0 ||
    std::abs(initial_reference.value.speed_m_s) < std::abs(input.measured_speed_m_s);
  solver.p[SPEED_RESPONSE] = positive(decelerating ?
    parameters_.speed_deceleration_response_s : parameters_.speed_acceleration_response_s);
  solver.p[CURVATURE_RESPONSE] = positive(parameters_.curvature_response_s);
  solver.p[COMMAND_DELAY] = positive(parameters_.command_delay_s);
  solver.p[SPEED_RESPONSE_RATE] = positive(decelerating ?
    std::abs(parameters_.minimum_acceleration_m_s2) :
    parameters_.maximum_acceleration_m_s2);
  solver.p[CURVATURE_RESPONSE_RATE] = positive(std::max(
    std::abs(parameters_.minimum_curvature_rate_m_inv_s),
    parameters_.maximum_curvature_rate_m_inv_s));

  if (initialized_) solver.shiftControlsStates();
  else {
    std::fill(solver.u, solver.u + solver.numControls * solver.numSteps, 0.0);
    previous_speed_command_ = input.measured_speed_m_s;
    previous_curvature_command_ = measured_curvature;
    initialized_ = true;
  }
  std::fill(solver.x, solver.x + solver.numStates * (solver.numSteps + 1), 0.0);
  std::fill(solver.x_ref, solver.x_ref + solver.numStates * solver.numSteps, 0.0);
  std::fill(solver.u_ref, solver.u_ref + solver.numControls * solver.numSteps, 0.0);
  solver.x[POSITION_X] = input.x_m;
  solver.x[POSITION_Y] = input.y_m;
  solver.x[YAW] = input.yaw_rad;
  solver.x[SPEED] = input.measured_speed_m_s;
  solver.x[CURVATURE] = measured_curvature;
  solver.x[DELAYED_SPEED_COMMAND] = previous_speed_command_;
  solver.x[DELAYED_CURVATURE_COMMAND] = previous_curvature_command_;
  solver.x[SPEED_COMMAND] = previous_speed_command_;
  solver.x[CURVATURE_COMMAND] = previous_curvature_command_;

  for (int update = 0; update < parameters_.reference_update_iterations; ++update) {
    solver.integrateSystem(solver.x, solver.u);
    std::size_t search_segment = initial_reference.segment;
    for (int step = 0; step < solver.numSteps; ++step) {
      const int predicted = (step + 1) * solver.numStates;
      const std::size_t step_last_segment = lastSegmentWithinDistance(
        input.route, search_segment, parameters_.reference_step_search_distance_m);
      const auto reference = projectToRoute(
        input.route, search_segment, step_last_segment,
        solver.x[predicted + POSITION_X], solver.x[predicted + POSITION_Y],
        solver.x[predicted + YAW], parameters_.reference_heading_tolerance_rad);
      if (!reference.valid) {
        reset();
        return output;
      }
      search_segment = reference.segment;
      const int offset = step * solver.numStates;
      solver.x_ref[offset + POSITION_X] = reference.value.x_m;
      solver.x_ref[offset + POSITION_Y] = reference.value.y_m;
      solver.x_ref[offset + YAW] = reference.value.yaw_rad;
      solver.x_ref[offset + SPEED] = reference.value.speed_m_s;
      solver.x_ref[offset + CURVATURE] = reference.value.curvature_m_inv;
      solver.x_ref[offset + DELAYED_SPEED_COMMAND] = reference.value.speed_m_s;
      solver.x_ref[offset + DELAYED_CURVATURE_COMMAND] = reference.value.curvature_m_inv;
      solver.x_ref[offset + SPEED_COMMAND] = reference.value.speed_m_s;
      solver.x_ref[offset + CURVATURE_COMMAND] = reference.value.curvature_m_inv;
    }
    solver.optimize(parameters_.solver_iterations);
  }

  solver.integrateSystem(solver.x, solver.u);
  const double speed_command = solver.x[solver.numStates + SPEED_COMMAND];
  const double curvature_command = solver.x[solver.numStates + CURVATURE_COMMAND];
  if (!std::isfinite(speed_command) || !std::isfinite(curvature_command)) {
    reset();
    return output;
  }
  output.speed_m_s = std::clamp(
    speed_command, parameters_.minimum_speed_m_s, parameters_.maximum_speed_m_s);
  const double target_speed_m_s = initial_reference.value.speed_m_s;
  if (target_speed_m_s > 1e-3) {
    const double minimum_tracking_speed = std::min(
      {parameters_.minimum_tracking_speed_m_s, target_speed_m_s,
        parameters_.maximum_speed_m_s});
    output.speed_m_s = std::max(output.speed_m_s, minimum_tracking_speed);
  } else if (target_speed_m_s < -1e-3) {
    const double minimum_tracking_speed = std::min(
      {parameters_.minimum_tracking_speed_m_s, -target_speed_m_s,
        -parameters_.minimum_speed_m_s});
    output.speed_m_s = std::min(output.speed_m_s, -minimum_tracking_speed);
  }
  output.curvature_m_inv = std::clamp(
    curvature_command, parameters_.minimum_curvature_m_inv,
    parameters_.maximum_curvature_m_inv);
  output.valid = true;
  previous_speed_command_ = output.speed_m_s;
  previous_curvature_command_ = output.curvature_m_inv;
  return output;
}

void PathTrackingNmpcController::reset()
{
  initialized_ = false;
  previous_speed_command_ = 0.0;
  previous_curvature_command_ = 0.0;
}

}  // namespace robosoft_core
