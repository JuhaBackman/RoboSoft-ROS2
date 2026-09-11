// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "path_tracking_nmpc_node.hpp"

#include <robosoft_interfaces/topics.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

namespace robosoft_core
{

PathTrackingNmpcNode::PathTrackingNmpcNode(const rclcpp::NodeOptions & options)
: Node("path_tracking_nmpc_node", options),
  last_robot_state_(0, 0, get_clock()->get_clock_type()),
  last_route_status_(0, 0, get_clock()->get_clock_type()),
  last_odometry_(0, 0, get_clock()->get_clock_type()),
  last_measured_twist_(0, 0, get_clock()->get_clock_type()),
  last_lidar_safety_(0, 0, get_clock()->get_clock_type())
{
  robot_name_ = declare_parameter<std::string>("robot_name", "");
  automatic_state_machine_ = declare_parameter<int>(
    "automatic_state_machine",
    robosoft_interfaces::msg::RobotState::STATE_MAIN);
  automatic_substate_ = declare_parameter<int>(
    "automatic_substate", robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO);
  controller_parameters_.speed_acceleration_response_s = declare_parameter<double>(
    "speed_acceleration_response_s", 4.5);
  controller_parameters_.speed_deceleration_response_s = declare_parameter<double>(
    "speed_deceleration_response_s", 1.35);
  controller_parameters_.curvature_response_s = declare_parameter<double>(
    "curvature_response_s", 1.4);
  controller_parameters_.command_delay_s = declare_parameter<double>(
    "command_delay_s", 0.1);
  controller_parameters_.minimum_speed_m_s = declare_parameter<double>(
    "minimum_speed_m_s", -1.2);
  controller_parameters_.maximum_speed_m_s = declare_parameter<double>(
    "maximum_speed_m_s", 2.3);
  controller_parameters_.minimum_tracking_speed_m_s = declare_parameter<double>(
    "minimum_tracking_speed_m_s", 0.0);
  controller_parameters_.minimum_acceleration_m_s2 = declare_parameter<double>(
    "minimum_acceleration_m_s2", -5.2);
  controller_parameters_.maximum_acceleration_m_s2 = declare_parameter<double>(
    "maximum_acceleration_m_s2", 5.6);
  controller_parameters_.minimum_curvature_m_inv = declare_parameter<double>(
    "minimum_curvature_m_inv", -0.6);
  controller_parameters_.maximum_curvature_m_inv = declare_parameter<double>(
    "maximum_curvature_m_inv", 0.6);
  controller_parameters_.minimum_curvature_rate_m_inv_s =
    declare_parameter<double>("minimum_curvature_rate_m_inv_s", -0.28);
  controller_parameters_.maximum_curvature_rate_m_inv_s =
    declare_parameter<double>("maximum_curvature_rate_m_inv_s", 0.28);
  controller_parameters_.position_weight = declare_parameter<double>(
    "position_weight", 8.0);
  controller_parameters_.heading_weight = declare_parameter<double>(
    "heading_weight", 25.0);
  controller_parameters_.speed_weight = declare_parameter<double>(
    "speed_weight", 4.0);
  controller_parameters_.curvature_weight = declare_parameter<double>(
    "curvature_weight", 12.0);
  controller_parameters_.command_state_weight = declare_parameter<double>(
    "command_state_weight", 0.2);
  controller_parameters_.speed_rate_weight = declare_parameter<double>(
    "speed_rate_weight", 4.0);
  controller_parameters_.curvature_rate_weight = declare_parameter<double>(
    "curvature_rate_weight", 12.0);
  controller_parameters_.terminal_weight_multiplier = declare_parameter<double>(
    "terminal_weight_multiplier", 3.0);
  controller_parameters_.initial_reference_search_distance_m =
    declare_parameter<double>("initial_reference_search_distance_m", 3.0);
  controller_parameters_.reference_step_search_distance_m =
    declare_parameter<double>("reference_step_search_distance_m", 2.0);
  controller_parameters_.reference_heading_tolerance_rad =
    declare_parameter<double>("reference_heading_tolerance_rad", kPi / 2.0);
  controller_parameters_.solver_iterations = declare_parameter<int>(
    "solver_iterations", 8);
  controller_parameters_.reference_update_iterations = declare_parameter<int>(
    "reference_update_iterations", 3);
  controller_.configure(controller_parameters_);
  maximum_speed_ms_ = controller_parameters_.maximum_speed_m_s;
  lidar_enabled_ = declare_parameter<bool>("enable_lidar", true);
  command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 300);
  lidar_maximum_speed_ms_ = maximum_speed_ms_;

  robot_state_sub_ = create_subscription<robosoft_interfaces::msg::RobotState>(
    robosoft_interfaces::kRobotStateTopic, 10,
    std::bind(&PathTrackingNmpcNode::onRobotState, this, std::placeholders::_1));
  route_status_sub_ = create_subscription<robosoft_interfaces::msg::RouteStatus>(
    robosoft_interfaces::kRouteStatusTopic, 10,
    std::bind(&PathTrackingNmpcNode::onRouteStatus, this, std::placeholders::_1));
  current_route_sub_ =
    create_subscription<robosoft_interfaces::msg::GuidanceLine>(
    robosoft_interfaces::kCurrentRouteTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&PathTrackingNmpcNode::onCurrentRoute, this, std::placeholders::_1));
  map_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    robosoft_interfaces::kMapOriginTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&PathTrackingNmpcNode::onMapOrigin, this, std::placeholders::_1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    robosoft_interfaces::kOdometryTopic, 10,
    std::bind(&PathTrackingNmpcNode::onOdometry, this, std::placeholders::_1));
  measured_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kMeasuredTwistTopic, 10,
    std::bind(&PathTrackingNmpcNode::onMeasuredTwist, this, std::placeholders::_1));
  if (lidar_enabled_) {
    lidar_safety_sub_ =
      create_subscription<robosoft_interfaces::msg::LidarSafetyStatus>(
      robosoft_interfaces::kLidarSafetyStatusTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathTrackingNmpcNode::onLidarSafety, this, std::placeholders::_1));
  }
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kPathTrackingCommandTopic, 10);
  timer_performance_pub_ =
    create_publisher<robosoft_interfaces::msg::TimerPerformance>(
    robosoft_interfaces::kPathTrackingTimerPerformanceTopic, 10);
  command_timer_ = create_wall_timer(
    100ms, std::bind(&PathTrackingNmpcNode::updateTimedCommand, this));
  publishStop();
}

void PathTrackingNmpcNode::updateTimedCommand()
{
  constexpr double target_period_ms = 100.0;
  const auto start = std::chrono::steady_clock::now();
  const double actual_period_ms = timer_started_ ?
    std::chrono::duration<double, std::milli>(start - previous_timer_start_).count() :
    target_period_ms;
  previous_timer_start_ = start;
  timer_started_ = true;

  updateCommand();

  const double duration_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  const double lateness_ms = std::max(
    {0.0, actual_period_ms - target_period_ms, duration_ms - target_period_ms});
  if (lateness_ms > 1.0) ++timer_deadline_misses_;
  robosoft_interfaces::msg::TimerPerformance performance;
  performance.node_name = get_name();
  performance.target_period_ms = target_period_ms;
  performance.actual_period_ms = actual_period_ms;
  performance.callback_duration_ms = duration_ms;
  performance.margin_ms = std::max(0.0, target_period_ms - duration_ms);
  performance.lateness_ms = lateness_ms;
  performance.deadline_misses = timer_deadline_misses_;
  timer_performance_pub_->publish(performance);
}

void PathTrackingNmpcNode::onRobotState(
  const robosoft_interfaces::msg::RobotState & message)
{
  if (!robot_name_.empty() && message.robot_name != robot_name_) return;
  robot_state_ = message;
  robot_state_received_ = true;
  last_robot_state_ = now();
}

void PathTrackingNmpcNode::onRouteStatus(
  const robosoft_interfaces::msg::RouteStatus & message)
{
  route_status_ = message;
  route_status_received_ = true;
  last_route_status_ = now();
}

void PathTrackingNmpcNode::onCurrentRoute(
  const robosoft_interfaces::msg::GuidanceLine & message)
{
  current_route_ = message;
  current_route_received_ = true;
  rebuildLocalRoute();
}

void PathTrackingNmpcNode::onMapOrigin(const sensor_msgs::msg::NavSatFix & message)
{
  if (!std::isfinite(message.latitude) || !std::isfinite(message.longitude)) return;
  map_origin_latitude_ = message.latitude;
  map_origin_longitude_ = message.longitude;
  map_origin_received_ = true;
  rebuildLocalRoute();
}

void PathTrackingNmpcNode::onOdometry(const nav_msgs::msg::Odometry & message)
{
  odometry_ = message;
  odometry_received_ = true;
  last_odometry_ = now();
}

void PathTrackingNmpcNode::rebuildLocalRoute()
{
  local_route_.clear();
  if (!current_route_received_ || !map_origin_received_) return;
  const double reference_latitude_rad = map_origin_latitude_ * kPi / 180.0;
  local_route_.reserve(current_route_.points.size());
  for (const auto & point : current_route_.points) {
    const double latitude_rad = point.latitude * kPi / 180.0;
    const double longitude_rad = point.longitude * kPi / 180.0;
    const double reference_longitude_rad = map_origin_longitude_ * kPi / 180.0;
    PathTrackingNmpcReference reference;
    reference.x_m = kEarthRadiusM *
      (longitude_rad - reference_longitude_rad) * std::cos(reference_latitude_rad);
    reference.y_m = kEarthRadiusM * (latitude_rad - reference_latitude_rad);
    reference.yaw_rad = point.yaw;
    reference.speed_m_s = point.speed;
    local_route_.push_back(reference);
  }
  for (std::size_t index = 0; index + 1 < local_route_.size(); ++index) {
    const auto & next = local_route_[index + 1];
    auto & current = local_route_[index];
    const double distance = std::hypot(next.x_m - current.x_m, next.y_m - current.y_m);
    const double direction = current.speed_m_s < 0.0 ? -1.0 : 1.0;
    current.curvature_m_inv = distance > 1e-6 ?
      normalizeAngle(next.yaw_rad - current.yaw_rad) / (direction * distance) : 0.0;
  }
  if (local_route_.size() > 1) {
    local_route_.back().curvature_m_inv = local_route_[local_route_.size() - 2].curvature_m_inv;
  }
}

void PathTrackingNmpcNode::onMeasuredTwist(
  const geometry_msgs::msg::TwistStamped & message)
{
  measured_speed_ms_ = message.twist.linear.x;
  measured_yaw_rate_rad_s_ = message.twist.angular.z;
  measured_twist_received_ = true;
  last_measured_twist_ = now();
}

void PathTrackingNmpcNode::onLidarSafety(
  const robosoft_interfaces::msg::LidarSafetyStatus & message)
{
  lidar_safety_received_ = true;
  lidar_healthy_ = message.healthy;
  lidar_maximum_speed_ms_ = std::clamp(
    message.maximum_speed_m_s, 0.0, maximum_speed_ms_);
  last_lidar_safety_ = now();
}

void PathTrackingNmpcNode::updateCommand()
{
  const double timeout = static_cast<double>(command_timeout_ms_) / 1000.0;
  const bool fresh = robot_state_received_ && route_status_received_ &&
    measured_twist_received_ && odometry_received_ && local_route_.size() >= 2 &&
    (now() - last_robot_state_).seconds() <= timeout &&
    (now() - last_route_status_).seconds() <= timeout &&
    (now() - last_odometry_).seconds() <= timeout &&
    (now() - last_measured_twist_).seconds() <= timeout &&
    (!lidar_enabled_ || (lidar_safety_received_ && lidar_healthy_ &&
    (now() - last_lidar_safety_).seconds() <= timeout));
  const bool automatic =
    robot_state_.state_machine == automatic_state_machine_ &&
    robot_state_.substate == automatic_substate_ && !robot_state_.emergency_stop;
  if (!fresh || !automatic ||
    route_status_.state !=
    robosoft_interfaces::msg::RouteStatus::STATE_FOLLOWING)
  {
    publishStop();
    return;
  }

  PathTrackingNmpcInput input;
  input.x_m = odometry_.pose.pose.position.x;
  input.y_m = odometry_.pose.pose.position.y;
  input.yaw_rad = yawOf(odometry_.pose.pose.orientation);
  input.measured_speed_m_s = measured_speed_ms_;
  input.measured_yaw_rate_rad_s = measured_yaw_rate_rad_s_;
  input.start_segment_index = route_status_.segment_index >= 0 ?
    static_cast<std::size_t>(route_status_.segment_index) : 0U;
  input.route = local_route_;
  auto output = controller_.calculate(input);
  if (!output.valid) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "NMPC solver returned invalid output");
    publishStop();
    return;
  }
  const double limited_speed = std::clamp(
    output.speed_m_s, -lidar_maximum_speed_ms_, lidar_maximum_speed_ms_);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = limited_speed;
  command.twist.angular.z = output.curvature_m_inv * limited_speed;
  command_pub_->publish(command);
}

void PathTrackingNmpcNode::publishStop()
{
  controller_.reset();
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command_pub_->publish(command);
}

}  // namespace robosoft_core

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::PathTrackingNmpcNode>());
  rclcpp::shutdown();
  return 0;
}
