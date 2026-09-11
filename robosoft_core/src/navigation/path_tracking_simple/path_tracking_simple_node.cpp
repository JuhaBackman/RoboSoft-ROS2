// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "path_tracking_simple_node.hpp"

#include <robosoft_core/navigation/path_tracking_simple_controller.hpp>
#include <robosoft_interfaces/topics.hpp>

#include <algorithm>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_core
{

PathTrackingSimpleNode::PathTrackingSimpleNode(const rclcpp::NodeOptions & options)
: Node("path_tracking_simple_node", options),
  last_robot_state_(0, 0, get_clock()->get_clock_type()),
  last_route_status_(0, 0, get_clock()->get_clock_type()),
  last_measured_twist_(0, 0, get_clock()->get_clock_type()),
  last_lidar_safety_(0, 0, get_clock()->get_clock_type())
{
  robot_name_ = declare_parameter<std::string>("robot_name", "");
  automatic_state_machine_ = declare_parameter<int>(
    "automatic_state_machine",
    robosoft_interfaces::msg::RobotState::STATE_MAIN);
  automatic_substate_ = declare_parameter<int>(
    "automatic_substate", robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO);
  lateral_gain_ = declare_parameter<double>("lateral_gain", -500.0);
  heading_gain_ = declare_parameter<double>("heading_gain", -5.0);
  minimum_speed_ms_ = declare_parameter<double>("minimum_speed_ms", 0.3);
  speed_ramp_ms_ = declare_parameter<double>("speed_ramp_ms", 0.3);
  maximum_speed_ms_ = declare_parameter<double>("maximum_speed_ms", 2.0);
  maximum_curvature_m_inv_ =
    declare_parameter<double>("maximum_curvature_m_inv", 8.031);
  lookahead_curvature_weight_ =
    declare_parameter<double>("lookahead_curvature_weight", 0.5);
  lidar_enabled_ = declare_parameter<bool>("enable_lidar", true);
  command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 300);
  lidar_maximum_speed_ms_ = maximum_speed_ms_;

  robot_state_sub_ = create_subscription<robosoft_interfaces::msg::RobotState>(
    robosoft_interfaces::kRobotStateTopic, 10,
    std::bind(&PathTrackingSimpleNode::onRobotState, this, std::placeholders::_1));
  route_status_sub_ = create_subscription<robosoft_interfaces::msg::RouteStatus>(
    robosoft_interfaces::kRouteStatusTopic, 10,
    std::bind(&PathTrackingSimpleNode::onRouteStatus, this, std::placeholders::_1));
  measured_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kMeasuredTwistTopic, 10,
    std::bind(&PathTrackingSimpleNode::onMeasuredTwist, this, std::placeholders::_1));
  if (lidar_enabled_) {
    lidar_safety_sub_ =
      create_subscription<robosoft_interfaces::msg::LidarSafetyStatus>(
      robosoft_interfaces::kLidarSafetyStatusTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathTrackingSimpleNode::onLidarSafety, this, std::placeholders::_1));
  }
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kPathTrackingCommandTopic, 10);
  command_timer_ = create_wall_timer(
    100ms, std::bind(&PathTrackingSimpleNode::updateCommand, this));
  publishStop();
}

void PathTrackingSimpleNode::onRobotState(
  const robosoft_interfaces::msg::RobotState & message)
{
  if (!robot_name_.empty() && message.robot_name != robot_name_) return;
  robot_state_ = message;
  robot_state_received_ = true;
  last_robot_state_ = now();
}

void PathTrackingSimpleNode::onRouteStatus(
  const robosoft_interfaces::msg::RouteStatus & message)
{
  route_status_ = message;
  route_status_received_ = true;
  last_route_status_ = now();
}

void PathTrackingSimpleNode::onMeasuredTwist(
  const geometry_msgs::msg::TwistStamped & message)
{
  measured_speed_ms_ = message.twist.linear.x;
  measured_twist_received_ = true;
  last_measured_twist_ = now();
}

void PathTrackingSimpleNode::onLidarSafety(
  const robosoft_interfaces::msg::LidarSafetyStatus & message)
{
  lidar_safety_received_ = true;
  lidar_healthy_ = message.healthy;
  lidar_maximum_speed_ms_ = std::clamp(
    message.maximum_speed_m_s, 0.0, maximum_speed_ms_);
  last_lidar_safety_ = now();
}

void PathTrackingSimpleNode::updateCommand()
{
  const double timeout = static_cast<double>(command_timeout_ms_) / 1000.0;
  const bool fresh = robot_state_received_ && route_status_received_ &&
    measured_twist_received_ &&
    (now() - last_robot_state_).seconds() <= timeout &&
    (now() - last_route_status_).seconds() <= timeout &&
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

  PathTrackingInput input;
  input.path_curvature_km = route_status_.path_curvature;
  input.lookahead_curvature_km = route_status_.lookahead_curvature;
  input.cross_track_error_m = route_status_.cross_track_error;
  input.heading_error_rad = route_status_.heading_error;
  input.target_speed_ms = route_status_.target_point.speed;
  input.measured_speed_ms = measured_speed_ms_;
  auto output = calculateSimplePathTracking(
    input, lateral_gain_, heading_gain_, minimum_speed_ms_, speed_ramp_ms_,
    maximum_speed_ms_, maximum_curvature_m_inv_,
    lookahead_curvature_weight_);
  output.speed_ms = std::clamp(
    output.speed_ms, -lidar_maximum_speed_ms_, lidar_maximum_speed_ms_);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = output.speed_ms;
  command.twist.angular.z = output.curvature_m_inv * output.speed_ms;
  command_pub_->publish(command);
}

void PathTrackingSimpleNode::publishStop()
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command_pub_->publish(command);
}

}  // namespace robosoft_core

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::PathTrackingSimpleNode>());
  rclcpp::shutdown();
  return 0;
}
