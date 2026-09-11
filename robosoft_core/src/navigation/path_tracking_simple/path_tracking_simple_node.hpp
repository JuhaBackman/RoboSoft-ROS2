// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robosoft_interfaces/msg/lidar_safety_status.hpp>
#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>

#include <string>

namespace robosoft_core
{

/** Converts RouteControl errors into a robot-independent ROS velocity command. */
class PathTrackingSimpleNode : public rclcpp::Node
{
public:
  explicit PathTrackingSimpleNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onRobotState(const robosoft_interfaces::msg::RobotState & message);
  void onRouteStatus(const robosoft_interfaces::msg::RouteStatus & message);
  void onMeasuredTwist(const geometry_msgs::msg::TwistStamped & message);
  void onLidarSafety(
    const robosoft_interfaces::msg::LidarSafetyStatus & message);
  void updateCommand();
  void publishStop();

  rclcpp::Subscription<robosoft_interfaces::msg::RobotState>::SharedPtr
    robot_state_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RouteStatus>::SharedPtr
    route_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    measured_twist_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr lidar_safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  robosoft_interfaces::msg::RobotState robot_state_;
  robosoft_interfaces::msg::RouteStatus route_status_;
  rclcpp::Time last_robot_state_;
  rclcpp::Time last_route_status_;
  rclcpp::Time last_measured_twist_;
  rclcpp::Time last_lidar_safety_;
  std::string robot_name_;
  double measured_speed_ms_{0.0};
  double lateral_gain_{-500.0};
  double heading_gain_{-5.0};
  double minimum_speed_ms_{0.3};
  double speed_ramp_ms_{0.3};
  double maximum_speed_ms_{2.0};
  double maximum_curvature_m_inv_{8.031};
  double lookahead_curvature_weight_{0.5};
  double lidar_maximum_speed_ms_{0.0};
  int automatic_state_machine_{
    robosoft_interfaces::msg::RobotState::STATE_MAIN};
  int automatic_substate_{
    robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO};
  int command_timeout_ms_{300};
  bool robot_state_received_{false};
  bool route_status_received_{false};
  bool measured_twist_received_{false};
  bool lidar_safety_received_{false};
  bool lidar_healthy_{false};
  bool lidar_enabled_{true};
};

}  // namespace robosoft_core
