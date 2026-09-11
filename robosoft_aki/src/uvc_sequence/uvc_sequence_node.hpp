// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>
#include <robosoft_interfaces/msg/lidar_safety_status.hpp>
#include <robosoft_interfaces/msg/implement_command.hpp>
#include <robosoft_interfaces/msg/implement_status.hpp>
#include <ros2_isobus/msg/tecu_wheel_speed.hpp>

namespace robosoft_aki
{

/**
 * Applies AKI UVC sequencing to the generic core path-tracking command.
 *
 * Motion is allowed only while AkiMain reports MAIN_AUTO and RouteControl
 * reports FOLLOWING. A local watchdog publishes zero speed if either input
 * becomes stale.
 */
class UvcSequenceNode : public rclcpp::Node
{
public:
  explicit UvcSequenceNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onRobotState(const robosoft_interfaces::msg::RobotState & msg);
  void onRouteStatus(const robosoft_interfaces::msg::RouteStatus & msg);
  void onWheelSpeed(const ros2_isobus::msg::TecuWheelSpeed & msg);
  void onLidarSafety(
    const robosoft_interfaces::msg::LidarSafetyStatus & msg);
  void onImplementStatus(
    const robosoft_interfaces::msg::ImplementStatus & msg);
  void onPathTrackingCommand(const geometry_msgs::msg::TwistStamped & msg);
  void updateCommands();
  void startUvcSequence();
  void updateUvcSequence();
  void publishImplementCommand(uint8_t mode);
  void publishStop();

  enum class UvcSequenceState : uint8_t
  {
    IDLE,
    WAIT_ACTIVE,
    WAIT_COMPLETE,
    ADVANCE
  };

  rclcpp::Subscription<robosoft_interfaces::msg::RobotState>::SharedPtr
    robot_state_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RouteStatus>::SharedPtr
    route_status_sub_;
  rclcpp::Subscription<ros2_isobus::msg::TecuWheelSpeed>::SharedPtr
    wheel_speed_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr lidar_safety_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::ImplementStatus>::SharedPtr implement_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    path_tracking_command_sub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::ImplementCommand>::SharedPtr implement_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  robosoft_interfaces::msg::RobotState robot_state_;
  robosoft_interfaces::msg::RouteStatus route_status_;
  rclcpp::Time last_robot_state_;
  rclcpp::Time last_route_status_;
  rclcpp::Time last_lidar_safety_;
  rclcpp::Time last_path_tracking_command_;
  geometry_msgs::msg::TwistStamped path_tracking_command_;
  double maximum_speed_ms_{2.0};
  double lidar_maximum_speed_ms_{0.0};
  double wheel_distance_m_{0.0};
  double drive_direction_{1.0};
  double route_speed_limit_ms_{0.3};
  double uvc_advance_target_m_{0.0};
  double uvc_advance_distance_m_{0.9};
  double uvc_minimum_speed_ms_{0.3};
  double uvc_distance_gain_{0.1};
  int command_timeout_ms_{300};
  UvcSequenceState uvc_sequence_state_{UvcSequenceState::IDLE};
  robosoft_interfaces::msg::ImplementStatus implement_status_;
  bool robot_state_received_{false};
  bool route_status_received_{false};
  bool lidar_safety_received_{false};
  bool lidar_healthy_{false};
  bool implement_status_received_{false};
  bool path_tracking_command_received_{false};
  bool stopped_{true};
  bool lidar_enabled_{true};
  bool uvc_sequence_enabled_{true};
};

}  // namespace robosoft_aki
