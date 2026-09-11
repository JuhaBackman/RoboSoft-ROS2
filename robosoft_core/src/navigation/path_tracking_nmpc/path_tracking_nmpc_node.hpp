// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <robosoft_interfaces/msg/lidar_safety_status.hpp>
#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>
#include <robosoft_interfaces/msg/timer_performance.hpp>
#include <robosoft_core/navigation/path_tracking_nmpc_controller.hpp>

#include <chrono>
#include <cstdint>
#include <string>

namespace robosoft_core
{

/** Optimizes a local vehicle prediction against RouteControl's active route. */
class PathTrackingNmpcNode : public rclcpp::Node
{
public:
  explicit PathTrackingNmpcNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onRobotState(const robosoft_interfaces::msg::RobotState & message);
  void onRouteStatus(const robosoft_interfaces::msg::RouteStatus & message);
  void onCurrentRoute(const robosoft_interfaces::msg::GuidanceLine & message);
  void onMapOrigin(const sensor_msgs::msg::NavSatFix & message);
  void onOdometry(const nav_msgs::msg::Odometry & message);
  void onMeasuredTwist(const geometry_msgs::msg::TwistStamped & message);
  void onLidarSafety(
    const robosoft_interfaces::msg::LidarSafetyStatus & message);
  void updateCommand();
  void updateTimedCommand();
  void publishStop();
  void rebuildLocalRoute();

  rclcpp::Subscription<robosoft_interfaces::msg::RobotState>::SharedPtr
    robot_state_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RouteStatus>::SharedPtr
    route_status_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::GuidanceLine>::SharedPtr
    current_route_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    measured_twist_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr lidar_safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<robosoft_interfaces::msg::TimerPerformance>::SharedPtr
    timer_performance_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  robosoft_interfaces::msg::RobotState robot_state_;
  robosoft_interfaces::msg::RouteStatus route_status_;
  robosoft_interfaces::msg::GuidanceLine current_route_;
  nav_msgs::msg::Odometry odometry_;
  std::vector<PathTrackingNmpcReference> local_route_;
  rclcpp::Time last_robot_state_;
  rclcpp::Time last_route_status_;
  rclcpp::Time last_odometry_;
  rclcpp::Time last_measured_twist_;
  rclcpp::Time last_lidar_safety_;
  std::string robot_name_;
  double measured_speed_ms_{0.0};
  double measured_yaw_rate_rad_s_{0.0};
  double map_origin_latitude_{0.0};
  double map_origin_longitude_{0.0};
  PathTrackingNmpcController controller_;
  PathTrackingNmpcParameters controller_parameters_;
  double maximum_speed_ms_{2.3};
  double lidar_maximum_speed_ms_{0.0};
  int automatic_state_machine_{
    robosoft_interfaces::msg::RobotState::STATE_MAIN};
  int automatic_substate_{
    robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO};
  int command_timeout_ms_{300};
  bool robot_state_received_{false};
  bool route_status_received_{false};
  bool current_route_received_{false};
  bool map_origin_received_{false};
  bool odometry_received_{false};
  bool measured_twist_received_{false};
  bool lidar_safety_received_{false};
  bool lidar_healthy_{false};
  bool lidar_enabled_{true};
  std::chrono::steady_clock::time_point previous_timer_start_;
  bool timer_started_{false};
  uint64_t timer_deadline_misses_{0};
};

}  // namespace robosoft_core
