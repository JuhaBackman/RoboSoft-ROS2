// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>
#include <robosoft_interfaces/msg/robot_command.hpp>
#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/srv/modify_task_status.hpp>

namespace robosoft_core
{

/**
 * Shared ROS 2 route recorder and geometric route follower.
 *
 * The node owns TASK-aware route-control responsibilities:
 *  - records WGS84 guidance patterns from GNSS and vehicle odometry;
 *  - selects the nearest valid pattern in the active guidance group;
 *  - publishes the interpolated target point and tracking errors;
 *  - advances task status through TaskManager on completion or failure.
 *
 * Inputs: task_manager/task_loaded, GNSS position, odometry and robot_command.
 * Outputs: route_control/status, route_control/recorded_route,
 * route_control/current_route and route_control/path.
 *
 * The map origin published by gps_to_cartesian_node defines the shared local
 * ENU coordinate frame for odometry, TASK route geometry and visualization.
 */
class RouteControlNode : public rclcpp::Node {
public:
  explicit RouteControlNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Public RouteStatus state values; kept stable for robot state machines.
  enum RouteState : uint8_t
  {
    IDLE = 0,
    RECORDING = 1,
    FOLLOWING = 2,
    COMPLETED = 3,
    ERROR = 4
  };

  struct XY
  {
    double x;
    double y;
  };

  // ROS input callbacks.
  void onTask(const robosoft_interfaces::msg::TaskData & msg);
  void onGps(const sensor_msgs::msg::NavSatFix & msg);
  void onMapOrigin(const sensor_msgs::msg::NavSatFix & msg);
  void onOdometry(const nav_msgs::msg::Odometry & msg);
  void onMeasuredTwist(const geometry_msgs::msg::TwistStamped & msg);
  void onRobotCommand(const robosoft_interfaces::msg::RobotCommand & msg);
  void onRobotState(const robosoft_interfaces::msg::RobotState & msg);
  void applyTaskOperation();
  // Recording lifecycle. A completed segment always ends in a 0 m/s point.
  void startRecording();
  void stopRecording();
  void finishRecordedSegment();
  // Following lifecycle and guidance-pattern progression.
  bool startFollowing();
  bool advanceGuidance();
  void publishActiveGuidance();
  void publishEmptyPath();
  void stopFollowing();
  // Per-odometry updates for the active route mode.
  void updateRecording(const nav_msgs::msg::Odometry & odom);
  void updateFollowing(const nav_msgs::msg::Odometry & odom);
  void updateRotation(const nav_msgs::msg::Odometry & odom);
  void advanceSegmentOrComplete();
  // Safety and TaskManager synchronization.
  void checkWatchdogs();
  void publishStatus(const std::string & message = "");
  void completeTask();

  // Coordinate and angle helpers used by both recording and following.
  XY toLocal(double latitude, double longitude) const;
  XY odometryToLocal(
    const geometry_msgs::msg::Point & position) const;
  static double normalizeAngle(double angle);
  static double yawOf(const geometry_msgs::msg::Quaternion & orientation);
  static double distanceMeters(
    double lat1, double lon1,
    double lat2, double lon2);

  // ROS interfaces.
  rclcpp::Subscription<robosoft_interfaces::msg::TaskData>::SharedPtr task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    measured_twist_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RobotCommand>::SharedPtr
    robot_command_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RobotState>::SharedPtr
    robot_state_sub_;
  rclcpp::Publisher<robosoft_interfaces::msg::RouteStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<robosoft_interfaces::msg::GuidanceLine>::SharedPtr
    recorded_route_pub_;
  rclcpp::Publisher<robosoft_interfaces::msg::GuidanceLine>::SharedPtr
    active_guidance_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
  rclcpp::Client<robosoft_interfaces::srv::ModifyTaskStatus>::SharedPtr
    task_status_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Cached inputs and route-progress state.
  robosoft_interfaces::msg::TaskData task_;
  robosoft_interfaces::msg::GuidanceLine recorded_route_;
  robosoft_interfaces::msg::RouteStatus status_;
  sensor_msgs::msg::NavSatFix gps_;
  nav_msgs::msg::Odometry last_odometry_;
  geometry_msgs::msg::TwistStamped measured_twist_;
  RouteState state_{IDLE};
  int segment_index_{-1};
  int guidance_index_{-1};
  std::string guidance_group_key_;
  bool gps_valid_{false};
  bool map_origin_received_{false};
  bool odometry_valid_{false};
  bool measured_twist_received_{false};
  bool robot_state_received_{false};
  uint8_t robot_substate_{0};
  bool completion_requested_{false};
  bool task_status_request_in_flight_{false};
  int pending_task_status_{-1};
  rclcpp::Time last_gps_time_;
  rclcpp::Time last_odometry_time_;
  rclcpp::Time last_measured_twist_time_;
  // Tunable geometry and watchdog parameters.
  double map_origin_latitude_{0.0};
  double map_origin_longitude_{0.0};
  double record_distance_{1.0};
  double record_yaw_difference_{0.35};
  double lookahead_minimum_m_{0.5};
  double lookahead_time_s_{1.4};
  double lookahead_maximum_m_{3.0};
  double maximum_start_distance_{10.0};
  int gps_timeout_ms_{1000};
  int odometry_timeout_ms_{500};
  int measured_twist_timeout_ms_{500};
  // Recording metadata propagated into each RoutePoint.
  uint8_t implement_mode_{0};
  int driving_direction_{0};
  int recorded_segment_count_{0};
};

}  // namespace robosoft_core
