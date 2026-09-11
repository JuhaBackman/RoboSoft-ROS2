// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <robosoft_core/robot_main_node_base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <robosoft_interfaces/msg/implement_command.hpp>
#include <robosoft_interfaces/msg/implement_status.hpp>
#include <robosoft_interfaces/msg/lidar_safety_status.hpp>
#include <robosoft_interfaces/msg/localization_mode.hpp>
#include <robosoft_interfaces/msg/remote_control_status.hpp>
#include <robosoft_interfaces/msg/robot_command.hpp>
#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>
#include <robosoft_interfaces/msg/state_estimator_status.hpp>
#include <robosoft_interfaces/msg/timer_performance.hpp>
#include <ros2_isobus/msg/tecu_guidance_status.hpp>
#include <ros2_isobus/msg/tecu_wheel_speed.hpp>

namespace robosoft_aki
{

/**
 * AKI top-level state machine.
 *
 * The node implements AKI WAIT/MANUAL/RECORD/AUTO/STOP behaviour. ROS graph
 * discovery provides two-stage dependency initialization. Hardware protocol
 * and path-control nodes remain separate;
 * this class coordinates them through ROS interfaces.
 */
class AkiMainNode : public robosoft_core::RobotMainNodeBase
{
public:
  explicit AkiMainNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum RemoteMode : uint8_t
  {
    REMOTE_SAFE = 0,
    REMOTE_MANUAL = 1,
    REMOTE_AUTO_1 = 2,
    REMOTE_AUTO_2 = 3,
    REMOTE_AUTO_3 = 4,
    REMOTE_DIRECT_BODY = 5,
    REMOTE_DIRECT_IMPLEMENT = 6
  };

  void onRemote(const robosoft_interfaces::msg::RemoteControlStatus & msg);
  void onCommand(const robosoft_interfaces::msg::RobotCommand & msg);
  void onOdometry(const nav_msgs::msg::Odometry & msg);
  void onGps(const sensor_msgs::msg::NavSatFix & msg);
  void onRouteStatus(const robosoft_interfaces::msg::RouteStatus & msg);
  void onLidarSafety(
    const robosoft_interfaces::msg::LidarSafetyStatus & msg);
  void onImplementStatus(
    const robosoft_interfaces::msg::ImplementStatus & msg);
  void onStateEstimatorStatus(
    const robosoft_interfaces::msg::StateEstimatorStatus & msg);
  void onTecuWheelSpeed(const ros2_isobus::msg::TecuWheelSpeed & msg);
  void onTecuGuidanceStatus(
    const ros2_isobus::msg::TecuGuidanceStatus & msg);

  void runStateMachine();
  void runTimedStateMachine();
  void checkInitialization();
  void checkRequiredData();
  void onEnterState(SubState substate) override;
  void commandImplement(uint8_t mode);
  void updateEmergencyStopState();

  bool remoteAutomatic() const;
  bool remoteManualOrSafe() const;
  bool automaticOperationPermitted() const;
  bool automaticStartPermitted() const;
  bool tecuDataFresh() const;
  bool estimatorStatusFresh() const;
  bool estimatorRequired() const;

  rclcpp::Subscription<
    robosoft_interfaces::msg::RemoteControlStatus>::SharedPtr remote_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RobotCommand>::SharedPtr
    command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::RouteStatus>::SharedPtr route_status_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr lidar_safety_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::ImplementStatus>::SharedPtr implement_status_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::StateEstimatorStatus>::SharedPtr
    estimator_status_sub_;
  rclcpp::Subscription<ros2_isobus::msg::TecuWheelSpeed>::SharedPtr
    tecu_wheel_speed_sub_;
  rclcpp::Subscription<ros2_isobus::msg::TecuGuidanceStatus>::SharedPtr
    tecu_guidance_status_sub_;

  rclcpp::Publisher<robosoft_interfaces::msg::LocalizationMode>::SharedPtr
    localization_mode_pub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::ImplementCommand>::SharedPtr implement_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Publisher<robosoft_interfaces::msg::TimerPerformance>::SharedPtr
    timer_performance_pub_;

  robosoft_interfaces::msg::RemoteControlStatus remote_;

  std::vector<std::string> required_nodes_;
  std::vector<std::string> previously_missing_data_;
  rclcpp::Time last_gps_fix_;
  rclcpp::Time last_estimator_status_;
  rclcpp::Time last_tecu_wheel_speed_;
  rclcpp::Time last_tecu_guidance_status_;
  robosoft_interfaces::msg::StateEstimatorStatus estimator_status_;
  double tecu_timeout_s_{1.0};
  double estimator_timeout_s_{1.0};
  uint8_t commanded_implement_mode_{255};
  bool lidar_enabled_{true};
  bool remote_received_{false};
  bool gps_valid_{false};
  bool odometry_received_{false};
  bool lidar_safety_received_{false};
  bool lidar_healthy_{false};
  bool implement_status_received_{false};
  bool estimator_status_received_{false};
  bool tecu_wheel_speed_received_{false};
  bool tecu_guidance_status_received_{false};
  bool gui_start_requested_{false};
  bool software_emergency_stop_{false};
  bool remote_emergency_stop_{false};
  std::chrono::steady_clock::time_point previous_timer_start_;
  bool timer_started_{false};
  uint64_t timer_deadline_misses_{0};
};

}  // namespace robosoft_aki
