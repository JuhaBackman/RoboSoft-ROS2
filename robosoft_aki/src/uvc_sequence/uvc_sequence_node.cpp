// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "uvc_sequence_node.hpp"
#include "robosoft_interfaces/topics.hpp"
#include "topics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_aki
{

UvcSequenceNode::UvcSequenceNode(const rclcpp::NodeOptions & options)
: Node("aki_uvc_sequence_node", options),
  last_robot_state_(0, 0, get_clock()->get_clock_type()),
  last_route_status_(0, 0, get_clock()->get_clock_type()),
  last_lidar_safety_(0, 0, get_clock()->get_clock_type()),
  last_path_tracking_command_(0, 0, get_clock()->get_clock_type())
{
  maximum_speed_ms_ = declare_parameter<double>("maximum_speed_ms", 2.0);
  lidar_enabled_ = declare_parameter<bool>("enable_lidar", true);
  uvc_sequence_enabled_ =
    declare_parameter<bool>("enable_uvc_sequence", true);
  if (!lidar_enabled_) lidar_maximum_speed_ms_ = maximum_speed_ms_;
  uvc_advance_distance_m_ =
    declare_parameter<double>("uvc_advance_distance_m", 0.9);
  uvc_minimum_speed_ms_ =
    declare_parameter<double>("uvc_minimum_speed_ms", 0.3);
  uvc_distance_gain_ =
    declare_parameter<double>("uvc_distance_gain", 0.1);
  command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 300);

  robot_state_sub_ =
    create_subscription<robosoft_interfaces::msg::RobotState>(
    robosoft_interfaces::kRobotStateTopic, 10,
    std::bind(&UvcSequenceNode::onRobotState, this, std::placeholders::_1));
  route_status_sub_ =
    create_subscription<robosoft_interfaces::msg::RouteStatus>(
    robosoft_interfaces::kRouteStatusTopic, 10,
    std::bind(&UvcSequenceNode::onRouteStatus, this, std::placeholders::_1));
  wheel_speed_sub_ = create_subscription<ros2_isobus::msg::TecuWheelSpeed>(
    ros2_isobus::kTECUWheelSpeedTopic, 10,
    std::bind(&UvcSequenceNode::onWheelSpeed, this, std::placeholders::_1));
  if (lidar_enabled_) {
    lidar_safety_sub_ =
      create_subscription<robosoft_interfaces::msg::LidarSafetyStatus>(
      robosoft_interfaces::kLidarSafetyStatusTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &UvcSequenceNode::onLidarSafety, this, std::placeholders::_1));
  }
  implement_status_sub_ =
    create_subscription<robosoft_interfaces::msg::ImplementStatus>(
    robosoft_interfaces::kImplementStatusTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &UvcSequenceNode::onImplementStatus, this, std::placeholders::_1));
  path_tracking_command_sub_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kPathTrackingCommandTopic, 10,
    std::bind(
      &UvcSequenceNode::onPathTrackingCommand, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    robosoft_interfaces::kNavigationCommandTopic, 10);
  implement_pub_ =
    create_publisher<robosoft_interfaces::msg::ImplementCommand>(
    robosoft_interfaces::kImplementCommandTopic,
    rclcpp::QoS(1).reliable().transient_local());
  command_timer_ =
    create_wall_timer(100ms, std::bind(&UvcSequenceNode::updateCommands, this));

  publishStop();
  RCLCPP_INFO(
    get_logger(),
    "AKI command coordinator ready: core PathTracking -> UVC -> cmd_vel; sequence %s",
    uvc_sequence_enabled_ ? "enabled" : "bypassed");
}

void UvcSequenceNode::onRobotState(
  const robosoft_interfaces::msg::RobotState & msg)
{
  if (msg.robot_name != "aki") {
    return;
  }
  robot_state_ = msg;
  robot_state_received_ = true;
  last_robot_state_ = now();
}

void UvcSequenceNode::onRouteStatus(
  const robosoft_interfaces::msg::RouteStatus & msg)
{
  route_status_ = msg;
  route_status_received_ = true;
  last_route_status_ = now();
}

void UvcSequenceNode::onWheelSpeed(
  const ros2_isobus::msg::TecuWheelSpeed & msg)
{
  wheel_distance_m_ = msg.distance_m;
}

void UvcSequenceNode::onLidarSafety(
  const robosoft_interfaces::msg::LidarSafetyStatus & msg)
{
  lidar_safety_received_ = true;
  lidar_healthy_ = msg.healthy;
  lidar_maximum_speed_ms_ = std::clamp(
    msg.maximum_speed_m_s, 0.0, maximum_speed_ms_);
  last_lidar_safety_ = now();
}

void UvcSequenceNode::onImplementStatus(
  const robosoft_interfaces::msg::ImplementStatus & msg)
{
  implement_status_ = msg;
  implement_status_received_ = true;
}

void UvcSequenceNode::onPathTrackingCommand(
  const geometry_msgs::msg::TwistStamped & msg)
{
  path_tracking_command_ = msg;
  path_tracking_command_received_ = true;
  last_path_tracking_command_ = now();
}

void UvcSequenceNode::updateCommands()
{
  const auto timeout = static_cast<double>(command_timeout_ms_) / 1000.0;
  const bool inputs_fresh =
    robot_state_received_ && route_status_received_ &&
    path_tracking_command_received_ &&
    (!lidar_enabled_ || (lidar_safety_received_ && lidar_healthy_)) &&
    (now() - last_robot_state_).seconds() <= timeout &&
    (now() - last_route_status_).seconds() <= timeout &&
    (now() - last_path_tracking_command_).seconds() <= timeout &&
    (!lidar_enabled_ || (now() - last_lidar_safety_).seconds() <= timeout);
  const bool automatic =
    robot_state_.state_machine ==
    robosoft_interfaces::msg::RobotState::STATE_MAIN &&
    robot_state_.substate ==
    robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO &&
    !robot_state_.emergency_stop;
  const bool following =
    route_status_.state == robosoft_interfaces::msg::RouteStatus::STATE_FOLLOWING;

  if (!inputs_fresh || !automatic || !following) {
    if (uvc_sequence_state_ != UvcSequenceState::IDLE) {
      publishImplementCommand(
        robosoft_interfaces::msg::ImplementCommand::MODE_TRANSPORT);
    }
    uvc_sequence_state_ = UvcSequenceState::IDLE;
    publishStop();
    return;
  }

  if (route_status_.target_point.speed < 0.0) {
    drive_direction_ = -1.0;
    route_speed_limit_ms_ = std::abs(route_status_.target_point.speed);
  } else if (route_status_.target_point.speed > 0.0) {
    drive_direction_ = 1.0;
    route_speed_limit_ms_ = route_status_.target_point.speed;
  }

  if (uvc_sequence_enabled_ &&
    route_status_.target_point.implement_state ==
    robosoft_interfaces::msg::RoutePoint::IMPLEMENT_HEADLAND)
  {
    updateUvcSequence();
    if (uvc_sequence_state_ != UvcSequenceState::ADVANCE) {
      publishStop();
      return;
    }
    // The core tracker owns motion validity and publishes zero when any
    // required input, safety condition or operating state is invalid. Never
    // let the AKI-specific advance calculation bypass that guarded stop.
    if (std::abs(path_tracking_command_.twist.linear.x) <= 1e-6) {
      publishStop();
      return;
    }
  } else {
    if (uvc_sequence_state_ != UvcSequenceState::IDLE) {
      publishImplementCommand(
        robosoft_interfaces::msg::ImplementCommand::MODE_TRANSPORT);
    }
    uvc_sequence_state_ = UvcSequenceState::IDLE;
    auto command = path_tracking_command_;
    command.header.stamp = now();
    twist_pub_->publish(command);
    stopped_ = false;
    return;
  }

  // PathTracking remains the sole steering controller. ADVANCE only reduces
  // its speed according to the remaining treatment spacing. Scaling angular
  // velocity by the same ratio preserves the commanded path curvature.
  auto command = path_tracking_command_;
  command.header.stamp = now();
  const double incoming_speed = command.twist.linear.x;
  const double remaining =
    drive_direction_ * (uvc_advance_target_m_ - wheel_distance_m_);
  const double distance_speed_limit =
    remaining * uvc_distance_gain_ + uvc_minimum_speed_ms_;
  const double speed_magnitude = std::min({
    std::abs(incoming_speed), route_speed_limit_ms_, distance_speed_limit,
    lidar_maximum_speed_ms_});
  const double speed_ratio = speed_magnitude / std::abs(incoming_speed);
  command.twist.linear.x = std::copysign(speed_magnitude, incoming_speed);
  command.twist.angular.z *= speed_ratio;
  twist_pub_->publish(command);
  stopped_ = false;
}

void UvcSequenceNode::startUvcSequence()
{
  uvc_sequence_state_ = UvcSequenceState::WAIT_ACTIVE;
  publishImplementCommand(
    robosoft_interfaces::msg::ImplementCommand::MODE_HEADLAND);
  RCLCPP_INFO(get_logger(), "UVC sequence start requested");
}

void UvcSequenceNode::updateUvcSequence()
{
  switch (uvc_sequence_state_) {
    case UvcSequenceState::IDLE:
      startUvcSequence();
      break;
    case UvcSequenceState::WAIT_ACTIVE:
      if (implement_status_received_ && implement_status_.connected &&
        implement_status_.state ==
        robosoft_interfaces::msg::ImplementStatus::STATE_ON)
      {
        uvc_sequence_state_ = UvcSequenceState::WAIT_COMPLETE;
        RCLCPP_INFO(get_logger(), "UVC sequence started; holding position");
      }
      break;
    case UvcSequenceState::WAIT_COMPLETE:
      if (!implement_status_.connected) {
        break;
      }
      if (implement_status_.state ==
        robosoft_interfaces::msg::ImplementStatus::STATE_OFF)
      {
        uvc_advance_target_m_ =
          wheel_distance_m_ + drive_direction_ * uvc_advance_distance_m_;
        uvc_sequence_state_ = UvcSequenceState::ADVANCE;
        RCLCPP_INFO(
          get_logger(), "UVC sequence complete; advancing %.2f m",
          uvc_advance_distance_m_);
      }
      break;
    case UvcSequenceState::ADVANCE: {
        if (drive_direction_ *
          (uvc_advance_target_m_ - wheel_distance_m_) <= 0.0)
        {
          publishStop();
          startUvcSequence();
        }
        break;
      }
  }
}

void UvcSequenceNode::publishImplementCommand(uint8_t mode)
{
  robosoft_interfaces::msg::ImplementCommand command;
  command.mode = mode;
  command.timestamp = now().nanoseconds();
  implement_pub_->publish(command);
}

void UvcSequenceNode::publishStop()
{
  // Refresh zero commands at the control rate. This also replaces a previous
  // non-zero command inside TECU's periodic command window immediately.
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  twist_pub_->publish(command);
  stopped_ = true;
}

}  // namespace robosoft_aki

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_aki::UvcSequenceNode>());
  rclcpp::shutdown();
  return 0;
}
