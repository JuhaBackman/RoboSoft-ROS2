// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <robosoft_interfaces/msg/implement_command.hpp>
#include <robosoft_interfaces/msg/implement_status.hpp>
#include <ros2_isobus/msg/isobus_address_status.hpp>
#include <ros2_isobus/msg/isobus_frame.hpp>

#include "uvc_led_protocol.hpp"

namespace robosoft_aki
{

/**
 * AKI UVC/camera-light controller.
 *
 * An OFF->ON implement transition starts the controller's autonomous sequence
 * with proprietary PGN 0xFF30. Feedback PGN 0xFF31 stops command retries once
 * the hardware reports an active output. SAFE cancels pending transmission.
 */
class UvcLedNode : public rclcpp::Node
{
public:
  explicit UvcLedNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCommand(const robosoft_interfaces::msg::ImplementCommand & msg);
  void onFrame(const ros2_isobus::msg::IsobusFrame & msg);
  void onAddressStatus(const ros2_isobus::msg::IsobusAddressStatus & msg);
  void run();
  void publishCommand();
  void publishStatus();

  rclcpp::Subscription<
    robosoft_interfaces::msg::ImplementCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<ros2_isobus::msg::IsobusFrame>::SharedPtr frame_sub_;
  rclcpp::Subscription<
    ros2_isobus::msg::IsobusAddressStatus>::SharedPtr address_sub_;
  rclcpp::Publisher<ros2_isobus::msg::IsobusFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::ImplementStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  uvc_led_protocol::Command sequence_command_;
  robosoft_interfaces::msg::ImplementStatus status_;
  rclcpp::Time last_feedback_;
  uint8_t source_address_{0x1C};
  int status_source_address_{-1};
  int feedback_timeout_ms_{1000};
  uint8_t requested_mode_{3};
  bool address_received_{false};
  bool send_sequence_command_{false};
  bool feedback_received_{false};
  bool timeout_reported_{false};
};

}  // namespace robosoft_aki
