// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robosoft_interfaces/msg/remote_control_status.hpp>
#include <ros2_isobus/msg/isobus_frame.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <string>

namespace robosoft_aki
{

/**
 * Decodes the physical remote controller CAN protocol used by AKI.
 *
 * The controller transmits joystick data in PGN 0xFF00, switches in 0xFF01
 * and the speed selector in 0xFF02. Missing 0xFF01 traffic fails safe through
 * a watchdog and publishes connected=false, safety=false.
 */
class RemoteControllerNode : public rclcpp::Node {
public:
  explicit RemoteControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum Layout : uint8_t { AKI = 0, TURO = 1 };

  void onCanFrame(const ros2_isobus::msg::IsobusFrame & frame);
  void decodeAxes(const ros2_isobus::msg::IsobusFrame & frame);
  void decodeAkiControls(const ros2_isobus::msg::IsobusFrame & frame);
  void decodeTuroControls(const ros2_isobus::msg::IsobusFrame & frame);
  void checkWatchdog();
  void publishStatus();

  rclcpp::Subscription<ros2_isobus::msg::IsobusFrame>::SharedPtr can_sub_;
  rclcpp::Publisher<robosoft_interfaces::msg::RemoteControlStatus>::SharedPtr
    status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  robosoft_interfaces::msg::RemoteControlStatus status_;
  int16_t joystick_1_x_{0};
  int16_t joystick_1_y_{0};
  int16_t joystick_2_x_{0};
  int16_t joystick_2_y_{0};
  int16_t speed_{0};
  int8_t switch_1_{0};
  int8_t switch_2_{0};
  int8_t switch_3_{0};
  int8_t switch_4_{0};
  bool button_plus_{false};
  bool button_minus_{false};
  Layout layout_{AKI};
  uint8_t source_address_{0x80};
  int watchdog_timeout_ms_{300};
  bool controls_received_{false};
  bool watchdog_reported_{false};
  rclcpp::Time last_controls_time_;
};

}  // namespace robosoft_aki
