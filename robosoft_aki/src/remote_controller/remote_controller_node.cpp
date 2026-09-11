// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "remote_controller_node.hpp"
#include "remote_protocol.hpp"
#include "robosoft_interfaces/topics.hpp"
#include "topics.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_aki
{

RemoteControllerNode::RemoteControllerNode(const rclcpp::NodeOptions & options)
: Node("remote_controller_node", options),
  last_controls_time_(0, 0, get_clock()->get_clock_type())
{
  source_address_ =
    static_cast<uint8_t>(declare_parameter<int>("source_address", 0x80));
  watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 300);
  const auto layout = declare_parameter<std::string>("layout", "aki");
  if (layout == "turo") {
    layout_ = TURO;
  } else if (layout != "aki") {
    RCLCPP_WARN(get_logger(), "Unknown remote layout '%s'; using aki",
                layout.c_str());
  }

  status_.layout = layout_;
  status_.deadman = layout_ == TURO;
  can_sub_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
    ros2_isobus::kBusRxTopic, 100,
    std::bind(&RemoteControllerNode::onCanFrame, this, std::placeholders::_1));
  status_pub_ =
    create_publisher<robosoft_interfaces::msg::RemoteControlStatus>(
      robosoft_interfaces::kRemoteControlStatusTopic,
      rclcpp::QoS(1).reliable().transient_local());
  joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
  watchdog_timer_ =
    create_wall_timer(100ms, std::bind(&RemoteControllerNode::checkWatchdog,
      this));

  publishStatus();
  RCLCPP_INFO(get_logger(),
              "Physical remote ready: layout=%s, SA=0x%02X, PGN=0xFF00-0xFF02",
              layout_ == AKI ? "aki" : "turo", source_address_);
}

void RemoteControllerNode::onCanFrame(
  const ros2_isobus::msg::IsobusFrame & frame)
{
  if (frame.sa != source_address_ || frame.pgn < 0xFF00 ||
    frame.pgn > 0xFF02 || (frame.dlc != 0 && frame.dlc < 8))
  {
    return;
  }
  if (frame.pgn == 0xFF00) {
    decodeAxes(frame);
  } else if (frame.pgn == 0xFF01) {
    if (layout_ == AKI) {
      decodeAkiControls(frame);
    } else {
      decodeTuroControls(frame);
    }
    controls_received_ = true;
    watchdog_reported_ = false;
    last_controls_time_ = now();
    status_.connected = true;
  } else {
    speed_ =
      static_cast<int16_t>(remote_protocol::getBits(frame.data, 6, 10));
  }
  publishStatus();
}

void RemoteControllerNode::decodeAxes(
  const ros2_isobus::msg::IsobusFrame & frame)
{
  joystick_1_x_ = remote_protocol::signedAxis(frame.data, 6, 4);
  joystick_1_y_ = remote_protocol::signedAxis(frame.data, 22, 20);
  joystick_2_x_ = remote_protocol::signedAxis(frame.data, 54, 52);
  joystick_2_y_ = remote_protocol::signedAxis(frame.data, 38, 36);
}

void RemoteControllerNode::decodeAkiControls(
  const ros2_isobus::msg::IsobusFrame & frame)
{
  using remote_protocol::getBits;
  status_.safety = getBits(frame.data, 0, 2) == 1;
  status_.start = getBits(frame.data, 2, 2) == 1;
  button_plus_ = getBits(frame.data, 6, 2) == 1;
  button_minus_ = getBits(frame.data, 8, 2) == 1;
  switch_1_ = getBits(frame.data, 10, 2) == 1 ? -1 :
    (getBits(frame.data, 12, 2) == 1 ? 1 : 0);
  status_.deadman = getBits(frame.data, 14, 2) == 1;
  status_.mode = robosoft_interfaces::msg::RemoteControlStatus::MODE_SAFE;
  constexpr unsigned mode_bits[] = {18, 20, 22, 24, 26, 32};
  for (uint8_t index = 0; index < 6; ++index) {
    if (getBits(frame.data, mode_bits[index], 2) == 1) {
      status_.mode = index + 1;
    }
  }
  switch_2_ = getBits(frame.data, 30, 2) == 1 ? 1 :
    (getBits(frame.data, 44, 2) == 1 ? -1 : 0);
  switch_3_ = 0;
  switch_4_ = 0;
}

void RemoteControllerNode::decodeTuroControls(
  const ros2_isobus::msg::IsobusFrame & frame)
{
  using remote_protocol::getBits;
  status_.safety = getBits(frame.data, 48, 2) == 1;
  status_.start = getBits(frame.data, 2, 2) == 1;
  status_.deadman = true;
  status_.mode = robosoft_interfaces::msg::RemoteControlStatus::MODE_SAFE;
  for (uint8_t index = 0; index < 6; ++index) {
    if (getBits(frame.data, 6 + index * 2, 2) == 1) {
      status_.mode = index + 1;
    }
  }
  const auto switch_value = [&](unsigned negative, unsigned positive) {
      return static_cast<int8_t>(
        getBits(frame.data, negative, 2) == 1 ? -1 :
        (getBits(frame.data, positive, 2) == 1 ? 1 : 0));
    };
  switch_1_ = switch_value(18, 20);
  switch_2_ = switch_value(22, 24);
  switch_3_ = switch_value(26, 28);
  switch_4_ = getBits(frame.data, 30, 2) == 1 ? 1 : -1;
  status_.rotary_select =
    static_cast<int32_t>(getBits(frame.data, 32, 2) * 10 +
    getBits(frame.data, 34, 2) * 20 +
    getBits(frame.data, 36, 2) * 40 +
    getBits(frame.data, 38, 2) +
    getBits(frame.data, 40, 2) * 2 +
    getBits(frame.data, 42, 2) * 4 +
    getBits(frame.data, 44, 2) * 8);
}

void RemoteControllerNode::checkWatchdog()
{
  if (!controls_received_ ||
    (now() - last_controls_time_).nanoseconds() <=
    static_cast<int64_t>(watchdog_timeout_ms_) * 1000000LL)
  {
    return;
  }
  status_.connected = false;
  status_.safety = false;
  status_.start = false;
  status_.deadman = false;
  status_.mode = robosoft_interfaces::msg::RemoteControlStatus::MODE_SAFE;
  if (!watchdog_reported_) {
    RCLCPP_ERROR(get_logger(), "Physical remote status timeout");
    watchdog_reported_ = true;
  }
  publishStatus();
}

void RemoteControllerNode::publishStatus()
{
  status_.timestamp = now().nanoseconds();
  status_pub_->publish(status_);
  sensor_msgs::msg::Joy joy;
  joy.header.stamp = now();
  joy.header.frame_id = "remote_controller";
  // Joy is a measurement of the physical controls. Keep the raw axes visible
  // while safety is released so diagnostics and calibration remain useful;
  // motion permission is carried separately in RemoteControlStatus::safety.
  const float connected = status_.connected ? 1.0F : 0.0F;
  joy.axes = {
    connected * joystick_1_x_ / 1000.0F,
    connected * joystick_1_y_ / 1000.0F,
    connected * joystick_2_x_ / 1000.0F,
    connected * joystick_2_y_ / 1000.0F,
    connected * speed_ / 1000.0F,
    connected * static_cast<float>(switch_1_),
    connected * static_cast<float>(switch_2_),
    connected * static_cast<float>(switch_3_),
    connected * static_cast<float>(switch_4_)};
  joy.buttons = {
    connected > 0.0F && button_plus_, connected > 0.0F && button_minus_};
  joy_pub_->publish(joy);
}

}  // namespace robosoft_aki

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_aki::RemoteControllerNode>());
  rclcpp::shutdown();
  return 0;
}
