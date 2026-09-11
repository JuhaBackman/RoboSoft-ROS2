// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "uvc_led_node.hpp"

#include "robosoft_interfaces/topics.hpp"
#include "topics.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_aki
{

UvcLedNode::UvcLedNode(const rclcpp::NodeOptions & options)
: Node("uvc_led_node", options),
  last_feedback_(0, 0, get_clock()->get_clock_type())
{
  sequence_command_.camera_light_time_ds = static_cast<uint8_t>(
    declare_parameter<int>("camera_light_time_ds", 40));
  sequence_command_.camera_trigger_time_ds = static_cast<uint8_t>(
    declare_parameter<int>("camera_trigger_time_ds", 30));
  sequence_command_.camera_light_power = static_cast<uint16_t>(
    declare_parameter<int>("camera_light_power", 500));
  sequence_command_.uvc_light_time_ds = static_cast<uint8_t>(
    declare_parameter<int>("uvc_light_time_ds", 200));
  sequence_command_.uvc_light_distance_mm = static_cast<uint16_t>(
    declare_parameter<int>("uvc_light_distance_mm", 150));
  status_source_address_ =
    declare_parameter<int>("status_source_address", -1);
  feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 1000);

  command_sub_ =
    create_subscription<robosoft_interfaces::msg::ImplementCommand>(
    robosoft_interfaces::kImplementCommandTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UvcLedNode::onCommand, this, std::placeholders::_1));
  frame_sub_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
    ros2_isobus::kBusRxTopic, 100,
    std::bind(&UvcLedNode::onFrame, this, std::placeholders::_1));
  address_sub_ =
    create_subscription<ros2_isobus::msg::IsobusAddressStatus>(
    ros2_isobus::kAddressManagerStatus,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UvcLedNode::onAddressStatus, this, std::placeholders::_1));
  frame_pub_ = create_publisher<ros2_isobus::msg::IsobusFrame>(
    ros2_isobus::kBusTxTopic, 20);
  status_pub_ =
    create_publisher<robosoft_interfaces::msg::ImplementStatus>(
    robosoft_interfaces::kImplementStatusTopic,
    rclcpp::QoS(1).reliable().transient_local());
  timer_ = create_wall_timer(100ms, std::bind(&UvcLedNode::run, this));

  status_.state = robosoft_interfaces::msg::ImplementStatus::STATE_SAFE;
  publishStatus();
  RCLCPP_INFO(
    get_logger(), "AKI UVC/LED controller ready (PGN 0xFF30/0xFF31)");
}

void UvcLedNode::onCommand(
  const robosoft_interfaces::msg::ImplementCommand & msg)
{
  if (msg.mode > 3) {
    RCLCPP_WARN(get_logger(), "Unsupported AKI implement mode %u", msg.mode);
    return;
  }

  // The device has OFF=0, ON=1 and SAFE=3. A shared route containing mode 2
  // is therefore handled as OFF rather than interpreted as an unsafe command.
  requested_mode_ =
    msg.mode == robosoft_interfaces::msg::ImplementCommand::MODE_WORKING ?
    robosoft_interfaces::msg::ImplementCommand::MODE_TRANSPORT : msg.mode;
  if (msg.mode == robosoft_interfaces::msg::ImplementCommand::MODE_SAFE) {
    send_sequence_command_ = false;
    status_.state = robosoft_interfaces::msg::ImplementStatus::STATE_SAFE;
  } else if (
    msg.mode == robosoft_interfaces::msg::ImplementCommand::MODE_HEADLAND &&
    status_.state != robosoft_interfaces::msg::ImplementStatus::STATE_ON)
  {
    send_sequence_command_ = true;
    timeout_reported_ = false;
    RCLCPP_INFO(get_logger(), "UVC/LED sequence requested");
  }
  publishStatus();
}

void UvcLedNode::onFrame(const ros2_isobus::msg::IsobusFrame & msg)
{
  if (msg.pgn != uvc_led_protocol::kStatusPgn ||
    (status_source_address_ >= 0 &&
    msg.sa != static_cast<uint8_t>(status_source_address_)) ||
    (msg.dlc != 0 && msg.dlc < 7))
  {
    return;
  }

  const auto feedback = uvc_led_protocol::decodeStatus(msg.data);
  feedback_received_ = true;
  timeout_reported_ = false;
  last_feedback_ = now();
  status_.connected = true;
  status_.camera_light_on = feedback.camera_light_on;
  status_.camera_trigger_on = feedback.camera_trigger_on;
  status_.uvc_light_on = feedback.uvc_light_on;
  status_.uvc_distance_front_mm = feedback.uvc_distance_front_mm;
  status_.uvc_distance_back_mm = feedback.uvc_distance_back_mm;
  status_.time_on_ds = feedback.time_on_ds;
  status_.state = feedback.active() ? 1 : 0;

  if (feedback.active()) {
    send_sequence_command_ = false;
  }
  publishStatus();
}

void UvcLedNode::onAddressStatus(
  const ros2_isobus::msg::IsobusAddressStatus & msg)
{
  source_address_ = msg.sa;
  address_received_ = true;
}

void UvcLedNode::run()
{
  if (send_sequence_command_ &&
    requested_mode_ == robosoft_interfaces::msg::ImplementCommand::MODE_HEADLAND &&
    address_received_)
  {
    publishCommand();
  }

  if (feedback_received_ &&
    (now() - last_feedback_).nanoseconds() >
    static_cast<int64_t>(feedback_timeout_ms_) * 1000000LL)
  {
    send_sequence_command_ = false;
    status_.connected = false;
    status_.state = robosoft_interfaces::msg::ImplementStatus::STATE_SAFE;
    if (!timeout_reported_) {
      RCLCPP_ERROR(get_logger(), "UVC/LED feedback timeout");
      timeout_reported_ = true;
    }
    publishStatus();
  }
}

void UvcLedNode::publishCommand()
{
  ros2_isobus::msg::IsobusFrame frame;
  frame.timestamp = now();
  frame.priority = 3;
  frame.page = false;
  frame.pgn = uvc_led_protocol::kCommandPgn;
  frame.sa = source_address_;
  frame.pf = 0xFF;
  frame.ps = 0x30;
  frame.dlc = 8;
  frame.data = uvc_led_protocol::encodeCommand(sequence_command_);
  frame_pub_->publish(frame);
}

void UvcLedNode::publishStatus()
{
  status_.timestamp = now().nanoseconds();
  status_pub_->publish(status_);
}

}  // namespace robosoft_aki

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_aki::UvcLedNode>());
  rclcpp::shutdown();
  return 0;
}
