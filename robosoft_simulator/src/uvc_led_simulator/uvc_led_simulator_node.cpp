// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_simulator/aki_can_devices.hpp"
#include "robosoft_simulator/can_protocol.hpp"
#include "robosoft_simulator/isobus_frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ros2_isobus/msg/isobus_frame.hpp>
#include <topics.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace robosoft_simulator
{

class UvcLedSimulatorNode : public rclcpp::Node
{
public:
  UvcLedSimulatorNode()
  : Node("uvc_led_simulator_node")
  {
    declare_parameter<bool>("enabled", true);
    sequence_duration_s_ = declare_parameter<double>("sequence_duration_s", 20.0);
    bus_tx_pub_ = create_publisher<ros2_isobus::msg::IsobusFrame>(
      ros2_isobus::kBusTxTopic, 200);
    bus_rx_sub_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
      ros2_isobus::kBusRxTopic, 100,
      std::bind(&UvcLedSimulatorNode::onCanFrame, this, std::placeholders::_1));
    status_timer_ = create_wall_timer(100ms, std::bind(&UvcLedSimulatorNode::publishStatus, this));
    claim_timer_ = create_wall_timer(1s, std::bind(&UvcLedSimulatorNode::publishAddressClaim, this));
    publishAddressClaim();
    RCLCPP_INFO(get_logger(), "AKI UVC/LED simulator ready through ROS2ISOBUS");
  }

private:
  bool enabled() const
  {
    return get_parameter("enabled").as_bool();
  }

  void onCanFrame(const ros2_isobus::msg::IsobusFrame & message)
  {
    const auto frame = fromIsobusFrame(message);
    if (aki_can_devices::requestsAddressClaim(frame, aki_can_devices::kUvcSa)) {
      publishAddressClaim();
    } else if (frame.pgn() == 0xFF30U && enabled()) {
      if (!active_) {
        active_ = true;
        started_ = now();
        RCLCPP_INFO(get_logger(), "UVC/LED sequence command received");
      }
    }
  }

  void publishStatus()
  {
    if (!enabled()) {
      return;
    }
    const double elapsed = started_.nanoseconds() == 0 ? 0.0 :
      std::max(0.0, (now() - started_).seconds());
    if (active_ && elapsed >= sequence_duration_s_) {
      active_ = false;
      RCLCPP_INFO(get_logger(), "UVC/LED sequence completed");
    }
    bus_tx_pub_->publish(toIsobusFrame(aki_can_devices::pdu2(
        0xFF31, aki_can_devices::kUvcSa,
        can_protocol::encodeUvcStatus(active_, elapsed))));
  }

  void publishAddressClaim()
  {
    if (!enabled()) {
      return;
    }
    bus_tx_pub_->publish(toIsobusFrame(can_protocol::addressClaim(
      aki_can_devices::kUvcSa, aki_can_devices::kUvcName)));
  }

  rclcpp::Subscription<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_rx_sub_;
  rclcpp::Publisher<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_tx_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr claim_timer_;
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  bool active_{false};
  double sequence_duration_s_{20.0};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robosoft_simulator::UvcLedSimulatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("uvc_led_simulator_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
