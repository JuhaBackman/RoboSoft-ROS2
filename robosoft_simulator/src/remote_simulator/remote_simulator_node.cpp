// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_simulator/aki_can_devices.hpp"
#include "robosoft_simulator/can_protocol.hpp"
#include "robosoft_simulator/isobus_frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <robosoft_interfaces/msg/remote_control_status.hpp>
#include <ros2_isobus/msg/isobus_frame.hpp>
#include <topics.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace robosoft_simulator
{

class RemoteSimulatorNode : public rclcpp::Node
{
public:
  RemoteSimulatorNode()
  : Node("remote_simulator_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<int>("mode", 1);
    declare_parameter<bool>("safety", true);
    declare_parameter<bool>("start", false);
    declare_parameter<bool>("deadman", true);
    declare_parameter<int>("speed_selector", 500);
    declare_parameter<int>("joystick_1_x", 0);
    declare_parameter<int>("joystick_1_y", 0);
    declare_parameter<int>("joystick_2_x", 0);
    declare_parameter<int>("joystick_2_y", 0);
    use_joy_ = declare_parameter<bool>("use_joy", false);
    joy_timeout_s_ = std::max(
      0.1, declare_parameter<double>("joy_timeout_s", 0.5));
    axis_left_x_ = declare_parameter<int>("axis_left_x", 0);
    axis_left_y_ = declare_parameter<int>("axis_left_y", 1);
    axis_right_x_ = declare_parameter<int>("axis_right_x", 3);
    axis_right_y_ = declare_parameter<int>("axis_right_y", 4);
    axis_speed_ = declare_parameter<int>("axis_speed", 5);
    axis_left_x_sign_ = declare_parameter<double>("axis_left_x_sign", -1.0);
    axis_left_y_sign_ = declare_parameter<double>("axis_left_y_sign", 1.0);
    axis_right_x_sign_ = declare_parameter<double>("axis_right_x_sign", -1.0);
    axis_right_y_sign_ = declare_parameter<double>("axis_right_y_sign", 1.0);
    button_start_ = declare_parameter<int>("button_start", 0);
    button_mode_1_ = declare_parameter<int>("button_mode_1", 2);
    button_mode_2_ = declare_parameter<int>("button_mode_2", 1);
    button_mode_3_ = declare_parameter<int>("button_mode_3", 3);
    button_enable_left_ = declare_parameter<int>("button_enable_left", 4);
    button_deadman_ = declare_parameter<int>("button_deadman", 5);
    button_emergency_stop_share_ =
      declare_parameter<int>("button_emergency_stop_share", 8);
    button_emergency_stop_options_ =
      declare_parameter<int>("button_emergency_stop_options", 9);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&RemoteSimulatorNode::onJoy, this, std::placeholders::_1));
    local_status_pub_ =
      create_publisher<robosoft_interfaces::msg::RemoteControlStatus>(
      "simulator/remote/status", 10);
    local_joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      "simulator/remote/joy", rclcpp::SensorDataQoS());
    bus_tx_pub_ = create_publisher<ros2_isobus::msg::IsobusFrame>(
      ros2_isobus::kBusTxTopic, 200);
    bus_rx_sub_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
      ros2_isobus::kBusRxTopic, 100,
      std::bind(&RemoteSimulatorNode::onCanFrame, this, std::placeholders::_1));
    status_timer_ = create_wall_timer(100ms, std::bind(&RemoteSimulatorNode::publishStatus, this));
    claim_timer_ = create_wall_timer(1s, std::bind(&RemoteSimulatorNode::publishAddressClaim, this));
    publishAddressClaim();
    RCLCPP_INFO(get_logger(), "AKI physical remote simulator ready through ROS2ISOBUS");
  }

private:
  bool enabled() const
  {
    return get_parameter("enabled").as_bool();
  }

  int integerParameter(const char * name, int minimum, int maximum) const
  {
    return std::clamp<int>(get_parameter(name).as_int(), minimum, maximum);
  }

  static double axis(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < joy.axes.size() ?
      std::clamp(static_cast<double>(joy.axes[index]), -1.0, 1.0) : 0.0;
  }

  static bool button(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size() &&
      joy.buttons[index] != 0;
  }

  void onJoy(const sensor_msgs::msg::Joy & joy)
  {
    if (!use_joy_) return;
    joystick_1_x_ = static_cast<int>(std::lround(
      axis_left_x_sign_ * axis(joy, axis_left_x_) * 1000.0));
    joystick_1_y_ = static_cast<int>(std::lround(
      axis_left_y_sign_ * axis(joy, axis_left_y_) * 1000.0));
    joystick_2_x_ = static_cast<int>(std::lround(
      axis_right_x_sign_ * axis(joy, axis_right_x_) * 1000.0));
    joystick_2_y_ = static_cast<int>(std::lround(
      axis_right_y_sign_ * axis(joy, axis_right_y_) * 1000.0));
    speed_selector_ = static_cast<int>(std::lround(
      (1.0 - axis(joy, axis_speed_)) * 500.0));
    enable_left_ = button(joy, button_enable_left_);
    start_ = button(joy, button_start_);
    deadman_ = button(joy, button_deadman_);
    emergency_stop_ =
      button(joy, button_emergency_stop_share_) ||
      button(joy, button_emergency_stop_options_);
    if (button(joy, button_mode_1_)) {
      mode_ = robosoft_interfaces::msg::RemoteControlStatus::MODE_MANUAL;
    }
    if (button(joy, button_mode_2_)) {
      mode_ = robosoft_interfaces::msg::RemoteControlStatus::MODE_AUTO_1;
    }
    if (button(joy, button_mode_3_)) {
      mode_ = robosoft_interfaces::msg::RemoteControlStatus::MODE_AUTO_2;
    }
    joy_received_ = true;
    last_joy_time_ = now();
    if (!joy_layout_reported_) {
      RCLCPP_INFO(
        get_logger(), "Joystick connected: %zu axes, %zu buttons",
        joy.axes.size(), joy.buttons.size());
      joy_layout_reported_ = true;
    }
  }

  bool joyFresh() const
  {
    return use_joy_ && joy_received_ &&
      (now() - last_joy_time_).seconds() <= joy_timeout_s_;
  }

  void onCanFrame(const ros2_isobus::msg::IsobusFrame & message)
  {
    if (aki_can_devices::requestsAddressClaim(
        fromIsobusFrame(message), aki_can_devices::kRemoteSa))
    {
      publishAddressClaim();
    }
  }

  void publishStatus()
  {
    if (!enabled()) {
      return;
    }
    const bool use_live_joy = joyFresh();
    const int joystick_1_x = use_live_joy ? joystick_1_x_ :
      integerParameter("joystick_1_x", -1023, 1023);
    const int joystick_1_y = use_live_joy ? joystick_1_y_ :
      integerParameter("joystick_1_y", -1023, 1023);
    const int joystick_2_x = use_live_joy ? joystick_2_x_ :
      integerParameter("joystick_2_x", -1023, 1023);
    const int joystick_2_y = use_live_joy ? joystick_2_y_ :
      integerParameter("joystick_2_y", -1023, 1023);
    const int mode = use_live_joy ? mode_ : integerParameter("mode", 0, 6);
    // The AKI safety bit is the emergency-stop/safety-circuit OK signal,
    // not either of the momentary shoulder-button enable signals. A live
    // simulated remote represents a released emergency stop unless Share or
    // Options is held. Loss of joystick messages also opens the safety loop.
    const bool safety = use_joy_ ? (use_live_joy && !emergency_stop_) :
      get_parameter("safety").as_bool();
    const bool start = use_joy_ ? (use_live_joy && start_) :
      get_parameter("start").as_bool();
    const bool deadman = use_joy_ ? (use_live_joy && deadman_) :
      get_parameter("deadman").as_bool();
    const int speed_selector = use_live_joy ? speed_selector_ :
      integerParameter("speed_selector", 0, 1023);
    bus_tx_pub_->publish(toIsobusFrame(aki_can_devices::pdu2(
      0xFF00, aki_can_devices::kRemoteSa,
      can_protocol::encodeRemoteAxes(
        joystick_1_x, joystick_1_y, joystick_2_x, joystick_2_y))));
    bus_tx_pub_->publish(toIsobusFrame(aki_can_devices::pdu2(
      0xFF01, aki_can_devices::kRemoteSa,
      can_protocol::encodeAkiRemoteControls(
        static_cast<std::uint8_t>(mode), safety, start, deadman))));
    can_protocol::Payload speed_data{};
    can_protocol::setBits(
      speed_data, 6, 10,
      static_cast<std::uint32_t>(speed_selector));
    bus_tx_pub_->publish(toIsobusFrame(aki_can_devices::pdu2(
      0xFF02, aki_can_devices::kRemoteSa, speed_data)));
    publishLocalState(
      use_live_joy, joystick_1_x, joystick_1_y, joystick_2_x,
      joystick_2_y, speed_selector, mode, safety, start, deadman,
      use_live_joy && enable_left_);
  }

  void publishLocalState(
    bool connected, int joystick_1_x, int joystick_1_y,
    int joystick_2_x, int joystick_2_y, int speed_selector, int mode,
    bool safety, bool start, bool deadman, bool enable_left)
  {
    robosoft_interfaces::msg::RemoteControlStatus status;
    status.layout = robosoft_interfaces::msg::RemoteControlStatus::LAYOUT_AKI;
    status.mode = static_cast<std::uint8_t>(mode);
    status.connected = use_joy_ ? connected : enabled();
    status.safety = safety;
    status.start = start;
    status.deadman = deadman;
    status.timestamp = now().nanoseconds();
    local_status_pub_->publish(status);

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.header.frame_id = "simulated_remote";
    const float link = status.connected ? 1.0F : 0.0F;
    joy.axes = {
      link * joystick_1_x / 1000.0F,
      link * joystick_1_y / 1000.0F,
      link * joystick_2_x / 1000.0F,
      link * joystick_2_y / 1000.0F,
      link * speed_selector / 1000.0F};
    // Processed local controls used by the lower-level drive simulation:
    // start, left enable (L1), right enable/deadman (R1).
    joy.buttons = {status.start, enable_left, status.deadman};
    local_joy_pub_->publish(joy);
  }

  void publishAddressClaim()
  {
    if (!enabled()) {
      return;
    }
    bus_tx_pub_->publish(toIsobusFrame(can_protocol::addressClaim(
      aki_can_devices::kRemoteSa, aki_can_devices::kRemoteName)));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_rx_sub_;
  rclcpp::Publisher<ros2_isobus::msg::IsobusFrame>::SharedPtr bus_tx_pub_;
  rclcpp::Publisher<robosoft_interfaces::msg::RemoteControlStatus>::SharedPtr
    local_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr local_joy_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr claim_timer_;
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  bool use_joy_{false};
  bool joy_received_{false};
  bool joy_layout_reported_{false};
  bool enable_left_{false};
  bool start_{false};
  bool deadman_{false};
  bool emergency_stop_{false};
  int mode_{1};
  int joystick_1_x_{0};
  int joystick_1_y_{0};
  int joystick_2_x_{0};
  int joystick_2_y_{0};
  int speed_selector_{0};
  int axis_left_x_{0};
  int axis_left_y_{1};
  int axis_right_x_{3};
  int axis_right_y_{4};
  int axis_speed_{5};
  int button_start_{0};
  int button_mode_1_{2};
  int button_mode_2_{1};
  int button_mode_3_{3};
  int button_enable_left_{4};
  int button_deadman_{5};
  int button_emergency_stop_share_{8};
  int button_emergency_stop_options_{9};
  double axis_left_x_sign_{-1.0};
  double axis_left_y_sign_{1.0};
  double axis_right_x_sign_{-1.0};
  double axis_right_y_sign_{1.0};
  double joy_timeout_s_{0.5};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robosoft_simulator::RemoteSimulatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("remote_simulator_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
