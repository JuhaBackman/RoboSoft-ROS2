// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robosoft_interfaces/msg/remote_control_status.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_simulator
{

/** Simulates AKI's lower-level manual controller from the local Joy topic. */
class AkiDriveControllerSimulatorNode final : public rclcpp::Node
{
public:
  AkiDriveControllerSimulatorNode()
  : Node("aki_drive_controller_simulator_node"),
    last_joy_(0, 0, get_clock()->get_clock_type())
  {
    joy_timeout_s_ = std::max(0.1, declare_parameter<double>("joy_timeout_s", 0.5));
    maximum_speed_m_s_ = std::max(0.0, declare_parameter<double>("maximum_speed_m_s", 2.0));
    maximum_curvature_m_inv_ = std::max(
      0.0, declare_parameter<double>("maximum_curvature_m_inv", 1.0));
    steering_sign_ = std::clamp(
      declare_parameter<double>("steering_sign", -1.0), -1.0, 1.0);
    deadzone_ = std::clamp(
      declare_parameter<double>("axis_deadzone", 0.05), 0.0, 0.5);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "simulator/remote/joy", rclcpp::SensorDataQoS(),
      std::bind(&AkiDriveControllerSimulatorNode::onJoy, this, std::placeholders::_1));
    status_sub_ =
      create_subscription<robosoft_interfaces::msg::RemoteControlStatus>(
      "simulator/remote/status", 10,
      std::bind(
        &AkiDriveControllerSimulatorNode::onStatus, this,
        std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "vehicle/manual_command", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("vehicle/manual_active", 10);
    command_timer_ = create_wall_timer(
      50ms, std::bind(&AkiDriveControllerSimulatorNode::publishCommand, this));
    RCLCPP_INFO(
      get_logger(),
      "AKI lower-level drive controller simulator ready on processed remote topics");
  }

private:
  static double axis(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < joy.axes.size() ?
      std::clamp(static_cast<double>(joy.axes[index]), -1.0, 1.0) : 0.0;
  }

  static double applyDeadzone(double value, double deadzone)
  {
    if (std::abs(value) <= deadzone) return 0.0;
    return std::copysign(
      (std::abs(value) - deadzone) / (1.0 - deadzone), value);
  }

  static bool button(const sensor_msgs::msg::Joy & joy, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size() &&
      joy.buttons[index] != 0;
  }

  void onJoy(const sensor_msgs::msg::Joy & joy)
  {
    // RemoteSimulator publishes axes in the same order and direction as the
    // decoded physical AKI remote: J1 X/Y, J2 X/Y and speed selector.
    throttle_ = applyDeadzone(axis(joy, 1), deadzone_);
    steering_ = applyDeadzone(axis(joy, 2), deadzone_);
    enable_left_ = button(joy, 1);
    enable_right_ = button(joy, 2);
    joy_received_ = true;
    last_joy_ = now();
  }

  void onStatus(const robosoft_interfaces::msg::RemoteControlStatus & status)
  {
    remote_status_ = status;
    status_received_ = true;
    last_status_ = now();
  }

  void publishCommand()
  {
    const bool inputs_fresh = joy_received_ && status_received_ &&
      (now() - last_joy_).seconds() <= joy_timeout_s_ &&
      (now() - last_status_).seconds() <= joy_timeout_s_;
    const bool manual_mode = inputs_fresh && remote_status_.connected &&
      remote_status_.safety &&
      remote_status_.mode ==
      robosoft_interfaces::msg::RemoteControlStatus::MODE_MANUAL;
    const bool joystick_active = throttle_ != 0.0 || steering_ != 0.0;
    if (!manual_mode || !joystick_active) {
      manual_armed_ = false;
    } else if (!manual_armed_ && (enable_left_ || enable_right_))
    {
      // The real lower-level controller requires either enable button only
      // when joystick motion begins. Continued joystick motion keeps control
      // armed after the button is released.
      manual_armed_ = true;
    }
    const bool manual_active = manual_mode && joystick_active && manual_armed_;

    std_msgs::msg::Bool active;
    active.data = manual_active;
    active_pub_->publish(active);

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    if (manual_active) {
      command.twist.linear.x = throttle_ * maximum_speed_m_s_;
      const double curvature =
        steering_sign_ * steering_ * maximum_curvature_m_inv_;
      command.twist.angular.z = command.twist.linear.x * curvature;
    }
    command_pub_->publish(command);

    if (manual_active != manual_active_reported_) {
      RCLCPP_INFO(
        get_logger(), "Manual drive %s", manual_active ? "enabled" : "disabled");
      manual_active_reported_ = manual_active;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RemoteControlStatus>::SharedPtr
    status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::Time last_joy_;
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
  robosoft_interfaces::msg::RemoteControlStatus remote_status_;
  bool joy_received_{false};
  bool status_received_{false};
  bool manual_armed_{false};
  bool manual_active_reported_{false};
  bool enable_left_{false};
  bool enable_right_{false};
  double joy_timeout_s_{0.5};
  double maximum_speed_m_s_{2.0};
  double maximum_curvature_m_inv_{1.0};
  double steering_sign_{-1.0};
  double deadzone_{0.05};
  double throttle_{0.0};
  double steering_{0.0};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<robosoft_simulator::AkiDriveControllerSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
