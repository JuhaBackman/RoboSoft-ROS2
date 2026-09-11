// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_simulator
{

class VehicleModelNode : public rclcpp::Node
{
public:
  VehicleModelNode()
  : Node("vehicle_model_node"), last_update_(now())
  {
    speed_acceleration_response_s_ = std::max(
      0.0, declare_parameter<double>("speed_acceleration_response_s", 4.5));
    speed_deceleration_response_s_ = std::max(
      0.0, declare_parameter<double>("speed_deceleration_response_s", 1.35));
    curvature_response_s_ = std::max(
      0.0, declare_parameter<double>("curvature_response_s", 1.4));
    minimum_speed_m_s_ = declare_parameter<double>("minimum_speed_m_s", -1.2);
    maximum_speed_m_s_ = declare_parameter<double>("maximum_speed_m_s", 2.3);
    minimum_acceleration_m_s2_ =
      declare_parameter<double>("minimum_acceleration_m_s2", -5.2);
    maximum_acceleration_m_s2_ =
      declare_parameter<double>("maximum_acceleration_m_s2", 5.6);
    minimum_curvature_m_inv_ =
      declare_parameter<double>("minimum_curvature_m_inv", -0.6);
    maximum_curvature_m_inv_ =
      declare_parameter<double>("maximum_curvature_m_inv", 0.6);
    minimum_curvature_rate_m_inv_s_ =
      declare_parameter<double>("minimum_curvature_rate_m_inv_s", -0.28);
    maximum_curvature_rate_m_inv_s_ =
      declare_parameter<double>("maximum_curvature_rate_m_inv_s", 0.28);
    command_delay_s_ = std::max(
      0.0, declare_parameter<double>("command_delay_s", 0.1));
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    x_m_ = declare_parameter<double>("initial_x_m", 0.0);
    y_m_ = declare_parameter<double>("initial_y_m", 0.0);
    yaw_ = declare_parameter<double>("initial_yaw_rad", 0.0);

    automatic_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "vehicle/command", 10,
      std::bind(&VehicleModelNode::onAutomaticCommand, this, std::placeholders::_1));
    manual_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "vehicle/manual_command", 10,
      std::bind(&VehicleModelNode::onManualCommand, this, std::placeholders::_1));
    manual_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "vehicle/manual_active", 10,
      [this](const std_msgs::msg::Bool & active) {
        manual_active_ = active.data;
        last_manual_active_ = now();
      });
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("vehicle/odometry", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist", 10);
    update_timer_ = create_wall_timer(20ms, std::bind(&VehicleModelNode::update, this));
    RCLCPP_INFO(get_logger(), "AKI kinematic vehicle model ready");
  }

private:
  static void decodeCommand(
    const geometry_msgs::msg::TwistStamped & command,
    double & speed, double & curvature)
  {
    speed = command.twist.linear.x;
    curvature = std::abs(speed) > 1e-6 ? command.twist.angular.z / speed : 0.0;
  }

  void onAutomaticCommand(const geometry_msgs::msg::TwistStamped & command)
  {
    CommandSample sample;
    decodeCommand(command, sample.speed_m_s, sample.curvature_m_inv);
    sample.received = now();
    automatic_commands_.push_back(sample);
    if (automatic_commands_.size() > 100U) automatic_commands_.pop_front();
    last_automatic_command_ = sample.received;
    automatic_command_received_ = true;
  }

  void onManualCommand(const geometry_msgs::msg::TwistStamped & command)
  {
    CommandSample sample;
    decodeCommand(command, sample.speed_m_s, sample.curvature_m_inv);
    sample.received = now();
    manual_commands_.push_back(sample);
    if (manual_commands_.size() > 100U) manual_commands_.pop_front();
    last_manual_command_ = sample.received;
    manual_command_received_ = true;
  }

  void update()
  {
    const auto current_time = now();
    const double dt = std::clamp((current_time - last_update_).seconds(), 0.0, 0.1);
    last_update_ = current_time;
    applyDelayedCommand(automatic_commands_, current_time, automatic_speed_m_s_,
      automatic_curvature_m_inv_);
    applyDelayedCommand(manual_commands_, current_time, manual_speed_m_s_,
      manual_curvature_m_inv_);
    const bool manual_fresh = manual_active_ && manual_command_received_ &&
      (current_time - last_manual_active_).seconds() <= command_timeout_s_ &&
      (current_time - last_manual_command_).seconds() <= command_timeout_s_;
    const bool automatic_fresh = automatic_command_received_ &&
      (current_time - last_automatic_command_).seconds() <= command_timeout_s_;
    if (manual_fresh) {
      target_speed_m_s_ = manual_speed_m_s_;
      target_curvature_m_inv_ = manual_curvature_m_inv_;
    } else if (automatic_fresh) {
      target_speed_m_s_ = automatic_speed_m_s_;
      target_curvature_m_inv_ = automatic_curvature_m_inv_;
    } else {
      target_speed_m_s_ = 0.0;
      target_curvature_m_inv_ = 0.0;
    }
    target_speed_m_s_ = std::clamp(
      target_speed_m_s_, minimum_speed_m_s_, maximum_speed_m_s_);
    target_curvature_m_inv_ = std::clamp(
      target_curvature_m_inv_, minimum_curvature_m_inv_, maximum_curvature_m_inv_);

    const bool decelerating =
      speed_m_s_ * target_speed_m_s_ < 0.0 ||
      std::abs(target_speed_m_s_) < std::abs(speed_m_s_);
    const double speed_tau = decelerating ?
      speed_deceleration_response_s_ : speed_acceleration_response_s_;
    speed_m_s_ = updateFirstOrder(
      speed_m_s_, target_speed_m_s_, speed_tau,
      minimum_acceleration_m_s2_, maximum_acceleration_m_s2_, dt);
    curvature_m_inv_ = updateFirstOrder(
      curvature_m_inv_, target_curvature_m_inv_, curvature_response_s_,
      minimum_curvature_rate_m_inv_s_, maximum_curvature_rate_m_inv_s_, dt);
    speed_m_s_ = std::clamp(speed_m_s_, minimum_speed_m_s_, maximum_speed_m_s_);
    curvature_m_inv_ = std::clamp(
      curvature_m_inv_, minimum_curvature_m_inv_, maximum_curvature_m_inv_);
    yaw_ += speed_m_s_ * curvature_m_inv_ * dt;
    x_m_ += speed_m_s_ * std::cos(yaw_) * dt;
    y_m_ += speed_m_s_ * std::sin(yaw_) * dt;
    publishOdometry(current_time);
  }

  struct CommandSample
  {
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    double speed_m_s{0.0};
    double curvature_m_inv{0.0};
  };

  void applyDelayedCommand(
    std::deque<CommandSample> & commands, const rclcpp::Time & current_time,
    double & speed, double & curvature)
  {
    const auto cutoff = current_time - rclcpp::Duration::from_seconds(command_delay_s_);
    while (!commands.empty() && commands.front().received <= cutoff) {
      speed = commands.front().speed_m_s;
      curvature = commands.front().curvature_m_inv;
      commands.pop_front();
    }
  }

  static double updateFirstOrder(
    double current, double target, double time_constant,
    double minimum_rate, double maximum_rate, double dt)
  {
    if (dt <= 0.0) return current;
    const double requested_rate = time_constant <= 0.0 ?
      (target - current) / dt : (target - current) / time_constant;
    const double rate = std::clamp(requested_rate, minimum_rate, maximum_rate);
    const double next = current + rate * dt;
    if ((target - current) * (target - next) <= 0.0) return target;
    return next;
  }

  void publishOdometry(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = x_m_;
    odometry.pose.pose.position.y = y_m_;
    odometry.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    odometry.twist.twist.linear.x = speed_m_s_;
    odometry.twist.twist.angular.z = speed_m_s_ * curvature_m_inv_;
    odometry_pub_->publish(odometry);
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = child_frame_id_;
    twist.twist = odometry.twist.twist;
    twist_pub_->publish(twist);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    automatic_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    manual_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_active_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time last_update_;
  rclcpp::Time last_automatic_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_manual_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_manual_active_{0, 0, RCL_ROS_TIME};
  std::string frame_id_;
  std::string child_frame_id_;
  bool automatic_command_received_{false};
  bool manual_command_received_{false};
  bool manual_active_{false};
  std::deque<CommandSample> automatic_commands_;
  std::deque<CommandSample> manual_commands_;
  double speed_acceleration_response_s_{4.5};
  double speed_deceleration_response_s_{1.35};
  double curvature_response_s_{1.4};
  double minimum_speed_m_s_{-1.2};
  double maximum_speed_m_s_{2.3};
  double minimum_acceleration_m_s2_{-5.2};
  double maximum_acceleration_m_s2_{5.6};
  double minimum_curvature_m_inv_{-0.6};
  double maximum_curvature_m_inv_{0.6};
  double minimum_curvature_rate_m_inv_s_{-0.28};
  double maximum_curvature_rate_m_inv_s_{0.28};
  double command_delay_s_{0.1};
  double command_timeout_s_{0.5};
  double target_speed_m_s_{0.0};
  double target_curvature_m_inv_{0.0};
  double automatic_speed_m_s_{0.0};
  double automatic_curvature_m_inv_{0.0};
  double manual_speed_m_s_{0.0};
  double manual_curvature_m_inv_{0.0};
  double speed_m_s_{0.0};
  double curvature_m_inv_{0.0};
  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_{0.0};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_simulator::VehicleModelNode>());
  rclcpp::shutdown();
  return 0;
}
