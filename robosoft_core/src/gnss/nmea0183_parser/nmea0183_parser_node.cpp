// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "nmea0183_parser.hpp"
#include "robosoft_interfaces/topics.hpp"

using namespace std::chrono_literals;

namespace robosoft_core
{

class Nmea0183ParserNode final : public rclcpp::Node
{
public:
  explicit Nmea0183ParserNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nmea0183_parser_node", options)
  {
    message_timeout_ms_ = declare_parameter<int>("message_timeout_ms", 1500);
    sentence_sub_ = create_subscription<std_msgs::msg::String>(
      robosoft_interfaces::kNmea0183SentenceTopic, rclcpp::QoS(100),
      std::bind(
        &Nmea0183ParserNode::onSentence, this, std::placeholders::_1));
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kGnssFixTopic, rclcpp::SensorDataQoS());
    heading_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      robosoft_interfaces::kGnssHeadingTopic, rclcpp::SensorDataQoS());
    attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      robosoft_interfaces::kGnssAttitudeTopic, rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      robosoft_interfaces::kGnssImuTopic, rclcpp::SensorDataQoS());
    velocity_pub_ =
      create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      robosoft_interfaces::kGnssVelocityTopic, rclcpp::SensorDataQoS());
    watchdog_timer_ =
      create_wall_timer(500ms, std::bind(&Nmea0183ParserNode::watchdog, this));
  }

private:
  void onSentence(const std_msgs::msg::String & message)
  {
    const auto update = parser_.parse(message.data);
    if (!update) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid or unsupported NMEA0183 sentence received");
      return;
    }

    const auto stamp = now();
    last_message_ = stamp;
    have_message_ = true;
    if (update->uncertainty) {
      uncertainty_ = *update->uncertainty;
      have_uncertainty_ = true;
    }
    if (update->velocity) {
      velocity_ = *update->velocity;
      have_velocity_ = true;
      publishHeading(stamp);
      publishVelocity(stamp);
    }
    if (update->attitude) {
      attitude_ = *update->attitude;
      have_attitude_ = true;
      publishAttitude(stamp);
      publishHeading(stamp);
    }
    if (update->fix) {
      publishFix(*update->fix, stamp);
    }
  }

  void publishFix(const Nmea0183Fix & fix, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::NavSatFix output;
    output.header.stamp = stamp;
    output.header.frame_id = "gnss";
    output.latitude = fix.latitude;
    output.longitude = fix.longitude;
    output.altitude = fix.altitude;
    output.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    output.status.status =
      fix.fix_quality == 4 ?
      sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    output.position_covariance[0] =
      uncertainty_.longitude_stddev * uncertainty_.longitude_stddev;
    output.position_covariance[4] =
      uncertainty_.latitude_stddev * uncertainty_.latitude_stddev;
    output.position_covariance[8] =
      uncertainty_.altitude_stddev * uncertainty_.altitude_stddev;
    output.position_covariance_type =
      have_uncertainty_ ?
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN :
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(output);
  }

  void publishHeading(const rclcpp::Time & stamp)
  {
    if (!have_velocity_ && !have_attitude_) {
      return;
    }
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = "base_link";
    output.twist.linear.x = velocity_.speed_m_s;
    output.twist.angular.z =
      have_attitude_ ? attitude_.yaw :
      wrapAngle(-velocity_.compass_rad + M_PI / 2.0);
    heading_pub_->publish(output);
  }

  void publishAttitude(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::Vector3Stamped output;
    output.header.stamp = stamp;
    output.header.frame_id = "base_link";
    output.vector.x = attitude_.roll;
    output.vector.y = attitude_.pitch;
    output.vector.z = attitude_.yaw;
    attitude_pub_->publish(output);

    sensor_msgs::msg::Imu imu;
    imu.header = output.header;
    const double cr = std::cos(attitude_.roll * 0.5);
    const double sr = std::sin(attitude_.roll * 0.5);
    const double cp = std::cos(attitude_.pitch * 0.5);
    const double sp = std::sin(attitude_.pitch * 0.5);
    const double cy = std::cos(attitude_.yaw * 0.5);
    const double sy = std::sin(attitude_.yaw * 0.5);
    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
  }

  void publishVelocity(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = "odom";
    const double yaw = have_attitude_ ? attitude_.yaw :
      wrapAngle(-velocity_.compass_rad + M_PI / 2.0);
    output.twist.twist.linear.x = velocity_.speed_m_s * std::cos(yaw);
    output.twist.twist.linear.y = velocity_.speed_m_s * std::sin(yaw);
    velocity_pub_->publish(output);
  }

  void watchdog()
  {
    if (have_message_ &&
      (now() - last_message_).nanoseconds() >
      static_cast<int64_t>(message_timeout_ms_) * 1000000LL)
    {
      have_message_ = false;
      have_velocity_ = false;
      have_attitude_ = false;
      RCLCPP_ERROR(get_logger(), "NMEA0183 message timeout");
    }
  }

  static double wrapAngle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  int message_timeout_ms_{1500};
  bool have_message_{false};
  bool have_uncertainty_{false};
  bool have_velocity_{false};
  bool have_attitude_{false};
  rclcpp::Time last_message_{0, 0, RCL_ROS_TIME};
  Nmea0183Parser parser_;
  Nmea0183Uncertainty uncertainty_;
  Nmea0183Velocity velocity_;
  Nmea0183Attitude attitude_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sentence_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    velocity_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::Nmea0183ParserNode>());
  rclcpp::shutdown();
  return 0;
}
