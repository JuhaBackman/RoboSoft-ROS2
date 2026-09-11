// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robosoft_core
{

/**
 * @brief GPS to Cartesian Node
 *
 * Converts WGS84 GPS coordinates to local cartesian XY coordinates.
 * Combines GPS position with NMEA2000 heading/speed data.
 *
 * Subscriptions:
 *   - gnss/fix (sensor_msgs/NavSatFix) - NMEA2000 GPS position
 *   - gnss/heading (geometry_msgs/TwistStamped) - NMEA2000 COG/SOG
 *
 * Publishers:
 *   - odometry (nav_msgs/Odometry)
 *   - /map_origin (sensor_msgs/NavSatFix, transient-local)
 *
 * The equirectangular projection uses the configured WGS84 reference. The
 * heading callback only updates the cached COG/SOG values; each GNSS fix
 * triggers one complete Odometry publication.
 */
class GpsToCartesianNode : public rclcpp::Node {
public:
  explicit GpsToCartesianNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GpsToCartesianNode();

private:
  // Input callbacks and ROS interfaces.
  void onGpsMessage(const sensor_msgs::msg::NavSatFix & msg);
  void onHeadingMessage(const geometry_msgs::msg::TwistStamped & msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr heading_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Projection origin and latest COG/SOG sample.
  double reference_latitude_;
  double reference_longitude_;
  double current_heading_ = 0.0;
  double current_speed_ = 0.0;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
};

}  // namespace robosoft_core
