// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robosoft_interfaces/msg/lidar_safety_status.hpp>

namespace robosoft_core
{

class LidarSafetyNode : public rclcpp::Node
{
public:
  explicit LidarSafetyNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2 & message);
  void checkTimeout();
  void publishUnavailable();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time last_cloud_;
  std::string target_frame_{"base_link"};
  double minimum_forward_distance_m_{0.4};
  double forward_corridor_half_width_m_{0.4};
  double forward_ignore_distance_m_{0.05};
  double side_forward_min_m_{-0.28};
  double side_forward_max_m_{0.35};
  double side_ignore_distance_m_{0.05};
  double speed_ramp_per_s_{0.5};
  double maximum_speed_m_s_{2.0};
  int cloud_timeout_ms_{500};
  int transform_timeout_ms_{50};
  bool cloud_received_{false};
  bool timeout_reported_{false};
};

}  // namespace robosoft_core
