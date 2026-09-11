// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "lidar_safety_node.hpp"

#include "lidar_safety.hpp"
#include "robosoft_interfaces/topics.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

namespace robosoft_core
{

LidarSafetyNode::LidarSafetyNode(const rclcpp::NodeOptions & options)
: Node("lidar_safety_node", options),
  last_cloud_(0, 0, get_clock()->get_clock_type())
{
  minimum_forward_distance_m_ =
    declare_parameter<double>("minimum_forward_distance_m", 0.4);
  forward_corridor_half_width_m_ =
    declare_parameter<double>("forward_corridor_half_width_m", 0.4);
  forward_ignore_distance_m_ =
    declare_parameter<double>("forward_ignore_distance_m", 0.05);
  side_forward_min_m_ =
    declare_parameter<double>("side_forward_min_m", -0.28);
  side_forward_max_m_ =
    declare_parameter<double>("side_forward_max_m", 0.35);
  side_ignore_distance_m_ =
    declare_parameter<double>("side_ignore_distance_m", 0.05);
  speed_ramp_per_s_ = declare_parameter<double>("speed_ramp_per_s", 0.5);
  maximum_speed_m_s_ = declare_parameter<double>("maximum_speed_m_s", 2.0);
  cloud_timeout_ms_ = declare_parameter<int>("cloud_timeout_ms", 500);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS(),
    std::bind(
      &LidarSafetyNode::onPointCloud, this, std::placeholders::_1));
  status_pub_ =
    create_publisher<robosoft_interfaces::msg::LidarSafetyStatus>(
    robosoft_interfaces::kLidarSafetyStatusTopic,
    rclcpp::QoS(1).reliable().transient_local());
  timeout_timer_ =
    create_wall_timer(100ms, std::bind(&LidarSafetyNode::checkTimeout, this));

  publishUnavailable();
  RCLCPP_INFO(
    get_logger(),
    "Lidar safety ready: cloud timeout=%d ms, stop distance=%.2f m",
    cloud_timeout_ms_, minimum_forward_distance_m_);
}

void LidarSafetyNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2 & message)
{
  std::vector<LidarPoint2D> points;
  points.reserve(
    static_cast<std::size_t>(message.width) *
    static_cast<std::size_t>(message.height));
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    for (; x != x.end(); ++x, ++y) {
      points.push_back({static_cast<double>(*x), static_cast<double>(*y)});
    }
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Invalid lidar PointCloud2: %s", error.what());
    publishUnavailable();
    return;
  }

  LidarSafetyParameters parameters;
  parameters.minimum_forward_distance_m = minimum_forward_distance_m_;
  parameters.forward_corridor_half_width_m = forward_corridor_half_width_m_;
  parameters.forward_ignore_distance_m = forward_ignore_distance_m_;
  parameters.side_forward_min_m = side_forward_min_m_;
  parameters.side_forward_max_m = side_forward_max_m_;
  parameters.side_ignore_distance_m = side_ignore_distance_m_;
  parameters.speed_ramp_per_s = speed_ramp_per_s_;
  parameters.maximum_speed_m_s = maximum_speed_m_s_;
  const auto result = calculateLidarSafety(points, parameters);

  robosoft_interfaces::msg::LidarSafetyStatus status;
  status.timestamp = now().nanoseconds();
  status.healthy = true;
  status.closest_forward_m = result.closest_forward_m;
  status.closest_lateral_m = result.closest_lateral_m;
  status.maximum_speed_m_s = result.maximum_speed_m_s;
  status_pub_->publish(status);

  last_cloud_ = now();
  cloud_received_ = true;
  timeout_reported_ = false;
}

void LidarSafetyNode::checkTimeout()
{
  const bool timed_out =
    !cloud_received_ ||
    (now() - last_cloud_).nanoseconds() >
    static_cast<std::int64_t>(cloud_timeout_ms_) * 1000000LL;
  if (!timed_out) {
    return;
  }

  publishUnavailable();
  if (cloud_received_ && !timeout_reported_) {
    RCLCPP_ERROR(get_logger(), "SICK lidar point cloud timeout");
    timeout_reported_ = true;
  }
}

void LidarSafetyNode::publishUnavailable()
{
  robosoft_interfaces::msg::LidarSafetyStatus status;
  status.timestamp = now().nanoseconds();
  status.healthy = false;
  status.closest_forward_m = std::numeric_limits<double>::infinity();
  status.closest_lateral_m = std::numeric_limits<double>::infinity();
  status.maximum_speed_m_s = 0.0;
  status_pub_->publish(status);
}

}  // namespace robosoft_core

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::LidarSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
