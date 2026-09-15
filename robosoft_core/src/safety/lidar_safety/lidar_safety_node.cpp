// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "lidar_safety_node.hpp"

#include "lidar_safety.hpp"
#include "point_transform.hpp"
#include "robosoft_interfaces/topics.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>

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
  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  transform_timeout_ms_ = declare_parameter<int>("transform_timeout_ms", 50);
  if (!std::isfinite(minimum_forward_distance_m_) ||
    !std::isfinite(forward_corridor_half_width_m_) ||
    !std::isfinite(forward_ignore_distance_m_) ||
    !std::isfinite(side_forward_min_m_) ||
    !std::isfinite(side_forward_max_m_) ||
    !std::isfinite(side_ignore_distance_m_) ||
    !std::isfinite(speed_ramp_per_s_) ||
    !std::isfinite(maximum_speed_m_s_) ||
    forward_corridor_half_width_m_ <= 0.0 ||
    forward_ignore_distance_m_ >= minimum_forward_distance_m_ ||
    side_forward_min_m_ >= side_forward_max_m_ ||
    side_ignore_distance_m_ < 0.0 || speed_ramp_per_s_ < 0.0 ||
    maximum_speed_m_s_ < 0.0 || cloud_timeout_ms_ <= 0 ||
    target_frame_.empty() || transform_timeout_ms_ < 0)
  {
    throw std::invalid_argument("Invalid lidar safety geometry or timing parameters");
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
    std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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
  if (message.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring lidar cloud with an empty frame_id");
    publishUnavailable();
    return;
  }
  geometry_msgs::msg::TransformStamped sensor_to_target;
  try {
    sensor_to_target = tf_buffer_->lookupTransform(
      target_frame_, message.header.frame_id, message.header.stamp,
      rclcpp::Duration::from_seconds(
        static_cast<double>(transform_timeout_ms_) / 1000.0));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot transform lidar cloud from '%s' to '%s': %s",
      message.header.frame_id.c_str(), target_frame_.c_str(), error.what());
    publishUnavailable();
    return;
  }

  std::vector<LidarPoint2D> points;
  points.reserve(
    static_cast<std::size_t>(message.width) *
    static_cast<std::size_t>(message.height));
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    for (; x != x.end(); ++x, ++y) {
      const auto transformed = transformPoint(
        {static_cast<double>(*x), static_cast<double>(*y), 0.0},
        sensor_to_target.transform);
      points.push_back({transformed.x, transformed.y});
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
