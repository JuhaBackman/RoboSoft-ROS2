// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robosoft_interfaces/msg/landmark_detection_array.hpp>
#include <robosoft_interfaces/topics.hpp>

#include "lidar_object_detector.hpp"
#include "point_transform.hpp"

namespace robosoft_core
{

/// Converts an ordered planar point cloud into sensor-independent landmarks.
class LidarObjectDetectorNode final : public rclcpp::Node
{
public:
  explicit LidarObjectDetectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_object_detector_node", options)
  {
    target_frame_ = declare_parameter("target_frame", "base_link");
    transform_timeout_ms_ = declare_parameter("transform_timeout_ms", 50);
    if (target_frame_.empty() || transform_timeout_ms_ < 0) {
      throw std::invalid_argument(
              "target_frame must not be empty and transform_timeout_ms "
              "must not be negative");
    }
    const double edge_threshold =
      declare_parameter("lidar_edge_threshold", 0.20);
    const double object_size_min =
      declare_parameter("lidar_object_size_min", 0.02);
    const double object_size_max =
      declare_parameter("lidar_object_size_max", 0.08);

    detector_.setThresholds(
      edge_threshold, object_size_min, object_size_max);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = create_publisher<
      robosoft_interfaces::msg::LandmarkDetectionArray>(
      robosoft_interfaces::kLandmarkDetectionsTopic,
      rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(
        &LidarObjectDetectorNode::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (cloud.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring lidar cloud with an empty frame_id");
      return;
    }
    geometry_msgs::msg::TransformStamped sensor_to_target;
    try {
      sensor_to_target = tf_buffer_->lookupTransform(
        target_frame_, cloud.header.frame_id, cloud.header.stamp,
        rclcpp::Duration::from_seconds(
          static_cast<double>(transform_timeout_ms_) / 1000.0));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot transform lidar cloud from '%s' to '%s': %s",
        cloud.header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    std::vector<Point2D> scan;
    scan.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      for (; x != x.end(); ++x, ++y) {
        if (std::isfinite(*x) && std::isfinite(*y)) {
          scan.push_back({*x, *y});
        } else {
          scan.push_back({
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Lidar cloud has no usable x/y fields: %s", error.what());
      return;
    }

    robosoft_interfaces::msg::LandmarkDetectionArray output;
    output.header = cloud.header;
    output.header.frame_id = target_frame_;
    for (const auto & detection : detector_.detect(scan)) {
      const auto transformed = transformPoint(
        {detection.measurement.x, detection.measurement.y, 0.0},
        sensor_to_target.transform);
      robosoft_interfaces::msg::LandmarkDetection item;
      item.measurement.x = transformed.x;
      item.measurement.y = transformed.y;
      item.measurement.z = transformed.z;
      item.covariance_xx = detection.covariance[0];
      item.covariance_xy = detection.covariance[1];
      item.covariance_yx = detection.covariance[2];
      item.covariance_yy = detection.covariance[3];
      item.object_type = detection.object_type;
      item.confidence = detection.confidence;
      output.detections.push_back(std::move(item));
    }
    publisher_->publish(output);
  }

  int transform_timeout_ms_{50};
  std::string target_frame_;
  LidarObjectDetector detector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::LandmarkDetectionArray>::SharedPtr publisher_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::LidarObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
