// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "lidar_model.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace robosoft_simulator
{

/// Generates a TiM5xx-shaped PointCloud2 from TASK point obstacles.
class TaskLidarCloudGenerator final : public rclcpp::Node
{
public:
  explicit TaskLidarCloudGenerator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("task_lidar_cloud_generator_node", options)
  {
    const auto directory = declare_parameter<std::string>(
      "task_directory",
      ament_index_cpp::get_package_share_directory("robosoft_core") + "/tasks");
    const auto file = declare_parameter<std::string>("task_file", "metsapelto.XML");
    const double latitude =
      declare_parameter<double>("reference_latitude", 62.73155205516667);
    const double longitude =
      declare_parameter<double>("reference_longitude", 27.05362871683333);
    if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
      std::abs(latitude) >= 90.0 || std::abs(longitude) > 180.0)
    {
      throw std::runtime_error("Invalid lidar map origin");
    }
    poles_ = lidar::loadPoles(
      (std::filesystem::path(directory) / file).string(), latitude, longitude);

    offset_x_ = declare_parameter<double>("lidar_x_m", 1.094);
    offset_y_ = declare_parameter<double>("lidar_y_m", 0.0);
    offset_yaw_ = declare_parameter<double>("lidar_yaw_rad", 0.0);
    radius_ = declare_parameter<double>("pole_diameter_m", 0.07) * 0.5;
    min_range_ = declare_parameter<double>("range_min_m", 0.05);
    max_range_ = declare_parameter<double>("range_max_m", 25.0);
    timeout_ = declare_parameter<double>("odometry_timeout_s", 1.0);
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 15.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "cloud");
    const auto odometry_topic =
      declare_parameter<std::string>("odometry_topic", "vehicle/odometry");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "cloud");
    if (!std::isfinite(radius_) || radius_ <= 0.0 ||
      !std::isfinite(min_range_) || !std::isfinite(max_range_) ||
      min_range_ < 0.0 || max_range_ <= min_range_ || max_range_ > 65.0 ||
      !std::isfinite(timeout_) || timeout_ <= 0.0 ||
      !std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(offset_x_) || !std::isfinite(offset_y_) ||
      !std::isfinite(offset_yaw_))
    {
      throw std::runtime_error("Invalid TASK lidar geometry, range or timing parameters");
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, 10,
      std::bind(
        &TaskLidarCloudGenerator::onOdometry, this, std::placeholders::_1));
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&TaskLidarCloudGenerator::publishCloud, this));
    RCLCPP_INFO(
      get_logger(), "TASK lidar cloud generator: %zu poles, output '%s'",
      poles_.size(), output_topic.c_str());
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double kStartAngleRad = -135.0 * M_PI / 180.0;
  static constexpr double kAngleStepRad = 0.3333 * M_PI / 180.0;

  void onOdometry(const nav_msgs::msg::Odometry & odometry)
  {
    const auto & q = odometry.pose.pose.orientation;
    x_ = odometry.pose.pose.position.x;
    y_ = odometry.pose.pose.position.y;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    pose_received_ = std::isfinite(x_) && std::isfinite(y_) && std::isfinite(yaw_);
    last_pose_ = Clock::now();
  }

  bool poseFresh() const
  {
    return pose_received_ &&
           std::chrono::duration<double>(Clock::now() - last_pose_).count() < timeout_;
  }

  void publishCloud()
  {
    if (!poseFresh()) {
      return;
    }
    const double sensor_x =
      x_ + std::cos(yaw_) * offset_x_ - std::sin(yaw_) * offset_y_;
    const double sensor_y =
      y_ + std::sin(yaw_) * offset_x_ + std::cos(yaw_) * offset_y_;
    const auto ranges = lidar::ranges(
      poles_, sensor_x, sensor_y, yaw_ + offset_yaw_, radius_, min_range_, max_range_);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(ranges.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    const float missing = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t index = 0; index < ranges.size();
      ++index, ++x, ++y, ++z, ++intensity)
    {
      if (ranges[index] == 0) {
        *x = missing;
        *y = missing;
        *z = missing;
        *intensity = 0.0F;
        continue;
      }
      const double angle =
        kStartAngleRad + static_cast<double>(index) * kAngleStepRad;
      const double range = static_cast<double>(ranges[index]) / 1000.0;
      *x = static_cast<float>(range * std::cos(angle));
      *y = static_cast<float>(range * std::sin(angle));
      *z = 0.0F;
      *intensity = 100.0F;
    }
    cloud_pub_->publish(cloud);
  }

  std::vector<lidar::Pole> poles_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double offset_x_{1.094};
  double offset_y_{0.0};
  double offset_yaw_{0.0};
  double radius_{0.035};
  double min_range_{0.05};
  double max_range_{25.0};
  double timeout_{1.0};
  bool pose_received_{false};
  std::string frame_id_;
  Clock::time_point last_pose_{Clock::now()};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace robosoft_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<robosoft_simulator::TaskLidarCloudGenerator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("task_lidar_cloud_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
