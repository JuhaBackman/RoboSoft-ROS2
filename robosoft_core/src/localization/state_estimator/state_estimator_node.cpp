// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <robosoft_interfaces/msg/lidar_cluster_observation_array.hpp>
#include <robosoft_interfaces/msg/localization_mode.hpp>
#include <robosoft_interfaces/msg/state_estimator_status.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/srv/update_lidar_clusters.hpp>
#include <robosoft_interfaces/topics.hpp>

#include "lidar_cluster.hpp"
#include "state_estimator.hpp"

namespace robosoft_core
{

class StateEstimatorNode final : public rclcpp::Node
{
public:
  explicit StateEstimatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("state_estimator_node", options)
  {
    mapping_enabled_ = declare_parameter("lidar_mapping", false);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    const double lidar_x_offset =
      declare_parameter("lidar_x_offset", 1.094);
    const double lidar_angle_error =
      declare_parameter("lidar_angle_error", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cluster_.setMapping(mapping_enabled_);
    cluster_.setSensorPose(lidar_x_offset, lidar_angle_error);

    raw_odometry_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kRawOdometryTopic, 10,
      [this](const nav_msgs::msg::Odometry & message) {
        raw_odometry_ = message;
        have_odometry_ = true;
      });
    mode_sub_ =
      create_subscription<robosoft_interfaces::msg::LocalizationMode>(
      robosoft_interfaces::kLocalizationModeTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const robosoft_interfaces::msg::LocalizationMode & message) {
        mode_ = message.mode;
      });
    measured_twist_sub_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
      robosoft_interfaces::kMeasuredTwistTopic, 10,
      [this](const geometry_msgs::msg::TwistStamped & message) {
        const double speed = message.twist.linear.x;
        if (std::abs(speed) > 1e-6) {
          measured_curvature_ = message.twist.angular.z / speed;
        }
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&StateEstimatorNode::onCloud, this, std::placeholders::_1));
    task_sub_ = create_subscription<robosoft_interfaces::msg::TaskData>(
      robosoft_interfaces::kTaskLoadedTopic,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StateEstimatorNode::onTask, this, std::placeholders::_1));
    map_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kMapOriginTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const sensor_msgs::msg::NavSatFix & origin) {
        if (!std::isfinite(origin.latitude) ||
          !std::isfinite(origin.longitude))
        {
          RCLCPP_ERROR(get_logger(), "Ignoring invalid map origin");
          return;
        }
        reference_latitude_ = origin.latitude;
        reference_longitude_ = origin.longitude;
        map_origin_received_ = true;
        RCLCPP_INFO(
          get_logger(), "Map origin received: %.9f, %.9f",
          reference_latitude_, reference_longitude_);
        if (!current_task_.task_name.empty()) {
          loadTaskLandmarks();
        }
      });

    odometry_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kOdometryTopic, 10);
    observations_pub_ =
      create_publisher<
      robosoft_interfaces::msg::LidarClusterObservationArray>(
      robosoft_interfaces::kLidarClusterObservationsTopic, 10);
    landmarks_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      robosoft_interfaces::kLidarLandmarksTopic,
      rclcpp::QoS(1).reliable().transient_local());
    status_pub_ =
      create_publisher<robosoft_interfaces::msg::StateEstimatorStatus>(
      robosoft_interfaces::kStateEstimatorStatusTopic,
      rclcpp::QoS(1).reliable().transient_local());
    update_client_ =
      create_client<robosoft_interfaces::srv::UpdateLidarClusters>(
      robosoft_interfaces::kTaskUpdateLidarClustersService);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&StateEstimatorNode::tick, this));
  }

private:
  static constexpr double earth_radius = 6371000.0;

  static double yawOf(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quaternionOf(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  ClusterPoint toLocal(double latitude, double longitude) const
  {
    const double reference_latitude_rad = reference_latitude_ * M_PI / 180.0;
    return {
      earth_radius * std::cos(reference_latitude_rad) *
      (longitude - reference_longitude_) * M_PI / 180.0,
      earth_radius * (latitude - reference_latitude_) * M_PI / 180.0};
  }

  robosoft_interfaces::msg::LidarCluster toWgs84(
    const ClusterLandmark & landmark) const
  {
    robosoft_interfaces::msg::LidarCluster result;
    const double reference_latitude_rad = reference_latitude_ * M_PI / 180.0;
    result.latitude =
      reference_latitude_ + landmark.center.y / earth_radius * 180.0 / M_PI;
    result.longitude =
      reference_longitude_ +
      landmark.center.x /
      (earth_radius * std::cos(reference_latitude_rad)) * 180.0 / M_PI;
    result.covariance_xx = landmark.covariance[0];
    result.covariance_xy = landmark.covariance[1];
    result.covariance_yx = landmark.covariance[2];
    result.covariance_yy = landmark.covariance[3];
    return result;
  }

  void onTask(const robosoft_interfaces::msg::TaskData & task)
  {
    current_task_ = task;
    current_task_name_ = task.task_name;
    if (!map_origin_received_) {
      RCLCPP_INFO(
        get_logger(), "Waiting for map origin before loading task landmarks");
      return;
    }
    loadTaskLandmarks();
  }

  void loadTaskLandmarks()
  {
    std::vector<ClusterLandmark> landmarks;
    landmarks.reserve(current_task_.lidar_clusters.size());
    for (const auto & stored : current_task_.lidar_clusters) {
      ClusterLandmark landmark;
      landmark.center = toLocal(stored.latitude, stored.longitude);
      landmark.covariance = {
        stored.covariance_xx, stored.covariance_xy,
        stored.covariance_yx, stored.covariance_yy};
      landmarks.push_back(landmark);
    }
    cluster_.setLandmarks(std::move(landmarks));
    // Publish the entire TASK obstacle map in the same local frame as odometry.
    // GUI consumers need neither geographic projection nor origin parameters.
    geometry_msgs::msg::PoseArray map;
    map.header.stamp = now();
    map.header.frame_id = odom_frame_;
    for (const auto & landmark : cluster_.landmarks()) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = landmark.center.x;
      pose.position.y = landmark.center.y;
      pose.orientation.w = 1.0;
      map.poses.push_back(pose);
    }
    landmarks_pub_->publish(map);
    RCLCPP_INFO(
      get_logger(), "Loaded %zu lidar clusters from task '%s'",
      cluster_.landmarks().size(), current_task_name_.c_str());
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<ClusterPoint> scan;
    scan.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      for (; x != x.end(); ++x, ++y) {
        if (std::isfinite(*x) && std::isfinite(*y)) {
          scan.push_back({*x, *y});
        } else {
          // Preserve missing beams as surface boundaries for clustering.
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
    cluster_.process(scan);
  }

  void publishObservations(const std::vector<ClusterLandmark *> & active)
  {
    robosoft_interfaces::msg::LidarClusterObservationArray output;
    output.timestamp = now().nanoseconds();
    for (const auto * cluster : active) {
      robosoft_interfaces::msg::LidarClusterObservation item;
      item.map_x = cluster->center.x;
      item.map_y = cluster->center.y;
      item.measurement_x = cluster->measurement.x;
      item.measurement_y = cluster->measurement.y;
      item.covariance_xx = cluster->covariance[0];
      item.covariance_xy = cluster->covariance[1];
      item.covariance_yx = cluster->covariance[2];
      item.covariance_yy = cluster->covariance[3];
      output.observations.push_back(item);
    }
    observations_pub_->publish(output);
  }

  void synchronizeMap()
  {
    if (!map_origin_received_ || !mapping_enabled_ || current_task_name_.empty() ||
      !update_client_->service_is_ready())
    {
      return;
    }
    auto request =
      std::make_shared<
      robosoft_interfaces::srv::UpdateLidarClusters::Request>();
    for (const auto & landmark : cluster_.landmarks()) {
      if (landmark.covariance[0] < 0.1 && landmark.covariance[3] < 0.1) {
        request->lidar_clusters.push_back(toWgs84(landmark));
      }
    }
    update_client_->async_send_request(request);
  }

  void tick()
  {
    if (!have_odometry_) {
      publishStatus(0);
      return;
    }
    // Generic modes: 0 uses raw GNSS, 1 uses GNSS+lidar EKF and 2 omits
    // GNSS measurements while continuing from controls and landmarks.
    const bool use_estimator =
      mode_ == robosoft_interfaces::msg::LocalizationMode::GNSS_LIDAR_EKF ||
      mode_ ==
      robosoft_interfaces::msg::LocalizationMode::LANDMARK_DEAD_RECKONING;
    estimator_.setPositionMeasurement(
      raw_odometry_.pose.pose.position.x, raw_odometry_.pose.pose.position.y,
      yawOf(raw_odometry_.pose.pose.orientation),
      mode_ !=
      robosoft_interfaces::msg::LocalizationMode::LANDMARK_DEAD_RECKONING);
    estimator_.setControls(raw_odometry_.twist.twist.linear.x, measured_curvature_);

    auto active = cluster_.active(6);
    publishObservations(active);
    estimator_.update(active);
    if (!reliability_reported_ ||
      estimator_.reliable() != last_reported_reliability_)
    {
      if (estimator_.reliable()) {
        RCLCPP_INFO(get_logger(), "State estimate is reliable");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "State estimate is unreliable (P: x=%g, y=%g, yaw=%g)",
          estimator_.covarianceX(), estimator_.covarianceY(),
          estimator_.covarianceYaw());
      }
      last_reported_reliability_ = estimator_.reliable();
      reliability_reported_ = true;
    }
    publishStatus(active.size());

    auto output = raw_odometry_;
    if (use_estimator && estimator_.initialized()) {
      output.pose.pose.position.x = estimator_.x();
      output.pose.pose.position.y = estimator_.y();
      output.pose.pose.orientation = quaternionOf(estimator_.yaw());
      output.pose.covariance[0] = estimator_.covarianceX();
      output.pose.covariance[7] = estimator_.covarianceY();
      output.pose.covariance[35] = estimator_.covarianceYaw();
    }
    output.header.stamp = now();
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    odometry_pub_->publish(output);
    const double output_yaw = yawOf(output.pose.pose.orientation);
    cluster_.setPose(
      output.pose.pose.position.x, output.pose.pose.position.y, output_yaw);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    if (++map_sync_counter_ >= 20) {
      map_sync_counter_ = 0;
      synchronizeMap();
    }
  }

  void publishStatus(std::size_t active_clusters)
  {
    robosoft_interfaces::msg::StateEstimatorStatus status;
    status.timestamp = now().nanoseconds();
    status.initialized = estimator_.initialized();
    status.reliable = estimator_.reliable();
    status.mode = mode_;
    status.active_lidar_clusters =
      static_cast<uint32_t>(active_clusters);
    status.covariance_x = estimator_.covarianceX();
    status.covariance_y = estimator_.covarianceY();
    status.covariance_yaw = estimator_.covarianceYaw();
    status_pub_->publish(status);
  }

  double reference_latitude_{0.0};
  double reference_longitude_{0.0};
  double measured_curvature_{0.0};
  bool mapping_enabled_{false};
  bool have_odometry_{false};
  bool map_origin_received_{false};
  bool reliability_reported_{false};
  bool last_reported_reliability_{false};
  uint8_t mode_{0};
  unsigned map_sync_counter_{0};
  std::string current_task_name_;
  std::string odom_frame_;
  std::string base_frame_;
  nav_msgs::msg::Odometry raw_odometry_;
  robosoft_interfaces::msg::TaskData current_task_;
  LidarCluster cluster_;
  StateEstimator estimator_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    raw_odometry_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::LocalizationMode>::SharedPtr
    mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    measured_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::TaskData>::SharedPtr task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_pub_;
  bool publish_tf_{true};
  rclcpp::Publisher<
    robosoft_interfaces::msg::LidarClusterObservationArray>::SharedPtr
    observations_pub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::StateEstimatorStatus>::SharedPtr status_pub_;
  rclcpp::Client<robosoft_interfaces::srv::UpdateLidarClusters>::SharedPtr
    update_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
