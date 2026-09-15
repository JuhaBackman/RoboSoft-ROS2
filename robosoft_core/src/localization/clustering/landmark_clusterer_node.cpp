// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <robosoft_interfaces/msg/associated_landmark_array.hpp>
#include <robosoft_interfaces/msg/landmark_detection_array.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/srv/update_lidar_clusters.hpp>
#include <robosoft_interfaces/topics.hpp>

#include "landmark_clusterer.hpp"

namespace robosoft_core
{

/// Associates generic vehicle-frame detections with persistent TASK landmarks.
class LandmarkClustererNode final : public rclcpp::Node
{
public:
  explicit LandmarkClustererNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("landmark_clusterer_node", options)
  {
    mapping_enabled_ = declare_parameter("lidar_mapping", false);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    const double association_distance =
      declare_parameter("landmark_association_distance", 0.5);
    const auto maximum = declare_parameter("max_active_landmarks", 6);
    if (maximum < 1 || maximum > 6) {
      throw std::invalid_argument(
        "max_active_landmarks must be between 1 and the generated EKF "
        "capacity 6");
    }
    max_active_landmarks_ = static_cast<std::size_t>(maximum);
    clusterer_.setMapping(mapping_enabled_);
    clusterer_.setAssociationDistance(association_distance);

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kOdometryTopic, 10,
      [this](const nav_msgs::msg::Odometry & message) {
        odometry_ = message;
        have_odometry_ = true;
      });
    detections_sub_ = create_subscription<
      robosoft_interfaces::msg::LandmarkDetectionArray>(
      robosoft_interfaces::kLandmarkDetectionsTopic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &LandmarkClustererNode::onDetections, this, std::placeholders::_1));
    estimates_sub_ = create_subscription<
      robosoft_interfaces::msg::AssociatedLandmarkArray>(
      robosoft_interfaces::kLandmarkEstimatesTopic, 10,
      std::bind(
        &LandmarkClustererNode::onEstimates, this, std::placeholders::_1));
    task_sub_ = create_subscription<robosoft_interfaces::msg::TaskData>(
      robosoft_interfaces::kTaskLoadedTopic,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LandmarkClustererNode::onTask, this, std::placeholders::_1));
    map_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kMapOriginTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LandmarkClustererNode::onMapOrigin, this, std::placeholders::_1));

    associated_pub_ = create_publisher<
      robosoft_interfaces::msg::AssociatedLandmarkArray>(
      robosoft_interfaces::kAssociatedLandmarksTopic,
      rclcpp::SensorDataQoS());
    landmarks_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      robosoft_interfaces::kLidarLandmarksTopic,
      rclcpp::QoS(1).reliable().transient_local());
    update_client_ =
      create_client<robosoft_interfaces::srv::UpdateLidarClusters>(
      robosoft_interfaces::kTaskUpdateLidarClustersService);
    map_sync_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&LandmarkClustererNode::synchronizeMap, this));
  }

private:
  static constexpr double earth_radius = 6371000.0;

  static double yawOf(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  Point2D toLocal(double latitude, double longitude) const
  {
    const double reference_latitude_rad = reference_latitude_ * M_PI / 180.0;
    return {
      earth_radius * std::cos(reference_latitude_rad) *
      (longitude - reference_longitude_) * M_PI / 180.0,
      earth_radius * (latitude - reference_latitude_) * M_PI / 180.0};
  }

  robosoft_interfaces::msg::LidarCluster toWgs84(
    const Landmark & landmark) const
  {
    robosoft_interfaces::msg::LidarCluster result;
    const double reference_latitude_rad = reference_latitude_ * M_PI / 180.0;
    result.latitude =
      reference_latitude_ + landmark.center.y / earth_radius * 180.0 / M_PI;
    result.longitude =
      reference_longitude_ + landmark.center.x /
      (earth_radius * std::cos(reference_latitude_rad)) * 180.0 / M_PI;
    result.covariance_xx = landmark.covariance[0];
    result.covariance_xy = landmark.covariance[1];
    result.covariance_yx = landmark.covariance[2];
    result.covariance_yy = landmark.covariance[3];
    return result;
  }

  void onMapOrigin(const sensor_msgs::msg::NavSatFix & origin)
  {
    if (!std::isfinite(origin.latitude) || !std::isfinite(origin.longitude)) {
      RCLCPP_ERROR(get_logger(), "Ignoring invalid map origin");
      return;
    }
    reference_latitude_ = origin.latitude;
    reference_longitude_ = origin.longitude;
    map_origin_received_ = true;
    if (!current_task_.task_name.empty()) {
      loadTaskLandmarks();
    }
  }

  void onTask(const robosoft_interfaces::msg::TaskData & task)
  {
    current_task_ = task;
    current_task_name_ = task.task_name;
    current_map_id_ = task.task_name + "|" + task.version + "|" +
      std::to_string(task.timestamp);
    if (map_origin_received_) {
      loadTaskLandmarks();
    } else {
      RCLCPP_INFO(
        get_logger(), "Waiting for map origin before loading task landmarks");
    }
  }

  void loadTaskLandmarks()
  {
    std::vector<Landmark> landmarks;
    landmarks.reserve(current_task_.lidar_clusters.size());
    for (const auto & stored : current_task_.lidar_clusters) {
      Landmark landmark;
      landmark.center = toLocal(stored.latitude, stored.longitude);
      landmark.covariance = {
        stored.covariance_xx, stored.covariance_xy,
        stored.covariance_yx, stored.covariance_yy};
      landmarks.push_back(std::move(landmark));
    }
    clusterer_.setLandmarks(std::move(landmarks));
    publishLandmarkMap();
    RCLCPP_INFO(
      get_logger(), "Loaded %zu landmarks from task '%s'",
      clusterer_.landmarks().size(), current_task_name_.c_str());
  }

  void publishLandmarkMap()
  {
    geometry_msgs::msg::PoseArray map;
    map.header.stamp = now();
    map.header.frame_id = odom_frame_;
    for (const auto & landmark : clusterer_.landmarks()) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = landmark.center.x;
      pose.position.y = landmark.center.y;
      pose.orientation.w = 1.0;
      map.poses.push_back(pose);
    }
    landmarks_pub_->publish(map);
  }

  void onDetections(
    const robosoft_interfaces::msg::LandmarkDetectionArray & message)
  {
    robosoft_interfaces::msg::AssociatedLandmarkArray output;
    output.header = message.header;
    output.measurement_frame_id = message.header.frame_id;
    output.map_frame_id = odom_frame_;
    output.map_id = current_map_id_;
    if (!have_odometry_) {
      associated_pub_->publish(output);
      return;
    }

    clusterer_.setPose(
      odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
      yawOf(odometry_.pose.pose.orientation));
    std::vector<LandmarkDetection> detections;
    detections.reserve(message.detections.size());
    for (const auto & item : message.detections) {
      LandmarkDetection detection;
      detection.measurement = {item.measurement.x, item.measurement.y};
      detection.covariance = {
        item.covariance_xx, item.covariance_xy,
        item.covariance_yx, item.covariance_yy};
      detection.object_type = item.object_type;
      detection.confidence = item.confidence;
      detections.push_back(std::move(detection));
    }
    clusterer_.process(detections);

    for (const auto * landmark : clusterer_.active(max_active_landmarks_)) {
      robosoft_interfaces::msg::AssociatedLandmark item;
      item.landmark_id = landmark->id;
      item.measurement.x = landmark->measurement.x;
      item.measurement.y = landmark->measurement.y;
      item.map_position.x = landmark->center.x;
      item.map_position.y = landmark->center.y;
      item.measurement_covariance_xx = landmark->measurement_covariance[0];
      item.measurement_covariance_xy = landmark->measurement_covariance[1];
      item.measurement_covariance_yx = landmark->measurement_covariance[2];
      item.measurement_covariance_yy = landmark->measurement_covariance[3];
      item.map_covariance_xx = landmark->covariance[0];
      item.map_covariance_xy = landmark->covariance[1];
      item.map_covariance_yx = landmark->covariance[2];
      item.map_covariance_yy = landmark->covariance[3];
      item.object_type = landmark->object_type;
      item.confidence = landmark->confidence;
      output.landmarks.push_back(std::move(item));
    }
    associated_pub_->publish(output);
  }

  void onEstimates(
    const robosoft_interfaces::msg::AssociatedLandmarkArray & message)
  {
    if (message.map_id != current_map_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring landmark estimates for stale map '%s'",
        message.map_id.c_str());
      return;
    }
    for (const auto & item : message.landmarks) {
      clusterer_.updateLandmark(
        item.landmark_id, {item.map_position.x, item.map_position.y},
        {item.map_covariance_xx, item.map_covariance_xy,
          item.map_covariance_yx, item.map_covariance_yy});
    }
  }

  void synchronizeMap()
  {
    publishLandmarkMap();
    if (!map_origin_received_ || !mapping_enabled_ ||
      current_task_name_.empty() || !update_client_->service_is_ready())
    {
      return;
    }
    auto request = std::make_shared<
      robosoft_interfaces::srv::UpdateLidarClusters::Request>();
    for (const auto & landmark : clusterer_.landmarks()) {
      if (landmark.covariance[0] < 0.1 && landmark.covariance[3] < 0.1) {
        request->lidar_clusters.push_back(toWgs84(landmark));
      }
    }
    update_client_->async_send_request(request);
  }

  double reference_latitude_{0.0};
  double reference_longitude_{0.0};
  bool mapping_enabled_{false};
  bool have_odometry_{false};
  bool map_origin_received_{false};
  std::size_t max_active_landmarks_{6U};
  std::string current_map_id_;
  std::string current_task_name_;
  std::string odom_frame_;
  nav_msgs::msg::Odometry odometry_;
  robosoft_interfaces::msg::TaskData current_task_;
  LandmarkClusterer clusterer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::LandmarkDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::AssociatedLandmarkArray>::SharedPtr estimates_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::TaskData>::SharedPtr task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::AssociatedLandmarkArray>::SharedPtr associated_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_pub_;
  rclcpp::Client<robosoft_interfaces::srv::UpdateLidarClusters>::SharedPtr
    update_client_;
  rclcpp::TimerBase::SharedPtr map_sync_timer_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::LandmarkClustererNode>());
  rclcpp::shutdown();
  return 0;
}
