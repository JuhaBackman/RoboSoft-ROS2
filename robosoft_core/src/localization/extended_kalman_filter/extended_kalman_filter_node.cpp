// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <robosoft_interfaces/msg/associated_landmark_array.hpp>
#include <robosoft_interfaces/msg/lidar_cluster_observation_array.hpp>
#include <robosoft_interfaces/msg/localization_mode.hpp>
#include <robosoft_interfaces/msg/state_estimator_status.hpp>
#include <robosoft_interfaces/topics.hpp>

#include "extended_kalman_filter.hpp"

namespace robosoft_core
{

/// Fuses generic associated landmarks with GNSS and measured vehicle motion.
class ExtendedKalmanFilterNode final : public rclcpp::Node
{
public:
  explicit ExtendedKalmanFilterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("extended_kalman_filter_node", options)
  {
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    raw_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kRawOdometryTopic, 10,
      [this](const nav_msgs::msg::Odometry & message) {
        raw_odometry_ = message;
        have_odometry_ = true;
      });
    mode_sub_ = create_subscription<robosoft_interfaces::msg::LocalizationMode>(
      robosoft_interfaces::kLocalizationModeTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const robosoft_interfaces::msg::LocalizationMode & message) {
        mode_ = message.mode;
      });
    measured_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      robosoft_interfaces::kMeasuredTwistTopic, 10,
      [this](const geometry_msgs::msg::TwistStamped & message) {
        const double speed = message.twist.linear.x;
        if (std::abs(speed) > 1e-6) {
          measured_curvature_ = message.twist.angular.z / speed;
        }
      });
    associated_sub_ = create_subscription<
      robosoft_interfaces::msg::AssociatedLandmarkArray>(
      robosoft_interfaces::kAssociatedLandmarksTopic,
      rclcpp::SensorDataQoS(),
      [this](const robosoft_interfaces::msg::AssociatedLandmarkArray & message) {
        // Detections can arrive before a TASK has been loaded. They do not
        // belong to a persistent landmark map and must not be returned later
        // as estimator feedback for the newly loaded TASK.
        if (message.map_id.empty()) {
          return;
        }
        associated_ = message;
        associated_pending_ = true;
      });

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kOdometryTopic, 10);
    estimates_pub_ = create_publisher<
      robosoft_interfaces::msg::AssociatedLandmarkArray>(
      robosoft_interfaces::kLandmarkEstimatesTopic, 10);
    observations_pub_ = create_publisher<
      robosoft_interfaces::msg::LidarClusterObservationArray>(
      robosoft_interfaces::kLidarClusterObservationsTopic, 10);
    status_pub_ = create_publisher<
      robosoft_interfaces::msg::StateEstimatorStatus>(
      robosoft_interfaces::kStateEstimatorStatusTopic,
      rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ExtendedKalmanFilterNode::tick, this));
  }

private:
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

  std::vector<Landmark> decodeLandmarks() const
  {
    std::vector<Landmark> landmarks;
    landmarks.reserve(associated_.landmarks.size());
    for (const auto & item : associated_.landmarks) {
      Landmark landmark;
      landmark.id = item.landmark_id;
      landmark.measurement = {item.measurement.x, item.measurement.y};
      landmark.center = {item.map_position.x, item.map_position.y};
      landmark.measurement_covariance = {
        item.measurement_covariance_xx, item.measurement_covariance_xy,
        item.measurement_covariance_yx, item.measurement_covariance_yy};
      landmark.covariance = {
        item.map_covariance_xx, item.map_covariance_xy,
        item.map_covariance_yx, item.map_covariance_yy};
      landmark.object_type = item.object_type;
      landmark.confidence = item.confidence;
      landmark.measured = true;
      landmarks.push_back(std::move(landmark));
    }
    return landmarks;
  }

  void publishLandmarkResults(const std::vector<Landmark> & landmarks)
  {
    auto estimates = associated_;
    estimates.header.stamp = now();
    robosoft_interfaces::msg::LidarClusterObservationArray observations;
    observations.timestamp = now().nanoseconds();
    for (std::size_t index = 0; index < landmarks.size(); ++index) {
      const auto & landmark = landmarks[index];
      auto & estimate = estimates.landmarks[index];
      estimate.map_position.x = landmark.center.x;
      estimate.map_position.y = landmark.center.y;
      estimate.map_covariance_xx = landmark.covariance[0];
      estimate.map_covariance_xy = landmark.covariance[1];
      estimate.map_covariance_yx = landmark.covariance[2];
      estimate.map_covariance_yy = landmark.covariance[3];

      robosoft_interfaces::msg::LidarClusterObservation observation;
      observation.map_x = landmark.center.x;
      observation.map_y = landmark.center.y;
      observation.measurement_x = landmark.measurement.x;
      observation.measurement_y = landmark.measurement.y;
      observation.covariance_xx = landmark.covariance[0];
      observation.covariance_xy = landmark.covariance[1];
      observation.covariance_yx = landmark.covariance[2];
      observation.covariance_yy = landmark.covariance[3];
      observations.observations.push_back(std::move(observation));
    }
    estimates_pub_->publish(estimates);
    observations_pub_->publish(observations);
  }

  void tick()
  {
    if (!have_odometry_) {
      publishStatus(0U);
      return;
    }
    const bool use_estimator =
      mode_ == robosoft_interfaces::msg::LocalizationMode::GNSS_LIDAR_EKF ||
      mode_ ==
      robosoft_interfaces::msg::LocalizationMode::LANDMARK_DEAD_RECKONING;
    estimator_.setPositionMeasurement(
      raw_odometry_.pose.pose.position.x, raw_odometry_.pose.pose.position.y,
      yawOf(raw_odometry_.pose.pose.orientation),
      mode_ !=
      robosoft_interfaces::msg::LocalizationMode::LANDMARK_DEAD_RECKONING);
    estimator_.setControls(
      raw_odometry_.twist.twist.linear.x, measured_curvature_);

    auto landmarks = associated_pending_ ?
      decodeLandmarks() : std::vector<Landmark>{};
    std::vector<Landmark *> active;
    active.reserve(landmarks.size());
    for (auto & landmark : landmarks) {
      active.push_back(&landmark);
    }
    estimator_.update(active);
    if (associated_pending_) {
      publishLandmarkResults(landmarks);
      associated_.landmarks.clear();
      associated_pending_ = false;
    }

    if (!reliability_reported_ ||
      estimator_.reliable() != last_reported_reliability_)
    {
      if (estimator_.reliable()) {
        RCLCPP_INFO(get_logger(), "State estimate is reliable");
      } else {
        RCLCPP_WARN(
          get_logger(), "State estimate is unreliable (P: x=%g, y=%g, yaw=%g)",
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
  }

  void publishStatus(std::size_t active_landmarks)
  {
    robosoft_interfaces::msg::StateEstimatorStatus status;
    status.timestamp = now().nanoseconds();
    status.initialized = estimator_.initialized();
    status.reliable = estimator_.reliable();
    status.mode = mode_;
    status.active_lidar_clusters = static_cast<uint32_t>(active_landmarks);
    status.covariance_x = estimator_.covarianceX();
    status.covariance_y = estimator_.covarianceY();
    status.covariance_yaw = estimator_.covarianceYaw();
    status_pub_->publish(status);
  }

  double measured_curvature_{0.0};
  bool associated_pending_{false};
  bool have_odometry_{false};
  bool publish_tf_{true};
  bool reliability_reported_{false};
  bool last_reported_reliability_{false};
  uint8_t mode_{0};
  std::string odom_frame_;
  std::string base_frame_;
  nav_msgs::msg::Odometry raw_odometry_;
  robosoft_interfaces::msg::AssociatedLandmarkArray associated_;
  ExtendedKalmanFilter estimator_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odometry_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::LocalizationMode>::SharedPtr
    mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    measured_twist_sub_;
  rclcpp::Subscription<
    robosoft_interfaces::msg::AssociatedLandmarkArray>::SharedPtr associated_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::AssociatedLandmarkArray>::SharedPtr estimates_pub_;
  rclcpp::Publisher<
    robosoft_interfaces::msg::LidarClusterObservationArray>::SharedPtr
    observations_pub_;
  rclcpp::Publisher<robosoft_interfaces::msg::StateEstimatorStatus>::SharedPtr
    status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::ExtendedKalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
