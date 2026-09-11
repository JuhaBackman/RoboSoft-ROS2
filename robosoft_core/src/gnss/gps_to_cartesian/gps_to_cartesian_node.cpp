// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "gps_to_cartesian_node.hpp"
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "robosoft_interfaces/topics.hpp"

namespace robosoft_core
{

GpsToCartesianNode::GpsToCartesianNode(const rclcpp::NodeOptions & options)
: Node("gps_to_cartesian_node", options),
  reference_latitude_(0.0),
  reference_longitude_(0.0)
{
  this->declare_parameter<double>("reference_latitude", 60.0);
  this->declare_parameter<double>("reference_longitude", 25.0);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);

  reference_latitude_ = this->get_parameter("reference_latitude").as_double();
  reference_longitude_ = this->get_parameter("reference_longitude").as_double();

  map_origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    robosoft_interfaces::kMapOriginTopic,
    rclcpp::QoS(1).reliable().transient_local());
  sensor_msgs::msg::NavSatFix map_origin;
  map_origin.header.stamp = now();
  map_origin.header.frame_id = "map";
  map_origin.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  map_origin.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  map_origin.latitude = reference_latitude_;
  map_origin.longitude = reference_longitude_;
  map_origin.altitude = 0.0;
  map_origin_pub_->publish(map_origin);
  RCLCPP_INFO(
    get_logger(), "Map origin: %.9f, %.9f",
    reference_latitude_, reference_longitude_);

  // Generic GNSS position source selected by launch composition.
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kGnssFixTopic, rclcpp::SensorDataQoS(),
      std::bind(&GpsToCartesianNode::onGpsMessage, this,
                std::placeholders::_1));

  // Generic heading and ground-speed source selected by launch composition.
  heading_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      robosoft_interfaces::kGnssHeadingTopic, rclcpp::SensorDataQoS(),
      std::bind(&GpsToCartesianNode::onHeadingMessage, this,
                std::placeholders::_1));

  pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      robosoft_interfaces::kOdometryTopic, rclcpp::QoS(10));
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(this->get_logger(), "GpsToCartesianNode initialized");
}

GpsToCartesianNode::~GpsToCartesianNode() {}

void GpsToCartesianNode::onGpsMessage(const sensor_msgs::msg::NavSatFix & msg)
{
  // Convert WGS84 to local cartesian
  double lat_rad = msg.latitude * M_PI / 180.0;
  double lon_rad = msg.longitude * M_PI / 180.0;
  double ref_lat_rad = reference_latitude_ * M_PI / 180.0;
  double ref_lon_rad = reference_longitude_ * M_PI / 180.0;

  double earth_radius = 6371000.0;  // meters

  double y = earth_radius * (lat_rad - ref_lat_rad);
  double x = earth_radius * cos(ref_lat_rad) * (lon_rad - ref_lon_rad);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg.header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = msg.altitude;
  odom.pose.pose.orientation.z = std::sin(current_heading_ * 0.5);
  odom.pose.pose.orientation.w = std::cos(current_heading_ * 0.5);
  odom.pose.covariance[0] = msg.position_covariance[0];
  odom.pose.covariance[7] = msg.position_covariance[4];
  odom.pose.covariance[14] = msg.position_covariance[8];
  odom.twist.twist.linear.x = current_speed_;

  pub_->publish(odom);
  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = msg.altitude;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

void GpsToCartesianNode::onHeadingMessage(
  const geometry_msgs::msg::TwistStamped & msg)
{
  // Extract heading (yaw) from NMEA2000 COG/SOG
  current_heading_ = msg.twist.angular.z;  // yaw in radians
  current_speed_ = msg.twist.linear.x;     // forward speed m/s
}

}  // namespace robosoft_core

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robosoft_core::GpsToCartesianNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
