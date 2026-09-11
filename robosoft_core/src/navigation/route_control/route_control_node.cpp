// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "route_control_node.hpp"
#include "robosoft_interfaces/topics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace robosoft_core
{

namespace
{
constexpr double kEarthRadius = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

// Recorded segment names contain a suffix, while loaded TaskData normally has
// an explicit guidance_group_id. Normalize both representations for grouping.
std::string guidanceGroupKey(
  const robosoft_interfaces::msg::GuidanceLine & line)
{
  if (!line.guidance_group_id.empty()) {
    return line.guidance_group_id;
  }
  const auto marker = line.name.rfind("_recorded_");
  return marker == std::string::npos ? line.name :
         line.name.substr(0, marker);
}
}

RouteControlNode::RouteControlNode(const rclcpp::NodeOptions & options)
: Node("route_control_node", options)
{
  record_distance_ = declare_parameter<double>("record_distance", 1.0);
  record_yaw_difference_ =
    declare_parameter<double>("record_yaw_difference", 0.35);
  lookahead_minimum_m_ =
    declare_parameter<double>("lookahead_minimum_m", 0.5);
  lookahead_time_s_ = declare_parameter<double>("lookahead_time_s", 1.4);
  lookahead_maximum_m_ =
    declare_parameter<double>("lookahead_maximum_m", 3.0);
  if (lookahead_minimum_m_ < 0.0 ||
    lookahead_maximum_m_ < lookahead_minimum_m_ || lookahead_time_s_ < 0.0)
  {
    throw std::invalid_argument("Invalid route lookahead parameters");
  }
  maximum_start_distance_ =
    declare_parameter<double>("maximum_start_distance", 10.0);
  gps_timeout_ms_ = declare_parameter<int>("gps_timeout_ms", 1000);
  odometry_timeout_ms_ =
    declare_parameter<int>("odometry_timeout_ms", 500);
  measured_twist_timeout_ms_ =
    declare_parameter<int>("measured_twist_timeout_ms", 500);

  task_sub_ = create_subscription<robosoft_interfaces::msg::TaskData>(
      robosoft_interfaces::kTaskLoadedTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RouteControlNode::onTask, this, std::placeholders::_1));
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kGnssFixTopic, rclcpp::SensorDataQoS(),
      std::bind(&RouteControlNode::onGps, this, std::placeholders::_1));
  map_origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      robosoft_interfaces::kMapOriginTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RouteControlNode::onMapOrigin, this, std::placeholders::_1));
  odometry_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
          robosoft_interfaces::kOdometryTopic, 10,
          std::bind(&RouteControlNode::onOdometry, this,
                    std::placeholders::_1));
  measured_twist_sub_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
          robosoft_interfaces::kMeasuredTwistTopic, 10,
          std::bind(&RouteControlNode::onMeasuredTwist, this,
                    std::placeholders::_1));
  robot_command_sub_ =
    create_subscription<robosoft_interfaces::msg::RobotCommand>(
          robosoft_interfaces::kRobotCommandTopic, 10,
          std::bind(&RouteControlNode::onRobotCommand, this,
                    std::placeholders::_1));
  robot_state_sub_ =
    create_subscription<robosoft_interfaces::msg::RobotState>(
          robosoft_interfaces::kRobotStateTopic, 10,
          std::bind(&RouteControlNode::onRobotState, this,
                    std::placeholders::_1));

  status_pub_ = create_publisher<robosoft_interfaces::msg::RouteStatus>(
      robosoft_interfaces::kRouteStatusTopic, 10);
  recorded_route_pub_ =
    create_publisher<robosoft_interfaces::msg::GuidanceLine>(
          robosoft_interfaces::kRecordedRouteTopic, 10);
  active_guidance_pub_ =
    create_publisher<robosoft_interfaces::msg::GuidanceLine>(
          robosoft_interfaces::kCurrentRouteTopic,
          rclcpp::QoS(1).reliable().transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    robosoft_interfaces::kRoutePathTopic,
    rclcpp::QoS(1).reliable().transient_local());
  active_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    robosoft_interfaces::kActiveRoutePathTopic,
    rclcpp::QoS(1).reliable().transient_local());
  task_status_client_ =
    create_client<robosoft_interfaces::srv::ModifyTaskStatus>(
          robosoft_interfaces::kTaskModifyStatusService);
  last_gps_time_ = now();
  last_odometry_time_ = now();
  last_measured_twist_time_ = now();
  watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RouteControlNode::checkWatchdogs, this));

  status_.guidance_index = -1;
  status_.segment_index = -1;
  publishStatus("Idle");
}

// ---------------------------------------------------------------------------
// Input callbacks and mode selection
// ---------------------------------------------------------------------------

void RouteControlNode::onTask(
  const robosoft_interfaces::msg::TaskData & msg)
{
  const auto previous_status = task_.status;
  task_ = msg;

  if (map_origin_received_ && msg.active_guidance_index >= 0 &&
    msg.active_guidance_index < static_cast<int32_t>(msg.guidance_lines.size()))
  {
    guidance_index_ = msg.active_guidance_index;
    publishActiveGuidance();
  } else if (map_origin_received_) {
    guidance_index_ = -1;
    publishEmptyPath();
  }

  applyTaskOperation();

  if (previous_status != msg.status) {
    RCLCPP_INFO(get_logger(), "Task status %u -> %u", previous_status,
                msg.status);
  }
}

void RouteControlNode::onRobotState(
  const robosoft_interfaces::msg::RobotState & msg)
{
  robot_state_received_ = true;
  robot_substate_ = msg.substate;
  applyTaskOperation();
}

void RouteControlNode::applyTaskOperation()
{
  using TaskData = robosoft_interfaces::msg::TaskData;
  using RobotState = robosoft_interfaces::msg::RobotState;

  const bool running = task_.status == TaskData::STATUS_RUNNING;
  const bool record = running && map_origin_received_ && robot_state_received_ &&
    robot_substate_ == RobotState::SUBSTATE_MANUAL_RECORD;
  const bool follow = running && map_origin_received_ && robot_state_received_ &&
    robot_substate_ == RobotState::SUBSTATE_AUTO;

  if (record && state_ != RECORDING) {
    if (state_ == FOLLOWING) {stopFollowing();}
    startRecording();
  } else if (follow && state_ != FOLLOWING) {
    if (state_ == RECORDING) {stopRecording();}
    if (!startFollowing()) {completeTask();}
  } else if (!record && state_ == RECORDING) {
    stopRecording();
  } else if (!follow && state_ == FOLLOWING) {
    stopFollowing();
  }
}

void RouteControlNode::onGps(const sensor_msgs::msg::NavSatFix & msg)
{
  gps_ = msg;
  last_gps_time_ = now();
  gps_valid_ =
    msg.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(msg.latitude) && std::isfinite(msg.longitude);
}

void RouteControlNode::onMapOrigin(const sensor_msgs::msg::NavSatFix & msg)
{
  if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude)) {
    RCLCPP_ERROR(get_logger(), "Ignoring invalid map origin");
    return;
  }
  map_origin_latitude_ = msg.latitude;
  map_origin_longitude_ = msg.longitude;
  map_origin_received_ = true;
  RCLCPP_INFO(
    get_logger(), "Map origin received: %.9f, %.9f",
    map_origin_latitude_, map_origin_longitude_);
  if (!task_.guidance_lines.empty() && task_.active_guidance_index >= 0) {
    guidance_index_ = task_.active_guidance_index;
    publishActiveGuidance();
  } else {
    guidance_index_ = -1;
    publishEmptyPath();
  }
  applyTaskOperation();
}

void RouteControlNode::onOdometry(
  const nav_msgs::msg::Odometry & msg)
{
  last_odometry_time_ = now();
  last_odometry_ = msg;
  odometry_valid_ = true;
  if (state_ == RECORDING) {
    updateRecording(msg);
  } else if (state_ == FOLLOWING) {
    updateFollowing(msg);
  }
}

void RouteControlNode::onMeasuredTwist(
  const geometry_msgs::msg::TwistStamped & msg)
{
  measured_twist_ = msg;
  measured_twist_received_ = true;
  last_measured_twist_time_ = now();
}

void RouteControlNode::onRobotCommand(
  const robosoft_interfaces::msg::RobotCommand & msg)
{
  if (msg.implement_mode > 2 || msg.implement_mode == implement_mode_) {
    return;
  }
  if (state_ == RECORDING && !recorded_route_.points.empty()) {
    finishRecordedSegment();
  }
  implement_mode_ = msg.implement_mode;
}

// ---------------------------------------------------------------------------
// Route recording
// ---------------------------------------------------------------------------

void RouteControlNode::startRecording()
{
  recorded_route_ = robosoft_interfaces::msg::GuidanceLine();
  recorded_route_.name = task_.task_name + "_recorded";
  recorded_route_.is_active = false;
  recorded_route_.target_speed = 1.0F;
  state_ = RECORDING;
  segment_index_ = -1;
  driving_direction_ = 0;
  recorded_segment_count_ = 0;
  completion_requested_ = false;
  pending_task_status_ = -1;
  last_odometry_time_ = now();
  publishStatus("Recording started");
}

void RouteControlNode::stopRecording()
{
  if (!recorded_route_.points.empty()) {
    finishRecordedSegment();
  } else if (recorded_segment_count_ == 0) {
    RCLCPP_WARN(get_logger(), "Recorded route discarded: less than two points");
  }
  state_ = IDLE;
  publishStatus("Recording stopped");
}

void RouteControlNode::finishRecordedSegment()
{
  if (recorded_route_.points.empty()) {
    return;
  }

  // End every segment with a distinct 0 m/s target. Preserve the last moving
  // point when the current GNSS fix is at least 0.1 m farther along the path.
  auto & last_point = recorded_route_.points.back();
  if (gps_valid_ && odometry_valid_) {
    robosoft_interfaces::msg::RoutePoint stop_point;
    stop_point.latitude = gps_.latitude;
    stop_point.longitude = gps_.longitude;
    stop_point.altitude = static_cast<float>(gps_.altitude);
    stop_point.yaw = yawOf(last_odometry_.pose.pose.orientation);
    stop_point.speed = 0.0F;
    stop_point.implement_state = implement_mode_;
    const double endpoint_distance =
      distanceMeters(last_point.latitude, last_point.longitude,
                       stop_point.latitude, stop_point.longitude);
    if (endpoint_distance >= 0.1) {
      recorded_route_.points.push_back(stop_point);
    } else {
      last_point.altitude = stop_point.altitude;
      last_point.yaw = stop_point.yaw;
      last_point.speed = 0.0F;
      last_point.implement_state = stop_point.implement_state;
    }
  } else {
    last_point.speed = 0.0F;
  }

  if (recorded_route_.points.size() < 2) {
    RCLCPP_WARN(get_logger(),
                "Recorded route discarded: less than two distinct points");
    recorded_route_.points.clear();
    return;
  }
  recorded_route_.points.front().speed =
    driving_direction_ < 0 ? -0.5F : 0.5F;
  recorded_route_.name = task_.task_name + "_recorded_" +
    std::to_string(recorded_segment_count_);
  recorded_route_.designator = recorded_route_.name;
  recorded_route_.guidance_group_id = "GGP1";
  recorded_route_.guidance_pattern_id =
    "GPN" + std::to_string(recorded_segment_count_ + 1);
  recorded_route_.movement_type =
    robosoft_interfaces::msg::GuidanceLine::MOVEMENT_DRIVE;
  recorded_route_.sequence =
    static_cast<uint32_t>(recorded_segment_count_);
  recorded_route_.driving_direction =
    static_cast<int8_t>(driving_direction_);
  recorded_route_pub_->publish(recorded_route_);
  RCLCPP_INFO(get_logger(), "Recorded segment %d published (%zu points)",
              recorded_segment_count_, recorded_route_.points.size());
  ++recorded_segment_count_;
  recorded_route_.points.clear();
}

// ---------------------------------------------------------------------------
// Route-following lifecycle
// ---------------------------------------------------------------------------

bool RouteControlNode::startFollowing()
{
  int index = task_.active_guidance_index;
  if (index < 0 || index >= static_cast<int>(task_.guidance_lines.size())) {
    state_ = ERROR;
    publishStatus("No valid active route");
    return false;
  }
  guidance_group_key_ = guidanceGroupKey(task_.guidance_lines[index]);
  // The active index selects a guidance group. The actual starting pattern is
  // chosen from that group after the first odometry sample is available.
  int first_valid_index = -1;
  for (size_t i = 0; i < task_.guidance_lines.size(); ++i) {
    const auto & candidate = task_.guidance_lines[i];
    if (guidanceGroupKey(candidate) == guidance_group_key_ &&
      candidate.points.size() >= 2)
    {
      first_valid_index = static_cast<int>(i);
      break;
    }
  }
  if (first_valid_index < 0) {
    state_ = ERROR;
    publishStatus("No valid guidance pattern in active group");
    return false;
  }
  state_ = FOLLOWING;
  segment_index_ = -1;
  guidance_index_ = first_valid_index;
  completion_requested_ = false;
  pending_task_status_ = -1;
  last_odometry_time_ = now();
  status_.guidance_index = first_valid_index;
  publishStatus("Route following started");
  return true;
}

void RouteControlNode::stopFollowing()
{
  state_ = IDLE;
  segment_index_ = -1;
  guidance_index_ = -1;
  publishStatus("Route following stopped");
}

bool RouteControlNode::advanceGuidance()
{
  const int next = guidance_index_ + 1;
  if (next < 0 || next >= static_cast<int>(task_.guidance_lines.size())) {
    return false;
  }
  const auto & line = task_.guidance_lines[next];
  if (line.points.size() < 2 ||
    guidanceGroupKey(line) != guidance_group_key_)
  {
    return false;
  }
  guidance_index_ = next;
  segment_index_ = 0;
  status_.guidance_index = next;
  publishActiveGuidance();
  publishStatus("Next guidance pattern");
  return true;
}

void RouteControlNode::publishActiveGuidance()
{
  if (guidance_index_ < 0 ||
    guidance_index_ >= static_cast<int>(task_.guidance_lines.size()))
  {
    return;
  }
  auto line = task_.guidance_lines[guidance_index_];
  line.is_active = true;
  active_guidance_pub_->publish(line);

  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = "odom";
  nav_msgs::msg::Path active_path;
  active_path.header = path.header;
  const auto group_key = guidanceGroupKey(line);
  for (const auto & group_line : task_.guidance_lines) {
    if (guidanceGroupKey(group_line) != group_key) {
      continue;
    }
    for (const auto & point : group_line.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      const auto local = toLocal(point.latitude, point.longitude);
      pose.pose.position.x = local.x;
      pose.pose.position.y = local.y;
      pose.pose.position.z = point.altitude;
      pose.pose.orientation.z = std::sin(point.yaw * 0.5);
      pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      path.poses.push_back(pose);
      if (&group_line == &task_.guidance_lines[guidance_index_]) {
        active_path.poses.push_back(pose);
      }
    }
  }
  path_pub_->publish(path);
  active_path_pub_->publish(active_path);
}

void RouteControlNode::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = "odom";
  path_pub_->publish(path);
  active_path_pub_->publish(path);
}

void RouteControlNode::updateRecording(
  const nav_msgs::msg::Odometry & odom)
{
  // NMEA 0183 VTG reports speed over ground without a forward/reverse sign,
  // so use signed measured vehicle speed for this decision.
  const bool measured_twist_fresh = measured_twist_received_ &&
    (now() - last_measured_twist_time_).nanoseconds() <=
    static_cast<int64_t>(measured_twist_timeout_ms_) * 1000000LL;
  const double velocity = measured_twist_fresh ?
    measured_twist_.twist.linear.x : odom.twist.twist.linear.x;
  const double heading = yawOf(odom.pose.pose.orientation);
  if (!gps_valid_ || std::abs(velocity) <= 0.1) {
    return;
  }

  robosoft_interfaces::msg::RoutePoint point;
  point.latitude = gps_.latitude;
  point.longitude = gps_.longitude;
  point.altitude = static_cast<float>(gps_.altitude);
  point.yaw = heading;
  point.speed = velocity;
  point.implement_state = implement_mode_;

  // A direction reversal closes the current pattern. Rotation segments are
  // not synthesized; applications can supply them as separate patterns.
  const int direction = velocity > 0.1 ? 1 : -1;
  if (driving_direction_ != 0 && direction != driving_direction_ &&
    !recorded_route_.points.empty())
  {
    finishRecordedSegment();
  }
  driving_direction_ = direction;

  if (!recorded_route_.points.empty()) {
    const auto & last = recorded_route_.points.back();
    const auto distance = distanceMeters(
        last.latitude, last.longitude, point.latitude, point.longitude);
    const auto yaw_difference =
      std::abs(normalizeAngle(point.yaw - last.yaw));
    if (distance <= record_distance_ &&
      yaw_difference <= record_yaw_difference_)
    {
      return;
    }

    const auto last_xy = toLocal(last.latitude, last.longitude);
    const auto point_xy = toLocal(point.latitude, point.longitude);
    double travel_heading =
      std::atan2(point_xy.y - last_xy.y, point_xy.x - last_xy.x);
    if (direction < 0) {
      travel_heading = normalizeAngle(travel_heading + kPi);
    }
    if (std::abs(normalizeAngle(travel_heading - point.yaw)) > kPi * 0.5) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Recorded point rejected: point order conflicts with heading");
      return;
    }
  }

  recorded_route_.points.push_back(point);
  status_.segment_index =
    static_cast<int32_t>(recorded_route_.points.size()) - 1;
  publishStatus("Recording");
}

void RouteControlNode::updateFollowing(
  const nav_msgs::msg::Odometry & odom)
{
  const double heading = yawOf(odom.pose.pose.orientation);
  const double velocity = odom.twist.twist.linear.x;
  const auto vehicle = odometryToLocal(odom.pose.pose.position);
  if (segment_index_ < 0) {
    // Select the nearest heading-compatible segment across the complete
    // guidance group. This avoids assuming that active_guidance_index is also
    // the geometrically nearest pattern.
    double nearest = std::numeric_limits<double>::max();
    int nearest_guidance = -1;
    int nearest_segment = -1;
    for (size_t line_index = 0; line_index < task_.guidance_lines.size();
      ++line_index)
    {
      const auto & candidate = task_.guidance_lines[line_index];
      if (guidanceGroupKey(candidate) != guidance_group_key_ ||
        candidate.points.size() < 2)
      {
        continue;
      }
      if (candidate.movement_type ==
        robosoft_interfaces::msg::GuidanceLine::MOVEMENT_ROTATE)
      {
        for (const auto & point : candidate.points) {
          const auto xy = toLocal(point.latitude, point.longitude);
          const double distance =
            std::hypot(vehicle.x - xy.x, vehicle.y - xy.y);
          const bool heading_matches =
            std::abs(normalizeAngle(heading - point.yaw)) < kPi * 0.5;
          if (distance < nearest && heading_matches) {
            nearest = distance;
            nearest_guidance = static_cast<int>(line_index);
            nearest_segment = 0;
          }
        }
        continue;
      }
      for (size_t i = 0; i + 1 < candidate.points.size(); ++i) {
        const auto a =
          toLocal(candidate.points[i].latitude, candidate.points[i].longitude);
        const auto b = toLocal(candidate.points[i + 1].latitude,
                               candidate.points[i + 1].longitude);
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double length2 = dx * dx + dy * dy;
        if (length2 <= 1e-6) {continue;}
        const double t = std::clamp(
          ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy) / length2,
            0.0, 1.0);
        const double distance =
          std::hypot(vehicle.x - (a.x + t * dx),
                       vehicle.y - (a.y + t * dy));
        double expected_heading = candidate.points[i].yaw;
        if (!std::isfinite(expected_heading) ||
          std::abs(expected_heading) < 1e-6)
        {
          expected_heading = std::atan2(dy, dx);
          if (candidate.driving_direction < 0 ||
            candidate.points[i + 1].speed < -0.1F)
          {
            expected_heading = normalizeAngle(expected_heading + kPi);
          }
        }
        const bool heading_matches =
          std::abs(normalizeAngle(heading - expected_heading)) <
          kPi * 0.5;
        if (distance < nearest && heading_matches) {
          nearest = distance;
          nearest_guidance = static_cast<int>(line_index);
          nearest_segment = static_cast<int>(i);
        }
      }
    }
    if (nearest_guidance < 0 || nearest > maximum_start_distance_) {
      state_ = ERROR;
      publishStatus("No route point within start limits");
      completeTask();
      return;
    }
    guidance_index_ = nearest_guidance;
    segment_index_ = nearest_segment;
    status_.guidance_index = nearest_guidance;
    publishActiveGuidance();
  }

  const auto index = guidance_index_;
  const auto & line = task_.guidance_lines.at(index);
  if (line.movement_type ==
    robosoft_interfaces::msg::GuidanceLine::MOVEMENT_ROTATE)
  {
    updateRotation(odom);
    return;
  }

  const auto & a_point = line.points.at(segment_index_);
  const auto & b_point = line.points.at(segment_index_ + 1);
  const auto a = toLocal(a_point.latitude, a_point.longitude);
  const auto b = toLocal(b_point.latitude, b_point.longitude);
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1e-6) {
    state_ = ERROR;
    publishStatus("Zero-length route segment");
    return;
  }

  // raw_t describes progress on the infinite A->B line. The clamped value is
  // used for interpolation; raw_t > 1 advances to the next segment.
  const double raw_t =
    ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy) /
    length2;
  if (raw_t > 1.0 && segment_index_ + 2 < static_cast<int>(line.points.size())) {
    ++segment_index_;
    updateFollowing(odom);
    return;
  }

  const double t = std::clamp(raw_t, 0.0, 1.0);
  const double ex = vehicle.x - (a.x + t * dx);
  const double ey = vehicle.y - (a.y + t * dy);
  // Cross-product sign retains left/right information required by steering.
  const double signed_error = (dx * ey - dy * ex) / std::sqrt(length2);
  const double route_heading = std::atan2(dy, dx);
  double target_heading = route_heading;
  if (std::abs(a_point.yaw) > 1e-6 || std::abs(b_point.yaw) > 1e-6) {
    target_heading =
      normalizeAngle(a_point.yaw +
                       t * normalizeAngle(b_point.yaw - a_point.yaw));
  } else if (line.driving_direction < 0 || b_point.speed < -0.1F) {
    target_heading = normalizeAngle(route_heading + kPi);
  }

  // Continuous target values follow the vehicle projection along A->B, while
  // the implement state belongs to the segment destination.
  auto target_point = b_point;
  target_point.latitude =
    a_point.latitude + t * (b_point.latitude - a_point.latitude);
  target_point.longitude =
    a_point.longitude + t * (b_point.longitude - a_point.longitude);
  target_point.altitude = static_cast<float>(
    a_point.altitude + t * (b_point.altitude - a_point.altitude));
  target_point.yaw = static_cast<float>(target_heading);
  target_point.speed = static_cast<float>(
    a_point.speed + t * (b_point.speed - a_point.speed));
  target_point.implement_state = b_point.implement_state;
  const double path_curvature =
    (a_point.speed < 0.0F ? -1.0 : 1.0) * 1000.0 *
    std::tan(normalizeAngle(b_point.yaw - a_point.yaw)) /
    std::sqrt(length2);

  // Locate the lookahead reference by arc distance along the ordered TASK
  // points. Increasing point indices follow the recorded driving direction,
  // including routes whose signed speed is negative.
  const double requested_lookahead = std::clamp(
    lookahead_minimum_m_ + lookahead_time_s_ * std::abs(velocity),
    lookahead_minimum_m_, lookahead_maximum_m_);
  double remaining = requested_lookahead;
  double traversed = 0.0;
  size_t lookahead_segment = static_cast<size_t>(segment_index_);
  double lookahead_t = t;
  while (lookahead_segment + 1 < line.points.size()) {
    const auto segment_a = toLocal(
      line.points[lookahead_segment].latitude,
      line.points[lookahead_segment].longitude);
    const auto segment_b = toLocal(
      line.points[lookahead_segment + 1].latitude,
      line.points[lookahead_segment + 1].longitude);
    const double segment_length =
      std::hypot(segment_b.x - segment_a.x, segment_b.y - segment_a.y);
    const double available = segment_length * (1.0 - lookahead_t);
    if (segment_length > 1e-6 && remaining <= available) {
      lookahead_t += remaining / segment_length;
      traversed += remaining;
      remaining = 0.0;
      break;
    }
    traversed += available;
    remaining -= available;
    if (lookahead_segment + 2 >= line.points.size()) {
      lookahead_t = 1.0;
      break;
    }
    ++lookahead_segment;
    lookahead_t = 0.0;
  }

  const auto & lookahead_a = line.points[lookahead_segment];
  const auto & lookahead_b = line.points[lookahead_segment + 1];
  auto lookahead_point = lookahead_b;
  lookahead_point.latitude = lookahead_a.latitude + lookahead_t *
    (lookahead_b.latitude - lookahead_a.latitude);
  lookahead_point.longitude = lookahead_a.longitude + lookahead_t *
    (lookahead_b.longitude - lookahead_a.longitude);
  lookahead_point.altitude = static_cast<float>(
    lookahead_a.altitude + lookahead_t *
    (lookahead_b.altitude - lookahead_a.altitude));
  lookahead_point.yaw = static_cast<float>(normalizeAngle(
    lookahead_a.yaw + lookahead_t *
    normalizeAngle(lookahead_b.yaw - lookahead_a.yaw)));
  lookahead_point.speed = static_cast<float>(
    lookahead_a.speed + lookahead_t *
    (lookahead_b.speed - lookahead_a.speed));
  lookahead_point.implement_state = lookahead_b.implement_state;
  const auto lookahead_a_xy = toLocal(
    lookahead_a.latitude, lookahead_a.longitude);
  const auto lookahead_b_xy = toLocal(
    lookahead_b.latitude, lookahead_b.longitude);
  const double lookahead_segment_length = std::hypot(
    lookahead_b_xy.x - lookahead_a_xy.x,
    lookahead_b_xy.y - lookahead_a_xy.y);
  const double lookahead_curvature = lookahead_segment_length > 1e-6 ?
    (lookahead_a.speed < 0.0F ? -1.0 : 1.0) * 1000.0 *
    std::tan(normalizeAngle(lookahead_b.yaw - lookahead_a.yaw)) /
    lookahead_segment_length : 0.0;

  status_.state = FOLLOWING;
  status_.guidance_index = index;
  status_.segment_index = segment_index_;
  status_.cross_track_error = static_cast<float>(signed_error);
  status_.heading_error =
    static_cast<float>(normalizeAngle(target_heading - heading));
  status_.path_curvature = static_cast<float>(path_curvature);
  status_.target_point = target_point;
  status_.lookahead_point = lookahead_point;
  status_.lookahead_curvature = static_cast<float>(lookahead_curvature);
  status_.lookahead_distance = static_cast<float>(traversed);
  publishStatus();

  const bool stopped_at_target =
    std::abs(b_point.speed) < 0.1F && std::abs(velocity) < 0.1F;
  if (raw_t > 1.0 || stopped_at_target) {
    advanceSegmentOrComplete();
  }
}

void RouteControlNode::updateRotation(
  const nav_msgs::msg::Odometry & odom)
{
  const double heading = yawOf(odom.pose.pose.orientation);
  const double velocity = odom.twist.twist.linear.x;
  const auto & line = task_.guidance_lines.at(guidance_index_);
  const auto & a = line.points.at(segment_index_);
  const auto & b = line.points.at(segment_index_ + 1);

  // Rotation patterns are retained in the shared TASK format so applications
  // that support in-place rotation can execute them deterministically.
  const bool sign_ca = normalizeAngle(a.yaw - heading) > 0.0;
  const bool sign_cb = normalizeAngle(b.yaw - heading) > 0.0;
  const bool sign_ab = normalizeAngle(b.yaw - a.yaw) > 0.0;
  const bool stopped_at_target =
    std::abs(b.speed) < 0.1F && std::abs(velocity) < 0.1F;

  const robosoft_interfaces::msg::RoutePoint *target = &b;
  bool segment_complete = false;
  if (sign_ca == sign_cb && sign_cb == sign_ab) {
    target = &a;
  } else if (sign_ca == sign_cb || stopped_at_target) {
    segment_complete = true;
  }

  status_.state = FOLLOWING;
  status_.guidance_index = guidance_index_;
  status_.segment_index = segment_index_;
  status_.cross_track_error = 0.0F;
  status_.path_curvature = 0.0F;
  status_.heading_error =
    static_cast<float>(normalizeAngle(target->yaw - heading));
  status_.target_point = *target;
  status_.lookahead_point = *target;
  status_.lookahead_curvature = 0.0F;
  status_.lookahead_distance = 0.0F;
  publishStatus();

  if (segment_complete) {
    advanceSegmentOrComplete();
  }
}

void RouteControlNode::advanceSegmentOrComplete()
{
  const auto & line = task_.guidance_lines.at(guidance_index_);
  if (segment_index_ + 2 < static_cast<int>(line.points.size())) {
    ++segment_index_;
    return;
  }
  if (advanceGuidance()) {
    return;
  }
  state_ = COMPLETED;
  publishStatus("Route completed");
  completeTask();
}

void RouteControlNode::checkWatchdogs()
{
  // A service may be unavailable during startup. Pending terminal status is
  // retried from this timer instead of being lost.
  if (pending_task_status_ >= 0) {
    completeTask();
  }
  if (state_ != RECORDING && state_ != FOLLOWING) {
    return;
  }
  const auto current_time = now();
  const bool gps_failed =
    state_ == RECORDING &&
    (!gps_valid_ ||
    (current_time - last_gps_time_).nanoseconds() >
    static_cast<int64_t>(gps_timeout_ms_) * 1000000LL);
  const bool odometry_failed =
    (current_time - last_odometry_time_).nanoseconds() >
    static_cast<int64_t>(odometry_timeout_ms_) * 1000000LL;
  if (!gps_failed && !odometry_failed) {
    return;
  }

  if (state_ == RECORDING) {
    finishRecordedSegment();
  }
  state_ = ERROR;
  publishStatus(gps_failed ? "GPS data timeout" : "Odometry data timeout");
  completeTask();
}

void RouteControlNode::publishStatus(const std::string & message)
{
  status_.state = state_;
  status_.segment_index = segment_index_;
  if (state_ != FOLLOWING) {
    status_.cross_track_error = 0.0F;
    status_.heading_error = 0.0F;
    status_.path_curvature = 0.0F;
    status_.lookahead_curvature = 0.0F;
    status_.lookahead_distance = 0.0F;
  }
  if (!message.empty()) {status_.message = message;}
  status_.timestamp = now().nanoseconds();
  status_pub_->publish(status_);
}

void RouteControlNode::completeTask()
{
  if (pending_task_status_ < 0) {
    pending_task_status_ = state_ == COMPLETED ? 4 : 3;
  }
  if (completion_requested_ || task_status_request_in_flight_ ||
    !task_status_client_->service_is_ready())
  {
    return;
  }
  task_status_request_in_flight_ = true;
  auto request =
    std::make_shared<robosoft_interfaces::srv::ModifyTaskStatus::Request>();
  request->guidance_index = task_.active_guidance_index;
  request->new_status = static_cast<uint8_t>(pending_task_status_);
  // Keep at most one request in flight. Failed responses leave the status
  // pending so the watchdog timer retries it.
  task_status_client_->async_send_request(
      request,
    [this](
      rclcpp::Client<
        robosoft_interfaces::srv::ModifyTaskStatus>::SharedFuture future) {
      task_status_request_in_flight_ = false;
      const auto response = future.get();
      if (response->success) {
        completion_requested_ = true;
        pending_task_status_ = -1;
        return;
      }
      RCLCPP_WARN(get_logger(), "Task status update failed: %s",
                    response->error_message.c_str());
      });
}

RouteControlNode::XY RouteControlNode::toLocal(
  double latitude, double longitude) const
{
  const double lat = latitude * kPi / 180.0;
  const double lon = longitude * kPi / 180.0;
  const double ref_lat = map_origin_latitude_ * kPi / 180.0;
  const double ref_lon = map_origin_longitude_ * kPi / 180.0;
  return {kEarthRadius * std::cos(ref_lat) * (lon - ref_lon),
    kEarthRadius * (lat - ref_lat)};
}

RouteControlNode::XY RouteControlNode::odometryToLocal(
  const geometry_msgs::msg::Point & position) const
{
  return {position.x, position.y};
}

double RouteControlNode::normalizeAngle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

double RouteControlNode::yawOf(
  const geometry_msgs::msg::Quaternion & orientation)
{
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double RouteControlNode::distanceMeters(
  double lat1, double lon1, double lat2, double lon2)
{
  const double mean_lat = (lat1 + lat2) * 0.5 * kPi / 180.0;
  const double dy = (lat2 - lat1) * kPi / 180.0 * kEarthRadius;
  const double dx =
    (lon2 - lon1) * kPi / 180.0 * kEarthRadius * std::cos(mean_lat);
  return std::hypot(dx, dy);
}

}  // namespace robosoft_core

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::RouteControlNode>());
  rclcpp::shutdown();
  return 0;
}
