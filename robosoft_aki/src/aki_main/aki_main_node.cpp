// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "aki_main_node.hpp"

#include "robosoft_interfaces/topics.hpp"
#include "topics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace robosoft_aki
{

AkiMainNode::AkiMainNode(const rclcpp::NodeOptions & options)
: RobotMainNodeBase("aki_main_node", "aki", options),
  last_gps_fix_(0, 0, get_clock()->get_clock_type()),
  last_estimator_status_(0, 0, get_clock()->get_clock_type()),
  last_tecu_wheel_speed_(0, 0, get_clock()->get_clock_type()),
  last_tecu_guidance_status_(0, 0, get_clock()->get_clock_type())
{
  declare_parameter<std::vector<std::string>>(
    "required_nodes",
    {"/remote_controller_node", "/tecu_node", "/nmea2000_node",
      "/nmea2000_server", "/gps_to_cartesian_node",
      "/task_manager_node", "/route_control_node", "/path_tracking_node",
      "/aki_uvc_sequence_node",
      "/uvc_led_node", "/lidar_safety_node"});
  required_nodes_ = get_parameter("required_nodes").as_string_array();
  lidar_enabled_ = declare_parameter<bool>("enable_lidar", true);
  if (!lidar_enabled_) {
    required_nodes_.erase(
      std::remove(required_nodes_.begin(), required_nodes_.end(), "/lidar_safety_node"),
      required_nodes_.end());
  }
  tecu_timeout_s_ =
    declare_parameter<int>("tecu_timeout_ms", 1000) / 1000.0;
  estimator_timeout_s_ =
    declare_parameter<int>("state_estimator_timeout_ms", 1000) / 1000.0;

  remote_sub_ =
    create_subscription<robosoft_interfaces::msg::RemoteControlStatus>(
    robosoft_interfaces::kRemoteControlStatusTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&AkiMainNode::onRemote, this, std::placeholders::_1));
  command_sub_ = create_subscription<robosoft_interfaces::msg::RobotCommand>(
    robosoft_interfaces::kRobotCommandTopic, 10,
    std::bind(&AkiMainNode::onCommand, this, std::placeholders::_1));
  odometry_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
    robosoft_interfaces::kOdometryTopic, 10,
    std::bind(&AkiMainNode::onOdometry, this, std::placeholders::_1));
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    robosoft_interfaces::kGnssFixTopic, rclcpp::SensorDataQoS(),
    std::bind(&AkiMainNode::onGps, this, std::placeholders::_1));
  route_status_sub_ =
    create_subscription<robosoft_interfaces::msg::RouteStatus>(
    robosoft_interfaces::kRouteStatusTopic, 10,
    std::bind(&AkiMainNode::onRouteStatus, this, std::placeholders::_1));
  if (lidar_enabled_) {
    lidar_safety_sub_ =
      create_subscription<robosoft_interfaces::msg::LidarSafetyStatus>(
      robosoft_interfaces::kLidarSafetyStatusTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&AkiMainNode::onLidarSafety, this, std::placeholders::_1));
  }
  implement_status_sub_ =
    create_subscription<robosoft_interfaces::msg::ImplementStatus>(
    robosoft_interfaces::kImplementStatusTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&AkiMainNode::onImplementStatus, this, std::placeholders::_1));
  estimator_status_sub_ =
    create_subscription<robosoft_interfaces::msg::StateEstimatorStatus>(
    robosoft_interfaces::kStateEstimatorStatusTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &AkiMainNode::onStateEstimatorStatus, this, std::placeholders::_1));
  tecu_wheel_speed_sub_ =
    create_subscription<ros2_isobus::msg::TecuWheelSpeed>(
    ros2_isobus::kTECUWheelSpeedTopic, 10,
    std::bind(&AkiMainNode::onTecuWheelSpeed, this, std::placeholders::_1));
  tecu_guidance_status_sub_ =
    create_subscription<ros2_isobus::msg::TecuGuidanceStatus>(
    ros2_isobus::kTECUGuidanceStatusTopic, 10,
    std::bind(
      &AkiMainNode::onTecuGuidanceStatus, this, std::placeholders::_1));

  localization_mode_pub_ =
    create_publisher<robosoft_interfaces::msg::LocalizationMode>(
    robosoft_interfaces::kLocalizationModeTopic,
    rclcpp::QoS(1).reliable().transient_local());
  implement_pub_ =
    create_publisher<robosoft_interfaces::msg::ImplementCommand>(
    robosoft_interfaces::kImplementCommandTopic,
    rclcpp::QoS(1).reliable().transient_local());
  timer_performance_pub_ =
    create_publisher<robosoft_interfaces::msg::TimerPerformance>(
    robosoft_interfaces::kAkiMainTimerPerformanceTopic, 10);
  state_timer_ =
    create_wall_timer(100ms, std::bind(&AkiMainNode::runTimedStateMachine, this));
  onEnterState(substate_);
}

void AkiMainNode::runTimedStateMachine()
{
  constexpr double target_period_ms = 100.0;
  const auto start = std::chrono::steady_clock::now();
  const double actual_period_ms = timer_started_ ?
    std::chrono::duration<double, std::milli>(start - previous_timer_start_).count() :
    target_period_ms;
  previous_timer_start_ = start;
  timer_started_ = true;

  runStateMachine();

  const double duration_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  const double lateness_ms = std::max(
    {0.0, actual_period_ms - target_period_ms, duration_ms - target_period_ms});
  if (lateness_ms > 1.0) ++timer_deadline_misses_;
  robosoft_interfaces::msg::TimerPerformance performance;
  performance.node_name = get_name();
  performance.target_period_ms = target_period_ms;
  performance.actual_period_ms = actual_period_ms;
  performance.callback_duration_ms = duration_ms;
  performance.margin_ms = std::max(0.0, target_period_ms - duration_ms);
  performance.lateness_ms = lateness_ms;
  performance.deadline_misses = timer_deadline_misses_;
  timer_performance_pub_->publish(performance);
}

void AkiMainNode::onRemote(
  const robosoft_interfaces::msg::RemoteControlStatus & msg)
{
  if (msg.layout != robosoft_interfaces::msg::RemoteControlStatus::LAYOUT_AKI) {
    return;
  }
  remote_ = msg;
  remote_received_ = true;
  remote_emergency_stop_ = msg.connected && !msg.safety;
  updateEmergencyStopState();
  robosoft_interfaces::msg::LocalizationMode localization_mode;
  localization_mode.mode =
    robosoft_interfaces::msg::LocalizationMode::RAW_GNSS;
  if (msg.mode == REMOTE_AUTO_2) {
    localization_mode.mode =
      robosoft_interfaces::msg::LocalizationMode::GNSS_LIDAR_EKF;
  } else if (msg.mode == REMOTE_AUTO_3) {
    localization_mode.mode =
      robosoft_interfaces::msg::LocalizationMode::LANDMARK_DEAD_RECKONING;
  }
  localization_mode_pub_->publish(localization_mode);
}

void AkiMainNode::onCommand(
  const robosoft_interfaces::msg::RobotCommand & msg)
{
  if (msg.mode == robosoft_interfaces::msg::RobotCommand::MODE_EMERGENCY_STOP) {
    software_emergency_stop_ = true;
    updateEmergencyStopState();
    requestTaskStatus(TASK_PAUSED);
    transition(State::MAIN, SubState::MAIN_STOP);
    return;
  }

  software_emergency_stop_ = false;
  updateEmergencyStopState();
  if (msg.action == robosoft_interfaces::msg::RobotCommand::ACTION_START &&
    msg.mode == robosoft_interfaces::msg::RobotCommand::MODE_AUTO)
  {
    // GUI START is the operator authorization equivalent of pressing the
    // physical remote's start and deadman controls together.
    gui_start_requested_ = true;
  }

  if (msg.action == robosoft_interfaces::msg::RobotCommand::ACTION_PAUSE ||
    msg.mode == robosoft_interfaces::msg::RobotCommand::MODE_STOP)
  {
    gui_start_requested_ = false;
    requestTaskStatus(TASK_PAUSED);
    transition(State::MAIN, SubState::MAIN_STOP);
  } else if (msg.action ==
    robosoft_interfaces::msg::RobotCommand::ACTION_RECORD_ON)
  {
    requestTaskStatus(TASK_RUNNING, -1);
  } else if (msg.action ==
    robosoft_interfaces::msg::RobotCommand::ACTION_RECORD_OFF)
  {
    requestTaskStatus(TASK_PAUSED);
  } else if ((
      msg.action == robosoft_interfaces::msg::RobotCommand::ACTION_START ||
      msg.action == robosoft_interfaces::msg::RobotCommand::ACTION_RESUME) &&
    msg.mode == robosoft_interfaces::msg::RobotCommand::MODE_AUTO)
  {
    if (!remoteAutomatic()) {
      gui_start_requested_ = false;
      RCLCPP_WARN(
        get_logger(),
        "GUI start rejected: physical remote is not in AUTO mode");
    } else if (substate_ != SubState::MAIN_STOP) {
      requestTaskStatus(TASK_PAUSED);
      transition(State::MAIN, SubState::MAIN_STOP);
    } else if (automaticStartPermitted()) {
      requestTaskStatus(TASK_RUNNING);
      gui_start_requested_ = false;
    } else {
      gui_start_requested_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Automatic start rejected: route, GNSS, TECU, estimator or safety "
        "condition is not ready");
    }
  }
}

void AkiMainNode::onOdometry(
  const nav_msgs::msg::Odometry &)
{
  odometry_received_ = true;
}

void AkiMainNode::onGps(const sensor_msgs::msg::NavSatFix & msg)
{
  gps_valid_ =
    msg.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(msg.latitude) && std::isfinite(msg.longitude);
  if (gps_valid_) {
    last_gps_fix_ = now();
  }
}

void AkiMainNode::onRouteStatus(
  const robosoft_interfaces::msg::RouteStatus & msg)
{
  if (msg.state == robosoft_interfaces::msg::RouteStatus::STATE_ERROR &&
    substate_ == SubState::MAIN_AUTO)
  {
    requestTaskStatus(TASK_PAUSED);
    robot_state_.error_code = 2;
    robot_state_.error_message = "RouteControl: " + msg.message;
    transition(State::MAIN, SubState::MAIN_STOP);
    return;
  }
  // The UVC coordinator triggers state 1 sequences only after motion has
  // stopped. Other implement states remain top-level state-machine commands.
  if (substate_ == SubState::MAIN_AUTO &&
    msg.target_point.implement_state <=
    robosoft_interfaces::msg::RoutePoint::IMPLEMENT_SAFE &&
    msg.target_point.implement_state !=
    robosoft_interfaces::msg::RoutePoint::IMPLEMENT_HEADLAND)
  {
    commandImplement(msg.target_point.implement_state);
  }
}

void AkiMainNode::onLidarSafety(
  const robosoft_interfaces::msg::LidarSafetyStatus & msg)
{
  lidar_safety_received_ = true;
  lidar_healthy_ = msg.healthy;
}

void AkiMainNode::onImplementStatus(
  const robosoft_interfaces::msg::ImplementStatus &)
{
  implement_status_received_ = true;
}

void AkiMainNode::onStateEstimatorStatus(
  const robosoft_interfaces::msg::StateEstimatorStatus & msg)
{
  estimator_status_ = msg;
  estimator_status_received_ = true;
  last_estimator_status_ = now();
}

void AkiMainNode::onTecuWheelSpeed(
  const ros2_isobus::msg::TecuWheelSpeed &)
{
  tecu_wheel_speed_received_ = true;
  last_tecu_wheel_speed_ = now();
}

void AkiMainNode::onTecuGuidanceStatus(
  const ros2_isobus::msg::TecuGuidanceStatus &)
{
  tecu_guidance_status_received_ = true;
  last_tecu_guidance_status_ = now();
}

void AkiMainNode::runStateMachine()
{
  if (state_ == State::INIT) {
    checkInitialization();
    publishState();
    return;
  }

  if (gps_valid_ && (now() - last_gps_fix_).seconds() > 2.0) {
    gps_valid_ = false;
  }

  // A disconnected physical remote stops every active operation. In AUTO the
  // emergency-stop/safety circuit must remain OK. Manual driving, including
  // route recording, is armed by separate enable controls in the lower-level
  // controller and does not repurpose the safety-circuit status as a button.
  const bool remote_lost = !remote_received_ || !remote_.connected;
  const bool automatic_deadman_released =
    substate_ == SubState::MAIN_AUTO && !remote_.safety;
  if ((remote_lost || automatic_deadman_released) &&
    substate_ != SubState::MAIN_WAIT &&
    substate_ != SubState::MAIN_MANUAL)
  {
    requestTaskStatus(TASK_PAUSED);
    transition(State::MAIN, SubState::MAIN_MANUAL);
  }

  switch (substate_) {
    case SubState::MAIN_WAIT:
      if (taskLoaded()) {
        transition(State::MAIN, SubState::MAIN_STOP);
      }
      break;
    case SubState::MAIN_MANUAL:
      if (task_status_ == TASK_RUNNING) {
        transition(State::MAIN, SubState::MAIN_MANUAL_RECORD);
      } else if (remoteAutomatic()) {
        transition(State::MAIN, SubState::MAIN_STOP);
      }
      break;
    case SubState::MAIN_MANUAL_RECORD:
      if (!gps_valid_) {
        requestTaskStatus(TASK_PAUSED);
        transition(State::MAIN, SubState::MAIN_MANUAL);
      } else if (task_status_ != TASK_RUNNING) {
        transition(State::MAIN, SubState::MAIN_MANUAL);
      } else if (remote_.mode == REMOTE_SAFE || remoteAutomatic()) {
        requestTaskStatus(TASK_PAUSED);
        transition(State::MAIN, SubState::MAIN_STOP);
      }
      break;
    case SubState::MAIN_AUTO:
      if (!tecuDataFresh()) {
        requestTaskStatus(TASK_PAUSED);
        robot_state_.error_code = 3;
        robot_state_.error_message = "TECU data timeout";
        transition(State::MAIN, SubState::MAIN_STOP);
      } else if (
        estimatorRequired() &&
        (!estimatorStatusFresh() || !estimator_status_.reliable))
      {
        requestTaskStatus(TASK_PAUSED);
        robot_state_.error_code = 4;
        robot_state_.error_message = "State estimate unreliable";
        transition(State::MAIN, SubState::MAIN_STOP);
      } else if (!gps_valid_ || task_status_ != TASK_RUNNING) {
        requestTaskStatus(TASK_PAUSED);
        transition(State::MAIN, SubState::MAIN_STOP);
      } else if (remoteManualOrSafe()) {
        requestTaskStatus(TASK_PAUSED);
        transition(State::MAIN, SubState::MAIN_MANUAL);
      }
      break;
    case SubState::MAIN_STOP:
      if (remoteManualOrSafe()) {
        transition(State::MAIN, SubState::MAIN_MANUAL);
      } else if (task_status_ == TASK_RUNNING) {
        if (automaticOperationPermitted()) {
          transition(State::MAIN, SubState::MAIN_AUTO);
        } else {
          requestTaskStatus(TASK_PAUSED);
        }
      } else if (automaticStartPermitted() &&
        (remote_.start || gui_start_requested_))
      {
        requestTaskStatus(TASK_RUNNING);
        gui_start_requested_ = false;
      }
      break;
    default:
      transition(State::MAIN, SubState::MAIN_WAIT);
      break;
  }
  publishState();
}

void AkiMainNode::checkInitialization()
{
  if (substate_ == SubState::INIT_WAIT_NODES) {
    RobotMainNodeBase::checkRequiredNodes(required_nodes_);
  } else if (substate_ == SubState::INIT_WAIT_DATA) {
    checkRequiredData();
  }
}

void AkiMainNode::checkRequiredData()
{
  std::vector<std::string> missing;
  if (!remote_received_ || !remote_.connected) {
    missing.emplace_back("remote control");
  }
  if (!gps_valid_) {
    missing.emplace_back("valid GNSS fix");
  }
  if (!odometry_received_) {
    missing.emplace_back("odometry");
  }
  if (lidar_enabled_ && (!lidar_safety_received_ || !lidar_healthy_)) {
    missing.emplace_back("healthy lidar");
  }
  if (!implement_status_received_) {
    missing.emplace_back("UVC/LED status");
  }
  if (!tecu_wheel_speed_received_ ||
    (now() - last_tecu_wheel_speed_).seconds() > tecu_timeout_s_)
  {
    missing.emplace_back("TECU wheel speed");
  }
  if (!tecu_guidance_status_received_ ||
    (now() - last_tecu_guidance_status_).seconds() > tecu_timeout_s_)
  {
    missing.emplace_back("TECU guidance status");
  }
  if (!estimatorStatusFresh()) {
    missing.emplace_back("state estimator status");
  }

  if (missing.empty()) {
    initialization_complete_ = true;
    robot_state_.is_connected = true;
    robot_state_.error_code = 0;
    robot_state_.error_message.clear();
    previously_missing_data_.clear();
    transition(State::MAIN, SubState::MAIN_WAIT);
    return;
  }

  std::string names;
  for (const auto & name : missing) {
    names += (names.empty() ? "" : ", ") + name;
  }
  robot_state_.is_connected = false;
  robot_state_.error_message = "messages: " + names;
  if (missing != previously_missing_data_) {
    RCLCPP_INFO(get_logger(), "INIT waiting for data: %s", names.c_str());
    previously_missing_data_ = std::move(missing);
  }
}

void AkiMainNode::onEnterState(SubState substate)
{
  robot_state_.mode =
    substate == SubState::MAIN_AUTO ||
    (substate == SubState::MAIN_STOP && remoteAutomatic()) ? 1 : 0;

  switch (substate) {
    case SubState::INIT_WAIT_DATA:
      robot_state_.error_message = "Waiting for required data";
      break;
    case SubState::MAIN_WAIT:
      commandImplement(robosoft_interfaces::msg::ImplementCommand::MODE_SAFE);
      robot_state_.error_message = "Waiting for task";
      break;
    case SubState::MAIN_MANUAL:
      commandImplement(robosoft_interfaces::msg::ImplementCommand::MODE_SAFE);
      robot_state_.error_message = "Manual";
      break;
    case SubState::MAIN_MANUAL_RECORD:
      commandImplement(robosoft_interfaces::msg::ImplementCommand::MODE_SAFE);
      robot_state_.error_message = "Manual recording";
      break;
    case SubState::MAIN_AUTO:
      robot_state_.error_message = "Automatic";
      break;
    case SubState::MAIN_STOP:
      commandImplement(
        robosoft_interfaces::msg::ImplementCommand::MODE_TRANSPORT);
      if (robot_state_.error_code == 0) {
        robot_state_.error_message = "Stopped";
      }
      break;
    default:
      robot_state_.error_message = "Initializing";
      break;
  }
}

void AkiMainNode::commandImplement(uint8_t mode)
{
  if (mode > 3 || mode == commanded_implement_mode_) {
    return;
  }
  commanded_implement_mode_ = mode;
  robosoft_interfaces::msg::ImplementCommand command;
  command.mode = mode;
  command.timestamp = now().nanoseconds();
  implement_pub_->publish(command);
}

void AkiMainNode::updateEmergencyStopState()
{
  robot_state_.emergency_stop =
    software_emergency_stop_ || remote_emergency_stop_;
  if (robot_state_.emergency_stop) {
    robot_state_.error_code = 1;
    robot_state_.error_message = remote_emergency_stop_ ?
      "REMOTE EMERGENCY STOP" : "EMERGENCY STOP";
  } else if (robot_state_.error_code == 1) {
    robot_state_.error_code = 0;
    robot_state_.error_message.clear();
  }
}

bool AkiMainNode::remoteAutomatic() const
{
  return remote_.connected && remote_.safety &&
    remote_.mode >= REMOTE_AUTO_1 && remote_.mode <= REMOTE_AUTO_3;
}

bool AkiMainNode::remoteManualOrSafe() const
{
  return !remote_.connected || !remote_.safety ||
    remote_.mode == REMOTE_SAFE || remote_.mode == REMOTE_MANUAL ||
    remote_.mode == REMOTE_DIRECT_BODY ||
    remote_.mode == REMOTE_DIRECT_IMPLEMENT;
}

bool AkiMainNode::automaticStartPermitted() const
{
  return automaticOperationPermitted() &&
    (gui_start_requested_ || remote_.deadman);
}

bool AkiMainNode::automaticOperationPermitted() const
{
  const bool remote_safety_ok =
    remote_.connected && remote_.safety;
  return remoteAutomatic() && remote_safety_ok &&
    routeAvailable() && gps_valid_ &&
    tecuDataFresh() &&
    (!estimatorRequired() ||
    (estimatorStatusFresh() && estimator_status_.reliable)) &&
    !robot_state_.emergency_stop;
}

bool AkiMainNode::tecuDataFresh() const
{
  return tecu_wheel_speed_received_ && tecu_guidance_status_received_ &&
    (now() - last_tecu_wheel_speed_).seconds() <= tecu_timeout_s_ &&
    (now() - last_tecu_guidance_status_).seconds() <= tecu_timeout_s_;
}

bool AkiMainNode::estimatorStatusFresh() const
{
  return estimator_status_received_ &&
    (now() - last_estimator_status_).seconds() <= estimator_timeout_s_;
}

bool AkiMainNode::estimatorRequired() const
{
  return remote_.mode == REMOTE_AUTO_2 || remote_.mode == REMOTE_AUTO_3;
}

}  // namespace robosoft_aki

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_aki::AkiMainNode>());
  rclcpp::shutdown();
  return 0;
}
