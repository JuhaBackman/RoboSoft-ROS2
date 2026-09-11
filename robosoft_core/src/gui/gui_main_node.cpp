// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "gui_main_node.hpp"
#include "robosoft_interfaces/topics.hpp"

#include <QDir>
#include <QFileInfo>
#include <QVariantMap>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace robosoft_core
{

GuiNode::GuiNode(rclcpp::Node::SharedPtr ros_node, QObject *parent)
: QObject(parent), node_(std::move(ros_node)), is_connected_(false)
{
  robot_name_ = QString::fromStdString(
    node_->declare_parameter<std::string>("robot_name", "robot"));
  robot_display_name_ = QString::fromStdString(
    node_->declare_parameter<std::string>("robot_display_name", "ROBOT"));
  application_title_ = QString::fromStdString(
    node_->declare_parameter<std::string>(
      "application_title", "RoboSoft GUI"));
  default_task_file_ = QString::fromStdString(
    node_->declare_parameter<std::string>(
      "default_task_file", "EMPTY_TASK.XML"));
  task_directory_ = QString::fromStdString(
    node_->declare_parameter<std::string>(
      "task_directory", "/opt/ros2_ws/tasks"));
  gui_mode_selection_enabled_ =
    node_->declare_parameter<bool>("gui_mode_selection_enabled", true);

  // Create subscriptions
  landmarks_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    robosoft_interfaces::kLidarLandmarksTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const geometry_msgs::msg::PoseArray & map) {
      QVariantList points;
      for (const auto & pose : map.poses) {
        if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y)) {
          points.append(QVariantMap{{"x", pose.position.x}, {"y", pose.position.y}});
        }
      }
      Q_EMIT lidarLandmarksUpdated(points);
    });
  lidar_observations_sub_ = node_->create_subscription<
    robosoft_interfaces::msg::LidarClusterObservationArray>(
    robosoft_interfaces::kLidarClusterObservationsTopic, 10,
    [this](const robosoft_interfaces::msg::LidarClusterObservationArray & message) {
      QVariantList points;
      for (const auto & observation : message.observations) {
        if (std::isfinite(observation.measurement_x) &&
          std::isfinite(observation.measurement_y))
        {
          points.append(QVariantMap{
            {"x", observation.measurement_x}, {"y", observation.measurement_y}});
        }
      }
      Q_EMIT lidarObservationsUpdated(points);
    });
  lidar_safety_sub_ = node_->create_subscription<
    robosoft_interfaces::msg::LidarSafetyStatus>(
    robosoft_interfaces::kLidarSafetyStatusTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const robosoft_interfaces::msg::LidarSafetyStatus & message) {
      const bool lidar_stop =
        !message.healthy || message.maximum_speed_m_s <= 0.0;
      if (lidar_stop != lidar_stop_active_) {
        lidar_stop_active_ = lidar_stop;
        Q_EMIT safetyStopChanged(
          lidar_stop_active_, emergency_stop_active_);
      }
    });
  robot_state_sub_ =
    node_->create_subscription<robosoft_interfaces::msg::RobotState>(
          robosoft_interfaces::kRobotStateTopic, rclcpp::QoS(10),
          std::bind(&GuiNode::onRobotStateMessage, this,
                    std::placeholders::_1));

  odometry_sub_ =
    node_->create_subscription<nav_msgs::msg::Odometry>(
          robosoft_interfaces::kOdometryTopic, rclcpp::QoS(10),
          std::bind(&GuiNode::onOdometryMessage, this,
                    std::placeholders::_1));

  route_status_sub_ =
    node_->create_subscription<robosoft_interfaces::msg::RouteStatus>(
          robosoft_interfaces::kRouteStatusTopic, rclcpp::QoS(10),
          std::bind(&GuiNode::onRouteStatusMessage, this,
                    std::placeholders::_1));

  rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      robosoft_interfaces::kRosoutTopic, rclcpp::RosoutQoS(),
      std::bind(&GuiNode::onRosoutMessage, this,
                std::placeholders::_1));

  task_loaded_sub_ =
    node_->create_subscription<robosoft_interfaces::msg::TaskData>(
          robosoft_interfaces::kTaskLoadedTopic,
          rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&GuiNode::onTaskLoadedMessage, this,
                    std::placeholders::_1));
  route_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      robosoft_interfaces::kRoutePathTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GuiNode::onRoutePathMessage, this, std::placeholders::_1));
  active_route_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      robosoft_interfaces::kActiveRoutePathTopic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &GuiNode::onActiveRoutePathMessage, this, std::placeholders::_1));
  main_timer_performance_sub_ =
    node_->create_subscription<robosoft_interfaces::msg::TimerPerformance>(
    robosoft_interfaces::kAkiMainTimerPerformanceTopic, 10,
    [this](const robosoft_interfaces::msg::TimerPerformance & message) {
      onTimerPerformanceMessage("Main", message);
    });
  path_tracking_timer_performance_sub_ =
    node_->create_subscription<robosoft_interfaces::msg::TimerPerformance>(
    robosoft_interfaces::kPathTrackingTimerPerformanceTopic, 10,
    [this](const robosoft_interfaces::msg::TimerPerformance & message) {
      onTimerPerformanceMessage("NMPC", message);
    });

  // Remote robots are visualization-only. Discovering them dynamically keeps
  // the local robot interfaces and all command routing unchanged.
  remote_topic_discovery_timer_ = node_->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&GuiNode::discoverRemoteRobotTopics, this));
  discoverRemoteRobotTopics();

  // Create publishers
  command_pub_ =
    node_->create_publisher<robosoft_interfaces::msg::RobotCommand>(
          robosoft_interfaces::kRobotCommandTopic, rclcpp::QoS(10));

  // Create service clients
  select_task_client_ =
    node_->create_client<robosoft_interfaces::srv::SelectTask>(
          robosoft_interfaces::kTaskSelectService);
  save_task_client_ = node_->create_client<robosoft_interfaces::srv::SaveTask>(
      robosoft_interfaces::kTaskSaveService);
  modify_status_client_ =
    node_->create_client<robosoft_interfaces::srv::ModifyTaskStatus>(
      robosoft_interfaces::kTaskModifyStatusService);

  is_connected_ = true;
  Q_EMIT connectionStatusChanged(true);

  RCLCPP_INFO(
    node_->get_logger(), "GuiNode initialized for robot '%s'",
    robot_name_.toStdString().c_str());
}

GuiNode::~GuiNode()
{
  const auto topics = topic_processes_.keys();
  for (const auto & topic : topics) {
    stopTopicMonitor(topic);
  }
}

void GuiNode::onTimerPerformanceMessage(
  const QString & component,
  const robosoft_interfaces::msg::TimerPerformance & msg)
{
  constexpr std::size_t frequency_window_size = 100U;
  if (std::isfinite(msg.actual_period_ms) && msg.actual_period_ms > 0.0F &&
    std::isfinite(msg.target_period_ms) && msg.target_period_ms > 0.0F)
  {
    auto & window = frequency_windows_[component];
    window.periods_ms.push_back(msg.actual_period_ms);
    window.period_sum_ms += msg.actual_period_ms;
    if (window.periods_ms.size() > frequency_window_size) {
      window.period_sum_ms -= window.periods_ms.front();
      window.periods_ms.pop_front();
    }
    const double average_hz =
      1000.0 * static_cast<double>(window.periods_ms.size()) /
      window.period_sum_ms;
    Q_EMIT timerFrequencyUpdated(
      component, static_cast<float>(average_hz),
      1000.0F / msg.target_period_ms,
      static_cast<int>(window.periods_ms.size()));
  }
  Q_EMIT timerPerformanceUpdated(
    component, msg.target_period_ms, msg.actual_period_ms,
    msg.callback_duration_ms, msg.margin_ms, msg.lateness_ms,
    static_cast<qulonglong>(msg.deadline_misses));
}

// ---------------------------------------------------------------------------
// QML command surface
// ---------------------------------------------------------------------------

void GuiNode::selectTask(const QString & task_name)
{
  selectTaskAsync(task_name);
}

void GuiNode::saveTask(const QString & task_name)
{
  if (current_task_.task_name.empty()) {
    current_task_ = robosoft_interfaces::msg::TaskData();
    current_task_.task_name = QFileInfo(task_name).completeBaseName().toStdString();
    if (current_task_.task_name.empty()) {
      Q_EMIT errorOccurred("Cannot create task: filename is empty");
      return;
    }
    current_task_.version = "1.0";
    current_task_.timestamp = node_->now().seconds();
    current_task_.active_guidance_index = -1;
    current_task_.status =
      robosoft_interfaces::msg::TaskData::STATUS_PLANNED;
  }
  saveTaskAsync(task_name, current_task_);
}

void GuiNode::setRobotMode(uint8_t mode)
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = mode;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::publishCommand(
  float speed, float steering,
  uint8_t implement_mode)
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_MANUAL;
  cmd.speed = speed;
  cmd.steering_angle = steering;
  cmd.implement_mode = implement_mode;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::startAutonomous()
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_AUTO;
  cmd.action = robosoft_interfaces::msg::RobotCommand::ACTION_START;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::stopAutonomous()
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_STOP;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::startRecording()
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_MANUAL;
  cmd.action = robosoft_interfaces::msg::RobotCommand::ACTION_RECORD_ON;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::stopRecording()
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_MANUAL;
  cmd.action = robosoft_interfaces::msg::RobotCommand::ACTION_RECORD_OFF;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::selectGuidance(int index)
{
  if (index < 0 ||
    index >= static_cast<int>(current_task_.guidance_lines.size()))
  {
    Q_EMIT errorOccurred("Cannot select route: index out of range");
    return;
  }
  if (!modify_status_client_->service_is_ready()) {
    Q_EMIT errorOccurred("Cannot select route: TaskManager unavailable");
    return;
  }
  auto request =
    std::make_shared<robosoft_interfaces::srv::ModifyTaskStatus::Request>();
  request->new_status = current_task_.status;
  request->guidance_index = index;
  modify_status_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<robosoft_interfaces::srv::ModifyTaskStatus>::SharedFuture
      future) {
      const auto response = future.get();
      if (!response->success) {
        Q_EMIT errorOccurred(QString::fromStdString(response->error_message));
      }
    });
}

void GuiNode::emergencyStop()
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  cmd.mode = robosoft_interfaces::msg::RobotCommand::MODE_EMERGENCY_STOP;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

void GuiNode::setImplementMode(uint8_t mode)
{
  auto cmd = robosoft_interfaces::msg::RobotCommand();
  // Keep the currently selected driving mode; the default value (manual)
  // must not pull the robot out of AUTO when only the implement is changed.
  cmd.mode = current_state_.mode;
  cmd.implement_mode = mode;
  cmd.timestamp = node_->now().nanoseconds();
  command_pub_->publish(cmd);
}

// ---------------------------------------------------------------------------
// Runtime topic inspector
// ---------------------------------------------------------------------------

QStringList GuiNode::availableTopics() const
{
  QStringList topics;
  const auto graph_topics = node_->get_topic_names_and_types();
  for (const auto &[name, types] : graph_topics) {
    if (!types.empty()) {
      topics.append(QString::fromStdString(name));
    }
  }
  topics.sort(Qt::CaseInsensitive);
  return topics;
}

void GuiNode::startTopicMonitor(const QString & topic_name)
{
  if (topic_name.isEmpty() || topic_processes_.contains(topic_name)) {
    return;
  }

  // Generic rclcpp subscriptions need a compile-time message type. The debug
  // page uses ros2 topic echo for runtime discovery of arbitrary message types.
  auto *process = new QProcess(this);
  topic_processes_.insert(topic_name, process);
  topic_buffers_.insert(topic_name, QString());

  connect(process, &QProcess::readyReadStandardOutput, this,
    [this, process, topic_name]() {
      QString & buffer = topic_buffers_[topic_name];
      buffer += QString::fromUtf8(process->readAllStandardOutput());

      const QString separator = QStringLiteral("\n---\n");
      qsizetype separator_pos = buffer.indexOf(separator);
      while (separator_pos >= 0) {
        const QString sample = buffer.left(separator_pos).trimmed();
        buffer.remove(0, separator_pos + separator.size());
        if (!sample.isEmpty()) {
          Q_EMIT topicSample(topic_name, sample);
        }
        separator_pos = buffer.indexOf(separator);
      }
          });

  connect(process, &QProcess::readyReadStandardError, this,
    [this, process, topic_name]() {
      const QString error =
      QString::fromUtf8(process->readAllStandardError()).trimmed();
      if (!error.isEmpty()) {
        Q_EMIT topicMonitorError(topic_name, error);
      }
          });

  connect(process, &QProcess::errorOccurred, this,
    [this, topic_name](QProcess::ProcessError) {
      auto *topic_process = topic_processes_.value(topic_name, nullptr);
      if (topic_process != nullptr) {
        Q_EMIT topicMonitorError(topic_name,
                                       topic_process->errorString());
      }
          });

  process->start(QStringLiteral("ros2"),
    {QStringLiteral("topic"), QStringLiteral("echo"), topic_name,
      QStringLiteral("--qos-reliability"),
      QStringLiteral("best_effort"),
      QStringLiteral("--qos-durability"),
      QStringLiteral("volatile")});
}

void GuiNode::stopTopicMonitor(const QString & topic_name)
{
  QProcess *process = topic_processes_.take(topic_name);
  topic_buffers_.remove(topic_name);
  if (process == nullptr) {
    return;
  }

  if (process->state() != QProcess::NotRunning) {
    process->terminate();
    if (!process->waitForFinished(1000)) {
      process->kill();
      process->waitForFinished(1000);
    }
  }
  process->deleteLater();
}

// ---------------------------------------------------------------------------
// Cached state exposed to QML
// ---------------------------------------------------------------------------

QString GuiNode::getRobotName() const {return robot_name_;}

QString GuiNode::getRobotDisplayName() const {return robot_display_name_;}

QString GuiNode::getApplicationTitle() const {return application_title_;}

QString GuiNode::getDefaultTaskFile() const {return default_task_file_;}

bool GuiNode::getGuiModeSelectionEnabled() const
{
  return gui_mode_selection_enabled_;
}

bool GuiNode::getVirtualKeyboardEnabled() const
{
#ifdef ROBOSOFT_USE_VIRTUAL_KEYBOARD
  return true;
#else
  return false;
#endif
}

QStringList GuiNode::availableTaskFiles() const
{
  QDir task_directory(task_directory_);
  task_directory.setNameFilters({"*.xml", "*.XML"});
  task_directory.setFilter(QDir::Files | QDir::Readable);
  task_directory.setSorting(QDir::Name | QDir::IgnoreCase);
  return task_directory.entryList();
}

QString GuiNode::getCurrentState() const
{
  switch (current_state_.state_machine) {
    case robosoft_interfaces::msg::RobotState::STATE_INIT:
      return "INIT";
    case robosoft_interfaces::msg::RobotState::STATE_MAIN:
      switch (current_state_.substate) {
        case robosoft_interfaces::msg::RobotState::SUBSTATE_WAIT: return "WAIT";
        case robosoft_interfaces::msg::RobotState::SUBSTATE_MANUAL: return "MANUAL";
        case robosoft_interfaces::msg::RobotState::SUBSTATE_MANUAL_RECORD:
          return "MANUAL_RECORD";
        case robosoft_interfaces::msg::RobotState::SUBSTATE_AUTO: return "AUTO";
        case robosoft_interfaces::msg::RobotState::SUBSTATE_STOP: return "STOP";
        default: return "MAIN";
      }
    case robosoft_interfaces::msg::RobotState::STATE_EXIT:
      return "EXIT";
    default:
      return "UNKNOWN";
  }
}

QString GuiNode::getLastError() const
{
  return QString::fromStdString(current_state_.error_message);
}

// ---------------------------------------------------------------------------
// ROS-to-Qt callbacks
// ---------------------------------------------------------------------------

void GuiNode::onRobotStateMessage(
  const robosoft_interfaces::msg::RobotState & msg)
{
  if (QString::fromStdString(msg.robot_name).compare(
      robot_name_, Qt::CaseInsensitive) != 0)
  {
    return;
  }
  current_state_ = msg;
  if (msg.emergency_stop != emergency_stop_active_) {
    emergency_stop_active_ = msg.emergency_stop;
    Q_EMIT safetyStopChanged(lidar_stop_active_, emergency_stop_active_);
  }
  Q_EMIT robotStateChanged(QString::number(msg.state_machine),
                          QString::number(msg.substate));
  Q_EMIT setupStatusChanged(
    "CONNECTED", getCurrentState(),
    msg.state_machine == robosoft_interfaces::msg::RobotState::STATE_INIT ?
    QString::fromStdString(msg.error_message) : QStringLiteral("-"));
  Q_EMIT robotModeChanged(
    msg.mode == robosoft_interfaces::msg::RobotState::MODE_AUTO);
}

void GuiNode::onOdometryMessage(
  const nav_msgs::msg::Odometry & msg)
{
  current_odometry_ = msg;
  const auto & q = msg.pose.pose.orientation;
  const double heading = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  Q_EMIT odometryUpdated(
    msg.pose.pose.position.x, msg.pose.pose.position.y, heading,
    msg.twist.twist.linear.x);
}

void GuiNode::onRouteStatusMessage(
  const robosoft_interfaces::msg::RouteStatus & msg)
{
  Q_EMIT lateralErrorUpdated(msg.cross_track_error);
}

void GuiNode::discoverRemoteRobotTopics()
{
  constexpr auto prefix = "/robots/";
  constexpr auto odometry_suffix = "/odometry";
  constexpr auto path_suffix = "/path";

  const auto graph_topics = node_->get_topic_names_and_types();
  for (const auto & [topic, types] : graph_topics) {
    if (topic.rfind(prefix, 0) != 0) {
      continue;
    }

    const auto add_odometry_subscription = [&]() {
        const std::string name = topic.substr(
          std::char_traits<char>::length(prefix),
          topic.size() - std::char_traits<char>::length(prefix) -
          std::char_traits<char>::length(odometry_suffix));
        if (name.empty() || name.find('/') != std::string::npos) {
          return;
        }
        const QString qtopic = QString::fromStdString(topic);
        if (remote_odometry_subs_.contains(qtopic)) {
          return;
        }
        const QString qname = QString::fromStdString(name);
        remote_odometry_subs_.insert(
          qtopic, node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS(),
            [this, qname](const nav_msgs::msg::Odometry & msg) {
              const auto & q = msg.pose.pose.orientation;
              const double heading = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
              Q_EMIT remoteOdometryUpdated(
                qname, msg.pose.pose.position.x, msg.pose.pose.position.y,
                heading, msg.twist.twist.linear.x);
            }));
        RCLCPP_INFO(
          node_->get_logger(), "Displaying remote robot '%s' from %s",
          name.c_str(), topic.c_str());
      };

    const auto add_path_subscription = [&]() {
        const std::string name = topic.substr(
          std::char_traits<char>::length(prefix),
          topic.size() - std::char_traits<char>::length(prefix) -
          std::char_traits<char>::length(path_suffix));
        if (name.empty() || name.find('/') != std::string::npos) {
          return;
        }
        const QString qtopic = QString::fromStdString(topic);
        if (remote_path_subs_.contains(qtopic)) {
          return;
        }
        const QString qname = QString::fromStdString(name);
        remote_path_subs_.insert(
          qtopic, node_->create_subscription<nav_msgs::msg::Path>(
            topic, rclcpp::SensorDataQoS(),
            [this, qname](const nav_msgs::msg::Path & msg) {
              QVariantList points;
              points.reserve(static_cast<qsizetype>(msg.poses.size()));
              for (const auto & pose : msg.poses) {
                QVariantMap point;
                point.insert("x", pose.pose.position.x);
                point.insert("y", pose.pose.position.y);
                points.append(point);
              }
              Q_EMIT remotePathUpdated(qname, points);
            }));
      };

    const bool is_odometry = topic.size() > std::char_traits<char>::length(
      prefix) + std::char_traits<char>::length(odometry_suffix) &&
      topic.compare(topic.size() - std::char_traits<char>::length(odometry_suffix),
      std::char_traits<char>::length(odometry_suffix), odometry_suffix) == 0;
    const bool is_path = topic.size() > std::char_traits<char>::length(prefix) +
      std::char_traits<char>::length(path_suffix) &&
      topic.compare(topic.size() - std::char_traits<char>::length(path_suffix),
      std::char_traits<char>::length(path_suffix), path_suffix) == 0;

    if (is_odometry &&
      std::find(types.begin(), types.end(), "nav_msgs/msg/Odometry") != types.end())
    {
      add_odometry_subscription();
    } else if (is_path &&
      std::find(types.begin(), types.end(), "nav_msgs/msg/Path") != types.end())
    {
      add_path_subscription();
    }
  }
}

void GuiNode::onRosoutMessage(const rcl_interfaces::msg::Log & msg)
{
  // /rosout is the only GUI log source. The logger name is used as the
  // component group in the operator log view.
  QString level_str;
  switch (msg.level) {
    case rcl_interfaces::msg::Log::DEBUG:
      level_str = "DEBUG";
      break;
    case rcl_interfaces::msg::Log::INFO:
      level_str = "INFO";
      break;
    case rcl_interfaces::msg::Log::WARN:
      level_str = "WARN";
      break;
    case rcl_interfaces::msg::Log::ERROR:
      level_str = "ERROR";
      break;
    case rcl_interfaces::msg::Log::FATAL:
      level_str = "FATAL";
      break;
    default:
      level_str = "UNKNOWN";
  }
  Q_EMIT diagnosticUpdate(QString::fromStdString(msg.name), level_str,
                          QString::fromStdString(msg.msg));
}

void GuiNode::onTaskLoadedMessage(
  const robosoft_interfaces::msg::TaskData & msg)
{
  current_task_ = msg;
  RCLCPP_INFO(
    node_->get_logger(), "Task '%s' received: %zu route(s), active index %d",
    msg.task_name.c_str(), msg.guidance_lines.size(),
    msg.active_guidance_index);
  Q_EMIT taskLoaded(QString::fromStdString(msg.task_name));
  QStringList names;
  for (const auto & line : msg.guidance_lines) {
    names.push_back(QString::fromStdString(line.name));
  }
  Q_EMIT guidanceLinesChanged(names, msg.active_guidance_index);
}

void GuiNode::onRoutePathMessage(const nav_msgs::msg::Path & msg)
{
  QVariantList points;
  points.reserve(static_cast<qsizetype>(msg.poses.size()));
  for (const auto & pose : msg.poses) {
    QVariantMap item;
    item["x"] = pose.pose.position.x;
    item["y"] = pose.pose.position.y;
    points.push_back(item);
  }
  RCLCPP_INFO(
    node_->get_logger(), "Displaying local route path with %zu point(s)",
    msg.poses.size());
  Q_EMIT routeUpdated(points);
}

void GuiNode::onActiveRoutePathMessage(const nav_msgs::msg::Path & msg)
{
  QVariantList points;
  points.reserve(static_cast<qsizetype>(msg.poses.size()));
  for (const auto & pose : msg.poses) {
    QVariantMap item;
    item["x"] = pose.pose.position.x;
    item["y"] = pose.pose.position.y;
    points.push_back(item);
  }
  Q_EMIT activeRouteUpdated(points);
}

// ---------------------------------------------------------------------------
// Asynchronous TaskManager clients
// ---------------------------------------------------------------------------

void GuiNode::selectTaskAsync(const QString & task_name)
{
  if (!select_task_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(),
                "SelectTask service not ready, waiting...");
    return;
  }

  auto request =
    std::make_shared<robosoft_interfaces::srv::SelectTask::Request>();
  request->task_name = task_name.toStdString();
  request->robot_name = robot_name_.toStdString();

  select_task_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<robosoft_interfaces::srv::SelectTask>::SharedFuture
      future) {
      const auto response = future.get();
      if (!response->success) {
        Q_EMIT errorOccurred(
          QString::fromStdString(response->error_message));
      }
    });
}

void GuiNode::saveTaskAsync(
  const QString & task_name,
  const robosoft_interfaces::msg::TaskData & task)
{
  if (!save_task_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "SaveTask service not ready, waiting...");
    return;
  }

  auto request =
    std::make_shared<robosoft_interfaces::srv::SaveTask::Request>();
  request->task_name = task_name.toStdString();
  request->task_data = task;

  save_task_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<robosoft_interfaces::srv::SaveTask>::SharedFuture
      future) {
      const auto response = future.get();
      if (!response->success) {
        Q_EMIT errorOccurred(
          QString::fromStdString(response->error_message));
        return;
      }
      Q_EMIT taskSaved(QString::fromStdString(response->file_path));
    });
}

}  // namespace robosoft_core
