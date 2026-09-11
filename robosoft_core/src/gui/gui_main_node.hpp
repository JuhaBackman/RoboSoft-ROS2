// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <deque>
#include <memory>

#include <QObject>
#include <QString>
#include <QHash>
#include <QProcess>
#include <QStringList>
#include <QVariantList>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <robosoft_interfaces/msg/lidar_cluster_observation_array.hpp>
#include <robosoft_interfaces/msg/lidar_safety_status.hpp>
#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/msg/timer_performance.hpp>
#include <robosoft_interfaces/msg/robot_command.hpp>
#include <robosoft_interfaces/msg/route_status.hpp>
#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <robosoft_interfaces/srv/select_task.hpp>
#include <robosoft_interfaces/srv/save_task.hpp>
#include <robosoft_interfaces/srv/modify_task_status.hpp>

namespace robosoft_core
{

/**
 * @brief ROS2 Node bridge for Qt6 GUI
 *
 * Bridges ROS2 pub/sub with Qt signals/slots.
 * Exposed to QML via Q_INVOKABLE methods and Q_SIGNALS.
 *
 * Threading:
 * - Qt event loop runs on main thread
 * - ROS2 spin runs on separate background thread
 */
class GuiNode : public QObject {
  Q_OBJECT
  Q_PROPERTY(QString robotName READ getRobotName CONSTANT)
  Q_PROPERTY(QString robotDisplayName READ getRobotDisplayName CONSTANT)
  Q_PROPERTY(QString applicationTitle READ getApplicationTitle CONSTANT)
  Q_PROPERTY(QString defaultTaskFile READ getDefaultTaskFile CONSTANT)
  Q_PROPERTY(bool guiModeSelectionEnabled READ getGuiModeSelectionEnabled CONSTANT)
  Q_PROPERTY(bool virtualKeyboardEnabled READ getVirtualKeyboardEnabled CONSTANT)

public:
  explicit GuiNode(
    rclcpp::Node::SharedPtr ros_node,
    QObject *parent = nullptr);
  virtual ~GuiNode();

  // Task management (callable from QML)
  Q_INVOKABLE void selectTask(const QString & task_name);
  Q_INVOKABLE void saveTask(const QString & task_name);

  // Robot control (callable from QML)
  Q_INVOKABLE void setRobotMode(uint8_t mode);     // 0=manual, 1=auto
  Q_INVOKABLE void publishCommand(
    float speed, float steering,
    uint8_t implement_mode);
  Q_INVOKABLE void startAutonomous();
  Q_INVOKABLE void stopAutonomous();
  Q_INVOKABLE void startRecording();
  Q_INVOKABLE void stopRecording();
  Q_INVOKABLE void selectGuidance(int index);
  Q_INVOKABLE void emergencyStop();
  Q_INVOKABLE void setImplementMode(uint8_t mode);

  // Generic topic inspection for the Debug page. ros2 topic echo performs
  // runtime type discovery, so this also works with message packages that the
  // GUI was not compiled against.
  Q_INVOKABLE QStringList availableTopics() const;
  Q_INVOKABLE void startTopicMonitor(const QString & topic_name);
  Q_INVOKABLE void stopTopicMonitor(const QString & topic_name);

  // Status queries (callable from QML)
  Q_INVOKABLE QString getRobotName() const;
  Q_INVOKABLE QString getRobotDisplayName() const;
  Q_INVOKABLE QString getApplicationTitle() const;
  Q_INVOKABLE QString getDefaultTaskFile() const;
  Q_INVOKABLE bool getGuiModeSelectionEnabled() const;
  Q_INVOKABLE bool getVirtualKeyboardEnabled() const;
  Q_INVOKABLE QStringList availableTaskFiles() const;
  Q_INVOKABLE QString getCurrentState() const;
  Q_INVOKABLE QString getLastError() const;

Q_SIGNALS:
  // Signals emitted to QML
  void robotStateChanged(const QString & state, const QString & substate);
  void setupStatusChanged(
    const QString & ros_status, const QString & main_state,
    const QString & waiting_for);
  void robotModeChanged(bool automatic);
  void taskLoaded(const QString & task_name);
  void taskSaved(const QString & filepath);
  void odometryUpdated(float x, float y, float heading, float velocity);
  void lateralErrorUpdated(float error);
  void lidarLandmarksUpdated(const QVariantList & points);
  void lidarObservationsUpdated(const QVariantList & points);
  void timerPerformanceUpdated(
    const QString & component, float targetPeriodMs, float actualPeriodMs,
    float callbackDurationMs, float marginMs, float latenessMs,
    qulonglong deadlineMisses);
  void timerFrequencyUpdated(
    const QString & component, float averageHz, float targetHz,
    int sampleCount);
  void safetyStopChanged(bool lidar_stop, bool emergency_stop);
  void remoteOdometryUpdated(
    const QString & name, float x, float y, float heading, float velocity);
  void remotePathUpdated(const QString & name, const QVariantList & points);
  void diagnosticUpdate(
    const QString & component, const QString & level,
    const QString & message);
  void errorOccurred(const QString & error_message);
  void connectionStatusChanged(bool connected);
  void topicSample(const QString & topic_name, const QString & yaml);
  void topicMonitorError(const QString & topic_name, const QString & message);
  void routeUpdated(const QVariantList & points);
  void activeRouteUpdated(const QVariantList & points);
  void guidanceLinesChanged(const QStringList & names, int active_index);

private:
  // ROS2 subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::LidarClusterObservationArray>::SharedPtr
    lidar_observations_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::LidarSafetyStatus>::SharedPtr
    lidar_safety_sub_;
  void onRobotStateMessage(
    const robosoft_interfaces::msg::RobotState & msg);
  void onOdometryMessage(
    const nav_msgs::msg::Odometry & msg);
  void onRouteStatusMessage(
    const robosoft_interfaces::msg::RouteStatus & msg);
  void onRosoutMessage(const rcl_interfaces::msg::Log & msg);
  void onTaskLoadedMessage(
    const robosoft_interfaces::msg::TaskData & msg);
  void onRoutePathMessage(const nav_msgs::msg::Path & msg);
  void onActiveRoutePathMessage(const nav_msgs::msg::Path & msg);
  void onTimerPerformanceMessage(
    const QString & component,
    const robosoft_interfaces::msg::TimerPerformance & msg);
  void discoverRemoteRobotTopics();

  // ROS2 service calls
  void selectTaskAsync(const QString & task_name);
  void saveTaskAsync(
    const QString & task_name,
    const robosoft_interfaces::msg::TaskData & task);

  rclcpp::Node::SharedPtr node_;

  // Subscriptions
  rclcpp::Subscription<robosoft_interfaces::msg::RobotState>::SharedPtr
    robot_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::RouteStatus>::SharedPtr
    route_status_sub_;
  QHash<QString, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
    remote_odometry_subs_;
  QHash<QString, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr>
    remote_path_subs_;
  rclcpp::TimerBase::SharedPtr remote_topic_discovery_timer_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::TaskData>::SharedPtr
    task_loaded_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr active_route_path_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::TimerPerformance>::SharedPtr
    main_timer_performance_sub_;
  rclcpp::Subscription<robosoft_interfaces::msg::TimerPerformance>::SharedPtr
    path_tracking_timer_performance_sub_;

  // Publishers
  rclcpp::Publisher<robosoft_interfaces::msg::RobotCommand>::SharedPtr
    command_pub_;

  // Service clients
  rclcpp::Client<robosoft_interfaces::srv::SelectTask>::SharedPtr
    select_task_client_;
  rclcpp::Client<robosoft_interfaces::srv::SaveTask>::SharedPtr
    save_task_client_;
  rclcpp::Client<robosoft_interfaces::srv::ModifyTaskStatus>::SharedPtr
    modify_status_client_;

  // State cache
  robosoft_interfaces::msg::RobotState current_state_;
  nav_msgs::msg::Odometry current_odometry_;
  robosoft_interfaces::msg::TaskData current_task_;
  QString robot_name_;
  QString robot_display_name_;
  QString application_title_;
  QString default_task_file_;
  QString task_directory_;
  bool gui_mode_selection_enabled_{true};
  bool is_connected_;
  bool lidar_stop_active_{false};
  bool emergency_stop_active_{false};

  struct FrequencyWindow
  {
    std::deque<double> periods_ms;
    double period_sum_ms{0.0};
  };
  QHash<QString, FrequencyWindow> frequency_windows_;

  QHash<QString, QProcess *> topic_processes_;
  QHash<QString, QString> topic_buffers_;
};

}  // namespace robosoft_core
