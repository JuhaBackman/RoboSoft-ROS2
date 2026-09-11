// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <robosoft_interfaces/srv/select_task.hpp>
#include <robosoft_interfaces/srv/save_task.hpp>
#include <robosoft_interfaces/srv/update_lidar_clusters.hpp>
#include <robosoft_interfaces/srv/modify_task_status.hpp>
#include "task.hpp"
#include <memory>

namespace robosoft_core
{

/**
 * @brief Task Manager Node
 *
 * Owns the currently selected task, persistent Task XML I/O and task status.
 * RouteControl publishes completed recorded patterns to this node; the
 * patterns are appended to the active task and optionally autosaved.
 *
 * Services:
 *   - task_manager/select_task (SelectTask)
 *   - task_manager/save_task (SaveTask)
 *   - task_manager/modify_status (ModifyTaskStatus)
 *
 * Publishers:
 *   - task_manager/task_loaded (transient-local TaskData snapshot)
 *
 * Subscriptions:
 *   - route_control/recorded_route (completed GuidanceLine segment)
 */
class TaskManagerNode : public rclcpp::Node {
public:
  explicit TaskManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~TaskManagerNode();

private:
  // ROS interfaces.
  rclcpp::Service<robosoft_interfaces::srv::SelectTask>::SharedPtr
    select_task_service_;
  rclcpp::Service<robosoft_interfaces::srv::SaveTask>::SharedPtr
    save_task_service_;
  rclcpp::Service<robosoft_interfaces::srv::UpdateLidarClusters>::SharedPtr
    update_lidar_clusters_service_;
  rclcpp::Service<robosoft_interfaces::srv::ModifyTaskStatus>::SharedPtr
    modify_status_service_;

  // Publishers
  rclcpp::Publisher<robosoft_interfaces::msg::TaskData>::SharedPtr
    task_pub_;
  rclcpp::Subscription<robosoft_interfaces::msg::GuidanceLine>::SharedPtr
    recorded_route_sub_;

  // Service and route-recording callbacks.
  void onSelectTask(
    const std::shared_ptr<robosoft_interfaces::srv::SelectTask::Request>
    request,
    std::shared_ptr<robosoft_interfaces::srv::SelectTask::Response>
    response);

  void onSaveTask(
    const std::shared_ptr<robosoft_interfaces::srv::SaveTask::Request>
    request,
    std::shared_ptr<robosoft_interfaces::srv::SaveTask::Response>
    response);
  void onUpdateLidarClusters(
    const std::shared_ptr<
      robosoft_interfaces::srv::UpdateLidarClusters::Request> request,
    std::shared_ptr<
      robosoft_interfaces::srv::UpdateLidarClusters::Response> response);

  void onModifyTaskStatus(
    const std::shared_ptr<robosoft_interfaces::srv::ModifyTaskStatus::Request>
    request,
    std::shared_ptr<robosoft_interfaces::srv::ModifyTaskStatus::Response>
    response);
  void onRecordedRoute(
    const robosoft_interfaces::msg::GuidanceLine & route);

  // Task XML conversion. Filenames are resolved under task_directory.
  bool loadTaskFromFile(
    const std::string & filename,
    robosoft_interfaces::msg::TaskData & task);
  bool saveTaskToFile(
    const std::string & filename,
    const robosoft_interfaces::msg::TaskData & task);

  // Selected task, persistence policy and recording insertion point.
  robosoft_interfaces::msg::TaskData current_task_;
  std::string task_directory_;
  std::string current_task_filepath_;
  bool autosave_recorded_routes_{true};
  Task::ExtensionFormat extension_format_{Task::ExtensionFormat::XML_ELEMENTS};
  int32_t recording_guidance_index_{-1};
};

}  // namespace robosoft_core
