// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "task_manager_node.hpp"
#include "task_path.hpp"
#include "robosoft_interfaces/topics.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robosoft_core
{

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: Node("task_manager_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("task_directory",
                                       "/opt/robosoft/tasks");
  this->declare_parameter<bool>("autosave_recorded_routes", true);
  this->declare_parameter<std::string>("task_extension_format",
                                       "xml_elements");

  // Get parameters
  task_directory_ = this->get_parameter("task_directory").as_string();
  autosave_recorded_routes_ =
    this->get_parameter("autosave_recorded_routes").as_bool();
  const auto extension_format =
    this->get_parameter("task_extension_format").as_string();
  if (extension_format == "designator_json") {
    extension_format_ =
      Task::ExtensionFormat::DESIGNATOR_JSON;
  } else if (extension_format != "xml_elements") {
    RCLCPP_WARN(this->get_logger(),
                "Unknown task_extension_format '%s'; using xml_elements",
                extension_format.c_str());
  }

  // Create services
  select_task_service_ =
    this->create_service<robosoft_interfaces::srv::SelectTask>(
          robosoft_interfaces::kTaskSelectService,
          std::bind(&TaskManagerNode::onSelectTask, this,
                    std::placeholders::_1, std::placeholders::_2));

  save_task_service_ =
    this->create_service<robosoft_interfaces::srv::SaveTask>(
          robosoft_interfaces::kTaskSaveService,
          std::bind(&TaskManagerNode::onSaveTask, this,
          std::placeholders::_1, std::placeholders::_2));
  update_lidar_clusters_service_ =
    create_service<robosoft_interfaces::srv::UpdateLidarClusters>(
    robosoft_interfaces::kTaskUpdateLidarClustersService,
    std::bind(
      &TaskManagerNode::onUpdateLidarClusters, this,
      std::placeholders::_1, std::placeholders::_2));

  modify_status_service_ =
    this->create_service<robosoft_interfaces::srv::ModifyTaskStatus>(
          robosoft_interfaces::kTaskModifyStatusService,
          std::bind(&TaskManagerNode::onModifyTaskStatus, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Create publishers
  task_pub_ = this->create_publisher<robosoft_interfaces::msg::TaskData>(
      robosoft_interfaces::kTaskLoadedTopic,
      rclcpp::QoS(1).reliable().transient_local());
  recorded_route_sub_ =
    this->create_subscription<robosoft_interfaces::msg::GuidanceLine>(
          robosoft_interfaces::kRecordedRouteTopic, rclcpp::QoS(10),
          std::bind(&TaskManagerNode::onRecordedRoute, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TaskManagerNode initialized");
}

void TaskManagerNode::onUpdateLidarClusters(
  const std::shared_ptr<
    robosoft_interfaces::srv::UpdateLidarClusters::Request> request,
  std::shared_ptr<
    robosoft_interfaces::srv::UpdateLidarClusters::Response> response)
{
  if (current_task_.task_name.empty()) {
    response->success = false;
    response->error_message = "No task is loaded";
    return;
  }
  current_task_.lidar_clusters = request->lidar_clusters;
  task_pub_->publish(current_task_);
  response->success = true;
}

TaskManagerNode::~TaskManagerNode() {}

// ---------------------------------------------------------------------------
// ROS service and recorded-route callbacks
// ---------------------------------------------------------------------------

void TaskManagerNode::onSelectTask(
  const std::shared_ptr<robosoft_interfaces::srv::SelectTask::Request>
  request,
  std::shared_ptr<robosoft_interfaces::srv::SelectTask::Response>
  response)
{
  RCLCPP_INFO(this->get_logger(), "Received SelectTask request: %s",
              request->task_name.c_str());

  const std::string filepath =
    resolveTaskPath(task_directory_, request->task_name);

  if (loadTaskFromFile(filepath, current_task_)) {
    current_task_filepath_ = filepath;
    recording_guidance_index_ = -1;
    response->success = true;
    response->task_data = current_task_;
    task_pub_->publish(current_task_);
    RCLCPP_INFO(this->get_logger(), "Task loaded successfully");
  } else {
    response->success = false;
    response->error_message = "Failed to load task file";
    RCLCPP_ERROR(this->get_logger(), "Failed to load task: %s",
                 filepath.c_str());
  }
}

void TaskManagerNode::onRecordedRoute(
  const robosoft_interfaces::msg::GuidanceLine & route)
{
  // RouteControl publishes complete patterns. Each pattern is appended to the
  // current recording group and the first appended pattern remains active.
  if (current_task_.task_name.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Recorded route rejected: no task loaded");
    return;
  }
  if (route.points.size() < 2) {
    RCLCPP_ERROR(this->get_logger(),
                 "Recorded route rejected: less than two points");
    return;
  }

  auto stored_route = route;
  stored_route.is_active = true;
  for (auto & line : current_task_.guidance_lines) {
    line.is_active = false;
  }
  current_task_.guidance_lines.push_back(stored_route);
  const auto appended_index =
    static_cast<int32_t>(current_task_.guidance_lines.size()) - 1;
  if (recording_guidance_index_ < 0) {
    recording_guidance_index_ = appended_index;
  }
  current_task_.active_guidance_index = recording_guidance_index_;
  current_task_.recorded_point_count +=
    static_cast<int32_t>(route.points.size());
  task_pub_->publish(current_task_);

  if (autosave_recorded_routes_ && !current_task_filepath_.empty() &&
    !saveTaskToFile(current_task_filepath_, current_task_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to autosave recorded route");
  } else {
    RCLCPP_INFO(this->get_logger(), "Recorded route stored (%zu points)",
                route.points.size());
  }
}

void TaskManagerNode::onSaveTask(
  const std::shared_ptr<robosoft_interfaces::srv::SaveTask::Request>
  request,
  std::shared_ptr<robosoft_interfaces::srv::SaveTask::Response>
  response)
{
  RCLCPP_INFO(this->get_logger(), "Received SaveTask request: %s",
              request->task_name.c_str());

  const std::string filepath =
    resolveTaskPath(task_directory_, request->task_name);

  if (saveTaskToFile(filepath, request->task_data)) {
    current_task_ = request->task_data;
    current_task_filepath_ = filepath;
    task_pub_->publish(current_task_);
    response->success = true;
    response->file_path = filepath;
    RCLCPP_INFO(this->get_logger(), "Task saved successfully to %s",
                filepath.c_str());
  } else {
    response->success = false;
    response->error_message = "Failed to save task file";
    RCLCPP_ERROR(this->get_logger(), "Failed to save task to %s",
                 filepath.c_str());
  }
}

void TaskManagerNode::onModifyTaskStatus(
  const std::shared_ptr<robosoft_interfaces::srv::ModifyTaskStatus::Request>
  request,
  std::shared_ptr<robosoft_interfaces::srv::ModifyTaskStatus::Response>
  response)
{
  RCLCPP_INFO(this->get_logger(), "Received ModifyTaskStatus request");

  if (current_task_.task_name.empty()) {
    response->success = false;
    response->error_message = "No task is loaded";
    return;
  }
  if (request->new_status >
    robosoft_interfaces::msg::TaskData::STATUS_CANCELED)
  {
    response->success = false;
    response->error_message = "Invalid task status";
    return;
  }
  if (request->guidance_index < -1 ||
    request->guidance_index >=
    static_cast<int32_t>(current_task_.guidance_lines.size()))
  {
    response->success = false;
    response->error_message = "Guidance index is out of range";
    return;
  }
  if (request->guidance_index >= 0) {
    current_task_.active_guidance_index = request->guidance_index;
  }
  // TSK.G=RUNNING represents both manual recording and automatic execution.
  // A -1 guidance index identifies a new recording;
  // RobotState.substate disambiguates the operation for RouteControl.
  if (request->new_status ==
    robosoft_interfaces::msg::TaskData::STATUS_RUNNING &&
    request->guidance_index == -1 &&
    current_task_.status != robosoft_interfaces::msg::TaskData::STATUS_RUNNING)
  {
    recording_guidance_index_ =
      static_cast<int32_t>(current_task_.guidance_lines.size());
  }
  // TaskData is the authoritative shared snapshot. Publish every accepted
  // status mutation through the transient-local topic.
  current_task_.status = request->new_status;
  task_pub_->publish(current_task_);
  response->success = true;
  response->error_message.clear();
  RCLCPP_INFO(this->get_logger(), "Task '%s' status changed to %u",
              current_task_.task_name.c_str(), current_task_.status);
}

// ---------------------------------------------------------------------------
// Persistent Task XML conversion
// ---------------------------------------------------------------------------

bool TaskManagerNode::loadTaskFromFile(
  const std::string & filename, robosoft_interfaces::msg::TaskData & task)
{
  try {
    // Task owns ISO 11783 parsing and extension-format details.
    Task task_loader;
    task_loader.setExtensionFormat(extension_format_);

    if (!task_loader.loadFromFile(filename)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load task from file: %s",
                   filename.c_str());
      return false;
    }

    task = task_loader.toMessage();
    RCLCPP_INFO(this->get_logger(), "Successfully loaded task: %s from %s",
                task.task_name.c_str(), filename.c_str());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while loading task file %s: %s", filename.c_str(),
                 e.what());
    return false;
  }
}

bool TaskManagerNode::saveTaskToFile(
  const std::string & filename,
  const robosoft_interfaces::msg::TaskData & task)
{
  try {
    Task task_saver(task);
    task_saver.setExtensionFormat(extension_format_);

    if (!task_saver.saveToFile(filename)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save task to file: %s",
                   filename.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Successfully saved task: %s to %s",
                task.task_name.c_str(), filename.c_str());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while saving task file %s: %s", filename.c_str(),
                 e.what());
    return false;
  }
}

}  // namespace robosoft_core

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robosoft_core::TaskManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
