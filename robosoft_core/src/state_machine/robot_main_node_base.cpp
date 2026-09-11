// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_core/robot_main_node_base.hpp"

#include "robosoft_interfaces/topics.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <utility>

namespace robosoft_core
{

RobotMainNodeBase::RobotMainNodeBase(
  const std::string & node_name, const std::string & robot_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  task_sub_ = create_subscription<robosoft_interfaces::msg::TaskData>(
    robosoft_interfaces::kTaskLoadedTopic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&RobotMainNodeBase::onTask, this, std::placeholders::_1));
  state_pub_ = create_publisher<robosoft_interfaces::msg::RobotState>(
    robosoft_interfaces::kRobotStateTopic, 10);
  task_status_client_ =
    create_client<robosoft_interfaces::srv::ModifyTaskStatus>(
    robosoft_interfaces::kTaskModifyStatusService);

  robot_state_.robot_name = robot_name;
  robot_state_.state_machine = static_cast<uint8_t>(state_);
  robot_state_.substate = static_cast<uint8_t>(substate_);
  robot_state_.error_message = "Initializing";
}

void RobotMainNodeBase::onTask(
  const robosoft_interfaces::msg::TaskData & msg)
{
  current_task_ = msg;
  task_status_ = msg.status;
  if (pending_task_status_ == task_status_) {
    pending_task_status_ = -1;
  }
}

void RobotMainNodeBase::transition(State state, SubState substate)
{
  if (state == state_ && substate == substate_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "State: %s -> %s", substateName(substate_),
    substateName(substate));
  onExitState(substate_);
  state_ = state;
  substate_ = substate;
  robot_state_.state_machine = static_cast<uint8_t>(state_);
  robot_state_.substate = static_cast<uint8_t>(substate_);
  onEnterState(substate_);
}

void RobotMainNodeBase::publishState()
{
  robot_state_.header.stamp = now();
  robot_state_.is_connected = initialization_complete_;
  state_pub_->publish(robot_state_);
}

void RobotMainNodeBase::requestTaskStatus(
  uint8_t status, int32_t guidance_index)
{
  if (!taskLoaded() || task_status_ == status ||
    pending_task_status_ == status)
  {
    return;
  }
  if (!task_status_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "TaskManager status service is unavailable");
    return;
  }

  pending_task_status_ = status;
  auto request =
    std::make_shared<robosoft_interfaces::srv::ModifyTaskStatus::Request>();
  request->new_status = status;
  if (guidance_index == -2) {
    const auto active_index = current_task_.active_guidance_index;
    request->guidance_index =
      active_index >= 0 &&
      active_index < static_cast<int32_t>(current_task_.guidance_lines.size()) ?
      active_index : -1;
  } else {
    request->guidance_index = guidance_index;
  }

  task_status_client_->async_send_request(
    request,
    [this, status](
      rclcpp::Client<robosoft_interfaces::srv::ModifyTaskStatus>::SharedFuture
      future)
    {
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(
          get_logger(), "Task status %u rejected: %s", status,
          response->error_message.c_str());
      }
      if (pending_task_status_ == status) {
        pending_task_status_ = -1;
      }
    });
}

void RobotMainNodeBase::checkRequiredNodes(
  const std::vector<std::string> & required_nodes)
{
  const auto visible_nodes = get_node_names();
  std::vector<std::string> missing;
  for (const auto & required : required_nodes) {
    const auto name =
      required.empty() || required.front() == '/' ? required : "/" + required;
    if (std::find(visible_nodes.begin(), visible_nodes.end(), name) ==
      visible_nodes.end())
    {
      missing.push_back(name);
    }
  }

  if (missing.empty()) {
    previously_missing_nodes_.clear();
    transition(State::INIT, SubState::INIT_WAIT_DATA);
    return;
  }

  std::string names;
  for (const auto & name : missing) {
    names += (names.empty() ? "" : ", ") + name;
  }
  robot_state_.is_connected = false;
  robot_state_.error_message = "nodes: " + names;
  if (missing != previously_missing_nodes_) {
    RCLCPP_INFO(get_logger(), "INIT waiting for nodes: %s", names.c_str());
    previously_missing_nodes_ = std::move(missing);
  }
}

bool RobotMainNodeBase::taskLoaded() const
{
  return !current_task_.task_name.empty();
}

bool RobotMainNodeBase::routeAvailable() const
{
  return taskLoaded() && !current_task_.guidance_lines.empty();
}

bool RobotMainNodeBase::taskStatusServiceReady() const
{
  return task_status_client_->service_is_ready();
}

void RobotMainNodeBase::onExitState(SubState)
{
}

const char * RobotMainNodeBase::substateName(SubState substate)
{
  switch (substate) {
    case SubState::INIT_WAIT_NODES: return "INIT_WAIT_NODES";
    case SubState::INIT_WAIT_DATA: return "INIT_WAIT_DATA";
    case SubState::MAIN_WAIT: return "MAIN_WAIT";
    case SubState::MAIN_MANUAL: return "MAIN_MANUAL";
    case SubState::MAIN_MANUAL_RECORD: return "MAIN_MANUAL_RECORD";
    case SubState::MAIN_AUTO: return "MAIN_AUTO";
    case SubState::MAIN_STOP: return "MAIN_STOP";
  }
  return "UNKNOWN";
}

}  // namespace robosoft_core
