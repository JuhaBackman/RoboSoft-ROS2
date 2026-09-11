// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include <robosoft_interfaces/msg/robot_state.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/srv/modify_task_status.hpp>

namespace robosoft_core
{

/**
 * Shared foundation for RoboSoft robot-level state machines.
 *
 * The class owns the state identifiers, TaskManager synchronization, required
 * node discovery and RobotState publication that are identical for every
 * robot. Derived nodes retain their hardware-specific subscriptions, safety
 * guards and state-entry actions.
 */
class RobotMainNodeBase : public rclcpp::Node
{
public:
  /// Common top-level and operational states for RoboSoft applications.
  enum class State : uint8_t { INIT = 0, MAIN = 1, EXIT = 2 };
  enum class SubState : uint8_t
  {
    INIT_WAIT_NODES = 0,
    INIT_WAIT_DATA = 1,
    MAIN_WAIT = 8,
    MAIN_MANUAL = 9,
    MAIN_MANUAL_RECORD = 10,
    MAIN_AUTO = 11,
    MAIN_STOP = 12
  };

protected:
  RobotMainNodeBase(
    const std::string & node_name, const std::string & robot_name,
    const rclcpp::NodeOptions & options);

  /// Change state, update the public state fields and invoke robot actions.
  void transition(State state, SubState substate);

  /// Publish the current state with a fresh timestamp.
  void publishState();

  /// Request an ISO task-state change from TaskManager.
  void requestTaskStatus(uint8_t status, int32_t guidance_index = -2);

  /// Advance INIT_WAIT_NODES once every configured ROS node is visible.
  void checkRequiredNodes(const std::vector<std::string> & required_nodes);

  bool taskLoaded() const;
  bool routeAvailable() const;
  bool taskStatusServiceReady() const;
  static const char * substateName(SubState substate);

  /// Derived classes perform implement commands and presentation updates here.
  virtual void onEnterState(SubState substate) = 0;
  virtual void onExitState(SubState substate);

  static constexpr uint8_t TASK_NOT_SET =
    robosoft_interfaces::msg::TaskData::STATUS_NOT_SET;
  static constexpr uint8_t TASK_PLANNED =
    robosoft_interfaces::msg::TaskData::STATUS_PLANNED;
  static constexpr uint8_t TASK_RUNNING =
    robosoft_interfaces::msg::TaskData::STATUS_RUNNING;
  static constexpr uint8_t TASK_PAUSED =
    robosoft_interfaces::msg::TaskData::STATUS_PAUSED;
  static constexpr uint8_t TASK_COMPLETED =
    robosoft_interfaces::msg::TaskData::STATUS_COMPLETED;
  static constexpr uint8_t TASK_TEMPLATE =
    robosoft_interfaces::msg::TaskData::STATUS_TEMPLATE;
  static constexpr uint8_t TASK_CANCELED =
    robosoft_interfaces::msg::TaskData::STATUS_CANCELED;

  State state_{State::INIT};
  SubState substate_{SubState::INIT_WAIT_NODES};
  robosoft_interfaces::msg::RobotState robot_state_;
  robosoft_interfaces::msg::TaskData current_task_;
  uint8_t task_status_{TASK_NOT_SET};
  int pending_task_status_{-1};
  bool initialization_complete_{false};
  std::vector<std::string> previously_missing_nodes_;

private:
  void onTask(const robosoft_interfaces::msg::TaskData & msg);

  rclcpp::Subscription<robosoft_interfaces::msg::TaskData>::SharedPtr task_sub_;
  rclcpp::Publisher<robosoft_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Client<robosoft_interfaces::srv::ModifyTaskStatus>::SharedPtr
    task_status_client_;
};

}  // namespace robosoft_core
