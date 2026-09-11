// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <rclcpp/rclcpp.hpp>
#include <ros2_isobus/msg/aux_valve_status.hpp>
#include <ros2_isobus/msg/tecu_rear_hitch_status.hpp>
#include <ros2_isobus/msg/tecu_rear_pto_status.hpp>
#include <topics.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>

namespace robosoft_simulator
{

/** Publishes the physical tractor measurements consumed by the T-ECU server. */
class TractorStateSimulatorNode : public rclcpp::Node
{
public:
  TractorStateSimulatorNode()
  : Node("tractor_state_simulator_node")
  {
    const double publish_rate_hz = std::max(
      1.0, declare_parameter<double>("publish_rate_hz", 10.0));
    engine_speed_rpm_ = declare_parameter<double>("engine_speed_rpm", 1500.0);
    maximum_power_time_min_ = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      declare_parameter<int>("maximum_power_time_min", 250), 0, 250));
    key_switch_active_ = declare_parameter<bool>("key_switch_active", true);
    guidance_ready_ = declare_parameter<bool>("guidance_ready", true);
    mechanical_lockout_ = declare_parameter<bool>("mechanical_lockout", false);
    hitch_position_percent_ = std::clamp(
      declare_parameter<double>("rear_hitch_position_percent", 0.0), 0.0, 100.0);
    hitch_in_work_ = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      declare_parameter<int>("rear_hitch_in_work", 0), 0, 3));
    pto_rpm_ = std::max(0.0, declare_parameter<double>("rear_pto_rpm", 0.0));
    pto_engaged_ = declare_parameter<bool>("rear_pto_engaged", false);
    aux_valve_count_ = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      declare_parameter<int>("aux_valve_count", 8), 0, 16));

    engine_speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      ros2_isobus::kTECUServerEngineSpeedTopic, 10);
    maximum_power_time_pub_ = create_publisher<std_msgs::msg::UInt8>(
      ros2_isobus::kTECUServerMaximumPowerTimeTopic, 10);
    key_switch_pub_ = create_publisher<std_msgs::msg::Bool>(
      ros2_isobus::kTECUServerKeySwitchTopic, 10);
    guidance_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      ros2_isobus::kTECUServerGuidanceReadyTopic, 10);
    mechanical_lockout_pub_ = create_publisher<std_msgs::msg::Bool>(
      ros2_isobus::kTECUServerMechanicalLockoutTopic, 10);
    hitch_pub_ = create_publisher<ros2_isobus::msg::TecuRearHitchStatus>(
      ros2_isobus::kTECUServerRearHitchStatusTopic, 10);
    pto_pub_ = create_publisher<ros2_isobus::msg::TecuRearPtoStatus>(
      ros2_isobus::kTECUServerRearPtoStatusTopic, 10);
    valve_pub_ = create_publisher<ros2_isobus::msg::AuxValveStatus>(
      ros2_isobus::kTECUServerAuxValveStatusTopic, 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(period, std::bind(&TractorStateSimulatorNode::publish, this));
    RCLCPP_INFO(
      get_logger(), "Tractor state simulator ready at %.1f Hz with %u AUX valves",
      publish_rate_hz, aux_valve_count_);
  }

private:
  void publish()
  {
    std_msgs::msg::Float64 engine_speed;
    engine_speed.data = engine_speed_rpm_;
    engine_speed_pub_->publish(engine_speed);

    std_msgs::msg::UInt8 maximum_power_time;
    maximum_power_time.data = maximum_power_time_min_;
    maximum_power_time_pub_->publish(maximum_power_time);

    std_msgs::msg::Bool state;
    state.data = key_switch_active_;
    key_switch_pub_->publish(state);
    state.data = guidance_ready_;
    guidance_ready_pub_->publish(state);
    state.data = mechanical_lockout_;
    mechanical_lockout_pub_->publish(state);

    ros2_isobus::msg::TecuRearHitchStatus hitch;
    hitch.position_percent = hitch_position_percent_;
    hitch.in_work = hitch_in_work_;
    hitch.position_limit_status = 0U;
    hitch.nominal_lower_link_force = 0.0;
    hitch.draft_n = 0.0;
    hitch.exit_code = 0U;
    hitch_pub_->publish(hitch);

    ros2_isobus::msg::TecuRearPtoStatus pto;
    pto.rpm = pto_rpm_;
    pto.setpoint_rpm = pto_rpm_;
    pto.engagement = pto_engaged_ ? 1U : 0U;
    pto.mode = 0U;
    pto.economy_mode = 0U;
    pto.engagement_request = 0U;
    pto.mode_request = 0U;
    pto.economy_request = 0U;
    pto.speed_limit_status = 0U;
    pto_pub_->publish(pto);

    for (std::uint8_t valve_number = 0U; valve_number < aux_valve_count_; ++valve_number) {
      ros2_isobus::msg::AuxValveStatus valve;
      valve.valve_number = valve_number;
      valve.extend_flow_percent = 0.0F;
      valve.retract_flow_percent = 0.0F;
      valve.state = 0U;
      valve.failsafe = false;
      valve_pub_->publish(valve);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr engine_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr maximum_power_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr key_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr guidance_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mechanical_lockout_pub_;
  rclcpp::Publisher<ros2_isobus::msg::TecuRearHitchStatus>::SharedPtr hitch_pub_;
  rclcpp::Publisher<ros2_isobus::msg::TecuRearPtoStatus>::SharedPtr pto_pub_;
  rclcpp::Publisher<ros2_isobus::msg::AuxValveStatus>::SharedPtr valve_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double engine_speed_rpm_{1500.0};
  double hitch_position_percent_{0.0};
  double pto_rpm_{0.0};
  std::uint8_t maximum_power_time_min_{250U};
  std::uint8_t hitch_in_work_{0U};
  std::uint8_t aux_valve_count_{8U};
  bool key_switch_active_{true};
  bool guidance_ready_{true};
  bool mechanical_lockout_{false};
  bool pto_engaged_{false};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_simulator::TractorStateSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
