#!/usr/bin/env python3
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

"""Start the external AKI CAN, serial and Ethernet simulators and vehicle model."""

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run the configurable ROS2ISOBUS T-ECU server and AKI hardware simulators."""
    config = (
        Path(get_package_share_directory("robosoft_simulator"))
        / "config"
        / "aki_can_simulator.yaml"
    )
    interface = LaunchConfiguration("can_interface")
    disable_can_loopback = LaunchConfiguration("disable_can_loopback")
    domain_id = LaunchConfiguration("domain_id")
    enable_remote = LaunchConfiguration("enable_remote")
    enable_joystick = LaunchConfiguration("enable_joystick")
    joystick_device_id = LaunchConfiguration("joystick_device_id")
    enable_uvc_led = LaunchConfiguration("enable_uvc_led")
    gnss_serial_device = LaunchConfiguration("gnss_serial_device")
    command_delay_s = LaunchConfiguration("command_delay_s")
    measurement_delay_s = LaunchConfiguration("measurement_delay_s")
    # Both sensor simulators must project TASK coordinates against one origin.
    with config.open(encoding="utf-8") as config_stream:
        gnss = yaml.safe_load(config_stream)["gnss_serial_simulator_node"]["ros__parameters"]
    lidar_origin = {key: gnss[key] for key in ("reference_latitude", "reference_longitude")}
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_lidar_simulator", default_value="true",
                description="Generate TASK lidar clouds and serve them over TiM5xx TCP",
            ),
            DeclareLaunchArgument(
                "domain_id",
                default_value="42",
                description=(
                    "ROS 2 domain used only inside the simulator computer; "
                    "robot communication uses CAN, serial and lidar TCP"
                ),
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface connected to the robot system",
            ),
            DeclareLaunchArgument(
                "disable_can_loopback",
                default_value="true",
                description="Disable local SocketCAN loopback on physical CAN",
            ),
            DeclareLaunchArgument(
                "enable_remote",
                default_value="true",
                description="Start the AKI physical remote-control simulator",
            ),
            DeclareLaunchArgument(
                "enable_joystick",
                default_value="false",
                description="Read a game controller with the ROS 2 joy node",
            ),
            DeclareLaunchArgument(
                "joystick_device_id",
                default_value="0",
                description="Linux joystick index used by joy_node",
            ),
            DeclareLaunchArgument(
                "enable_uvc_led",
                default_value="true",
                description="Start the AKI UVC/LED controller simulator",
            ),
            DeclareLaunchArgument(
                "gnss_serial_device",
                default_value="/tmp/aki_gnss_sim",
                description="Simulator side of the virtual GNSS serial pair",
            ),
            DeclareLaunchArgument(
                "command_delay_s",
                default_value="0.1",
                description="Delay from a drive command to the vehicle model",
            ),
            DeclareLaunchArgument(
                "measurement_delay_s",
                default_value="0.1",
                description="Delay from vehicle motion to the reported GNSS measurement",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            Node(
                package="robosoft_simulator",
                executable="sick_tim5xx_simulator_node",
                name="sick_tim5xx_simulator_node",
                output="screen",
                parameters=[str(config)],
                condition=IfCondition(LaunchConfiguration("enable_lidar_simulator")),
            ),
            Node(
                package="robosoft_simulator",
                executable="task_lidar_cloud_generator_node",
                name="task_lidar_cloud_generator_node",
                output="screen",
                parameters=[str(config), lidar_origin],
                condition=IfCondition(LaunchConfiguration("enable_lidar_simulator")),
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": joystick_device_id,
                        "deadzone": 0.08,
                        "autorepeat_rate": 20.0,
                    },
                ],
                condition=IfCondition(enable_joystick),
            ),
            Node(
                package="robosoft_simulator",
                executable="vehicle_model_node",
                name="vehicle_model_node",
                output="screen",
                parameters=[str(config), {"command_delay_s": command_delay_s}],
            ),
            Node(
                package="robosoft_simulator",
                executable="tractor_state_simulator_node",
                name="tractor_state_simulator_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="robosoft_simulator",
                executable="gnss_serial_simulator_node",
                name="gnss_serial_simulator_node",
                output="screen",
                parameters=[
                    str(config),
                    {
                        "device": gnss_serial_device,
                        "measurement_delay_s": measurement_delay_s,
                    },
                ],
            ),
            Node(
                package="ros2_isobus",
                executable="can_bridge_node",
                name="can_bridge_node",
                output="screen",
                parameters=[
                    str(config),
                    {
                        "interface": interface,
                        "disable_loopback": disable_can_loopback,
                    },
                ],
            ),
            Node(
                package="ros2_isobus",
                executable="address_manager_node",
                name="address_manager_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="ros2_isobus",
                executable="tecu_server_node",
                name="tecu_server_node",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("ISOBUS/tecu/server/inputs/twist_measured", "vehicle/twist"),
                    ("ISOBUS/tecu/server/commands/twist", "vehicle/command"),
                ],
            ),
            Node(
                package="robosoft_simulator",
                executable="remote_simulator_node",
                name="remote_simulator_node",
                output="screen",
                parameters=[
                    str(config),
                    {"use_joy": enable_joystick},
                ],
                condition=IfCondition(enable_remote),
            ),
            Node(
                package="robosoft_simulator",
                executable="aki_drive_controller_simulator_node",
                name="aki_drive_controller_simulator_node",
                output="screen",
                parameters=[str(config)],
                condition=IfCondition(enable_joystick),
            ),
            Node(
                package="robosoft_simulator",
                executable="uvc_led_simulator_node",
                name="uvc_led_simulator_node",
                output="screen",
                parameters=[str(config)],
                condition=IfCondition(enable_uvc_led),
            ),
        ]
    )
