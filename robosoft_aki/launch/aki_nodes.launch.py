#!/usr/bin/env python3
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

"""Start the RoboSoft AKI control stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def create_task_manager_node(context, config_file):
    """Create TaskManager, overriding its YAML directory only when requested."""
    parameters = [config_file]
    task_directory = LaunchConfiguration("task_directory").perform(context)
    if task_directory:
        parameters.append({"task_directory": task_directory})

    return [
        Node(
            package="robosoft_core",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    """Start ISOBUS, shared navigation and AKI-specific nodes."""
    config = (
        Path(get_package_share_directory("robosoft_aki"))
        / "config"
        / "aki_params.yaml"
    )
    can_interface = LaunchConfiguration("can_interface")
    disable_can_loopback = LaunchConfiguration("disable_can_loopback")
    tecu_server_name = LaunchConfiguration("tecu_server_name")
    tecu_server_address = LaunchConfiguration("tecu_server_address")
    serial_device = LaunchConfiguration("serial_device")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_uvc_sequence = LaunchConfiguration("enable_uvc_sequence")
    path_tracking_algorithm = LaunchConfiguration("path_tracking_algorithm")
    remote_watchdog_timeout_ms = LaunchConfiguration("remote_watchdog_timeout_ms")
    transform_defaults = {
        "gnss_x": "0.0",
        "gnss_y": "0.0",
        "gnss_z": "0.0",
        "gnss_roll": "0.0",
        "gnss_pitch": "0.0",
        "gnss_yaw": "0.0",
        "lidar_x": "1.094",
        "lidar_y": "0.0",
        "lidar_z": "0.0",
        "lidar_roll": "0.0",
        "lidar_pitch": "0.0",
        "lidar_yaw": "0.0",
    }
    transform_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=value,
            description=f"Static {name.replace('_', ' ')} transform value",
        )
        for name, value in transform_defaults.items()
    ]
    can_remappings = [
        ("ISOBUS/bus_rx_frames", "/ISOBUS/bus_rx_frames"),
        ("ISOBUS/bus_tx_frames", "/ISOBUS/bus_tx_frames"),
    ]

    return LaunchDescription(
        transform_arguments + [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by AKI",
            ),
            DeclareLaunchArgument(
                "disable_can_loopback",
                default_value="true",
                description="Disable local SocketCAN loopback on physical CAN",
            ),
            DeclareLaunchArgument(
                "tecu_server_name",
                default_value="A0001900AAA00006",
                description="ISO NAME of the steering/T-ECU command server",
            ),
            DeclareLaunchArgument(
                "tecu_server_address",
                default_value="19",
                description="Fallback source address of the steering/T-ECU server",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="/dev/ttymxc2",
                description="Bidirectional GNSS/NTRIP serial device",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="Start and require lidar safety functions",
            ),
            DeclareLaunchArgument(
                "enable_uvc_sequence",
                default_value="true",
                description=(
                    "Run treatment stop/advance sequences for implement state 1; "
                    "false forwards path tracking continuously"
                ),
            ),
            DeclareLaunchArgument(
                "path_tracking_algorithm",
                default_value="nmpc",
                choices=["simple", "nmpc"],
                description="Path-tracking controller implementation",
            ),
            DeclareLaunchArgument(
                "remote_watchdog_timeout_ms",
                default_value="300",
                description="Maximum interval between physical remote status frames",
            ),
            DeclareLaunchArgument(
                "task_directory",
                default_value="",
                description=(
                    "Optional TaskManager directory override; an empty value "
                    "uses the path from aki_params.yaml"
                ),
            ),
            Node(
                package="ros2_isobus",
                executable="can_bridge_node",
                name="can_bridge_node",
                output="screen",
                parameters=[
                    {"interface": can_interface},
                    {"disable_loopback": disable_can_loopback},
                ],
                remappings=can_remappings,
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
                executable="tecu_node",
                name="tecu_node",
                output="screen",
                arguments=["--class3"],
                parameters=[
                    str(config),
                    {
                        "esp_name_hex": tecu_server_name,
                        "esp_sa": tecu_server_address,
                    },
                ],
                remappings=can_remappings,
            ),
            Node(
                package="ros2_isobus",
                executable="nmea2000_node",
                name="nmea2000_node",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("ISOBUS/bus_rx_frames", "/ISOBUS/bus_rx_frames"),
                ],
            ),
            Node(
                package="ros2_isobus",
                executable="nmea2000_server",
                name="nmea2000_server",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("ISOBUS/bus_tx_frames", "/ISOBUS/bus_tx_frames"),
                    (
                        "ISOBUS/nmea2000/tx/imu",
                        "/gnss/imu",
                    ),
                    (
                        "ISOBUS/nmea2000/tx/velocity",
                        "/gnss/velocity",
                    ),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="serial_node",
                name="serial_node",
                output="screen",
                parameters=[str(config), {"device": serial_device}],
                remappings=[
                    ("serial/line", "/nmea0183/sentence"),
                    ("serial/write", "/ntrip/corrections"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="nmea0183_parser_node",
                name="nmea0183_parser_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="robosoft_core",
                executable="ntrip_client_node",
                name="ntrip_client_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="robosoft_core",
                executable="gps_to_cartesian_node",
                name="gps_to_cartesian_node",
                output="screen",
                parameters=[str(config), {"publish_tf": False}],
                remappings=[("odometry", "/odometry/raw")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gnss_static_transform",
                arguments=[
                    "--x", LaunchConfiguration("gnss_x"),
                    "--y", LaunchConfiguration("gnss_y"),
                    "--z", LaunchConfiguration("gnss_z"),
                    "--yaw", LaunchConfiguration("gnss_yaw"),
                    "--pitch", LaunchConfiguration("gnss_pitch"),
                    "--roll", LaunchConfiguration("gnss_roll"),
                    "--frame-id", "base_link",
                    "--child-frame-id", "gnss_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_static_transform",
                condition=IfCondition(enable_lidar),
                arguments=[
                    "--x", LaunchConfiguration("lidar_x"),
                    "--y", LaunchConfiguration("lidar_y"),
                    "--z", LaunchConfiguration("lidar_z"),
                    "--yaw", LaunchConfiguration("lidar_yaw"),
                    "--pitch", LaunchConfiguration("lidar_pitch"),
                    "--roll", LaunchConfiguration("lidar_roll"),
                    "--frame-id", "base_link",
                    "--child-frame-id", "lidar_link",
                ],
            ),
            OpaqueFunction(
                function=create_task_manager_node,
                args=[str(config)],
            ),
            Node(
                package="robosoft_core",
                executable="route_control_node",
                name="route_control_node",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("vehicle/twist_measured", "/ISOBUS/tecu/twist_measured"),
                ],
            ),
            Node(
                package="robosoft_aki",
                executable="remote_controller_node",
                name="remote_controller_node",
                output="screen",
                parameters=[
                    str(config),
                    {"watchdog_timeout_ms": remote_watchdog_timeout_ms},
                ],
                remappings=[
                    ("ISOBUS/bus_rx_frames", "/ISOBUS/bus_rx_frames"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="lidar_safety_node",
                name="lidar_safety_node",
                output="screen",
                parameters=[str(config)],
                condition=IfCondition(enable_lidar),
                remappings=[
                    ("cloud", "/cloud"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="lidar_object_detector_node",
                name="lidar_object_detector_node",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("cloud", "/cloud"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="landmark_clusterer_node",
                name="landmark_clusterer_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="robosoft_core",
                executable="extended_kalman_filter_node",
                name="extended_kalman_filter_node",
                output="screen",
                parameters=[str(config)],
                remappings=[
                    ("vehicle/twist_measured", "/ISOBUS/tecu/twist_measured"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="path_tracking_simple_node",
                name="path_tracking_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", path_tracking_algorithm, "' == 'simple'"])
                ),
                parameters=[str(config), {"enable_lidar": enable_lidar}],
                remappings=[
                    ("vehicle/twist_measured", "/ISOBUS/tecu/twist_measured"),
                ],
            ),
            Node(
                package="robosoft_core",
                executable="path_tracking_nmpc_node",
                name="path_tracking_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", path_tracking_algorithm, "' == 'nmpc'"])
                ),
                parameters=[str(config), {"enable_lidar": enable_lidar}],
                remappings=[
                    ("vehicle/twist_measured", "/ISOBUS/tecu/twist_measured"),
                ],
            ),
            Node(
                package="robosoft_aki",
                executable="aki_uvc_sequence_node",
                name="aki_uvc_sequence_node",
                output="screen",
                parameters=[
                    str(config),
                    {
                        "enable_lidar": enable_lidar,
                        "enable_uvc_sequence": enable_uvc_sequence,
                    },
                ],
                remappings=[
                    ("navigation/cmd_vel", "/ISOBUS/tecu/twist_command"),
                ],
            ),
            Node(
                package="robosoft_aki",
                executable="uvc_led_node",
                name="uvc_led_node",
                output="screen",
                parameters=[str(config)],
                remappings=can_remappings,
            ),
            Node(
                package="robosoft_aki",
                executable="aki_main_node",
                name="aki_main_node",
                output="screen",
                parameters=[str(config), {"enable_lidar": enable_lidar}],
            ),
        ]
    )
