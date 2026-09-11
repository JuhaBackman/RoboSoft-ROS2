#!/usr/bin/env python3
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

"""Start the local RoboSoft GUI with its machine-specific configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_gui_node(context, config):
    """Create the GUI, overriding its YAML task directory when requested."""
    parameters = [config]
    task_directory = LaunchConfiguration("task_directory").perform(context)
    if task_directory:
        parameters.append({"task_directory": task_directory})

    return [
        Node(
            package="robosoft_core",
            executable="robosoft_gui_node",
            name="robosoft_gui_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("robosoft_core"))
        / "config"
        / "gui_params.yaml"
    )
    config = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Machine-specific RoboSoft GUI parameter file",
            ),
            DeclareLaunchArgument(
                "task_directory",
                default_value="",
                description=(
                    "Optional GUI TASK-list directory override; an empty "
                    "value uses the path from the parameter file"
                ),
            ),
            OpaqueFunction(
                function=create_gui_node,
                args=[config],
            ),
        ]
    )
