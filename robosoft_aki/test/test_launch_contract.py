# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only

"""Check AKI launch wiring and ownership of the active route topic."""

from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]


def test_route_control_is_the_only_application_route_publisher():
    """Machine state nodes must not compete with RouteControl's latched route."""
    route_control = (
        SOURCE_ROOT
        / "robosoft_core/src/navigation/route_control/route_control_node.cpp"
    ).read_text(encoding="utf-8")
    aki_main = (
        SOURCE_ROOT / "robosoft_aki/src/aki_main/aki_main_node.cpp"
    ).read_text(encoding="utf-8")
    assert "kCurrentRouteTopic" in route_control
    assert "kCurrentRouteTopic" not in aki_main


def test_launch_connects_generic_gnss_and_configurable_sensor_frames():
    """Keep generic GNSS topics and expose every static sensor transform."""
    launch = (
        SOURCE_ROOT / "robosoft_aki/launch/aki_nodes.launch.py"
    ).read_text(encoding="utf-8")

    assert '"/gnss/imu"' in launch
    assert '"/gnss/velocity"' in launch
    assert 'LaunchConfiguration("serial_device")' in launch
    assert '{"device": serial_device}' in launch
    assert 'LaunchConfiguration("enable_lidar")' in launch
    assert 'condition=IfCondition(enable_lidar)' in launch
    assert '"enable_lidar": enable_lidar' in launch
    assert 'LaunchConfiguration("enable_uvc_sequence")' in launch
    assert '"enable_uvc_sequence": enable_uvc_sequence' in launch
    for sensor in ("gnss", "lidar"):
        for component in ("x", "y", "z", "roll", "pitch", "yaw"):
            assert f'LaunchConfiguration("{sensor}_{component}")' in launch


def test_path_tracking_has_one_final_tecu_command_publisher():
    """Core tracks the path while the AKI UVC coordinator owns final output."""
    launch = (
        SOURCE_ROOT / "robosoft_aki/launch/aki_nodes.launch.py"
    ).read_text(encoding="utf-8")
    core_tracker = (
        SOURCE_ROOT
        / "robosoft_core/src/navigation/path_tracking_simple/path_tracking_simple_node.cpp"
    ).read_text(encoding="utf-8")

    assert 'executable="path_tracking_simple_node"' in launch
    assert 'executable="path_tracking_nmpc_node"' in launch
    assert 'name="path_tracking_node"' in launch
    assert 'executable="aki_uvc_sequence_node"' in launch
    assert launch.count('"/ISOBUS/tecu/twist_command"') == 1
    assert "kPathTrackingCommandTopic" in core_tracker
    assert "ros2_isobus" not in core_tracker


def test_localization_nodes_use_generic_core_inputs():
    """Keep remote and ISOBUS adaptation outside shared localization."""
    launch = (
        SOURCE_ROOT / "robosoft_aki/launch/aki_nodes.launch.py"
    ).read_text(encoding="utf-8")
    detector = (
        SOURCE_ROOT
        / "robosoft_core/src/localization/object_detection/lidar_object_detector_node.cpp"
    ).read_text(encoding="utf-8")
    clusterer = (
        SOURCE_ROOT
        / "robosoft_core/src/localization/clustering/landmark_clusterer_node.cpp"
    ).read_text(encoding="utf-8")
    estimator = (
        SOURCE_ROOT
        / "robosoft_core/src/localization/extended_kalman_filter/extended_kalman_filter_node.cpp"
    ).read_text(encoding="utf-8")
    aki_main = (
        SOURCE_ROOT / "robosoft_aki/src/aki_main/aki_main_node.cpp"
    ).read_text(encoding="utf-8")

    assert 'package="robosoft_core"' in launch
    assert 'executable="lidar_object_detector_node"' in launch
    assert 'executable="landmark_clusterer_node"' in launch
    assert 'executable="extended_kalman_filter_node"' in launch
    assert '"vehicle/twist_measured", "/ISOBUS/tecu/twist_measured"' in launch
    assert "kLocalizationModeTopic" in estimator
    assert "kMeasuredTwistTopic" in estimator
    for source in (detector, clusterer, estimator):
        assert "ros2_isobus" not in source
        assert "RemoteControlStatus" not in source
    assert "kLocalizationModeTopic" in aki_main
