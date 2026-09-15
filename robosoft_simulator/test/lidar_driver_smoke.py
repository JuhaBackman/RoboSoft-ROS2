#!/usr/bin/env python3
# Copyright 2026 Juha Backman / Natural Resources Institute Finland
# SPDX-License-Identifier: GPL-3.0-only
"""Integration test with the installed, unmodified SICK TiM5xx driver.

Run after sourcing the workspace. Uses a separate domain and TCP port, without
CAN, joystick or serial hardware. Validates scan geometry and reconnects.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    rclpy.init()
    node = rclpy.create_node("lidar_smoke_probe")
    publisher = node.create_publisher(Odometry, "vehicle/odometry", 10)
    scans = []
    subscription = node.create_subscription(LaserScan, "/scan", scans.append, 10)
    processes = []
    with tempfile.TemporaryDirectory(prefix="robosoft-lidar-test-") as directory:
        root = Path(directory)
        longitude = 25 + 5 / (6371000 * math.cos(math.radians(60))) * 180 / math.pi
        task = root / "map.XML"
        task.write_text(
            '<ISO11783_TaskData><PFD><PNT A="5" C="60" D="' + str(longitude) +
            '"/><PNT A="1" C="60" D="25"/></PFD></ISO11783_TaskData>'
        )
        simulator = str(Path(get_package_prefix("robosoft_simulator")) /
                        "lib/robosoft_simulator/sick_tim5xx_simulator_node")
        generator = str(Path(get_package_prefix("robosoft_simulator")) /
                        "lib/robosoft_simulator/task_lidar_cloud_generator_node")
        driver = str(Path(get_package_prefix("sick_scan_xd")) /
                     "lib/sick_scan_xd/sick_generic_caller")
        launch = str(Path(get_package_share_directory("sick_scan_xd")) /
                     "launch/sick_tim_5xx.launch")
        logs = []

        def start(command, name):
            log = (root / name).open("w+")
            logs.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            processes.append(process)
            return process

        def stop(process):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

        def receive(x, expected):
            scans.clear()
            matching = []
            first_stamp = node.get_clock().now().nanoseconds
            until = time.monotonic() + 30
            while time.monotonic() < until:
                odometry = Odometry()
                odometry.header.stamp = node.get_clock().now().to_msg()
                odometry.pose.pose.orientation.w = 1.0
                odometry.pose.pose.position.x = float(x)
                publisher.publish(odometry)
                rclpy.spin_once(node, timeout_sec=0.04)
                if scans:
                    scan = scans[-1]
                    scans.clear()
                    stamp = scan.header.stamp.sec * 1000000000 + scan.header.stamp.nanosec
                    if stamp < first_stamp:
                        continue
                    centre = round(-scan.angle_min / scan.angle_increment)
                    if abs(scan.ranges[centre] - expected) < 0.01:
                        assert len(scan.ranges) == 811, len(scan.ranges)
                        assert abs(scan.angle_min - math.radians(-135)) < 0.001
                        assert abs(scan.scan_time - 1 / 15) < 0.001
                        assert len(scan.intensities) == 811
                        assert scan.intensities[centre] > 0
                        matching.append(stamp)
                        if len(matching) < 20:
                            continue
                        rate = (len(matching) - 1) * 1e9 / (matching[-1] - matching[0])
                        assert 13 < rate < 17, rate
                        print(f"driver scan: {len(scan.ranges)} rays, centre={scan.ranges[centre]:.3f} m")
                        return
                assert all(p.poll() is None for p in processes if p is not old_driver)
            raise AssertionError("No correct SICK scan received within 30 seconds")

        old_driver = None
        passed = False
        try:
            start([generator, "--ros-args", "-p", f"task_file:={task}",
                   "-p", "reference_latitude:=60.0", "-p", "reference_longitude:=25.0",
                   "-p", "lidar_x_m:=0.0", "-p", "output_topic:=simulated_cloud"],
                  "generator.log")
            start([simulator, "--ros-args", "-p", "cloud_topic:=simulated_cloud",
                   "-p", "port:=22112",
                   "--log-level", "sick_tim5xx_simulator_node:=debug"], "simulator.log")
            time.sleep(0.5)
            command = [driver, launch, "hostname:=127.0.0.1", "port:=22112",
                       "frame_id:=lidar_link", "tf_publish_rate:=0.0"]
            old_driver = start(command, "driver.log")
            receive(0, 4.965)
            receive(1, 3.965)
            stop(old_driver)
            start(command, "driver_reconnected.log")
            receive(0, 4.965)
            print("PASS: real SICK driver startup, geometry, movement and reconnect")
            passed = True
        finally:
            for process in reversed(processes):
                stop(process)
            for log in logs:
                log.flush()
                log.seek(0)
                content = log.read()
                if not passed or any(p.returncode not in (0, -signal.SIGINT) for p in processes):
                    print(f"--- {Path(log.name).name} ---\n{content[-12000:]}")
                log.close()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
