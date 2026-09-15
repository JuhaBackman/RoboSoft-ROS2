# robosoft_core

RoboSoft Core is a reusable ROS 2 framework for building mobile-robot control
systems. It provides modular building blocks for GNSS input, localization,
ISO 11783 TASK management, route control, path tracking, lidar safety, operator
interaction and robot-level state coordination. Applications select the
components they need and add hardware adapters and safety policies for their
own platform.

## Overview

- **GNSS**: serial transport, NMEA 0183 parsing, NTRIP corrections and local
  Cartesian conversion. Docs: [src/gnss/README.md](src/gnss/README.md)
- **GUI**: Qt 6/QML operator interface, task controls, route display and ROS
  topic monitor. Docs: [src/gui/README.md](src/gui/README.md)
- **Localization**: raw GNSS selection, lidar landmark association and the
  VIATOC-generated EKF adapter. Docs:
  [src/localization/README.md](src/localization/README.md)
- **Navigation**: TASK route recording and selection, path tracking and the
  optional Nav2 integration boundary. Docs:
  [src/navigation/README.md](src/navigation/README.md)
- **Safety**: vendor-independent lidar corridor and fail-safe speed limit.
  Docs: [src/safety/README.md](src/safety/README.md)
- **State machine**: reusable operating states, TaskManager synchronization,
  startup dependency discovery and robot-state publication. Docs:
  [src/state_machine/README.md](src/state_machine/README.md)
- **Task management**: ISO 11783-10 XML conversion, runtime task ownership and
  recorded-route persistence. Docs: [src/task/README.md](src/task/README.md)

Sources are grouped by these responsibilities. Each node and its directly
related implementation files share one lower-case directory below the group.

## Nodes

| Node | Group | Responsibility |
| --- | --- | --- |
| `serial_node` | GNSS | Bidirectional serial transport |
| `nmea0183_parser_node` | GNSS | Standard ROS measurements from NMEA 0183 |
| `ntrip_client_node` | GNSS | NTRIP v1/v2 RTCM corrections |
| `gps_to_cartesian_node` | GNSS | Shared map origin and local odometry |
| `robosoft_gui_node` | GUI | Optional operator interface |
| `lidar_object_detector_node` | Localization | Ordered lidar scan to vehicle-frame landmark detections |
| `landmark_clusterer_node` | Localization | Detection association with persistent TASK landmarks |
| `extended_kalman_filter_node` | Localization | GNSS, motion and associated-landmark state estimation |
| `route_control_node` | Navigation | Route recording, interpolation and segment selection |
| `path_tracking_simple_node` | Navigation | Lightweight curvature/error feedback controller |
| `path_tracking_nmpc_node` | Navigation | VIATOC-generated nonlinear predictive controller |
| `lidar_safety_node` | Safety | Point-cloud obstacle corridor and speed limit |
| `task_manager_node` | Task | Task selection, persistence and status |

`RobotMainNodeBase` is an exported library from which an application's main
coordination node can derive. It is not a separately launched node.

The public route and localization boundaries use standard ROS types where
possible: `nav_msgs/Path`, `nav_msgs/Odometry`,
`geometry_msgs/TwistStamped`, `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` and
TF2. ISO 11783 TASK semantics that do not fit those messages remain in
`robosoft_interfaces`.

## Building an application

A robot application can use only the core components relevant to its sensors,
actuators and operating model. A typical integration consists of:

1. Derive an application main node from `RobotMainNodeBase` and implement the
   platform's startup, operating-mode and safety transition guards.
2. Launch the required GNSS, localization, task, navigation, safety and GUI
   nodes with application-owned parameters.
3. Connect standard ROS measurements to the core interfaces directly or by
   launch remapping.
4. Implement hardware adapters that translate the guarded standard command
   topics to the vehicle and implement protocols.
5. Validate watchdogs, command limits, emergency stops and stopping behaviour
   on the target platform.

Core nodes do not require a particular vehicle bus, remote control, actuator
protocol or robot geometry. These choices remain in the consuming application.
The component READMEs document each ROS interface and its configuration
contract.

## Configuration

Example GNSS and GUI parameters are under `config/`. Applications should
provide their own launch files and parameter overlays for the selected core
nodes. Serial devices, NTRIP credentials, sensor transforms, vehicle limits
and safety dimensions must be configured for each deployment.

Development task files and the ISO 11783 TASKDATA example are under `tasks/`
and install to `share/robosoft_core/tasks`. Production systems should set
`task_directory` to a writable machine-specific location. See the
[task documentation](src/task/README.md).

## Build options

The Qt 6/QML GUI is built by default. Embedded targets without Qt can disable
it at configure time:

```bash
colcon build --packages-up-to robosoft_core \
  --cmake-args -DBUILD_ROBOSOFT_GUI=OFF
```

An application launch file normally starts the selected core nodes together
with its hardware adapters and main coordination node. The GUI can also be
launched separately:

```bash
ros2 launch robosoft_core gui.launch.py
```

## Build and test

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robosoft_core
colcon test --packages-select robosoft_core
colcon test-result --verbose
```

The test suite covers NMEA and NTRIP protocols, TASK XML and path handling,
path tracking, lidar safety, landmark detection and association, and the EKF
adapter.

## Safety

This package can contribute commands to mobile robots and machinery. It is
not a safety-certified controller. Emergency stops, command watchdogs,
actuator limits, sensor transforms and stopping distances must be designed and
validated for the target platform before use on physical equipment.

## License

GPL-3.0-only. See [LICENSE](LICENSE).
