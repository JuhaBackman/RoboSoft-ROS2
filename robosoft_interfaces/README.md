# robosoft_interfaces

ROS 2 interface definitions for applications built with RoboSoft. This package
contains messages, services and canonical topic names only; it has no robot,
hardware or control-algorithm implementation.

Standard ROS interfaces are used whenever they represent the data without
losing meaning. Examples include `nav_msgs/Odometry`, `nav_msgs/Path`,
`geometry_msgs/TwistStamped`, `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` and
`sensor_msgs/PointCloud2`. The custom interfaces below cover RoboSoft operating
state, ISO 11783 TASK semantics, route metadata, implement coordination and
safety information that standard messages do not express.

## Interface groups

| Group | Messages and services | Purpose |
| --- | --- | --- |
| Robot coordination | `RobotCommand`, `RobotState`, `RemoteControlStatus` | Operator commands, operating state and external controls |
| TASK and routes | `TaskData`, `GuidanceLine`, `RoutePoint`, `RouteStatus` | ISO TASK state and route metadata beyond `nav_msgs/Path` |
| Task services | `SelectTask`, `SaveTask`, `ModifyTaskStatus` | Runtime task ownership and persistence |
| Implements | `ImplementCommand`, `ImplementStatus` | Logical implement modes and feedback |
| Localization | `LocalizationMode`, `StateEstimatorStatus`, `LidarCluster`, `LidarClusterObservation`, `LidarClusterObservationArray`, `UpdateLidarClusters` | Estimator selection, quality and TASK landmarks |
| Safety | `LidarSafetyStatus` | Obstacle-derived speed limit and health |
| Runtime performance | `TimerPerformance` | Timer period, callback duration, remaining margin, lateness and cumulative deadline misses |

Canonical topic and service names for C++ consumers are declared in
[`include/robosoft_interfaces/topics.hpp`](include/robosoft_interfaces/topics.hpp).
Applications should use these constants or equivalent launch remappings rather
than duplicate names in source code.

## Design rules

- Prefer standard ROS messages at public sensor, pose, path and velocity
  boundaries.
- Keep custom messages independent of a particular CAN implementation or
  physical robot wherever possible.
- Use constants declared inside messages instead of undocumented numeric state
  values.
- Add fields compatibly when possible. A breaking interface change requires
  rebuilding and coordinating every dependent package.
- Keep durable ISO 11783 TASK information in `TaskData`; do not replace it with
  a controller-specific path representation.

## Build and inspection

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robosoft_interfaces
source install/setup.bash

ros2 interface show robosoft_interfaces/msg/RobotState
ros2 interface show robosoft_interfaces/msg/TaskData
ros2 interface show robosoft_interfaces/srv/SelectTask
```

## License

GPL-3.0-only. See [LICENSE](LICENSE).
