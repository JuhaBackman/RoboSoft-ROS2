# Navigation

TASK-aware route recording, route selection and generic path tracking.

## Components

| Node or library | Responsibility |
| --- | --- |
| `route_control_node` | Records TASK guidance patterns, selects segments and calculates tracking references |
| [`path_tracking_simple_node`](path_tracking_simple/README.md) | Lightweight curvature feed-forward and P-type error feedback |
| [`path_tracking_nmpc_node`](path_tracking_nmpc/README.md) | Nonlinear predictive tracking with a VIATOC-generated solver |
| `path_tracking_simple_controller.hpp` | Reusable simple control law exported by `robosoft_core` |
| `path_tracking_nmpc_controller.hpp` | Runtime configuration wrapper for the generated NMPC solver |

RouteControl owns TASK-specific behaviour: forward/reverse patterns, point
speeds, implement modes and transitions between guidance lines. PathTracking
is robot independent and does not publish directly to a hardware interface.
A robot-specific coordinator forwards or modifies
`navigation/path_tracking_command` and owns the final actuator command.

## RouteControl interfaces

### Subscriptions

| Topic | Type | Purpose |
| --- | --- | --- |
| `task_manager/task_loaded` | `TaskData` | Current task and guidance patterns |
| `gnss/fix` | `sensor_msgs/NavSatFix` | Position validity watchdog |
| `/map_origin` | `sensor_msgs/NavSatFix` | TASK WGS84-to-local conversion origin |
| `odometry` | `nav_msgs/Odometry` | Current local pose |
| `vehicle/twist_measured` | `geometry_msgs/TwistStamped` | Signed speed for recording and following |
| `robot_command` | `RobotCommand` | Record and operation commands |
| `robot_state/status` | `RobotState` | Current robot operating state |

### Publications

| Topic | Type | Purpose |
| --- | --- | --- |
| `route_control/status` | `RouteStatus` | Current and lookahead references with tracking errors |
| `route_control/recorded_route` | `GuidanceLine` | Completed manually recorded segment |
| `route_control/current_route` | `GuidanceLine` | Active TASK pattern with full metadata |
| `route_control/path` | `nav_msgs/Path` | Complete active guidance group for visualization |
| `route_control/active_path` | `nav_msgs/Path` | Current single pattern for tracking or external controllers |

The current-route and Path outputs use reliable transient-local QoS where a
late subscriber needs the active selection. Recording splits patterns when
driving direction or implement state changes and appends a zero-speed final
point to every completed pattern.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `record_distance` | `1.0` | Maximum distance between recorded points in metres |
| `record_yaw_difference` | `0.35` | Heading-change threshold for recording in radians |
| `lookahead_minimum_m` | `0.5` | Minimum route arc distance to the lookahead point |
| `lookahead_time_s` | `1.4` | Speed-dependent lookahead time multiplier |
| `lookahead_maximum_m` | `3.0` | Maximum route arc distance to the lookahead point |
| `maximum_start_distance` | `10.0` | Maximum accepted distance from a route start in metres |
| `gps_timeout_ms` | `1000` | GNSS watchdog |
| `odometry_timeout_ms` | `500` | Pose watchdog |
| `measured_twist_timeout_ms` | `500` | Signed-speed watchdog |

## PathTracking interfaces

Both nodes subscribe to `robot_state/status`, `route_control/status`,
`vehicle/twist_measured` and, when enabled, the transient-local
`robosoft/lidar/safety_status`. The NMPC implementation additionally consumes
`route_control/current_route`, `/map_origin` and `odometry` to construct its
map-frame prediction and full reference horizon. Each implementation publishes
`geometry_msgs/TwistStamped` on `navigation/path_tracking_command` at 10 Hz.
Missing commands, a non-automatic robot state or invalid lidar safety data
produce a zero command. Both implementations use the same node interfaces and
may therefore be selected in a robot launch without changing downstream
adapters. The executable names are `path_tracking_simple_node` and
`path_tracking_nmpc_node`; a launch may give the selected process the stable
runtime name `path_tracking_node`.

### Simple controller parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `robot_name` | empty | Robot identifier copied to diagnostics/log context |
| `automatic_state_machine` | `RobotState.STATE_MAIN` | State-machine value required for motion |
| `automatic_substate` | `RobotState.SUBSTATE_AUTO` | Operational substate required for motion |
| `lateral_gain` | `-500.0` | Lateral-error gain |
| `heading_gain` | `-5.0` | Heading-error gain |
| `minimum_speed_ms` | `0.3` | Crawl speed outside the accurate tracking band |
| `speed_ramp_ms` | `0.3` | Speed ramp used near the route |
| `maximum_speed_ms` | `2.0` | Maximum commanded speed magnitude |
| `maximum_curvature_m_inv` | `8.031` | Absolute curvature limit |
| `lookahead_curvature_weight` | `0.5` | Lookahead share of curvature feed-forward; current curvature supplies the remainder |
| `enable_lidar` | `true` | Apply lidar speed limits and watchdog |
| `command_timeout_ms` | `300` | Route-status command watchdog |

The NMPC model, objective and regeneration procedure are documented in
[`path_tracking_nmpc/README.md`](path_tracking_nmpc/README.md). All identified
vehicle values are runtime ROS parameters and do not require regenerating the
VIATOC solver.

## Optional Nav2 integration

RoboSoft does not require Nav2. A downstream adapter may send each
`route_control/active_path` as a `nav2_msgs/action/FollowPath` goal and replace
the native path tracker. The adapter must still preserve TASK-only speed,
direction and implement information from `route_control/current_route` and
`route_control/status`; `nav_msgs/Path` alone does not contain those fields.
The combined `route_control/path` is for visualization and may contain several
forward and reverse patterns, so it must not be used as one controller goal.
