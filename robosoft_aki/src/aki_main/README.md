# AKI main state machine

`aki_main_node` coordinates AKI operating states, task status and subsystem
health. Hardware protocols, localization and path control remain in dedicated
nodes. The node derives its common state identifiers, TaskManager connection,
INIT node discovery and `RobotState` publication from
`robosoft_core::RobotMainNodeBase`; AKI remote and safety rules remain here.

## State model

Initialization is two-stage:

1. `INIT_WAIT_NODES` waits for every node listed in `required_nodes`.
2. `INIT_WAIT_DATA` waits for valid remote, GNSS, odometry, TECU, implement,
   localization and optional lidar feedback.

The main substates are `MAIN_WAIT`, `MAIN_MANUAL`, `MAIN_MANUAL_RECORD`,
`MAIN_AUTO` and `MAIN_STOP`. The physical remote selects MANUAL or AUTO. GUI
START supplies automatic-start authorization, while loss of required feedback
returns the application to a safe stopped state.

## Interfaces

Subscriptions include `robosoft/remote/status`, `robot_command`,
`task_manager/task_loaded`, `odometry`, `gnss/fix`, `route_control/status`,
`robosoft/implement/status`, `robosoft/state_estimator/status`, TECU wheel
speed and TECU guidance status. When lidar is enabled, the node also consumes
`robosoft/lidar/safety_status`.

The node publishes:

- `robot_state/status` (`RobotState`)
- `localization/mode` (`LocalizationMode`, reliable transient-local)
- `robosoft/implement/command` (`ImplementCommand`, reliable transient-local)

Task transitions use the `task_manager/modify_status` service.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `required_nodes` | built-in list | ROS graph nodes required before data initialization |
| `enable_lidar` | `true` | Require lidar safety and use lidar localization modes |
| `tecu_timeout_ms` | `1000` | Wheel-speed and guidance feedback watchdog |
| `state_estimator_timeout_ms` | `1000` | Localization-status watchdog |

The robot parameter file defines the deployment-specific required-node list.
