# UVC motion sequence

`aki_uvc_sequence_node` is the AKI-specific coordinator between generic path
tracking, treatment hardware and the final ROS2ISOBUS motion command.

During normal automatic driving it forwards the guarded core path-tracking
command. At a treatment point it stops, requests the UVC working sequence,
waits for active and completed feedback, advances the configured distance and
starts the next treatment cycle. PathTracking remains the only steering
controller. During ADVANCE this node only limits linear speed and scales
angular speed by the same ratio, preserving the PathTracking curvature.
Set `enable_uvc_sequence` to `false` to ignore treatment-point sequencing and
forward the path-tracking command continuously without changing TASK data.

## Sequence states

```text
IDLE -> WAIT_ACTIVE -> WAIT_COMPLETE -> ADVANCE -> WAIT_ACTIVE
```

Motion is permitted only while the AKI state machine reports AUTO and
RouteControl reports FOLLOWING. Stale state, route, path command, implement or
enabled lidar feedback produces a continuously refreshed zero command.

## Interfaces

The node subscribes `robot_state/status`, `route_control/status`, TECU wheel
speed, `robosoft/implement/status`, `navigation/path_tracking_command` and,
when enabled, `robosoft/lidar/safety_status`. It publishes
`navigation/cmd_vel` and transient-local `robosoft/implement/command`.
AKI launch remaps `navigation/cmd_vel` to the ROS2ISOBUS TECU Class 3 command.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `enable_lidar` | `true` | Apply lidar validity and speed limits |
| `enable_uvc_sequence` | `true` | Stop and run UVC sequences at route points whose implement state is `1`; when `false`, drive through them continuously |
| `maximum_speed_ms` | `2.0` | Maximum final speed magnitude |
| `uvc_advance_distance_m` | `0.9` | Distance driven after treatment |
| `uvc_minimum_speed_ms` | `0.3` | Minimum requested speed while approaching the next treatment point |
| `uvc_distance_gain` | `0.1` | Advance-distance control gain |
| `command_timeout_ms` | `300` | Input-command and state watchdog |

The route and curvature command is produced exclusively by
the selected RoboSoft path-tracking node; this node does not contain a second
path-tracking control law.
