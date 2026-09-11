# Safety

Reusable sensor-level safety processing. Robot state machines and final
actuator watchdogs remain responsible for system-level safe-state transitions.

## Components

| Node or library | Responsibility |
| --- | --- |
| `lidar_safety_node` | Converts a vendor-independent point cloud into a safe speed limit |
| `lidar_safety` | Corridor evaluation and speed-ramp calculations |

The node consumes `sensor_msgs/PointCloud2` on `cloud`. Robot launch files
start the appropriate lidar driver, publish the sensor transform and remap the
driver output if necessary.

## Output

`robosoft/lidar/safety_status` uses
`robosoft_interfaces/LidarSafetyStatus` and reliable transient-local QoS. It
reports connection/validity, detected obstacles and the maximum allowed speed.
Missing or stale point clouds publish an unavailable fail-safe status rather
than retaining the previous speed limit.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `minimum_forward_distance_m` | `0.4` | Stop distance in the forward corridor |
| `forward_corridor_half_width_m` | `0.4` | Half-width of the forward corridor |
| `forward_ignore_distance_m` | `0.05` | Near-sensor forward exclusion |
| `side_forward_min_m` | `-0.28` | Rear boundary of the side region |
| `side_forward_max_m` | `0.35` | Front boundary of the side region |
| `side_ignore_distance_m` | `0.05` | Near-sensor side exclusion |
| `speed_ramp_per_s` | `0.5` | Recovery rate for the allowed speed |
| `maximum_speed_m_s` | `2.0` | Upper bound for a clear point cloud |
| `cloud_timeout_ms` | `500` | Point-cloud watchdog timeout |

The defaults are algorithm defaults, not a safety certification. Corridor
geometry, transforms, stopping distance and timeout must be validated for each
physical robot before autonomous operation.
