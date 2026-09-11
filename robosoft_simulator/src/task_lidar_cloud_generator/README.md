# TASK lidar cloud generator

`task_lidar_cloud_generator_node` ray-casts point obstacles from an ISO 11783
TASK file and publishes a TiM5xx-shaped `sensor_msgs/PointCloud2` on `cloud`.
It models the simulated environment and sensor geometry but contains no SICK
Ethernet protocol. The output can therefore be consumed directly by ROS
algorithms or passed to
[`sick_tim5xx_simulator_node`](../sick_tim5xx_simulator/README.md) for testing a
controller through the real `sick_scan_xd` driver.

## Map and measurements

Each direct `PNT` child of a `PFD` with `A="5"` is a vertical circular pole.
`C` is latitude and `D` longitude in degrees. Route points, polygon vertices and
other point types are ignored. Relative `task_file` names resolve against
`task_directory`; absolute paths are supported. The map is loaded once during
startup.

The map and `vehicle/odometry` use one local ENU projection origin. Lidar mount
offsets are relative to the vehicle's rear-axle centre. The generated cloud has
811 ordered points from -135 to +135 degrees. A hit has finite `x`, `y`, `z`
and synthetic `intensity`; no-return beams contain NaN coordinates. The model
includes nearest-pole occlusion but not noise, reflectivity, beam divergence,
motion distortion or a ground plane.

## Parameters and interfaces

| Parameter | Default | Meaning |
| --- | --- | --- |
| `task_file` | `metsapelto.XML` | TASK obstacle map |
| `task_directory` | installed core tasks | Base for relative filenames |
| `reference_latitude`, `reference_longitude` | example GNSS origin | Local ENU origin |
| `odometry_topic` | `vehicle/odometry` | Vehicle `nav_msgs/Odometry` input |
| `output_topic` | `cloud` | Generated `sensor_msgs/PointCloud2` output |
| `frame_id` | `cloud` | Output cloud frame |
| `publish_rate_hz` | `15.0` | Cloud rate |
| `lidar_x_m`, `lidar_y_m`, `lidar_yaw_rad` | `1.094`, `0`, `0` | Sensor mount |
| `pole_diameter_m` | `0.07` | Circular obstacle diameter |
| `range_min_m`, `range_max_m` | `0.05`, `25` | Simulated range limits |
| `odometry_timeout_s` | `1.0` | Pause output when motion data is stale |

The AKI simulator launch starts this node and the generic Ethernet adapter
together. It supplies the cloud generator with the same map origin used by the
GNSS simulator.
