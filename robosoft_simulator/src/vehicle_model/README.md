# Vehicle model

`vehicle_model_node` integrates planar vehicle motion from a
`geometry_msgs/TwistStamped` command and publishes `nav_msgs/Odometry`.
Command delay, velocity and curvature limits, rate limits and first-order time
constants are parameters, allowing the same node to represent different
platform dynamics without changing source code.

The initial planar pose is configured with `initial_x_m`, `initial_y_m` and
`initial_yaw_rad`. The GNSS simulator's reference latitude and longitude
define the WGS84 position corresponding to `(0, 0)` in this local frame.

The model is intentionally a controller-integration model rather than a
high-fidelity physics engine. It is suitable for command-chain, TASK execution
and watchdog testing.
