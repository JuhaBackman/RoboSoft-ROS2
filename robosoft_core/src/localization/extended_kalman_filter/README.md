# Extended Kalman filter

`ExtendedKalmanFilter` is the hand-written adapter around the generated
`VehicleEKF` implementation. The generated files in `generated/` are kept
unchanged; ROS messages, landmark containers and mode handling remain outside
them.

The `extended_kalman_filter_node` subscribes `odometry/raw`,
`vehicle/twist_measured`, `localization/mode` and
`localization/associated_landmarks`. It publishes filtered `odometry`,
`robosoft/state_estimator/status`, updated landmark estimates on
`localization/landmark_estimates` and, when enabled, the `odom -> base_link`
TF transform. `robosoft/lidar/cluster_observations` remains a compact
visualization output.

## State and process model

The fixed 15-state vector contains the vehicle pose followed by at most six
two-dimensional map landmarks:

```text
[robot_x, robot_y, robot_yaw,
 landmark_1_x, landmark_1_y, ...,
 landmark_6_x, landmark_6_y]
```

The vehicle process model integrates measured speed and curvature. In
continuous form its planar motion is equivalent to the following model. The
generated implementation discretizes it at a fixed `0.1 s` interval.

```text
robot_x_dot   = speed * cos(robot_yaw)
robot_y_dot   = speed * sin(robot_yaw)
robot_yaw_dot = speed * curvature
landmark_i_x_dot = landmark_i_y_dot = 0
```

Landmarks are stationary map states. Because the generated state vector has a
fixed size, only currently observed landmarks are inserted, nearest first.

## Measurements

The first three measurements are the local GNSS position and heading when the
selected mode permits them. Each active landmark contributes its measured
position in the vehicle frame. For a map landmark `(lx, ly)` and vehicle pose
`(x, y, yaw)`, the expected measurement is the map displacement rotated by
`-yaw`:

```text
dx = lx - x
dy = ly - y
measurement_x =  cos(yaw) * dx + sin(yaw) * dy
measurement_y = -sin(yaw) * dx + cos(yaw) * dy
```

Unused measurement slots receive infinite variance. The adapter transfers each
active landmark's covariance into the generated filter and writes its updated
map position and covariance back after the estimate.

The estimate is reported reliable when diagonal covariance is no greater than
`1e-4` for vehicle `x` and `y`, and `1e-5` for yaw. These are estimator health
limits, not measurement-validity gates.

## Generated sources

`generated/ekf.*` and `generated/VehicleEKF.*` are generated numerical code.
Do not apply formatting or hand-written ROS changes to these files. Changes to
the state-vector size, process model or measurement model require regeneration;
container, topic and parameter changes belong in the adapter or node.

## Node parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `odom_frame` | `odom` | Filter output and landmark-map frame |
| `base_frame` | `base_link` | Vehicle state and landmark-measurement frame |
| `publish_tf` | `true` | Publish the `odom -> base_link` transform |
