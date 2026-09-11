# Localization

Shared vehicle-state estimation and lidar-landmark association.

## Components

| Node or library | Responsibility |
| --- | --- |
| `state_estimator_node` | Selects localization mode, runs the EKF adapter and publishes vehicle odometry |
| `state_estimator` | Hand-written adapter around the generated filter |
| `lidar_cluster` | Associates point-cloud observations with TASK landmarks |
| `generated` | VIATOC-generated EKF and vehicle-model sources |

The files under `state_estimator/generated` are generated artifacts and must
remain unchanged. ROS subscriptions, coordinate conversion and landmark logic
belong in the hand-written translation units around them.

## Localization modes

`localization/mode` uses `robosoft_interfaces/LocalizationMode` constants:

| Constant | Behaviour |
| --- | --- |
| `RAW_GNSS` | Forward raw local GNSS odometry |
| `GNSS_LIDAR_EKF` | Fuse GNSS motion and lidar landmark observations |
| `LANDMARK_DEAD_RECKONING` | Continue landmark/dead-reckoning estimation without normal GNSS correction |

The application state machine chooses the mode. The estimator is independent
of platform-specific operator-control and vehicle-bus interfaces.

## Interfaces

### Subscriptions

| Topic | Type | Purpose |
| --- | --- | --- |
| `odometry/raw` | `nav_msgs/Odometry` | Local GNSS-derived input |
| `localization/mode` | `LocalizationMode` | Transient-local estimator mode |
| `vehicle/twist_measured` | `geometry_msgs/TwistStamped` | Signed measured vehicle motion |
| `cloud` | `sensor_msgs/PointCloud2` | Lidar point cloud |
| `task_manager/task_loaded` | `TaskData` | TASK lidar landmarks |
| `/map_origin` | `sensor_msgs/NavSatFix` | WGS84 origin used for TASK conversion |

### Publications

| Topic | Type | Purpose |
| --- | --- | --- |
| `odometry` | `nav_msgs/Odometry` | Selected or estimated local vehicle state |
| `robosoft/lidar/cluster_observations` | `LidarClusterObservationArray` | Associated landmark observations |
| `robosoft/lidar/landmarks` | `geometry_msgs/PoseArray` | Complete TASK obstacle map in `odom_frame`; reliable/transient-local, refreshed on task or map-origin changes |
| `robosoft/state_estimator/status` | `StateEstimatorStatus` | Mode, validity and estimator reliability |
| `odom -> base_link` | TF2 | Optional vehicle transform |

When lidar mapping is enabled, new clusters are stored through
`task_manager/update_lidar_clusters`.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `lidar_mapping` | `false` | Record new lidar landmarks instead of only matching them |
| `odom_frame` | `odom` | Output odometry frame |
| `base_frame` | `base_link` | Vehicle frame |
| `publish_tf` | `true` | Publish `odom -> base_link` |
| `lidar_x_offset` | `1.094` | Lidar longitudinal offset from vehicle origin in metres |
| `lidar_angle_error` | `0.0` | Static lidar yaw correction in radians |
