# Landmark clustering and association

`LandmarkClusterer` connects sensor-independent `LandmarkDetection` objects to
persistent point landmarks. Despite the name *clustering*, the runtime
operation is nearest-neighbour data association against an existing map, with
optional creation of new map landmarks.

The `landmark_clusterer_node` subscribes
`localization/landmark_detections`, filtered `odometry`, transient-local TASK
data and `/map_origin`. It publishes associated observations on
`localization/associated_landmarks` and the full map as a transient-local
`geometry_msgs/PoseArray` on `robosoft/lidar/landmarks`.

## Processing

1. Transform each vehicle-frame detection into the local map using the current
   vehicle pose.
2. Find the nearest map landmark inside `landmark_association_distance`.
3. Reject the detection if no landmark passes the gate and mapping is disabled.
4. If mapping is enabled, create a landmark for an unmatched detection.
5. If several detections associate with the same landmark in one scan, keep
   the detection whose provisional map position is nearest the landmark.
6. Sort associated landmarks by vehicle distance and pass only the nearest
   `max_active_landmarks` to the fixed-size EKF.

The size gate in object detection, association-distance gate, duplicate
selection and EKF covariance checks together suppress outliers. There is no
separate sample-count or statistical outlier-removal filter in this pipeline.

When mapping is enabled, landmarks whose estimated diagonal covariance is
below `0.1` are eligible for persistence back into the TASK file.

## Configuration

| Node parameter | Default | Meaning |
| --- | --- | --- |
| `landmark_association_distance` | `0.5` m | Exclusive nearest-neighbour association radius |
| `max_active_landmarks` | `6` | Nearest associated landmarks passed to the EKF; generated filter supports at most six |
| `lidar_mapping` | `false` | Create unmatched landmarks and synchronize reliable landmarks to TASK data |
| `odom_frame` | `odom` | Cartesian TASK-landmark and vehicle-pose frame |

The node consumes `localization/landmark_estimates` from the EKF to retain
updated landmark positions and covariance across scans. Arrays include the
active TASK map identifier; feedback for an older task is rejected. When
mapping is enabled, reliable landmarks are synchronized through
`task_manager/update_lidar_clusters` every two seconds.
