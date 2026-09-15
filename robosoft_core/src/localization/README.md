# Localization

RoboSoft localization combines a local GNSS pose, vehicle motion and mapped
landmark observations. Object recognition, data association and state
estimation are separate stages so that another detector can produce landmark
observations without changing the clustering or EKF implementation.

```text
sensor_msgs/PointCloud2
          |
          v
LidarObjectDetector       ordered scan -> vehicle-frame object centres
          |
          v
LandmarkClusterer         detections -> associated TASK/map landmarks
          |
          v
ExtendedKalmanFilter      GNSS + motion + associated landmarks -> odometry
```

## Components

| Component | Responsibility | Documentation |
| --- | --- | --- |
| `object_detection` | Extract compact foreground objects from an ordered planar lidar scan | [Object detection](object_detection/README.md) |
| `clustering` | Associate sensor-independent detections with persistent map landmarks | [Clustering and association](clustering/README.md) |
| `extended_kalman_filter` | Fuse vehicle and landmark states through the generated EKF | [Extended Kalman filter](extended_kalman_filter/README.md) |

The common `LandmarkDetection` type contains a two-dimensional observation in
the vehicle frame. This boundary is intentionally independent of lidar scan
format and object class. A camera, another lidar detector or a semantic
detector can therefore feed the same association and estimation stages when it
can produce compatible, mapped point landmarks.

## Localization modes

`localization/mode` uses `robosoft_interfaces/LocalizationMode` constants:

| Constant | Behaviour |
| --- | --- |
| `RAW_GNSS` | Forward raw local GNSS odometry |
| `GNSS_LIDAR_EKF` | Fuse GNSS motion and lidar landmark observations |
| `LANDMARK_DEAD_RECKONING` | Continue landmark/dead-reckoning estimation without normal GNSS correction |

The application state machine chooses the mode. Localization remains
independent of the vehicle bus, actuators and operator controls.

## Coordinate frames

- `/map_origin` defines the WGS84 origin of the local Cartesian map.
- `odometry/raw`, map landmarks and estimated `odometry` use `odom_frame`.
- Detector output and EKF landmark measurements use `base_frame`, with `x`
  forward and `y` left.
- The detector resolves the point-cloud sensor frame to its configured vehicle
  frame through TF2.
- `extended_kalman_filter_node` can publish the `odom -> base_link` transform.

The TASK manager retains landmark coordinates in WGS84. The clusterer converts
them once to the shared local frame using `/map_origin`.

## ROS pipeline

The implementation is divided into three independently replaceable nodes:

| Node | Input | Output |
| --- | --- | --- |
| `lidar_object_detector_node` | `sensor_msgs/PointCloud2` on `cloud` | `LandmarkDetectionArray` on `localization/landmark_detections` |
| `landmark_clusterer_node` | Detections, `odometry`, TASK data and `/map_origin` | `AssociatedLandmarkArray` on `localization/associated_landmarks` |
| `extended_kalman_filter_node` | Raw odometry, measured motion, localization mode and associated landmarks | Filtered `odometry`, estimator status and `localization/landmark_estimates` |

The estimate topic feeds updated map-landmark states back to the clusterer.
Every associated array carries a TASK map identifier, so delayed estimates
cannot modify landmarks belonging to a newly loaded task.
