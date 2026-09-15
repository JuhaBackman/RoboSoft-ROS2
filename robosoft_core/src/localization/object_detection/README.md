# Lidar object detection

`LidarObjectDetector` extracts small foreground surfaces from an ordered
two-dimensional lidar scan. Its default geometry is intended for circular
pole-like landmarks, but its output is the generic vehicle-frame
`LandmarkDetection` type.

The `lidar_object_detector_node` subscribes `sensor_msgs/PointCloud2` on
`cloud` and publishes `robosoft_interfaces/LandmarkDetectionArray` on
`localization/landmark_detections`. Each result includes measurement
covariance, object type and confidence. The output coordinates are in the
configured target frame. The node resolves the transform from the point-cloud
header frame through TF2; it does not contain sensor mounting offsets.

## Algorithm

For consecutive lidar beams, the detector:

1. Calculates radial distance from the lidar origin.
2. Starts a candidate when the radial distance falls by more than the edge
   threshold. A valid return after missing data is also treated as a boundary.
3. Ends the candidate when the distance rises by more than the threshold or a
   missing return is encountered.
4. Measures the chord between the candidate's first and last Cartesian scan
   points and accepts it only when it is within the configured object-size
   interval.
5. Averages the candidate points in the scan frame.
6. Uses TF2 to transform each accepted centre into `target_frame`.

Radial discontinuities distinguish the foreground surface from its
background. Cartesian chord length then rejects surfaces which cannot have the
expected physical width. These checks form the first outlier gates; the
association stage applies additional spatial checks.

The input order must follow scan angle. Unordered point clouds must first be
projected or sorted into scan order. Only planar `x` and `y` fields are used.

## Configuration

| Node parameter | Default | Meaning |
| --- | --- | --- |
| `lidar_edge_threshold` | `0.20` m | Minimum radial range jump at an object edge |
| `lidar_object_size_min` | `0.02` m | Exclusive minimum Cartesian chord width |
| `lidar_object_size_max` | `0.08` m | Exclusive maximum Cartesian chord width |
| `target_frame` | `base_link` | Vehicle frame used for published detections |
| `transform_timeout_ms` | `50` ms | Maximum wait for the scan-frame TF transform |

The cloud publisher must set a valid `header.frame_id`, and the application
must publish a transform from that sensor frame to `target_frame`.

To recognize another object class, replace or extend this stage and retain the
vehicle-frame `LandmarkDetection` contract. Clustering and the EKF do not
depend on how an object was recognized.
