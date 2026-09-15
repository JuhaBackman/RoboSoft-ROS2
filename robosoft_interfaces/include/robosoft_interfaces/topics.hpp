// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

namespace robosoft_interfaces
{

// Shared application topics. Protocol-owned ISOBUS topics remain in
// ros2_isobus/topics.hpp so there is one authoritative name per interface.
constexpr char kRobotCommandTopic[] = "robot_command";
constexpr char kRobotStateTopic[] = "robot_state/status";
constexpr char kOdometryTopic[] = "odometry";
constexpr char kMeasuredTwistTopic[] = "vehicle/twist_measured";
constexpr char kRoutePathTopic[] = "route_control/path";
constexpr char kActiveRoutePathTopic[] = "route_control/active_path";
constexpr char kMapOriginTopic[] = "/map_origin";
constexpr char kGnssFixTopic[] = "gnss/fix";
constexpr char kGnssHeadingTopic[] = "gnss/heading";
constexpr char kGnssAttitudeTopic[] = "gnss/attitude";
constexpr char kGnssImuTopic[] = "gnss/imu";
constexpr char kGnssVelocityTopic[] = "gnss/velocity";
constexpr char kNmea0183SentenceTopic[] = "nmea0183/sentence";
constexpr char kNtripCorrectionsTopic[] = "ntrip/corrections";
constexpr char kSerialLineTopic[] = "serial/line";
constexpr char kSerialWriteTopic[] = "serial/write";
constexpr char kRosoutTopic[] = "rosout";
constexpr char kDiagnosticsTopic[] = "diagnostics";
constexpr char kAkiMainTimerPerformanceTopic[] =
  "performance/aki_main";
constexpr char kPathTrackingTimerPerformanceTopic[] =
  "performance/path_tracking";

// TaskManager services and its transient-local task snapshot.
constexpr char kTaskLoadedTopic[] = "task_manager/task_loaded";
constexpr char kTaskSelectService[] = "task_manager/select_task";
constexpr char kTaskSaveService[] = "task_manager/save_task";
constexpr char kTaskModifyStatusService[] = "task_manager/modify_status";
constexpr char kTaskUpdateLidarClustersService[] =
  "task_manager/update_lidar_clusters";

// Route recording, selection and tracking.
constexpr char kRouteStatusTopic[] = "route_control/status";
constexpr char kRecordedRouteTopic[] = "route_control/recorded_route";
constexpr char kCurrentRouteTopic[] = "route_control/current_route";
constexpr char kPathTrackingCommandTopic[] = "navigation/path_tracking_command";
constexpr char kNavigationCommandTopic[] = "navigation/cmd_vel";

// Application subsystem interfaces.
constexpr char kRemoteControlStatusTopic[] = "robosoft/remote/status";
constexpr char kImplementCommandTopic[] = "robosoft/implement/command";
constexpr char kImplementStatusTopic[] = "robosoft/implement/status";
constexpr char kLidarSafetyStatusTopic[] = "robosoft/lidar/safety_status";
constexpr char kLidarClusterObservationsTopic[] =
  "robosoft/lidar/cluster_observations";
constexpr char kLidarLandmarksTopic[] = "robosoft/lidar/landmarks";
constexpr char kLandmarkDetectionsTopic[] =
  "localization/landmark_detections";
constexpr char kAssociatedLandmarksTopic[] =
  "localization/associated_landmarks";
constexpr char kLandmarkEstimatesTopic[] =
  "localization/landmark_estimates";
constexpr char kStateEstimatorStatusTopic[] =
  "robosoft/state_estimator/status";
constexpr char kRawOdometryTopic[] = "odometry/raw";
constexpr char kLocalizationModeTopic[] = "localization/mode";

}  // namespace robosoft_interfaces
