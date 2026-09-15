cmake_minimum_required(VERSION 3.16)
project(robosoft_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Custom message files
set(msg_files
  "msg/RoutePoint.msg"
  "msg/GuidanceLine.msg"
  "msg/TaskData.msg"
  "msg/RobotState.msg"
  "msg/RobotCommand.msg"
  "msg/RemoteControlStatus.msg"
  "msg/RouteStatus.msg"
  "msg/ImplementCommand.msg"
  "msg/ImplementStatus.msg"
  "msg/LidarSafetyStatus.msg"
  "msg/LocalizationMode.msg"
  "msg/LidarCluster.msg"
  "msg/LidarClusterObservation.msg"
  "msg/LidarClusterObservationArray.msg"
  "msg/LandmarkDetection.msg"
  "msg/LandmarkDetectionArray.msg"
  "msg/AssociatedLandmark.msg"
  "msg/AssociatedLandmarkArray.msg"
  "msg/StateEstimatorStatus.msg"
  "msg/TimerPerformance.msg"
)

# Custom service files
set(srv_files
  "srv/SelectTask.srv"
  "srv/SaveTask.srv"
  "srv/ModifyTaskStatus.srv"
  "srv/UpdateLidarClusters.srv"
)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
