cmake_minimum_required(VERSION 3.16)
project(robosoft_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robosoft_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tinyxml2 REQUIRED)

option(BUILD_ROBOSOFT_GUI "Build the Qt 6 RoboSoft operator GUI" ON)

add_library(robot_main_state_machine SHARED
  src/state_machine/robot_main_node_base.cpp
)
target_include_directories(robot_main_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_main_state_machine
  rclcpp
  robosoft_interfaces
)

if(BUILD_ROBOSOFT_GUI)
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_AUTOUIC ON)

  add_executable(robosoft_gui_node
    src/gui/main.cpp
    src/gui/gui_main_node.cpp
    src/gui/gui_main_node.hpp
    resources/gui/qml.qrc
  )
  target_compile_definitions(robosoft_gui_node PRIVATE QT_NO_KEYWORDS)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|ARM|aarch64|AARCH64)")
    target_compile_definitions(robosoft_gui_node PRIVATE
      ROBOSOFT_USE_VIRTUAL_KEYBOARD
    )
  endif()
  target_include_directories(robosoft_gui_node PRIVATE src/gui)
  # Keep ament's dependency link calls on an interface target. Qt's static
  # finalization adds keyword-signature links to the executable, and CMake
  # does not allow mixing those with ament's plain-signature calls.
  add_library(robosoft_gui_ros_dependencies INTERFACE)
  ament_target_dependencies(robosoft_gui_ros_dependencies INTERFACE
    geometry_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    robosoft_interfaces
  )
  target_link_libraries(robosoft_gui_node PRIVATE
    robosoft_gui_ros_dependencies
    Qt6::Core
    Qt6::Gui
    Qt6::QuickControls2
    Qt6::Quick
    Qt6::Qml
  )
  # Static embedded Qt builds do not discover platform plugins from shared
  # libraries at runtime. Import linuxfb when that plugin target is available;
  # dynamic desktop Qt installations do not expose this target and are
  # unaffected.
  if(TARGET Qt6::QLinuxFbIntegrationPlugin)
    target_compile_definitions(robosoft_gui_node PRIVATE
      ROBOSOFT_STATIC_LINUXFB_PLUGIN
    )
    target_link_libraries(robosoft_gui_node PRIVATE
      Qt6::QLinuxFbIntegrationPlugin
    )
  endif()
  qt_finalize_executable(robosoft_gui_node)
  qt_import_qml_plugins(robosoft_gui_node)
  install(TARGETS robosoft_gui_node DESTINATION lib/${PROJECT_NAME})
endif()

# ==========================================
# GNSS NODES
# ==========================================
add_executable(nmea0183_parser_node
  src/gnss/nmea0183_parser/nmea0183_parser_node.cpp
  src/gnss/nmea0183_parser/nmea0183_parser.cpp
)
target_include_directories(nmea0183_parser_node PRIVATE
  src/gnss/nmea0183_parser
)
ament_target_dependencies(nmea0183_parser_node
  geometry_msgs rclcpp robosoft_interfaces sensor_msgs std_msgs
)

add_executable(serial_node
  src/gnss/serial/serial_node.cpp
)
ament_target_dependencies(serial_node rclcpp robosoft_interfaces std_msgs)

add_executable(ntrip_client_node
  src/gnss/ntrip_client/ntrip_client_node.cpp
)
target_include_directories(ntrip_client_node PRIVATE
  src/gnss/ntrip_client
)
ament_target_dependencies(ntrip_client_node
  rclcpp robosoft_interfaces std_msgs
)

add_executable(gps_to_cartesian_node
  src/gnss/gps_to_cartesian/gps_to_cartesian_node.cpp
)
target_include_directories(gps_to_cartesian_node PRIVATE
  src/gnss/gps_to_cartesian
)
ament_target_dependencies(gps_to_cartesian_node
  geometry_msgs nav_msgs rclcpp robosoft_interfaces sensor_msgs tf2_ros
)

add_library(task_xml STATIC
  src/task/task_manager/task.cpp
)
target_include_directories(task_xml PUBLIC
  src/task/task_manager
)
ament_target_dependencies(task_xml
  robosoft_interfaces
  geometry_msgs
  tinyxml2
)
target_link_libraries(task_xml tinyxml2::tinyxml2)

# VIATOC generated EKF sources are compiled unchanged. ROS adaptation and
# lidar association remain in separate hand-written translation units.
add_library(state_estimator_core
  src/localization/state_estimator/generated/ekf.cpp
  src/localization/state_estimator/generated/VehicleEKF.cpp
  src/localization/state_estimator/lidar_cluster.cpp
  src/localization/state_estimator/state_estimator.cpp
)
target_include_directories(state_estimator_core PUBLIC
  src/localization/state_estimator
  src/localization/state_estimator/generated
)

add_executable(state_estimator_node
  src/localization/state_estimator/state_estimator_node.cpp
)
target_include_directories(state_estimator_node PRIVATE
  src/localization/state_estimator
  src/localization/state_estimator/generated
)
target_link_libraries(state_estimator_node state_estimator_core)
ament_target_dependencies(state_estimator_node
  geometry_msgs
  nav_msgs
  rclcpp
  robosoft_interfaces
  sensor_msgs
  std_msgs
  tf2_ros
)

# ==========================================
# ROUTE CONTROL NODE
# ==========================================
add_executable(route_control_node
  src/navigation/route_control/route_control_node.cpp
)
target_include_directories(route_control_node PRIVATE
  src/navigation/route_control
)
ament_target_dependencies(route_control_node
  rclcpp
  robosoft_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
)

add_executable(path_tracking_simple_node
  src/navigation/path_tracking_simple/path_tracking_simple_node.cpp
)
target_include_directories(path_tracking_simple_node PRIVATE
  include
  src/navigation/path_tracking_simple
)
ament_target_dependencies(path_tracking_simple_node
  geometry_msgs
  rclcpp
  robosoft_interfaces
)

# VIATOC-generated files are checked in so building RoboSoft does not require
# the generator. Physical model values and objective weights remain runtime
# parameters configured by PathTrackingNmpcController.
add_library(path_tracking_nmpc_solver STATIC
  src/navigation/path_tracking_nmpc/generated/PathTrackingNmpcSolver.cpp
  src/navigation/path_tracking_nmpc/generated/nmpc.cpp
  src/navigation/path_tracking_nmpc/generated/projection.cpp
  src/navigation/path_tracking_nmpc/generated/solver.cpp
  src/navigation/path_tracking_nmpc/path_tracking_nmpc_controller.cpp
)
set_source_files_properties(
  src/navigation/path_tracking_nmpc/generated/PathTrackingNmpcSolver.cpp
  src/navigation/path_tracking_nmpc/generated/nmpc.cpp
  src/navigation/path_tracking_nmpc/generated/projection.cpp
  src/navigation/path_tracking_nmpc/generated/solver.cpp
  PROPERTIES COMPILE_OPTIONS "-w"
)
target_include_directories(path_tracking_nmpc_solver PUBLIC
  include
  src/navigation/path_tracking_nmpc/generated
)

add_executable(path_tracking_nmpc_node
  src/navigation/path_tracking_nmpc/path_tracking_nmpc_node.cpp
)
target_include_directories(path_tracking_nmpc_node PRIVATE
  include
  src/navigation/path_tracking_nmpc
)
target_link_libraries(path_tracking_nmpc_node path_tracking_nmpc_solver)
ament_target_dependencies(path_tracking_nmpc_node
  geometry_msgs
  nav_msgs
  rclcpp
  robosoft_interfaces
  sensor_msgs
)

add_executable(lidar_safety_node
  src/safety/lidar_safety/lidar_safety_node.cpp
)
target_include_directories(lidar_safety_node PRIVATE
  src/safety/lidar_safety
)
ament_target_dependencies(lidar_safety_node
  rclcpp
  robosoft_interfaces
  sensor_msgs
)
# ==========================================
# TASK MANAGER NODE
# ==========================================
add_executable(task_manager_node
  src/task/task_manager/task_manager_node.cpp
)
target_include_directories(task_manager_node PRIVATE
  src/task/task_manager
)
target_link_libraries(task_manager_node
  task_xml
)
ament_target_dependencies(task_manager_node
  rclcpp
  robosoft_interfaces
)

# ==========================================
# Install targets
# ==========================================
install(TARGETS
  robot_main_state_machine
  EXPORT robot_main_state_machine_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  gps_to_cartesian_node
  lidar_safety_node
  nmea0183_parser_node
  ntrip_client_node
  path_tracking_simple_node
  path_tracking_nmpc_node
  route_control_node
  serial_node
  state_estimator_node
  task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY tasks/
  DESTINATION share/${PROJECT_NAME}/tasks
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_nmea0183_parser
    test/gnss/test_nmea0183_parser.cpp
    src/gnss/nmea0183_parser/nmea0183_parser.cpp
  )
  target_include_directories(test_nmea0183_parser PRIVATE
    src/gnss/nmea0183_parser
  )

  ament_add_gtest(
    test_ntrip_protocol
    test/gnss/test_ntrip_protocol.cpp
  )
  target_include_directories(test_ntrip_protocol PRIVATE
    src/gnss/ntrip_client
  )

  ament_add_gtest(test_task_path test/task/test_task_path.cpp)
  target_include_directories(test_task_path PRIVATE
    src/task/task_manager
  )
  add_executable(test_task_xml_roundtrip
    test/task/test_task_xml_roundtrip.cpp
  )
  target_include_directories(test_task_xml_roundtrip PRIVATE
    src/task/task_manager
  )
  target_compile_definitions(test_task_xml_roundtrip PRIVATE
    ROBOSOFT_EMPTY_TASK_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tasks/EMPTY_TASK.XML"
    ROBOSOFT_TASK_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/tasks"
  )
  target_link_libraries(test_task_xml_roundtrip task_xml)
  add_test(NAME task_xml_roundtrip COMMAND test_task_xml_roundtrip)

  ament_add_gtest(
    test_path_tracking_simple_controller
    test/navigation/test_path_tracking_simple_controller.cpp
  )
  target_include_directories(test_path_tracking_simple_controller PRIVATE include)

  ament_add_gtest(
    test_path_tracking_nmpc_controller
    test/navigation/test_path_tracking_nmpc_controller.cpp
  )
  target_include_directories(test_path_tracking_nmpc_controller PRIVATE include)
  target_link_libraries(
    test_path_tracking_nmpc_controller path_tracking_nmpc_solver)

  ament_add_gtest(test_lidar_safety test/safety/test_lidar_safety.cpp)
  target_include_directories(test_lidar_safety PRIVATE
    src/safety/lidar_safety
  )

  ament_add_gtest(
    test_lidar_cluster
    test/localization/test_lidar_cluster.cpp
    src/localization/state_estimator/lidar_cluster.cpp
  )
  target_include_directories(test_lidar_cluster PRIVATE
    src/localization/state_estimator
  )

  ament_add_gtest(
    test_state_estimator
    test/localization/test_state_estimator.cpp
  )
  target_include_directories(test_state_estimator PRIVATE
    src/localization/state_estimator
    src/localization/state_estimator/generated
  )
  target_link_libraries(test_state_estimator state_estimator_core)
endif()

ament_export_include_directories(include)
ament_export_targets(robot_main_state_machine_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robosoft_interfaces)
ament_package()
