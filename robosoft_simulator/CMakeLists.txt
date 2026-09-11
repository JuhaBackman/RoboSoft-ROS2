cmake_minimum_required(VERSION 3.16)
project(robosoft_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_isobus REQUIRED)
find_package(robosoft_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2 REQUIRED)

add_executable(sick_tim5xx_simulator_node
  src/sick_tim5xx_simulator/sick_tim5xx_simulator_node.cpp
)
target_include_directories(sick_tim5xx_simulator_node PRIVATE include)
ament_target_dependencies(sick_tim5xx_simulator_node rclcpp sensor_msgs)

add_executable(task_lidar_cloud_generator_node
  src/task_lidar_cloud_generator/task_lidar_cloud_generator_node.cpp
)
target_include_directories(task_lidar_cloud_generator_node PRIVATE include)
ament_target_dependencies(
  task_lidar_cloud_generator_node rclcpp nav_msgs sensor_msgs ament_index_cpp)
target_link_libraries(task_lidar_cloud_generator_node tinyxml2::tinyxml2)

add_executable(vehicle_model_node src/vehicle_model/vehicle_model_node.cpp)
ament_target_dependencies(vehicle_model_node rclcpp geometry_msgs nav_msgs)

add_executable(tractor_state_simulator_node
  src/tractor_state_simulator/tractor_state_simulator_node.cpp
)
ament_target_dependencies(tractor_state_simulator_node rclcpp std_msgs ros2_isobus)

add_executable(gnss_serial_simulator_node
  src/gnss_serial_simulator/gnss_serial_simulator_node.cpp
)
target_include_directories(gnss_serial_simulator_node PRIVATE include)
ament_target_dependencies(gnss_serial_simulator_node rclcpp nav_msgs)

add_executable(remote_simulator_node
  src/remote_simulator/remote_simulator_node.cpp
)
target_include_directories(remote_simulator_node PRIVATE include)
ament_target_dependencies(
  remote_simulator_node rclcpp sensor_msgs ros2_isobus robosoft_interfaces)

add_executable(aki_drive_controller_simulator_node
  src/aki_drive_controller_simulator/aki_drive_controller_simulator_node.cpp
)
ament_target_dependencies(
  aki_drive_controller_simulator_node
  rclcpp geometry_msgs sensor_msgs std_msgs robosoft_interfaces)

add_executable(uvc_led_simulator_node
  src/uvc_led_simulator/uvc_led_simulator_node.cpp
)
target_include_directories(uvc_led_simulator_node PRIVATE include)
ament_target_dependencies(uvc_led_simulator_node rclcpp ros2_isobus)

install(TARGETS
  sick_tim5xx_simulator_node
  task_lidar_cloud_generator_node
  vehicle_model_node
  tractor_state_simulator_node
  gnss_serial_simulator_node
  remote_simulator_node
  aki_drive_controller_simulator_node
  uvc_led_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar test/test_lidar.cpp)
  target_include_directories(test_lidar PRIVATE
    include
    src/sick_tim5xx_simulator
    src/task_lidar_cloud_generator
  )
  target_link_libraries(test_lidar tinyxml2::tinyxml2)
  ament_add_gtest(test_aki_can_protocol test/test_aki_can_protocol.cpp)
  target_include_directories(test_aki_can_protocol PRIVATE include)
  ament_target_dependencies(test_aki_can_protocol ros2_isobus)
  ament_add_gtest(test_nmea0183 test/test_nmea0183.cpp)
  target_include_directories(test_nmea0183 PRIVATE include)
endif()

ament_package()
