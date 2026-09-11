cmake_minimum_required(VERSION 3.16)
project(robosoft_aki)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robosoft_core REQUIRED)
find_package(robosoft_interfaces REQUIRED)
find_package(ros2_isobus REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ==========================================
# AKI MAIN STATE MACHINE
# ==========================================
add_executable(aki_main_node
  src/aki_main/aki_main_node.cpp
)
target_include_directories(aki_main_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/aki_main
)
ament_target_dependencies(aki_main_node
  rclcpp
  robosoft_core
  robosoft_interfaces
  ros2_isobus
  sensor_msgs
  nav_msgs
)

# ==========================================
# AKI PATH TRACKING -> ROS2 ISOBUS ADAPTER
# ==========================================
add_executable(aki_uvc_sequence_node
  src/uvc_sequence/uvc_sequence_node.cpp
)
target_include_directories(aki_uvc_sequence_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uvc_sequence
)
ament_target_dependencies(aki_uvc_sequence_node
  rclcpp
  robosoft_interfaces
  ros2_isobus
)

# ==========================================
# AKI UVC / CAMERA LIGHT CONTROLLER
# ==========================================
add_executable(uvc_led_node
  src/uvc_led/uvc_led_node.cpp
)
target_include_directories(uvc_led_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uvc_led
)
ament_target_dependencies(uvc_led_node
  rclcpp
  robosoft_interfaces
  ros2_isobus
)


# ==========================================
# AKI PHYSICAL REMOTE CONTROLLER NODE
# ==========================================
add_executable(remote_controller_node
  src/remote_controller/remote_controller_node.cpp
)
target_include_directories(remote_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/remote_controller
)
ament_target_dependencies(remote_controller_node
  rclcpp
  robosoft_interfaces
  ros2_isobus
  sensor_msgs
)

install(TARGETS
  aki_main_node
  aki_uvc_sequence_node
  remote_controller_node
  uvc_led_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# Install launch and configuration
# ==========================================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ==========================================
# Tests
# ==========================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
  ament_add_gtest(test_remote_protocol test/test_remote_protocol.cpp)
  target_include_directories(test_remote_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/remote_controller
  )
  ament_add_gtest(test_uvc_led_protocol test/test_uvc_led_protocol.cpp)
  target_include_directories(test_uvc_led_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uvc_led
  )
endif()

ament_package()
