cmake_minimum_required(VERSION 3.8)
project(ros2_bt_turtlebot4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

add_executable(turtlebot4_bt_node 
  src/turtlebot4_bt_combined.cpp
)

ament_target_dependencies(turtlebot4_bt_node ${dependencies})

# Install targets
install(TARGETS
  turtlebot4_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
