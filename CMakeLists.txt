cmake_minimum_required(VERSION 3.8)
project(tf2_goal_pose)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf2_goal_pose_node src/tf2_goal_pose_node.cpp)

ament_target_dependencies(
  tf2_goal_pose_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  tf2_goal_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()