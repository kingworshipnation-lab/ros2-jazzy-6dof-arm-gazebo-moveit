cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(pose_goal_follower src/pose_goal_follower.cpp)
ament_target_dependencies(
  pose_goal_follower
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2_ros
  trajectory_msgs
)

add_executable(target_pose_planner src/target_pose_planner.cpp)
ament_target_dependencies(
  target_pose_planner
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

add_executable(send_pose src/send_pose.cpp)
ament_target_dependencies(
  send_pose
  geometry_msgs
  rclcpp
)

add_executable(send_pose_goal src/send_pose_goal.cpp)
ament_target_dependencies(
  send_pose_goal
  geometry_msgs
  rclcpp
)

add_executable(target_pose_demo src/target_pose_demo.cpp)
ament_target_dependencies(
  target_pose_demo
  geometry_msgs
  rclcpp
)

add_executable(tracking_demo src/tracking_demo.cpp)
ament_target_dependencies(
  tracking_demo
  geometry_msgs
  rclcpp
)

add_executable(trajectory_target_publisher src/trajectory_target_publisher.cpp)
ament_target_dependencies(
  trajectory_target_publisher
  geometry_msgs
  rclcpp
)

install(
  TARGETS
    pose_goal_follower
    target_pose_planner
    send_pose
    send_pose_goal
    target_pose_demo
    tracking_demo
    trajectory_target_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
