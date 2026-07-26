cmake_minimum_required(VERSION 3.8)
project(husky_tunnel_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



add_executable(
  tunnel_backtracking_explorer
  src/tunnel_backtracking_explorer.cpp
)
ament_target_dependencies(
  tunnel_backtracking_explorer
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
)
install(
  TARGETS tunnel_backtracking_explorer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
  DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/lio_cloud_adapter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/wheel_odom_relay.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pcd_to_nav2_map.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/inspection_mission.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/frontier_explorer.py DESTINATION lib/${PROJECT_NAME})
ament_package()
