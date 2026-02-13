cmake_minimum_required(VERSION 3.10)
project(grumpy_navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(talker src/talker.cpp)
add_executable(path_planner src/path_planner.cpp)
add_executable(fake_goal_publisher src/fake_goal_publisher.cpp)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(path_planner
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(fake_goal_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  talker
  path_planner
  fake_goal_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
