cmake_minimum_required(VERSION 3.8)
project(fastrack_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TrackedPose.srv"
  "srv/PlannedPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

include_directories(
  include
)

add_executable(simple_tracker
  src/simple_tracker.cpp
)
add_executable(simple_planner
  src/simple_planner.cpp
)

add_executable(robot_controller
  src/robot_controller.cpp
)

ament_target_dependencies(simple_tracker
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(simple_planner
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(robot_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

rosidl_target_interfaces(simple_planner
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(simple_tracker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(robot_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")




install(TARGETS
  simple_tracker
  simple_planner
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
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

ament_package()
