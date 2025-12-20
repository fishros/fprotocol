cmake_minimum_required(VERSION 3.8)
project(server_ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

# Build ROS2 node with fprotocol and RobotProto
add_executable(server_ros2_cpp_node
  src/server_ros2_cpp_node.cpp
  src/fprotocol/fprotocol.cpp
  src/fprotocol/RobotProto.cpp
)

target_include_directories(server_ros2_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/fprotocol>
)

ament_target_dependencies(server_ros2_cpp_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS
  server_ros2_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
