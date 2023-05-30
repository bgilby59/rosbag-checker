cmake_minimum_required(VERSION 3.8)
project(rosbag_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(rosbag_checker_cpp src/rosbag_checker_node.cpp)
ament_target_dependencies(rosbag_checker_cpp rclcpp rosbag2_cpp yaml-cpp)

install(TARGETS
  rosbag_checker_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
