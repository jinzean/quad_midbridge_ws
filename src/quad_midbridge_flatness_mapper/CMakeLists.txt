cmake_minimum_required(VERSION 3.8)
project(quad_midbridge_flatness_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quad_midbridge_msgs REQUIRED)

add_executable(flatness_mapper_node src/flatness_mapper_node.cpp)
ament_target_dependencies(flatness_mapper_node rclcpp geometry_msgs quad_midbridge_msgs)

install(TARGETS flatness_mapper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
