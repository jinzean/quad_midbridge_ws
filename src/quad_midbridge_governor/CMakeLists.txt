cmake_minimum_required(VERSION 3.8)
project(quad_midbridge_governor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad_midbridge_msgs REQUIRED)

add_executable(governor_node src/governor_node.cpp)
ament_target_dependencies(governor_node rclcpp quad_midbridge_msgs)

install(TARGETS governor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
