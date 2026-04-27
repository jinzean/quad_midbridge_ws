cmake_minimum_required(VERSION 3.8)
project(quad_midbridge_mpcc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED "Enable wiring for generated acados C code" OFF)
set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR "" CACHE PATH "Include dir of generated acados C solver")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR "" CACHE PATH "Library dir of generated acados C solver")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS "" CACHE STRING "Semicolon-separated generated/acados libs")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quad_midbridge_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(mpcc_solver_core
  src/factory.cpp
  src/reference_model_solver.cpp
  src/full_state_placeholder_solver.cpp
  src/acados_problem_builder.cpp
  src/acados_skeleton_solver.cpp
  src/acados_generated_bindings.cpp
  src/acados_generated_adapter.cpp
  src/acados_generated_solver.cpp
)

if(QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED)
  target_compile_definitions(mpcc_solver_core PUBLIC QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED)
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR)
    target_include_directories(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR})
  endif()
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR)
    target_link_directories(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR})
  endif()
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS)
    target_link_libraries(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS})
  endif()
endif()

target_include_directories(mpcc_solver_core PUBLIC include)

ament_target_dependencies(mpcc_solver_core
  rclcpp
  geometry_msgs
  quad_midbridge_msgs
  px4_msgs
)

add_executable(mpcc_node src/mpcc_node.cpp)
target_include_directories(mpcc_node PUBLIC include)
target_link_libraries(mpcc_node mpcc_solver_core)
ament_target_dependencies(mpcc_node
  rclcpp
  geometry_msgs
  quad_midbridge_msgs
  px4_msgs
)

install(TARGETS
  mpcc_solver_core
  mpcc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

ament_package()
