cmake_minimum_required(VERSION 3.8)
project(quad_midbridge_mpcc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED "Enable wiring for generated acados C code" OFF)
set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR "" CACHE PATH "Include dir of generated acados C solver")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_INCLUDE_DIR "" CACHE PATH "Include dir of acados headers, e.g. ACADOS_SOURCE_DIR/include")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_EXTRA_INCLUDE_DIRS "" CACHE STRING "Semicolon-separated extra acados include dirs")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR "" CACHE PATH "Library dir of generated acados C solver")
set(QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR "" CACHE PATH "Library dir of acados, e.g. ACADOS_SOURCE_DIR/lib")
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
  set(_generated_code_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mpcc_acados_generated/c_generated_code")
  if(NOT QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR AND EXISTS "${_generated_code_dir}/acados_solver_quad_midbridge_mpcc_ocp.h")
    set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR "${_generated_code_dir}")
  endif()
  if(NOT QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR AND EXISTS "${_generated_code_dir}/libacados_ocp_solver_quad_midbridge_mpcc_ocp.so")
    set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR "${_generated_code_dir}")
  endif()
  if(NOT QUAD_MIDBRIDGE_MPCC_ACADOS_INCLUDE_DIR AND DEFINED ENV{ACADOS_SOURCE_DIR})
    set(QUAD_MIDBRIDGE_MPCC_ACADOS_INCLUDE_DIR "$ENV{ACADOS_SOURCE_DIR}/include")
  endif()
  if(NOT QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR AND DEFINED ENV{ACADOS_SOURCE_DIR})
    set(QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR "$ENV{ACADOS_SOURCE_DIR}/lib")
  endif()
  if(NOT QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS)
    set(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS
      acados_ocp_solver_quad_midbridge_mpcc_ocp
      acados
      hpipm
      blasfeo
      qpOASES_e
      m
    )
  endif()

  target_compile_definitions(mpcc_solver_core PUBLIC QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED)
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR)
    target_include_directories(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR})
  endif()

  set(_quad_midbridge_acados_include_dirs)
  foreach(_acados_include_dir IN LISTS
      QUAD_MIDBRIDGE_MPCC_ACADOS_INCLUDE_DIR
      QUAD_MIDBRIDGE_MPCC_ACADOS_EXTRA_INCLUDE_DIRS)
    if(_acados_include_dir)
      list(APPEND _quad_midbridge_acados_include_dirs "${_acados_include_dir}")
      foreach(_acados_subdir blasfeo/include hpipm/include)
        if(EXISTS "${_acados_include_dir}/${_acados_subdir}")
          list(APPEND _quad_midbridge_acados_include_dirs "${_acados_include_dir}/${_acados_subdir}")
        endif()
      endforeach()
    endif()
  endforeach()
  if(_quad_midbridge_acados_include_dirs)
    list(REMOVE_DUPLICATES _quad_midbridge_acados_include_dirs)
    target_include_directories(mpcc_solver_core PUBLIC ${_quad_midbridge_acados_include_dirs})
  endif()

  set(_quad_midbridge_acados_runtime_lib_dirs)
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR)
    target_link_directories(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR})
    list(APPEND _quad_midbridge_acados_runtime_lib_dirs "${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR}")
  endif()
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR)
    target_link_directories(mpcc_solver_core PUBLIC ${QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR})
    list(APPEND _quad_midbridge_acados_runtime_lib_dirs "${QUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR}")
  endif()
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS)
    target_link_libraries(mpcc_solver_core ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS})
  endif()
  if(_quad_midbridge_acados_runtime_lib_dirs)
    list(REMOVE_DUPLICATES _quad_midbridge_acados_runtime_lib_dirs)
    set_target_properties(mpcc_solver_core PROPERTIES
      BUILD_RPATH "${_quad_midbridge_acados_runtime_lib_dirs}"
      INSTALL_RPATH "${_quad_midbridge_acados_runtime_lib_dirs}"
    )
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

if(QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED AND _quad_midbridge_acados_runtime_lib_dirs)
  target_link_options(mpcc_node PRIVATE "LINKER:--no-as-needed")
  if(QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS)
    target_link_libraries(mpcc_node ${QUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS})
  endif()
  set_target_properties(mpcc_node PROPERTIES
    BUILD_RPATH "${_quad_midbridge_acados_runtime_lib_dirs}"
    INSTALL_RPATH "${_quad_midbridge_acados_runtime_lib_dirs}"
  )
endif()

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

install(DIRECTORY tools/
  DESTINATION share/${PROJECT_NAME}/tools
)

ament_package()
