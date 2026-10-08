cmake_minimum_required(VERSION 3.8)
project(nmpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(AP_PNC_ROOT "$ENV{AP_PNC_DIR}")
if(NOT AP_PNC_ROOT)
  get_filename_component(AP_PNC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()
if(NOT IS_ABSOLUTE "${AP_PNC_ROOT}")
  message(FATAL_ERROR "AP_PNC_DIR must be absolute")
endif()
set(NMPC_BUNDLE_PATH "${AP_PNC_ROOT}/.artifacts/nmpc_solver/libnmpc_bundle.a" CACHE FILEPATH
    "NMPC static bundle (CMake override, not a per-file environment variable)")

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()
# src/ owns the ROS-free solver wrapper and tracking logic.
add_subdirectory(src/controller)

# Composable component: also produces the standalone `nmpc_node` executable.
ament_auto_add_library(${PROJECT_NAME}_component SHARED node/src/nmpc_node.cpp)

target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

target_link_libraries(${PROJECT_NAME}_component
  nmpc_tracking_core
  Eigen3::Eigen
  fmt::fmt
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_component rclcpp_components)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "NMPC"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# launch files live in the bringup package

ament_auto_package()
