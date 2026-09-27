cmake_minimum_required(VERSION 3.8)
project(nmpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(NMPC_BUNDLE_PATH "$ENV{NMPC_BUNDLE_PATH}" CACHE PATH
    "Full path to libnmpc_bundle.a")

if(NOT NMPC_BUNDLE_PATH)
  message(FATAL_ERROR
    "NMPC_BUNDLE_PATH must be set to the location of libnmpc_bundle.a")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

# Composable component: also produces the standalone `nmpc_node` executable.
ament_auto_add_library(${PROJECT_NAME}_component SHARED src/nmpc_node.cpp)

target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/solver/include
)

target_link_libraries(${PROJECT_NAME}_component
  ${NMPC_BUNDLE_PATH}
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
