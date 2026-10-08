cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Static core libs are linked into the shared composable component.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# src/ : pure C++ planning core (ROS-free); node/ owns the ROS adapter.
add_subdirectory(src)

# ROS adapter: composable component. Defined in this root CMakeLists on purpose:
# rclcpp_components_register_node() registers via set_property(DIRECTORY
# ${PROJECT_SOURCE_DIR} ...), which is only picked up by ament_package() when
# called from the same directory.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interface REQUIRED)
find_package(std_msgs REQUIRED)

add_library(planner_component SHARED
  node/src/planner_node.cpp
  node/src/reference_buffer.cpp
)
target_include_directories(planner_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/node/include)
target_link_libraries(planner_component planner_core)
ament_target_dependencies(planner_component
  rclcpp
  rclcpp_components
  interface
  std_msgs
)
install(TARGETS planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rclcpp_components_register_node(planner_component
  PLUGIN "planner::node::PlannerNode"
  EXECUTABLE planner_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
