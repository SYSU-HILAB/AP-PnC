# Thin ROS adapter around planner_core: the only place that depends on rclcpp.
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(planner_node
  src/planner_node.cpp
  src/trajectory_buffer.cpp
)
target_include_directories(planner_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(planner_node planner_core)
ament_target_dependencies(planner_node
  rclcpp
  interface
  std_msgs
)

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
