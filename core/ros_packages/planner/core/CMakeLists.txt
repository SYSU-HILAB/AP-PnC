# Pure C++ planning core: no ROS, no rclcpp, no message types.
# Sources here are also compiled directly by the pybind11 binding (python/).

add_subdirectory(gcopter)
add_subdirectory(basic_trajectories)
add_subdirectory(tailsitter_df)

add_library(planner_core STATIC
  src/problem_config.cpp
  src/planner.cpp
  src/trajectory.cpp
)
target_include_directories(planner_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(planner_core
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
    gcopter
    basic_trajectory
    lemniscate
    circle
    sin
    line
    verloop
    tailsitter_df
)
