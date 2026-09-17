cmake_minimum_required(VERSION 3.5)
project(trajectory_server)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-DSTATIC_GO=1)
add_definitions(-DDEBUG_VIZ=1)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# some package installed by miniconda
set(HOME $ENV{HOME})

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

# Include directories

# New modularized source files (global_planning.cpp has been split)
set(${PROJECT_NAME}_SRCS
    src/planning_config.cpp
    src/planning_utils.cpp
    src/attitude_computer.cpp
    src/trajectory_planner.cpp
    src/global_planning_main.cpp
)
# Declare a C++ executable
# include_directories(include/gcopter)
ament_auto_add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS}) # Add other source files as needed
target_link_libraries(
        ${PROJECT_NAME}_node
        Eigen3::Eigen  # Eigen support namespace CMake
        yaml-cpp
)

# Link the aerodynamics static libs from the colcon install tree
# (resolved via CMAKE_PREFIX_PATH pointing at the merge-install root).
find_library(AERO_ALPHA_LIB NAMES aerodynamics_alpha_solver REQUIRED)
find_library(AERO_LIB NAMES aerodynamics_aerodynamics_lib REQUIRED)
find_library(RT_LIB NAMES rt_nonfinite_matlab REQUIRED)
target_link_libraries(${PROJECT_NAME}_node ${AERO_ALPHA_LIB} ${AERO_LIB} ${RT_LIB})

ament_target_dependencies(${PROJECT_NAME}_node aerodynamics)

# ament_auto_package installs the target; launch/config live in bringup now
ament_auto_package()
