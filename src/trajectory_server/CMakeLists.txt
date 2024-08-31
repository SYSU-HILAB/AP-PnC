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
# sincle autodiff is not easily found

# Include directories

# only global_planning.cpp
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)
# Declare a C++ executable
# include_directories(include/gcopter)
ament_auto_add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS}) # Add other source files as needed
list(APPEND CMAKE_PREFIX_PATH "${HOME}/miniconda3")
find_package(autodiff REQUIRED)
find_package(yaml-cpp REQUIRED)
target_link_libraries(
        ${PROJECT_NAME}_node
        Eigen3::Eigen  # Eigen support namespace CMake
        autodiff::autodiff
)

install(DIRECTORY
        launch config meshes
        DESTINATION share/${PROJECT_NAME}
)

# Specify libraries to link a library or executable target against
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp pcl_conversions pcl_ros)

# Install executables and/or libraries
ament_auto_package()