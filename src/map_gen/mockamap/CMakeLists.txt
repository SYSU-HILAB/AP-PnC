cmake_minimum_required(VERSION 3.5)
project(mockamap)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package( PCL REQUIRED )
# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Include directories

# we ignore the source file ces_randommap.cpp
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)
# Declare a C++ executable
ament_auto_add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS}) # Add other source files as needed
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})
install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
)

# Specify libraries to link a library or executable target against
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp pcl_conversions pcl_ros)

# Install executables and/or libraries
ament_auto_package ()