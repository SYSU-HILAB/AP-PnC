cmake_minimum_required(VERSION 3.12)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Ceres REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(px4_interface src/px4_interface.cpp)
target_link_libraries(
  px4_interface
  Ceres::ceres
  Eigen3::Eigen
)

# launch files live in the bringup package

ament_auto_package()
