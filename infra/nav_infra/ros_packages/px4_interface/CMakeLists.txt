cmake_minimum_required(VERSION 3.12)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(nmpc_bridge src/nmpc_bridge.cpp)
target_link_libraries(nmpc_bridge yaml-cpp Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_px4_interface test/test_px4_interface.cpp)
  target_include_directories(test_px4_interface PRIVATE include)
  target_link_libraries(test_px4_interface yaml-cpp)
endif()

# launch files live in the bringup package
ament_auto_package()
