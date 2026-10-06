cmake_minimum_required(VERSION 3.8)
project(ekf_quat_pose)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include/ekf_quat_pose
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ekf_quat src/ekf_node_vio_timesync_with_acc_pub.cpp src/conversion.cpp)
add_executable(imu_add_noise src/imu_add_noise.cpp src/conversion.cpp)

ament_target_dependencies(ekf_quat
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(imu_add_noise
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS ekf_quat
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS imu_add_noise
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
