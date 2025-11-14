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
# find_package(interface REQUIRED)
# add_executable(offboard_control src/offboard_control.cpp)
ament_auto_add_executable(offboard_control src/offboard_control.cpp)
ament_auto_add_executable(offboard_mode src/offboard_mode.cpp)
ament_auto_add_library(helper src/helper.cpp)
target_link_libraries(
  helper
  Ceres::ceres
  Eigen3::Eigen
)
# ament_target_dependencies(
#   offboard_control
#   rclcpp
#   px4_msgs
# )

ament_auto_add_executable(px4_interface src/px4_interface.cpp)
target_link_libraries(
  px4_interface
  Eigen3::Eigen
)



if(BUILD_TESTING)
  # find_package(  REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_add_gtest(test_px4_interface test/test_px4_interface.cpp)
  # ament_target_dependencies(test_px4_interface rclcpp px4_msgs "interface" nav_msgs std_msgs Eigen3)


  # ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
