cmake_minimum_required(VERSION 3.8)
project(basic_trajectories)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE Release)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(basic_trajectory SHARED src/trajectory.cpp)
target_link_libraries(basic_trajectory Eigen3::Eigen)

ament_auto_add_library(lemniscate SHARED src/lemniscate.cpp)
target_link_libraries(lemniscate Eigen3::Eigen)

ament_auto_add_library(circle SHARED src/circle.cpp)
target_link_libraries(circle Eigen3::Eigen)

ament_auto_add_library(sin SHARED src/sin.cpp)
target_link_libraries(sin Eigen3::Eigen)

ament_auto_add_library(line SHARED src/line.cpp)
target_link_libraries(line Eigen3::Eigen)

ament_auto_add_library(verloop SHARED src/vertical_loop.cpp)
target_link_libraries(verloop Eigen3::Eigen)



ament_auto_add_executable(trajectory_csv src/trajectory_csv.cpp)
target_link_libraries(trajectory_csv
Eigen3::Eigen 
basic_trajectory
sin
verloop
)


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

ament_auto_package()
