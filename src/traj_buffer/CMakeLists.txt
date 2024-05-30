cmake_minimum_required(VERSION 3.8)
project(traj_buffer)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

ament_auto_find_build_dependencies()

# ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
# target_link_libraries(
#   Eigen3::Eigen
# )

ament_auto_add_executable(vis_controls src/vis_controls.cpp)
target_link_libraries(
  Eigen3::Eigen
)
ament_auto_add_library(calculate_cz src/calculate_cz.cpp)
target_link_libraries(
  Eigen3::Eigen
)

ament_auto_add_executable(traj_buffer src/traj_buffer.cpp)
target_link_libraries(
  traj_buffer
  Eigen3::Eigen
  fmt::fmt
)

ament_auto_add_executable(life_cycle src/life_cycle.cpp)
target_link_libraries(
  life_cycle
  Eigen3::Eigen
  fmt::fmt
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
