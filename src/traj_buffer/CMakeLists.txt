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
add_subdirectory(external)

ament_auto_find_build_dependencies()

ament_auto_add_library(aerodyn src/aerodyn.cpp)
target_link_libraries(aerodyn
  Eigen3::Eigen
  main_library
)


ament_auto_add_executable(traj_buffer src/traj_buffer.cpp)
target_link_libraries(traj_buffer
  Eigen3::Eigen
  fmt::fmt
  aerodyn
)

ament_auto_add_executable(aero_test src/aero_test.cpp)

target_link_libraries(aero_test
  # main_libray is bspline library
  main_library
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
