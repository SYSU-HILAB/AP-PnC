cmake_minimum_required(VERSION 3.8)
project(simple_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_sim_node
  src/simple_sim_node.cpp
  src/scene.cpp
  src/lidar_sim.cpp
  src/dynamics.cpp
  src/controller.cpp
)

target_link_libraries(simple_sim_node
  Eigen3::Eigen
  ${aerodynamics_LIBRARIES}
)

target_compile_features(simple_sim_node PUBLIC cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
