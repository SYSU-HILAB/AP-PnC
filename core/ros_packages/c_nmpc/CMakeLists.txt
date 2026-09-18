cmake_minimum_required(VERSION 3.8)
project(c_nmpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(NMPC_BUNDLE_PATH "$ENV{NMPC_BUNDLE_PATH}" CACHE PATH
    "Full path to libnmpc_bundle.a")

if(NOT NMPC_BUNDLE_PATH)
  message(FATAL_ERROR
    "NMPC_BUNDLE_PATH must be set to the location of libnmpc_bundle.a")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node src/c_nmpc.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_wrapper/include
)

target_link_libraries(${PROJECT_NAME}_node
  ${NMPC_BUNDLE_PATH}
  Eigen3::Eigen
  fmt::fmt
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# launch files live in the bringup package

ament_auto_package()
