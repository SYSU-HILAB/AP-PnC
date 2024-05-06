cmake_minimum_required(VERSION 3.8)
project(tailsitter_df)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(HOME $ENV{HOME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(tailsitter_df  SHARED  src/df.cpp)
# ament_auto
# ament_target_dependencies(df)
list(APPEND CMAKE_PREFIX_PATH "${HOME}/miniconda3")
find_package(yaml-cpp REQUIRED)
find_package(autodiff REQUIRED)

target_link_libraries(tailsitter_df  Eigen3::Eigen autodiff::autodiff yaml-cpp)

ament_auto_add_executable(test_main  src/main.cpp)
target_link_libraries(test_main)
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

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

# ament_package()
ament_auto_package()
