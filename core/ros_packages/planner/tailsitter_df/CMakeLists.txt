cmake_minimum_required(VERSION 3.8)
project(tailsitter_df)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(HOME $ENV{HOME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(tailsitter_df  SHARED  src/tailsitter_df.cpp)

target_link_libraries(tailsitter_df
  Eigen3::Eigen

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
  # uncrustify is skipped too: the package follows the repo .clang-format
  # style, and the vendored autodiff headers are not ament-uncrustify clean.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tailsitter_df
    test/test_tailsitter_df.cpp
  )
  target_link_libraries(test_tailsitter_df
    tailsitter_df
  )
endif()

# install(DIRECTORY
#     config
#     DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
ament_auto_package()
