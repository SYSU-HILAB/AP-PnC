cmake_minimum_required(VERSION 3.8)
project(c_nmpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

set(MODEL "tailsitter_flu" CACHE STRING "The model_name of NMPC")
set(TAILSITTER_PLANNING_DIR $ENV{TAILSITTER_PLANNING_DIR} CACHE PATH "The root path of pnc")
set(ACADOS_ROOT  $ENV{ACADOS_SOURCE_DIR} CACHE PATH "The root path of acados")

include_directories(${ACADOS_ROOT}/include)
include_directories(${ACADOS_ROOT}/include/blasfeo/include)
include_directories(${ACADOS_ROOT}/include/hpipm/include)
include_directories(${ACADOS_ROOT}/include/acados)
include_directories(${TAILSITTER_PLANNING_DIR}/c_generated_code)
link_directories(${ACADOS_ROOT}/lib)
link_directories(${TAILSITTER_PLANNING_DIR}/c_generated_code)



# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node src/c_nmpc.cpp) # Add other source files as needed

target_link_libraries(
        ${PROJECT_NAME}_node
        Eigen3::Eigen  # Eigen support namespace CMake
        acados_ocp_solver_${MODEL}
        acados
        hpipm
        blasfeo
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

# install(DIRECTORY
#         launch config meshes
#         DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package()
