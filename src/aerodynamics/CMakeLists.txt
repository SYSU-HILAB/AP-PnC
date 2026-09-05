cmake_minimum_required(VERSION 3.8)
project(aerodynamics)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Zhang Lyu aerodynamics
add_library(
  rt_nonfinite_matlab
  STATIC
  src/MatlabGenerated/Zhang/rt_nonfinite.cpp
)

add_library(
  ${PROJECT_NAME}_aerodynamics_lib
  STATIC
  src/MatlabGenerated/Zhang/Aerodynamics.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_aerodynamics_lib
  rt_nonfinite_matlab
)

target_include_directories(
  ${PROJECT_NAME}_aerodynamics_lib
  PUBLIC
  src/MatlabGenerated/Zhang/include
)

# Lyu aerodynamics
ament_auto_add_library(${PROJECT_NAME}_lyu src/zhang_lyu_aerodynamics.cpp)
target_link_libraries(${PROJECT_NAME}_lyu
    ${PROJECT_NAME}_aerodynamics_lib
)

# Advanced Lift Drag aerodynamics
ament_auto_add_library(${PROJECT_NAME}_advanced_lift_drag
    src/advanced_lift_drag.cpp
)
# Phi aerodynamics
ament_auto_add_library(${PROJECT_NAME}_phi src/phi_aerodynamics.cpp)
# B-spline aerodynamics
ament_auto_add_library(${PROJECT_NAME}_bspline src/bspline_aerodynamics.cpp)
# Non aerodynamics
ament_auto_add_library(${PROJECT_NAME}_non src/non_aerodynamics.cpp)

# AeroAlphaSolver: Force balance equation solver using Newton-Raphson
ament_auto_add_library(${PROJECT_NAME}_alpha_solver
    src/aero_alpha_solver.cpp
)
target_link_libraries(${PROJECT_NAME}_alpha_solver
    yaml-cpp
    ${PROJECT_NAME}_lyu
    ${PROJECT_NAME}_aerodynamics_lib
)

# Testing for aerodynamics
ament_auto_add_executable(aero_comparison src/aero_comparison.cpp)
target_link_libraries(aero_comparison
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_non
  ${PROJECT_NAME}_advanced_lift_drag
  ${PROJECT_NAME}_bspline
)

# Generate ML training data from all aerodynamic types
ament_auto_add_executable(generate_aero_ml_data src/generate_aero_ml_data.cpp)
target_link_libraries(generate_aero_ml_data
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_advanced_lift_drag
  ${PROJECT_NAME}_bspline
)

# GTest for AeroAlphaSolver
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_aero_alpha_solver
    test/test_aero_alpha_solver.cpp
  )
  target_link_libraries(test_aero_alpha_solver
    ${PROJECT_NAME}_alpha_solver
  )
  ament_target_dependencies(test_aero_alpha_solver
    rclcpp
  )
endif()

ament_auto_package()

option(BUILD_GAZEBO_PLUGINS "Build Gazebo plugins" ON)

if(BUILD_GAZEBO_PLUGINS)
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  ament_auto_add_library(AerodynamicsPlugin
    SHARED
    src/gazebo_plugins/AerodynamicsPlugin.cc
  )

  target_link_libraries(AerodynamicsPlugin
    ${PROJECT_NAME}_phi
    ${PROJECT_NAME}_lyu
    ${PROJECT_NAME}_non
    ${PROJECT_NAME}_advanced_lift_drag
    ${PROJECT_NAME}_bspline
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  )

  install(
    FILES ${CMAKE_BINARY_DIR}/libAerodynamicsPlugin.so
    DESTINATION ~/.gz/sim/plugins/
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
endif()
