cmake_minimum_required(VERSION 3.8)
project(aerodynamics)


set(CMAKE_BUILD_TYPE Debug)
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

# Testing for aerodynamics
ament_auto_add_executable(aero_comparison src/aero_comparison.cpp)
target_link_libraries(aero_comparison
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_non
  ${PROJECT_NAME}_advanced_lift_drag
  ${PROJECT_NAME}_bspline
)

ament_auto_package()

# # --------------------------------------------------------------------------- #
# gazebo plugins
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(AerodynamicsPlugin
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
# install target *_gazebo to ~/.gz/sim/plugins
install(
  FILES ${CMAKE_BINARY_DIR}/libAerodynamicsPlugin.so
  DESTINATION ~/.gz/sim/plugins/
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
