cmake_minimum_required(VERSION 3.8)
project(aerodynamics)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
endif()
include_directories(${EIGEN3_INCLUDE_DIR})
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
ament_auto_add_library(${PROJECT_NAME}_lyu
  src/zhang_lyu_aerodynamics.cpp
  src/MatlabGenerated/Zhang/Aerodynamics.cpp
  src/MatlabGenerated/Zhang/rt_nonfinite.cpp
)
target_include_directories(${PROJECT_NAME}_lyu
  PUBLIC
  src/MatlabGenerated/Zhang/include
)

# Advanced Lift Drag aerodynamics ("advanced")
ament_auto_add_library(${PROJECT_NAME}_advanced_lift_drag
    src/advanced_lift_drag.cpp
)
target_link_libraries(${PROJECT_NAME}_advanced_lift_drag
  yaml-cpp
)
# Phi aerodynamics ("phi")
ament_auto_add_library(${PROJECT_NAME}_phi src/phi_aerodynamics.cpp)
target_link_libraries(${PROJECT_NAME}_phi yaml-cpp)
# B-spline aerodynamics ("ma")
ament_auto_add_library(${PROJECT_NAME}_bspline src/bspline_aerodynamics.cpp)
target_link_libraries(${PROJECT_NAME}_bspline yaml-cpp)
# Non aerodynamics ("none")
ament_auto_add_library(${PROJECT_NAME}_non src/non_aerodynamics.cpp)

# Shared factory: the single definition of canonical model names
# (lyu | phi | ma | advanced | none); every simulator selects through it.
ament_auto_add_library(${PROJECT_NAME}_aero_factory src/aero_factory.cpp)
target_link_libraries(${PROJECT_NAME}_aero_factory
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_bspline
  ${PROJECT_NAME}_advanced_lift_drag
  ${PROJECT_NAME}_non
)

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
  ${PROJECT_NAME}_aero_factory
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_non
  ${PROJECT_NAME}_advanced_lift_drag
  ${PROJECT_NAME}_bspline
)

# Generate ML training data from all aerodynamic types
ament_auto_add_executable(generate_aero_ml_data src/generate_aero_ml_data.cpp)
target_link_libraries(generate_aero_ml_data
  ${PROJECT_NAME}_aero_factory
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

  ament_add_gtest(test_bspline_basis
    test/test_bspline_basis.cpp
  )
  target_link_libraries(test_bspline_basis
    ${PROJECT_NAME}_bspline
  )
endif()

# Plain targets without the ${PROJECT_NAME}_ prefix are not auto-installed;
# export them so consumers link against the install tree only.
install(TARGETS rt_nonfinite_matlab ${PROJECT_NAME}_aerodynamics_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
