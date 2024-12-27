cmake_minimum_required(VERSION 3.8)
project(aerodynamics)


set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Phi aerodynamics
ament_auto_add_library(${PROJECT_NAME}_phi src/phi_aerodynamics.cpp) # Add other source files as needed
target_link_libraries(${PROJECT_NAME}_phi Eigen3::Eigen)

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

# Phi aerodynamics
ament_auto_add_library(${PROJECT_NAME}_lyu src/zhang_lyu_aerodynamics.cpp) # Add other source files as needed
target_link_libraries(${PROJECT_NAME}_lyu Eigen3::Eigen ${PROJECT_NAME}_aerodynamics_lib)


ament_auto_add_library(${PROJECT_NAME}_bspline src/bspline_aerodynamics.cpp) # Add other source files as needed
target_link_libraries(${PROJECT_NAME}_bspline Eigen3::Eigen)

# Non aerodynamics
ament_auto_add_library(${PROJECT_NAME}_non src/non_aerodynamics.cpp)
target_link_libraries(${PROJECT_NAME}_non Eigen3::Eigen)

# Testing for aerodynamics
ament_auto_add_executable(aerodynamics_node src/weird_test.cpp)
target_link_libraries(aerodynamics_node 
  ${PROJECT_NAME}_phi
  ${PROJECT_NAME}_lyu
  ${PROJECT_NAME}_non
)




ament_auto_package()