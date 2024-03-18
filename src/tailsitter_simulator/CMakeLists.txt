cmake_minimum_required(VERSION 3.8)
project(tailsitter_simulator)


set(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

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


ament_auto_add_executable(test_zhang src/MatlabGenerated/testZhang/main.cpp)
target_link_libraries(
  test_zhang
  ${PROJECT_NAME}_aerodynamics_lib
  rt_nonfinite_matlab
)
#ament_auto_add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS}) # Add other source files as needed
ament_auto_add_library(${PROJECT_NAME}_lib src/dynamics/Tailsitter.cpp)
target_link_libraries(${PROJECT_NAME}_lib 
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  ${PROJECT_NAME}_aerodynamics_lib
)

## Declare a cpp executable

ament_auto_add_executable(${PROJECT_NAME}_interface src/sim_interface.cpp) # Add other source files as needed

#add_executable(odom_visualization src/odom_visualization.cpp)
# add_executable(quadrotor_simulator_so3
#         src/quadrotor_simulator_so3.cpp)


# target_link_libraries(quadrotor_simulator_so3
#         ${catkin_LIBRARIES}
#         quadrotor_dynamics
# )


ament_auto_add_executable(${PROJECT_NAME}_test_node src/test_dynamics/test_dynamics.cpp) # Add other source files as needed
target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME}_lib)



ament_auto_add_executable(${PROJECT_NAME}_hover src/hover_example.cpp) # Add other source files as needed
# target_link_libraries(${PROJECT_NAME}_hover ${PROJECT_NAME}_lib)
ament_auto_add_executable(zero_copy_sub src/zero_copy.cpp) # Add other source files as needed


install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}

)

# target_link_libraries(test_quad
#         ${catkin_LIBRARIES}
#         quadrotor_dynamics
# )

ament_auto_package()