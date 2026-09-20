cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller/controller.cpp
  src/controller/flatness_math.cpp
  src/controller/pid_position.cpp
  src/controller/attitude_feedback.cpp
  src/controller/thrust_limiter.cpp
  src/controller/yaw_target.cpp
  src/controller/throttle_manager.cpp
  src/controller/alg0_differential_flatness.cpp
  src/controller/alg1_geometric.cpp
  src/controller/alg2_rotor_drag.cpp
  src/input.cpp
)

target_include_directories(px4ctrl_node PRIVATE src include)
target_link_libraries(px4ctrl_node Eigen3::Eigen)
ament_target_dependencies(px4ctrl_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  std_srvs
  quadrotor_msgs
  interface
)

install(TARGETS px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_add_gtest(px4ctrl_yaw_contract_test
  test/yaw_contract_test.cpp
  src/controller/controller.cpp
  src/controller/flatness_math.cpp
  src/controller/pid_position.cpp
  src/controller/attitude_feedback.cpp
  src/controller/thrust_limiter.cpp
  src/controller/yaw_target.cpp
  src/controller/throttle_manager.cpp
  src/controller/alg0_differential_flatness.cpp
  src/controller/alg1_geometric.cpp
  src/controller/alg2_rotor_drag.cpp
  src/input.cpp
  src/PX4CtrlParam.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
if(TARGET px4ctrl_yaw_contract_test)
  target_include_directories(px4ctrl_yaw_contract_test PRIVATE src include)
  target_link_libraries(px4ctrl_yaw_contract_test Eigen3::Eigen)
  ament_target_dependencies(px4ctrl_yaw_contract_test
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    mavros_msgs
    quadrotor_msgs
    interface
  )
endif()

ament_add_gtest(px4ctrl_throttle_model_test
  test/throttle_model_test.cpp
)
if(TARGET px4ctrl_throttle_model_test)
  target_include_directories(px4ctrl_throttle_model_test PRIVATE include)
endif()

ament_package()
