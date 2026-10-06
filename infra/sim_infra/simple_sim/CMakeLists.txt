cmake_minimum_required(VERSION 3.16)
project(simple_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(AP_PNC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
file(RELATIVE_PATH BUILD_RELATIVE "${AP_PNC_DIR}/.artifacts" "${CMAKE_BINARY_DIR}")
if(BUILD_RELATIVE MATCHES "^\\.\\." OR IS_ABSOLUTE "${BUILD_RELATIVE}")
  message(FATAL_ERROR "Build output must live under ${AP_PNC_DIR}/.artifacts")
endif()
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${AP_PNC_DIR}/.artifacts/colcon/install" CACHE PATH "Install location" FORCE)
endif()
include(CTest)
option(SIMPLE_SIM_BUILD_APP "Build planner/controller adapters and headless executable" ON)
set(SIMPLE_SIM_ROS_DEFAULT OFF)
if(DEFINED ENV{AMENT_PREFIX_PATH})
  set(SIMPLE_SIM_ROS_DEFAULT ON)
endif()
option(SIMPLE_SIM_WITH_ROS "Build optional ROS observation/control adapter" ${SIMPLE_SIM_ROS_DEFAULT})
option(SIMPLE_SIM_WITH_NMPC "Link Linux acados static bundle" ON)
if(SIMPLE_SIM_WITH_ROS AND NOT SIMPLE_SIM_BUILD_APP)
  message(FATAL_ERROR "ROS adapter requires SIMPLE_SIM_BUILD_APP")
endif()

find_package(Eigen3 QUIET)
if(NOT TARGET Eigen3::Eigen)
  # A cache argument also permits standalone testing with an existing header tree.
  find_path(EIGEN3_INCLUDE_DIR Eigen/Core PATH_SUFFIXES eigen3 REQUIRED)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

add_library(simple_sim_core STATIC
  core/src/config.cpp core/src/frames.cpp core/src/plant.cpp core/src/rate_loop.cpp
  core/src/runner.cpp core/src/motor_model.cpp core/src/throttle_model.cpp
)
target_include_directories(simple_sim_core PUBLIC core/include)
target_link_libraries(simple_sim_core PUBLIC Eigen3::Eigen)

if(SIMPLE_SIM_WITH_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
endif()

if(SIMPLE_SIM_BUILD_APP)
  find_package(yaml-cpp REQUIRED)
  if(TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
    add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
  endif()
  # Reuse owner sources directly: neither reference definitions nor aero algorithms live here.
  add_subdirectory(${AP_PNC_DIR}/core/ros_packages/planner/core
                   ${CMAKE_CURRENT_BINARY_DIR}/planner_core)
  set(AERO_DIR ${AP_PNC_DIR}/infra/sim_infra/aerodynamics)
  # All five canonical models (lyu | phi | ma | advanced | none) compile
  # in-tree, exactly like the aerodynamics package builds them; parameters
  # load from config/aero/<model>.yaml at runtime (AP_PNC_DIR).
  find_package(spdlog REQUIRED)
  add_library(simple_sim_aero STATIC
    ${AERO_DIR}/src/zhang_lyu_aerodynamics.cpp
    ${AERO_DIR}/src/phi_aerodynamics.cpp
    ${AERO_DIR}/src/bspline_aerodynamics.cpp
    ${AERO_DIR}/src/advanced_lift_drag.cpp
    ${AERO_DIR}/src/non_aerodynamics.cpp
    ${AERO_DIR}/src/aero_factory.cpp
    ${AERO_DIR}/src/MatlabGenerated/Zhang/Aerodynamics.cpp
    ${AERO_DIR}/src/MatlabGenerated/Zhang/rt_nonfinite.cpp
  )
  target_include_directories(simple_sim_aero PUBLIC ${AERO_DIR}/include
    PRIVATE ${AERO_DIR}/src/MatlabGenerated/Zhang/include)
  target_link_libraries(simple_sim_aero PUBLIC Eigen3::Eigen spdlog::spdlog yaml-cpp)

  add_library(simple_sim_adapters STATIC adapters/src/se3_controller.cpp adapters/src/aero_model.cpp)
  target_include_directories(simple_sim_adapters PUBLIC adapters/include)
  target_link_libraries(simple_sim_adapters PUBLIC simple_sim_core planner_core simple_sim_aero)
  set(NMPC_BUNDLE_SHA256 "disabled")
  if(SIMPLE_SIM_WITH_NMPC)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
      message(FATAL_ERROR "NMPC bundle is Linux-only. For core/baseline tests use -DSIMPLE_SIM_WITH_NMPC=OFF.")
    endif()
    set(NMPC_BUNDLE_PATH "${AP_PNC_DIR}/.artifacts/nmpc_solver/libnmpc_bundle.a" CACHE FILEPATH "NMPC static bundle")
    add_subdirectory(${AP_PNC_DIR}/core/ros_packages/nmpc/controller
                     ${CMAKE_CURRENT_BINARY_DIR}/nmpc_controller)
    file(SHA256 "${NMPC_BUNDLE_PATH}" NMPC_BUNDLE_SHA256)
    target_sources(simple_sim_adapters PRIVATE adapters/src/nmpc_controller.cpp)
    target_link_libraries(simple_sim_adapters PUBLIC nmpc_tracking_core m dl pthread)
  endif()

  execute_process(COMMAND git -C ${AP_PNC_DIR} rev-parse HEAD
    OUTPUT_VARIABLE BUILD_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND git -C ${AP_PNC_DIR} status --porcelain
    OUTPUT_VARIABLE BUILD_STATUS OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(BUILD_STATUS)
    set(BUILD_DIRTY true)
  else()
    set(BUILD_DIRTY false)
  endif()
  add_library(simple_sim_io STATIC io/src/config_loader.cpp io/src/experiment.cpp)
  target_include_directories(simple_sim_io PUBLIC io/include)
  target_link_libraries(simple_sim_io PUBLIC simple_sim_adapters yaml-cpp)
  target_compile_definitions(simple_sim_io PRIVATE
    AP_PNC_SOURCE_ROOT="${AP_PNC_DIR}"
    AP_PNC_BUILD_REVISION="${BUILD_REVISION}"
    AP_PNC_BUILD_DIRTY="${BUILD_DIRTY}"
    AP_PNC_BUILD_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    AP_PNC_BUILD_PLATFORM="${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}"
    AP_PNC_NMPC_BUNDLE_SHA256="${NMPC_BUNDLE_SHA256}"
  )
  if(SIMPLE_SIM_WITH_NMPC)
    target_compile_definitions(simple_sim_io PRIVATE SIMPLE_SIM_WITH_NMPC)
  endif()
  add_executable(simple_sim_run app/main.cpp)
  target_link_libraries(simple_sim_run PRIVATE simple_sim_io)
  install(TARGETS simple_sim_run RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_executable(simple_sim_throttle_model_test test/test_throttle_model.cpp)
  target_link_libraries(simple_sim_throttle_model_test PRIVATE simple_sim_core)
  add_test(NAME simple_sim_throttle_model COMMAND simple_sim_throttle_model_test)
  add_executable(simple_sim_motor_test test/test_motor.cpp)
  target_link_libraries(simple_sim_motor_test PRIVATE simple_sim_core)
  add_test(NAME simple_sim_motor COMMAND simple_sim_motor_test)
  add_executable(simple_sim_core_test test/test_core.cpp)
  target_link_libraries(simple_sim_core_test PRIVATE simple_sim_core Threads::Threads)
  add_test(NAME simple_sim_core COMMAND simple_sim_core_test)
  add_executable(simple_sim_nmpc_contract_test test/test_nmpc_contract.cpp
    ${AP_PNC_DIR}/core/ros_packages/nmpc/controller/src/tracking_controller.cpp)
  target_include_directories(simple_sim_nmpc_contract_test PRIVATE
    ${AP_PNC_DIR}/core/ros_packages/nmpc/controller/include
    ${AP_PNC_DIR}/core/ros_packages/nmpc/solver/include)
  add_test(NAME simple_sim_nmpc_contract COMMAND simple_sim_nmpc_contract_test)
  if(SIMPLE_SIM_BUILD_APP)
    add_executable(simple_sim_adapters_test test/test_adapters.cpp)
    target_link_libraries(simple_sim_adapters_test PRIVATE simple_sim_adapters)
    add_test(NAME simple_sim_adapters COMMAND simple_sim_adapters_test)
    if(SIMPLE_SIM_WITH_NMPC)
      add_executable(simple_sim_acceptance_test test/test_acceptance.cpp)
      target_link_libraries(simple_sim_acceptance_test PRIVATE simple_sim_io)
      add_test(NAME simple_sim_acceptance_v9 COMMAND simple_sim_acceptance_test)
      set_tests_properties(simple_sim_acceptance_v9 PROPERTIES TIMEOUT 150 LABELS integration)
    else()
      find_package(Python3 REQUIRED COMPONENTS Interpreter)
      add_test(NAME simple_sim_repeatability COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_repeatability.py $<TARGET_FILE:simple_sim_run>
        ${AP_PNC_DIR} se3)
      set_tests_properties(simple_sim_repeatability PROPERTIES TIMEOUT 150 LABELS integration)
    endif()
  endif()
endif()

if(SIMPLE_SIM_WITH_ROS)
  add_executable(simple_sim_node ros/src/sim_node.cpp)
  target_link_libraries(simple_sim_node simple_sim_io)
  ament_target_dependencies(simple_sim_node rclcpp nav_msgs geometry_msgs rosgraph_msgs sensor_msgs std_srvs)
  install(TARGETS simple_sim_node RUNTIME DESTINATION lib/${PROJECT_NAME})
  if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(ROS_TEST_CONTROLLER se3)
    if(SIMPLE_SIM_WITH_NMPC)
      set(ROS_TEST_CONTROLLER nmpc)
    endif()
    add_test(NAME simple_sim_ros_smoke COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_smoke.py $<TARGET_FILE:simple_sim_node>
      ${AP_PNC_DIR} ${ROS_TEST_CONTROLLER})
    set_tests_properties(simple_sim_ros_smoke PROPERTIES TIMEOUT 60 LABELS integration)
  endif()
  # Runtime resources are installed by bringup, never this package.
  ament_package()
endif()
