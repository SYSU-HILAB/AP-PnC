cmake_minimum_required(VERSION 3.8)
project(AerodynamicsPlugin)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(AerodynamicsPlugin SHARED
  AerodynamicsPlugin.cc
)

target_include_directories(AerodynamicsPlugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/simulations/aerodynamics/include
  ${CMAKE_SOURCE_DIR}/simulations/aerodynamics/src/MatlabGenerated/Zhang/include
)

target_link_libraries(AerodynamicsPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(
  TARGETS AerodynamicsPlugin
  DESTINATION ~/.gz/sim/plugins/
)
