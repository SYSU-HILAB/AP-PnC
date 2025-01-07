cmake_minimum_required(VERSION 3.8)
project(utils)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SQLite3 REQUIRED)

# Add include directories
include_directories(include)

# Create an interface library for common dependencies
add_library(${PROJECT_NAME}_common INTERFACE)
target_link_libraries(${PROJECT_NAME}_common INTERFACE
  SQLite::SQLite3
  spdlog::spdlog
  fmt::fmt
  Eigen3::Eigen
)
target_include_directories(${PROJECT_NAME}_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Config reader library
add_library(${PROJECT_NAME}_config_reader
  src/sq_config_reader.cpp
)
target_include_directories(${PROJECT_NAME}_config_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_config_reader
  ${PROJECT_NAME}_common
)

add_library(${PROJECT_NAME}_config_reader_phi
  src/phi_aero_config_reader.cpp
)
target_include_directories(${PROJECT_NAME}_config_reader_phi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_config_reader_phi
  ${PROJECT_NAME}_config_reader
)

# Add BSpline aero config reader library
add_library(${PROJECT_NAME}_config_reader_bspline
  src/bspline_aero_config_reader.cpp
)
target_include_directories(${PROJECT_NAME}_config_reader_bspline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_config_reader_bspline
  ${PROJECT_NAME}_config_reader
)

# Add Advanced Lift Drag config reader library
add_library(${PROJECT_NAME}_config_reader_advanced_lift_drag
  src/advanced_lift_drag_config_reader.cpp
)
target_include_directories(${PROJECT_NAME}_config_reader_advanced_lift_drag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_config_reader_advanced_lift_drag
  ${PROJECT_NAME}_config_reader
)

# Testing for config reader
# add_executable(test_reader src/test_reader.cpp)
# target_include_directories(test_reader PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )
# target_link_libraries(test_reader
#   ${PROJECT_NAME}_config_reader_phi
#   ${PROJECT_NAME}_config_reader_bspline
#   ${PROJECT_NAME}_config_reader_advanced_lift_drag
# )

ament_export_targets(${PROJECT_NAME}_common_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(SQLite3 spdlog fmt Eigen3)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_config_reader
    ${PROJECT_NAME}_config_reader_phi
    ${PROJECT_NAME}_config_reader_bspline
    ${PROJECT_NAME}_config_reader_advanced_lift_drag
  EXPORT ${PROJECT_NAME}_common_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()


