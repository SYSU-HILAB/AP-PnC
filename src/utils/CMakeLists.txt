cmake_minimum_required(VERSION 3.8)
project(utils)


# set(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Config reader library
ament_auto_add_library(${PROJECT_NAME}_config_reader
  src/sq_config_reader.cpp

)

ament_auto_add_library(${PROJECT_NAME}_config_reader_phi
  src/phi_aero_config_reader.cpp
)
target_link_libraries(${PROJECT_NAME}_config_reader_phi
  ${PROJECT_NAME}_config_reader
)

# Add BSpline aero config reader library
ament_auto_add_library(${PROJECT_NAME}_config_reader_bspline
  src/bspline_aero_config_reader.cpp
)
target_link_libraries(${PROJECT_NAME}_config_reader_bspline
  ${PROJECT_NAME}_config_reader
)


target_include_directories(${PROJECT_NAME}_config_reader
  PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}_config_reader
  SQLite::SQLite3
)



# Testing for config reader
ament_auto_add_executable(test_reader src/test_reader.cpp)
target_link_libraries(test_reader
  ${PROJECT_NAME}_config_reader_phi
  ${PROJECT_NAME}_config_reader_bspline
) 

# install(
#   DIRECTORY launch config 
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package()