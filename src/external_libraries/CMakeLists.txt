cmake_minimum_required(VERSION 3.8)
project(external_libraries)

# Find dependencies
find_package(ament_cmake REQUIRED)
add_subdirectory(tomlplusplus)
add_subdirectory(yamlcpp)

install(TARGETS yaml-cpp
    EXPORT yamlcpp-targets
    DESTINATION lib
)

# install(TARGETS tomlplusplus
#     EXPORT tomlplusplus-targets
#     DESTINATION lib
# )

ament_export_targets(yamlcpp-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

# ament_export_targets(tomlplusplus HAS_LIBRARY_TARGET)
# ament_export_dependencies(tomlplusplus)

# some bugs in tomlplusplus
install(DIRECTORY ${tomlplusplus_SOURCE_DIR}/include/
  DESTINATION include
)

ament_package()
