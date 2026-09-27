# Aerodynamics configuration readers (formerly the standalone `utils` package).
# CMake subdirectory of the `aerodynamics` package -- no project()/ament_package().
# Static so the shared aero model libs embed them (no extra runtime .so).

add_library(aerodynamics_config_common INTERFACE)
target_link_libraries(aerodynamics_config_common INTERFACE
  SQLite::SQLite3
  spdlog::spdlog
  fmt::fmt
  Eigen3::Eigen
)
target_include_directories(aerodynamics_config_common INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(aerodynamics_config_reader STATIC src/sq_config_reader.cpp)
target_link_libraries(aerodynamics_config_reader aerodynamics_config_common)

add_library(aerodynamics_config_reader_phi STATIC src/phi_aero_config_reader.cpp)
target_link_libraries(aerodynamics_config_reader_phi aerodynamics_config_reader)

add_library(aerodynamics_config_reader_bspline STATIC src/bspline_aero_config_reader.cpp)
target_link_libraries(aerodynamics_config_reader_bspline aerodynamics_config_reader)

add_library(aerodynamics_config_reader_advanced_lift_drag STATIC
  src/advanced_lift_drag_config_reader.cpp)
target_link_libraries(aerodynamics_config_reader_advanced_lift_drag
  aerodynamics_config_reader)
