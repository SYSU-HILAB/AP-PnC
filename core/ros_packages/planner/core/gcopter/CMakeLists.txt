# Vendored header-only GCOPTER/MINCO library, exposed as an INTERFACE target.
add_library(gcopter INTERFACE)
target_include_directories(gcopter INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
