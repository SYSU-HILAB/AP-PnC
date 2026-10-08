# Tailsitter differential-flatness map.
# Header-only: ENU/FLU analytic flatness (flu_reference.hpp values plus the
# hand-derived Jacobian in flu_flatness.hpp). The vendored autodiff kernel was
# removed; there is no autodiff dependency and no code generation here.
find_package(Eigen3 REQUIRED)

add_library(tailsitter_df INTERFACE)
target_include_directories(tailsitter_df INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tailsitter_df INTERFACE Eigen3::Eigen)
