# Tailsitter differential-flatness model (vendored autodiff headers included).
# Plain CMake subdirectory of the `planner` package -- no ament_package().
find_package(Eigen3 REQUIRED)

add_library(tailsitter_df STATIC src/tailsitter_df.cpp)
target_include_directories(tailsitter_df PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(tailsitter_df Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tailsitter_df test/test_tailsitter_df.cpp)
  target_link_libraries(test_tailsitter_df tailsitter_df)
endif()
