# Basic seed trajectories (lemniscate/circle/sin/line/vertical loop).
# Plain CMake subdirectory of the `planner` package -- no ament_package().
find_package(Eigen3 REQUIRED)

add_library(basic_trajectory STATIC src/trajectory.cpp)
target_include_directories(basic_trajectory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(basic_trajectory Eigen3::Eigen)

add_library(lemniscate STATIC src/lemniscate.cpp)
target_include_directories(lemniscate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(lemniscate Eigen3::Eigen basic_trajectory)

add_library(circle STATIC src/circle.cpp)
target_include_directories(circle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(circle Eigen3::Eigen basic_trajectory)

add_library(sin STATIC src/sin.cpp)
target_include_directories(sin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sin Eigen3::Eigen basic_trajectory)

add_library(line STATIC src/line.cpp)
target_include_directories(line PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(line Eigen3::Eigen basic_trajectory)

add_library(verloop STATIC src/vertical_loop.cpp)
target_include_directories(verloop PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(verloop Eigen3::Eigen basic_trajectory)
