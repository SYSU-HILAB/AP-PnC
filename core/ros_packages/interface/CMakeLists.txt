cmake_minimum_required(VERSION 3.8)
project(interface)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_add_executable(thrust_control_pid src/at_control.cpp)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Num.msg"
#   "srv/AddThreeInts.srv"
#  )
add_compile_options(
        -Wno-dev
)
set(MSGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/msg)
message(WARNING ${MSGS_DIR})
set(MSGS "" CACHE STRING "everytime update" FORCE)
file(GLOB MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${MSGS_DIR}/*.msg)

message(WARNING ${MSGS})

# set(MSGS_DIR "ASDLFKJ")


rosidl_generate_interfaces(
        ${PROJECT_NAME} ${MSGS}
        DEPENDENCIES std_msgs geometry_msgs
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
