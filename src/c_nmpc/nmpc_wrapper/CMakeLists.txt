cmake_minimum_required(VERSION 3.10)
project(nmpc_wrapper C CXX)

set(AP_PNC_DIR  $ENV{AP_PNC_DIR}  CACHE PATH "AP-PnC git root")
set(ACADOS_ROOT $ENV{ACADOS_SOURCE_DIR} CACHE PATH "acados source root")

file(GLOB_RECURSE SOLVER_SRCS
  ${AP_PNC_DIR}/c_generated_code/*.c
)

add_library(nmpc_solver STATIC
  src/nmpc_solver.cpp
  ${SOLVER_SRCS}
)

target_include_directories(nmpc_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(nmpc_solver PRIVATE
  ${AP_PNC_DIR}/c_generated_code
  ${ACADOS_ROOT}/include
  ${ACADOS_ROOT}/include/acados
  ${ACADOS_ROOT}/include/blasfeo/include
  ${ACADOS_ROOT}/include/hpipm/include
)

set_target_properties(nmpc_solver PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  C_STANDARD 99
  CXX_STANDARD 17
)
