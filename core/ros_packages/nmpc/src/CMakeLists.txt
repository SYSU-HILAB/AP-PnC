cmake_minimum_required(VERSION 3.10)
project(nmpc_solver C CXX)

set(AP_PNC_DIR  $ENV{AP_PNC_DIR}  CACHE PATH "AP-PnC git root")
set(ACADOS_ROOT $ENV{ACADOS_SOURCE_DIR} CACHE PATH "acados source root")
set(GEN_DIR ${AP_PNC_DIR}/.artifacts/c_generated_code)

file(GLOB_RECURSE SOLVER_SRCS
  ${GEN_DIR}/*.c
)

add_library(nmpc_solver STATIC
  nmpc_solver.cpp
  ${SOLVER_SRCS}
)

target_include_directories(nmpc_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(nmpc_solver PRIVATE
  ${GEN_DIR}
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
