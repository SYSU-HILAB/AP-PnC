# Shared ROS-free tracking logic. libnmpc_bundle.a owns the generated solver.
if(NOT EXISTS "${NMPC_BUNDLE_PATH}")
  message(FATAL_ERROR "NMPC bundle missing: ${NMPC_BUNDLE_PATH}. Run ap-pnc gen-nmpc-lib on Linux.")
endif()
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The NMPC bundle is Linux-only")
endif()
execute_process(COMMAND ${CMAKE_NM} --defined-only ${NMPC_BUNDLE_PATH}
  RESULT_VARIABLE NMPC_NM_RESULT OUTPUT_VARIABLE NMPC_SYMBOLS ERROR_QUIET)
string(FIND "${NMPC_SYMBOLS}" " nmpc_solver_stage_dt" NMPC_GRID_SYMBOL)
if(NOT NMPC_NM_RESULT EQUAL 0 OR NMPC_GRID_SYMBOL EQUAL -1)
  message(FATAL_ERROR "NMPC bundle has an old wrapper API; run ap-pnc gen-nmpc-lib on Linux")
endif()
add_library(nmpc_tracking_core STATIC src/tracking_controller.cpp)
target_compile_features(nmpc_tracking_core PUBLIC cxx_std_17)
target_include_directories(nmpc_tracking_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(nmpc_tracking_core PUBLIC ${NMPC_BUNDLE_PATH})
set_target_properties(nmpc_tracking_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
