find_package(Boost COMPONENTS unit_test_framework)
find_package(Armadillo)
find_package(Eigen3)

if (Boost_unit_test_framework_FOUND)

  add_executable(test
    main.cpp
    bspline/support/Grid_test.cpp
    bspline/support/Support_test.cpp
    bspline/Spline_test.cpp
    bspline/operators/GenericOperators_test.cpp
    bspline/operators/ScalarOperators_test.cpp
    bspline/operators/DerivativeAndPosition_test.cpp
  )
  
  target_compile_definitions(test PUBLIC 
    BSPLINE_ADD_TEST_CHECKS
  )

  target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(test 
       ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
       main_library
  )
  add_test(test test)

  if (ARMADILLO_FOUND OR Eigen3_FOUND)
    #Add spline-interpolation test
    target_sources(test PRIVATE
      bspline/interpolation/interpolation-test.cpp
    )

    if (ARMADILLO_FOUND)                                     
      target_link_libraries(test ${ARMADILLO_LIBRARIES})
      target_compile_definitions(test PUBLIC 
             BSPLINE_INTERPOLATION_USE_ARMADILLO
      )
    endif (ARMADILLO_FOUND)
  
    if (Eigen3_FOUND)
      target_link_libraries(test
          Eigen3::Eigen
          examples_objects
      )

      target_compile_definitions(test PUBLIC 
             BSPLINE_INTERPOLATION_USE_EIGEN
      )

      target_sources(test PRIVATE
          example-tests.cpp
      )
    endif (Eigen3_FOUND)
  endif (ARMADILLO_FOUND OR Eigen3_FOUND)
  
  target_compile_options(test PRIVATE -Wno-deprecated-declarations)

  enable_testing()
  
  add_custom_target(run-test
    COMMAND
      ${CMAKE_CURRENT_BINARY_DIR}/test --detect_memory_leaks -p
    DEPENDS test
  )
else()
  message(WARNING "Boost test framework not found. Not building unit tests")
endif (Boost_unit_test_framework_FOUND)
