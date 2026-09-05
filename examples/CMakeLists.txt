find_package(Eigen3)

if (Eigen3_FOUND)
  add_library(examples_objects OBJECT
    harmonic-oscillator.cpp
    spline-potential.cpp
    hydrogen.cpp
    diffusion.cpp
  )

  target_compile_definitions(examples_objects PUBLIC 
      BSPLINE_INTERPOLATION_USE_EIGEN
      BSPLINE_ADD_TEST_CHECKS
  )

  target_link_libraries(examples_objects PUBLIC
      Eigen3::Eigen
      main_library
  )


  target_include_directories(examples_objects PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_executable(examples 
    main.cpp 
  )

  target_link_libraries(examples PUBLIC examples_objects)

  # Might be needed, if you want to use boost multiprecision types as data_t in the examples.
  # target_include_directories(examples_objects PUBLIC /usr/include/eigen3)
else()
  message(WARNING "Eigen3 framework not found. Not building examples.")
endif (Eigen3_FOUND)
