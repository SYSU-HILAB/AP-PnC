/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

/**
 * This files contains macros that may be used to enable checks for the validity
 * of objects if the macro BSPLINE_ADD_TEST_CHECKS is defined.
 */
#ifndef BSPLINE_TEST_CHECKS_H
#define BSPLINE_TEST_CHECKS_H

#ifndef BSPLINE_DOXYGEN_IGNORE

#ifdef BSPLINE_ADD_TEST_CHECKS

#define DURING_TEST_CHECK_VALIDITY() checkValidity()

#define DURING_TEST_CHECK_VALIDITY_OF(obj) obj.checkValidity()

#else
// Empty definitions of the same macros.

#define DURING_TEST_CHECK_VALIDITY() \
  do {                               \
  } while (false)

#define DURING_TEST_CHECK_VALIDITY_OF(obj) \
  do {                                     \
  } while (false)

#endif  // BSPLINE_ADD_TEST_CHECKS

#endif  // BSPLINE_DOXYGEN_IGNORE

#endif  // BSPLINE_TEST_CHECKS_H
