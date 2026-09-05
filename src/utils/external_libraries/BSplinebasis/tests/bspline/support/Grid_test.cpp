/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#include <bspline/support/Support.h>

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <type_traits>

static_assert(!std::is_move_constructible_v<bspline::support::Grid<double>> &&
                  !std::is_move_assignable_v<bspline::support::Grid<double>>,
              "Grid is movable.");

extern const std::vector<double> DEFAULT_GRID_DATA;
const std::vector<double> DEFAULT_GRID_DATA{
    -7.0l, -6.85l, -6.55l, -6.3l, -6.0l, -5.75l, -5.53l, -5.2l, -4.75l, -4.5l,
    -3.0l, -2.5l,  -1.5l,  -1.0l, 0.0l,  0.5l,   1.5l,   2.5l,  3.5l,   4.0l,
    4.35l, 4.55l,  4.95l,  5.4l,  5.7l,  6.1l,   6.35l,  6.5l,  6.85l,  7.0l};

BOOST_AUTO_TEST_SUITE(GridTestSuite)

/*!
 * Passes if the constructor of the Grid throws if we try to construct an empty
 * grid.
 */
BOOST_AUTO_TEST_CASE(ConstructEmptyGridThrows) {
  using Grid = bspline::support::Grid<double>;
  using BSplineException = bspline::exceptions::BSplineException;

  BOOST_REQUIRE_THROW(Grid grid({}), BSplineException);
  BOOST_REQUIRE_THROW(Grid grid(std::vector<double>{}), BSplineException);
  BOOST_REQUIRE_THROW(Grid grid(nullptr), BSplineException);
  BOOST_REQUIRE_THROW(
      Grid grid(DEFAULT_GRID_DATA.begin(), DEFAULT_GRID_DATA.begin()),
      BSplineException);
}

/*!
 * Passes if the constructor of the Grid throws if we try to construct a Grid
 * with only one element.
 */
BOOST_AUTO_TEST_CASE(ConstructGridWithOneElementThrows) {
  using Grid = bspline::support::Grid<double>;
  using BSplineException = bspline::exceptions::BSplineException;

  BOOST_REQUIRE_THROW(Grid grid({3.0}), BSplineException);
  BOOST_REQUIRE_THROW(Grid grid(std::vector<double>{3.0}), BSplineException);
  BOOST_REQUIRE_THROW(
      Grid grid(DEFAULT_GRID_DATA.begin(), DEFAULT_GRID_DATA.begin() + 1),
      BSplineException);
}

/*!
 * Passes if the constructor of the Grid throws if we try to construct a grid
 * whose grid points are not steadily increasing.
 */
BOOST_AUTO_TEST_CASE(ConstructNotSteadilyIncreasingThrows) {
  using Grid = bspline::support::Grid<double>;
  using BSplineException = bspline::exceptions::BSplineException;

  BOOST_REQUIRE_THROW(Grid grid({1.0, 2.0, 3.0, 3.0, 4.0, 5.0}),
                      BSplineException);
  BOOST_REQUIRE_THROW(
      Grid grid(std::vector<double>{-3.0, -2.0, 0.0, -1.0, 2.0, 3.0}),
      BSplineException);
}

/*!
 * Passes if the grid can be copied.
 */
BOOST_AUTO_TEST_CASE(CopyGrid) {  // A Grid cannot be moved, only copied.
  using Grid = bspline::support::Grid<double>;

  Grid grid1(DEFAULT_GRID_DATA);
  auto grid2 = grid1;
  BOOST_TEST((grid1.getData() != nullptr && grid2.getData() != nullptr));
  BOOST_TEST(grid1 == grid2);
}

/*!
 * Passes if the Grid can be iterated over using a range-based for loop.
 */
BOOST_AUTO_TEST_CASE(Iteration) {
  using Grid = bspline::support::Grid<double>;

  const Grid grid1(DEFAULT_GRID_DATA);

  std::vector<double> gridData2;
  for (const auto &val : grid1) {
    gridData2.push_back(val);
  }
  BOOST_TEST(DEFAULT_GRID_DATA == gridData2);
  std::vector<double> gridData3{grid1.begin(), grid1.end()};
  BOOST_TEST(DEFAULT_GRID_DATA == gridData3);
}

/*!
 * Passes if the (in)equality operators correctly compare grids.
 */
BOOST_AUTO_TEST_CASE(Equality) {
  using Grid = bspline::support::Grid<double>;

  const Grid grid(DEFAULT_GRID_DATA);
  const Grid gridCopy = grid;
  const Grid gridEquivalent(DEFAULT_GRID_DATA);
  auto alteredGridData = DEFAULT_GRID_DATA;
  alteredGridData[alteredGridData.size() / 2] += 1.0e-4;
  const Grid gridInequivalent(alteredGridData);

  BOOST_TEST(grid == gridCopy);
  BOOST_TEST(!(grid != gridCopy));

  BOOST_TEST(grid == gridEquivalent);
  BOOST_TEST(!(grid != gridEquivalent));

  BOOST_TEST(!(grid == gridInequivalent));
  BOOST_TEST(grid != gridInequivalent);
}

/*!
 * Passes if the method at() behaves as expected, giving access to existant
 * elements and throwing exception in the case of an attempted access to
 * non-existant elements.
 */
BOOST_AUTO_TEST_CASE(At) {
  using Grid = bspline::support::Grid<double>;
  using BSplineException = bspline::exceptions::BSplineException;

  const Grid grid(DEFAULT_GRID_DATA);

  const std::vector<size_t> validIndices{0, DEFAULT_GRID_DATA.size() / 2,
                                         DEFAULT_GRID_DATA.size() - 1};
  for (size_t validIndex : validIndices) {
    BOOST_REQUIRE_NO_THROW(grid.at(validIndex));
    BOOST_TEST(grid.at(validIndex) == DEFAULT_GRID_DATA.at(validIndex));
  }

  BOOST_REQUIRE_THROW(grid.at(DEFAULT_GRID_DATA.size()), BSplineException);
  BOOST_REQUIRE_THROW(grid.at(DEFAULT_GRID_DATA.size() + 5), BSplineException);
  BOOST_REQUIRE_THROW(grid.at(2 * DEFAULT_GRID_DATA.size()), BSplineException);
}

/*!
 * Passes if front() and back() return the expected elements.
 */
BOOST_AUTO_TEST_CASE(FrontAndBack) {
  using Grid = bspline::support::Grid<double>;

  const Grid grid(DEFAULT_GRID_DATA);

  BOOST_TEST(grid.front() == DEFAULT_GRID_DATA.front());
  BOOST_TEST(grid.back() == DEFAULT_GRID_DATA.back());
}

/*!
 * Passes if the method findElement() correctly returns the index of an element
 * in the Grid and throws if the requested element could not be found.
 */
BOOST_AUTO_TEST_CASE(FindElement) {
  using Grid = bspline::support::Grid<double>;
  using BSplineException = bspline::exceptions::BSplineException;

  const Grid grid(DEFAULT_GRID_DATA);

  BOOST_TEST(grid.size() == DEFAULT_GRID_DATA.size());

  for (size_t i = 0; i < DEFAULT_GRID_DATA.size(); i++) {
    const auto index = grid.findElement(DEFAULT_GRID_DATA.at(i));
    BOOST_TEST(index == i);
  }

  for (size_t i = 0; i < DEFAULT_GRID_DATA.size(); i++) {
    // These elements could not be found in the grid.
    BOOST_REQUIRE_THROW(grid.findElement(DEFAULT_GRID_DATA.at(i) + 1.0e-4),
                        BSplineException);
  }

  const double tooSmall = DEFAULT_GRID_DATA.front() - 1.0e-4;
  BOOST_REQUIRE_THROW(grid.findElement(tooSmall), BSplineException);
}

BOOST_AUTO_TEST_SUITE_END()
