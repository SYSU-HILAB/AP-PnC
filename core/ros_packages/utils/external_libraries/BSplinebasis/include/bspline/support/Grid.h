/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_SUPPORT_GRID_H
#define BSPLINE_SUPPORT_GRID_H

#include <bspline/exceptions/BSplineException.h>
#include <bspline/internal/misc.h>
#include <bspline/internal/test_checks.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace bspline::support {
using namespace bspline::exceptions;

/*!
 * @brief Represents a global Grid.
 * @tparam T The datatype of the Grid elements.
 */
template <typename T>
class Grid final {
 private:
  /*! The gridpoints. */
  std::shared_ptr<const std::vector<T>> _data;

  /*!
   * @brief Checks whether Grid points are steadily increasing.
   * @returns True if the Grid points are steadily increasing, false otherwise.
   */
  bool isSteadilyIncreasing() const {
    for (size_t i = 1; i < _data->size(); i++) {
      if ((*_data)[i - 1] >= (*_data)[i]) {
        return false;
      }
    }
    return true;
  }

  /*!
   * Checks the validity of this Grid and throws if the Grid is not in a
   * valid state.
   *
   * @brief Checks whether this Grid fulfills the classes invariants.
   * @throws BSplineException if this object is not in a valid state.
   */
  void checkValidity() const {
    // Check the validity of the provided data.
    if (!_data || _data->size() < 2) {
      throw BSplineException(ErrorCode::MISSING_DATA);
    } else if (!isSteadilyIncreasing()) {
      throw BSplineException(ErrorCode::INCONSISTENT_DATA,
                             "The grid points are not steadily increasing.");
    }
  }

 public:
  /*!
   * @brief Iterator type.
   */
  using const_iterator = typename std::vector<T>::const_iterator;

  Grid() = delete;

  /*!
   * @brief Constructs a Grid from a set of begin and end iterators.
   * @param begin The iterator referencing the first element to be copied into
   * the Grid.
   * @param end The iterator referencing the element behind the last element to
   * be copied into the Grid.
   * @tparam Iter The type of the two iterators.
   * @throws BSplineException If the Grid is empty or contains only a
   * single element, or if the elements are not in steadily increasing order.
   */
  template <typename Iter>
  Grid(Iter begin, Iter end)
      : Grid(std::make_shared<const std::vector<T>>(begin, end)) {}

  /*!
   * @brief Constructs a Grid from a std::vector.
   * @param v The input vector.
   * @throws BSplineException If the Grid is empty or contains only a
   * single element, or if the elements are not in steadily increasing order.
   */
  explicit Grid(std::vector<T> v)
      : Grid(std::make_shared<const std::vector<T>>(std::move(v))){};

  /*!
   * @brief Constructs a Grid from a std::initializer_list.
   * @param v The input initializer_list.
   * @throws BSplineException If the Grid is empty or contains only a
   * single element, or if the elements are not in steadily increasing order.
   */
  explicit Grid(const std::initializer_list<T> &v) : Grid(v.begin(), v.end()){};

  /*!
   * @brief Constructs a Grid from a std::shared_ptr<const std::vector<T>>.
   * @param data A shared pointer to the Grid elements.
   * @throws BSplineException If the Grid is empty or contains only a
   * single element, or if the elements are not in steadily increasing order.
   */
  explicit Grid(std::shared_ptr<const std::vector<T>> data)
      : _data(std::move(data)) {
    checkValidity();
  };

  /*!
   * @brief Default copy constructor.
   * @param g Grid to be copied.
   */
  Grid(const Grid &g) noexcept = default;

  /*!
   * @brief Default copy assignment operator.
   * @param g Grid to be copied.
   */
  Grid &operator=(const Grid &g) noexcept = default;

  /*!
   * @brief Default destructor.
   */
  ~Grid() = default;

  /*!
   * @brief Explicitly deleted move constructor.
   * @param g Grid to (not) be moved.
   */
  Grid(Grid &&g) = delete;

  /*!
   * @brief Explicitly deleted move assignment operator.
   * @param g Grid to (not) be moved.
   */
  Grid &operator=(Grid &&g) = delete;

  /*!
   * @brief Comparison operator.
   * @param g The Grid to compare this Grid with.
   * @returns Returns true if the Grids represent the same logical Grid.
   */
  bool operator==(const Grid &g) const {
    DURING_TEST_CHECK_VALIDITY();
    DURING_TEST_CHECK_VALIDITY_OF(g);
    if (_data == g._data) {
      // Fast path: Compare pointers.
      return true;
    }

    // Slow path: Compare logically.
    return (*_data) == (*g._data);
  }

  /*!
   * @brief Negated comparison operator.
   * @param g The Grid to compare this Grid with.
   * @returns Returns false if the Grids represent the same logical Grid.
   */
  bool operator!=(const Grid &g) const { return !(*this == g); }

  /*!
   * @brief Returns the number of elements of the Grid.
   * @returns The number of Grid points.
   */
  size_t size() const {
    DURING_TEST_CHECK_VALIDITY();
    return _data->size();
  };

  /*!
   * Returns a shared pointer to the elements of this Grid.
   *
   * @brief Gives access to the underlying data.
   * @returns A shared pointer to the elements of this Grid.
   */
  std::shared_ptr<const std::vector<T>> getData() const {
    DURING_TEST_CHECK_VALIDITY();
    return _data;
  };

  /*!
   * Checks whether the Grids holds any elements. Note: It is not possible to
   * construct an empty Grid, so this method will always return false.
   *
   * @brief Checks whether the spline is empty.
   * @returns True if this Grid holds no element.
   */
  bool empty() const {
    DURING_TEST_CHECK_VALIDITY();
    return _data->empty();
  };

  /*!
   * Returns a reference to the ith element of the Grid. Performs no bounds
   * checks.
   *
   * @brief Gives access to the i-th element of the Grid.
   * @param i The index of the element to be returned.
   * @returns A reference to the ith element.
   */
  const T &operator[](size_t i) const {
    DURING_TEST_CHECK_VALIDITY();
    return (*_data)[i];
  };

  /*!
   * Returns a reference to the ith element of the Grid. Checks the bounds.
   *
   * @brief Gives access to the i-th element of the Grid.
   * @param i The index of the element to be returned.
   * @returns A reference to the ith element.
   * @throws BSplineException If the access is out of bounds.
   */
  const T &at(size_t i) const {
    DURING_TEST_CHECK_VALIDITY();
    if (i >= size()) {
      throw BSplineException(ErrorCode::INVALID_ACCESS);
    }
    return (*_data)[i];
  };

  /*!
   * @brief Returns a reference to the first element of the Grid.
   * @returns A reference to the first element.
   * @throws BSplineException If the Grid is empty.
   */
  const T &front() const {
    DURING_TEST_CHECK_VALIDITY();
    if (empty()) {
      throw BSplineException(ErrorCode::INVALID_ACCESS);
    }
    return _data->front();
  };

  /*!
   * @brief Returns a reference to the last element of the Grid.
   * @returns A reference to the last element.
   * @throws BSplineException If the Grid is empty.
   */
  const T &back() const {
    DURING_TEST_CHECK_VALIDITY();
    if (empty()) {
      throw BSplineException(ErrorCode::INVALID_ACCESS);
    }
    return _data->back();
  };

  /*!
   * @brief Returns the begin iterator of the Grid.
   * @returns An iterator to the first element.
   */
  const_iterator begin() const {
    DURING_TEST_CHECK_VALIDITY();
    return _data->begin();
  };

  /*!
   * @brief Returns the end iterator of the Grid..
   * @returns An iterator pointing behind the last element.
   */
  const_iterator end() const {
    DURING_TEST_CHECK_VALIDITY();
    return _data->end();
  };

  /*!
   * @brief Returns the index corresponding to the element x.
   * @param x The element to be searched for.
   * @throws BSplineException If the element could not be found.
   */
  size_t findElement(const T &x) const {
    DURING_TEST_CHECK_VALIDITY();
    const auto beginIt = begin();
    const auto endIt = end();
    const auto elementIt = std::lower_bound(beginIt, endIt, x);

    if (elementIt == endIt || *elementIt != x) {
      // Element was not found
      throw BSplineException(ErrorCode::INCONSISTENT_DATA);
    } else {
      return std::distance(beginIt, elementIt);
    }
  };
};
}  // namespace bspline::support
#endif  // BSPLINE_SUPPORT_GRID_H
