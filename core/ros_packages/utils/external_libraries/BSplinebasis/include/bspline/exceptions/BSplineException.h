/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

#ifndef BSPLINE_BSPLINEEXCEPTION_H
#define BSPLINE_BSPLINEEXCEPTION_H

#include <sstream>
#include <string>

/*!
 * The namespace containing the exceptions and errorCodes.
 */
namespace bspline::exceptions {

/*!
 * The error codes, which may be expected.
 */
enum class ErrorCode {
  DIFFERING_GRIDS,
  INCONSISTENT_DATA,
  MISSING_DATA,
  INVALID_ACCESS,
  UNDETERMINED
};

/*!
 * Returns the default error message for an error code.
 *
 * @param errorCode The errorCode for which the default message is requested.
 * @returns The default error message corresponding to the errorCode.
 */
inline std::string getErrorMessage(ErrorCode errorCode) {
  switch (errorCode) {
    case ErrorCode::DIFFERING_GRIDS:
      return "The requested operation is not implemented for objects defined "
             "on non-equivalent grids.";
    case ErrorCode::INCONSISTENT_DATA:
      return "The data provided is inconsistent.";
    case ErrorCode::MISSING_DATA:
      return "Mandatory data was not provided.";
    case ErrorCode::INVALID_ACCESS:
      return "Attempted access of nonexistent data.";
    case ErrorCode::UNDETERMINED:
      return "The cause of the error is undetermined.";
    default:
      return "Errorcode unknown.";
  }
}

/*!
 * Returns the errorCode name.
 *
 * @param errorCode The errorCode for which the name is requested.
 * @returns The name of the errorCode as a string.
 */
inline std::string getErrorCodeName(ErrorCode errorCode) {
  switch (errorCode) {
    case ErrorCode::DIFFERING_GRIDS:
      return "DIFFERING_GRIDS";
    case ErrorCode::INCONSISTENT_DATA:
      return "INCONSISTENT_DATA";
    case ErrorCode::MISSING_DATA:
      return "MISSING_DATA";
    case ErrorCode::INVALID_ACCESS:
      return "INVALID_ACCESS";
    case ErrorCode::UNDETERMINED:
      return "UNDETERMINED";
    default:
      return "UNKNOWN_ERRORCODE";
  }
}

/*!
 * The main exception class.
 */
class BSplineException final : public std::exception {
 private:
  /*! The error code. */
  ErrorCode _errorCode;
  /*! The string returned by the what() method. */
  std::string _whatString;

  /*!
   * Generates the std::string returned in the what() method.
   *
   * @param errorCode The errorCode to generate the what string for.
   * @param message A message string that can be custom.
   * @returns The generated what string.
   */
  std::string generateWhatString(ErrorCode errorCode,
                                 const std::string &message) const {
    std::stringstream ret;
    ret << "BSplineException (code: " << getErrorCodeName(errorCode)
        << "): " << message;
    return ret.str();
  };

 public:
  /*!
   * Uses the default error message corresponding to the error code.
   *
   * @param errorCode The errorCode.
   */
  explicit BSplineException(ErrorCode errorCode)
      : _errorCode(errorCode),
        _whatString(
            generateWhatString(errorCode, getErrorMessage(errorCode))){};

  /*!
   * Uses a custom error message.
   *
   * @param errorCode The errorCode.
   * @param message The custom error message.
   */

  BSplineException(ErrorCode errorCode, std::string message)
      : _errorCode(errorCode),
        _whatString(generateWhatString(errorCode, message)){};

  /*!
   * Returns a string representation of the exception.
   *
   * @returns The what string.
   */
  const char *what() const noexcept override { return _whatString.c_str(); };

  /*!
   * Returns the error code of this exception.
   *
   * @returns the errorCode.
   */
  ErrorCode getErrorCode() const { return _errorCode; };
};

}  // namespace bspline::exceptions
#endif  // BSPLINE_BSPLINEEXCEPTION_H
