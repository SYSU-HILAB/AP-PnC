#pragma once

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace aerodynamics
{
  inline constexpr char kFrameConvention[] = "flu-alpha-xz-beta-left-v1";

  // Aircraft velocity relative to air, expressed in body FLU. Collective
  // thrust is +Z. At rest both angles are zero; pure lateral flow has alpha=0.
  // Angles describe the input, independently of a model's low-speed force law.
  inline std::pair<double, double> air_angles(const Eigen::Vector3d &velocity)
  {
    if (!velocity.allFinite())
      throw std::invalid_argument("nonfinite body-FLU air velocity");
    const double longitudinal = std::hypot(velocity.x(), velocity.z());
    const double alpha =
        longitudinal == 0.0 ? 0.0 : std::atan2(velocity.x(), velocity.z());
    const double beta =
        velocity.isZero(0.0) ? 0.0 : std::atan2(velocity.y(), longitudinal);
    return {alpha, beta};
  }
}  // namespace aerodynamics
