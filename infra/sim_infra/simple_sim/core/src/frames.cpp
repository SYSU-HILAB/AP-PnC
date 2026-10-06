#include "simple_sim/types.hpp"

namespace simple_sim
{
  // Single source of truth for the legacy wing/component frame. Every aero and
  // motor consumer calls these helpers; do not re-derive (z,-y,x) anywhere
  // else.
  Eigen::Vector3d wing_from_flu(const Eigen::Vector3d &body_vector)
  {
    return {body_vector.z(), -body_vector.y(), body_vector.x()};
  }

  Eigen::Vector3d flu_from_wing(const Eigen::Vector3d &wing_vector)
  {
    // C_LB is symmetric, so the inverse mapping has the identical form.
    return wing_from_flu(wing_vector);
  }
}  // namespace simple_sim
