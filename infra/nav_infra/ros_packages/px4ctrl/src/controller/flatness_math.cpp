#include "flatness_math.h"

#include <cmath>

bool almostZero(double value)
{
  return fabs(value) < 1.0e-3;
}

void normalizeWithGrad(const Eigen::Vector3d &x, const Eigen::Vector3d &xd,
                       Eigen::Vector3d &xNor, Eigen::Vector3d &xNord)
{
  const double xSqrNorm = x.squaredNorm();
  const double xNorm    = sqrt(xSqrNorm);
  xNor                  = x / xNorm;
  xNord                 = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
  return;
}

Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d    &x_B_prototype,
                                       const Eigen::Vector3d    &x_C,
                                       const Eigen::Vector3d    &y_C,
                                       const Eigen::Quaterniond &est_q)
{
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm()))
  {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated = est_q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm()))
    {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    }
    else
    {
      x_B = x_B_projected.normalized();
    }
  }
  else
  {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  if (x_B.dot(x_C) < 0.0)
  {
    x_B = -x_B;
  }

  return x_B;
}
