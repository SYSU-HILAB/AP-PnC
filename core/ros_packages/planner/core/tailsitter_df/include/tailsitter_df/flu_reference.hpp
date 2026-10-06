#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace tailsitter_df
{
  // Main-frame reference map, derived directly for +Z collective thrust.
  // Simplified aero specific force = k*|v|*v_body_x along +X body.
  // No simulation legacy-wing transform is used here. Zero wind is assumed.
  struct FluReference
  {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d omega;
    double          specific_thrust;
    bool            fallback;
  };

  inline FluReference flu_reference(const Eigen::Vector3d &v,
                                    const Eigen::Vector3d &a,
                                    const Eigen::Vector3d &jerk,
                                    double gravity = 9.81, double k = -0.09408)
  {
    if (!v.allFinite() || !a.allFinite() || !jerk.allFinite() ||
        !std::isfinite(gravity) || gravity <= 0 || !std::isfinite(k))
      throw std::invalid_argument("invalid FLU flatness input");
    const Eigen::Vector3d eta       = a + gravity * Eigen::Vector3d::UnitZ();
    const double          speed     = v.norm();
    const bool            low_speed = speed < 0.5;
    const bool            fallback =
        low_speed || (eta - v * (v.dot(eta) / (speed * speed))).norm() < 0.05;
    Eigen::Vector3d zb, yb, xb, zdot, ydot, xdot;
    if (!fallback)
    {
      const Eigen::Vector3d h = eta - k * speed * v;
      zb                      = h.normalized();
      const auto hdot =
          (jerk - k * ((v.dot(a) / speed) * v + speed * a)).eval();
      zdot                     = (hdot - zb * zb.dot(hdot)) / h.norm();
      const Eigen::Vector3d y  = eta.cross(v);
      const Eigen::Vector3d dy = jerk.cross(v) + eta.cross(a);
      yb                       = y.normalized();
      ydot                     = (dy - yb * yb.dot(dy)) / y.norm();
      xb                       = yb.cross(zb);
      xdot                     = ydot.cross(zb) + yb.cross(zdot);
    }
    else
    {
      // Deterministic hover/freefall/projected-acceleration policy. At guard
      // surfaces no smoothness is claimed. Different from legacy blending.
      const double n = eta.norm();
      zb   = n > 0.05 ? (eta / n).eval() : Eigen::Vector3d::UnitZ().eval();
      zdot = n > 0.05 ? ((jerk - zb * zb.dot(jerk)) / n).eval()
                      : Eigen::Vector3d::Zero().eval();
      Eigen::Vector3d heading = Eigen::Vector3d::UnitX();
      if ((heading - zb * zb.dot(heading)).norm() < 0.05)
        heading = Eigen::Vector3d::UnitY();
      const Eigen::Vector3d y = zb.cross(heading), dy = zdot.cross(heading);
      yb   = y.normalized();
      ydot = (dy - yb * yb.dot(dy)) / y.norm();
      xb   = yb.cross(zb);
      xdot = ydot.cross(zb) + yb.cross(zdot);
    }
    Eigen::Matrix3d r, dr;
    r << xb, yb, zb;
    dr << xdot, ydot, zdot;
    const Eigen::Matrix3d s = r.transpose() * dr;
    const Eigen::Vector3d omega(0.5 * (s(2, 1) - s(1, 2)),
                                0.5 * (s(0, 2) - s(2, 0)),
                                0.5 * (s(1, 0) - s(0, 1)));
    return {r, omega, eta.dot(zb), fallback};
  }
}  // namespace tailsitter_df
