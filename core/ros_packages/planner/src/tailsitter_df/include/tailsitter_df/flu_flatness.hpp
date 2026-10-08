#pragma once

// ENU/FLU differential flatness for the MINCO optimizer cost.
//
// This replaces the legacy FRD/NED autodiff kernel. No autodiff and no code
// generation: values come from the already-validated `flu_reference` map and
// the Jacobian is hand-derived here.
//
// Conventions: world ENU, body FLU, collective thrust along +Z_B, zero wind.
//   eta     = a + g z                      (specific force, ENU)
//   h       = eta - k |v| v                (k = aero slope, ~ -0.09408)
//   z_B     = h/|h|,  y_B = (eta x v)^,  x_B = y_B x z_B,  R_WB = [x_B y_B z_B]
//   a_T     = eta . z_B                    (collective specific thrust, N/kg)
//   f_aero  = eta - a_T z_B = k |v| v_Bx x_B    (body-X only, by construction)
//   omega_B = vee(R_WB^T Rdot_WB)          (pure FLU body rates)
//
// Why the residual is body-X only: y_B is perpendicular to eta by construction,
// so eta . y_B = 0. And z_B || (eta - k|v|v) gives eta . x_B = k|v| v_Bx.
// Therefore f_aero = (eta . x_B) x_B = k |v| v_Bx x_B, with no body-Y term.
//
// The guard branches (near hover, or degenerate projected acceleration) keep
// the deterministic policy of `flu_reference`; no smoothness is claimed across
// that boundary and the reported sub-gradient is documented as approximate
// there.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tailsitter_df/flu_reference.hpp>

namespace tailsitter_df
{
  struct FluFlatnessParams
  {
    double gravity     = 9.81;
    double k           = -0.09408;
    double speed_guard = 0.5;
    double proj_guard  = 0.05;
  };

  struct FluFlatnessResult
  {
    Eigen::Matrix3d rotation        = Eigen::Matrix3d::Identity();  // R_WB
    Eigen::Vector3d omega           = Eigen::Vector3d::Zero();      // FLU rad/s
    double          specific_thrust = 0.0;                          // N/kg
    bool            nominal         = false;
    // Rows [a_T, omega_x, omega_y, omega_z]; columns [v(3), a(3), j(3)].
    Eigen::Matrix<double, 4, 9> jacobian = Eigen::Matrix<double, 4, 9>::Zero();
  };

  namespace detail
  {
    inline Eigen::Matrix3d skew(const Eigen::Vector3d &x)
    {
      Eigen::Matrix3d s;
      s << 0.0, -x.z(), x.y(), x.z(), 0.0, -x.x(), -x.y(), x.x(), 0.0;
      return s;
    }

    // Column k is d(w)/d(q_k) for w = (I - z z^T) u.
    inline Eigen::Matrix3d projected_derivative(const Eigen::Vector3d &z,
                                                const Eigen::Vector3d &u,
                                                const Eigen::Matrix3d &dz,
                                                const Eigen::Matrix3d &du)
    {
      const Eigen::Matrix3d projection =
          Eigen::Matrix3d::Identity() - z * z.transpose();
      const double    z_dot_u = z.dot(u);
      Eigen::Matrix3d dw;
      for (int axis = 0; axis < 3; ++axis)
      {
        const Eigen::Vector3d dz_axis = dz.col(axis);
        dw.col(axis) =
            projection * du.col(axis) - dz_axis * z_dot_u - z * dz_axis.dot(u);
      }
      return dw;
    }

    // Column k is d(u/|u|)/d(q_k) with norm = |u|.
    inline Eigen::Matrix3d normalized_derivative(const Eigen::Vector3d &u,
                                                 double                 norm,
                                                 const Eigen::Matrix3d &du)
    {
      const Eigen::Vector3d unit = u / norm;
      const Eigen::Matrix3d projection =
          Eigen::Matrix3d::Identity() - unit * unit.transpose();
      Eigen::Matrix3d dunit;
      for (int axis = 0; axis < 3; ++axis)
        dunit.col(axis) = projection * du.col(axis) / norm;
      return dunit;
    }
  }  // namespace detail

  // FLU flatness value plus the analytic Jacobian d[a_T, omega_B]/d[v, a, j].
  inline FluFlatnessResult flu_flatness(const Eigen::Vector3d   &v,
                                        const Eigen::Vector3d   &a,
                                        const Eigen::Vector3d   &j,
                                        const FluFlatnessParams &params = {})
  {
    if (!v.allFinite() || !a.allFinite() || !j.allFinite() ||
        !std::isfinite(params.gravity) || params.gravity <= 0.0 ||
        !std::isfinite(params.k))
      throw std::invalid_argument("invalid FLU flatness input");

    const FluReference base = flu_reference(v, a, j, params.gravity, params.k);
    FluFlatnessResult  result;
    result.rotation        = base.rotation;
    result.omega           = base.omega;
    result.specific_thrust = base.specific_thrust;
    result.nominal         = !base.fallback;
    if (!result.nominal)
    {
      // Guard branch: keep the deterministic attitude and thrust of
      // flu_reference, report zero rates, and expose only the well-defined
      // thrust gradient. No smoothness is claimed across this boundary.
      const Eigen::Vector3d eta = a + params.gravity * Eigen::Vector3d::UnitZ();
      const double          norm = eta.norm();
      if (norm > params.proj_guard)
      {
        const Eigen::Vector3d zb = result.rotation.col(2);
        const Eigen::Matrix3d dzda =
            (Eigen::Matrix3d::Identity() - zb * zb.transpose()) / norm;
        result.jacobian.row(0).segment<3>(3) =
            eta.transpose() * dzda + zb.transpose();
      }
      return result;
    }

    using Eigen::Matrix3d;
    using Eigen::Vector3d;
    const Matrix3d identity = Matrix3d::Identity();
    const double   gravity  = params.gravity;
    const double   k        = params.k;

    const Vector3d eta     = a + gravity * Vector3d::UnitZ();
    const double   speed   = v.norm();
    const Vector3d h       = eta - k * speed * v;
    const double   h_norm  = h.norm();
    const Vector3d zb      = h / h_norm;
    const Vector3d y       = eta.cross(v);
    const double   y_norm  = y.norm();
    const Vector3d yb      = y / y_norm;
    const Vector3d xb      = yb.cross(zb);
    const double   v_dot_a = v.dot(a);

    // hdot = j - k[ (v.a/|v|) v + |v| a ] with vdot = a and adot = j.
    const Vector3d h_dot  = j - k * ((v_dot_a / speed) * v + speed * a);
    const Vector3d y_dot  = j.cross(v) + eta.cross(a);
    const Vector3d zb_dot = (identity - zb * zb.transpose()) * h_dot / h_norm;
    const Vector3d yb_dot = (identity - yb * yb.transpose()) * y_dot / y_norm;
    const Vector3d xb_dot = yb_dot.cross(zb) + yb.cross(zb_dot);

    Matrix3d frame, frame_rate;
    frame.col(0)      = xb;
    frame.col(1)      = yb;
    frame.col(2)      = zb;
    frame_rate.col(0) = xb_dot;
    frame_rate.col(1) = yb_dot;
    frame_rate.col(2) = zb_dot;

    // First-order input blocks.
    const Matrix3d zero  = Matrix3d::Zero();
    const Matrix3d dh_dv = -k * (v * v.transpose() / speed + speed * identity);
    // dhdot/dv;  dhdot/da = -k (v v^T/|v| + |v| I);  dhdot/dj = I
    const Matrix3d dhdot_dv =
        -k * ((a * v.transpose() + v * a.transpose()) / speed -
              (v_dot_a / (speed * speed * speed)) * v * v.transpose() +
              (v_dot_a / speed) * identity);
    const Matrix3d dhdot_da =
        -k * (v * v.transpose() / speed + speed * identity);
    const Matrix3d dy_dv    = detail::skew(eta);
    const Matrix3d dy_da    = -detail::skew(v);
    const Matrix3d dydot_dv = detail::skew(j);
    // ydot = j x v + eta x a with eta = a + g z, so eta depends on a too:
    // d(eta x a)/da = skew(eta) - skew(a) = skew(eta - a) = g skew(z).
    const Matrix3d dydot_da = gravity * detail::skew(Vector3d::UnitZ());
    const Matrix3d dydot_dj = -detail::skew(v);

    const std::array<Matrix3d, 3> dh    = {dh_dv, identity, zero};
    const std::array<Matrix3d, 3> dhdot = {dhdot_dv, dhdot_da, identity};
    const std::array<Matrix3d, 3> dy    = {dy_dv, dy_da, zero};
    const std::array<Matrix3d, 3> dydot = {dydot_dv, dydot_da, dydot_dj};

    const Vector3d unit_h = h_dot / h_norm;
    const Vector3d unit_y = y_dot / y_norm;

    for (int block = 0; block < 3; ++block)
    {
      const Matrix3d dzb = detail::normalized_derivative(h, h_norm, dh[block]);
      const Matrix3d dyb = detail::normalized_derivative(y, y_norm, dy[block]);

      Matrix3d dunit_h, dunit_y;
      for (int axis = 0; axis < 3; ++axis)
      {
        dunit_h.col(axis) =
            dhdot[block].col(axis) / h_norm -
            unit_h * (h.dot(dh[block].col(axis)) / (h_norm * h_norm));
        dunit_y.col(axis) =
            dydot[block].col(axis) / y_norm -
            unit_y * (y.dot(dy[block].col(axis)) / (y_norm * y_norm));
      }
      const Matrix3d dzb_dot =
          detail::projected_derivative(zb, unit_h, dzb, dunit_h);
      const Matrix3d dyb_dot =
          detail::projected_derivative(yb, unit_y, dyb, dunit_y);

      const Matrix3d skew_zb     = detail::skew(zb);
      const Matrix3d skew_yb     = detail::skew(yb);
      const Matrix3d skew_zb_dot = detail::skew(zb_dot);
      const Matrix3d skew_yb_dot = detail::skew(yb_dot);

      for (int axis = 0; axis < 3; ++axis)
      {
        const int      column = block * 3 + axis;
        const Vector3d dxb = -skew_zb * dyb.col(axis) + skew_yb * dzb.col(axis);
        const Vector3d dxb_dot =
            -skew_zb * dyb_dot.col(axis) + skew_yb_dot * dzb.col(axis) -
            skew_zb_dot * dyb.col(axis) + skew_yb * dzb_dot.col(axis);

        Matrix3d d_frame, d_frame_rate;
        d_frame.col(0)      = dxb;
        d_frame.col(1)      = dyb.col(axis);
        d_frame.col(2)      = dzb.col(axis);
        d_frame_rate.col(0) = dxb_dot;
        d_frame_rate.col(1) = dyb_dot.col(axis);
        d_frame_rate.col(2) = dzb_dot.col(axis);

        // omega = vee((S - S^T)/2) with S = R^T Rdot. The half factor is
        // required: flu_reference reports the antisymmetric part, so the
        // Jacobian must use the same combination.
        const Matrix3d d_omega =
            d_frame.transpose() * frame_rate + frame.transpose() * d_frame_rate;
        result.jacobian(1, column) = 0.5 * (d_omega(2, 1) - d_omega(1, 2));
        result.jacobian(2, column) = 0.5 * (d_omega(0, 2) - d_omega(2, 0));
        result.jacobian(3, column) = 0.5 * (d_omega(1, 0) - d_omega(0, 1));

        Vector3d d_eta = Vector3d::Zero();
        if (block == 1)
          d_eta = Vector3d::Unit(axis);
        result.jacobian(0, column) = eta.dot(dzb.col(axis)) + d_eta.dot(zb);
      }
    }
    return result;
  }
}  // namespace tailsitter_df
