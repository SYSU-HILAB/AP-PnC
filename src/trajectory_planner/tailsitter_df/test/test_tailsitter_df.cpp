/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <tailsitter_df/tailsitter_df.hpp>

using namespace tailsitter_df;

namespace
{

  const Eigen::Vector3d kGravity(0.0, 0.0, -9.8);

  // Recover the spatial frame from the body axes and alpha:
  // x_s = cos(a)xb + sin(a)zb, z_s = cos(a)zb - sin(a)xb, y_s = yb.
  void spatialFrame(const Data &d, Eigen::Vector3d &x_s, Eigen::Vector3d &y_s,
                    Eigen::Vector3d &z_s)
  {
    const double          ca = std::cos(d.getAlpha());
    const double          sa = std::sin(d.getAlpha());
    const Eigen::Vector3d xb = d.getXb();
    const Eigen::Vector3d yb = d.getYb();
    const Eigen::Vector3d zb = d.getZb();
    x_s                      = ca * xb + sa * zb;
    y_s                      = yb;
    z_s                      = ca * zb - sa * xb;
  }

}  // namespace

TEST(TailsitterDFTest, NominalTransportIdentities)
{
  Data &data = Data::getInstance();
  data.reset();

  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  const Eigen::Vector3d a(0.0, 4.0, -9.8);
  const Eigen::Vector3d j(0.5, 0.0, 0.0);

  Eigen::Vector3d       omega;
  double                thrust;
  const Eigen::MatrixXd J = data.df_backward_auto(v, a, j, thrust, omega);

  EXPECT_TRUE(J.allFinite());
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(omega.allFinite());

  Eigen::Vector3d x_s, y_s, z_s;
  spatialFrame(data, x_s, y_s, z_s);

  const Eigen::Vector3d a_p = a - kGravity;
  const double          V   = v.norm();

  // Spatial frame orthonormality and handedness.
  EXPECT_NEAR(x_s.norm(), 1.0, 1e-9);
  EXPECT_NEAR(y_s.norm(), 1.0, 1e-9);
  EXPECT_NEAR(z_s.norm(), 1.0, 1e-9);
  EXPECT_NEAR(x_s.dot(y_s), 0.0, 1e-9);
  EXPECT_NEAR(y_s.dot(z_s), 0.0, 1e-9);
  EXPECT_NEAR(z_s.dot(x_s), 0.0, 1e-9);
  EXPECT_TRUE(y_s.isApprox(z_s.cross(x_s), 1e-9));

  // Projected proper acceleration components.
  EXPECT_NEAR(data.getSApX(), x_s.dot(a_p), 1e-9);
  EXPECT_NEAR(data.getSApZ(), z_s.dot(a_p), 1e-9);

  // omega_sz = y_s^T g / V.
  EXPECT_NEAR(data.getSOmegaZ(), y_s.dot(kGravity) / V, 1e-9);

  // omega_sx = (S_ap_x * omega_sz - y_s^T j) / S_ap_z.
  EXPECT_NEAR(data.getSOmegaX() * data.getSApZ(),
              data.getSApX() * data.getSOmegaZ() - y_s.dot(j), 1e-9);

  // S_ap_x_dot = x_s^T j - omega_sy * S_ap_z.
  EXPECT_NEAR(data.getSApXDot(),
              x_s.dot(j) - data.getSOmegaY() * data.getSApZ(), 1e-9);

  // S_ap_z_dot = z_s^T j + omega_sy * S_ap_x.
  EXPECT_NEAR(data.getSApZDot(),
              z_s.dot(j) + data.getSOmegaY() * data.getSApX(), 1e-9);

  // Forward output matches the stored spatial states.
  EXPECT_NEAR(thrust, x_s.dot(a), 1e-9);
  EXPECT_NEAR(omega.x(), data.getSOmegaX(), 1e-9);
  EXPECT_NEAR(omega.y(), data.getBOmegaY(), 1e-9);
  EXPECT_NEAR(omega.z(), data.getSOmegaZ(), 1e-9);
}

TEST(TailsitterDFTest, NominalAboveThresholdFinite)
{
  Data &data = Data::getInstance();
  data.reset();

  // |proj| = 0.2 m/s^2 > apThreshold: stays on the nominal path.
  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  const Eigen::Vector3d a(2.0, 0.2, -9.8);
  const Eigen::Vector3d j(0.0, 0.0, 0.0);

  Eigen::Vector3d       omega;
  double                thrust;
  const Eigen::MatrixXd J = data.df_backward_auto(v, a, j, thrust, omega);

  EXPECT_TRUE(J.allFinite());
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(omega.allFinite());
  EXPECT_GT(std::abs(data.getSApZ()), apThreshold);
}

TEST(TailsitterDFTest, ProjectedAccelerationSingularityExact)
{
  Data &data = Data::getInstance();
  data.reset();

  // a_p = a - g is exactly parallel to v, so (I - vv^T/V^2) a_p = 0 and the
  // flatness map is out of domain (deterministic reproducer from the issue).
  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  const Eigen::Vector3d a(2.0, 0.0, -9.8);
  const Eigen::Vector3d j(0.0, 0.0, 0.0);

  Eigen::Vector3d       omega;
  double                thrust;
  const Eigen::MatrixXd J = data.df_backward_auto(v, a, j, thrust, omega);

  // No unguarded normalization/division: finite outputs and Jacobian.
  EXPECT_TRUE(J.allFinite());
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(omega.allFinite());

  // Policy: tangent acceleration is well defined, angular rates are zero.
  EXPECT_NEAR(thrust, v.dot(a) / v.norm(), 1e-9);
  EXPECT_NEAR(omega.norm(), 0.0, 1e-12);
  EXPECT_NEAR(data.getSApZ(), 0.0, 1e-12);
  EXPECT_NEAR(data.getSOmegaX(), 0.0, 1e-12);
}

TEST(TailsitterDFTest, ProjectedAccelerationSingularityNear)
{
  Data &data = Data::getInstance();
  data.reset();

  // a_p nearly parallel to v: |proj| = 0.01 m/s^2 < apThreshold triggers the
  // deterministic fallback instead of dividing by a near-zero S_ap_z.
  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  const Eigen::Vector3d a(2.0, 0.01, -9.8);
  const Eigen::Vector3d j(0.0, 0.0, 0.0);

  Eigen::Vector3d       omega;
  double                thrust;
  const Eigen::MatrixXd J = data.df_backward_auto(v, a, j, thrust, omega);

  EXPECT_TRUE(J.allFinite());
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(omega.allFinite());
  EXPECT_NEAR(omega.norm(), 0.0, 1e-12);
}

TEST(TailsitterDFTest, LowVelocitySingularityFinite)
{
  Data &data = Data::getInstance();
  data.reset();

  const Eigen::Vector3d v(0.1, 0.0, 0.0);
  const Eigen::Vector3d a(0.0, 0.0, -5.0);
  const Eigen::Vector3d j(0.0, 0.0, 0.0);

  Eigen::Vector3d       omega;
  double                thrust;
  const Eigen::MatrixXd J = data.df_backward_auto(v, a, j, thrust, omega);

  EXPECT_TRUE(J.allFinite());
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(omega.allFinite());
}

TEST(TailsitterDFTest, FallbackFrameIsDeterministic)
{
  Data &data = Data::getInstance();
  data.reset();

  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  const Eigen::Vector3d a(2.0, 0.0, -9.8);
  const Eigen::Vector3d j(0.0, 0.0, 0.0);

  Eigen::Vector3d omega1, omega2;
  double          thrust1, thrust2;
  data.df_backward_auto(v, a, j, thrust1, omega1);
  const Eigen::Quaterniond q1 = data.getQuaternion();
  data.df_backward_auto(v, a, j, thrust2, omega2);
  const Eigen::Quaterniond q2 = data.getQuaternion();

  EXPECT_NEAR(thrust1, thrust2, 0.0);
  EXPECT_TRUE(omega1.isApprox(omega2, 0.0));
  EXPECT_TRUE(q1.isApprox(q2, 1e-12));

  // Fallback frame must be a valid orthonormal body frame.
  const Eigen::Matrix3d att = data.getAttitude();
  EXPECT_TRUE(att.determinant() > 0.0);
  EXPECT_TRUE(att.isUnitary(1e-9));
}
