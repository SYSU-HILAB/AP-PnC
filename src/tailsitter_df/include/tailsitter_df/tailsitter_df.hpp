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
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#pragma once

#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

namespace tailsitter_df
{
  // this algorithm is implemented assuming the body frame is FRD convention.
  // the world frame is assumed to be NED.
  using namespace autodiff;
  using namespace autodiff::detail;
  using real                = autodiff::real1st;
  using realV3              = autodiff::Vector3real;
  using realM3              = autodiff::Matrix3real;
  const realV3   g          = (realV3() << 0, 0, 9.8).finished();
  constexpr real vThreshold = 0.5;

  class Data
  {
   private:
    Data() { zb_fixed.normalize(); }
    Data(const Data &)            = delete;
    Data &operator=(const Data &) = delete;

    // Private autodiff members
    realV3                  aeroForceB = realV3::Zero();
    realV3                  eta;
    realV3                  xb, yb, zb;
    realV3                  vel;
    realV3                  temp;
    realM3                  attitude;
    Eigen::Quaternion<real> q;
    real                    S_av_x;
    real                    S_av_z;
    real                    B_aeroForce_z;
    real                    alpha;
    real                    alpha_dot;
    real                    B_Omega_b_y;
    real                    alpha_prev;
    realV3                  at;
    realV3                  zb_fixed = (realV3() << 0, 1, 0).finished();

   public:
    Eigen::Matrix<double, 4, 9> J;

    static Data &getInstance()
    {
      static Data instance;
      return instance;
    }
    // Friend declarations for functions that need direct access
    friend VectorXreal df_forward_nominal(const Vector3real &v,
                                          const Vector3real &a,
                                          const Vector3real &j);
    friend VectorXreal df_forward_singularity_vel(const Vector3real &v,
                                                  const Vector3real &a,
                                                  const Vector3real &j);

    void reset(bool circled = true)
    {
      auto   xb_init_always = (realV3() << 0, 0, 1).finished();
      realV3 yb_init;
      if (circled)
      {
        yb_init = (realV3() << 1, 0, 0).finished();
      }
      else
      {
        yb_init = (realV3() << 0, -1, 0).finished();
      }
      zb_fixed = xb_init_always.cross(yb_init);
      zb_fixed.normalize();
      yb_init.normalize();
      zb_fixed.normalize();
    }

    // Add df_backward_auto as a member function
    Eigen::MatrixXd df_backward_auto(const Eigen::Vector3d &vel,
                                     const Eigen::Vector3d &acc,
                                     const Eigen::Vector3d &jerk,
                                     double &thrust, Eigen::Vector3d &omega);

    // Jacobian-related functions
    void jacobian_at(Eigen::Vector3d &gradv, Eigen::Vector3d &grada,
                     Eigen::Vector3d &gradj)
    {
      static Eigen::RowVector4d acc_thrust_index =
          (Eigen::RowVector4d() << 1, 0, 0, 0).finished();
      gradv = (acc_thrust_index * J).head(3).transpose();
      grada = (acc_thrust_index * J).segment(3, 3).transpose();
      gradj = (acc_thrust_index * J).tail(3).transpose();
    }

    void jacobian_omega(Eigen::Matrix3d &gradv, Eigen::Matrix3d &grada,
                        Eigen::Matrix3d &gradj)
    {
      gradv = J.block<3, 3>(1, 0).transpose();
      grada = J.block<3, 3>(1, 3).transpose();
      gradj = J.block<3, 3>(1, 6).transpose();
    }

    // Public getters returning Eigen types
    Eigen::Vector3d getAeroForceB() const
    {
      return Eigen::Vector3d(aeroForceB(0).val(), aeroForceB(1).val(),
                             aeroForceB(2).val());
    }

    Eigen::Vector3d getEta() const
    {
      return Eigen::Vector3d(eta(0).val(), eta(1).val(), eta(2).val());
    }

    Eigen::Vector3d getXb() const
    {
      return Eigen::Vector3d(xb(0).val(), xb(1).val(), xb(2).val());
    }

    Eigen::Vector3d getYb() const
    {
      return Eigen::Vector3d(yb(0).val(), yb(1).val(), yb(2).val());
    }

    Eigen::Vector3d getZb() const
    {
      return Eigen::Vector3d(zb(0).val(), zb(1).val(), zb(2).val());
    }

    Eigen::Matrix3d getAttitude() const
    {
      Eigen::Matrix3d result;
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          result(i, j) = attitude(i, j).val();
        }
      }
      return result;
    }

    Eigen::Quaterniond getQuaternion() const
    {
      return Eigen::Quaterniond(q.w().val(), q.x().val(), q.y().val(),
                                q.z().val());
    }

    double getAlpha() const { return alpha.val(); }
    double getAlphaDot() const { return alpha_dot.val(); }
    double getBOmegaY() const { return B_Omega_b_y.val(); }
    double getSAvX() const { return S_av_x.val(); }
    double getSAvZ() const { return S_av_z.val(); }
    double getBAeroForceZ() const { return B_aeroForce_z.val(); }
  };
}  // namespace tailsitter_df
