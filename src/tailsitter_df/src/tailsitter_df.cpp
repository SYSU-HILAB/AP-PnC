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

// C++ includes
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <cassert>
#include <cstdlib>
#include <tailsitter_df/tailsitter_df.hpp>
#include <toml++/toml.hpp>

constexpr auto &out = std::cout;

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using namespace autodiff;
namespace tailsitter_df
{

  Eigen::Vector3d &getVelJacobThrAcc();
  Eigen::Vector3d &getAccJacobThrAcc();
  Eigen::Vector3d &getJerJacobThrAcc();

  Eigen::Matrix3d &getVelJacobOmega();
  Eigen::Matrix3d &getAccJacobOmega();
  Eigen::Matrix3d &getJerJacobOmega();
  void             toEigen(const VectorXreal &F, double &acc_thrust,
                           Eigen::Vector3d &omega);
  void             init(VectorXreal &z);
  VectorXreal      init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &jerk);
  // differential flatness forward, returns at_norm and angular rate
  VectorXreal     df_forward_nominal(const Vector3real &v, const Vector3real &a,
                                     const Vector3real &j);
  VectorXreal     df_forward_singularity_vel(const Vector3real &v,
                                             const Vector3real &a,
                                             const Vector3real &j);
  VectorXreal     df_forward(const VectorXreal &z);
  Eigen::MatrixXd df_backward(autodiff::VectorXreal &P);

  void df_test();

  class Model
  {
   private:
    realM3 phi_;
    double mass_ = 2;

    Model();
    Model(const Model &)            = delete;
    Model &operator=(const Model &) = delete;

   public:
    static Model    &getInstance();
    realM3          &getPhi();
    Eigen::Vector3d &Phi_z();

    double aeroForceZ(const Eigen::Vector3d &vb);
    realV3 getAeroForceB(const realV3 &vb);
    realM3 getPartialDerivativeOfFbOverVb(const realV3 &vb);
    realV3 getAeroForceAccB(const realV3 &vb);
  };

  tailsitter_df::Model &tailsitter_model = Model::getInstance();
  Data                 &data             = Data::getInstance();

  using vec = autodiff::VectorXreal;

  Model &Model::getInstance()
  {
    static Model instance;  // 静态局部变量
    return instance;
  }

  Model::Model()
  {
    // Update staitc parameter;
    const char *env_p = std::getenv("AP_PNC_DIR");
    std::string sv(env_p);
    sv += "/global.toml";
    toml::table tbl    = toml::parse_file(sv);
    auto        config = tbl["tailsitter_df"];

    auto phi = config["phi"].as_array();
    for (int i = 0; i < phi->size(); i++)
    {
      double val  = phi->get(i)->as_floating_point()->get();
      getPhi()(i) = val;
    }

    mass_ = config["mass"].as_floating_point()->get();
  }

  realM3 &Model::getPhi()
  {
    return phi_;
  };

  Eigen::Vector3d &Model::Phi_z()
  {
    static Eigen::Vector3d temp;
    temp << getPhi().row(2)(0).val(), getPhi().row(2)(1).val(),
        getPhi().row(2)(2).val();
    return temp;
  };

  realV3 Model::getAeroForceB(const realV3 &vb)
  {
    real   V    = vb.norm();
    realV3 temp = V * getPhi() * vb;
    temp        = temp * mass_;
    return temp;
  };

  realV3 Model::getAeroForceAccB(const realV3 &vb)
  {
    real V = vb.norm();
    return V * getPhi() * vb;
  };

  void toEigen(const VectorXreal &F, double &acc_thrust, Eigen::Vector3d &omega)
  {
    acc_thrust = F(0).val();
    omega << F(1).val(), F(2).val(), F(3).val();
  }

  using namespace std;

  VectorXreal df_forward_nominal(const realV3 &v, const realV3 &a,
                                 const realV3 &j)
  {
    realV3 &xb = data.xb;
    realV3 &yb = data.yb;
    realV3 &zb = data.zb;

    auto   &attitude      = data.attitude;
    auto   &q             = data.q;
    auto   &alpha         = data.alpha;
    auto   &at            = data.at;
    realV3 &aeroForceB    = data.aeroForceB;
    real   &B_aeroForce_z = data.B_aeroForce_z;
    real   &S_av_x        = data.S_av_x;
    real   &S_av_z        = data.S_av_z;
    real   &B_Omega_b_y   = data.B_Omega_b_y;
    real   &alpha_dot     = data.alpha_dot;

    realV3 x_v    = v / v.norm();
    realV3 temp   = -(realM3::Identity() - x_v * x_v.transpose()) * (a - g);
    realV3 z_v    = temp / temp.norm();
    realV3 y_v    = z_v.cross(x_v);
    real   temp_x = x_v.transpose() * (a - g);
    real   temp_z = z_v.transpose() * (a - g);
    S_av_x        = temp_x;
    S_av_z        = temp_z;

    real tan_acc = x_v.transpose() * a;

    real V        = v.norm();
    real omega_vz = (y_v.transpose() * g);
    omega_vz      = omega_vz / V;
    real omega_vx = (y_v.transpose() * j);
    omega_vx      = ((omega_vz * S_av_x) - omega_vx) / S_av_z;
    // real omega_vy = (z_v.transpose() * g);
    // omega_vy      = (-S_av_z - omega_vy) / V;
    real omega_vy = -z_v.transpose() * a;
    omega_vy      = omega_vy / V;
    // omega_vy = () / V;

    double phi_zz              = tailsitter_model.getPhi().row(2)(2).val();
    aeroForceB                 = tailsitter_model.getAeroForceB(v);
    B_aeroForce_z              = aeroForceB(2);
    real             V_squared = V * V;
    constexpr double phi_33    = -0.09408;
    alpha                      = atan2(-S_av_z, S_av_x - phi_33 * V_squared);

    yb    = y_v;
    xb    = cos(alpha) * x_v - sin(alpha) * z_v;
    zb    = cos(alpha) * z_v + sin(alpha) * x_v;
    at(0) = S_av_x * cos(alpha) - S_av_z * sin(alpha);
    attitude << xb, yb, zb;

    q               = Eigen::Quaternion<real>(attitude);
    
    real S_av_z_dot = z_v.transpose() * j;
    S_av_z_dot      = S_av_z_dot + omega_vy * S_av_x;

    real S_av_x_dot = x_v.transpose() * j;
    S_av_x_dot      = S_av_x_dot - omega_vy * S_av_z;

    realV3 vb = attitude.transpose() * v;

    real S_a_x = x_v.transpose() * a;
    real S_a_z = z_v.transpose() * a;

    real phi_dot_xx = phi_33 * 2 * v.transpose() * a;

    real increment = S_av_x - phi_33 * V_squared;
    //
    real denominator = (S_av_z * S_av_z + increment * increment);

    // real numerator = (S_av_x_dot - phi_dot_xx);

    real acc_on_vel = v.transpose() * a;

    real numerator1 =
        phi_33 * (V_squared * S_av_z_dot - 2 * S_av_z * acc_on_vel);

    real numerator2 = S_av_z * S_av_x_dot - S_av_x * S_av_z_dot;

    real numerator = numerator1 + numerator2;

    alpha_dot = numerator / denominator;

    // A = Sapz ; B = Sapx;

    // procedure1
    // realV3 ab   = attitude.transpose() * a;
    // real   ab_x = x_v.transpose() * a;
    // real   ab_z = z_v.transpose() * a;

    // real infs = ab_z * cos(alpha) - ab_x * sin(alpha);
    // infs      = infs / V;
    // alpha_dot = (infs - omega_vy) / 2.0;

    // real term1 = zb.transpose() * a;
    // term1      = term1 * vb(0);
    // real term2 = xb.transpose() * a;
    // term2      = term2 * vb(2);
    // real V2    = v.transpose() * v;
    // alpha_dot  = (term1 - term2);
    // alpha_dot  = alpha_dot / V2;

    // real term1 = S_a_x * cos(alpha);
    // real term2 = S_a_z * sin(alpha);
    // alpha_dot  = term1 - term2;
    // alpha_dot  = alpha_dot / V;
    // alpha_dot  = alpha_dot - omega_vy;
    // alpha_dot = S_a_z / V;
    // alpha_dot = alpha_dot - omega_vy;
    // B_omega_s_y = S_omega_s_y;
    // alpha_dot = alpha_dot - omega_vy;
    // alpha_dot = alpha_dot / 2.0;

    // real at_dot = (S_av_x_dot - alpha_dot * S_av_z) * cos(alpha) -
    //               (S_av_z_dot + alpha_dot * S_av_x) * sin(alpha);

    B_Omega_b_y = omega_vy + alpha_dot;

    return (Eigen::Vector4<real>() << tan_acc, omega_vx, B_Omega_b_y, omega_vz)
        .finished();
  }

  VectorXreal df_forward_singularity_vel(const Vector3real &v,
                                         const Vector3real &a,
                                         const Vector3real &)
  {
    realV3 &eta      = data.eta;
    realV3 &xb       = data.xb;
    realV3 &yb       = data.yb;
    realV3 &zb       = data.zb;
    auto   &q        = data.q;
    auto   &at       = data.at;
    realV3 &zb_fixed = data.zb_fixed;

    eta = a - g;
    at  = eta;
    xb  = at;
    xb.normalize();

    real   scaling   = v.norm() / vThreshold;
    realV3 yb_from_v = v.cross(eta);
    yb_from_v.normalize();
    realV3 yb_from_fixed_frame = zb_fixed.cross(xb);
    yb_from_fixed_frame.normalize();
    yb = (1 - scaling) * yb_from_fixed_frame +  scaling * yb_from_v;
    yb.normalize();
    zb = xb.cross(yb);
    zb.normalize();

    realM3 rot = (realM3() << xb, yb, zb).finished();
    q          = Eigen::Quaternion<real>(rot);

    return Eigen::Vector4<real>::Zero();
  }

  VectorXreal df_forward(const VectorXreal &z)
  {
    Vector3real v = z.segment(0, 3);  // the input vector x with 5 variables
    Vector3real a = z.segment(3, 3);  // the input vector x with 5 variables
    Vector3real j = z.segment(6, 3);  // the input vector x with 5 variables

    if (v.norm() < vThreshold)
    {
      return df_forward_singularity_vel(v, a, j);
    }
    else
    {
      return df_forward_nominal(v, a, j);
    }
  }

  Eigen::MatrixXd df_backward(autodiff::VectorXreal &P)
  {
    static autodiff::VectorXreal F;
    data.J =
        autodiff::jacobian(tailsitter_df::df_forward, autodiff::detail::wrt(P),
                           autodiff::detail::at(P), F);
    return data.J;
  }

  Eigen::MatrixXd Data::df_backward_auto(const Eigen::Vector3d &vel,
                                         const Eigen::Vector3d &acc,
                                         const Eigen::Vector3d &jerk,
                                         double &thrust, Eigen::Vector3d &omega)
  {
    static autodiff::VectorXreal F_auto;
    VectorXreal                  P = init(vel, acc, jerk);
    J = autodiff::jacobian(tailsitter_df::df_forward, autodiff::detail::wrt(P),
                           autodiff::detail::at(P), F_auto);

    tailsitter_df::toEigen(F_auto, thrust, omega);
    return J;
  }

  void jacobian_at(Eigen::Vector3d &gradv, Eigen::Vector3d &grada,
                   Eigen::Vector3d &gradj)
  {
    // Assuming Vector3d;
    static Eigen::RowVector4d acc_thrust_index =
        (Eigen::RowVector4d() << 1, 0, 0, 0).finished();
    gradv = (acc_thrust_index * data.J).head(3).transpose();
    grada = (acc_thrust_index * data.J).segment(3, 3).transpose();
    gradj = (acc_thrust_index * data.J).tail(3).transpose();
  }

  void jacobian_omega(Eigen::Matrix3d &gradv, Eigen::Matrix3d &grada,
                      Eigen::Matrix3d &gradj)
  {
    // for example gradv =
    // [
    //   o_1_v1, o_2_v1, o_3_v1,
    //   o_1_v2, o_2_v2, o_3_v2,
    //   o_1_v3, o_2_v3, o_3_v3,
    // ]
    gradv = data.J.block<3, 3>(1, 0).transpose();
    grada = data.J.block<3, 3>(1, 3).transpose();
    gradj = data.J.block<3, 3>(1, 6).transpose();
  }

  Eigen::Matrix3d &getVelJacobOmega()
  {
    // for example gradv =
    // [
    //   o_1_v1, o_2_v1, o_3_v1,
    //   o_1_v2, o_2_v2, o_3_v2,
    //   o_1_v3, o_2_v3, o_3_v3,
    // ]
    static Eigen::Matrix3d gradv = data.J.block<3, 3>(1, 0).transpose();
    gradv                        = data.J.block<3, 3>(1, 0).transpose();
    return gradv;
  }

  Eigen::Matrix3d &getAccJacobOmega()
  {
    // for example gradv =
    // [
    //   o_1_v1, o_2_v1, o_3_v1,
    //   o_1_v2, o_2_v2, o_3_v2,
    //   o_1_v3, o_2_v3, o_3_v3,
    // ]
    static Eigen::Matrix3d grada = data.J.block<3, 3>(1, 3).transpose();
    grada                        = data.J.block<3, 3>(1, 3).transpose();
    return grada;
  }

  Eigen::Matrix3d &getJerJacobOmega()
  {
    // for example gradv =
    // [
    //   o_1_v1, o_2_v1, o_3_v1,
    //   o_1_v2, o_2_v2, o_3_v2,
    //   o_1_v3, o_2_v3, o_3_v3,
    // ]
    static Eigen::Matrix3d gradj = data.J.block<3, 3>(1, 6).transpose();
    gradj                        = data.J.block<3, 3>(1, 6).transpose();
    return gradj;
  }

  VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                   const Eigen::Vector3d &jerk)
  {
    VectorXreal z;
    z.resize(9);
    z << vel(0), vel(1), vel(2), acc(0), acc(1), acc(2), jerk(0), jerk(1),
        jerk(2);
    return z;
  }

}  // namespace tailsitter_df
